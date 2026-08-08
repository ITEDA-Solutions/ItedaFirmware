#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>
#include <sys/time.h>

// -------------------- CONFIGURATION --------------------
const char* VERSION = "2.7";
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";
const char* GPRS_APN = "internet";  // Airtel Kenya
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// Network unlock key (NCK) for a carrier-locked module — the code the seller
// supplies for a "+CPIN: PH-NET PIN" module. Leave empty if not locked.
const char* GSM_NCK = "";

// -------------------- PIN DEFINITIONS --------------------
#define DHTPIN1 7    // Chamber Bottom
#define DHTPIN2 8    // Chamber Middle
#define DHTPIN3 9    // Chamber Top
#define DHTPIN4 10   // Ambient
#define DHTTYPE DHT11

#define MOISTURE1 1
#define MOISTURE2 2
#define MOISTURE3 4
#define MOISTURE4 5

#define HEATER_1   21
#define HEATER_2   10
#define FAN_RELAY  18

#define LED_RED    15
#define LED_YELLOW 16
#define LED_GREEN  13

#define CURRENT_PIN 6

// GSM 900A — IO11 -> 3VT (module TX), IO12 -> 3VR (module RX)
#define GSM_RX 11
#define GSM_TX 12
#define GSM_BAUD 9600

// -------------------- GLOBALS --------------------
HardwareSerial GSM(1);

// gsmReady    — module answers AT (wiring/power/baud are correct)
// gsmSimOk    — a SIM is physically present and unlocked
// gsmDataReady— SIM registered on the network, so a GPRS send can be attempted
bool gsmReady = false;
bool gsmSimOk = false;
bool gsmRegistered = false;
bool gsmDataReady = false;
bool gsmCarrierLocked = false;   // module rejects this operator (PH-NET PIN)

int gsmSignal = -1;
int gsmRegStatus = -1;

String gsmIMEI = "";
String gsmICCID = "";
String gsmNumber = "";
String gsmOperator = "";

const char* lastSendMethod = "none";
bool lastSendSuccess = false;
DHT dhts[] = {
  {DHTPIN1, DHTTYPE},
  {DHTPIN2, DHTTYPE},
  {DHTPIN3, DHTTYPE},
  {DHTPIN4, DHTTYPE}
};

double Setpoint = 47.5, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

bool heaterActive = false;

unsigned long lastBlinkGreen = 0;
unsigned long lastBlinkRed = 0;

bool greenState = false;
bool redState = false;

// -------------------- GSM --------------------
String getTimestamp();   // defined under UTILITIES, used by gsmSyncTime()

String gsmSendAT(const char* cmd, unsigned long timeoutMs = 2000) {
  while (GSM.available()) {
    GSM.read();
  }

  GSM.println(cmd);
  Serial.printf("[GSM] >> %s\n", cmd);

  String response;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (GSM.available()) {
      response += (char)GSM.read();
    }
    if (response.indexOf("OK") >= 0 || response.indexOf("ERROR") >= 0) {
      break;
    }
    delay(10);
  }

  response.trim();
  if (response.length() > 0) {
    Serial.printf("[GSM] << %s\n", response.c_str());
  }

  return response;
}

// Returns the text following `tag` up to the end of that line, e.g.
// gsmValueAfter("+CSQ: 18,0", "+CSQ:") -> "18,0"
String gsmValueAfter(const String& resp, const char* tag) {
  int i = resp.indexOf(tag);
  if (i < 0) {
    return "";
  }

  i += strlen(tag);

  int end = resp.indexOf('\n', i);
  if (end < 0) {
    end = resp.length();
  }

  String value = resp.substring(i, end);
  value.trim();

  return value;
}

// Pulls the n-th quoted field out of a response line (1-based).
String gsmQuotedField(const String& resp, int index) {
  int pos = 0;

  for (int found = 0; found < index; found++) {
    int open = resp.indexOf('"', pos);
    if (open < 0) {
      return "";
    }

    int close = resp.indexOf('"', open + 1);
    if (close < 0) {
      return "";
    }

    if (found == index - 1) {
      return resp.substring(open + 1, close);
    }

    pos = close + 1;
  }

  return "";
}

// AT+GSN and AT+CCID answer with a bare digit string, so keep only the digits.
String gsmDigitsOnly(const String& resp, unsigned int minLength) {
  String out;

  for (unsigned int i = 0; i < resp.length(); i++) {
    if (isdigit(resp.charAt(i))) {
      out += resp.charAt(i);
    }
  }

  if (out.length() < minLength) {
    return "";
  }

  return out;
}

int gsmParseCsq(const String& resp) {
  String value = gsmValueAfter(resp, "+CSQ:");
  if (value.length() == 0) {
    return -1;
  }

  int comma = value.indexOf(',');
  if (comma > 0) {
    value = value.substring(0, comma);
  }

  value.trim();
  return value.toInt();
}

// +CREG: <n>,<stat> — 1 = registered home, 5 = registered roaming.
int gsmParseCreg(const String& resp) {
  String value = gsmValueAfter(resp, "+CREG:");
  if (value.length() == 0) {
    return -1;
  }

  int comma = value.indexOf(',');
  if (comma < 0) {
    return -1;
  }

  return value.substring(comma + 1).toInt();
}

// Reads network time from the module (AT+CCLK?) and sets the ESP32 clock, so
// payload timestamps are correct on a GSM-only boot with no NTP.
bool gsmSyncTime() {
  String resp = gsmSendAT("AT+CCLK?");

  String clk = gsmQuotedField(resp, 1);   // "yy/MM/dd,hh:mm:ss+zz"
  if (clk.length() < 17) {
    return false;
  }

  struct tm t = {0};

  int year, month, day, hour, minute, second;
  int zone = 0;              // stays 0 if the module omits the offset field

  if (sscanf(clk.c_str(), "%d/%d/%d,%d:%d:%d%d",
             &year, &month, &day, &hour, &minute, &second, &zone) < 6) {
    return false;
  }

  if (year < 24) {           // module clock never got a network update
    return false;
  }

  t.tm_year = year + 100;    // struct tm counts from 1900, module gives yy
  t.tm_mon  = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min  = minute;
  t.tm_sec  = second;

  time_t local = mktime(&t);

  // <zone> is the offset from UTC in quarter-hours; back it out to get UTC.
  struct timeval tv = { local - (zone * 15 * 60), 0 };
  settimeofday(&tv, NULL);

  Serial.printf("[GSM] Clock synced from network: %s\n", getTimestamp().c_str());
  return true;
}

// Full bring-up: proves the module, then the SIM, then the network. Each stage
// is reported separately so a boot log distinguishes "no module" from
// "no SIM" from "SIM present but no coverage".
bool gsmInit() {
  Serial.println("\n[GSM] ---------- SIM900A bring-up ----------");

  gsmSimOk = false;
  gsmRegistered = false;
  gsmDataReady = false;
  gsmCarrierLocked = false;
  gsmSignal = -1;
  gsmRegStatus = -1;

  GSM.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(1000);

  // ---- 1. Module alive? Answers with no SIM inserted. ----
  bool alive = false;

  for (int attempt = 1; attempt <= 3 && !alive; attempt++) {
    if (gsmSendAT("AT", 3000).indexOf("OK") >= 0) {
      alive = true;
    } else {
      Serial.printf("[GSM] No reply to AT (attempt %d/3)...\n", attempt);
      delay(2000);
    }
  }

  if (!alive) {
    Serial.println("[GSM] Module not responding — check TX/RX wiring, GND and the 5V/2A supply.");
    return false;
  }

  gsmSendAT("ATE0");        // echo off, keeps the parsing simple
  gsmSendAT("AT+CMEE=2");   // verbose errors ("SIM not inserted" not "ERROR")

  gsmIMEI = gsmDigitsOnly(gsmSendAT("AT+GSN"), 15);
  Serial.printf("[GSM] Module OK. IMEI: %s\n",
                gsmIMEI.length() ? gsmIMEI.c_str() : "unknown");

  // The UART answers AT several seconds before the SIM interface is powered.
  // In minimum-functionality mode (CFUN=0) the SIM is not powered at all, and
  // every CPIN query reports "not inserted" no matter what is in the holder.
  String fun = gsmSendAT("AT+CFUN?", 5000);

  if (gsmValueAfter(fun, "+CFUN:").toInt() != 1) {
    Serial.println("[GSM] Module was in low-functionality mode — enabling full RF/SIM...");
    gsmSendAT("AT+CFUN=1", 15000);
    delay(3000);
  }

  // ---- 2. SIM present? This is the check that a valid CSQ does NOT prove. ----
  // Poll rather than ask once: a cold SIM900A commonly reports "SIM busy" or
  // "NOT READY" for the first 5-15 s after power-up.
  String pin;

  unsigned long simStart = millis();

  while (millis() - simStart < 20000) {
    pin = gsmSendAT("AT+CPIN?", 5000);

    if (pin.indexOf("READY") >= 0) {
      gsmSimOk = true;
      break;
    }

    // "PH-NET PIN" and friends are personalization locks on the MODULE, not
    // the card. The SIM has already been read at this point — waiting or
    // reseating it changes nothing, so stop polling immediately.
    if (pin.indexOf("PH-") >= 0) {
      if (strlen(GSM_NCK) == 0) {
        break;
      }

      Serial.println("[GSM] Module is network-locked — trying the configured NCK...");

      String unlock = String("AT+CPIN=\"") + GSM_NCK + "\"";
      gsmSendAT(unlock.c_str(), 10000);
      delay(3000);

      pin = gsmSendAT("AT+CPIN?", 5000);

      if (pin.indexOf("READY") >= 0) {
        Serial.println("[GSM] Network lock cleared.");
        gsmSimOk = true;
      } else {
        Serial.println("[GSM] NCK rejected — the code does not match this module.");
      }

      break;
    }

    if (pin.indexOf("SIM PIN") >= 0 || pin.indexOf("SIM PUK") >= 0) {
      break;    // locked, waiting will not help
    }

    Serial.println("[GSM] SIM not ready yet, waiting...");
    delay(2000);
  }

  if (!gsmSimOk) {
    if (pin.indexOf("PH-") >= 0) {
      gsmCarrierLocked = true;
      Serial.println("[GSM] ***** MODULE IS CARRIER-LOCKED *****");
      Serial.printf ("[GSM] Reply: %s\n", pin.c_str());
      Serial.println("[GSM] The SIM IS being read — the module refuses this operator.");
      Serial.println("[GSM] Fix: get the NCK unlock code from the seller and set GSM_NCK,");
      Serial.println("[GSM]      or use a SIM from the operator the module is locked to,");
      Serial.println("[GSM]      or replace the module with an unlocked one.");
      Serial.printf ("[GSM] Quote this IMEI to the seller: %s\n",
                     gsmIMEI.length() ? gsmIMEI.c_str() : "unknown");
    } else if (pin.indexOf("SIM PIN") >= 0) {
      Serial.println("[GSM] SIM is PIN-locked — disable the PIN on a phone first.");
    } else if (pin.indexOf("SIM PUK") >= 0) {
      Serial.println("[GSM] SIM is PUK-locked — unlock it on a phone.");
    } else {
      Serial.println("[GSM] NO SIM DETECTED after 20s.");
      Serial.println("[GSM]   1. Power: SIM900A needs 5V/2A of its OWN — it browns out on ESP32/USB power.");
      Serial.println("[GSM]   2. Holder: card clicked fully in, gold contacts DOWN, notch matching the outline.");
      Serial.println("[GSM]   3. Adapter: a nano SIM in a cheap adapter often loses contact — try a full-size card.");
      Serial.printf ("[GSM]   Last CPIN reply was: %s\n", pin.length() ? pin.c_str() : "(no reply)");
    }

    Serial.println("[GSM] Data will fall back to WiFi.");
    return true;    // module itself is fine
  }

  gsmICCID = gsmDigitsOnly(gsmSendAT("AT+CCID", 5000), 15);
  Serial.printf("[GSM] SIM detected. ICCID: %s\n",
                gsmICCID.length() ? gsmICCID.c_str() : "unreadable");

  // ---- 3. Own number. Only present if the operator wrote MSISDN to the SIM. ----
  gsmNumber = gsmQuotedField(gsmSendAT("AT+CNUM", 5000), 2);

  if (gsmNumber.length()) {
    Serial.printf("[GSM] SIM number: %s\n", gsmNumber.c_str());
  } else {
    Serial.println("[GSM] SIM number not stored on the card (normal — not an error).");
  }

  // ---- 4. Wait for network registration. ----
  gsmSendAT("AT+CLTS=1");   // enable network time, used by gsmSyncTime()
  gsmSendAT("AT+CREG=0");

  unsigned long start = millis();

  while (millis() - start < 60000) {
    gsmRegStatus = gsmParseCreg(gsmSendAT("AT+CREG?"));

    if (gsmRegStatus == 1 || gsmRegStatus == 5) {
      gsmRegistered = true;
      break;
    }

    if (gsmRegStatus == 3) {
      Serial.println("[GSM] Registration DENIED — SIM may be inactive or unregistered with the operator.");
      break;
    }

    Serial.println("[GSM] Searching for network...");
    delay(3000);
  }

  gsmSignal = gsmParseCsq(gsmSendAT("AT+CSQ"));

  if (!gsmRegistered) {
    Serial.printf("[GSM] Not registered (CREG stat=%d, CSQ=%d) — check antenna, coverage and SIM credit.\n",
                  gsmRegStatus, gsmSignal);
    return true;
  }

  gsmOperator = gsmQuotedField(gsmSendAT("AT+COPS?", 10000), 1);

  Serial.printf("[GSM] Registered on %s (%s), CSQ %d (~%d dBm)\n",
                gsmOperator.length() ? gsmOperator.c_str() : "network",
                gsmRegStatus == 5 ? "roaming" : "home",
                gsmSignal,
                gsmSignal > 0 && gsmSignal < 32 ? -113 + (2 * gsmSignal) : 0);

  if (gsmSignal >= 0 && gsmSignal < 10) {
    Serial.println("[GSM] WARNING: weak signal — GPRS uploads may time out. Reposition the antenna.");
  }

  // ---- 5. GPRS attach + clear any bearer left open by a previous run. ----
  gsmSendAT("AT+CGATT=1", 15000);

  String attached = gsmSendAT("AT+CGATT?");
  if (gsmValueAfter(attached, "+CGATT:").toInt() != 1) {
    Serial.println("[GSM] GPRS not attached — the SIM may have no data bundle.");
  }

  gsmSendAT("AT+SAPBR=0,1", 10000);   // close stale bearer, error here is fine

  gsmSyncTime();

  gsmDataReady = true;

  Serial.println("[GSM] READY — GSM is the primary data link.");
  Serial.println("[GSM] --------------------------------------");

  return true;
}

bool gsmGprsAttach() {
  gsmSendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");

  String apnCmd = String("AT+SAPBR=3,1,\"APN\",\"") + GPRS_APN + "\"";
  gsmSendAT(apnCmd.c_str());

  String result = gsmSendAT("AT+SAPBR=1,1", 15000);

  if (result.indexOf("OK") >= 0) {
    return true;
  }

  // "already open" from a previous cycle still gives a usable bearer.
  String query = gsmSendAT("AT+SAPBR=2,1", 10000);
  return query.indexOf("+SAPBR: 1,1") >= 0;
}

// Closes the GPRS bearer. Note the argument order: <cmd_type>,<cid> — so
// closing cid 1 is "0,1".
void gsmGprsDetach() {
  gsmSendAT("AT+SAPBR=0,1", 10000);
}

bool sendViaGSM(const String& json) {
  Serial.println("[API] Sending data via GSM...");

  if (!gsmDataReady) {
    Serial.println("[GSM] Skipped — no registered SIM.");
    return false;
  }

  if (!gsmGprsAttach()) {
    Serial.println("[GSM] GPRS attach failed — check the APN and that the SIM has data.");
    return false;
  }

  gsmSendAT("AT+HTTPTERM");   // drop a session left open by a failed cycle

  if (gsmSendAT("AT+HTTPINIT").indexOf("OK") < 0) {
    Serial.println("[GSM] HTTP init failed.");
    gsmGprsDetach();
    return false;
  }

  gsmSendAT("AT+HTTPPARA=\"CID\",1");

  String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + API_URL + "\"";
  gsmSendAT(urlCmd.c_str());
  gsmSendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  gsmSendAT("AT+HTTPPARA=\"REDIR\",1");

  if (String(API_URL).startsWith("https")) {
    if (gsmSendAT("AT+HTTPSSL=1").indexOf("OK") < 0) {
      Serial.println("[GSM] WARNING: module rejected AT+HTTPSSL — this firmware cannot do HTTPS.");
    }
  }

  String authHeader = String("Authorization: Bearer ") + AUTH_TOKEN;
  String authCmd = String("AT+HTTPPARA=\"USERDATA\",\"") + authHeader + "\"";
  gsmSendAT(authCmd.c_str());

  String dataCmd = "AT+HTTPDATA=" + String(json.length()) + ",30000";
  GSM.println(dataCmd);
  Serial.printf("[GSM] >> %s\n", dataCmd.c_str());

  unsigned long start = millis();
  String prompt;

  while (millis() - start < 5000) {
    while (GSM.available()) {
      prompt += (char)GSM.read();
    }
    if (prompt.indexOf("DOWNLOAD") >= 0) {
      break;
    }
    delay(10);
  }

  if (prompt.indexOf("DOWNLOAD") < 0) {
    Serial.println("[GSM] HTTPDATA prompt not received.");
    gsmSendAT("AT+HTTPTERM");
    gsmGprsDetach();
    return false;
  }

  GSM.print(json);
  delay(1000);

  String action = gsmSendAT("AT+HTTPACTION=1", 60000);
  gsmSendAT("AT+HTTPREAD");
  gsmSendAT("AT+HTTPTERM");
  gsmGprsDetach();

  bool ok = (
    action.indexOf(",200,") >= 0 ||
    action.indexOf(",201,") >= 0
  );

  if (ok) {
    Serial.println("[API] Data sent successfully via GSM.");
  } else {
    Serial.printf("[GSM] HTTP POST failed: %s\n", action.c_str());
  }

  return ok;
}

bool sendViaWiFi(const String& json) {
  Serial.println("[API] Sending data via WiFi...");

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;

  if (!https.begin(client, API_URL)) {
    Serial.println("[WIFI] Failed to begin HTTPS request.");
    return false;
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));

  int httpCode = https.POST(json);

  Serial.printf("[API] POST Result: %d\n", httpCode);

  if (httpCode > 0) {
    Serial.println("[API] Response:");
    Serial.println(https.getString());
  } else {
    Serial.println("[API] Failed Request");
  }

  https.end();

  bool ok = (httpCode >= 200 && httpCode < 300);

  if (ok) {
    Serial.println("[API] Data sent successfully via WiFi.");
  }

  return ok;
}

// -------------------- UTILITIES --------------------
String getDeviceID() {
  uint64_t chipid = ESP.getEfuseMac();

  char deviceID[25];

  snprintf(
    deviceID,
    sizeof(deviceID),
    "ITEDA-%04X%08X",
    (uint16_t)(chipid >> 32),
    (uint32_t)chipid
  );

  return String(deviceID);
}

String getTimestamp() {
  time_t now;
  time(&now);

  struct tm *ti = gmtime(&now);

  if (ti->tm_year < 100) {
    return "NTP_NOT_READY";
  }

  char buf[30];
  strftime(buf, 30, "%Y-%m-%dT%H:%M:%SZ", ti);

  return String(buf);
}

// -------------------- OTA --------------------
void checkOTA() {

  Serial.println("\n[SYSTEM] Checking for updates...");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] Skipped — WiFi not connected.");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(20000);

  HTTPClient http;
  http.setTimeout(20000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, MANIFEST_URL)) {
    Serial.println("[OTA] Failed to begin manifest request.");
    return;
  }

  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    Serial.printf("[OTA] Failed to fetch manifest. HTTP Code: %d\n", code);
    http.end();
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, http.getString());

  // Must release the manifest connection before starting the binary download
  // on the same WiFiClientSecure — otherwise HTTPUpdate silently fails.
  http.end();

  if (err) {
    Serial.printf("[OTA] Manifest JSON parse error: %s\n", err.c_str());
    return;
  }

  const char* newVersion = doc["version"] | "";
  const char* binUrl = doc["bin_url"] | "";

  if (strlen(newVersion) == 0 || strlen(binUrl) == 0) {
    Serial.println("[OTA] Manifest missing version or bin_url.");
    return;
  }

  Serial.printf("[OTA] Manifest version: %s | Device version: %s\n", newVersion, VERSION);
  Serial.printf("[OTA] bin_url: %s\n", binUrl);

  if (strcmp(newVersion, VERSION) == 0) {
    Serial.println("[OTA] System up to date.");
    return;
  }

  // Copy out of the JsonDocument before any further network use.
  String updateUrl = String(binUrl);

  Serial.printf(
    "[OTA] New version found: %s (current: %s). Downloading...\n",
    newVersion,
    VERSION
  );

  digitalWrite(LED_YELLOW, HIGH);

  // Fresh TLS client for the binary download.
  WiFiClientSecure updateClient;
  updateClient.setInsecure();
  updateClient.setTimeout(20000);

  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);

  t_httpUpdate_return ret = httpUpdate.update(updateClient, updateUrl);

  digitalWrite(LED_YELLOW, LOW);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(
        "[OTA] Update failed (%d): %s\n",
        httpUpdate.getLastError(),
        httpUpdate.getLastErrorString().c_str()
      );
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA] No updates available.");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("[OTA] Update OK — rebooting.");
      break;
  }
}

// -------------------- API --------------------
void buildPayload(JsonDocument& doc, float t[], float h[], int m[], int currentRaw) {

  // =========================================================
  // MATCHING SUPABASE sensor_readings TABLE
  // =========================================================

  doc["dryer_id"] = getDeviceID();

  doc["timestamp"] = getTimestamp();

  doc["firmware_version"] = VERSION;

  // Main DB columns
  doc["chamber_temp"] = t[1];          // Middle chamber temp
  doc["ambient_temp"] = t[3];
  doc["heater_temp"] = t[0];           // Top sensor near heater

  doc["internal_humidity"] = h[1];
  doc["external_humidity"] = h[3];

  doc["fan_status"] = true;

  doc["heater_status"] = heaterActive;

  doc["door_status"] = false;          // No door sensor yet

  doc["power_consumption_w"] = currentRaw;

  doc["charging_status"] = "unknown";

  doc["active_preset_id"] = nullptr;

  // =========================================================
  // EXTRA SENSOR VALUES -> JSONB sensor_values
  // =========================================================

  JsonObject sensorValues = doc.createNestedObject("sensor_values");

  // Temperatures
  sensorValues["temp_chamber_bottom"] = t[0];
  sensorValues["temp_chamber_middle"] = t[1];
  sensorValues["temp_chamber_top"] = t[2];
  sensorValues["temp_ambient"] = t[3];

  // Humidity
  sensorValues["hum_chamber_bottom"] = h[0];
  sensorValues["hum_chamber_middle"] = h[1];
  sensorValues["hum_chamber_top"] = h[2];
  sensorValues["hum_ambient"] = h[3];

  // Moisture
  sensorValues["tray_1_moisture"] = m[0];
  sensorValues["tray_2_moisture"] = m[1];
  sensorValues["tray_3_moisture"] = m[2];
  sensorValues["tray_4_moisture"] = m[3];

  // Fan
  sensorValues["fan_speed_rpm"] = 0;   // No RPM sensor yet

  // PID + system telemetry
  sensorValues["pid_setpoint"] = Setpoint;
  sensorValues["pid_output"] = Output;
  sensorValues["pid_duty_percent"] = (Output / WindowSize) * 100.0;

  sensorValues["current_adc_raw"] = currentRaw;

  sensorValues["firmware_version"] = VERSION;

  sensorValues["gsm_ready"] = gsmReady;
  sensorValues["gsm_sim_present"] = gsmSimOk;
  sensorValues["gsm_carrier_locked"] = gsmCarrierLocked;
  sensorValues["gsm_registered"] = gsmRegistered;
  sensorValues["gsm_data_ready"] = gsmDataReady;
  sensorValues["gsm_signal_csq"] = gsmSignal;
  sensorValues["gsm_signal_dbm"] =
    (gsmSignal > 0 && gsmSignal < 32) ? -113 + (2 * gsmSignal) : 0;
  sensorValues["gsm_operator"] = gsmOperator;
  sensorValues["gsm_number"] = gsmNumber;
  sensorValues["gsm_iccid"] = gsmICCID;
  sensorValues["gsm_imei"] = gsmIMEI;
  sensorValues["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  sensorValues["wifi_rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;

  // Outcome of the transmission before this one — the result of the current
  // send is only known after the payload has already left the device.
  sensorValues["previous_send_success"] = lastSendSuccess;
  sensorValues["previous_send_method"] = lastSendMethod;

  sensorValues["uptime_ms"] = millis();
}

// Stamps the transport carrying this payload so the server sees the link
// that actually delivered it, not the one used on the previous cycle.
void setTransport(JsonDocument& doc, const char* method) {
  doc["connection_type"] = method;
  doc["sensor_values"]["data_send_method"] = method;
}

void sendPayload(float t[], float h[], int m[], int currentRaw) {

  // Heap, not stack: the added modem identity fields push this past what the
  // 8 KB loop-task stack can safely hold alongside the serialized copy.
  DynamicJsonDocument doc(3072);

  buildPayload(doc, t, h, m, currentRaw);

  bool wifiUp = (WiFi.status() == WL_CONNECTED);

  setTransport(doc, gsmDataReady ? "gsm" : (wifiUp ? "wifi" : "none"));

  // ---------------- SERIAL DEBUG ----------------
  Serial.println("\n================================================");
  Serial.printf(">>> OUTGOING PAYLOAD v%s <<<\n", VERSION);

  serializeJsonPretty(doc, Serial);

  Serial.println("\n================================================");

  // ---------------- SEND (GSM primary, WiFi fallback) ----------------
  bool sent = false;
  const char* method = "none";

  if (gsmDataReady) {
    String json;
    serializeJson(doc, json);

    sent = sendViaGSM(json);

    if (sent) {
      method = "gsm";
    }
  }

  if (!sent && wifiUp) {
    if (!gsmDataReady) {
      Serial.println("[API] GSM unavailable — falling back to WiFi...");
    } else {
      Serial.println("[API] GSM send failed — falling back to WiFi...");
    }

    setTransport(doc, "wifi");

    String json;
    serializeJson(doc, json);

    sent = sendViaWiFi(json);

    if (sent) {
      method = "wifi";
    }
  }

  if (!sent) {
    Serial.println("[API] Failed to send data — no working connection.");
  }

  lastSendMethod = method;
  lastSendSuccess = sent;

  Serial.printf(
    "\n[API] >>> Data sent via: %s (%s) <<<\n\n",
    lastSendMethod,
    lastSendSuccess ? "success" : "failed"
  );
}

// -------------------- MAIN --------------------
void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println("\n====================================");
  Serial.printf(" ITEDA DRYER SYSTEM v%s\n", VERSION);
  Serial.printf(" Hardware ID: %s\n", getDeviceID().c_str());
  Serial.println("====================================");

  pinMode(HEATER_1, OUTPUT);
  pinMode(HEATER_2, OUTPUT);

  pinMode(FAN_RELAY, OUTPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  for (int i = 0; i < 4; i++) {
    dhts[i].begin();
  }

  analogReadResolution(12);

  // ---------------- GSM (primary link) ----------------
  gsmReady = gsmInit();

  // ---------------- WIFI (fallback link + OTA) ----------------
  Serial.print("[WIFI] Connecting to " + String(ssid));

  WiFi.begin(ssid, password);

  unsigned long wifiStart = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - wifiStart < 30000
  ) {

    digitalWrite(
      LED_YELLOW,
      !digitalRead(LED_YELLOW)
    );

    Serial.print(".");

    delay(500);
  }

  digitalWrite(LED_YELLOW, LOW);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected!");
    Serial.println(WiFi.localIP());
    configTime(0, 0, "pool.ntp.org");
  } else if (gsmDataReady) {
    Serial.println("\n[WIFI] Not connected — running on GSM only (OTA disabled).");
  } else {
    Serial.println("\n[WIFI] Connection failed and no GSM data link — device is offline.");
  }

  // ---------------- LINK SUMMARY ----------------
  Serial.println("\n[SYSTEM] ---------- Link status ----------");
  Serial.printf("[SYSTEM] GSM module   : %s\n", gsmReady ? "detected" : "NOT DETECTED");
  Serial.printf("[SYSTEM] SIM card     : %s\n",
                gsmSimOk ? "detected"
                         : (gsmCarrierLocked ? "read, but module is CARRIER-LOCKED" : "NOT DETECTED"));
  Serial.printf("[SYSTEM] Registration : %s\n", gsmRegistered ? "registered" : "not registered");
  Serial.printf("[SYSTEM] Primary link : %s\n",
                gsmDataReady ? "GSM/GPRS"
                             : (WiFi.status() == WL_CONNECTED ? "WiFi (GSM unavailable)" : "none"));
  Serial.println("[SYSTEM] ---------------------------------\n");

  // ---------------- PID ----------------
  windowStartTime = millis();

  myPID.SetOutputLimits(0, WindowSize);

  myPID.SetMode(AUTOMATIC);

  // ---------------- OTA ----------------
  checkOTA();
}

void loop() {

  float ts[4];
  float hs[4];

  // ---------------- READ DHT ----------------
  for (int i = 0; i < 4; i++) {

    ts[i] = dhts[i].readTemperature();

    hs[i] = dhts[i].readHumidity();
  }

  // ---------------- READ MOISTURE ----------------
  int ms[4] = {
    analogRead(MOISTURE1),
    analogRead(MOISTURE2),
    analogRead(MOISTURE3),
    analogRead(MOISTURE4)
  };

  // ---------------- PID ----------------
  if (!isnan(ts[1])) {

    Input = ts[1];

    myPID.Compute();
  }

  // ---------------- HEATER WINDOW ----------------
  unsigned long now = millis();

  if (now - windowStartTime > WindowSize) {
    windowStartTime += WindowSize;
  }

  heaterActive = (
    Output > (now - windowStartTime)
  );

  // ---------------- OUTPUTS ----------------
  digitalWrite(HEATER_1, heaterActive);
  digitalWrite(HEATER_2, heaterActive);

  digitalWrite(FAN_RELAY, HIGH);

  // ---------------- GREEN HEARTBEAT ----------------
  if (now - lastBlinkGreen > 1000) {

    greenState = !greenState;

    digitalWrite(LED_GREEN, greenState);

    lastBlinkGreen = now;
  }

  // ---------------- RED HEATER STATUS ----------------
  if (heaterActive) {

    digitalWrite(LED_RED, HIGH);

  } else if (now - lastBlinkRed > 2000) {

    redState = !redState;

    digitalWrite(LED_RED, redState);

    lastBlinkRed = now;
  }

  // ---------------- API SEND ----------------
  static unsigned long lastSend = 0;

  if (now - lastSend > 10000) {

    sendPayload(
      ts,
      hs,
      ms,
      analogRead(CURRENT_PIN)
    );

    lastSend = now;
  }

  // ---------------- GSM HEARTBEAT ----------------
  static unsigned long lastGsm = 0;

  if (gsmReady && now - lastGsm > 60000) {
    gsmSignal = gsmParseCsq(gsmSendAT("AT+CSQ"));
    gsmRegStatus = gsmParseCreg(gsmSendAT("AT+CREG?"));

    bool wasRegistered = gsmRegistered;
    gsmRegistered = (gsmRegStatus == 1 || gsmRegStatus == 5);

    if (gsmRegistered && !wasRegistered) {
      // Came back (or a SIM was inserted after boot) — redo the bring-up so
      // the ICCID/number/operator fields are populated too.
      Serial.println("[GSM] Network recovered — re-running bring-up.");
      gsmReady = gsmInit();
    } else if (!gsmRegistered && wasRegistered) {
      Serial.printf("[GSM] Lost registration (stat=%d) — falling back to WiFi.\n", gsmRegStatus);
      gsmDataReady = false;
    } else if (!gsmSimOk && !gsmCarrierLocked) {
      // No card at boot: cheap re-check so inserting one doesn't need a reset.
      // Skipped when carrier-locked — that never clears without the NCK.
      if (gsmSendAT("AT+CPIN?", 5000).indexOf("READY") >= 0) {
        Serial.println("[GSM] SIM inserted — re-running bring-up.");
        gsmReady = gsmInit();
      }
    }

    lastGsm = now;
  }

  // ---------------- OTA CHECK ----------------
  static unsigned long lastOTA = 0;

  if (now - lastOTA > 3600000) {

    checkOTA();

    lastOTA = now;
  }
}
