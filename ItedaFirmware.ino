#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>

// -------------------- CONFIGURATION --------------------
const char* VERSION = "2.0";
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";
const char* GPRS_APN = "internet";  // Airtel Kenya
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

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

bool gsmReady = false;
int gsmSignal = -1;
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

bool gsmInit() {
  Serial.println("\n[GSM] Initializing SIM900A...");

  GSM.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  delay(1000);

  String at = gsmSendAT("AT", 3000);
  if (at.indexOf("OK") < 0) {
    Serial.println("[GSM] No response to AT — check wiring and power.");
    return false;
  }

  gsmSendAT("ATE0");  // disable echo

  String csq = gsmSendAT("AT+CSQ");
  int comma = csq.indexOf("+CSQ:");
  if (comma >= 0) {
    gsmSignal = csq.substring(comma + 6, comma + 8).toInt();
    Serial.printf("[GSM] Signal strength (CSQ): %d\n", gsmSignal);
  }

  Serial.println("[GSM] Module responding.");
  return true;
}

bool gsmGprsAttach() {
  gsmSendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");

  String apnCmd = String("AT+SAPBR=3,1,\"APN\",\"") + GPRS_APN + "\"";
  gsmSendAT(apnCmd.c_str());

  String result = gsmSendAT("AT+SAPBR=1,1", 15000);
  return result.indexOf("OK") >= 0;
}

bool sendViaGSM(const String& json) {
  Serial.println("[API] Sending data via GSM...");

  if (!gsmGprsAttach()) {
    Serial.println("[GSM] GPRS attach failed.");
    return false;
  }

  if (gsmSendAT("AT+HTTPINIT").indexOf("OK") < 0) {
    Serial.println("[GSM] HTTP init failed.");
    return false;
  }

  String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + API_URL + "\"";
  gsmSendAT(urlCmd.c_str());
  gsmSendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");

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
    gsmSendAT("AT+SAPBR=1,0");
    return false;
  }

  GSM.print(json);
  delay(1000);

  String action = gsmSendAT("AT+HTTPACTION=1", 60000);
  gsmSendAT("AT+HTTPREAD");
  gsmSendAT("AT+HTTPTERM");
  gsmSendAT("AT+SAPBR=1,0");

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

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;

  if (http.begin(client, MANIFEST_URL)) {

    int code = http.GET();

    if (code == HTTP_CODE_OK) {

      StaticJsonDocument<256> doc;

      deserializeJson(doc, http.getString());

      const char* newVersion = doc["version"];

      if (strcmp(newVersion, VERSION) != 0) {

        Serial.printf(
          "[OTA] New Version Found: %s. Current: %s\n",
          newVersion,
          VERSION
        );

        digitalWrite(LED_YELLOW, HIGH);

        httpUpdate.update(client, (const char*)doc["bin_url"]);

      } else {

        Serial.println("[OTA] System up to date.");
      }

    } else {

      Serial.printf(
        "[OTA] Failed to fetch manifest. HTTP Code: %d\n",
        code
      );
    }

    http.end();
  }
}

// -------------------- API --------------------
void sendPayload(float t[], float h[], int m[], int currentRaw) {

  StaticJsonDocument<2048> doc;

  // =========================================================
  // MATCHING SUPABASE sensor_readings TABLE
  // =========================================================

  doc["dryer_id"] = getDeviceID();

  doc["timestamp"] = getTimestamp();

  // Main DB columns
  doc["chamber_temp"] = t[1];          // Middle chamber temp
  doc["ambient_temp"] = t[3];
  doc["heater_temp"] = t[0];           // Top sensor near heater

  doc["internal_humidity"] = h[1];
  doc["external_humidity"] = h[3];

  doc["fan_speed_rpm"] = 0;            // No RPM sensor yet
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

  // PID + system telemetry
  sensorValues["pid_setpoint"] = Setpoint;
  sensorValues["pid_output"] = Output;
  sensorValues["pid_duty_percent"] = (Output / WindowSize) * 100.0;

  sensorValues["current_adc_raw"] = currentRaw;

  sensorValues["firmware_version"] = VERSION;

  sensorValues["gsm_ready"] = gsmReady;
  sensorValues["gsm_signal_csq"] = gsmSignal;
  sensorValues["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  sensorValues["wifi_rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  sensorValues["data_send_method"] = lastSendMethod;
  sensorValues["data_send_success"] = lastSendSuccess;

  sensorValues["uptime_ms"] = millis();

  // =========================================================

  String json;
  serializeJson(doc, json);

  // ---------------- SERIAL DEBUG ----------------
  Serial.println("\n================================================");
  Serial.printf(">>> OUTGOING PAYLOAD v%s <<<\n", VERSION);

  serializeJsonPretty(doc, Serial);

  Serial.println("\n================================================");

  // ---------------- SEND (WiFi first, GSM fallback) ----------------
  bool sent = false;

  if (WiFi.status() == WL_CONNECTED) {
    sent = sendViaWiFi(json);
    if (sent) {
      lastSendMethod = "wifi";
    }
  }

  if (!sent && gsmReady) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[API] WiFi unavailable — trying GSM...");
    } else {
      Serial.println("[API] WiFi send failed — trying GSM...");
    }
    sent = sendViaGSM(json);
    if (sent) {
      lastSendMethod = "gsm";
    }
  }

  if (!sent) {
    lastSendMethod = "none";
    Serial.println("[API] Failed to send data — no working connection.");
  }

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

  // ---------------- WIFI ----------------
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
  } else {
    Serial.println("\n[WIFI] Connection failed — will use GSM for data if available.");
  }

  // ---------------- PID ----------------
  windowStartTime = millis();

  myPID.SetOutputLimits(0, WindowSize);

  myPID.SetMode(AUTOMATIC);

  // ---------------- GSM ----------------
  gsmReady = gsmInit();

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
    String csq = gsmSendAT("AT+CSQ");
    int comma = csq.indexOf("+CSQ:");
    if (comma >= 0) {
      gsmSignal = csq.substring(comma + 6, comma + 8).toInt();
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
