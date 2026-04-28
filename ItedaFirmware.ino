#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>

// -------------------- CONFIGURATION --------------------
const char* VERSION = "v1.2"; 
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- PIN DEFINITIONS --------------------
#define DHTPIN1 7    // Chamber Bottom
#define DHTPIN2 8    // Chamber Middle
#define DHTPIN3 11   // Chamber Top
#define DHTPIN4 12   // Ambient
#define DHTTYPE DHT22

#define MOISTURE1 1  // Tray 1
#define MOISTURE2 2  // Tray 2
#define MOISTURE3 4  // Tray 3
#define MOISTURE4 5  // Tray 4

#define HEATER_1   21
#define HEATER_2   10
#define FAN_RELAY  18

#define LED_RED    15
#define LED_YELLOW 16
#define LED_GREEN  13

#define CURRENT_PIN 6

// -------------------- GLOBALS --------------------
DHT dhts[] = {{DHTPIN1, DHTTYPE}, {DHTPIN2, DHTTYPE}, {DHTPIN3, DHTTYPE}, {DHTPIN4, DHTTYPE}};
double Setpoint = 47.5, Input, Output;
double Kp=2, Ki=5, Kd=1; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000; 
unsigned long windowStartTime;
bool heaterActive = false;

unsigned long lastBlinkGreen = 0;
unsigned long lastBlinkRed = 0;
bool greenState = false, redState = false;

// -------------------- UTILITIES --------------------
String getDeviceID() {
  uint64_t chipid = ESP.getEfuseMac(); 
  char deviceID[25];
  snprintf(deviceID, sizeof(deviceID), "ITEDA-%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  return String(deviceID);
}

String getTimestamp() {
  time_t now; time(&now);
  struct tm *ti = gmtime(&now);
  if (ti->tm_year < 100) return "NTP_NOT_READY";
  char buf[30]; strftime(buf, 30, "%Y-%m-%dT%H:%M:%SZ", ti);
  return String(buf);
}

// -------------------- OTA --------------------
void checkOTA() {
  Serial.println("\n[SYSTEM] Checking for updates...");
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;

  if (http.begin(client, MANIFEST_URL)) {
    int code = http.GET();
    if (code == HTTP_CODE_OK) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, http.getString());
      const char* newVersion = doc["version"];
      if (strcmp(newVersion, VERSION) != 0) {
        Serial.printf("[OTA] New Version Found: %s. Current: %s. Downloading...\n", newVersion, VERSION);
        digitalWrite(LED_YELLOW, HIGH);
        httpUpdate.update(client, (const char*)doc["bin_url"]);
      } else {
        Serial.println("[OTA] System up to date.");
      }
    } else {
      Serial.printf("[OTA] Failed to fetch manifest. HTTP Code: %d\n", code);
    }
    http.end();
  }
}

// -------------------- API --------------------
void sendPayload(float t[], float h[], int m[], int currentRaw) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Disconnected! Cannot send payload.");
    return;
  }

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  StaticJsonDocument<1024> doc;

  doc["dryer_id"] = getDeviceID(); 
  doc["version"] = VERSION;
  doc["timestamp"] = getTimestamp();
  doc["temp_chamber_bottom"] = t[0]; 
  doc["temp_chamber_middle"] = t[1];
  doc["temp_chamber_top"] = t[2];
  doc["temp_ambient"] = t[3];
  doc["hum_chamber_bottom"] = h[0];
  doc["hum_ambient"] = h[3];
  doc["tray_1_moisture"] = m[0];
  doc["tray_2_moisture"] = m[1];
  doc["tray_3_moisture"] = m[2];
  doc["tray_4_moisture"] = m[3];
  doc["heater_active"] = heaterActive;
  doc["fan_active"] = true;
  doc["pid_duty_percent"] = (Output / WindowSize) * 100.0;
  doc["current_adc_raw"] = currentRaw;

  String json;
  serializeJson(doc, json);

  // --- EXTENSIVE SERIAL DEBUGGING ---
  Serial.println("\n" + String(new char[50]{'-'}));
  Serial.println(">>> OUTGOING DATA PAYLOAD <<<");
  Serial.printf("Device ID : %s\n", getDeviceID().c_str());
  Serial.printf("Timestamp : %s\n", getTimestamp().c_str());
  Serial.println("--- Temperatures ---");
  Serial.printf(" Bottom: %.2fC | Mid: %.2fC | Top: %.2fC | Amb: %.2fC\n", t[0], t[1], t[2], t[3]);
  Serial.println("--- Trays (Moisture) ---");
  Serial.printf(" T1: %d | T2: %d | T3: %d | T4: %d\n", m[0], m[1], m[2], m[3]);
  Serial.println("--- Control Status ---");
  Serial.printf(" Heater: %s | PID Duty: %.1f%% | Current Raw: %d\n", heaterActive ? "ON" : "OFF", (Output/WindowSize)*100.0, currentRaw);
  Serial.println(String(new char[50]{'-'}));

  if (https.begin(client, API_URL)) {
    https.addHeader("Content-Type", "application/json");
    https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));
    int httpCode = https.POST(json);
    Serial.printf("[API] POST Result: %d\n", httpCode);
    if(httpCode != 200) {
      Serial.println("[API] Error: " + https.getString());
    }
    https.end();
  }
}

// -------------------- MAIN --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n====================================");
  Serial.printf("   ITEDA DRYER SYSTEM v%s\n", VERSION);
  Serial.printf("   Hardware ID: %s\n", getDeviceID().c_str());
  Serial.println("====================================");

  pinMode(HEATER_1, OUTPUT); pinMode(HEATER_2, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT); pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT); pinMode(LED_GREEN, OUTPUT);

  for (int i = 0; i < 4; i++) dhts[i].begin();
  analogReadResolution(12);

  Serial.print("[WIFI] Connecting to " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW));
    Serial.print(".");
    delay(500);
  }
  digitalWrite(LED_YELLOW, LOW);
  Serial.println("\n[WIFI] Connected! IP: " + WiFi.localIP().toString());

  configTime(0, 0, "pool.ntp.org");
  windowStartTime = millis();
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  checkOTA(); 
}

void loop() {
  float ts[4], hs[4];
  for (int i = 0; i < 4; i++) {
    ts[i] = dhts[i].readTemperature();
    hs[i] = dhts[i].readHumidity();
  }
  int ms[4] = {analogRead(MOISTURE1), analogRead(MOISTURE2), analogRead(MOISTURE3), analogRead(MOISTURE4)};

  if (!isnan(ts[0])) {
    Input = ts[0];
    myPID.Compute();
  }

  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) windowStartTime += WindowSize;
  heaterActive = (Output > (now - windowStartTime));

  digitalWrite(HEATER_1, heaterActive);
  digitalWrite(HEATER_2, heaterActive);
  digitalWrite(FAN_RELAY, HIGH);

  // Status Pulse (Green)
  if (now - lastBlinkGreen > 1000) {
    greenState = !greenState;
    digitalWrite(LED_GREEN, greenState);
    lastBlinkGreen = now;
  }
  
  // Heater Indicator (Red)
  if (heaterActive) {
    digitalWrite(LED_RED, HIGH);
  } else if (now - lastBlinkRed > 2000) {
    redState = !redState;
    digitalWrite(LED_RED, redState);
    lastBlinkRed = now;
  }

  // Periodic API Task
  static unsigned long lastSend = 0;
  if (now - lastSend > 10000) {
    sendPayload(ts, hs, ms, analogRead(CURRENT_PIN));
    lastSend = now;
  }

  // Hourly OTA Task
  static unsigned long lastOTA = 0;
  if (now - lastOTA > 3600000) {
    checkOTA();
    lastOTA = now;
  }
}
