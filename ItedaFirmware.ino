#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>

// -------------------- CONFIGURATION --------------------
const char* VERSION = "v1.0.6"; 
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- PIN DEFINITIONS --------------------
#define DHTPIN1 7
#define DHTPIN2 8
#define DHTPIN3 11
#define DHTPIN4 12
#define DHTTYPE DHT22

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

// -------------------- GLOBALS --------------------
DHT dhts[] = {
  {DHTPIN1, DHTTYPE}, {DHTPIN2, DHTTYPE},
  {DHTPIN3, DHTTYPE}, {DHTPIN4, DHTTYPE}
};

double Setpoint = 47.5, Input, Output;
double Kp=2, Ki=5, Kd=1; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000; 
unsigned long windowStartTime;
bool heaterActive = false;

// -------------------- UTILITIES --------------------
String getTimestamp() {
  time_t now; time(&now);
  struct tm *ti = gmtime(&now);
  if (ti->tm_year < 100) return "NTP_NOT_READY";
  char buf[30]; strftime(buf, 30, "%Y-%m-%dT%H:%M:%SZ", ti);
  return String(buf);
}

// -------------------- OTA --------------------
void checkOTA() {
  Serial.println("[OTA] Checking for updates...");
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;

  if (http.begin(client, MANIFEST_URL)) {
    int code = http.GET();
    Serial.print("[OTA] HTTP Response: ");
    Serial.println(code);

    if (code == HTTP_CODE_OK) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, http.getString());

      const char* newVersion = doc["version"];
      Serial.print("[OTA] Current: ");
      Serial.print(VERSION);
      Serial.print(" | Available: ");
      Serial.println(newVersion);

      if (strcmp(newVersion, VERSION) != 0) {
        Serial.println("[OTA] New version found! Updating...");
        digitalWrite(LED_YELLOW, HIGH);
        httpUpdate.update(client, (const char*)doc["bin_url"]);
      } else {
        Serial.println("[OTA] Already up to date.");
      }
    }
    http.end();
  } else {
    Serial.println("[OTA] Failed to connect to manifest.");
  }
}

// -------------------- API --------------------
void sendPayload(float t[], float h[], int m[], int current) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[API] WiFi not connected, skipping send.");
    return;
  }

  Serial.println("[API] Sending payload...");

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  StaticJsonDocument<1024> doc;
  
  doc["dryer_id"] = "ITEDA_DRYER_01";
  doc["version"] = VERSION;
  doc["timestamp"] = getTimestamp();
  doc["temp_chamber"] = t[0];
  doc["hum_chamber"] = h[0];
  doc["moisture_1"] = m[0];
  doc["heater_active"] = heaterActive;
  doc["pid_duty"] = (Output / WindowSize) * 100.0;
  doc["current_raw"] = current;

  String json;
  serializeJson(doc, json);

  Serial.print("[API] Payload: ");
  Serial.println(json);

  if (https.begin(client, API_URL)) {
    https.addHeader("Content-Type", "application/json");
    https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));
    int response = https.POST(json);

    Serial.print("[API] Response code: ");
    Serial.println(response);

    https.end();
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n===== SYSTEM BOOT =====");
  Serial.print("Firmware Version: ");
  Serial.println(VERSION);

  pinMode(HEATER_1, OUTPUT); 
  pinMode(HEATER_2, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT); 
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT); 
  pinMode(LED_GREEN, OUTPUT);

  for (int i = 0; i < 4; i++) dhts[i].begin();
  analogReadResolution(12);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW));
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, HIGH);
  
  configTime(0, 0, "pool.ntp.org");
  Serial.println("NTP configured");

  windowStartTime = millis();
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  checkOTA(); 
}

// -------------------- LOOP --------------------
void loop() {
  float ts[4], hs[4];

  for (int i = 0; i < 4; i++) {
    ts[i] = dhts[i].readTemperature();
    hs[i] = dhts[i].readHumidity();

    Serial.print("T"); Serial.print(i); Serial.print(": ");
    Serial.print(ts[i]); Serial.print("C | H");
    Serial.print(i); Serial.print(": ");
    Serial.print(hs[i]); Serial.print("%  ");
  }
  Serial.println();

  int ms[4] = {
    analogRead(MOISTURE1), analogRead(MOISTURE2),
    analogRead(MOISTURE3), analogRead(MOISTURE4)
  };

  Serial.print("Moisture: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(ms[i]); Serial.print(" ");
  }
  Serial.println();

  if (!isnan(ts[0])) {
    Input = (double)ts[0];
    myPID.Compute();

    Serial.print("PID -> Input: ");
    Serial.print(Input);
    Serial.print(" | Output: ");
    Serial.print(Output);
    Serial.print(" | Duty: ");
    Serial.print((Output / WindowSize) * 100.0);
    Serial.println("%");
  }

  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) windowStartTime += WindowSize;

  heaterActive = (Output > (now - windowStartTime));

  Serial.print("Heater: ");
  Serial.println(heaterActive ? "ON" : "OFF");

  digitalWrite(HEATER_1, heaterActive);
  digitalWrite(HEATER_2, heaterActive);
  digitalWrite(LED_RED, heaterActive);
  digitalWrite(FAN_RELAY, HIGH); 

  static unsigned long lastSend = 0;
  if (now - lastSend > 10000) {
    sendPayload(ts, hs, ms, analogRead(CURRENT_PIN));
    lastSend = now;
  }

  static unsigned long lastOTA = 0;
  if (now - lastOTA > 3600000) {
    checkOTA();
    lastOTA = now;
  }

  delay(2000); // makes logs readable
}
