#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// -------------------- SERVER --------------------
WebServer server(80);

// -------------------- WIFI --------------------
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";

// -------------------- API --------------------
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- FIRMWARE VERSION --------------------
#define FIRMWARE_VERSION "v1.0.0"

// -------------------- PINS --------------------
#define DHTPIN1 4
#define DHTPIN2 5
#define DHTTYPE DHT22

#define HEATER_RELAY 15
#define FAN_RELAY 16

#define CURRENT_PIN 33
#define DOOR_PIN 17

// -------------------- SENSORS --------------------
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);

// -------------------- CONTROL --------------------
bool autoMode = true;
int heaterState = 0;
int fanState = 1;

float setpoint = 45.0;
float band = 5.0;

// -------------------- POWER --------------------
float voltage = 12.0;
float current = 0;
float power = 0;

// -------------------- TIMERS --------------------
unsigned long lastSensorRead = 0;
unsigned long lastUpload = 0;
unsigned long lastWiFiCheck = 0;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long UPLOAD_INTERVAL = 5000;
const unsigned long WIFI_INTERVAL = 10000;

// ==========================================================
// Check for OTA
// ==========================================================
bool checkForOTA() {
  WiFiClientSecure client;
  client.setInsecure();
  
  HTTPClient http;
  http.begin(client, "https://<username>.github.io/<repo>/ota/manifest.json"); // Your GitHub Pages URL
  int code = http.GET();
  
  if (code != 200) {
    Serial.println("[OTA] Failed to fetch manifest");
    http.end();
    return false;
  }
  
  String payload = http.getString();
  Serial.println("[OTA] Manifest received: " + payload);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("[OTA] Failed to parse manifest");
    http.end();
    return false;
  }

  const char* latest_version = doc["version"];
  const char* bin_url = doc["bin_url"];

  if (String(latest_version) != FIRMWARE_VERSION) {
    Serial.printf("[OTA] New firmware found: %s -> %s\n", FIRMWARE_VERSION, latest_version);
    // Trigger OTA download here using ESP32 HTTP Update library
    // ESPhttpUpdate.update(bin_url);
  } else {
    Serial.println("[OTA] Firmware up-to-date");
  }

  http.end();
  return true;
}

// ==========================================================
// STRUCT
// ==========================================================
struct SensorData {
  String timestamp;
  float chamber_temp;
  float ambient_temp;
  float heater_temp;
  float internal_humidity;
  float external_humidity;

  int fan_speed_rpm;
  bool fan_status;
  bool heater_status;
  bool door_status;

  float solar_voltage;
  int battery_level;
  float battery_voltage;

  float power_consumption_w;
  String charging_status;
};

// ==========================================================
// TIME
// ==========================================================
String getTimestamp() {
  time_t now;
  time(&now);

  // If time is not synced, return a safe fallback
  if (now < 1600000000) { // roughly 2020
    Serial.println("[TIME] NTP not synced, using fallback timestamp");
    return "2026-01-01T00:00:00Z";
  }

  struct tm *timeinfo = gmtime(&now);
  char buffer[30];
  strftime(buffer, 30, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
  return String(buffer);
}

// ==========================================================
// STORAGE
// ==========================================================
void saveToQueue(String json) {
  Serial.println("[QUEUE] Saving data locally");
  File file = SPIFFS.open("/queue.txt", FILE_APPEND);
  if (!file) {
    Serial.println("[ERROR] Failed to open queue file");
    return;
  }
  file.println(json);
  file.close();
}

bool sendToServer(String payload) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[API] WiFi not connected, skipping send");
    return false;
  }

  Serial.println("[API] Sending payload:");
  Serial.println(payload);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;

  if (!https.begin(client, API_URL)) {
    Serial.println("[ERROR] HTTPS begin failed");
    return false;
  }

  https.addHeader("Content-Type", "application/json");
  https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));

  int code = https.POST(payload);

  Serial.print("[API] Response code: ");
  Serial.println(code);

  String response = https.getString();
  Serial.println("[API] Response:");
  Serial.println(response);

  // Remote control parsing
  if (code > 0) {
    if (response.indexOf("\"mode\":\"manual\"") != -1) {
      autoMode = false;
      Serial.println("[CONTROL] Switched to MANUAL mode");
    }
    if (response.indexOf("\"mode\":\"auto\"") != -1) {
      autoMode = true;
      Serial.println("[CONTROL] Switched to AUTO mode");
    }

    if (!autoMode) {
      heaterState = (response.indexOf("\"heater\":1") != -1);
      fanState = (response.indexOf("\"fan\":1") != -1);
      Serial.printf("[CONTROL] Remote -> Heater:%d Fan:%d\n", heaterState, fanState);
    }
  }

  https.end();

  return (code > 0 && code < 300);
}

void flushQueue() {
  Serial.println("[QUEUE] Attempting to flush stored data");

  File file = SPIFFS.open("/queue.txt", FILE_READ);
  if (!file) {
    Serial.println("[QUEUE] No stored data");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (!sendToServer(line)) {
      Serial.println("[QUEUE] Failed to send queued data");
      file.close();
      return;
    }
  }

  file.close();
  SPIFFS.remove("/queue.txt");
  Serial.println("[QUEUE] Successfully flushed queue");
}

// ==========================================================
// CONTROL LOGIC
// ==========================================================
void runControlLogic(float chamberTemp) {
  if (autoMode) {
    if (chamberTemp < setpoint - band) heaterState = 1;
    else if (chamberTemp > setpoint + band) heaterState = 0;
    fanState = 1;
  }

  digitalWrite(HEATER_RELAY, heaterState ? HIGH : LOW);
  digitalWrite(FAN_RELAY, fanState ? HIGH : LOW);

  Serial.printf("[CONTROL] Heater:%d Fan:%d Mode:%s\n",
                heaterState, fanState, autoMode ? "AUTO" : "MANUAL");
}

// ==========================================================
// WIFI
// ==========================================================
void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }
}

// ==========================================================
// BUILD PAYLOAD
// ==========================================================
String buildPayload(SensorData d) {
  String json = "{";

  json += "\"dryer_id\":\"REAL_DRYER\",";
  json += "\"timestamp\":\"" + d.timestamp + "\",";

  // Use safe defaults if sensor failed
  json += "\"chamber_temp\":" + String(d.chamber_temp) + ",";
  json += "\"ambient_temp\":" + String(d.ambient_temp) + ",";
  json += "\"heater_temp\":" + String(d.heater_temp) + ",";
  json += "\"internal_humidity\":" + String(d.internal_humidity) + ",";
  json += "\"external_humidity\":" + String(d.external_humidity) + ",";

  json += "\"fan_speed_rpm\":0,";
  json += "\"fan_status\":" + String(d.fan_status ? "true" : "false") + ",";
  json += "\"heater_status\":" + String(d.heater_status ? "true" : "false") + ",";
  json += "\"door_status\":" + String(d.door_status ? "true" : "false") + ",";

  // Safe fallback values
  json += "\"solar_voltage\":" + String(d.solar_voltage >= 0 ? d.solar_voltage : 12.0) + ",";
  json += "\"battery_level\":" + String(d.battery_level > 0 ? d.battery_level : 50) + ",";
  json += "\"battery_voltage\":" + String(d.battery_voltage >= 8 ? d.battery_voltage : 12.0) + ",";

  json += "\"power_consumption_w\":" + String(d.power_consumption_w) + ",";
  json += "\"charging_status\":\"" + d.charging_status + "\"";

  json += "}";
  return json;
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== SYSTEM START ===");

  pinMode(HEATER_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);

  dht1.begin();
  dht2.begin();

  if (!SPIFFS.begin(true)) {
    Serial.println("[ERROR] SPIFFS failed");
  } else {
    Serial.println("[OK] SPIFFS mounted");
  }

  Serial.print("[WIFI] Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[WIFI] Connected!");
  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("[TIME] Syncing time...");
  configTime(0, 0, "pool.ntp.org");
  delay(2000);

  ArduinoOTA.onStart([]() { Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] End"); });
  ArduinoOTA.begin();

  // ----- WebServer for logging -----
  server.on("/logs", [&server]() {
      String html = "<html><body><pre>";
      html += "Heater state: " + String(heaterState) + "\n";
      html += "Fan state: " + String(fanState) + "\n";
      html += "Auto mode: " + String(autoMode ? "AUTO" : "MANUAL") + "\n";
      html += "Chamber Temp: " + String(dht1.readTemperature()) + "\n";
      html += "Ambient Temp: " + String(dht2.readTemperature()) + "\n";
      html += "Chamber Humidity: " + String(dht1.readHumidity()) + "\n";
      html += "Ambient Humidity: " + String(dht2.readHumidity()) + "\n";
      html += "Current (A): " + String(current) + "\n";
      html += "Power (W): " + String(power) + "\n";
      html += "</pre></body></html>";
      server.send(200, "text/html", html);
  });

  server.begin();
  Serial.println("[SERVER] Web server started");

  esp_task_wdt_add(NULL);

  Serial.println("=== SETUP COMPLETE ===");
}

// ==========================================================
// LOOP
// ==========================================================
void loop() {
  ArduinoOTA.handle();
  esp_task_wdt_reset();

  server.handleClient(); // <--- Add this for network logging

  unsigned long now = millis();

  // WIFI CHECK
  if (now - lastWiFiCheck > WIFI_INTERVAL) {
    lastWiFiCheck = now;
    ensureWiFi();
  }

  // SENSOR READ
  if (now - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = now;

    Serial.println("\n[LOOP] Reading sensors...");

    float t1 = dht1.readTemperature();
    float h1 = dht1.readHumidity();
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();

    // Handle failed reads gracefully
    if (isnan(t1)) t1 = -1.0;
    if (isnan(h1)) h1 = -1.0;
    if (isnan(t2)) t2 = -1.0;
    if (isnan(h2)) h2 = -1.0;

    Serial.printf("[SENSORS] T1: %.2f H1: %.2f | T2: %.2f H2: %.2f\n", t1, h1, t2, h2);

    int adc = analogRead(CURRENT_PIN);
    float vout = adc * (3.3 / 4095.0);
    current = vout;
    power = voltage * current;

    Serial.printf("[POWER] Current: %.3f A | Power: %.2f W\n", current, power);

    SensorData d;
    d.timestamp = getTimestamp();
    d.chamber_temp = t1;
    d.ambient_temp = t2;
    d.heater_temp = t1;
    d.internal_humidity = h1;
    d.external_humidity = h2;

    d.fan_status = fanState;
    d.heater_status = heaterState;
    d.door_status = digitalRead(DOOR_PIN) == LOW;

    d.power_consumption_w = power;
    d.charging_status = "unknown";

    // Defaults in case data missing
    d.solar_voltage = 12.0;
    d.battery_level = 50;
    d.battery_voltage = 12;

    runControlLogic(t1);

    String payload = buildPayload(d);

    if (!sendToServer(payload)) {
      saveToQueue(payload);
    }
  }

  // RETRY QUEUE
  if (now - lastUpload > UPLOAD_INTERVAL) {
    lastUpload = now;
    flushQueue();
  }
}