#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <ArduinoJson.h>
#include <HTTPUpdate.h>

// -------------------- SERVER --------------------
WebServer server(80);

// -------------------- WIFI --------------------
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";

// -------------------- API --------------------
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- FIRMWARE VERSION --------------------
#define FIRMWARE_VERSION "v1.0.11"

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
unsigned long lastOTACheck = 0;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long UPLOAD_INTERVAL = 5000;
const unsigned long WIFI_INTERVAL = 10000;
const unsigned long OTA_INTERVAL = 60000;

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

  bool fan_status;
  bool heater_status;
  bool door_status;

  float solar_voltage;
  int battery_level;
  float battery_voltage;

  float power_consumption_w;
  String charging_status;
};
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <ArduinoJson.h>
#include <HTTPUpdate.h>

// -------------------- SERVER --------------------
WebServer server(80);

// -------------------- WIFI --------------------
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";

// -------------------- API --------------------
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- FIRMWARE VERSION --------------------
#define FIRMWARE_VERSION "v1.0.11"

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
unsigned long lastOTACheck = 0;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long UPLOAD_INTERVAL = 5000;
const unsigned long WIFI_INTERVAL = 10000;
const unsigned long OTA_INTERVAL = 60000;

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
// OTA
// ==========================================================
bool checkForOTA() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, "https://bujo-eayn.github.io/ItedaFirmware/manifest.json");

  int code = http.GET();

  if (code != 200) {
    Serial.println("[OTA] Failed to fetch manifest");
    http.end();
    return false;
  }

  String payload = http.getString();

  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, payload)) {
    Serial.println("[OTA] JSON parse error");
    http.end();
    return false;
  }

  const char* latest_version = doc["version"];
  const char* bin_url = doc["bin_url"];

  if (String(latest_version) != FIRMWARE_VERSION) {
    Serial.println("[OTA] Updating firmware...");

    t_httpUpdate_return ret = httpUpdate.update(client, bin_url);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("[OTA] Failed (%d): %s\n",
          httpUpdate.getLastError(),
          httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[OTA] No updates");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("[OTA] Update OK");
        break;
    }
  } else {
    Serial.println("[OTA] Up-to-date");
  }

  http.end();
  return true;
}

// ==========================================================
// TIME
// ==========================================================
String getTimestamp() {
  time_t now;
  time(&now);

  if (now < 1600000000) {
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
  File file = SPIFFS.open("/queue.txt", FILE_APPEND);
  if (!file) return;
  file.println(json);
  file.close();
}

bool sendToServer(String payload) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (!https.begin(client, API_URL)) return false;

  https.addHeader("Content-Type", "application/json");
  https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));

  int code = https.POST(payload);
  String response = https.getString();

  if (code > 0) {
    if (response.indexOf("\"mode\":\"manual\"") != -1) autoMode = false;
    if (response.indexOf("\"mode\":\"auto\"") != -1) autoMode = true;

    if (!autoMode) {
      heaterState = (response.indexOf("\"heater\":1") != -1);
      fanState = (response.indexOf("\"fan\":1") != -1);
    }
  }

  https.end();
  return (code > 0 && code < 300);
}

void flushQueue() {
  File file = SPIFFS.open("/queue.txt", FILE_READ);
  if (!file) return;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (!sendToServer(line)) {
      file.close();
      return;
    }
  }

  file.close();
  SPIFFS.remove("/queue.txt");
}

// ==========================================================
// CONTROL
// ==========================================================
void runControlLogic(float chamberTemp) {
  if (autoMode) {
    if (chamberTemp < setpoint - band) heaterState = 1;
    else if (chamberTemp > setpoint + band) heaterState = 0;
    fanState = 1;
  }

  digitalWrite(HEATER_RELAY, heaterState);
  digitalWrite(FAN_RELAY, fanState);
}

// ==========================================================
// WIFI
// ==========================================================
void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }
}

// ==========================================================
// PAYLOAD
// ==========================================================
String buildPayload(SensorData d) {
  String json = "{";

  json += "\"dryer_id\":\"REAL_DRYER\",";
  json += "\"timestamp\":\"" + d.timestamp + "\",";

  json += "\"chamber_temp\":" + String(d.chamber_temp) + ",";
  json += "\"ambient_temp\":" + String(d.ambient_temp) + ",";
  json += "\"heater_temp\":" + String(d.heater_temp) + ",";
  json += "\"internal_humidity\":" + String(d.internal_humidity) + ",";
  json += "\"external_humidity\":" + String(d.external_humidity) + ",";

  json += "\"fan_status\":" + String(d.fan_status ? "true" : "false") + ",";
  json += "\"heater_status\":" + String(d.heater_status ? "true" : "false") + ",";
  json += "\"door_status\":" + String(d.door_status ? "true" : "false") + ",";

  json += "\"solar_voltage\":" + String(d.solar_voltage) + ",";
  json += "\"battery_level\":" + String(d.battery_level) + ",";
  json += "\"battery_voltage\":" + String(d.battery_voltage) + ",";

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

  pinMode(HEATER_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);

  dht1.begin();
  dht2.begin();

  SPIFFS.begin(true);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTime(0, 0, "pool.ntp.org");

  ArduinoOTA.begin();

  server.on("/logs", []() {
    server.send(200, "text/plain", "System OK");
  });
  server.begin();

  esp_task_wdt_add(NULL);
}

// ==========================================================
// LOOP
// ==========================================================
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  esp_task_wdt_reset();

  unsigned long now = millis();

  if (now - lastWiFiCheck > WIFI_INTERVAL) {
    lastWiFiCheck = now;
    ensureWiFi();
  }

  if (now - lastOTACheck > OTA_INTERVAL) {
    lastOTACheck = now;
    checkForOTA();
  }

  if (now - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = now;

    float t1 = dht1.readTemperature();
    float h1 = dht1.readHumidity();
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();

    if (isnan(t1)) t1 = -1;
    if (isnan(h1)) h1 = -1;
    if (isnan(t2)) t2 = -1;
    if (isnan(h2)) h2 = -1;

    int adc = analogRead(CURRENT_PIN);
    current = adc * (3.3 / 4095.0);
    power = voltage * current;

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

    d.solar_voltage = 12.0;
    d.battery_level = 50;
    d.battery_voltage = 12;

    runControlLogic(t1);

    String payload = buildPayload(d);

    if (!sendToServer(payload)) {
      saveToQueue(payload);
    }
  }

  if (now - lastUpload > UPLOAD_INTERVAL) {
    lastUpload = now;
    flushQueue();
  }
}
// ==========================================================
// OTA
// ==========================================================
bool checkForOTA() {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, "https://bujo-eayn.github.io/ItedaFirmware/manifest.json");

  int code = http.GET();

  if (code != 200) {
    Serial.println("[OTA] Failed to fetch manifest");
    http.end();
    return false;
  }

  String payload = http.getString();

  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, payload)) {
    Serial.println("[OTA] JSON parse error");
    http.end();
    return false;
  }

  const char* latest_version = doc["version"];
  const char* bin_url = doc["bin_url"];

  if (String(latest_version) != FIRMWARE_VERSION) {
    Serial.println("[OTA] Updating firmware...");

    t_httpUpdate_return ret = httpUpdate.update(client, bin_url);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("[OTA] Failed (%d): %s\n",
          httpUpdate.getLastError(),
          httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[OTA] No updates");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("[OTA] Update OK");
        break;
    }
  } else {
    Serial.println("[OTA] Up-to-date");
  }

  http.end();
  return true;
}

// ==========================================================
// TIME
// ==========================================================
String getTimestamp() {
  time_t now;
  time(&now);

  if (now < 1600000000) {
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
  File file = SPIFFS.open("/queue.txt", FILE_APPEND);
  if (!file) return;
  file.println(json);
  file.close();
}

bool sendToServer(String payload) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  if (!https.begin(client, API_URL)) return false;

  https.addHeader("Content-Type", "application/json");
  https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));

  int code = https.POST(payload);
  String response = https.getString();

  if (code > 0) {
    if (response.indexOf("\"mode\":\"manual\"") != -1) autoMode = false;
    if (response.indexOf("\"mode\":\"auto\"") != -1) autoMode = true;

    if (!autoMode) {
      heaterState = (response.indexOf("\"heater\":1") != -1);
      fanState = (response.indexOf("\"fan\":1") != -1);
    }
  }

  https.end();
  return (code > 0 && code < 300);
}

void flushQueue() {
  File file = SPIFFS.open("/queue.txt", FILE_READ);
  if (!file) return;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (!sendToServer(line)) {
      file.close();
      return;
    }
  }

  file.close();
  SPIFFS.remove("/queue.txt");
}

// ==========================================================
// CONTROL
// ==========================================================
void runControlLogic(float chamberTemp) {
  if (autoMode) {
    if (chamberTemp < setpoint - band) heaterState = 1;
    else if (chamberTemp > setpoint + band) heaterState = 0;
    fanState = 1;
  }

  digitalWrite(HEATER_RELAY, heaterState);
  digitalWrite(FAN_RELAY, fanState);
}

// ==========================================================
// WIFI
// ==========================================================
void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
  }
}

// ==========================================================
// PAYLOAD
// ==========================================================
String buildPayload(SensorData d) {
  String json = "{";

  json += "\"dryer_id\":\"REAL_DRYER\",";
  json += "\"timestamp\":\"" + d.timestamp + "\",";

  json += "\"chamber_temp\":" + String(d.chamber_temp) + ",";
  json += "\"ambient_temp\":" + String(d.ambient_temp) + ",";
  json += "\"heater_temp\":" + String(d.heater_temp) + ",";
  json += "\"internal_humidity\":" + String(d.internal_humidity) + ",";
  json += "\"external_humidity\":" + String(d.external_humidity) + ",";

  json += "\"fan_status\":" + String(d.fan_status ? "true" : "false") + ",";
  json += "\"heater_status\":" + String(d.heater_status ? "true" : "false") + ",";
  json += "\"door_status\":" + String(d.door_status ? "true" : "false") + ",";

  json += "\"solar_voltage\":" + String(d.solar_voltage) + ",";
  json += "\"battery_level\":" + String(d.battery_level) + ",";
  json += "\"battery_voltage\":" + String(d.battery_voltage) + ",";

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

  pinMode(HEATER_RELAY, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);
  pinMode(DOOR_PIN, INPUT_PULLUP);

  dht1.begin();
  dht2.begin();

  SPIFFS.begin(true);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTime(0, 0, "pool.ntp.org");

  ArduinoOTA.begin();

  server.on("/logs", []() {
    server.send(200, "text/plain", "System OK");
  });
  server.begin();

  esp_task_wdt_add(NULL);
}

// ==========================================================
// LOOP
// ==========================================================
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  esp_task_wdt_reset();

  unsigned long now = millis();

  if (now - lastWiFiCheck > WIFI_INTERVAL) {
    lastWiFiCheck = now;
    ensureWiFi();
  }

  if (now - lastOTACheck > OTA_INTERVAL) {
    lastOTACheck = now;
    checkForOTA();
  }

  if (now - lastSensorRead > SENSOR_INTERVAL) {
    lastSensorRead = now;

    float t1 = dht1.readTemperature();
    float h1 = dht1.readHumidity();
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();

    if (isnan(t1)) t1 = -1;
    if (isnan(h1)) h1 = -1;
    if (isnan(t2)) t2 = -1;
    if (isnan(h2)) h2 = -1;

    int adc = analogRead(CURRENT_PIN);
    current = adc * (3.3 / 4095.0);
    power = voltage * current;

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

    d.solar_voltage = 12.0;
    d.battery_level = 50;
    d.battery_voltage = 12;

    runControlLogic(t1);

    String payload = buildPayload(d);

    if (!sendToServer(payload)) {
      saveToQueue(payload);
    }
  }

  if (now - lastUpload > UPLOAD_INTERVAL) {
    lastUpload = now;
    flushQueue();
  }
}
