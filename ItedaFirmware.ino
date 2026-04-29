#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>

// ---------------- CONFIG ----------------
const char* VERSION = "2.0";
const char* ssid = "masinde";
const char* password = "14414@Starehe";

const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// ---------------- PINS ----------------
#define DHTPIN1 7
#define DHTPIN2 8
#define DHTPIN3 9
#define DHTPIN4 10
#define DHTTYPE DHT11

#define HEATER_1 21
#define HEATER_2 17
#define FAN_RELAY 18

#define MOISTURE1 1
#define MOISTURE2 2
#define MOISTURE3 4
#define MOISTURE4 5

// ---------------- DHT ----------------
DHT dhts[] = {
  {DHTPIN1, DHTTYPE},
  {DHTPIN2, DHTTYPE},
  {DHTPIN3, DHTTYPE},
  {DHTPIN4, DHTTYPE}
};

// ---------------- PID ----------------
double Setpoint = 47.5, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;

// ---------------- SHARED DATA ----------------
struct SystemData {
  float temp[4];
  float hum[4];
  int moisture[4];
  int current;
  double pidInput;
  double pidOutput;
  bool heater;
};

SystemData data;

SemaphoreHandle_t dataMutex;

// ---------------- DEVICE ID ----------------
String getDeviceID() {
  uint64_t chipid = ESP.getEfuseMac();
  char id[25];
  snprintf(id, sizeof(id), "ITEDA-%04X%08X",
           (uint16_t)(chipid >> 32),
           (uint32_t)chipid);
  return String(id);
}

// ---------------- OTA ----------------
void checkOTA() {

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;

  if (http.begin(client, MANIFEST_URL)) {

    int code = http.GET();

    if (code == 200) {

      StaticJsonDocument<256> doc;
      deserializeJson(doc, http.getString());

      const char* newVersion = doc["version"];

      if (strcmp(newVersion, VERSION) != 0) {

        Serial.println("[OTA] Updating...");
        httpUpdate.update(client, doc["bin_url"]);

      } else {
        Serial.println("[OTA] Up to date");
      }
    }

    http.end();
  }
}

// ---------------- API ----------------
void sendPayload() {

  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;

  StaticJsonDocument<1024> doc;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  doc["dryer_id"] = getDeviceID();
  doc["heater_status"] = data.heater;
  doc["chamber_temp"] = data.temp[1];

  xSemaphoreGive(dataMutex);

  String json;
  serializeJson(doc, json);

  https.begin(client, API_URL);
  https.addHeader("Content-Type", "application/json");
  https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));

  https.POST(json);
  https.end();
}

// =========================================================
// SENSOR TASK
// =========================================================
void sensorTask(void *pv) {

  Serial.println("[TASK] Sensor task started");

  while (true) {

    float t[4], h[4];

    for (int i = 0; i < 4; i++) {
      t[i] = dhts[i].readTemperature();
      h[i] = dhts[i].readHumidity();
    }

    int m[4] = {
      analogRead(MOISTURE1),
      analogRead(MOISTURE2),
      analogRead(MOISTURE3),
      analogRead(MOISTURE4)
    };

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    for (int i = 0; i < 4; i++) {
      if (!isnan(t[i])) data.temp[i] = t[i];
      if (!isnan(h[i])) data.hum[i] = h[i];
      data.moisture[i] = m[i];
    }

    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// =========================================================
// CONTROL TASK
// =========================================================
void controlTask(void *pv) {

  Serial.println("[TASK] Control task started");

  unsigned long windowStart = millis();

  while (true) {

    float input = 0;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    input = data.temp[1];
    xSemaphoreGive(dataMutex);

    if (!isnan(input)) {
      Input = input;
      myPID.Compute();
    }

    unsigned long now = millis();

    if (now - windowStart > WindowSize)
      windowStart += WindowSize;

    bool heater = (Output > (now - windowStart));

    digitalWrite(HEATER_1, heater);
    digitalWrite(HEATER_2, heater);
    digitalWrite(FAN_RELAY, HIGH);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    data.heater = heater;
    data.pidInput = Input;
    data.pidOutput = Output;
    xSemaphoreGive(dataMutex);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// =========================================================
// NETWORK TASK
// =========================================================
void networkTask(void *pv) {

  Serial.println("[TASK] Network task started");

  while (true) {

    sendPayload();

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// =========================================================
// OTA TASK
// =========================================================
void otaTask(void *pv) {

  Serial.println("[TASK] OTA task started");

  while (true) {

    if (WiFi.status() == WL_CONNECTED) {
      checkOTA();
    }

    vTaskDelay(pdMS_TO_TICKS(3600000));
  }
}

// =========================================================
// SETUP
// =========================================================
void setup() {

  Serial.begin(115200);
  delay(1500);

  Serial.println("\n[BOOT] ITEDA SYSTEM STARTING...");

  dataMutex = xSemaphoreCreateMutex();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\n[WIFI] Connected");

  configTime(0, 0, "pool.ntp.org");

  for (int i = 0; i < 4; i++)
    dhts[i].begin();

  pinMode(HEATER_1, OUTPUT);
  pinMode(HEATER_2, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);

  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  // ---------------- TASKS ----------------
  xTaskCreate(sensorTask,  "sensor", 4096, NULL, 3, NULL);
  xTaskCreate(controlTask, "control", 4096, NULL, 4, NULL);
  xTaskCreate(networkTask, "network", 8192, NULL, 2, NULL);
  xTaskCreate(otaTask,     "ota",     6144, NULL, 1, NULL);
}

// =========================================================
// LOOP NOT USED
// =========================================================
void loop() {
  vTaskDelay(portMAX_DELAY);
}
