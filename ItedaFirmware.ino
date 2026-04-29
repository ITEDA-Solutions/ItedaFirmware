#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>

// -------------------- CONFIGURATION --------------------
const char* VERSION = "1.9";
const char* ssid = "masinde";
const char* password = "14414@Starehe";
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- PINS --------------------
#define DHTPIN1 7
#define DHTPIN2 8
#define DHTPIN3 9
#define DHTPIN4 10
#define DHTTYPE DHT11

#define MOISTURE1 1
#define MOISTURE2 2
#define MOISTURE3 4
#define MOISTURE4 5

#define HEATER_1 21
#define HEATER_2 17
#define FAN_RELAY 18

#define LED_RED 15
#define LED_YELLOW 16
#define LED_GREEN 13

#define CURRENT_PIN 6

// -------------------- DHT --------------------
DHT dhts[] = {
  {DHTPIN1, DHTTYPE},
  {DHTPIN2, DHTTYPE},
  {DHTPIN3, DHTTYPE},
  {DHTPIN4, DHTTYPE}
};

// -------------------- PID --------------------
double Setpoint = 47.5, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;

// -------------------- SHARED DATA --------------------
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

// -------------------- UTILS --------------------
String getDeviceID() {
  uint64_t chipid = ESP.getEfuseMac();
  char id[25];
  snprintf(id, sizeof(id), "ITEDA-%04X%08X",
           (uint16_t)(chipid >> 32),
           (uint32_t)chipid);
  return String(id);
}

String getTimestamp() {
  time_t now;
  time(&now);
  struct tm *ti = gmtime(&now);

  if (ti->tm_year < 100) return "NTP_NOT_READY";

  char buf[30];
  strftime(buf, 30, "%Y-%m-%dT%H:%M:%SZ", ti);
  return String(buf);
}

// -------------------- OTA --------------------
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

// -------------------- API SEND --------------------
void sendPayloadFromSharedData() {

  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;

  StaticJsonDocument<2048> doc;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  doc["dryer_id"] = getDeviceID();
  doc["timestamp"] = getTimestamp();

  doc["chamber_temp"] = data.temp[1];
  doc["ambient_temp"] = data.temp[3];
  doc["heater_temp"] = data.temp[2];

  doc["internal_humidity"] = data.hum[1];
  doc["external_humidity"] = data.hum[3];

  doc["heater_status"] = data.heater;
  doc["power_consumption_w"] = data.current;

  JsonObject sensorValues = doc.createNestedObject("sensor_values");

  for (int i = 0; i < 4; i++) {
    sensorValues["temp_" + String(i)] = data.temp[i];
    sensorValues["hum_" + String(i)] = data.hum[i];
    sensorValues["moisture_" + String(i)] = data.moisture[i];
  }

  sensorValues["pid_input"] = data.pidInput;
  sensorValues["pid_output"] = data.pidOutput;

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
// TASK 1: SENSOR TASK
// =========================================================
void sensorTask(void *pv) {

  while (true) {

    float t[4], h[4];

    for (int i = 0; i < 4; i++) {
      t[i] = dhts[i].readTemperature();
      h[i] = dhts[i].readHumidity();
    }

    int m[4] = {
      analogRead(MOISTURE1),
      analogRead(MOISTURE2),
      MOISTURE3,
      MOISTURE4
    };

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    for (int i = 0; i < 4; i++) {
      data.temp[i] = t[i];
      data.hum[i] = h[i];
      data.moisture[i] = m[i];
    }

    xSemaphoreGive(dataMutex);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// =========================================================
// TASK 2: CONTROL TASK (PID)
// =========================================================
void controlTask(void *pv) {

  unsigned long windowStart = millis();

  while (true) {

    float input;

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

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// =========================================================
// TASK 3: NETWORK TASK
// =========================================================
void networkTask(void *pv) {

  while (true) {

    sendPayloadFromSharedData();

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

// =========================================================
// TASK 4: OTA TASK
// =========================================================
void otaTask(void *pv) {

  while (true) {

    if (WiFi.status() == WL_CONNECTED) {
      checkOTA();
    }

    vTaskDelay(3600000 / portTICK_PERIOD_MS);
  }
}

// =========================================================
// SETUP
// =========================================================
void setup() {

  Serial.begin(115200);
  delay(1000);

  dataMutex = xSemaphoreCreateMutex();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  configTime(0, 0, "pool.ntp.org");

  for (int i = 0; i < 4; i++)
    dhts[i].begin();

  pinMode(HEATER_1, OUTPUT);
  pinMode(HEATER_2, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT);

  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);

  // ---------------- TASKS ----------------
  xTaskCreatePinnedToCore(sensorTask,  "sensor", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(networkTask, "network", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(otaTask,     "ota",     6144, NULL, 1, NULL, 0);
}

// =========================================================
// LOOP (NOT USED)
// =========================================================
void loop() {
  vTaskDelay(portMAX_DELAY);
}
