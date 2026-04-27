#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PID_v1.h>
#include <time.h>

// -------------------- CONFIGURATION --------------------
// The CI/CD script now pulls this string to generate the manifest.json
const char* VERSION = "v1.0.5"; 
const char* ssid = "dono-call";
const char* password = "@ubiquitoU5";
const char* API_URL = "https://iteda-solutions-dryers-platform.vercel.app/api/sensor-data";
const char* MANIFEST_URL = "https://iteda-solutions.github.io/ItedaFirmware/manifest.json";
const char* AUTH_TOKEN = "YOUR_TOKEN";

// -------------------- PIN DEFINITIONS (Per Schematic V1.0) --------------------
// DHT Sensors [cite: 40, 61, 76, 77]
#define DHTPIN1 7    // DHTS1 (Main Control)
#define DHTPIN2 8    // DHTS2
#define DHTPIN3 11   // DHTS3
#define DHTPIN4 12   // DHTS4
#define DHTTYPE DHT22

// Moisture Sensors (Analog MOS1-MOS4) [cite: 20, 28, 21, 22]
#define MOISTURE1 1  // MOS1
#define MOISTURE2 2  // MOS2
#define MOISTURE3 4  // MOS3
#define MOISTURE4 5  // MOS4

// Relays & Actuators [cite: 51, 56, 89]
#define HEATER_1   21   // RHEAT
#define HEATER_2   10   // RHEAT2
#define FAN_RELAY  18   // RFAN

// Status LEDs [cite: 44, 48, 82]
#define LED_RED    15   // LED1 (Heat)
#define LED_YELLOW 16   // LED2 (OTA Status)
#define LED_GREEN  13   // LED3 (System OK)

#define CURRENT_PIN 6   // CSENSOR [cite: 36]

// -------------------- GLOBALS --------------------
DHT dhts[] = {{DHTPIN1, DHTTYPE}, {DHTPIN2, DHTTYPE}, {DHTPIN3, DHTTYPE}, {DHTPIN4, DHTTYPE}};

double Setpoint = 47.5; // Target temperature midpoint
double Input, Output;
double Kp=2.0, Ki=5.0, Kd=1.0; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000; // 5-second relay window
unsigned long windowStartTime;
bool heaterActive = false;

// -------------------- OTA & TIME --------------------
String getTimestamp() {
  time_t now; time(&now);
  struct tm *ti = gmtime(&now);
  if (ti->tm_year < 100) return "NTP_NOT_READY";
  char buf[30]; strftime(buf, 30, "%Y-%m-%dT%H:%M:%SZ", ti);
  return String(buf);
}

void checkOTA() {
  Serial.println("Checking OTA...");
  WiFiClientSecure client; 
  client.setInsecure();
  HTTPClient http;
  if (http.begin(client, MANIFEST_URL)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, http.getString());
      if (strcmp(doc["version"], VERSION) != 0) {
        Serial.println("New version detected. Updating...");
        digitalWrite(LED_YELLOW, HIGH);
        httpUpdate.update(client, (const char*)doc["bin_url"]);
      }
    }
    http.end();
  }
}

// -------------------- DATA TRANSMISSION --------------------
void sendPayload(float t[], float h[], int m[], int current) {
  if (WiFi.status() != WL_CONNECTED) return;
  
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;

  StaticJsonDocument<1024> doc;
  doc["dryer_id"] = "ITEDA_DRYER_01";
  doc["version"] = VERSION;
  doc["timestamp"] = getTimestamp();

  // Primary control sensor data
  doc["temp_chamber"] = t[0]; 
  doc["hum_chamber"] = h[0];

  // Secondary sensors
  doc["temp_int_2"] = t[1];
  doc["temp_int_3"] = t[2];
  doc["temp_ambient"] = t[3];
  doc["hum_ambient"] = h[3];

  // Moisture levels (0-4095)
  doc["moisture_1"] = m[0];
  doc["moisture_2"] = m[1];
  doc["moisture_3"] = m[2];
  doc["moisture_4"] = m[3];

  // System status
  doc["heater_active"] = heaterActive;
  doc["fan_active"] = true; 
  doc["pid_duty_cycle"] = (Output / WindowSize) * 100.0;
  doc["current_adc"] = current;

  String json;
  serializeJson(doc, json);

  if (https.begin(client, API_URL)) {
    https.addHeader("Content-Type", "application/json");
    https.addHeader("Authorization", "Bearer " + String(AUTH_TOKEN));
    https.POST(json);
    https.end();
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  pinMode(HEATER_1, OUTPUT); pinMode(HEATER_2, OUTPUT);
  pinMode(FAN_RELAY, OUTPUT); pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT); pinMode(LED_GREEN, OUTPUT);

  for (int i = 0; i < 4; i++) dhts[i].begin();
  analogReadResolution(12);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW)); // Blink yellow during boot
  }
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, HIGH); // Green constant = System Online
  
  configTime(0, 0, "pool.ntp.org");
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
  }
  int ms[4] = {analogRead(MOISTURE1), analogRead(MOISTURE2), analogRead(MOISTURE3), analogRead(MOISTURE4)};
  
  // PID Control via DHTS1 (IO7)
  if (!isnan(ts[0])) {
    Input = (double)ts[0];
    myPID.Compute();
  }

  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) windowStartTime += WindowSize;
  
  // Time-proportioning relay logic
  heaterActive = (Output > (now - windowStartTime));

  digitalWrite(HEATER_1, heaterActive);
  digitalWrite(HEATER_2, heaterActive);
  digitalWrite(LED_RED, heaterActive); // Red LED mimics heater activity
  digitalWrite(FAN_RELAY, HIGH); // Fan always runs during drying

  // Periodic Reporting (10s)
  static unsigned long lastSend = 0;
  if (now - lastSend > 10000) {
    sendPayload(ts, hs, ms, analogRead(CURRENT_PIN));
    lastSend = now;
  }

  // Hourly OTA Check
  static unsigned long lastOTA = 0;
  if (now - lastOTA > 3600000) {
    checkOTA();
    lastOTA = now;
  }
}
