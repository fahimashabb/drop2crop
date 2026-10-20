/*
 * Drop2Crop - Global Cloud IoT Precision Irrigation & Fertigation Controller
 * Hardware: ESP32-S3 Dev Board
 * 
 * Features:
 *   - Auto Captive Portal WiFi Provisioning ("Drop2Crop_WiFi_Setup" on 192.168.4.1)
 *   - Hold BOOT button (GPIO 0) for 3s to reconfigure WiFi at any new location
 *   - Persistent NVS Flash memory for WiFi & Multi-Timer Schedules
 *   - NTP Live Time Synchronization (UTC+6 Bangladesh Standard Time)
 *   - Multi-Schedule Timer Engine (up to 10 daily schedules with durations)
 *   - Capacitive Soil Moisture Sensor (GPIO 4)
 *   - Raindrop Sensor with 24-Hour Lockout Cycle (GPIO 5 DO, GPIO 6 AO)
 *   - HC-SR04 Ultrasonic Reservoir Level Monitor (GPIO 7 Trig, GPIO 8 Echo)
 *   - 5V Dual Relay Pump & Aux Valve Actuator (GPIO 9, GPIO 10 - Active LOW)
 *   - Mosquitto MQTT Cloud Bridge (test.mosquitto.org)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>

// ==================== PIN DEFINITIONS ====================
#define PIN_SOIL_ANALOG      4   // Capacitive Soil Moisture (ADC1_CH3)
#define PIN_RAIN_DIGITAL     5   // Rain Sensor Digital (DO)
#define PIN_RAIN_ANALOG      6   // Rain Sensor Analog (AO)
#define PIN_TRIG             7   // HC-SR04 Trigger
#define PIN_ECHO             8   // HC-SR04 Echo
#define PIN_RELAY_PUMP       9   // Relay 1: Water Pump (Active LOW)
#define PIN_RELAY_VALVE      10  // Relay 2: Aux/Fertigation (Active LOW)
#define PIN_BOOT_BUTTON      0   // Onboard BOOT Button for WiFi Reset

// ==================== CALIBRATION & THRESHOLDS ====================
const int SOIL_DRY_RAW   = 3150;
const int SOIL_WET_RAW   = 1350;

float reservoirMaxDepthCm = 45.0; // Distance when tank is EMPTY
float reservoirMinDistCm  = 5.0;  // Distance when tank is FULL

const int SOIL_DRY_THRESHOLD   = 35; // Trigger irrigation < 35%
const int SOIL_WET_THRESHOLD   = 70; // Target saturation 70%
const int TANK_ALERT_THRESHOLD = 20; // Low water warning < 20%
const int TANK_EMPTY_CUTOFF    = 15; // Cut off pump < 15% (dry-run protection)

const unsigned long RAIN_LOCKOUT_MS = 86400000UL; // 24 Hours in milliseconds

// ==================== TIMERS & SCHEDULER ====================
#define MAX_SCHEDULES 10

struct TimerSchedule {
  bool enabled;
  uint8_t hour;         // 0 - 23
  uint8_t minute;       // 0 - 59
  uint16_t durationMin; // Duration in minutes
  bool ranToday;
};

TimerSchedule schedules[MAX_SCHEDULES];
int numSchedules = 0;

bool scheduleRunning = false;
unsigned long scheduleEndTime = 0;
char nextRunString[32] = "None";

// ==================== OPERATION MODES ====================
enum OperationMode {
  MODE_AUTO_IRRIGATE = 0, // Soil moisture triggers pump
  MODE_SCHEDULE_ONLY = 1, // Strictly runs on timer schedules
  MODE_ALERT_ONLY    = 2, // Sensor alerts only, manual control
  MODE_FULL_MANUAL   = 3  // Fully manual control
};

OperationMode currentMode = MODE_AUTO_IRRIGATE;

// ==================== NETWORKING & CAPTIVE PORTAL ====================
Preferences preferences;

String wifiSSID = "Dracarys";
String wifiPass = "itsanewform";

bool inAPMode = false;
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* MQTT_BROKER = "test.mosquitto.org";
const int   MQTT_PORT   = 1883;

char deviceId[32] = "drop2crop_2884858a2564";
char topicTelemetry[64];
char topicCommand[64];
char topicStatus[64];

// ==================== SENSOR & SYSTEM STATE ====================
int soilRaw = 0;
int soilPercent = 0;

bool instantRain = false;
int rainAnalog = 0;
unsigned long lastRainTriggerTime = 0;
bool rainLocked = false;
unsigned long rainRemainingSec = 0;

float waterDistCm = 0.0;
int waterTankPercent = 0;

bool pumpActive = false;
bool valveActive = false;

// NTP Time variables (UTC+6 Bangladesh Standard Time)
const long  GMT_OFFSET_SEC = 21600;
const int   DAYLIGHT_OFFSET_SEC = 0;
struct tm   timeinfo;
bool        timeSynchronized = false;
char        currentTimeStr[16] = "--:--:--";

unsigned long lastSensorPoll = 0;
const unsigned long SENSOR_POLL_INTERVAL = 1500;

unsigned long lastCloudReport = 0;
const unsigned long CLOUD_REPORT_INTERVAL = 2000;

unsigned long lastScheduleCheck = 0;
unsigned long buttonPressStart = 0;

// ==================== SCHEDULE & WIFI STORAGE (NVS FLASH) ====================
void loadCredentials() {
  preferences.begin("d2c_cfg", false);
  wifiSSID = preferences.getString("ssid", "Dracarys");
  wifiPass = preferences.getString("pass", "itsanewform");
  preferences.end();
  Serial.printf("[NVS] Loaded WiFi Config: SSID='%s'\n", wifiSSID.c_str());

  preferences.begin("d2c_sched", false);
  numSchedules = preferences.getInt("count", 0);
  if (numSchedules > MAX_SCHEDULES) numSchedules = MAX_SCHEDULES;

  for (int i = 0; i < numSchedules; i++) {
    char keyEn[8], keyHr[8], keyMin[8], keyDur[8];
    snprintf(keyEn, sizeof(keyEn), "e%d", i);
    snprintf(keyHr, sizeof(keyHr), "h%d", i);
    snprintf(keyMin, sizeof(keyMin), "m%d", i);
    snprintf(keyDur, sizeof(keyDur), "d%d", i);

    schedules[i].enabled = preferences.getBool(keyEn, true);
    schedules[i].hour = preferences.getUChar(keyHr, 6);
    schedules[i].minute = preferences.getUChar(keyMin, 0);
    schedules[i].durationMin = preferences.getUShort(keyDur, 20);
    schedules[i].ranToday = false;
  }
  preferences.end();

  if (numSchedules == 0) {
    schedules[0] = {true, 6, 0, 20, false};   // 06:00 AM for 20 mins
    schedules[1] = {true, 18, 0, 40, false};  // 06:00 PM for 40 mins
    numSchedules = 2;
    saveSchedulesToNVS();
  }
  Serial.printf("[NVS] Loaded %d schedules from Flash memory.\n", numSchedules);
}

void saveWifiCredentials(const String& s, const String& p) {
  preferences.begin("d2c_cfg", false);
  preferences.putString("ssid", s);
  preferences.putString("pass", p);
  preferences.end();
  Serial.println("[NVS] New WiFi credentials saved to Flash!");
}

void saveSchedulesToNVS() {
  preferences.begin("d2c_sched", false);
  preferences.putInt("count", numSchedules);

  for (int i = 0; i < numSchedules; i++) {
    char keyEn[8], keyHr[8], keyMin[8], keyDur[8];
    snprintf(keyEn, sizeof(keyEn), "e%d", i);
    snprintf(keyHr, sizeof(keyHr), "h%d", i);
    snprintf(keyMin, sizeof(keyMin), "m%d", i);
    snprintf(keyDur, sizeof(keyDur), "d%d", i);

    preferences.putBool(keyEn, schedules[i].enabled);
    preferences.putUChar(keyHr, schedules[i].hour);
    preferences.putUChar(keyMin, schedules[i].minute);
    preferences.putUShort(keyDur, schedules[i].durationMin);
  }
  preferences.end();
  Serial.println("[NVS] Schedules saved to Flash.");
}

// ==================== CAPTIVE PORTAL WEB SERVER ====================
const char CAPTIVE_PORTAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Drop2Crop WiFi Setup</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background: #0f172a; color: #f8fafc; padding: 20px; display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 100vh; }
    .card { background: #1e293b; border-radius: 16px; padding: 24px; width: 100%; max-width: 400px; border: 1px solid #334155; box-shadow: 0 10px 25px rgba(0,0,0,0.5); }
    h1 { font-size: 20px; color: #10b981; margin-bottom: 8px; display: flex; align-items: center; gap: 8px; }
    p { font-size: 13px; color: #94a3b8; margin-bottom: 20px; line-height: 1.4; }
    label { font-size: 12px; font-weight: 600; color: #cbd5e1; display: block; margin-bottom: 6px; }
    input, select { width: 100%; padding: 12px; border-radius: 10px; border: 1px solid #475569; background: #0f172a; color: #fff; font-size: 14px; margin-bottom: 16px; outline: none; }
    input:focus, select:focus { border-color: #10b981; }
    .btn { width: 100%; padding: 14px; background: #10b981; color: #022c22; font-weight: bold; border: none; border-radius: 10px; font-size: 15px; cursor: pointer; transition: 0.2s; }
    .btn:active { transform: scale(0.98); background: #059669; }
    .footer { font-size: 11px; text-align: center; color: #64748b; margin-top: 16px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>🌱 Drop2Crop WiFi Setup</h1>
    <p>Connect this device to a new 2.4 GHz Wi-Fi network at your new location.</p>
    <form action="/save" method="POST">
      <label for="ssid">Select or Enter 2.4GHz Wi-Fi Name</label>
      <input type="text" id="ssid" name="ssid" placeholder="Network Name (SSID)" required>
      
      <label for="pass">Wi-Fi Password</label>
      <input type="password" id="pass" name="pass" placeholder="Enter Password">
      
      <button type="submit" class="btn">Connect & Save to Memory</button>
    </form>
    <div class="footer">Drop2Crop System · ESP32-S3 Flash Provisioning</div>
  </div>
</body>
</html>
)rawliteral";

void startCaptivePortal() {
  inAPMode = true;
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Drop2Crop_WiFi_Setup", "");
  delay(200);

  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

  dnsServer.start(DNS_PORT, "*", apIP);

  server.on("/", []() {
    server.send(200, "text/html", CAPTIVE_PORTAL_HTML);
  });

  server.on("/save", HTTP_POST, []() {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("pass");
    if (newSSID.length() > 0) {
      saveWifiCredentials(newSSID, newPass);
      server.send(200, "text/html", "<html><body style='background:#0f172a;color:#10b981;font-family:sans-serif;text-align:center;padding:40px;'><h2>WiFi Saved!</h2><p style='color:#fff;'>Rebooting and connecting to " + newSSID + "...</p></body></html>");
      delay(2000);
      ESP.restart();
    } else {
      server.send(400, "text/plain", "Missing SSID");
    }
  });

  server.onNotFound([]() {
    server.send(200, "text/html", CAPTIVE_PORTAL_HTML);
  });

  server.begin();
  Serial.println("\n==============================================");
  Serial.println("  [CAPTIVE PORTAL ACTIVE]");
  Serial.println("  Connect phone to WiFi: 'Drop2Crop_WiFi_Setup'");
  Serial.println("  Open browser: http://192.168.4.1");
  Serial.println("==============================================\n");
}

// ==================== ACTUATOR CONTROL ====================
void setPump(bool state, const char* reason = "Manual") {
  if (state && waterTankPercent < TANK_EMPTY_CUTOFF) {
    Serial.printf("[SAFETY BLOCK] Pump (%s) blocked: Tank < 15%% empty!\n", reason);
    pumpActive = false;
    digitalWrite(PIN_RELAY_PUMP, HIGH);
    return;
  }

  if (state && rainLocked && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.printf("[SAFETY BLOCK] Pump (%s) blocked: 24h Rain Lockout!\n", reason);
    pumpActive = false;
    digitalWrite(PIN_RELAY_PUMP, HIGH);
    return;
  }

  pumpActive = state;
  digitalWrite(PIN_RELAY_PUMP, pumpActive ? LOW : HIGH);
  Serial.printf("[RELAY] Pump state -> %s (Trigger: %s)\n", pumpActive ? "ON" : "OFF", reason);
}

void setValve(bool state) {
  valveActive = state;
  digitalWrite(PIN_RELAY_VALVE, valveActive ? LOW : HIGH);
}

// ==================== SENSOR READING ====================
void readSensors() {
  // 1. Soil Moisture
  soilRaw = analogRead(PIN_SOIL_ANALOG);
  soilPercent = map(soilRaw, SOIL_DRY_RAW, SOIL_WET_RAW, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);

  // 2. Rain Sensor
  instantRain = (digitalRead(PIN_RAIN_DIGITAL) == LOW);
  rainAnalog = analogRead(PIN_RAIN_ANALOG);

  if (instantRain) {
    lastRainTriggerTime = millis();
    rainLocked = true;
  }

  if (rainLocked) {
    unsigned long elapsed = millis() - lastRainTriggerTime;
    if (elapsed < RAIN_LOCKOUT_MS) {
      rainRemainingSec = (RAIN_LOCKOUT_MS - elapsed) / 1000UL;
    } else {
      rainLocked = false;
      rainRemainingSec = 0;
      Serial.println("[RAIN] 24-Hour Lockout completed. Normal operations unlocked.");
    }
  } else {
    rainRemainingSec = 0;
  }

  // 3. HC-SR04 Ultrasonic Water Level
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0) {
    waterDistCm = reservoirMaxDepthCm;
  } else {
    waterDistCm = (duration * 0.0343) / 2.0;
  }

  float usableHeight = reservoirMaxDepthCm - reservoirMinDistCm;
  if (usableHeight > 0) {
    float waterDepth = reservoirMaxDepthCm - waterDistCm;
    waterTankPercent = (int)((waterDepth / usableHeight) * 100.0);
    waterTankPercent = constrain(waterTankPercent, 0, 100);
  } else {
    waterTankPercent = 50;
  }
}

// ==================== TIME & SCHEDULER ENGINE ====================
void updateLocalTime() {
  if (getLocalTime(&timeinfo)) {
    timeSynchronized = true;
    snprintf(currentTimeStr, sizeof(currentTimeStr), "%02d:%02d:%02d",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
}

void processScheduler() {
  if (!timeSynchronized) return;

  int curHour = timeinfo.tm_hour;
  int curMin  = timeinfo.tm_min;

  if (curHour == 0 && curMin == 0 && timeinfo.tm_sec <= 2) {
    for (int i = 0; i < numSchedules; i++) {
      schedules[i].ranToday = false;
    }
  }

  if (scheduleRunning) {
    if (millis() >= scheduleEndTime) {
      scheduleRunning = false;
      Serial.println("[TIMER] Scheduled run finished. Turning pump OFF.");
      setPump(false, "Timer Finished");
    }
    return;
  }

  if (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY) {
    for (int i = 0; i < numSchedules; i++) {
      if (schedules[i].enabled && !schedules[i].ranToday) {
        if (curHour == schedules[i].hour && curMin == schedules[i].minute) {
          schedules[i].ranToday = true;

          if (rainLocked) {
            Serial.printf("[TIMER SKIP] Schedule %d (%02d:%02d) skipped due to Rain Lockout.\n",
                          i + 1, schedules[i].hour, schedules[i].minute);
            continue;
          }
          if (waterTankPercent < TANK_EMPTY_CUTOFF) {
            Serial.printf("[TIMER SKIP] Schedule %d (%02d:%02d) skipped: reservoir empty!\n",
                          i + 1, schedules[i].hour, schedules[i].minute);
            continue;
          }

          scheduleRunning = true;
          scheduleEndTime = millis() + (schedules[i].durationMin * 60000UL);
          Serial.printf("[TIMER TRIGGER] Starting Schedule %d: %02d:%02d for %d mins!\n",
                        i + 1, schedules[i].hour, schedules[i].minute, schedules[i].durationMin);
          setPump(true, "Timer Schedule");
          break;
        }
      }
    }
  }

  int minDiff = 999999;
  int nextIdx = -1;
  int curTotalMin = curHour * 60 + curMin;

  for (int i = 0; i < numSchedules; i++) {
    if (schedules[i].enabled) {
      int schedTotalMin = schedules[i].hour * 60 + schedules[i].minute;
      int diff = schedTotalMin - curTotalMin;
      if (diff <= 0) diff += 1440;
      if (diff < minDiff) {
        minDiff = diff;
        nextIdx = i;
      }
    }
  }

  if (nextIdx >= 0) {
    int hrs = minDiff / 60;
    int mins = minDiff % 60;
    snprintf(nextRunString, sizeof(nextRunString), "%02d:%02d (%dm in %dh %dm)",
             schedules[nextIdx].hour, schedules[nextIdx].minute,
             schedules[nextIdx].durationMin, hrs, mins);
  } else {
    snprintf(nextRunString, sizeof(nextRunString), "None configured");
  }
}

// ==================== AUTOMATION ENGINE ====================
void processAutomation() {
  if (waterTankPercent < TANK_EMPTY_CUTOFF && pumpActive) {
    Serial.println("[SAFETY CUTOFF] Tank empty! Stopping pump.");
    scheduleRunning = false;
    setPump(false, "Emergency Cutoff");
    return;
  }

  if (rainLocked && pumpActive && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.println("[SAFETY CUTOFF] Rain lockout active! Stopping pump.");
    scheduleRunning = false;
    setPump(false, "Rain Lockout");
    return;
  }

  if (currentMode == MODE_AUTO_IRRIGATE && !scheduleRunning) {
    if (soilPercent < SOIL_DRY_THRESHOLD && !pumpActive && !rainLocked && waterTankPercent >= TANK_EMPTY_CUTOFF) {
      Serial.println("[AUTO TRIGGER] Soil moisture < 35%. Starting irrigation.");
      setPump(true, "Soil Sensor Auto");
    } else if (soilPercent >= SOIL_WET_THRESHOLD && pumpActive) {
      Serial.println("[AUTO SHUTOFF] Optimal moisture reached. Stopping pump.");
      setPump(false, "Soil Saturated");
    }
  }
}

// ==================== MQTT MESSAGING ====================
void publishTelemetry() {
  StaticJsonDocument<512> doc;
  doc["dev"]      = deviceId;
  doc["time"]     = currentTimeStr;
  doc["soil"]     = soilPercent;
  doc["soilRaw"]  = soilRaw;
  doc["soilAlert"]= (soilPercent < SOIL_DRY_THRESHOLD);
  
  doc["water"]    = waterTankPercent;
  doc["waterCm"]  = serialized(String(waterDistCm, 1));
  doc["waterLow"] = (waterTankPercent < TANK_ALERT_THRESHOLD);
  doc["waterEmpty"] = (waterTankPercent < TANK_EMPTY_CUTOFF);

  doc["rainNow"]  = instantRain;
  doc["rainLock"] = rainLocked;
  doc["rainSec"]  = rainRemainingSec;

  doc["pump"]     = pumpActive;
  doc["mode"]     = (int)currentMode;
  doc["schedRun"] = scheduleRunning;
  doc["nextRun"]  = nextRunString;

  JsonArray arr = doc.createNestedArray("schedules");
  for (int i = 0; i < numSchedules; i++) {
    JsonObject s = arr.createNestedObject();
    s["id"]  = i;
    s["en"]  = schedules[i].enabled;
    s["h"]   = schedules[i].hour;
    s["m"]   = schedules[i].minute;
    s["dur"] = schedules[i].durationMin;
  }

  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(topicTelemetry, buffer, n);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("[MQTT] JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("pump")) {
    const char* pCmd = doc["pump"];
    scheduleRunning = false;
    if (strcmp(pCmd, "on") == 0) {
      setPump(true, "App Remote Command");
    } else if (strcmp(pCmd, "off") == 0) {
      setPump(false, "App Remote Command");
    }
  }

  if (doc.containsKey("mode")) {
    int m = doc["mode"];
    if (m >= 0 && m <= 3) {
      currentMode = (OperationMode)m;
      Serial.printf("[COMMAND] Mode updated to: %d\n", m);
    }
  }

  if (doc.containsKey("clearRainLock")) {
    rainLocked = false;
    rainRemainingSec = 0;
    Serial.println("[COMMAND] Rain lockout cleared.");
  }

  if (doc.containsKey("addSched")) {
    if (numSchedules < MAX_SCHEDULES) {
      JsonObject s = doc["addSched"];
      schedules[numSchedules].enabled = true;
      schedules[numSchedules].hour = s["h"] | 6;
      schedules[numSchedules].minute = s["m"] | 0;
      schedules[numSchedules].durationMin = s["dur"] | 15;
      schedules[numSchedules].ranToday = false;
      numSchedules++;
      saveSchedulesToNVS();
      Serial.printf("[COMMAND] Added schedule slot %d: %02d:%02d (%d min)\n",
                    numSchedules, schedules[numSchedules-1].hour,
                    schedules[numSchedules-1].minute, schedules[numSchedules-1].durationMin);
    }
  }

  if (doc.containsKey("delSched")) {
    int idx = doc["delSched"];
    if (idx >= 0 && idx < numSchedules) {
      for (int i = idx; i < numSchedules - 1; i++) {
        schedules[i] = schedules[i + 1];
      }
      numSchedules--;
      saveSchedulesToNVS();
      Serial.printf("[COMMAND] Deleted schedule slot %d.\n", idx);
    }
  }

  if (doc.containsKey("toggleSched")) {
    int idx = doc["toggleSched"];
    if (idx >= 0 && idx < numSchedules) {
      schedules[idx].enabled = doc["en"];
      saveSchedulesToNVS();
      Serial.printf("[COMMAND] Toggled schedule %d.\n", idx);
    }
  }

  if (doc.containsKey("resetWifi")) {
    Serial.println("[COMMAND] Remote WiFi reset received. Launching Captive Portal...");
    startCaptivePortal();
    return;
  }

  publishTelemetry();
}

void connectMQTT() {
  if (mqttClient.connected()) return;

  Serial.print("[MQTT] Connecting to broker ");
  Serial.print(MQTT_BROKER);
  Serial.print(" as ");
  Serial.println(deviceId);

  if (mqttClient.connect(deviceId)) {
    Serial.println("[MQTT] Connected to Cloud Broker!");
    mqttClient.publish(topicStatus, "{\"online\":true}");
    mqttClient.subscribe(topicCommand);
    Serial.print("[MQTT] Subscribed to: ");
    Serial.println(topicCommand);
    publishTelemetry();
  } else {
    Serial.print("[MQTT] Connection failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n==============================================");
  Serial.println("  Drop2Crop - Cloud IoT & Precision Station");
  Serial.println("==============================================");

  // Initialize unique topics based on MAC address
  WiFi.mode(WIFI_STA);
  delay(100);
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(deviceId, sizeof(deviceId), "drop2crop_%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  snprintf(topicTelemetry, sizeof(topicTelemetry), "drop2crop/%s/telemetry", deviceId);
  snprintf(topicCommand,   sizeof(topicCommand),   "drop2crop/%s/cmd", deviceId);
  snprintf(topicStatus,    sizeof(topicStatus),    "drop2crop/%s/status", deviceId);

  Serial.printf("Device ID:        %s\n", deviceId);
  Serial.printf("Telemetry Topic:  %s\n", topicTelemetry);
  Serial.printf("Command Topic:    %s\n", topicCommand);

  // Initialize Pins
  pinMode(PIN_SOIL_ANALOG, INPUT);
  pinMode(PIN_RAIN_DIGITAL, INPUT_PULLUP);
  pinMode(PIN_RAIN_ANALOG, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP); // Onboard BOOT button

  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_RELAY_VALVE, OUTPUT);

  setPump(false, "System Boot");
  setValve(false);

  // Load Saved WiFi & Schedules from Flash Memory
  loadCredentials();

  // Attempt to connect to saved WiFi (timeout 12s)
  Serial.printf("[WiFi] Connecting to saved network: %s ...\n", wifiSSID.c_str());
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 12000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected to %s! IP: %s\n", wifiSSID.c_str(), WiFi.localIP().toString().c_str());
    // Setup MQTT & NTP
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com");
  } else {
    Serial.println("\n[WiFi] Could not connect to saved network. Starting Captive Portal for new WiFi setup...");
    startCaptivePortal();
  }
}

void loop() {
  // If in Captive Portal Mode, handle DNS and Web Server requests
  if (inAPMode) {
    dnsServer.processNextRequest();
    server.handleClient();
    return;
  }

  // Check Physical BOOT button (Hold for 3s to reset WiFi anytime)
  if (digitalRead(PIN_BOOT_BUTTON) == LOW) {
    if (buttonPressStart == 0) {
      buttonPressStart = millis();
    } else if (millis() - buttonPressStart > 3000) {
      Serial.println("\n[BUTTON] BOOT button held 3s! Triggering WiFi Captive Portal...");
      startCaptivePortal();
      buttonPressStart = 0;
      return;
    }
  } else {
    buttonPressStart = 0;
  }

  // Normal Connected Operation
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      static unsigned long lastMqttRetry = 0;
      if (millis() - lastMqttRetry > 4000) {
        lastMqttRetry = millis();
        connectMQTT();
      }
    } else {
      mqttClient.loop();
    }
  }

  // Scheduler Engine Check (Every 1s)
  unsigned long now = millis();
  if (now - lastScheduleCheck >= 1000) {
    lastScheduleCheck = now;
    updateLocalTime();
    processScheduler();
  }

  // Sensor Polling & Safety Automation (Every 1.5s)
  if (now - lastSensorPoll >= SENSOR_POLL_INTERVAL) {
    lastSensorPoll = now;
    readSensors();
    processAutomation();
  }

  // Cloud Telemetry Reporting (Every 2s)
  if (now - lastCloudReport >= CLOUD_REPORT_INTERVAL) {
    lastCloudReport = now;

    if (mqttClient.connected()) {
      publishTelemetry();
    }

    Serial.printf("[%s] WiFi:%s | Soil:%d%% | Water:%d%%(%.1fcm) | Rain:%s(lock:%us) | Pump:%s | Mode:%d | Next:%s\n",
                  currentTimeStr,
                  (WiFi.status() == WL_CONNECTED) ? "OK" : "NO",
                  soilPercent, waterTankPercent, waterDistCm,
                  instantRain ? "WET" : "DRY", rainRemainingSec,
                  pumpActive ? "ON" : "OFF", (int)currentMode, nextRunString);
  }
}
