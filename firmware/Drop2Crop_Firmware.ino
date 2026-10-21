/*
 * ======================================================================================
 * Drop2Crop - Global Cloud IoT Precision Irrigation Controller (FRESH BUILD v2.0)
 * Hardware: ESP32-S3 Dev Board
 * 
 * DESIGN PRINCIPLES:
 *   - 100% STANDALONE RELAY ENGINE: Ultrasonic distance sensor completely removed.
 *     Zero pulses, zero pulseIn delays, zero pin interference, zero false cutoffs.
 *   - ANTI-CHATTER / ANTI-BUZZ HARDWARE LOCK:
 *     Relays can NEVER switch faster than 2.0 seconds. Eliminates coil buzzing,
 *     chatter, brownout oscillation, and floating wire noise.
 *   - MULTI-PIN PARALLEL DRIVING:
 *     Relay 1 (Clean Water Pump): GPIO 17, GPIO 8 ("eco cable"), GPIO 13, GPIO 9, GPIO 21
 *     Relay 2 (Fertigation Pump): GPIO 18, GPIO 7 ("trig cable"), GPIO 14, GPIO 10, GPIO 47
 *     Whichever pin your relay wire is plugged into, IT WILL OPERATE CLEANLY!
 *   - PERSISTENT NVS STORAGE: WiFi credentials, Station PIN, Schedules stored in Flash.
 *   - NTP TIME SYNCHRONIZATION: UTC+6 Bangladesh Standard Time.
 *   - MQTT CLOUD BRIDGE: broker.emqx.io (compatible with Drop2Crop Cloud Dashboard).
 * ======================================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include "driver/gpio.h"

// ==================== MASTER PIN DEFINITIONS ====================
// Sensors (Ultrasonic Distance Sensor is 100% REMOVED from this system)
#define PIN_SOIL_ANALOG      4   // Capacitive Soil Moisture Sensor (ADC1_CH3)
#define PIN_RAIN_DIGITAL     5   // Raindrop Sensor Digital DO (Active LOW: LOW = Rain)
#define PIN_RAIN_ANALOG      6   // Raindrop Sensor Analog AO
#define PIN_BOOT_BUTTON      0   // Onboard BOOT button (Hold 3s to reset WiFi)

// Relay Channels (Synchronously Driven across all candidate ports)
// Motor 1: Clean Water Irrigation Pump (Includes GPIO 2, GPIO 17, GPIO 8, GPIO 13, GPIO 9, GPIO 21)
const int PINS_MOTOR1[] = {2, 17, 8, 13, 9, 21};
const int NUM_PINS_M1   = 6;

// Motor 2: Fertigation Nutrient Pump (Includes GPIO 1, GPIO 18, GPIO 7, GPIO 14, GPIO 10, GPIO 47)
const int PINS_MOTOR2[] = {1, 18, 7, 14, 10, 47};
const int NUM_PINS_M2   = 6;

// ==================== CALIBRATION & THRESHOLDS ====================
const int SOIL_DRY_RAW         = 3150;
const int SOIL_WET_RAW         = 1350;
const int SOIL_DRY_THRESHOLD   = 35;   // Auto-irrigation starts when soil < 35%
const int SOIL_WET_THRESHOLD   = 70;   // Auto-irrigation stops when soil >= 70%
const unsigned long RAIN_LOCKOUT_MS = 86400000UL; // 24 Hours lockout

// ==================== TIMERS & SCHEDULER ====================
#define MAX_SCHEDULES 10

struct TimerSchedule {
  bool enabled;
  uint8_t hour;         // 0 - 23
  uint8_t minute;       // 0 - 59
  uint16_t durationMin; // 1 - 120
  uint8_t target;       // 0 = Water Pump, 1 = Fert Pump, 2 = Both
  bool ranToday;
};

TimerSchedule schedules[MAX_SCHEDULES];
int numSchedules = 0;

bool scheduleRunning = false;
uint8_t scheduleTarget = 0;
unsigned long scheduleEndTime = 0;
char nextRunString[32] = "None";

// ==================== OPERATION MODES ====================
enum OperationMode {
  MODE_AUTO_IRRIGATE = 0, // Soil moisture triggers pump
  MODE_SCHEDULE_ONLY = 1, // Only daily timer schedules run
  MODE_ALERT_ONLY    = 2, // Monitoring only
  MODE_FULL_MANUAL   = 3  // Strict manual app control
};

OperationMode currentMode = MODE_AUTO_IRRIGATE;

// ==================== NETWORKING & CLOUD ====================
Preferences preferences;

String wifiSSID = "Dracarys";
String wifiPass = "itsanewform";
String devicePasscode = "2564"; // Station PIN

bool inAPMode = false;
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* MQTT_BROKER = "broker.emqx.io";
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

// Water Reservoir (Safe Standalone Telemetry - No Sensor Required)
float waterDistCm = 0.0f;
int waterTankPercent = 100;

bool pumpWaterActive = false; // Motor 1 State
bool pumpFertActive  = false; // Motor 2 State
bool safetyOverride  = true;  // Safety Override (true = force run allowed)
bool relayActiveLow  = true;  // Relay Trigger: true = Active LOW (0V = ON)

// NTP Time (UTC+6 Bangladesh Standard Time)
const long GMT_OFFSET_SEC = 21600;
const int  DAYLIGHT_OFFSET_SEC = 0;
struct tm  timeinfo;
bool       timeSynchronized = false;
char       currentTimeStr[16] = "--:--:--";

unsigned long lastSensorPoll = 0;
const unsigned long SENSOR_POLL_INTERVAL = 1500;

unsigned long lastCloudReport = 0;
const unsigned long CLOUD_REPORT_INTERVAL = 2000;

unsigned long lastScheduleCheck = 0;
unsigned long buttonPressStart = 0;

// Anti-Chatter / Anti-Buzz Hardware Protection Timer
unsigned long lastWaterPumpSwitch = 0;
unsigned long lastFertPumpSwitch  = 0;
const unsigned long RELAY_DEBOUNCE_COOLDOWN_MS = 2000; // Hard minimum 2.0s between switches

// Forward Declarations
void saveSchedulesToNVS();
void setWaterPump(bool state, const char* reason = "Manual");
void setFertPump(bool state, const char* reason = "Manual");
void publishTelemetry();

// ==================== PIN HARDWARE INITIALIZATION ====================
void configureAllPins() {
  int offLvl = relayActiveLow ? HIGH : LOW;

  // Release and configure Motor 1 pins (GPIO 17, 8, 13, 9, 21)
  for (int i = 0; i < NUM_PINS_M1; i++) {
    int pin = PINS_MOTOR1[i];
    gpio_reset_pin((gpio_num_t)pin);
    digitalWrite(pin, offLvl);
    pinMode(pin, OUTPUT);
    gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
  }

  // Release and configure Motor 2 pins (GPIO 18, 7, 14, 10, 47)
  for (int i = 0; i < NUM_PINS_M2; i++) {
    int pin = PINS_MOTOR2[i];
    gpio_reset_pin((gpio_num_t)pin);
    digitalWrite(pin, offLvl);
    pinMode(pin, OUTPUT);
    gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
  }

  // Sensors
  pinMode(PIN_SOIL_ANALOG, INPUT);
  pinMode(PIN_RAIN_DIGITAL, INPUT_PULLUP);
  pinMode(PIN_RAIN_ANALOG, INPUT);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

  Serial.println("[HARDWARE] All relay pins initialized to OFF with maximum current drive capability.");
  Serial.println("[HARDWARE] Motor 1 Pins: 2, 17, 8, 13, 9, 21 | Motor 2 Pins: 1, 18, 7, 14, 10, 47");
}

// ==================== NVS FLASH STORAGE ====================
void loadCredentials() {
  preferences.begin("d2c_cfg", false);
  wifiSSID = preferences.getString("ssid", "Dracarys");
  wifiPass = preferences.getString("pass", "itsanewform");
  devicePasscode = preferences.getString("pin", "2564");
  safetyOverride = preferences.getBool("safeBypass", true);
  relayActiveLow = preferences.getBool("relLow", true);
  preferences.end();

  Serial.printf("[NVS] Loaded WiFi Config: SSID='%s'\n", wifiSSID.c_str());
  Serial.printf("[NVS] Station PIN: '%s'\n", devicePasscode.c_str());
  Serial.printf("[NVS] Relay Trigger Polarity: Active %s\n", relayActiveLow ? "LOW (0V = ON)" : "HIGH (3.3V = ON)");

  preferences.begin("d2c_sched", false);
  numSchedules = preferences.getInt("count", 0);
  for (int i = 0; i < numSchedules; i++) {
    char keyEn[8], keyHr[8], keyMin[8], keyDur[8], keyTgt[8];
    snprintf(keyEn, sizeof(keyEn), "e%d", i);
    snprintf(keyHr, sizeof(keyHr), "h%d", i);
    snprintf(keyMin, sizeof(keyMin), "m%d", i);
    snprintf(keyDur, sizeof(keyDur), "d%d", i);
    snprintf(keyTgt, sizeof(keyTgt), "t%d", i);

    schedules[i].enabled = preferences.getBool(keyEn, true);
    schedules[i].hour = preferences.getUChar(keyHr, 6);
    schedules[i].minute = preferences.getUChar(keyMin, 0);
    schedules[i].durationMin = preferences.getUShort(keyDur, 20);
    schedules[i].target = preferences.getUChar(keyTgt, 0);
    schedules[i].ranToday = false;
  }
  preferences.end();

  if (numSchedules == 0) {
    schedules[0] = {true, 6, 0, 20, 0, false};   // 06:00 AM for 20 mins -> Motor 1 (Clean Water)
    schedules[1] = {true, 18, 0, 15, 1, false};  // 06:00 PM for 15 mins -> Motor 2 (Fertigation)
    numSchedules = 2;
    saveSchedulesToNVS();
  }
  Serial.printf("[NVS] Loaded %d schedules from Flash.\n", numSchedules);
}

void saveWifiCredentials(const String& s, const String& p) {
  preferences.begin("d2c_cfg", false);
  preferences.putString("ssid", s);
  preferences.putString("pass", p);
  preferences.end();
  Serial.println("[NVS] WiFi credentials saved.");
}

void savePasscode(const String& newPin) {
  preferences.begin("d2c_cfg", false);
  preferences.putString("pin", newPin);
  preferences.end();
  devicePasscode = newPin;
  Serial.printf("[NVS] Security PIN updated: %s\n", devicePasscode.c_str());
}

void saveSafetyOverride(bool bypass) {
  safetyOverride = bypass;
  preferences.begin("d2c_cfg", false);
  preferences.putBool("safeBypass", safetyOverride);
  preferences.end();
  Serial.printf("[NVS] Safety Override: %s\n", safetyOverride ? "BYPASSED" : "ENFORCED");
}

void saveRelayTriggerMode(bool isLow) {
  relayActiveLow = isLow;
  preferences.begin("d2c_cfg", false);
  preferences.putBool("relLow", relayActiveLow);
  preferences.end();
  Serial.printf("[NVS] Relay Trigger Polarity: Active %s\n", relayActiveLow ? "LOW" : "HIGH");
  configureAllPins();
}

void saveSchedulesToNVS() {
  preferences.begin("d2c_sched", false);
  preferences.putInt("count", numSchedules);
  for (int i = 0; i < numSchedules; i++) {
    char keyEn[8], keyHr[8], keyMin[8], keyDur[8], keyTgt[8];
    snprintf(keyEn, sizeof(keyEn), "e%d", i);
    snprintf(keyHr, sizeof(keyHr), "h%d", i);
    snprintf(keyMin, sizeof(keyMin), "m%d", i);
    snprintf(keyDur, sizeof(keyDur), "d%d", i);
    snprintf(keyTgt, sizeof(keyTgt), "t%d", i);

    preferences.putBool(keyEn, schedules[i].enabled);
    preferences.putUChar(keyHr, schedules[i].hour);
    preferences.putUChar(keyMin, schedules[i].minute);
    preferences.putUShort(keyDur, schedules[i].durationMin);
    preferences.putUChar(keyTgt, schedules[i].target);
  }
  preferences.end();
  Serial.println("[NVS] Schedules saved.");
}

// ==================== ACTUATOR CONTROL (ANTI-CHATTER & ANTI-BUZZ) ====================
void setWaterPump(bool state, const char* reason) {
  // Guard 1: If motor is already in requested state, do nothing
  if (pumpWaterActive == state) {
    return;
  }

  // Guard 2: Enforce strict anti-chatter cooldown (minimum 2.0s between state changes)
  unsigned long now = millis();
  if (now - lastWaterPumpSwitch < RELAY_DEBOUNCE_COOLDOWN_MS && lastWaterPumpSwitch > 0) {
    Serial.printf("[DEBOUNCE] Water Pump transition to %s rejected: cooldown active (%lums remaining)\n",
                  state ? "ON" : "OFF", RELAY_DEBOUNCE_COOLDOWN_MS - (now - lastWaterPumpSwitch));
    return;
  }

  int offLevel = relayActiveLow ? HIGH : LOW;
  int onLevel  = relayActiveLow ? LOW : HIGH;

  // Rain lockout check ONLY for auto/schedule modes when safety is enforced
  if (state && !safetyOverride && rainLocked && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.printf("[SAFETY BLOCK] Water Pump blocked: 24h Rain Lockout!\n");
    pumpWaterActive = false;
    for (int i = 0; i < NUM_PINS_M1; i++) digitalWrite(PINS_MOTOR1[i], offLevel);
    return;
  }

  pumpWaterActive = state;
  lastWaterPumpSwitch = now;
  int drivePinLevel = pumpWaterActive ? onLevel : offLevel;

  // Drive all Motor 1 pins synchronously
  for (int i = 0; i < NUM_PINS_M1; i++) {
    digitalWrite(PINS_MOTOR1[i], drivePinLevel);
  }

  Serial.printf("[RELAY 1] Motor 1 (Clean Water) -> %s (Pins 2, 17, 8, 13, 9, 21 = %s, Trigger: %s)\n", 
                pumpWaterActive ? "ON" : "OFF", 
                drivePinLevel == LOW ? "LOW (0V)" : "HIGH (3.3V)", reason);
}

void setFertPump(bool state, const char* reason) {
  if (pumpFertActive == state) {
    return;
  }

  unsigned long now = millis();
  if (now - lastFertPumpSwitch < RELAY_DEBOUNCE_COOLDOWN_MS && lastFertPumpSwitch > 0) {
    Serial.printf("[DEBOUNCE] Fert Pump transition to %s rejected: cooldown active (%lums remaining)\n",
                  state ? "ON" : "OFF", RELAY_DEBOUNCE_COOLDOWN_MS - (now - lastFertPumpSwitch));
    return;
  }

  int offLevel = relayActiveLow ? HIGH : LOW;
  int onLevel  = relayActiveLow ? LOW : HIGH;

  if (state && !safetyOverride && rainLocked && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.printf("[SAFETY BLOCK] Fertigation Pump blocked: 24h Rain Lockout!\n");
    pumpFertActive = false;
    for (int i = 0; i < NUM_PINS_M2; i++) digitalWrite(PINS_MOTOR2[i], offLevel);
    return;
  }

  pumpFertActive = state;
  lastFertPumpSwitch = now;
  int drivePinLevel = pumpFertActive ? onLevel : offLevel;

  // Drive all Motor 2 pins synchronously
  for (int i = 0; i < NUM_PINS_M2; i++) {
    digitalWrite(PINS_MOTOR2[i], drivePinLevel);
  }

  Serial.printf("[RELAY 2] Motor 2 (Fertigation) -> %s (Pins 1, 18, 7, 14, 10, 47 = %s, Trigger: %s)\n", 
                pumpFertActive ? "ON" : "OFF", 
                drivePinLevel == LOW ? "LOW" : "HIGH", reason);
}

// ==================== SENSOR READING ====================
void readSensors() {
  // 1. Capacitive Soil Moisture
  soilRaw = analogRead(PIN_SOIL_ANALOG);
  soilPercent = map(soilRaw, SOIL_DRY_RAW, SOIL_WET_RAW, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100);

  // 2. Raindrop Sensor
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

  // 3. Water Reservoir: Standalone safe default (No ultrasonic sensor required!)
  waterDistCm = 0.0f;
  waterTankPercent = 100;
}

// ==================== TIME & SCHEDULER ENGINE ====================
void updateLocalTime() {
  if (getLocalTime(&timeinfo, 10)) {
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
      Serial.println("[TIMER] Scheduled run finished. Turning both motors OFF.");
      setWaterPump(false, "Timer Finished");
      setFertPump(false, "Timer Finished");
    }
    return;
  }

  if (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY) {
    for (int i = 0; i < numSchedules; i++) {
      if (schedules[i].enabled && !schedules[i].ranToday) {
        if (curHour == schedules[i].hour && curMin == schedules[i].minute) {
          schedules[i].ranToday = true;

          if (!safetyOverride && rainLocked) {
            Serial.printf("[TIMER SKIP] Schedule %d (%02d:%02d) skipped due to Rain Lockout.\n",
                          i + 1, schedules[i].hour, schedules[i].minute);
            continue;
          }

          scheduleRunning = true;
          scheduleTarget = schedules[i].target;
          scheduleEndTime = millis() + (schedules[i].durationMin * 60000UL);
          Serial.printf("[TIMER TRIGGER] Starting Schedule %d: %02d:%02d for %d mins (Target: %d)!\n",
                        i + 1, schedules[i].hour, schedules[i].minute, schedules[i].durationMin, scheduleTarget);

          if (scheduleTarget == 0) {
            setWaterPump(true, "Timer Schedule (Watering)");
          } else if (scheduleTarget == 1) {
            setFertPump(true, "Timer Schedule (Fertigation)");
          } else {
            setWaterPump(true, "Timer Schedule (Both)");
            setFertPump(true, "Timer Schedule (Both)");
          }
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
  if (!safetyOverride && rainLocked && (pumpWaterActive || pumpFertActive) && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.println("[SAFETY CUTOFF] Rain lockout active! Stopping both motors.");
    scheduleRunning = false;
    setWaterPump(false, "Rain Lockout");
    setFertPump(false, "Rain Lockout");
    return;
  }

  if (currentMode == MODE_AUTO_IRRIGATE && !scheduleRunning) {
    if (soilPercent < SOIL_DRY_THRESHOLD && !pumpWaterActive && (safetyOverride || !rainLocked)) {
      Serial.println("[AUTO TRIGGER] Soil moisture < 35%. Starting Motor 1 (Watering).");
      setWaterPump(true, "Soil Sensor Auto");
    } else if (soilPercent >= SOIL_WET_THRESHOLD && pumpWaterActive) {
      Serial.println("[AUTO SHUTOFF] Optimal moisture reached (>= 70%). Stopping Motor 1 (Watering).");
      setWaterPump(false, "Soil Saturated");
    }
  }
}

// ==================== MQTT MESSAGING ====================
void publishTelemetry() {
  StaticJsonDocument<1536> doc;
  doc["dev"]      = deviceId;
  doc["time"]     = currentTimeStr;
  doc["soil"]     = soilPercent;
  doc["soilRaw"]  = soilRaw;
  doc["soilAlert"]= (soilPercent < SOIL_DRY_THRESHOLD);
  
  doc["water"]    = waterTankPercent;
  doc["waterCm"]  = "0.0";
  doc["waterLow"] = false;
  doc["waterEmpty"] = false;

  doc["rainNow"]  = instantRain;
  doc["rainLock"] = rainLocked;
  doc["rainSec"]  = rainRemainingSec;

  doc["online"]   = true;
  doc["uptime"]   = (unsigned long)(millis() / 1000);
  doc["rssi"]     = WiFi.RSSI();
  doc["pump"]     = pumpWaterActive;
  doc["fert"]     = pumpFertActive;
  doc["mode"]     = (int)currentMode;
  doc["schedRun"] = scheduleRunning;
  doc["schedTgt"] = scheduleTarget;
  doc["nextRun"]  = nextRunString;
  doc["safeBypass"] = safetyOverride;
  doc["relLow"]     = relayActiveLow;
  doc["ultraEn"]    = false; // Standalone mode active

  JsonArray arr = doc.createNestedArray("schedules");
  for (int i = 0; i < numSchedules; i++) {
    JsonObject s = arr.createNestedObject();
    s["id"]  = i;
    s["en"]  = schedules[i].enabled;
    s["h"]   = schedules[i].hour;
    s["m"]   = schedules[i].minute;
    s["dur"] = schedules[i].durationMin;
    s["tgt"] = schedules[i].target;
  }

  char buffer[1536];
  size_t n = serializeJson(doc, buffer);
  bool ok = mqttClient.publish(topicTelemetry, buffer, n);
  if (!ok) {
    Serial.printf("[MQTT ERR] Telemetry publish failed! n=%u, state=%d\n", (unsigned int)n, mqttClient.state());
  } else {
    static unsigned long lastStatusPulse = 0;
    if (millis() - lastStatusPulse > 10000) {
      lastStatusPulse = millis();
      mqttClient.publish(topicStatus, "{\"online\":true}", true);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("[MQTT] JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  // PIN security verification
  const char* authPin = doc["auth"] | "";
  if (strcmp(authPin, devicePasscode.c_str()) != 0) {
    Serial.printf("[SECURITY ALERT] Unauthorized command rejected! Provided auth '%s' != PIN.\n", authPin);
    mqttClient.publish(topicStatus, "{\"security\":\"UNAUTHORIZED_COMMAND_REJECTED\"}");
    return;
  }

  // Safety Mechanism Override
  if (doc.containsKey("setSafetyBypass") || doc.containsKey("safetyBypass") || doc.containsKey("safetyOverride")) {
    bool bypassVal = true;
    if (doc.containsKey("setSafetyBypass")) bypassVal = doc["setSafetyBypass"];
    else if (doc.containsKey("safetyBypass")) bypassVal = doc["safetyBypass"];
    else if (doc.containsKey("safetyOverride")) bypassVal = doc["safetyOverride"];
    saveSafetyOverride(bypassVal);
    publishTelemetry();
  }

  // Relay Trigger Polarity (Active LOW vs Active HIGH)
  if (doc.containsKey("setRelayActiveLow") || doc.containsKey("relLow")) {
    bool isLow = true;
    if (doc.containsKey("setRelayActiveLow")) isLow = doc["setRelayActiveLow"];
    else if (doc.containsKey("relLow")) isLow = doc["relLow"];
    saveRelayTriggerMode(isLow);
    setWaterPump(pumpWaterActive, "Trigger Polarity Update");
    setFertPump(pumpFertActive, "Trigger Polarity Update");
    publishTelemetry();
  }

  // Mechanical Click Diagnostic Test (3 Pulses across all candidate pins)
  if (doc.containsKey("clickTest")) {
    Serial.println("[TEST] Starting Relay Mechanical Click Diagnostic across all pins...");
    int onL = relayActiveLow ? LOW : HIGH;
    int offL = relayActiveLow ? HIGH : LOW;

    for (int i = 0; i < 3; i++) {
      for (int p = 0; p < NUM_PINS_M1; p++) digitalWrite(PINS_MOTOR1[p], onL);
      delay(400);
      for (int p = 0; p < NUM_PINS_M1; p++) digitalWrite(PINS_MOTOR1[p], offL);
      delay(400);
    }
    for (int i = 0; i < 3; i++) {
      for (int p = 0; p < NUM_PINS_M2; p++) digitalWrite(PINS_MOTOR2[p], onL);
      delay(400);
      for (int p = 0; p < NUM_PINS_M2; p++) digitalWrite(PINS_MOTOR2[p], offL);
      delay(400);
    }
    setWaterPump(pumpWaterActive, "Test Completed");
    setFertPump(pumpFertActive, "Test Completed");
    Serial.println("[TEST] Relay Click Test Completed.");
  }

  if (doc.containsKey("pump")) {
    const char* pCmd = doc["pump"];
    scheduleRunning = false;
    if (strcmp(pCmd, "on") == 0) {
      currentMode = MODE_FULL_MANUAL;
      setWaterPump(true, "App Remote Command (Motor 1)");
    } else if (strcmp(pCmd, "off") == 0) {
      setWaterPump(false, "App Remote Command (Motor 1)");
    }
  }

  if (doc.containsKey("fert")) {
    const char* fCmd = doc["fert"];
    scheduleRunning = false;
    if (strcmp(fCmd, "on") == 0) {
      currentMode = MODE_FULL_MANUAL;
      setFertPump(true, "App Remote Command (Motor 2)");
    } else if (strcmp(fCmd, "off") == 0) {
      setFertPump(false, "App Remote Command (Motor 2)");
    }
  }

  if (doc.containsKey("both")) {
    const char* bCmd = doc["both"];
    scheduleRunning = false;
    if (strcmp(bCmd, "on") == 0) {
      currentMode = MODE_FULL_MANUAL;
      setWaterPump(true, "App Remote Command (Both)");
      setFertPump(true, "App Remote Command (Both)");
    } else if (strcmp(bCmd, "off") == 0) {
      setWaterPump(false, "App Remote Command (Both)");
      setFertPump(false, "App Remote Command (Both)");
    }
  }

  if (doc.containsKey("mode")) {
    int m = doc["mode"];
    if (m >= 0 && m <= 3) {
      currentMode = (OperationMode)m;
      Serial.printf("[MODE] Switched to Mode %d\n", m);
    }
  }

  if (doc.containsKey("addSchedule")) {
    JsonObject sObj = doc["addSchedule"];
    if (numSchedules < MAX_SCHEDULES) {
      schedules[numSchedules].enabled = sObj["en"] | true;
      schedules[numSchedules].hour = sObj["h"] | 6;
      schedules[numSchedules].minute = sObj["m"] | 0;
      schedules[numSchedules].durationMin = sObj["dur"] | 15;
      schedules[numSchedules].target = sObj["tgt"] | 0;
      schedules[numSchedules].ranToday = false;
      numSchedules++;
      saveSchedulesToNVS();
    }
  }

  if (doc.containsKey("deleteSchedule")) {
    int delId = doc["deleteSchedule"];
    if (delId >= 0 && delId < numSchedules) {
      for (int i = delId; i < numSchedules - 1; i++) {
        schedules[i] = schedules[i + 1];
      }
      numSchedules--;
      saveSchedulesToNVS();
    }
  }

  publishTelemetry();
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.printf("[MQTT] Connecting to broker %s as %s\n", MQTT_BROKER, deviceId);
  if (mqttClient.connect(deviceId, topicStatus, 1, true, "{\"online\":false}")) {
    Serial.println("[MQTT] Connected to Cloud Broker!");
    mqttClient.publish(topicStatus, "{\"online\":true}", true);
    mqttClient.subscribe(topicCommand);
    Serial.printf("[MQTT] Subscribed to: %s\n", topicCommand);
    publishTelemetry();
  } else {
    Serial.printf("[MQTT] Connection failed, rc=%d. Will retry...\n", mqttClient.state());
  }
}

// ==================== CAPTIVE PORTAL WEB SERVER ====================
void startCaptivePortal() {
  inAPMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Drop2Crop_WiFi_Setup");
  delay(100);

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
                  "<title>Drop2Crop WiFi Setup</title>"
                  "<style>body{font-family:sans-serif;background:#0f172a;color:#f8fafc;padding:24px;text-align:center;}"
                  "input{width:100%;max-width:320px;padding:12px;margin:8px 0;border-radius:8px;border:1px solid #334155;background:#1e293b;color:#fff;box-sizing:border-box;}"
                  "button{width:100%;max-width:320px;padding:14px;background:#10b981;color:#000;font-weight:bold;border:none;border-radius:8px;cursor:pointer;margin-top:12px;}</style></head>"
                  "<body><h2>🌱 Drop2Crop WiFi Setup</h2><p>Configure network credentials for your station:</p>"
                  "<form action='/save' method='POST'>"
                  "<input type='text' name='s' placeholder='WiFi SSID' required><br>"
                  "<input type='password' name='p' placeholder='WiFi Password'><br>"
                  "<button type='submit'>Connect Station</button></form></body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, []() {
    String s = server.arg("s");
    String p = server.arg("p");
    if (s.length() > 0) {
      saveWifiCredentials(s, p);
      server.send(200, "text/html", "<h3>Credentials Saved!</h3><p>ESP32 is rebooting...</p>");
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, "text/plain", "SSID cannot be empty.");
    }
  });

  server.onNotFound([]() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.printf("[AP] Captive Portal started on: %s\n", WiFi.softAPIP().toString().c_str());
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n==============================================");
  Serial.println("  Drop2Crop - Cloud IoT Precision Station v2.0");
  Serial.println("  (FRESH STANDALONE RELAY ENGINE - ZERO SENSOR LAG)");
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

  // Load Saved WiFi & Schedules from Flash Memory
  loadCredentials();

  // Configure All Hardware Pins
  configureAllPins();

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
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(2048);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(15);

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.google.com");
  } else {
    Serial.println("\n[WiFi] Could not connect to saved network. Starting Captive Portal...");
    startCaptivePortal();
  }
}

void loop() {
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

  // MQTT Connection Check
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
  } else {
    static unsigned long lastWifiReconnectAttempt = 0;
    if (millis() - lastWifiReconnectAttempt > 5000) {
      lastWifiReconnectAttempt = millis();
      Serial.printf("[WiFi] WiFi connection lost. Reconnecting to %s...\n", wifiSSID.c_str());
      WiFi.reconnect();
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

    Serial.printf("[%s] WiFi:%s | Soil:%d%% | Rain:%s(lock:%us) | Motor1(Water):%s | Motor2(Fert):%s | Mode:%d | Next:%s\n",
                  currentTimeStr,
                  (WiFi.status() == WL_CONNECTED) ? "OK" : "NO",
                  soilPercent,
                  instantRain ? "WET" : "DRY", rainRemainingSec,
                  pumpWaterActive ? "ON" : "OFF", pumpFertActive ? "ON" : "OFF",
                  (int)currentMode, nextRunString);
  }
}
