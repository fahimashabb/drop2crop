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
 *   - 5V Dual Relay Pump & Aux Valve Actuator (GPIO 17, GPIO 18 - Active LOW)
 *   - Mosquitto MQTT Cloud Bridge (test.mosquitto.org)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include "driver/gpio.h"

// ==================== PIN DEFINITIONS ====================
#define PIN_SOIL_ANALOG      4   // Capacitive Soil Moisture (ADC1_CH3)
#define PIN_RAIN_DIGITAL     5   // Rain Sensor Digital (DO)
#define PIN_RAIN_ANALOG      6   // Rain Sensor Analog (AO)

// Ultrasonic Reservoir Level Monitor (GPIO 7 & 8 - Standard Dedicated Ports)
#define PIN_TRIG             7   // HC-SR04 Trigger (GPIO 7)
#define PIN_ECHO             8   // HC-SR04 Echo (GPIO 8)

// Primary Clean Relays (GPIO 13 & 14 - Pure GPIOs, Free from JTAG, SPI Flash, Boot Strapping, and USB)
#define PIN_RELAY_PUMP       13  // Primary Relay 1: Clean Water Pump (Active LOW)
#define PIN_RELAY_VALVE      14  // Primary Relay 2: Aux/Fertigation (Active LOW)

// Mirrored Safe Relays (GPIO 21 & 47 - Driven in parallel for maximum hardware flexibility)
#define PIN_RELAY_PUMP_ALT   21  // Alternate Relay 1: Clean Water Pump
#define PIN_RELAY_VALVE_ALT  47  // Alternate Relay 2: Aux/Fertigation

#define PIN_BOOT_BUTTON      0   // Onboard BOOT Button for WiFi Reset

// ==================== CALIBRATION & THRESHOLDS ====================
const int SOIL_DRY_RAW   = 3150;
const int SOIL_WET_RAW   = 1350;

// Tank Level Calibration: 
// 0.0 cm = Full tank (sensor at 0 distance / test obstacle)
// 45.0 cm = Empty tank (sensor distance to bottom of empty reservoir)
float reservoirMaxDepthCm = 45.0; // Distance when tank is EMPTY
float reservoirMinDistCm  = 0.0;  // Calibrated zero-distance: 0 cm = 100% full

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
  uint8_t target;       // 0 = Motor 1 (Clean Water), 1 = Motor 2 (Fertigation), 2 = Both
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
  MODE_SCHEDULE_ONLY = 1, // Strictly runs on timer schedules
  MODE_ALERT_ONLY    = 2, // Sensor alerts only, manual control
  MODE_FULL_MANUAL   = 3  // Fully manual control
};

OperationMode currentMode = MODE_AUTO_IRRIGATE;

// ==================== NETWORKING & CAPTIVE PORTAL ====================
Preferences preferences;

String wifiSSID = "Dracarys";
String wifiPass = "itsanewform";
String devicePasscode = "2564"; // Default security PIN (last 4 digits of device MAC)

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

float waterDistCm = 0.0;
int waterTankPercent = 0;

bool pumpWaterActive = false; // Motor 1: Clean Water Irrigation Pump (Relay 1, GPIO 17)
bool pumpFertActive  = false; // Motor 2: Fertigation Nutrient Pump (Relay 2, GPIO 18)
bool safetyOverride  = false; // Safety Bypass: true = allow forcing motors ON/OFF regardless of water/rain
bool relayActiveLow  = true;  // Relay Trigger: true = Active LOW (0V triggers), false = Active HIGH (3.3V triggers)
bool ultrasonicEnabled = false; // Standalone Relay Mode: false = distance sensor detached, pins 8 & 7 driven as relays

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

void configurePins();

// ==================== SCHEDULE & WIFI STORAGE (NVS FLASH) ====================
void loadCredentials() {
  preferences.begin("d2c_cfg", false);
  wifiSSID = preferences.getString("ssid", "Dracarys");
  wifiPass = preferences.getString("pass", "itsanewform");
  devicePasscode = preferences.getString("pin", "2564");
  safetyOverride = preferences.getBool("safeBypass", false);
  relayActiveLow = preferences.getBool("relLow", true);
  ultrasonicEnabled = preferences.getBool("ultraEn", false); // Default: Standalone Mode (No ultrasonic sensor required)
  preferences.end();
  Serial.printf("[NVS] Loaded WiFi Config: SSID='%s'\n", wifiSSID.c_str());
  Serial.printf("[NVS] Station Security PIN configured: '%s'\n", devicePasscode.c_str());
  Serial.printf("[NVS] Safety Mechanism: %s\n", safetyOverride ? "BYPASSED (Force Override Active)" : "ENFORCED (Safeties Active)");
  Serial.printf("[NVS] Relay Trigger Polarity: Active %s\n", relayActiveLow ? "LOW (0V = ON)" : "HIGH (3.3V = ON)");
  Serial.printf("[NVS] Sensor Operation Mode: %s\n", ultrasonicEnabled ? "ULTRASONIC SENSOR ENABLED" : "STANDALONE RELAYS (No Ultrasonic Sensor)");

  preferences.begin("d2c_sched", false);
  numSchedules = preferences.getInt("count", 0);
  if (numSchedules > MAX_SCHEDULES) numSchedules = MAX_SCHEDULES;

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
    schedules[i].target = preferences.getUChar(keyTgt, 0); // 0=Water, 1=Fert, 2=Both
    schedules[i].ranToday = false;
  }
  preferences.end();

  if (numSchedules == 0) {
    schedules[0] = {true, 6, 0, 20, 0, false};   // 06:00 AM for 20 mins -> Motor 1 (Clean Water)
    schedules[1] = {true, 18, 0, 15, 1, false};  // 06:00 PM for 15 mins -> Motor 2 (Fertigation)
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

void savePasscode(const String& newPin) {
  preferences.begin("d2c_cfg", false);
  preferences.putString("pin", newPin);
  preferences.end();
  devicePasscode = newPin;
  Serial.printf("[SECURITY] Security PIN updated in Flash NVS: %s\n", devicePasscode.c_str());
}

void saveSafetyOverride(bool bypass) {
  safetyOverride = bypass;
  preferences.begin("d2c_cfg", false);
  preferences.putBool("safeBypass", safetyOverride);
  preferences.end();
  Serial.printf("[SAFETY] Safety Override updated in Flash NVS: %s\n", 
                safetyOverride ? "BYPASSED (Force Override Active)" : "ENFORCED (Safeties Active)");
}

void saveRelayTriggerMode(bool isLow) {
  relayActiveLow = isLow;
  preferences.begin("d2c_cfg", false);
  preferences.putBool("relLow", relayActiveLow);
  preferences.end();
  Serial.printf("[RELAY] Trigger Polarity updated in Flash NVS: Active %s\n", relayActiveLow ? "LOW" : "HIGH");
}

void configurePins() {
  int offLvl = relayActiveLow ? HIGH : LOW;

  // Release all candidate pins from JTAG/peripheral matrices for pure, clean GPIO control
  gpio_reset_pin((gpio_num_t)PIN_RELAY_PUMP);
  gpio_reset_pin((gpio_num_t)PIN_RELAY_VALVE);
  gpio_reset_pin((gpio_num_t)PIN_RELAY_PUMP_ALT);
  gpio_reset_pin((gpio_num_t)PIN_RELAY_VALVE_ALT);
  gpio_reset_pin((gpio_num_t)17);
  gpio_reset_pin((gpio_num_t)18);
  gpio_reset_pin((gpio_num_t)9);
  gpio_reset_pin((gpio_num_t)10);
  gpio_reset_pin((gpio_num_t)PIN_TRIG); // GPIO 7
  gpio_reset_pin((gpio_num_t)PIN_ECHO); // GPIO 8

  // Set maximum drive capability on all relay lines
  gpio_set_drive_capability((gpio_num_t)PIN_RELAY_PUMP, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_RELAY_VALVE, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_RELAY_PUMP_ALT, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_RELAY_VALVE_ALT, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)17, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)9, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)10, GPIO_DRIVE_CAP_3);

  digitalWrite(PIN_RELAY_PUMP, offLvl);
  digitalWrite(PIN_RELAY_VALVE, offLvl);
  digitalWrite(PIN_RELAY_PUMP_ALT, offLvl);
  digitalWrite(PIN_RELAY_VALVE_ALT, offLvl);
  digitalWrite(17, offLvl);
  digitalWrite(18, offLvl);
  digitalWrite(9, offLvl);
  digitalWrite(10, offLvl);

  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_RELAY_VALVE, OUTPUT);
  pinMode(PIN_RELAY_PUMP_ALT, OUTPUT);
  pinMode(PIN_RELAY_VALVE_ALT, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  if (ultrasonicEnabled) {
    pinMode(PIN_TRIG, OUTPUT);
    digitalWrite(PIN_TRIG, LOW);
    pinMode(PIN_ECHO, INPUT_PULLDOWN);
    Serial.println("[PINS] Ultrasonic Mode: GPIO 7 = Trig, GPIO 8 = Echo");
  } else {
    // Standalone Relay Mode: Pins 7 & 8 are safe OUTPUTS driven in parallel with relays!
    gpio_set_drive_capability((gpio_num_t)7, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)8, GPIO_DRIVE_CAP_3);
    digitalWrite(7, offLvl);
    digitalWrite(8, offLvl);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    Serial.println("[PINS] Standalone Relay Mode: GPIO 8 = Relay 1 (Water), GPIO 7 = Relay 2 (Fert)");
  }
}

void saveUltrasonicMode(bool en) {
  ultrasonicEnabled = en;
  preferences.begin("d2c_cfg", false);
  preferences.putBool("ultraEn", en);
  preferences.end();
  Serial.printf("[NVS] Ultrasonic Sensor Mode updated to: %s\n", en ? "ENABLED" : "DISABLED (Standalone Relays)");
  configurePins();
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

      <label for="pin">Security PIN (Optional - Default: 2564)</label>
      <input type="text" id="pin" name="pin" placeholder="Leave empty for current or set 4-digit PIN">
      
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
    String newPIN  = server.arg("pin");
    if (newSSID.length() > 0) {
      saveWifiCredentials(newSSID, newPass);
      if (newPIN.length() >= 4) {
        savePasscode(newPIN);
      }
      server.send(200, "text/html", "<html><body style='background:#0f172a;color:#10b981;font-family:sans-serif;text-align:center;padding:40px;'><h2>Settings Saved!</h2><p style='color:#fff;'>Rebooting and connecting to " + newSSID + "...</p></body></html>");
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

// ==================== ACTUATOR CONTROL (SINGLE CRISP CLICK - NO CHATTER) ====================
unsigned long lastWaterPumpSwitch = 0;
unsigned long lastFertPumpSwitch  = 0;
const unsigned long RELAY_DEBOUNCE_COOLDOWN_MS = 1500; // Minimum 1.5s between mechanical state changes

void setWaterPump(bool state, const char* reason = "Manual") {
  // Guard 1: If motor is ALREADY in requested state, DO NOTHING! Prevents fluctuating signals.
  if (pumpWaterActive == state) {
    return;
  }

  // Guard 2: Enforce anti-chatter cooldown so relay clicks only ONCE
  unsigned long now = millis();
  if (now - lastWaterPumpSwitch < RELAY_DEBOUNCE_COOLDOWN_MS && lastWaterPumpSwitch > 0) {
    Serial.printf("[DEBOUNCE] Water Pump transition to %s rejected: cooldown active (%lums remaining)\n",
                  state ? "ON" : "OFF", RELAY_DEBOUNCE_COOLDOWN_MS - (now - lastWaterPumpSwitch));
    return;
  }

  int offLevel = relayActiveLow ? HIGH : LOW;
  int onLevel  = relayActiveLow ? LOW : HIGH;

  // Rain lockout check ONLY for auto/schedule modes when safety is enforced (NEVER blocks manual)
  if (state && !safetyOverride && rainLocked && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.printf("[SAFETY BLOCK] Water Pump (%s) blocked: 24h Rain Lockout! (Safety Active)\n", reason);
    pumpWaterActive = false;
    digitalWrite(PIN_RELAY_PUMP, offLevel);
    digitalWrite(PIN_RELAY_PUMP_ALT, offLevel);
    digitalWrite(17, offLevel);
    digitalWrite(9, offLevel);
    if (!ultrasonicEnabled) digitalWrite(8, offLevel);
    return;
  }

  pumpWaterActive = state;
  lastWaterPumpSwitch = now;
  int drivePinLevel = pumpWaterActive ? onLevel : offLevel;
  digitalWrite(PIN_RELAY_PUMP, drivePinLevel);      // GPIO 13
  digitalWrite(PIN_RELAY_PUMP_ALT, drivePinLevel);  // GPIO 21
  digitalWrite(17, drivePinLevel);                  // GPIO 17
  digitalWrite(9, drivePinLevel);                   // GPIO 9
  if (!ultrasonicEnabled) {
    digitalWrite(8, drivePinLevel);                 // GPIO 8 (Mirrored Relay 1 in Standalone Mode)
  }
  Serial.printf("[RELAY 1] Motor 1 (Clean Water) -> %s (Pins 13, 17, 9, 21%s = %s, Trigger: %s%s)\n", 
                pumpWaterActive ? "ON" : "OFF", 
                !ultrasonicEnabled ? " & 8" : "",
                drivePinLevel == LOW ? "LOW" : "HIGH",
                reason, safetyOverride ? " [SAFETY BYPASS ACTIVE]" : "");
}

void setFertPump(bool state, const char* reason = "Manual") {
  // Guard 1: If motor is ALREADY in requested state, DO NOTHING!
  if (pumpFertActive == state) {
    return;
  }

  // Guard 2: Enforce anti-chatter cooldown so relay clicks only ONCE
  unsigned long now = millis();
  if (now - lastFertPumpSwitch < RELAY_DEBOUNCE_COOLDOWN_MS && lastFertPumpSwitch > 0) {
    Serial.printf("[DEBOUNCE] Fert Pump transition to %s rejected: cooldown active (%lums remaining)\n",
                  state ? "ON" : "OFF", RELAY_DEBOUNCE_COOLDOWN_MS - (now - lastFertPumpSwitch));
    return;
  }

  int offLevel = relayActiveLow ? HIGH : LOW;
  int onLevel  = relayActiveLow ? LOW : HIGH;

  if (state && !safetyOverride && rainLocked && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.printf("[SAFETY BLOCK] Fertigation Pump (%s) blocked: 24h Rain Lockout! (Safety Active)\n", reason);
    pumpFertActive = false;
    digitalWrite(PIN_RELAY_VALVE, offLevel);
    digitalWrite(PIN_RELAY_VALVE_ALT, offLevel);
    digitalWrite(18, offLevel);
    digitalWrite(10, offLevel);
    if (!ultrasonicEnabled) digitalWrite(7, offLevel);
    return;
  }

  pumpFertActive = state;
  lastFertPumpSwitch = now;
  int drivePinLevel = pumpFertActive ? onLevel : offLevel;
  digitalWrite(PIN_RELAY_VALVE, drivePinLevel);     // GPIO 14
  digitalWrite(PIN_RELAY_VALVE_ALT, drivePinLevel); // GPIO 47
  digitalWrite(18, drivePinLevel);                  // GPIO 18
  digitalWrite(10, drivePinLevel);                  // GPIO 10
  if (!ultrasonicEnabled) {
    digitalWrite(7, drivePinLevel);                 // GPIO 7 (Mirrored Relay 2 in Standalone Mode)
  }
  Serial.printf("[RELAY 2] Motor 2 (Fertigation) -> %s (Pins 14, 18, 10, 47%s = %s, Trigger: %s%s)\n", 
                pumpFertActive ? "ON" : "OFF", 
                !ultrasonicEnabled ? " & 7" : "",
                drivePinLevel == LOW ? "LOW" : "HIGH",
                reason, safetyOverride ? " [SAFETY BYPASS ACTIVE]" : "");
}

void setPump(bool state, const char* reason = "Manual") {
  setWaterPump(state, reason);
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

  // 3. HC-SR04 Ultrasonic Water Level (Only active when ultrasonicEnabled == true)
  if (ultrasonicEnabled) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(4);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long duration = pulseIn(PIN_ECHO, HIGH, 26000); // 26ms max (~4.5m)
    static bool firstDistSample = true;
    if (duration > 0) {
      float measuredCm = (duration * 0.0343f) / 2.0f;
      // Zero-Distance Calibration: When object is flush against transducer face (< 1.5 cm), clamp to 0.0 cm
      if (measuredCm < 1.5f) {
        measuredCm = 0.0f;
      }
      if (measuredCm > reservoirMaxDepthCm) {
        measuredCm = reservoirMaxDepthCm;
      }

      // Exponential Moving Average (EMA) to eliminate any jitter
      if (firstDistSample) {
        waterDistCm = measuredCm;
        firstDistSample = false;
      } else {
        waterDistCm = (0.35f * measuredCm) + (0.65f * waterDistCm);
      }
      if (waterDistCm < 0.2f) waterDistCm = 0.0f;

      float usableHeight = reservoirMaxDepthCm - reservoirMinDistCm; // 45.0 cm - 0.0 cm = 45.0 cm
      if (usableHeight > 0.0f) {
        float waterDepth = reservoirMaxDepthCm - waterDistCm;
        waterTankPercent = (int)round((waterDepth / usableHeight) * 100.0f);
        waterTankPercent = constrain(waterTankPercent, 0, 100);
      } else {
        waterTankPercent = 100;
      }
    } else {
      // Sensor disconnected / unplugged / timed out:
      firstDistSample = true;
      waterDistCm = 0.0f;
      waterTankPercent = 100;
    }
  } else {
    // Standalone Relay Mode: Distance sensor completely disabled
    // Zero pulsing, zero pulseIn, zero pin blocking, zero false triggers!
    waterDistCm = 0.0f;
    waterTankPercent = 100;
  }
}

// ==================== TIME & SCHEDULER ENGINE ====================
void updateLocalTime() {
  // Use non-blocking 10ms timeout to prevent freezing the main loop / MQTT keepalive
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
          Serial.printf("[TIMER TRIGGER] Starting Schedule %d: %02d:%02d for %d mins (Target Motor: %d)!\n",
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
  // Rain Lockout Safety Check (Auto and Schedule modes only)
  if (!safetyOverride && rainLocked && (pumpWaterActive || pumpFertActive) && (currentMode == MODE_AUTO_IRRIGATE || currentMode == MODE_SCHEDULE_ONLY)) {
    Serial.println("[SAFETY CUTOFF] Rain lockout active! Stopping both motors.");
    scheduleRunning = false;
    setWaterPump(false, "Rain Lockout");
    setFertPump(false, "Rain Lockout");
    return;
  }

  // Automatic Irrigation Logic (Strictly active ONLY in MODE_AUTO_IRRIGATE)
  // Completely decoupled from Ultrasonic Water Reservoir sensor!
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
  doc["waterCm"]  = serialized(String(waterDistCm, 1));
  doc["waterLow"] = (waterTankPercent < TANK_ALERT_THRESHOLD);
  doc["waterEmpty"] = (waterTankPercent < TANK_EMPTY_CUTOFF);

  doc["rainNow"]  = instantRain;
  doc["rainLock"] = rainLocked;
  doc["rainSec"]  = rainRemainingSec;

  doc["online"]   = true;
  doc["uptime"]   = (unsigned long)(millis() / 1000);
  doc["rssi"]     = WiFi.RSSI();
  doc["pump"]     = pumpWaterActive; // Motor 1: Clean Water
  doc["fert"]     = pumpFertActive;  // Motor 2: Fertigation Nutrient
  doc["mode"]     = (int)currentMode;
  doc["schedRun"] = scheduleRunning;
  doc["schedTgt"] = scheduleTarget;
  doc["nextRun"]  = nextRunString;
  doc["safeBypass"] = safetyOverride; // Safety Bypass: true = force mode active, false = safeties enforced
  doc["relLow"]     = relayActiveLow;  // Relay Trigger: true = Active LOW, false = Active HIGH
  doc["ultraEn"]    = ultrasonicEnabled; // Sensor Mode: true = Ultrasonic Active, false = Standalone Relays

  JsonArray arr = doc.createNestedArray("schedules");
  for (int i = 0; i < numSchedules; i++) {
    JsonObject s = arr.createNestedObject();
    s["id"]  = i;
    s["en"]  = schedules[i].enabled;
    s["h"]   = schedules[i].hour;
    s["m"]   = schedules[i].minute;
    s["dur"] = schedules[i].durationMin;
    s["tgt"] = schedules[i].target; // 0=Water, 1=Fert, 2=Both
  }

  char buffer[1536];
  size_t n = serializeJson(doc, buffer);
  bool ok = mqttClient.publish(topicTelemetry, buffer, n);
  if (!ok) {
    Serial.printf("[MQTT ERR] Telemetry publish failed! n=%u, bufSize=%u, state=%d\n", 
                  (unsigned int)n, mqttClient.getBufferSize(), mqttClient.state());
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

  // Security Verification: Every incoming control command MUST include matching "auth" PIN
  const char* authPin = doc["auth"] | "";
  if (strcmp(authPin, devicePasscode.c_str()) != 0) {
    Serial.printf("[SECURITY ALERT] Unauthorized command rejected! Provided auth '%s' does not match device PIN.\n", authPin);
    mqttClient.publish(topicStatus, "{\"security\":\"UNAUTHORIZED_COMMAND_REJECTED\"}");
    return;
  }

  // Handle Safety Mechanism Override Setting (Auth PIN already validated)
  if (doc.containsKey("setSafetyBypass") || doc.containsKey("safetyBypass") || doc.containsKey("safetyOverride")) {
    bool bypassVal = false;
    if (doc.containsKey("setSafetyBypass")) bypassVal = doc["setSafetyBypass"];
    else if (doc.containsKey("safetyBypass")) bypassVal = doc["safetyBypass"];
    else if (doc.containsKey("safetyOverride")) bypassVal = doc["safetyOverride"];
    saveSafetyOverride(bypassVal);
    publishTelemetry();
  }

  // Handle Relay Trigger Polarity (Active LOW vs Active HIGH)
  if (doc.containsKey("setRelayActiveLow") || doc.containsKey("relLow")) {
    bool isLow = true;
    if (doc.containsKey("setRelayActiveLow")) isLow = doc["setRelayActiveLow"];
    else if (doc.containsKey("relLow")) isLow = doc["relLow"];
    saveRelayTriggerMode(isLow);
    setWaterPump(pumpWaterActive, "Trigger Polarity Update");
    setFertPump(pumpFertActive, "Trigger Polarity Update");
    publishTelemetry();
  }

  // Handle Ultrasonic Sensor Mode Setting (Standalone Relays vs Ultrasonic Sensor)
  if (doc.containsKey("setUltra") || doc.containsKey("ultraEn")) {
    bool uVal = false;
    if (doc.containsKey("setUltra")) uVal = doc["setUltra"];
    else if (doc.containsKey("ultraEn")) uVal = doc["ultraEn"];
    saveUltrasonicMode(uVal);
    publishTelemetry();
  }

  // Mechanical Click Diagnostic Test: Pulses Relay 1 & Relay 2 to test coils
  if (doc.containsKey("clickTest")) {
    Serial.println("[TEST] Starting Relay Mechanical Click Diagnostic across all candidate pins...");
    for (int i = 0; i < 3; i++) {
      digitalWrite(PIN_RELAY_PUMP, relayActiveLow ? LOW : HIGH);
      digitalWrite(PIN_RELAY_PUMP_ALT, relayActiveLow ? LOW : HIGH);
      digitalWrite(17, relayActiveLow ? LOW : HIGH);
      digitalWrite(9, relayActiveLow ? LOW : HIGH);
      if (!ultrasonicEnabled) digitalWrite(8, relayActiveLow ? LOW : HIGH);
      delay(400);
      digitalWrite(PIN_RELAY_PUMP, relayActiveLow ? HIGH : LOW);
      digitalWrite(PIN_RELAY_PUMP_ALT, relayActiveLow ? HIGH : LOW);
      digitalWrite(17, relayActiveLow ? HIGH : LOW);
      digitalWrite(9, relayActiveLow ? HIGH : LOW);
      if (!ultrasonicEnabled) digitalWrite(8, relayActiveLow ? HIGH : LOW);
      delay(400);
    }
    for (int i = 0; i < 3; i++) {
      digitalWrite(PIN_RELAY_VALVE, relayActiveLow ? LOW : HIGH);
      digitalWrite(PIN_RELAY_VALVE_ALT, relayActiveLow ? LOW : HIGH);
      digitalWrite(18, relayActiveLow ? LOW : HIGH);
      digitalWrite(10, relayActiveLow ? LOW : HIGH);
      if (!ultrasonicEnabled) digitalWrite(7, relayActiveLow ? LOW : HIGH);
      delay(400);
      digitalWrite(PIN_RELAY_VALVE, relayActiveLow ? HIGH : LOW);
      digitalWrite(PIN_RELAY_VALVE_ALT, relayActiveLow ? HIGH : LOW);
      digitalWrite(18, relayActiveLow ? HIGH : LOW);
      digitalWrite(10, relayActiveLow ? HIGH : LOW);
      if (!ultrasonicEnabled) digitalWrite(7, relayActiveLow ? HIGH : LOW);
      delay(400);
    }
    setWaterPump(pumpWaterActive, "Test Completed");
    setFertPump(pumpFertActive, "Test Completed");
    Serial.println("[TEST] Relay Click Test Completed.");
  }

  if (doc.containsKey("force")) {
    bool forceVal = doc["force"];
    if (forceVal && !safetyOverride) {
      saveSafetyOverride(true);
    }
  }

  // Allow updating the PIN from the authenticated app
  if (doc.containsKey("newPin")) {
    const char* np = doc["newPin"];
    if (strlen(np) >= 4) {
      savePasscode(String(np));
      publishTelemetry();
      return;
    }
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
      schedules[numSchedules].target = s["tgt"] | 0; // 0=Water, 1=Fert, 2=Both
      schedules[numSchedules].ranToday = false;
      numSchedules++;
      saveSchedulesToNVS();
      Serial.printf("[COMMAND] Added schedule slot %d: %02d:%02d (%d min, target %d)\n",
                    numSchedules, schedules[numSchedules-1].hour,
                    schedules[numSchedules-1].minute, schedules[numSchedules-1].durationMin,
                    schedules[numSchedules-1].target);
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

  mqttClient.setBufferSize(2048);

  Serial.print("[MQTT] Connecting to broker ");
  Serial.print(MQTT_BROKER);
  Serial.print(" as ");
  Serial.println(deviceId);

  // Connect with Last Will and Testament (LWT)
  // When ESP32 loses power or unplugs, broker automatically broadcasts retained {"online":false}
  if (mqttClient.connect(deviceId, topicStatus, 0, true, "{\"online\":false}")) {
    Serial.println("[MQTT] Connected to Cloud Broker!");
    // Publish retained online status so web clients connecting later instantly see the online state
    mqttClient.publish(topicStatus, "{\"online\":true}", true);
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
  delay(300);
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

  // Load Saved WiFi & Schedules & Config from Flash Memory
  loadCredentials();

  // Initialize Pins
  pinMode(PIN_SOIL_ANALOG, INPUT);
  pinMode(PIN_RAIN_DIGITAL, INPUT_PULLUP);
  pinMode(PIN_RAIN_ANALOG, INPUT);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP); // Onboard BOOT button

  // Configure Relays & Sensor Pins according to active mode
  configurePins();

  setWaterPump(false, "System Boot");
  setFertPump(false, "System Boot");

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
    mqttClient.setBufferSize(2048);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(15);

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
  } else {
    // Auto-reconnect if WiFi disconnects
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

    Serial.printf("[%s] WiFi:%s | Soil:%d%% | Water:%d%%(%.1fcm) | Rain:%s(lock:%us) | Motor1(Water):%s | Motor2(Fert):%s | Mode:%d | Next:%s\n",
                  currentTimeStr,
                  (WiFi.status() == WL_CONNECTED) ? "OK" : "NO",
                  soilPercent, waterTankPercent, waterDistCm,
                  instantRain ? "WET" : "DRY", rainRemainingSec,
                  pumpWaterActive ? "ON" : "OFF", pumpFertActive ? "ON" : "OFF",
                  (int)currentMode, nextRunString);
  }
}
