// =====================================================================
//  Smart-Solar-Energy-Management-System" 
//  Smart Solar Energy Management & Power Monitor
//
//  CHANGES IN THIS VERSION (v7):
//   1) PHONE-LIKE BATTERY %  (the fix for "% rises by itself")
//        - While the battery is NOT charging, the displayed % can only go
//          DOWN or stay the same. It never rises just because a load was
//          switched off (voltage rebound).
//        - It can only go UP while real charging current is flowing from the
//          panel/charger (charge-side INA226 reads > 0.05 A).
//        - Reboot or the "Re-sync %" button re-reads the % from the measured
//          voltage (use it after charging the battery with an external charger).
//        - If the charge-side sensor is missing the ratchet is disabled (the
//          system cannot know whether it is charging).
//   2) DEMO BUTTONS (Manual Overrides section, next to the polarity buttons):
//        Demo NORMAL (3 loads) / Demo CRITICAL (2 loads) / Demo INTENSE (1 load)
//        force that level so you can show every step in the presentation.
//        "Demo OFF" returns to the real battery-based levels.
//   3) MORE GRAPHS: power, battery voltage (terminal vs resting), battery %
//      (displayed vs voltage-estimated), loads ON count, currents, energy.
//   4) 3 load levels (auto):  NORMAL = 3 loads, CRITICAL = 2 loads (Aux shed),
//      INTENSE = 1 load (LED only), with hysteresis + timing (see below).
//   5) Load-compensated resting voltage (internal resistance, serial r0.30).
//   6) Reverse leakage is never shown as solar production (clamped to 0 W).
//
//  Hotspot:  Name SolarDashboard   Password solar1234   http://192.168.4.1
//
//  SD MODULE WIRING (3.3 V module):
//   3V3 -> 3V3     GND -> GND     CS -> GPIO 5
//   SCK -> GPIO 18   MOSI -> GPIO 23   MISO -> GPIO 19
//  TIP: use a short, thick USB cable. A weak cable drops the 3V3 rail.
//
//  ARDUINO IDE NOTE: keep this file inside a folder with the same name
//  (SolarSentry_SteadyRelay_v7_FinalDemo/SolarSentry_SteadyRelay_v7_FinalDemo.ino)
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <esp_random.h>   // explicit include so esp_random() resolves on every core version
#include <INA226.h>       // Rob Tillaart's INA226 library
#include <Preferences.h>   // stores relay polarity in flash so it survives reboots

// ================= Wi-Fi hotspot (always on) =================
const char* ap_ssid     = "SolarDashboard";
const char* ap_password = "solar1234";
const int   AP_CHANNEL  = 1;          // 1, 6 or 11 are the usual clean 2.4 GHz channels
const int   AP_MAX_CONN = 4;          // how many devices can join at once
// Lower TX power = smaller current spike (helps weak USB cables / brown-outs).
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_15dBm;

bool isAccessPointMode = true;   // always true in this version
WebServer server(80);

const bool SIMULATION_MODE = false;   // using real sensors

// ================= Hardware Pins =================
const int PIN_SD_CS   = 5;
const int PIN_SD_SCK  = 18;
const int PIN_SD_MISO = 19;
const int PIN_SD_MOSI = 23;
const uint32_t SD_SPI_HZ = 1000000;   // 1 MHz = tolerant of long wires / noise

// Battery voltage sensor (analog divider) - feeds ESP32 ADC directly
const int VOLTAGE_SENSOR_PIN = 34;
float VOLTAGE_CALIBRATION = 5.0;      // tune against a multimeter (serial "c12.57")
// Battery % is mapped linearly between these two RESTING voltages.
const float BATT_EMPTY_V = 11.0f;     // 0 %
const float BATT_FULL_V  = 12.9f;     // 100 %

// Internal resistance of battery + wiring + connectors (ohms). Used to add
// back the voltage sag caused by the load. Tune with serial  r0.30
float BATT_INTERNAL_R = 0.20f;

// Charging is "real" only above this charge-side current (amps).
const float CHARGING_MIN_A = 0.05f;

// Relay 1 = LED load, Relay 2 = Fan load, Relay 3 = Aux load
const int RELAY1_PIN = 25;
const int RELAY2_PIN = 26;
const int RELAY3_PIN = 33;            // was 27 - moved because of a faulty breadboard section
const int RELAY_PINS[3] = { RELAY1_PIN, RELAY2_PIN, RELAY3_PIN };
const char* const RELAY_NAMES[3] = { "LED", "Fan", "Aux load" };

// RELAY POLARITY: flip live with serial 7/8/9 or the web buttons (saved to flash).
bool relayActiveLow[3] = { false, false, false };

bool relayState[3] = { false, false, false };
unsigned long relayLastChangeMs[3] = { 0, 0, 0 };

// Anti-chatter: in AUTO mode a relay may not change state again within this
// time. Manual buttons / serial commands / demo buttons ignore it.
const unsigned long MIN_RELAY_HOLD_MS = 5000;

Preferences prefs;

// Battery voltage source: true = load-side INA226 (accurate), false = analog divider on pin 34.
bool useInaForBatt = true;

void driveRelayPin(int i) {
  bool level = (relayState[i] != relayActiveLow[i]);
  digitalWrite(RELAY_PINS[i], level ? HIGH : LOW);
}

// Change a relay. Returns true only if the state really changed.
bool setRelayState(int i, bool on, bool force) {
  if (relayState[i] == on) return false;
  unsigned long now = millis();
  if (!force && (now - relayLastChangeMs[i]) < MIN_RELAY_HOLD_MS) return false;
  relayState[i] = on;
  relayLastChangeMs[i] = now;
  driveRelayPin(i);
  return true;
}

void loadPolarity() {
  uint8_t mask = prefs.getUChar("pol", 0);
  for (int i = 0; i < 3; i++) relayActiveLow[i] = (mask >> i) & 1;
}

void flipPolarity(int i) {
  relayActiveLow[i] = !relayActiveLow[i];
  uint8_t mask = 0;
  for (int k = 0; k < 3; k++) if (relayActiveLow[k]) mask |= (1 << k);
  prefs.putUChar("pol", mask);
  driveRelayPin(i);   // re-apply current state with the new polarity
  Serial.print("Relay ");
  Serial.print(i + 1);
  Serial.print(" polarity flipped -> ");
  Serial.println(relayActiveLow[i] ? "ACTIVE-LOW (ON = pin LOW)" : "ACTIVE-HIGH (ON = pin HIGH)");
}

bool sdAvailable = false;

// ================= INA226 #1: charging side =================
#define INA226_CHARGE_ADDR 0x40
INA226 inaCharge(INA226_CHARGE_ADDR);
bool inaChargeOk = false;

// ================= INA226 #2: load side =================
#define INA226_LOAD_ADDR 0x44
INA226 inaLoad(INA226_LOAD_ADDR);
bool inaLoadOk = false;

// Periodic re-detection of missing sensors.
unsigned long lastSensorRescanMs = 0;
const unsigned long RESCAN_INTERVAL_MS = 10000;

// ================= Simple exponential moving average filter =================
float emaFilter(float previous, float fresh, float alpha) {
  if (previous == 0.0f) return fresh;   // first sample: no history yet, snap to it
  return previous + alpha * (fresh - previous);
}

// ================= System state (3 levels) =================
//   NORMAL   : 3 loads ON   (LED + Fan + Aux)
//   CRITICAL : 2 loads ON   (LED + Fan)
//   INTENSE  : 1 load ON    (LED only)
// Higher number = worse level.
enum SysState { STATE_NORMAL = 0, STATE_CRITICAL = 1, STATE_INTENSE = 2 };

const char* stateName(int s) {
  switch (s) {
    case STATE_NORMAL:   return "NORMAL";
    case STATE_CRITICAL: return "CRITICAL";
    case STATE_INTENSE:  return "INTENSE";
  }
  return "NORMAL";
}

struct Reading {
  float solarVoltage = 17.2, solarCurrent = 0.51, solarPower = 8.7;
  float batteryVoltage = 12.6, batteryPercent = 78;
  float chargingPower = 6.4;
  float loadPower = 3.6, loadCurrent = 0.30;
  float energyConsumedWh = 12.8, energyChargedWh = 24.5, dailySolarWh = 0.0;
  float runtimeHours = 8.33;
  bool ledOn = true, fanOn = true, auxOn = true;
  bool autoMode = true;
  SysState sysState = STATE_NORMAL;
  // Panel voltage below battery voltage: current leaks backward through the
  // panel (INA226 reads negative). It is clamped to 0 and never shown as solar.
  bool reverseFlow = false;
} r;

// ---- Battery % tracking (phone-like) and demo mode ----
static float emaRestV = 0;       // load-compensated RESTING battery voltage (slow filter)
float estPercent = 78;           // % estimated from resting voltage (can wander up/down)
bool  pctSeeded = false;         // false -> next sample seeds r.batteryPercent from estPercent
bool  isCharging = false;        // true only while real charging current flows
int   demoLevel = -1;            // -1 = off; 0/1/2 = forced level for the presentation

String lastAlert = "System boot - all sensors nominal";
unsigned long lastSampleMs = 0;
const unsigned long SAMPLE_INTERVAL_MS = 2000;
// Set to your REAL battery energy: Wh = volts x amp-hours (12 V x 7 Ah = 84 Wh).
const float BATTERY_CAPACITY_WH = 40.0;

unsigned long lastDayResetMs = 0;
const unsigned long DAY_MS = 24UL * 3600UL * 1000UL;

float wander(float value, float minV, float maxV, float step) {
  value += (random(-100, 101) / 100.0) * step;
  if (value < minV) value = minV;
  if (value > maxV) value = maxV;
  return value;
}

// ================= Wi-Fi hotspot start =================
bool startHotspot() {
  WiFi.persistent(false);        // never write Wi-Fi settings to flash
  WiFi.disconnect(true);         // drop any old station connection / config
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_AP);
  delay(100);

  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));

  bool ok = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    ok = WiFi.softAP(ap_ssid, ap_password, AP_CHANNEL, 0, AP_MAX_CONN);
    if (ok) break;
    Serial.print("softAP failed, attempt ");
    Serial.println(attempt);
    delay(500);
  }

  if (ok) {
    delay(200);
    WiFi.setTxPower(WIFI_TX_POWER);
    WiFi.setSleep(false);   // Wi-Fi power-save adds 100-300 ms lag to every button press
    isAccessPointMode = true;
    Serial.println("Hotspot started.");
    Serial.print("Wi-Fi name: ");  Serial.println(ap_ssid);
    Serial.print("Password:   ");  Serial.println(ap_password);
    Serial.print("Open: http://"); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("HOTSPOT FAILED TO START! (check power supply / USB cable)");
  }
  return ok;
}

// ================= INA226 init =================
void initINA226() {
  if (!inaCharge.begin()) {
    Serial.println("INA226 (charge side, 0x40) NOT FOUND!");
    inaChargeOk = false;
  } else {
    inaChargeOk = true;
    Serial.println("INA226 (charge side, 0x40) CONNECTED.");
    int result = inaCharge.setMaxCurrentShunt(0.8, 0.1);
    Serial.print("Charge-side calibration result: ");
    Serial.println(result);
    if (result != 0) {
      Serial.println("WARNING: charge-side INA226 calibration failed! Disabling sensor.");
      inaChargeOk = false;
    }
  }

  if (!inaLoad.begin()) {
    Serial.println("INA226 (load side, 0x44) NOT FOUND!");
    inaLoadOk = false;
  } else {
    inaLoadOk = true;
    Serial.println("INA226 (load side, 0x44) CONNECTED.");
    int result = inaLoad.setMaxCurrentShunt(0.8, 0.1);
    Serial.print("Load-side calibration result: ");
    Serial.println(result);
    if (result != 0) {
      Serial.println("WARNING: load-side INA226 calibration failed! Disabling sensor.");
      inaLoadOk = false;
    }
  }
}

void rescanMissingSensors() {
  if (!inaChargeOk) {
    if (inaCharge.begin() && inaCharge.setMaxCurrentShunt(0.8, 0.1) == 0) {
      inaChargeOk = true;
      Serial.println("Charge-side INA226 (0x40) came back online.");
      lastAlert = "Charge-side sensor reconnected";
    }
  }
  if (!inaLoadOk) {
    if (inaLoad.begin() && inaLoad.setMaxCurrentShunt(0.8, 0.1) == 0) {
      inaLoadOk = true;
      Serial.println("Load-side INA226 (0x44) came back online.");
      lastAlert = "Load-side sensor reconnected";
    }
  }
}

// ================= SD card init (explicit pins, slow speed, retries) =================
void initSDCard() {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  delay(300);

  sdAvailable = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    if (SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ)) {
      sdAvailable = true;
      break;
    }
    Serial.print("SD mount failed, attempt ");
    Serial.println(attempt);
    SD.end();
    delay(500);
  }

  if (!sdAvailable) {
    Serial.println("SD Card Mount Failed! (check wiring, 3V3 power / USB cable, FAT format)");
    return;
  }
  Serial.println("SD Card Initialized successfully.");

  if (!SD.exists("/log.csv")) {
    File logFile = SD.open("/log.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("Millis,Solar_V,Solar_A,Solar_W,Batt_V,Batt_Pct,Load_W,State");
      logFile.close();
    }
  }
}

void logToSD() {
  if (!sdAvailable) return;
  File logFile = SD.open("/log.csv", FILE_APPEND);
  if (logFile) {
    logFile.print(millis());                  logFile.print(",");
    logFile.print(r.solarVoltage, 1);         logFile.print(",");
    logFile.print(r.solarCurrent, 2);         logFile.print(",");
    logFile.print(r.solarPower, 1);           logFile.print(",");
    logFile.print(r.batteryVoltage, 2);       logFile.print(",");
    logFile.print(r.batteryPercent, 0);       logFile.print(",");
    logFile.print(r.loadPower, 1);            logFile.print(",");
    logFile.println(stateName(r.sysState));
    logFile.close();
  }
}

// ---- State-machine timing and thresholds ----
const unsigned long DEGRADE_DELAY_MS = 10000;   // going worse: must hold 10 s
const unsigned long RECOVER_DELAY_MS = 60000;   // getting better: must hold 60 s

const float CRITICAL_ENTER_PCT = 50.0f;   // NORMAL   -> CRITICAL  (below 50 %)
const float NORMAL_RETURN_PCT  = 60.0f;   // CRITICAL -> NORMAL    (at or above 60 %)
const float INTENSE_ENTER_PCT  = 30.0f;   // any      -> INTENSE   (below 30 %)
const float INTENSE_EXIT_PCT   = 40.0f;   // INTENSE  -> CRITICAL  (at or above 40 %)

SysState pendingState = STATE_NORMAL;
bool pendingActive = false;
unsigned long pendingSinceMs = 0;
bool stateSeeded = false;

// Which state WOULD we be in right now (no timing). Recovery is one level at a time.
int evaluateTarget() {
  float p = r.batteryPercent;
  switch (r.sysState) {
    case STATE_NORMAL:
      if (p < INTENSE_ENTER_PCT)  return STATE_INTENSE;
      if (p < CRITICAL_ENTER_PCT) return STATE_CRITICAL;
      return STATE_NORMAL;
    case STATE_CRITICAL:
      if (p < INTENSE_ENTER_PCT)  return STATE_INTENSE;
      if (p >= NORMAL_RETURN_PCT) return STATE_NORMAL;
      return STATE_CRITICAL;
    case STATE_INTENSE:
      if (p >= INTENSE_EXIT_PCT)  return STATE_CRITICAL;
      return STATE_INTENSE;
  }
  return r.sysState;
}

void announceState(int s) {
  if (s == STATE_INTENSE)       lastAlert = "INTENSE: only LED (1 load) permitted";
  else if (s == STATE_CRITICAL) lastAlert = "CRITICAL: Aux shed, LED + Fan on (2 loads)";
  else                          lastAlert = "NORMAL: all 3 loads permitted";
}

void updateSystemState() {
  unsigned long now = millis();

  // DEMO: the level is forced by the dashboard buttons.
  if (demoLevel >= 0) {
    r.sysState = (SysState)demoLevel;
    pendingActive = false;
    stateSeeded = true;
    return;
  }

  // First sample after boot: jump straight to the right state.
  if (!stateSeeded) {
    stateSeeded = true;
    if (r.batteryPercent < INTENSE_ENTER_PCT)       r.sysState = STATE_INTENSE;
    else if (r.batteryPercent < CRITICAL_ENTER_PCT) r.sysState = STATE_CRITICAL;
    else                                            r.sysState = STATE_NORMAL;
    announceState(r.sysState);
    return;
  }

  SysState target = (SysState)evaluateTarget();
  if (target == r.sysState) { pendingActive = false; return; }

  if (!pendingActive || pendingState != target) {   // condition just appeared: start timing
    pendingActive = true;
    pendingState = target;
    pendingSinceMs = now;
    return;
  }

  bool degrading = ((int)target > (int)r.sysState);
  unsigned long needed = degrading ? DEGRADE_DELAY_MS : RECOVER_DELAY_MS;
  if (now - pendingSinceMs >= needed) {
    r.sysState = target;
    pendingActive = false;
    announceState(target);
  }
}

// force=true bypasses the relay hold timer (mode switch, demo buttons).
void applyLoadLogic(bool force) {
  if (r.autoMode) {
    bool fanPermit = (r.sysState != STATE_INTENSE);   // NORMAL + CRITICAL
    bool auxPermit = (r.sysState == STATE_NORMAL);    // NORMAL only
    setRelayState(0, true, force);                    // LED = essential load, always on in auto
    if (setRelayState(1, fanPermit, force) && !fanPermit && demoLevel < 0)
      lastAlert = "INTENSE: Fan disconnected (battery " + String(r.batteryPercent, 0) + "%)";
    if (setRelayState(2, auxPermit, force) && !auxPermit && demoLevel < 0)
      lastAlert = "Aux load disconnected (battery " + String(r.batteryPercent, 0) + "%)";
  }
  for (int i = 0; i < 3; i++) driveRelayPin(i);
  r.ledOn = relayState[0];
  r.fanOn = relayState[1];
  r.auxOn = relayState[2];
}

void updateEnergyAndRuntime(float dtMs) {
  float hours = dtMs / 3600000.0f;
  r.energyConsumedWh += r.loadPower * hours;
  if (r.chargingPower > 0) r.energyChargedWh += r.chargingPower * hours;

  r.dailySolarWh += (r.chargingPower > 0 ? r.chargingPower * hours : 0.0f);
  if (millis() - lastDayResetMs >= DAY_MS) {
    r.dailySolarWh = 0.0f;
    lastDayResetMs = millis();
  }

  if (r.loadPower > 0.05f) {
    r.runtimeHours = (r.batteryPercent / 100.0f * BATTERY_CAPACITY_WH) / r.loadPower;
    if (r.runtimeHours > 99.0f) r.runtimeHours = 99.0f;
  } else {
    r.runtimeHours = 99.0f;
  }
}

void sampleSimulated(float dtMs) {
  r.solarVoltage = wander(r.solarVoltage, 15.0, 19.5, 0.3);
  r.solarCurrent = wander(r.solarCurrent, 0.1, 0.9, 0.03);
  r.solarPower   = r.solarVoltage * r.solarCurrent;

  r.loadCurrent = wander(r.loadCurrent, 0.15, 0.5, 0.02);
  r.loadPower   = r.batteryVoltage * r.loadCurrent;

  r.chargingPower = max(0.0f, r.solarPower - r.loadPower * 0.2f);
  r.reverseFlow = false;

  float netPowerW = r.chargingPower - r.loadPower;
  float hours = dtMs / 3600000.0f;
  r.batteryPercent += (netPowerW * hours / BATTERY_CAPACITY_WH) * 100.0f;
  if (r.batteryPercent < 0)   r.batteryPercent = 0;
  if (r.batteryPercent > 100) r.batteryPercent = 100;

  r.batteryVoltage = 11.0f + (r.batteryPercent / 100.0f) * 1.9f;
  emaRestV = r.batteryVoltage;
  estPercent = r.batteryPercent;
  isCharging = true;
  pctSeeded = true;

  updateEnergyAndRuntime(dtMs);
}

// EMA state kept between calls (0 = "no history yet", handled inside emaFilter)
static float emaBattV = 0, emaSolarV = 0, emaSolarA = 0, emaLoadA = 0;
static float lastSensorOutV = 0;     // volts seen on pin 34
static float lastAnalogBattV = 0;    // battery volts computed from the analog divider
static float lastInaBusV = 0;        // battery volts read by the load-side INA226
static bool  battFromIna = false;    // which source is feeding the battery value right now
static bool  prevReverseFlow = false;

// Median-style ADC read in MILLIVOLTS (31 samples, average the middle 15).
float readBattAdcMedian() {
  const int N = 31;
  int s[N];
  for (int i = 0; i < N; i++) {
    s[i] = analogReadMilliVolts(VOLTAGE_SENSOR_PIN);
    delayMicroseconds(200);
  }
  for (int i = 1; i < N; i++) {
    int k = s[i], j = i - 1;
    while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
    s[j + 1] = k;
  }
  long sum = 0;
  for (int i = 8; i < 23; i++) sum += s[i];
  return sum / 15.0f;
}

void sampleReal(float dtMs) {
  // --- Battery terminal voltage ---
  float rawMv = readBattAdcMedian();
  float sensorOutputVoltage = rawMv / 1000.0f;
  lastSensorOutV = sensorOutputVoltage;
  lastAnalogBattV = sensorOutputVoltage * VOLTAGE_CALIBRATION;

  lastInaBusV = inaLoadOk ? inaLoad.getBusVoltage() : 0.0f;
  bool inaUsable = inaLoadOk && lastInaBusV > 5.0f && lastInaBusV < 20.0f;

  bool useIna = useInaForBatt && inaUsable;
  if (useIna != battFromIna) {      // source changed: restart the smoothing filters
    battFromIna = useIna;
    emaBattV = 0;
    emaRestV = 0;
  }
  float rawBattV = useIna ? lastInaBusV : lastAnalogBattV;
  emaBattV = emaFilter(emaBattV, rawBattV, 0.25f);
  r.batteryVoltage = emaBattV;      // terminal voltage

  // --- Charge side: INA226 #1 (0x40) ---
  if (inaChargeOk) {
    float rawSolarV = inaCharge.getBusVoltage();
    float rawSolarA = inaCharge.getCurrent_mA() / 1000.0f;
    emaSolarV = emaFilter(emaSolarV, rawSolarV, 0.25f);
    emaSolarA = emaFilter(emaSolarA, rawSolarA, 0.25f);

    r.reverseFlow = (emaSolarA < -0.01f);

    r.solarVoltage  = emaSolarV;
    r.solarCurrent  = r.reverseFlow ? 0.0f : emaSolarA;
    r.solarPower    = r.solarVoltage * r.solarCurrent;
    r.chargingPower = r.reverseFlow ? 0.0f : (r.batteryVoltage * r.solarCurrent);

    // Announce only when the leak STARTS, so it does not overwrite other alerts every 2 s.
    if (r.reverseFlow && !prevReverseFlow) {
      lastAlert = "Panel voltage below battery voltage - not charging (add Schottky blocking diode)";
    }
    prevReverseFlow = r.reverseFlow;
  } else {
    r.solarVoltage = 0;
    r.solarCurrent = 0;
    r.solarPower = 0;
    r.chargingPower = 0;
    r.reverseFlow = false;
    prevReverseFlow = false;
  }

  // --- Load side: INA226 #2 (0x44) ---
  if (inaLoadOk) {
    float rawLoadA = inaLoad.getCurrent_mA() / 1000.0f;
    emaLoadA = emaFilter(emaLoadA, rawLoadA, 0.25f);
    r.loadCurrent = max(0.0f, emaLoadA);
    r.loadPower   = r.batteryVoltage * r.loadCurrent;
  } else {
    r.loadCurrent = 0;
    r.loadPower = 0;
  }

  // --- Resting voltage (load-compensated) -> estimated % ---
  float netChargeA = r.solarCurrent - r.loadCurrent;           // + charging, - discharging
  float vRest = r.batteryVoltage - netChargeA * BATT_INTERNAL_R;
  emaRestV = emaFilter(emaRestV, vRest, 0.05f);                // slow filter

  estPercent = (emaRestV - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
  if (estPercent < 0)   estPercent = 0;
  if (estPercent > 100) estPercent = 100;

  // --- PHONE-LIKE DISPLAYED % ---
  // Charging (real current from the panel/charger): % may follow the estimate up or down.
  // Not charging: % can only go DOWN or hold. It never rises on its own.
  // If the charge sensor is missing we cannot tell, so we follow the estimate.
  isCharging = (!inaChargeOk) || (!r.reverseFlow && r.solarCurrent > CHARGING_MIN_A);

  if (!pctSeeded) {
    r.batteryPercent = estPercent;
    pctSeeded = true;
  } else if (isCharging) {
    r.batteryPercent = estPercent;
  } else if (estPercent < r.batteryPercent) {
    r.batteryPercent = estPercent;
  }

  updateEnergyAndRuntime(dtMs);
}

void printStatusLine() {
  Serial.print("[");
  Serial.print(r.autoMode ? "AUTO" : "MANUAL");
  if (demoLevel >= 0) Serial.print("/DEMO");
  Serial.print("] ");
  Serial.print(stateName(r.sysState));
  Serial.print(" | batt ");
  Serial.print(r.batteryVoltage, 2);
  Serial.print(" V (");
  Serial.print(r.batteryPercent, 0);
  Serial.print("% shown, ");
  Serial.print(estPercent, 0);
  Serial.print("% est, ");
  Serial.print(isCharging ? "CHARGING" : "not charging");
  Serial.print(", src ");
  Serial.print(battFromIna ? "INA226" : "analog");
  Serial.print(") | R1 ");
  Serial.print(relayState[0] ? "ON" : "OFF");
  Serial.print("  R2 ");
  Serial.print(relayState[1] ? "ON" : "OFF");
  Serial.print("  R3 ");
  Serial.print(relayState[2] ? "ON" : "OFF");
  Serial.print(" | Wi-Fi clients: ");
  Serial.println(WiFi.softAPgetStationNum());
  if (!SIMULATION_MODE && !battFromIna && lastSensorOutV < 0.05f)
    Serial.println("   !! Pin 34 reads ~0 V: voltage sensor not connected / wrong pin -> battery shows 0% and loads shed");
  else if (demoLevel >= 0)
    Serial.println("   -> DEMO level forced from the dashboard. Press 'Demo OFF' to return to real battery levels.");
  else if (r.autoMode && r.sysState == STATE_CRITICAL)
    Serial.println("   -> CRITICAL: Aux held OFF by battery protection (needs 60% to return to NORMAL).");
  else if (r.autoMode && r.sysState == STATE_INTENSE)
    Serial.println("   -> INTENSE: Fan + Aux held OFF by battery protection (needs 40% to return to CRITICAL).");
}

unsigned long lastLogMs = 0;
const unsigned long LOG_INTERVAL_MS = 10000;

void sample(float dtMs) {
  if (SIMULATION_MODE) sampleSimulated(dtMs); else sampleReal(dtMs);
  updateSystemState();
  applyLoadLogic(false);
  if (millis() - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = millis();
    logToSD();
    printStatusLine();
  }
}

String jsonState() {
  int rh = (int)r.runtimeHours;
  int rm = (int)((r.runtimeHours - rh) * 60);

  String ip = WiFi.softAPIP().toString();

  String j;
  j.reserve(960);
  j = "{";
  j += "\"solarPower\":"     + String(r.solarPower, 1) + ",";
  j += "\"solarVoltage\":"   + String(r.solarVoltage, 1) + ",";
  j += "\"solarCurrent\":"   + String(r.solarCurrent, 2) + ",";
  j += "\"batteryVoltage\":" + String(r.batteryVoltage, 2) + ",";
  j += "\"restVoltage\":"    + String(emaRestV, 2) + ",";
  j += "\"batteryPercent\":" + String(r.batteryPercent, 0) + ",";
  j += "\"estPercent\":"     + String(estPercent, 0) + ",";
  j += "\"charging\":"       + String(isCharging ? "true" : "false") + ",";
  j += "\"demo\":"           + String(demoLevel >= 0 ? "true" : "false") + ",";
  j += "\"chargingPower\":"  + String(r.chargingPower, 1) + ",";
  j += "\"loadPower\":"      + String(r.loadPower, 1) + ",";
  j += "\"loadCurrent\":"    + String(r.loadCurrent, 2) + ",";
  j += "\"energyConsumed\":" + String(r.energyConsumedWh, 1) + ",";
  j += "\"energyCharged\":"  + String(r.energyChargedWh, 1) + ",";
  j += "\"dailySolar\":"     + String(r.dailySolarWh, 1) + ",";
  j += "\"runtimeH\":"       + String(rh) + ",";
  j += "\"runtimeM\":"       + String(rm) + ",";
  j += "\"ledOn\":"          + String(r.ledOn ? "true" : "false") + ",";
  j += "\"fanOn\":"          + String(r.fanOn ? "true" : "false") + ",";
  j += "\"auxOn\":"          + String(r.auxOn ? "true" : "false") + ",";
  j += "\"autoMode\":"       + String(r.autoMode ? "true" : "false") + ",";
  j += "\"sysState\":\""     + String(stateName(r.sysState)) + "\",";
  j += "\"alert\":\""        + lastAlert + "\",";
  j += "\"simulated\":"      + String(SIMULATION_MODE ? "true" : "false") + ",";
  j += "\"sdReady\":"        + String(sdAvailable ? "true" : "false") + ",";
  j += "\"chargeSensorOk\":" + String(inaChargeOk ? "true" : "false") + ",";
  j += "\"loadSensorOk\":"   + String(inaLoadOk ? "true" : "false") + ",";
  j += "\"reverseFlow\":"    + String(r.reverseFlow ? "true" : "false") + ",";
  j += "\"ip\":\""           + ip + "\"";
  j += "}";
  return j;
}

void handleData() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonState());
}

// /raw -> SHOW the log in the browser as plain CSV text
void handleLogs() {
  if (!sdAvailable) {
    server.send(404, "text/plain", "SD Card Not Detected");
    return;
  }
  File logFile = SD.open("/log.csv", FILE_READ);
  if (!logFile) {
    server.send(404, "text/plain", "Log File Missing");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(logFile, "text/plain");
  logFile.close();
}

// /download -> save log.csv as a file
void handleDownload() {
  if (!sdAvailable) {
    server.send(404, "text/plain", "SD Card Not Detected");
    return;
  }
  File logFile = SD.open("/log.csv", FILE_READ);
  if (!logFile) {
    server.send(404, "text/plain", "Log File Missing");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=log.csv");
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(logFile, "text/csv");
  logFile.close();
}

void handleControl() {
  bool modeChanged = false;
  if (server.hasArg("mode")) {
    bool wantAuto = (server.arg("mode") == "auto");
    if (!wantAuto && demoLevel >= 0) {   // leaving auto ends the demo
      demoLevel = -1;
      stateSeeded = false;
    }
    if (wantAuto != r.autoMode) { r.autoMode = wantAuto; modeChanged = true; }
  }
  if (!r.autoMode) {
    if (server.hasArg("led")) setRelayState(0, server.arg("led") == "on", true);
    if (server.hasArg("fan")) setRelayState(1, server.arg("fan") == "on", true);
    if (server.hasArg("aux")) setRelayState(2, server.arg("aux") == "on", true);
  }
  applyLoadLogic(modeChanged);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonState());
}

// /polarity?relay=1|2|3  -> flips that channel's polarity (saved to flash)
void handlePolarity() {
  if (server.hasArg("relay")) {
    int n = server.arg("relay").toInt();
    if (n >= 1 && n <= 3) flipPolarity(n - 1);
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonState());
}

// /demo?level=0|1|2|off  -> force a load level for the presentation (0 NORMAL, 1 CRITICAL, 2 INTENSE)
void handleDemo() {
  if (server.hasArg("level")) {
    String lv = server.arg("level");
    if (lv == "off") {
      demoLevel = -1;
      stateSeeded = false;     // re-pick the level from the real battery %
      updateSystemState();
      lastAlert = "Demo OFF - back to real battery-based levels";
    } else {
      int n = lv.toInt();
      if (n >= 0 && n <= 2) {
        demoLevel = n;
        r.autoMode = true;     // levels only drive the relays in auto mode
        r.sysState = (SysState)n;
        pendingActive = false;
        stateSeeded = true;
        announceState(n);
        lastAlert = "DEMO - " + lastAlert;
      }
    }
  }
  applyLoadLogic(true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonState());
}

// /resync -> set the displayed % from the measured (compensated) voltage.
// Use after charging the battery with an external charger.
void handleResync() {
  r.batteryPercent = estPercent;
  pctSeeded = true;
  if (demoLevel < 0) {
    stateSeeded = false;
    updateSystemState();
  }
  lastAlert = "Battery % re-synced to measured voltage (" + String(r.batteryPercent, 0) + "%)";
  applyLoadLogic(true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonState());
}

// ================= Web page (stored in flash, not copied to RAM) =================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SolarSentry Telemetry Grid</title>
<style>
:root {
  --bg: #f8fafc; --panel: #ffffff; --panel-border: #e2e8f0;
  --text: #0f172a; --muted: #64748b; --cyan: #0284c7; --amber: #d97706;
  --teal: #0d9488; --red: #dc2626; --blue: #2563eb; --chart-grid: #e2e8f0;
  --font-mono: 'Courier New', monospace;
  --font-sans: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
}

[data-theme="dark"] {
  --bg: #0f172a; --panel: #1e293b; --panel-border: #334155;
  --text: #f8fafc; --muted: #94a3b8; --cyan: #38bdf8; --amber: #f59e0b;
  --teal: #14b8a6; --red: #ef4444; --blue: #3b82f6; --chart-grid: #334155;
}

* { box-sizing: border-box; }
body {
  margin: 0; background: var(--bg);
  color: var(--text); font-family: var(--font-sans); padding: 24px 16px 60px; min-height: 100vh;
  transition: background-color 0.3s, color 0.3s;
}
.wrap { max-width: 1100px; margin: 0 auto; }
header {
  display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid var(--panel-border);
  padding-bottom: 16px; margin-bottom: 24px; flex-wrap: wrap; gap: 10px;
}
header h1 { font-size: 24px; margin: 0; font-weight: 700; color: var(--text); }
header p { margin: 4px 0 0; color: var(--muted); font-size: 13px; }
.header-right { display: flex; align-items: center; gap: 12px; }
.badge {
  display: flex; align-items: center; gap: 8px; font-family: var(--font-mono); font-size: 12px; font-weight:700;
  padding: 6px 14px; border-radius: 20px; border: 1px solid currentColor; background: var(--panel);
}
.badge.NORMAL { color: var(--teal); }
.badge.CRITICAL { color: var(--amber); }
.badge.INTENSE { color: var(--red); }
.dot { width: 8px; height: 8px; border-radius: 50%; background: currentColor; }

.theme-toggle-btn {
  background: var(--panel); color: var(--text); border: 1px solid var(--panel-border);
  padding: 6px 14px; border-radius: 20px; font-size: 12px; font-weight: 700; cursor: pointer;
  display: flex; align-items: center; gap: 6px; font-family: var(--font-sans); transition: all 0.2s;
}
.theme-toggle-btn:hover { border-color: var(--cyan); }

.flow {
  display: grid; grid-template-columns: 1fr auto 1fr auto 1fr; align-items: center; background: var(--panel);
  border: 1px solid var(--panel-border); border-radius: 12px; padding: 24px 16px; margin-bottom: 20px;
  box-shadow: 0 4px 6px -1px rgba(0,0,0,0.05);
}
.flow .stage { text-align: center; }
.flow .stage .num { font-family: var(--font-mono); font-size: 32px; font-weight: 700; }
.flow .stage .label { font-size: 11px; color: var(--muted); margin-top: 4px; text-transform: uppercase; font-weight: 600; }
.flow .stage.in .num { color: var(--amber); }
.flow .stage.out .num { color: var(--teal); }
.arrow { color: var(--muted); font-size: 24px; text-align: center; font-family: var(--font-mono); }

@media(max-width:640px) { .flow { grid-template-columns: 1fr; gap: 12px; } .arrow { transform: rotate(90deg); } }

.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 14px; margin-bottom: 20px; }
.card {
  background: var(--panel); border: 1px solid var(--panel-border); border-radius: 12px; padding: 18px;
  box-shadow: 0 1px 3px rgba(0,0,0,0.05);
}
.card h3 { margin: 0 0 12px; font-size: 11px; font-weight: 700; color: var(--muted); text-transform: uppercase; letter-spacing: .05em; }
.bar-track { height: 8px; background: var(--bg); border-radius: 4px; overflow: hidden; margin-top: 12px; }
.bar-fill { height: 100%; background: var(--teal); border-radius: 4px; transition: width .4s ease-out; }
.row { display: flex; justify-content: space-between; align-items: baseline; font-family: var(--font-mono); }
.row .v { font-size: 24px; font-weight: 700; color: var(--text); }
.row .u { font-size: 12px; color: var(--muted); }
.tag { display:inline-block; margin-top:8px; font-size:10px; font-family: var(--font-mono); font-weight:700; padding:3px 8px; border-radius:10px; }
.tag.warn { background: rgba(217,119,6,0.15); color: var(--amber); border:1px solid var(--amber); }
.tag.bad { background: rgba(220,38,38,0.15); color: var(--red); border:1px solid var(--red); }
.tag.ok { background: rgba(13,148,136,0.15); color: var(--teal); border:1px solid var(--teal); }

.section-title { font-size: 11px; color: var(--cyan); text-transform: uppercase; letter-spacing: .1em; margin: 28px 0 12px; font-weight: 700; }
.loads { background: var(--panel); border: 1px solid var(--panel-border); border-radius: 12px; padding: 4px 18px; margin-bottom: 20px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }
.load-row { display: flex; justify-content: space-between; align-items: center; padding: 14px 0; border-bottom: 1px solid var(--panel-border); font-size: 14px; }
.load-row:last-child { border-bottom: none; }
.load-row .pri { color: var(--cyan); font-family: var(--font-mono); font-size: 11px; width: 50px; flex-shrink: 0; font-weight: 700; }
.load-row .name { flex: 1; font-weight: 600; }
.state { font-family: var(--font-mono); font-size: 11px; padding: 4px 12px; border-radius: 12px; font-weight: 700; }
.state.on { background: rgba(13, 148, 136, 0.15); color: var(--teal); border: 1px solid var(--teal); }
.state.off { background: var(--bg); color: var(--muted); border: 1px solid var(--panel-border); }

.controls { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 8px; }
.hint { font-size: 11px; color: var(--muted); margin: 0 0 20px; }
button, a.btn {
  font-family: var(--font-sans); background: var(--panel); color: var(--text); border: 1px solid var(--panel-border);
  padding: 10px 20px; border-radius: 8px; font-size: 13px; font-weight: 600; cursor: pointer; text-decoration: none; display: inline-block;
  transition: all 0.2s ease; box-shadow: 0 1px 2px rgba(0,0,0,0.05);
}
button.primary { background: var(--cyan); color: #fff; border-color: var(--cyan); }
button.demo { border-color: var(--amber); color: var(--amber); }
button.demo.active { background: var(--amber); color: #fff; }
button:disabled { opacity: 0.4; cursor: not-allowed; }
button:hover:not(:disabled), a.btn:hover { filter: brightness(0.95); }
button.primary:hover:not(:disabled) { filter: brightness(0.9); }

.charts { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 14px; margin-bottom: 20px; }
.chart-card { background: var(--panel); border: 1px solid var(--panel-border); border-radius: 12px; padding: 16px; height: 240px; display: flex; flex-direction: column; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }
.chart-title { font-size: 11px; font-weight: 700; color: var(--text); text-transform: uppercase; letter-spacing: .05em; margin-bottom: 6px; }
.chart-legend { display: flex; gap: 14px; font-size: 11px; color: var(--muted); margin-bottom: 10px; font-family: var(--font-mono); font-weight: 600; flex-wrap: wrap; }
.chart-legend span { display: flex; align-items: center; gap: 6px; }
.chart-legend i { width: 8px; height: 8px; border-radius: 50%; display: inline-block; }
.chart-card canvas { flex: 1; width: 100% !important; height: 100% !important; display: block; }

.alert-log { background: var(--panel); border: 1px solid var(--panel-border); border-radius: 12px; padding: 14px 18px; font-family: var(--font-mono); font-size: 12px; color: var(--muted); margin-bottom: 20px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }
.alert-log .entry { display: flex; gap: 12px; padding: 6px 0; border-bottom: 1px solid var(--panel-border); }
.alert-log .entry:last-child { border-bottom: none; }
.alert-log .entry .t { color: var(--cyan); flex-shrink: 0; font-weight:700; }

.sysinfo { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; background: var(--panel); border: 1px solid var(--panel-border); border-radius: 12px; padding: 16px; font-size: 13px; box-shadow: 0 1px 3px rgba(0,0,0,0.05); }
.sysinfo .item { display: flex; justify-content: space-between; color: var(--muted); flex-direction: column; gap: 4px; }
.sysinfo .item span.v { color: var(--text); font-family: var(--font-mono); font-weight: 700; }
.sysinfo .item span.v.bad { color: var(--red); }
.sysinfo .item span.v.ok { color: var(--teal); }
.simtag { font-family: var(--font-mono); font-size: 10px; color: var(--amber); border: 1px solid var(--amber); border-radius: 4px; padding: 2px 6px; font-weight: 700; }
footer { margin-top: 30px; color: var(--muted); font-size: 11px; font-family: var(--font-mono); text-align: center; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div>
      <h1>SolarSentry &middot; Energy Management Dashboard</h1>
      <p>ESP32 Real-time System Metrics <span class="simtag" id="simtag" style="display:none;">SIMULATED DATA</span> <span class="simtag" id="demotag" style="display:none;">DEMO LEVEL FORCED</span></p>
    </div>
    <div class="header-right">
      <button class="theme-toggle-btn" onclick="toggleTheme()">
        <span id="themeIcon">&#127769;</span> <span id="themeText">Dark Mode</span>
      </button>
      <div class="badge NORMAL" id="stateBadge"><span class="dot"></span><span id="stateText">NORMAL</span></div>
    </div>
  </header>

  <div class="flow">
    <div class="stage in"><div class="num" id="solarPower">-- W</div><div class="label">Solar Power In</div></div>
    <div class="arrow">&rarr;</div>
    <div class="stage batt"><div class="num" id="battPct">--%</div><div class="label">Battery &middot; <span id="battV">--</span> V</div></div>
    <div class="arrow">&rarr;</div>
    <div class="stage out"><div class="num" id="loadPower">-- W</div><div class="label">Load Power Out</div></div>
  </div>

  <div class="section-title">Solar Photovoltaic Array</div>
  <div class="grid">
    <div class="card">
      <h3>Solar Power</h3><div class="row"><span class="v" id="c_solarPower">--</span><span class="u">W</span></div>
      <span class="tag warn" id="reverseTag" style="display:none;">PANEL V &lt; BATTERY V - NOT CHARGING</span>
    </div>
    <div class="card"><h3>Solar Voltage</h3><div class="row"><span class="v" id="c_solarVoltage">--</span><span class="u">V</span></div></div>
    <div class="card"><h3>Solar Current</h3><div class="row"><span class="v" id="c_solarCurrent">--</span><span class="u">A</span></div></div>
    <div class="card"><h3>Charging Power</h3><div class="row"><span class="v" id="c_chargingPower">--</span><span class="u">W</span></div></div>
  </div>

  <div class="section-title">Battery Storage</div>
  <div class="grid">
    <div class="card">
      <h3>Battery Level</h3>
      <div class="row"><span class="v" id="c_battLevel">--</span><span class="u">%</span></div>
      <div class="bar-track"><div class="bar-fill" id="socBar" style="width:0%;"></div></div>
      <span class="tag ok" id="chgTag">--</span>
    </div>
    <div class="card"><h3>Battery Voltage</h3><div class="row"><span class="v" id="c_battV">--</span><span class="u">V</span></div></div>
    <div class="card"><h3>System State</h3><div class="row"><span class="v" id="c_battStatus" style="font-size:18px;">--</span></div></div>
    <div class="card"><h3>Est. Runtime</h3><div class="row"><span class="v" id="c_runtime">--</span><span class="u" id="c_runtimeU"></span></div></div>
  </div>

  <div class="section-title">Output Telemetry</div>
  <div class="grid">
    <div class="card">
      <h3>Load Power</h3><div class="row"><span class="v" id="c_loadPower">--</span><span class="u">W</span></div>
      <span class="tag bad" id="loadSensorTag" style="display:none;">LOAD SENSOR NOT FOUND</span>
    </div>
    <div class="card"><h3>Load Current</h3><div class="row"><span class="v" id="c_loadCurrent">--</span><span class="u">A</span></div></div>
    <div class="card"><h3>Energy Consumed</h3><div class="row"><span class="v" id="c_energyConsumed">--</span><span class="u">Wh</span></div></div>
    <div class="card"><h3>Solar Harvested Today</h3><div class="row"><span class="v" id="c_energyCharged">--</span><span class="u">Wh</span></div></div>
  </div>

  <div class="section-title">Load Allocation Matrix</div>
  <div class="loads">
    <div class="load-row"><span class="pri">R1</span><span class="name">Relay 1 &middot; LED (essential, always on)</span><span class="state on" id="ledState">ON</span></div>
    <div class="load-row"><span class="pri">R2</span><span class="name">Relay 2 &middot; Fan (off in INTENSE)</span><span class="state on" id="fanState">ON</span></div>
    <div class="load-row"><span class="pri">R3</span><span class="name">Relay 3 &middot; Aux (on only in NORMAL)</span><span class="state on" id="auxState">ON</span></div>
  </div>

  <div class="section-title">Manual Overrides, Demo &amp; Logs</div>
  <div class="controls">
    <button class="primary" id="btnAuto" onclick="setMode('auto')">Auto Mode</button>
    <button id="btnManual" onclick="setMode('manual')">Manual Mode</button>
    <button id="btnLedOn" onclick="setLoad('led','on')">Relay 1 On</button>
    <button id="btnLedOff" onclick="setLoad('led','off')">Relay 1 Off</button>
    <button id="btnFanOn" onclick="setLoad('fan','on')">Relay 2 On</button>
    <button id="btnFanOff" onclick="setLoad('fan','off')">Relay 2 Off</button>
    <button id="btnAuxOn" onclick="setLoad('aux','on')">Relay 3 On</button>
    <button id="btnAuxOff" onclick="setLoad('aux','off')">Relay 3 Off</button>
    <button onclick="flipPolarity(1)">Flip R1 polarity</button>
    <button onclick="flipPolarity(2)">Flip R2 polarity</button>
    <button onclick="flipPolarity(3)">Flip R3 polarity</button>
    <button class="demo" id="btnD0" onclick="demo('0')">Demo NORMAL (3 loads)</button>
    <button class="demo" id="btnD1" onclick="demo('1')">Demo CRITICAL (2 loads)</button>
    <button class="demo" id="btnD2" onclick="demo('2')">Demo INTENSE (1 load)</button>
    <button class="demo" onclick="demo('off')">Demo OFF (real levels)</button>
    <button onclick="resync()">Re-sync % to voltage</button>
    <a href="/logs" target="_blank" class="btn">View Log Data</a>
    <a href="/download" class="btn">Download CSV</a>
  </div>
  <p class="hint">Relay buttons only respond in <b>Manual Mode</b>. In Auto Mode the battery level decides: <b>NORMAL</b> = 3 loads, <b>CRITICAL</b> = 2 loads (Aux shed), <b>INTENSE</b> = 1 load (LED only). The orange <b>Demo</b> buttons force a level so you can show every step; <b>Demo OFF</b> returns to the real battery levels. Like a phone, the battery % only goes <b>down</b> while not charging and only goes <b>up</b> while real charging current flows; use <b>Re-sync %</b> after charging with an external charger. If a relay works backwards, press its <b>Flip polarity</b> button once (saved automatically).</p>

  <div class="section-title">Graphs</div>
  <div class="charts">
    <div class="chart-card">
      <div class="chart-title">Power</div>
      <div class="chart-legend"><span><i style="background:#d97706"></i>Solar (W)</span><span><i style="background:#0d9488"></i>Load (W)</span></div>
      <canvas id="chartPower"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title">Battery voltage</div>
      <div class="chart-legend"><span><i style="background:#2563eb"></i>Terminal (V)</span><span><i style="background:#0d9488"></i>Resting, load-compensated (V)</span></div>
      <canvas id="chartVolt"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title">Battery percentage</div>
      <div class="chart-legend"><span><i style="background:#2563eb"></i>Shown (%)</span><span><i style="background:#d97706"></i>Voltage estimate (%)</span></div>
      <canvas id="chartPct"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title">Loads switched on</div>
      <div class="chart-legend"><span><i style="background:#0d9488"></i>Number of loads ON (3 / 2 / 1)</span></div>
      <canvas id="chartLoads"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title">Currents</div>
      <div class="chart-legend"><span><i style="background:#dc2626"></i>Load (A)</span><span><i style="background:#d97706"></i>Solar (A)</span></div>
      <canvas id="chartCurrent"></canvas>
    </div>
    <div class="chart-card">
      <div class="chart-title">Energy</div>
      <div class="chart-legend"><span><i style="background:#d97706"></i>Charged (Wh)</span><span><i style="background:#dc2626"></i>Consumed (Wh)</span></div>
      <canvas id="chartEnergy"></canvas>
    </div>
  </div>

  <div class="section-title">Event Log</div>
  <div class="alert-log" id="alertLog"></div>

  <div class="section-title">Node Diagnostics</div>
  <div class="sysinfo">
    <div class="item">Controller <span class="v">ESP32 (WROOM)</span></div>
    <div class="item">IP Address <span class="v" id="ipAddr">--</span></div>
    <div class="item">SD Card <span class="v" id="sdStatus">--</span></div>
    <div class="item">Charge Sensor (0x40) <span class="v" id="chargeSensorStatus">--</span></div>
    <div class="item">Load Sensor (0x44) <span class="v" id="loadSensorStatus">--</span></div>
    <div class="item">Telemetry Sync <span class="v" id="lastUpdate">--</span></div>
  </div>

  <footer>SOLARSENTRY MICROGRID TELEMETRY SYSTEM &middot; ESP32 EDGE ENGINE</footer>
</div>
<script>
const MAXPOINTS = 90;
const hist = { solar: [], load: [], vterm: [], vrest: [], pct: [], est: [], loads: [], loadA: [], solarA: [], charged: [], consumed: [] };
let lastUpdateMs = Date.now();

function toggleTheme() {
  const currentTheme = document.documentElement.getAttribute('data-theme');
  applyTheme(currentTheme === 'dark' ? 'light' : 'dark');
}

function applyTheme(theme) {
  document.documentElement.setAttribute('data-theme', theme);
  try { localStorage.setItem('solar_theme', theme); } catch (e) {}
  document.getElementById('themeIcon').textContent = theme === 'dark' ? '\u2600\uFE0F' : '\uD83C\uDF19';
  document.getElementById('themeText').textContent = theme === 'dark' ? 'Light Mode' : 'Dark Mode';
  redrawAllCharts();
}

let savedTheme = 'light';
try { savedTheme = localStorage.getItem('solar_theme') || 'light'; } catch (e) {}

function setBadgeRow(id, state) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = state ? 'ON' : 'OFF';
  el.className = 'state ' + (state ? 'on' : 'off');
}

function drawChart(canvas, seriesList) {
  const cssWidth = canvas.clientWidth;
  const cssHeight = canvas.clientHeight;
  if (!cssWidth || !cssHeight) return;
  const dpr = window.devicePixelRatio || 1;
  canvas.width = cssWidth * dpr;
  canvas.height = cssHeight * dpr;
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssWidth, cssHeight);

  const allVals = seriesList.flatMap(s => s.data);
  if (allVals.length < 2) return;
  let min = Math.min(...allVals), max = Math.max(...allVals);
  if (min === max) { min -= 1; max += 1; }
  const pad = (max - min) * 0.15;
  min -= pad; max += pad;

  const P = { left: 38, right: 8, top: 12, bottom: 12 };
  const plotW = cssWidth - P.left - P.right;
  const plotH = cssHeight - P.top - P.bottom;

  const style = getComputedStyle(document.documentElement);
  ctx.strokeStyle = style.getPropertyValue('--chart-grid').trim() || '#e2e8f0';
  ctx.fillStyle = style.getPropertyValue('--muted').trim() || '#64748b';
  ctx.font = '10px "Courier New", monospace';
  ctx.lineWidth = 1;
  const ticks = 4;
  for (let i = 0; i <= ticks; i++) {
    const y = P.top + plotH - (i / ticks) * plotH;
    ctx.beginPath();
    ctx.moveTo(P.left, y);
    ctx.lineTo(cssWidth - P.right, y);
    ctx.stroke();
    const val = min + (i / ticks) * (max - min);
    ctx.fillText(val.toFixed(1), 2, y + 3);
  }

  seriesList.forEach(s => {
    const n = s.data.length;
    if (n < 2) return;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    s.data.forEach((v, i) => {
      const x = P.left + (i / (n - 1)) * plotW;
      const y = P.top + plotH - ((v - min) / (max - min)) * plotH;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
}

function redrawAllCharts() {
  drawChart(document.getElementById('chartPower'), [
    { data: hist.solar, color: '#d97706' },
    { data: hist.load, color: '#0d9488' }
  ]);
  drawChart(document.getElementById('chartVolt'), [
    { data: hist.vterm, color: '#2563eb' },
    { data: hist.vrest, color: '#0d9488' }
  ]);
  drawChart(document.getElementById('chartPct'), [
    { data: hist.pct, color: '#2563eb' },
    { data: hist.est, color: '#d97706' }
  ]);
  drawChart(document.getElementById('chartLoads'), [
    { data: hist.loads, color: '#0d9488' }
  ]);
  drawChart(document.getElementById('chartCurrent'), [
    { data: hist.loadA, color: '#dc2626' },
    { data: hist.solarA, color: '#d97706' }
  ]);
  drawChart(document.getElementById('chartEnergy'), [
    { data: hist.charged, color: '#d97706' },
    { data: hist.consumed, color: '#dc2626' }
  ]);
}

function pushHistory(s) {
  hist.solar.push(s.solarPower);
  hist.load.push(s.loadPower);
  hist.vterm.push(s.batteryVoltage);
  hist.vrest.push(s.restVoltage);
  hist.pct.push(s.batteryPercent);
  hist.est.push(s.estPercent);
  hist.loads.push((s.ledOn ? 1 : 0) + (s.fanOn ? 1 : 0) + (s.auxOn ? 1 : 0));
  hist.loadA.push(s.loadCurrent);
  hist.solarA.push(s.solarCurrent);
  hist.charged.push(s.energyCharged);
  hist.consumed.push(s.energyConsumed);
  Object.values(hist).forEach(arr => { if (arr.length > MAXPOINTS) arr.shift(); });
  redrawAllCharts();
}

const alerts = [];
function pushAlert(msg) {
  if (!msg) return;
  if (alerts.length && alerts[0].msg === msg) return;
  alerts.unshift({ t: new Date().toLocaleTimeString(), msg });
  if (alerts.length > 6) alerts.pop();
  const box = document.getElementById('alertLog');
  box.innerHTML = '';
  alerts.forEach(a => {
    const d = document.createElement('div');
    d.className = 'entry';
    const t = document.createElement('span');
    t.className = 't';
    t.textContent = a.t;
    d.appendChild(t);
    d.appendChild(document.createTextNode(a.msg));
    box.appendChild(d);
  });
}

function setState(s) {
  document.getElementById('solarPower').textContent = s.solarPower.toFixed(1) + ' W';
  document.getElementById('battPct').textContent = s.batteryPercent.toFixed(0) + '%';
  document.getElementById('battV').textContent = s.batteryVoltage.toFixed(1);
  document.getElementById('loadPower').textContent = s.loadPower.toFixed(1) + ' W';

  document.getElementById('c_solarPower').textContent = s.solarPower.toFixed(1);
  document.getElementById('c_solarVoltage').textContent = s.solarVoltage.toFixed(1);
  document.getElementById('c_solarCurrent').textContent = s.solarCurrent.toFixed(2);
  document.getElementById('c_chargingPower').textContent = s.chargingPower.toFixed(1);
  document.getElementById('reverseTag').style.display = s.reverseFlow ? 'inline-block' : 'none';

  document.getElementById('c_battLevel').textContent = s.batteryPercent.toFixed(0);
  document.getElementById('socBar').style.width = s.batteryPercent.toFixed(0) + '%';
  const chg = document.getElementById('chgTag');
  chg.textContent = s.charging ? 'CHARGING - % can rise' : 'NOT CHARGING - % holds or falls';
  chg.className = 'tag ' + (s.charging ? 'ok' : 'warn');
  document.getElementById('c_battV').textContent = s.batteryVoltage.toFixed(2);
  document.getElementById('c_battStatus').textContent = s.sysState;
  document.getElementById('c_runtime').textContent = s.runtimeH + 'h ' + s.runtimeM + 'm';

  document.getElementById('c_loadPower').textContent = s.loadPower.toFixed(1);
  document.getElementById('c_loadCurrent').textContent = s.loadCurrent.toFixed(2);
  document.getElementById('c_energyConsumed').textContent = s.energyConsumed.toFixed(1);
  document.getElementById('c_energyCharged').textContent = s.dailySolar.toFixed(1);
  document.getElementById('loadSensorTag').style.display = s.loadSensorOk ? 'none' : 'inline-block';

  setBadgeRow('ledState', s.ledOn);
  setBadgeRow('fanState', s.fanOn);
  setBadgeRow('auxState', s.auxOn);

  document.getElementById('btnAuto').className = s.autoMode ? 'primary' : '';
  document.getElementById('btnManual').className = s.autoMode ? '' : 'primary';
  ['btnLedOn','btnLedOff','btnFanOn','btnFanOff','btnAuxOn','btnAuxOff'].forEach(id => {
    document.getElementById(id).disabled = s.autoMode;
  });

  document.getElementById('demotag').style.display = s.demo ? 'inline-block' : 'none';
  ['NORMAL','CRITICAL','INTENSE'].forEach((nm, i) => {
    document.getElementById('btnD' + i).className = 'demo' + ((s.demo && s.sysState === nm) ? ' active' : '');
  });

  document.getElementById('stateBadge').className = 'badge ' + s.sysState;
  document.getElementById('stateText').textContent = s.sysState;

  document.getElementById('ipAddr').textContent = s.ip || location.host;
  document.getElementById('sdStatus').textContent = s.sdReady ? 'Connected / Logging' : 'Disconnected';
  document.getElementById('simtag').style.display = s.simulated ? 'inline-block' : 'none';

  const chargeEl = document.getElementById('chargeSensorStatus');
  chargeEl.textContent = s.chargeSensorOk ? 'Connected' : 'NOT FOUND';
  chargeEl.className = 'v ' + (s.chargeSensorOk ? 'ok' : 'bad');

  const loadEl = document.getElementById('loadSensorStatus');
  loadEl.textContent = s.loadSensorOk ? 'Connected' : 'NOT FOUND';
  loadEl.className = 'v ' + (s.loadSensorOk ? 'ok' : 'bad');

  lastUpdateMs = Date.now();
  pushAlert(s.alert);
  pushHistory(s);
}

function tickLastUpdate() {
  const secs = ((Date.now() - lastUpdateMs) / 1000).toFixed(1);
  document.getElementById('lastUpdate').textContent = secs + 's ago';
}

function refresh() {
  fetch('/data').then(r => r.json()).then(setState).catch(() => {});
}
function setMode(mode) {
  fetch('/control?mode=' + mode).then(r => r.json()).then(setState).catch(() => {});
}
function setLoad(load, val) {
  fetch('/control?' + load + '=' + val).then(r => r.json()).then(setState).catch(() => {});
}
function flipPolarity(n) {
  fetch('/polarity?relay=' + n).then(r => r.json()).then(setState).catch(() => {});
}
function demo(level) {
  fetch('/demo?level=' + level).then(r => r.json()).then(setState).catch(() => {});
}
function resync() {
  fetch('/resync').then(r => r.json()).then(setState).catch(() => {});
}

applyTheme(savedTheme);
refresh();
setInterval(refresh, 2000);
setInterval(tickLastUpdate, 500);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// ================= Log table page (/logs) =================
static const char LOG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SolarSentry Log</title>
<style>
body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;margin:16px;background:#f8fafc;color:#0f172a}
h1{font-size:20px;margin:0 0 4px}
h2{font-size:15px;margin:22px 0 4px}
p{color:#64748b;font-size:13px;margin:0 0 10px}
.bar{display:flex;gap:8px;flex-wrap:wrap;margin:10px 0}
button,a{font:600 13px sans-serif;padding:8px 14px;border-radius:8px;border:1px solid #cbd5e1;background:#fff;color:#0f172a;text-decoration:none;cursor:pointer}
.wrap{overflow-x:auto;background:#fff;border:1px solid #e2e8f0;border-radius:10px;max-height:420px;overflow-y:auto}
table{border-collapse:collapse;width:100%;font-family:'Courier New',monospace;font-size:13px}
th,td{padding:7px 12px;text-align:right;border-bottom:1px solid #e2e8f0;white-space:nowrap}
th{background:#f1f5f9;position:sticky;top:0;font-size:11px;color:#475569}
td:first-child,th:first-child,td:last-child,th:last-child{text-align:left}
.NORMAL{color:#0d9488;font-weight:700}.CRITICAL{color:#d97706;font-weight:700}
.INTENSE{color:#dc2626;font-weight:700}
</style></head><body>
<h1>SolarSentry &middot; Log Data</h1>
<p id="info">Loading...</p>
<div class="bar">
  <button onclick="load()">Refresh</button>
  <button id="lim" onclick="LIMIT=(LIMIT===100?100000:100);load()">Show all rows</button>
  <a href="/download">Download CSV</a>
  <a href="/">Back to dashboard</a>
</div>
<div id="out"></div>
<script>
let LIMIT = 100;
function uptime(ms){
  let s=Math.floor(ms/1000),h=Math.floor(s/3600),m=Math.floor(s%3600/60);s%=60;
  return h+'h '+String(m).padStart(2,'0')+'m '+String(s).padStart(2,'0')+'s';
}
function num(v,d){ const x=parseFloat(v); return (Math.abs(x)<0.005&&d===2)||(Math.abs(x)<0.05&&d===1)?(0).toFixed(d):x.toFixed(d); }
function load(){
  document.getElementById('info').textContent='Loading...';
  fetch('/raw').then(r=>r.text()).then(t=>{
    const rows=[];
    t.split(/\r?\n/).slice(1).forEach(l=>{
      const c=l.trim().split(',');
      if(c.length===8 && !isNaN(parseInt(c[0]))) rows.push(c);
    });
    // split into sessions wherever uptime goes backwards (a reboot)
    const sessions=[]; let cur=[]; let prev=-1;
    rows.forEach(c=>{
      const ms=parseInt(c[0]);
      if(ms<prev && cur.length){ sessions.push(cur); cur=[]; }
      cur.push(c); prev=ms;
    });
    if(cur.length) sessions.push(cur);

    const out=document.getElementById('out'); out.innerHTML='';
    sessions.map((s,i)=>({s,n:i+1})).reverse().forEach(({s,n})=>{
      const bv=s.map(c=>parseFloat(c[4])), sw=s.map(c=>parseFloat(c[3]));
      const h=document.createElement('h2');
      h.textContent='Session '+n+(n===sessions.length?' (latest)':'')+' \u2014 '+s.length+' rows, '+uptime(parseInt(s[s.length-1][0]));
      const p=document.createElement('p');
      p.textContent='Battery '+Math.min(...bv).toFixed(2)+' to '+Math.max(...bv).toFixed(2)+' V  |  Peak solar '+Math.max(...sw).toFixed(1)+' W';
      const w=document.createElement('div'); w.className='wrap';
      let html='<table><thead><tr><th>Uptime</th><th>Solar (V)</th><th>Solar (A)</th><th>Solar (W)</th><th>Battery (V)</th><th>Battery (%)</th><th>Load (W)</th><th>State</th></tr></thead><tbody>';
      s.slice().reverse().slice(0,LIMIT).forEach(c=>{
        html+='<tr><td>'+uptime(parseInt(c[0]))+'</td><td>'+num(c[1],1)+'</td><td>'+num(c[2],2)+'</td><td>'+num(c[3],1)+'</td><td>'+num(c[4],2)+'</td><td>'+num(c[5],0)+'</td><td>'+num(c[6],1)+'</td><td class="'+c[7]+'">'+c[7]+'</td></tr>';
      });
      w.innerHTML=html+'</tbody></table>';
      out.appendChild(h); out.appendChild(p); out.appendChild(w);
    });
    document.getElementById('info').textContent=rows.length+' valid rows in '+sessions.length+' session(s). Newest first.';
    document.getElementById('lim').textContent=(LIMIT===100)?'Show all rows':'Show last 100 per session';
  }).catch(()=>{document.getElementById('info').textContent='Could not load log.';});
}
load();
</script></body></html>
)rawliteral";

void handleLogsPage() {
  server.send_P(200, "text/html", LOG_HTML);
}

// ================= Serial relay control (bench testing) =================
void serialRelay(int i, bool on) {
  r.autoMode = false;
  if (demoLevel >= 0) { demoLevel = -1; stateSeeded = false; }
  setRelayState(i, on, true);
  applyLoadLogic(false);
  Serial.print("Relay ");
  Serial.print(i + 1);
  Serial.println(on ? ": ON" : ": OFF");
}

void printStatus() {
  Serial.println("---------------- STATUS ----------------");
  Serial.print("Mode: ");   Serial.print(r.autoMode ? "AUTO" : "MANUAL");
  Serial.println(demoLevel >= 0 ? "  (DEMO level forced)" : "");
  Serial.print("Hotspot: "); Serial.print(ap_ssid);
  Serial.print("  IP "); Serial.print(WiFi.softAPIP());
  Serial.print("  clients "); Serial.println(WiFi.softAPgetStationNum());
  Serial.print("SD card: "); Serial.println(sdAvailable ? "OK (logging)" : "NOT AVAILABLE");
  Serial.print("Battery source in use: "); Serial.println(battFromIna ? "INA226 (0x44)" : "analog pin 34");
  Serial.print("  INA226 bus voltage: "); Serial.print(lastInaBusV, 3);
  Serial.print(" V   (sensor ");  Serial.print(inaLoadOk ? "OK" : "NOT FOUND"); Serial.println(")");
  Serial.print("  Analog pin 34: "); Serial.print(lastSensorOutV, 3);
  Serial.print(" V at pin -> "); Serial.print(lastAnalogBattV, 3);
  Serial.print(" V battery   calibration: "); Serial.println(VOLTAGE_CALIBRATION, 3);
  Serial.print("State: ");  Serial.println(stateName(r.sysState));
  Serial.print("Battery terminal: "); Serial.print(r.batteryVoltage, 2); Serial.print(" V   resting (compensated): ");
  Serial.print(emaRestV, 2); Serial.println(" V");
  Serial.print("Battery % shown: "); Serial.print(r.batteryPercent, 0);
  Serial.print("   voltage estimate: "); Serial.print(estPercent, 0);
  Serial.print("   charging: "); Serial.println(isCharging ? "YES" : "no");
  Serial.print("Internal resistance used: "); Serial.print(BATT_INTERNAL_R, 3); Serial.println(" ohm");
  for (int i = 0; i < 3; i++) {
    Serial.print("Relay "); Serial.print(i + 1); Serial.print(" (");
    Serial.print(RELAY_NAMES[i]); Serial.print(", GPIO "); Serial.print(RELAY_PINS[i]); Serial.print("): ");
    Serial.print(relayState[i] ? "ON" : "OFF");
    Serial.print("   polarity: ");
    Serial.println(relayActiveLow[i] ? "active-LOW" : "active-HIGH");
  }
  Serial.println("----------------------------------------");
}

void handleSerialCommands() {
  while (Serial.available()) {
    char command = Serial.read();
    switch (command) {
      case '1': serialRelay(0, true);  break;
      case '0': serialRelay(0, false); break;
      case '2': serialRelay(1, true);  break;
      case '3': serialRelay(1, false); break;
      case '4': serialRelay(2, true);  break;
      case '5': serialRelay(2, false); break;
      case 'A': case 'a':
        serialRelay(0, true); serialRelay(1, true); serialRelay(2, true);
        break;
      case 'B': case 'b':
        serialRelay(0, false); serialRelay(1, false); serialRelay(2, false);
        break;
      case '7': flipPolarity(0); applyLoadLogic(false); break;
      case '8': flipPolarity(1); applyLoadLogic(false); break;
      case '9': flipPolarity(2); applyLoadLogic(false); break;
      case 'M': case 'm':
        r.autoMode = true;
        applyLoadLogic(true);
        Serial.println("AUTO mode ON");
        break;
      case 'V': case 'v':   // switch battery voltage source (INA226 <-> analog), saved to flash
        useInaForBatt = !useInaForBatt;
        prefs.putUChar("vsrc", useInaForBatt ? 1 : 0);
        Serial.print("Battery voltage source preference: ");
        Serial.println(useInaForBatt ? "INA226 (falls back to analog if unusable)" : "analog pin 34 only");
        break;
      case 'C': case 'c': {   // e.g. c12.57 = "my multimeter says 12.57 V" (calibrates the ANALOG sensor)
        float measured = Serial.parseFloat();
        if (measured < 5.0f || measured > 20.0f || lastSensorOutV < 0.05f) {
          Serial.println("Calibration failed: type e.g. c12.57 (multimeter volts, 5-20) and make sure pin 34 sensor is connected.");
        } else {
          VOLTAGE_CALIBRATION = measured / lastSensorOutV;
          prefs.putFloat("vcal2", VOLTAGE_CALIBRATION);
          emaBattV = 0;
          emaRestV = 0;
          Serial.print("Analog voltage calibration set to ");
          Serial.print(VOLTAGE_CALIBRATION, 3);
          Serial.println(" (saved)");
        }
        break;
      }
      case 'R': case 'r': {   // e.g. r0.30 = battery internal resistance in ohms (saved)
        float rr = Serial.parseFloat();
        if (rr < 0.0f || rr > 2.0f) {
          Serial.println("Internal resistance not changed: type e.g. r0.30 (ohms, 0 to 2).");
        } else {
          BATT_INTERNAL_R = rr;
          prefs.putFloat("rint", rr);
          emaRestV = 0;
          Serial.print("Battery internal resistance set to ");
          Serial.print(BATT_INTERNAL_R, 3);
          Serial.println(" ohm (saved)");
        }
        break;
      }
      case 'Y': case 'y':   // re-sync displayed % to the measured voltage
        r.batteryPercent = estPercent;
        pctSeeded = true;
        if (demoLevel < 0) stateSeeded = false;
        Serial.print("Battery % re-synced to ");
        Serial.print(r.batteryPercent, 0);
        Serial.println(" %");
        break;
      case 'S': case 's': printStatus(); break;
      default: break;   // ignores newline / carriage return
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting SolarSentry SteadyRelay ESP32 (v7 FinalDemo)...");

  // 1) Load saved settings, then latch every relay OFF before switching to OUTPUT.
  prefs.begin("ssentry", false);
  loadPolarity();
  VOLTAGE_CALIBRATION = prefs.getFloat("vcal2", VOLTAGE_CALIBRATION);
  BATT_INTERNAL_R = prefs.getFloat("rint", BATT_INTERNAL_R);
  useInaForBatt = (prefs.getUChar("vsrc", 1) != 0);
  Serial.setTimeout(100);
  for (int i = 0; i < 3; i++) {
    relayState[i] = false;
    relayLastChangeMs[i] = millis() - MIN_RELAY_HOLD_MS;   // first auto command is not held back
    driveRelayPin(i);
    pinMode(RELAY_PINS[i], OUTPUT);
  }

  // 2) HOTSPOT + WEB SERVER FIRST - before sensors and SD card.
  startHotspot();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/control", handleControl);
  server.on("/polarity", handlePolarity);
  server.on("/demo", handleDemo);         // force a load level for the presentation
  server.on("/resync", handleResync);     // re-sync % to measured voltage
  server.on("/logs", handleLogsPage);     // pretty table
  server.on("/raw", handleLogs);          // plain CSV text
  server.on("/download", handleDownload); // download log.csv
  server.begin();
  Serial.println("Web server active!");

  // 3) Now the slower hardware setup.
  randomSeed(esp_random());
  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_SENSOR_PIN, ADC_11db);   // full ~0-3.3 V input range

  Wire.begin(21, 22);   // SDA=GPIO21, SCL=GPIO22
  initINA226();

  initSDCard();

  lastDayResetMs = millis();
  lastSensorRescanMs = millis();

  Serial.println("--------------------------------------");
  Serial.println("RELAY PINS: R1 = GPIO 25, R2 = GPIO 26, R3 = GPIO 33");
  Serial.println("SD PINS:    CS = 5, SCK = 18, MISO = 19, MOSI = 23");
  Serial.println("LOAD LEVELS (auto): NORMAL = 3 loads, CRITICAL = 2 loads (Aux shed), INTENSE = 1 load (LED only)");
  Serial.println("BATTERY %: only falls while not charging, only rises while charging (like a phone)");
  Serial.println("SERIAL COMMANDS:");
  Serial.println("1 = Relay 1 ON   0 = Relay 1 OFF");
  Serial.println("2 = Relay 2 ON   3 = Relay 2 OFF");
  Serial.println("4 = Relay 3 ON   5 = Relay 3 OFF");
  Serial.println("A = All ON       B = All OFF");
  Serial.println("M = back to Auto mode      S = print status");
  Serial.println("7 / 8 / 9 = flip polarity of Relay 1 / 2 / 3 (saved)");
  Serial.println("v = switch battery voltage source (INA226 <-> analog), saved");
  Serial.println("c12.57 = calibrate the ANALOG sensor to your multimeter reading (saved)");
  Serial.println("r0.30 = set battery internal resistance in ohms (saved)");
  Serial.println("y = re-sync displayed battery % to the measured voltage");
  Serial.println("(relay commands switch to Manual mode)");
  Serial.println("--------------------------------------");
  for (int i = 0; i < 3; i++) {
    Serial.print("Relay "); Serial.print(i + 1); Serial.print(" polarity: ");
    Serial.println(relayActiveLow[i] ? "active-LOW" : "active-HIGH");
  }
  Serial.println("--------------------------------------");

  lastSampleMs = millis();
  sample(SAMPLE_INTERVAL_MS);   // first reading right away so the page has real data
}

unsigned long lastApCheckMs = 0;

void loop() {
  server.handleClient();
  handleSerialCommands();

  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    float dtMs = (float)(now - lastSampleMs);
    lastSampleMs = now;
    sample(dtMs);
  }

  if (now - lastSensorRescanMs >= RESCAN_INTERVAL_MS) {
    lastSensorRescanMs = now;
    rescanMissingSensors();
  }

  // Hotspot watchdog: if the access point is ever not running, bring it back.
  if (now - lastApCheckMs >= 10000) {
    lastApCheckMs = now;
    if ((WiFi.getMode() & WIFI_AP) == 0) {
      Serial.println("Hotspot was down - restarting it.");
      startHotspot();
    }
  }
}
