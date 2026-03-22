// =============================================================================
// tories_dual_tank v2.2
// Arduino Uno R4 Minima — Dual Tank Adaptive Self-Tuning Temperature Controller
// =============================================================================
//
// Added in v2.2
// --------------
// 1) Serial command interface for GUI:
//      SP,0,37.0   -> set MAIN setpoint
//      SP,1,36.5   -> set RES setpoint
//      GET         -> return both setpoints
//
// 2) Three physical buttons (updated mapping):
//      D2 = UP
//      D3 = middle / SET
//           single click -> toggle set mode
//           double click -> switch selected tank
//      D4 = DOWN
//
// 3) OLED marker:
//      '>' = selected tank
//      '*' = selected tank and set mode active
//
// Buttons use INPUT_PULLUP:
//   one side of button -> Arduino pin
//   other side         -> GND
// =============================================================================

// ---------- Includes ----------
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- MAX6675 ----------
const int thermoSO       = 12;
const int thermoSCK      = 13;
const int thermoCS_main  = 10;
const int thermoCS_res   = 9;

MAX6675 thermocoupleMain(thermoSCK, thermoCS_main, thermoSO);
MAX6675 thermocoupleRes(thermoSCK, thermoCS_res, thermoSO);

// ---------- SSR pins ----------
const int HEAT_PIN_MAIN = 7;
const int HEAT_PIN_RES  = 6;

// ---------- Button pins ----------
const int BTN_UP_PIN   = 2;
const int BTN_SET_PIN  = 3;
const int BTN_DOWN_PIN = 4;

// ---------- System constants ----------
const int TANK_COUNT = 2;
const int HEAT_PINS[TANK_COUNT] = {HEAT_PIN_MAIN, HEAT_PIN_RES};
const char* TANK_LABELS[TANK_COUNT] = {"MAIN", "RES "};

const double MAX_TEMP_C       = 41.0;     // hard cutoff
const double WARN_TEMP_C      = 39.0;     // visual warning
const double EMA_ALPHA        = 0.30;     // sensor smoothing
const double TREND_THRESHOLD  = 0.20;     // degC over trend window
const double AT_SETPOINT_BAND = 1.0;      // both tanks within +/- this
const int TREND_BUFFER_SIZE   = 5;        // 5 * 250ms = 1.25s

const unsigned long WINDOW_SIZE      = 1000UL; // SSR time-proportional window
const unsigned long SENSOR_INTERVAL  = 250UL;
const unsigned long CONTROL_INTERVAL = 500UL;
const unsigned long DISPLAY_INTERVAL = 250UL;
const unsigned long LOG_INTERVAL     = 500UL;
const unsigned long RETUNE_INTERVAL  = 5000UL;
const unsigned long WARMUP_TIME      = 30000UL;

// ---------- GUI / setpoint constants ----------
const double MIN_SETPOINT_C = 20.0;
const double MAX_SETPOINT_ALLOWED_C = 39.0;
const double BUTTON_SP_STEP_C = 0.1;

// ---------- Button timing ----------
const unsigned long BUTTON_DEBOUNCE_MS = 40;
const unsigned long DOUBLE_CLICK_MS    = 350;

// ---------- Adaptive controller constants ----------
const double D_FILTER_ALPHA = 0.25;  // derivative smoothing
const double RLS_FORGET     = 0.995; // RLS forgetting factor

const double INTEGRAL_MIN = -25.0;
const double INTEGRAL_MAX = 100.0;

// ---------- Per-tank state ----------
struct TankState {
  double rawTemp;
  double filteredTemp;
  double setpoint;
  double outputPct;
  bool heaterOn;
  unsigned long windowStart;

  float tempHistory[TREND_BUFFER_SIZE];
  int historyIndex;
  int historyCount;

  // Adaptive PID gains (current)
  double kp;
  double ki;
  double kd;

  // Safe defaults / seed gains
  double kpDefault;
  double kiDefault;
  double kdDefault;

  // Gain bounds
  double kpMin;
  double kpMax;
  double kiMin;
  double kiMax;
  double kdMin;
  double kdMax;

  // Controller internal state
  double integralTerm;
  double prevTemp;
  double dTempFilt;
  unsigned long lastControlMs;

  // Online identified model: T[k+1] = a*T[k] + b*u[k] + c
  double theta[3];
  double P[3][3];
  double modelPrevTemp;
  double prevModelOutputNorm;
  bool modelInitialized;
  bool modelValid;
  unsigned long lastRetuneMs;
  unsigned long warmupUntilMs;

  // Debug / telemetry
  double estTauSec;
  double estKproc;
};

TankState tanks[TANK_COUNT];

// ---------- Button state ----------
struct ButtonState {
  int pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
};

ButtonState btnUp   = {BTN_UP_PIN,   HIGH, HIGH, 0};
ButtonState btnSet  = {BTN_SET_PIN,  HIGH, HIGH, 0};
ButtonState btnDown = {BTN_DOWN_PIN, HIGH, HIGH, 0};

int selectedTank = 0;     // 0 = MAIN, 1 = RES
bool setMode = false;     // true = Up/Down edit selected tank setpoint

bool pendingSetSingleClick = false;
unsigned long lastSetClickMs = 0;

// ---------- Serial parser ----------
String serialLine = "";

// ---------- Timers ----------
unsigned long lastSensorRead    = 0;
unsigned long lastControlRun    = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastLogOutput     = 0;
unsigned long bootTime          = 0;

unsigned long timeAtSetpointSec   = 0;
unsigned long lastAtSetpointCheck = 0;
bool atSetpoint = false;

// ---------- Forward declarations ----------
double clampDouble(double x, double lo, double hi);
bool isValidTemp(double t);
double readThermocoupleC(int tank);
void zeroMatrix3(double M[3][3]);
void initTankState(int tank, double sp, double kp0, double ki0, double kd0);
void initDisplay();
void initSensors();
void initButtons();
void printLogHeader();
void readSensors();
void updateRLSModel(int tank);
void retuneAdaptiveGains(int tank, unsigned long nowMs);
void computeAdaptiveOutput(int tank, double dtSec);
void runControl();
void driveHeaters();
void updateDisplay();
void logData();
char getTrend(int tank);
int getStatus(int tank);
void formatTime(unsigned long totalSec, char* buf);
void drawTankRow(int tank, int yOffset);

// Button / setpoint / serial helpers
bool updateButtonPressed(ButtonState &btn);
void handleButtons();
void setTankSetpoint(int tank, double newSP);
void handleSerialCommands();
void processCommand(String line);

// =============================================================================
// HELPERS
// =============================================================================

double clampDouble(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool isValidTemp(double t) {
  return (!isnan(t) && t > -50.0 && t < 500.0);
}

double readThermocoupleC(int tank) {
  if (tank == 0) return thermocoupleMain.readCelsius();
  return thermocoupleRes.readCelsius();
}

void zeroMatrix3(double M[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      M[i][j] = 0.0;
    }
  }
}

void initTankState(int tank, double sp, double kp0, double ki0, double kd0) {
  tanks[tank].rawTemp      = NAN;
  tanks[tank].filteredTemp = sp;
  tanks[tank].setpoint     = sp;
  tanks[tank].outputPct    = 0.0;
  tanks[tank].heaterOn     = false;
  tanks[tank].windowStart  = 0;

  tanks[tank].historyIndex = 0;
  tanks[tank].historyCount = 0;
  for (int i = 0; i < TREND_BUFFER_SIZE; i++) {
    tanks[tank].tempHistory[i] = (float)sp;
  }

  tanks[tank].kp = kp0;
  tanks[tank].ki = ki0;
  tanks[tank].kd = kd0;

  tanks[tank].kpDefault = kp0;
  tanks[tank].kiDefault = ki0;
  tanks[tank].kdDefault = kd0;

  tanks[tank].kpMin = 2.0;
  tanks[tank].kpMax = 45.0;
  tanks[tank].kiMin = 0.00;
  tanks[tank].kiMax = 0.60;
  tanks[tank].kdMin = 0.00;
  tanks[tank].kdMax = 80.0;

  tanks[tank].integralTerm  = 0.0;
  tanks[tank].prevTemp      = sp;
  tanks[tank].dTempFilt     = 0.0;
  tanks[tank].lastControlMs = 0;

  tanks[tank].theta[0] = 0.995; // a
  tanks[tank].theta[1] = 0.050; // b
  tanks[tank].theta[2] = 0.0;   // c

  zeroMatrix3(tanks[tank].P);
  tanks[tank].P[0][0] = 1000.0;
  tanks[tank].P[1][1] = 1000.0;
  tanks[tank].P[2][2] = 1000.0;

  tanks[tank].modelPrevTemp       = sp;
  tanks[tank].prevModelOutputNorm = 0.0;
  tanks[tank].modelInitialized    = false;
  tanks[tank].modelValid          = false;
  tanks[tank].lastRetuneMs        = 0;
  tanks[tank].warmupUntilMs       = 0;
  tanks[tank].estTauSec           = NAN;
  tanks[tank].estKproc            = NAN;
}

char getTrend(int tank) {
  if (tanks[tank].historyCount < TREND_BUFFER_SIZE) return '-';

  int oldestIdx = tanks[tank].historyIndex;
  int newestIdx = (tanks[tank].historyIndex + TREND_BUFFER_SIZE - 1) % TREND_BUFFER_SIZE;

  float delta = tanks[tank].tempHistory[newestIdx] - tanks[tank].tempHistory[oldestIdx];

  if (delta > TREND_THRESHOLD) return 0x18;
  else if (delta < -TREND_THRESHOLD) return 0x19;
  else return 0x1A;
}

int getStatus(int tank) {
  if (!isValidTemp(tanks[tank].rawTemp)) return 2;
  if (tanks[tank].filteredTemp >= MAX_TEMP_C) return 2;
  if (tanks[tank].filteredTemp >= WARN_TEMP_C) return 1;
  return 0;
}

void formatTime(unsigned long totalSec, char* buf) {
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec % 3600UL) / 60UL;
  unsigned long s = totalSec % 60UL;
  sprintf(buf, "%02lu:%02lu:%02lu", h, m, s);
}

void setTankSetpoint(int tank, double newSP) {
  if (tank < 0 || tank >= TANK_COUNT) return;

  newSP = clampDouble(newSP, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);
  tanks[tank].setpoint = newSP;

  // soften response to setpoint changes
  tanks[tank].integralTerm = 0.0;
}

void drawTankRow(int tank, int yOffset) {
  int status = getStatus(tank);

  display.setCursor(0, yOffset);
  if (selectedTank == tank) {
    display.print(setMode ? "*" : ">");
  } else {
    display.print(" ");
  }
  display.print(TANK_LABELS[tank]);
  display.print(" ");

  if (status == 2) {
    display.print("ERR  ");
  } else {
    display.print(tanks[tank].filteredTemp, 1);
    display.print((char)0xF8);
    display.print("C ");
  }

  display.print(getTrend(tank));
  display.print(" ");

  if (status == 2)      display.print("ERR");
  else if (status == 1) display.print("WRN");
  else                  display.print("OK ");

  display.print(" ");
  display.print((int)tanks[tank].outputPct);
  display.print("%");

  display.setCursor(0, yOffset + 10);
  display.print("SP:");
  display.print(tanks[tank].setpoint, 1);
  display.print("  ");
  display.print((char)0xEB);
  double err = tanks[tank].setpoint - tanks[tank].filteredTemp;
  if (err >= 0) display.print("+");
  display.print(err, 1);
}

// =============================================================================
// BUTTONS / SERIAL
// =============================================================================

void initButtons() {
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_SET_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
}

bool updateButtonPressed(ButtonState &btn) {
  bool reading = digitalRead(btn.pin);
  unsigned long now = millis();

  if (reading != btn.lastReading) {
    btn.lastChangeMs = now;
    btn.lastReading = reading;
  }

  if ((now - btn.lastChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (reading != btn.stableState) {
      btn.stableState = reading;

      // INPUT_PULLUP: pressed = LOW
      if (btn.stableState == LOW) {
        return true;
      }
    }
  }

  return false;
}

void handleButtons() {
  unsigned long now = millis();

  if (updateButtonPressed(btnSet)) {
    if (pendingSetSingleClick && (now - lastSetClickMs <= DOUBLE_CLICK_MS)) {
      // double click = switch selected tank
      pendingSetSingleClick = false;
      selectedTank = (selectedTank + 1) % TANK_COUNT;
    } else {
      // wait to see if another click comes in
      pendingSetSingleClick = true;
      lastSetClickMs = now;
    }
  }

  // if second click never comes, treat as single click
  if (pendingSetSingleClick && (now - lastSetClickMs > DOUBLE_CLICK_MS)) {
    pendingSetSingleClick = false;
    setMode = !setMode;
  }

  // Up/Down only change SP while in set mode
  if (setMode) {
    if (updateButtonPressed(btnUp)) {
      setTankSetpoint(selectedTank, tanks[selectedTank].setpoint + BUTTON_SP_STEP_C);
    }

    if (updateButtonPressed(btnDown)) {
      setTankSetpoint(selectedTank, tanks[selectedTank].setpoint - BUTTON_SP_STEP_C);
    }
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      serialLine.trim();
      if (serialLine.length() > 0) {
        processCommand(serialLine);
      }
      serialLine = "";
    } else {
      serialLine += c;

      // safety guard
      if (serialLine.length() > 64) {
        serialLine = "";
      }
    }
  }
}

void processCommand(String line) {
  // Supported:
  // GET
  // SP,0,37.0
  // SP,1,36.5

  if (line.equalsIgnoreCase("GET")) {
    Serial.print("SETPOINTS,");
    Serial.print(tanks[0].setpoint, 2);
    Serial.print(",");
    Serial.println(tanks[1].setpoint, 2);
    return;
  }

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);

  if (c1 < 0 || c2 < 0) {
    Serial.println("ERR,BAD_FORMAT");
    return;
  }

  String cmd = line.substring(0, c1);
  String tankStr = line.substring(c1 + 1, c2);
  String valueStr = line.substring(c2 + 1);

  cmd.trim();
  tankStr.trim();
  valueStr.trim();

  if (!cmd.equalsIgnoreCase("SP")) {
    Serial.println("ERR,UNKNOWN_CMD");
    return;
  }

  int tank = tankStr.toInt();
  double newSP = valueStr.toFloat();

  if (tank < 0 || tank >= TANK_COUNT) {
    Serial.println("ERR,BAD_TANK");
    return;
  }

  if (newSP < MIN_SETPOINT_C || newSP > MAX_SETPOINT_ALLOWED_C) {
    Serial.println("ERR,BAD_SP");
    return;
  }

  setTankSetpoint(tank, newSP);

  Serial.print("OK,");
  Serial.print(tank);
  Serial.print(",");
  Serial.println(tanks[tank].setpoint, 2);
}

// =============================================================================
// INIT
// =============================================================================

void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED not found");
    while (1) { }
  }
  display.clearDisplay();
  display.display();
  display.cp437(true);
}

void initSensors() {
  for (int i = 0; i < TANK_COUNT; i++) {
    tanks[i].rawTemp = readThermocoupleC(i);

    if (isValidTemp(tanks[i].rawTemp)) {
      tanks[i].filteredTemp  = tanks[i].rawTemp;
      tanks[i].prevTemp      = tanks[i].rawTemp;
      tanks[i].modelPrevTemp = tanks[i].rawTemp;
    } else {
      tanks[i].filteredTemp  = tanks[i].setpoint;
      tanks[i].prevTemp      = tanks[i].setpoint;
      tanks[i].modelPrevTemp = tanks[i].setpoint;
    }

    for (int j = 0; j < TREND_BUFFER_SIZE; j++) {
      tanks[i].tempHistory[j] = (float)tanks[i].filteredTemp;
    }
  }
}

void printLogHeader() {
  Serial.println("# tories_dual_tank v2.2 | Uno R4 Minima | adaptive self-tuning + GUI + buttons");
  Serial.println("# Commands: GET | SP,0,37.0 | SP,1,36.5");
  Serial.println("# Buttons: D2=UP, D3=SET(single toggle set mode, double switch tank), D4=DOWN");
  Serial.println("# OLED: > selected, * selected+set mode");
  Serial.println("# Traces: MainTemp,MainSP,MainRaw,MainOut%,MainKp,MainKi,MainKd,MainTau,ResTemp,ResSP,ResRaw,ResOut%,ResKp,ResKi,ResKd,ResTau");
}

// =============================================================================
// SENSOR READ
// =============================================================================

void readSensors() {
  for (int i = 0; i < TANK_COUNT; i++) {
    double raw = readThermocoupleC(i);
    tanks[i].rawTemp = raw;

    if (isValidTemp(raw)) {
      tanks[i].filteredTemp = EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * tanks[i].filteredTemp;

      tanks[i].tempHistory[tanks[i].historyIndex] = (float)tanks[i].filteredTemp;
      tanks[i].historyIndex = (tanks[i].historyIndex + 1) % TREND_BUFFER_SIZE;
      if (tanks[i].historyCount < TREND_BUFFER_SIZE) tanks[i].historyCount++;
    }
  }
}

// =============================================================================
// ONLINE MODEL IDENTIFICATION (RLS)
// =============================================================================

void updateRLSModel(int tank) {
  TankState &t = tanks[tank];

  if (!t.modelInitialized) {
    t.modelPrevTemp = t.filteredTemp;
    t.modelInitialized = true;
    return;
  }

  double phi[3];
  phi[0] = t.modelPrevTemp;
  phi[1] = t.prevModelOutputNorm;
  phi[2] = 1.0;

  double Pphi[3] = {0.0, 0.0, 0.0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Pphi[i] += t.P[i][j] * phi[j];
    }
  }

  double denom = RLS_FORGET;
  for (int i = 0; i < 3; i++) {
    denom += phi[i] * Pphi[i];
  }
  if (denom < 1e-9) return;

  double K[3];
  for (int i = 0; i < 3; i++) {
    K[i] = Pphi[i] / denom;
  }

  double yHat = 0.0;
  for (int i = 0; i < 3; i++) {
    yHat += t.theta[i] * phi[i];
  }

  double err = t.filteredTemp - yHat;

  for (int i = 0; i < 3; i++) {
    t.theta[i] += K[i] * err;
  }

  double phiTP[3] = {0.0, 0.0, 0.0};
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 3; i++) {
      phiTP[j] += phi[i] * t.P[i][j];
    }
  }

  double newP[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      newP[i][j] = (t.P[i][j] - K[i] * phiTP[j]) / RLS_FORGET;
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      t.P[i][j] = newP[i][j];
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = i + 1; j < 3; j++) {
      double s = 0.5 * (t.P[i][j] + t.P[j][i]);
      t.P[i][j] = s;
      t.P[j][i] = s;
    }
  }

  t.modelPrevTemp = t.filteredTemp;
}

// =============================================================================
// PERIODIC SELF-RETUNING
// =============================================================================

void retuneAdaptiveGains(int tank, unsigned long nowMs) {
  TankState &t = tanks[tank];

  if (!t.modelInitialized) return;
  if (nowMs < t.warmupUntilMs) return;
  if ((nowMs - t.lastRetuneMs) < RETUNE_INTERVAL) return;

  t.lastRetuneMs = nowMs;

  double a = t.theta[0];
  double b = t.theta[1];
  double Ts = CONTROL_INTERVAL / 1000.0;

  if (!(a > 0.80 && a < 0.99995 && b > 0.0005)) {
    t.modelValid = false;
    return;
  }

  double tauSec = -Ts / log(a);
  double Kproc  = b / (1.0 - a);

  if (!(tauSec > 2.0 && tauSec < 3600.0 && Kproc > 0.05 && Kproc < 500.0)) {
    t.modelValid = false;
    return;
  }

  double deadTimeSec = 1.0;
  if (deadTimeSec < Ts) deadTimeSec = Ts;

  double lambdaSec = 0.40 * tauSec;
  if (lambdaSec < 8.0) lambdaSec = 8.0;

  double Kc = tauSec / (Kproc * (lambdaSec + deadTimeSec));
  double Ti = tauSec + 0.5 * deadTimeSec;
  double Td = (tauSec * deadTimeSec) / (2.0 * tauSec + deadTimeSec);

  double kpNew = 100.0 * Kc;
  double kiNew = 100.0 * Kc / Ti;
  double kdNew = 100.0 * Kc * Td;

  kpNew = clampDouble(kpNew, t.kpMin, t.kpMax);
  kiNew = clampDouble(kiNew, t.kiMin, t.kiMax);
  kdNew = clampDouble(kdNew, t.kdMin, t.kdMax);

  const double BETA = 0.15;
  t.kp = (1.0 - BETA) * t.kp + BETA * kpNew;
  t.ki = (1.0 - BETA) * t.ki + BETA * kiNew;
  t.kd = (1.0 - BETA) * t.kd + BETA * kdNew;

  t.estTauSec = tauSec;
  t.estKproc  = Kproc;
  t.modelValid = true;
}

// =============================================================================
// ADAPTIVE CONTROL LAW
// =============================================================================

void computeAdaptiveOutput(int tank, double dtSec) {
  TankState &t = tanks[tank];

  if (dtSec <= 0.0) dtSec = CONTROL_INTERVAL / 1000.0;

  double error = t.setpoint - t.filteredTemp;

  double dTemp = (t.filteredTemp - t.prevTemp) / dtSec;
  t.dTempFilt = (1.0 - D_FILTER_ALPHA) * t.dTempFilt + D_FILTER_ALPHA * dTemp;

  double kpEff = t.kp;
  double kiEff = t.ki;
  double kdEff = t.kd;

  if (error > 3.0) {
    kpEff *= 1.20;
    kiEff *= 0.35;
    kdEff *= 0.45;
  } else if (error > 1.0) {
    kpEff *= 1.05;
    kiEff *= 0.80;
    kdEff *= 0.75;
  } else if (error >= -0.10) {
    kpEff *= 0.85;
    kiEff *= 1.05;
    kdEff *= 1.25;
  } else {
    kpEff *= 0.55;
    kiEff  = 0.0;
    kdEff *= 1.80;
  }

  if (t.filteredTemp > (t.setpoint - 0.20) && t.dTempFilt > 0.04) {
    kpEff *= 0.75;
    kiEff *= 0.50;
    kdEff *= 1.40;
  }

  double pTerm = kpEff * error;
  double dTerm = -kdEff * t.dTempFilt;

  if (fabs(error) < 6.0) {
    t.integralTerm += kiEff * error * dtSec;
  }

  t.integralTerm = clampDouble(t.integralTerm, INTEGRAL_MIN, INTEGRAL_MAX);

  double uUnsat = pTerm + t.integralTerm + dTerm;
  double u = clampDouble(uUnsat, 0.0, 100.0);

  if (u != uUnsat) {
    t.integralTerm += 0.25 * (u - uUnsat);
    t.integralTerm = clampDouble(t.integralTerm, INTEGRAL_MIN, INTEGRAL_MAX);
    u = clampDouble(pTerm + t.integralTerm + dTerm, 0.0, 100.0);
  }

  if (error < -0.30) {
    t.integralTerm *= 0.96;
  }

  t.outputPct = u;
  t.prevTemp = t.filteredTemp;
}

// =============================================================================
// CONTROL PIPELINE
// =============================================================================

void runControl() {
  unsigned long now = millis();

  for (int i = 0; i < TANK_COUNT; i++) {
    TankState &t = tanks[i];

    double dtSec;
    if (t.lastControlMs == 0) dtSec = CONTROL_INTERVAL / 1000.0;
    else dtSec = (now - t.lastControlMs) / 1000.0;
    if (dtSec <= 0.0) dtSec = CONTROL_INTERVAL / 1000.0;
    t.lastControlMs = now;

    bool hardFault = (!isValidTemp(t.rawTemp)) || (t.filteredTemp >= MAX_TEMP_C);

    if (hardFault) {
      t.outputPct = 0.0;
      t.integralTerm = 0.0;
      t.dTempFilt = 0.0;
      t.prevTemp = t.filteredTemp;
      t.prevModelOutputNorm = 0.0;
      t.modelPrevTemp = t.filteredTemp;
      continue;
    }

    updateRLSModel(i);
    retuneAdaptiveGains(i, now);
    computeAdaptiveOutput(i, dtSec);

    t.prevModelOutputNorm = t.outputPct / 100.0;
  }

  bool bothInBand = true;
  for (int i = 0; i < TANK_COUNT; i++) {
    double err = fabs(tanks[i].filteredTemp - tanks[i].setpoint);
    if (err > AT_SETPOINT_BAND || getStatus(i) == 2) {
      bothInBand = false;
      break;
    }
  }

  if (bothInBand) {
    unsigned long elapsed = now - lastAtSetpointCheck;
    if (elapsed >= 1000UL) {
      unsigned long secs = elapsed / 1000UL;
      timeAtSetpointSec += secs;
      if (timeAtSetpointSec > 359999UL) timeAtSetpointSec = 359999UL;
      lastAtSetpointCheck = now - (elapsed % 1000UL);
    }
    atSetpoint = true;
  } else {
    timeAtSetpointSec = 0;
    lastAtSetpointCheck = now;
    atSetpoint = false;
  }
}

void driveHeaters() {
  unsigned long now = millis();

  for (int i = 0; i < TANK_COUNT; i++) {
    while (now - tanks[i].windowStart >= WINDOW_SIZE) {
      tanks[i].windowStart += WINDOW_SIZE;
    }

    unsigned long onTime = (unsigned long)(WINDOW_SIZE * (tanks[i].outputPct / 100.0));
    bool interlock = (getStatus(i) == 2);

    tanks[i].heaterOn = (!interlock) && ((now - tanks[i].windowStart) < onTime);
    digitalWrite(HEAT_PINS[i], tanks[i].heaterOn ? HIGH : LOW);
  }
}

// =============================================================================
// DISPLAY + LOGGING
// =============================================================================

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  drawTankRow(0, 0);
  drawTankRow(1, 26);

  char timeBuf[9];

  unsigned long uptimeSec = (millis() - bootTime) / 1000UL;
  formatTime(uptimeSec, timeBuf);
  display.setCursor(0, 54);
  display.print((char)0x1E);
  display.print(timeBuf);

  formatTime(timeAtSetpointSec, timeBuf);
  display.setCursor(68, 54);
  display.print("S ");
  display.print(timeBuf);

  display.display();
}

void logData() {
  Serial.print("MainTemp:");
  Serial.print(tanks[0].filteredTemp, 2);
  Serial.print(",MainSP:");
  Serial.print(tanks[0].setpoint, 2);
  Serial.print(",MainRaw:");
  Serial.print(tanks[0].rawTemp, 2);
  Serial.print(",MainOut%:");
  Serial.print(tanks[0].outputPct, 2);
  Serial.print(",MainKp:");
  Serial.print(tanks[0].kp, 3);
  Serial.print(",MainKi:");
  Serial.print(tanks[0].ki, 3);
  Serial.print(",MainKd:");
  Serial.print(tanks[0].kd, 3);
  Serial.print(",MainTau:");
  Serial.print(tanks[0].estTauSec, 2);

  Serial.print(",ResTemp:");
  Serial.print(tanks[1].filteredTemp, 2);
  Serial.print(",ResSP:");
  Serial.print(tanks[1].setpoint, 2);
  Serial.print(",ResRaw:");
  Serial.print(tanks[1].rawTemp, 2);
  Serial.print(",ResOut%:");
  Serial.print(tanks[1].outputPct, 2);
  Serial.print(",ResKp:");
  Serial.print(tanks[1].kp, 3);
  Serial.print(",ResKi:");
  Serial.print(tanks[1].ki, 3);
  Serial.print(",ResKd:");
  Serial.print(tanks[1].kd, 3);
  Serial.print(",ResTau:");
  Serial.println(tanks[1].estTauSec, 2);
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(25);

  pinMode(HEAT_PIN_MAIN, OUTPUT);
  pinMode(HEAT_PIN_RES, OUTPUT);
  digitalWrite(HEAT_PIN_MAIN, LOW);
  digitalWrite(HEAT_PIN_RES, LOW);

  initButtons();

  initTankState(0, 37.0, 10.0, 0.04, 16.0);
  initTankState(1, 37.0, 12.0, 0.05, 18.0);

  initDisplay();
  delay(500);
  initSensors();

  unsigned long now = millis();
  bootTime = now;

  for (int i = 0; i < TANK_COUNT; i++) {
    tanks[i].windowStart         = now;
    tanks[i].lastControlMs       = now;
    tanks[i].lastRetuneMs        = now;
    tanks[i].warmupUntilMs       = now + WARMUP_TIME;
    tanks[i].prevModelOutputNorm = 0.0;
  }

  lastSensorRead      = now;
  lastControlRun      = now;
  lastDisplayUpdate   = now;
  lastLogOutput       = now;
  lastAtSetpointCheck = now;

  printLogHeader();
}

void loop() {
  unsigned long now = millis();

  handleSerialCommands();
  handleButtons();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
  }

  if (now - lastControlRun >= CONTROL_INTERVAL) {
    lastControlRun = now;
    runControl();
  }

  driveHeaters();

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    updateDisplay();
  }

  if (now - lastLogOutput >= LOG_INTERVAL) {
    lastLogOutput = now;
    logData();
  }
}