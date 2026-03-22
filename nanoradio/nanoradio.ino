#include <SPI.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <max6675.h>

// ============================================================================
// Remote Nano TFT + Radio + 2x MAX6675 + 3 Buttons
// Layout style from dashboard version, but kept on the non-flicker partial
// update structure from the working version.
// ============================================================================

// ---------------- LCD ----------------
#define TFT_CS   A1
#define TFT_DC   A2
#define TFT_RST  A3
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ---------------- Radio ----------------
RF24 radio(9, 10);   // CE, CSN
bool radioInitOk = false;
bool lastRadioSendOk = false;

// IMPORTANT: MUST MATCH RECEIVER
const byte RADIO_ADDRESS[6] = "00001";

struct Payload {
  float mainTempC;
  float mainSetpointC;
  float resTempC;
  float resSetpointC;
  uint8_t selectedTank;
  uint8_t setMode;
  uint8_t counter;
};

Payload txPayload;
uint8_t txCounter = 0;

// ---------------- MAX6675 ----------------
const int thermoSCK      = 5;
const int thermoCS_main  = 6;
const int thermoSO_main  = 7;

const int thermoCS_res   = 8;
const int thermoSO_res   = A0;

MAX6675 thermocoupleMain(thermoSCK, thermoCS_main, thermoSO_main);
MAX6675 thermocoupleRes(thermoSCK, thermoCS_res, thermoSO_res);

// ---------------- Buttons ----------------
const int BTN_UP_PIN   = 2;
const int BTN_SET_PIN  = 3;
const int BTN_DOWN_PIN = 4;

// ---------------- Constants ----------------
const float EMA_ALPHA               = 0.30f;
const float MIN_SETPOINT_C          = 20.0f;
const float MAX_SETPOINT_ALLOWED_C  = 39.0f;
const float BUTTON_SP_STEP_C        = 0.1f;

const float WARN_TEMP_C             = 39.0f;
const float MAX_TEMP_C              = 41.0f;
const float AT_SETPOINT_BAND_C      = 1.0f;
const float TREND_THRESHOLD_C       = 0.20f;

const unsigned long MAX6675_STARTUP_MS  = 500UL;
const unsigned long SENSOR_INTERVAL     = 300UL;
const unsigned long RADIO_INTERVAL      = 300UL;
const unsigned long SERIAL_INTERVAL     = 500UL;
const unsigned long DISPLAY_INTERVAL    = 120UL;
const unsigned long BUTTON_DEBOUNCE_MS  = 30UL;
const unsigned long DOUBLE_CLICK_MS     = 350UL;
const unsigned long BTN_MSG_HOLD_MS     = 1200UL;

const int HISTORY_LEN = 8;

// ---------------- Layout ----------------
const int SCREEN_W = 320;
const int SCREEN_H = 240;

const int HEADER_X = 4;
const int HEADER_Y = 4;
const int HEADER_W = 312;
const int HEADER_H = 24;

const int CARD_W   = 150;
const int CARD_H   = 146;
const int CARD_Y   = 34;
const int MAIN_X   = 6;
const int RES_X    = 164;

const int FOOT_X   = 4;
const int FOOT_Y   = 186;
const int FOOT_W   = 312;
const int FOOT_H   = 48;

// ---------------- State ----------------
struct TankState {
  float rawTemp;
  float filteredTemp;
  float setpoint;

  float history[HISTORY_LEN];
  int historyIndex;
  int historyCount;
};

TankState mainTank, resTank;

struct ButtonState {
  int pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
};

ButtonState btnUp   = {BTN_UP_PIN,   HIGH, HIGH, 0};
ButtonState btnSet  = {BTN_SET_PIN,  HIGH, HIGH, 0};
ButtonState btnDown = {BTN_DOWN_PIN, HIGH, HIGH, 0};

int selectedTank = 0;   // 0 = MAIN, 1 = RES
bool setMode = false;

bool pendingSetSingleClick = false;
unsigned long lastSetClickMs = 0;

bool sensorsReady = false;
unsigned long sensorsReadyAt = 0;
unsigned long lastSensorRead = 0;
unsigned long lastRadioSend = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long bootMs = 0;

unsigned long timeAtSetpointSec = 0;
unsigned long lastAtSetpointCheck = 0;
bool atSetpoint = false;

const char* lastButtonEvent = "NONE";
unsigned long lastButtonEventAt = 0;

bool displayDirty = true;

// ============================================================================
// Helpers
// ============================================================================
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool isValidTemp(float t) {
  return (!isnan(t) && t > -50.0f && t < 500.0f);
}

void setButtonEvent(const char* evt) {
  lastButtonEvent = evt;
  lastButtonEventAt = millis();
  displayDirty = true;
}

void deselectRadio() {
  digitalWrite(10, HIGH);
}

void deselectTFT() {
  digitalWrite(TFT_CS, HIGH);
}

void setTankSetpoint(int tank, float newSP) {
  newSP = clampFloat(newSP, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);
  if (tank == 0) mainTank.setpoint = newSP;
  else           resTank.setpoint  = newSP;
  displayDirty = true;
}

void formatTimeHHMMSS(unsigned long totalSec, char* buf) {
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec % 3600UL) / 60UL;
  unsigned long s = totalSec % 60UL;
  sprintf(buf, "%02lu:%02lu:%02lu", h, m, s);
}

void seedHistory(TankState &t, float value) {
  t.historyIndex = 0;
  t.historyCount = 0;
  for (int i = 0; i < HISTORY_LEN; i++) {
    t.history[i] = value;
  }
}

void pushHistory(TankState &t, float value) {
  t.history[t.historyIndex] = value;
  t.historyIndex = (t.historyIndex + 1) % HISTORY_LEN;
  if (t.historyCount < HISTORY_LEN) t.historyCount++;
}

float computeStdDev(const TankState &t) {
  if (t.historyCount < 2) return NAN;

  float mean = 0.0f;
  for (int i = 0; i < t.historyCount; i++) mean += t.history[i];
  mean /= (float)t.historyCount;

  float sumSq = 0.0f;
  for (int i = 0; i < t.historyCount; i++) {
    float d = t.history[i] - mean;
    sumSq += d * d;
  }

  return sqrtf(sumSq / (float)(t.historyCount - 1));
}

int getTrendCode(const TankState &t) {
  if (t.historyCount < HISTORY_LEN) return 0;

  int oldestIdx = t.historyIndex;
  int newestIdx = (t.historyIndex + HISTORY_LEN - 1) % HISTORY_LEN;
  float delta = t.history[newestIdx] - t.history[oldestIdx];

  if (delta > TREND_THRESHOLD_C) return 1;
  if (delta < -TREND_THRESHOLD_C) return -1;
  return 0;
}

const char* getTrendText(const TankState &t) {
  int tr = getTrendCode(t);
  if (tr > 0) return "UP";
  if (tr < 0) return "DN";
  return "FLAT";
}

uint16_t getTrendColor(const TankState &t) {
  int tr = getTrendCode(t);
  if (tr > 0) return ST77XX_GREEN;
  if (tr < 0) return ST77XX_CYAN;
  return ST77XX_WHITE;
}

int getStatusCode(const TankState &t) {
  if (!isValidTemp(t.rawTemp)) return 2;
  if (t.filteredTemp >= MAX_TEMP_C) return 2;
  if (t.filteredTemp >= WARN_TEMP_C) return 1;
  return 0;
}

const char* getStatusText(const TankState &t) {
  int s = getStatusCode(t);
  if (s == 2) return "ERR";
  if (s == 1) return "WRN";
  return "OK";
}

uint16_t getStatusColor(const TankState &t) {
  int s = getStatusCode(t);
  if (s == 2) return ST77XX_RED;
  if (s == 1) return ST77XX_YELLOW;
  return ST77XX_GREEN;
}

bool bothAtSetpoint() {
  if (!isValidTemp(mainTank.rawTemp) || !isValidTemp(resTank.rawTemp)) return false;

  bool mainOK = fabs(mainTank.filteredTemp - mainTank.setpoint) <= AT_SETPOINT_BAND_C;
  bool resOK  = fabs(resTank.filteredTemp  - resTank.setpoint)  <= AT_SETPOINT_BAND_C;

  return mainOK && resOK &&
         getStatusCode(mainTank) != 2 &&
         getStatusCode(resTank)  != 2;
}

void updateAtSetpointTimer() {
  unsigned long now = millis();
  bool nowInBand = bothAtSetpoint();

  if (nowInBand) {
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
      if (btn.stableState == LOW) {
        return true;
      }
    }
  }
  return false;
}

// ============================================================================
// TFT helpers
// ============================================================================
void clearField(int x, int y, int w, int h) {
  deselectRadio();
  tft.fillRect(x, y, w, h, ST77XX_BLACK);
}

void drawTankFrame(int tankIndex, int x, int y) {
  bool isSelected = (selectedTank == tankIndex);
  uint16_t borderColor = isSelected ? ST77XX_YELLOW : ST77XX_WHITE;
  uint16_t titleFill   = isSelected ? (setMode ? ST77XX_CYAN : ST77XX_MAGENTA) : ST77XX_BLUE;

  deselectRadio();

  tft.drawRoundRect(x, y, CARD_W, CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, y + 1, CARD_W - 2, 20, 8, titleFill);
  tft.fillRect(x + 2, y + 16, CARD_W - 4, CARD_H - 18, ST77XX_BLACK);

  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x + 8, y + 4);
  if (isSelected) tft.print(setMode ? "* " : "> ");
  else            tft.print("  ");
  tft.print(tankIndex == 0 ? "MAIN" : "RES");

  // static row labels
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(x + 8,  y + 86);  tft.print("SP:");
  tft.setCursor(x + 8,  y + 98);  tft.print("dT:");
  tft.setCursor(x + 8,  y + 110); tft.print("RAW:");
  tft.setCursor(x + 8,  y + 122); tft.print("SD:");

  tft.setCursor(x + 86, y + 86);  tft.print("TRD:");
  tft.setCursor(x + 86, y + 98);  tft.print("ST:");
}

void drawStaticUI() {
  deselectRadio();
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);

  // Header
  tft.fillRoundRect(HEADER_X, HEADER_Y, HEADER_W, HEADER_H, 6, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(12, 9);
  tft.print("REMOTE DASH");

  tft.setTextSize(1);
  tft.setCursor(195, 8);  tft.print("RADIO");
  tft.setCursor(255, 8);  tft.print("RUN");

  // Cards
  drawTankFrame(0, MAIN_X, CARD_Y);
  drawTankFrame(1, RES_X,  CARD_Y);

  // Footer
  tft.drawRoundRect(FOOT_X, FOOT_Y, FOOT_W, FOOT_H, 6, ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(10, 194);   tft.print("BTN");
  tft.setCursor(112, 194);  tft.print("SEL");
  tft.setCursor(190, 194);  tft.print("MODE");
  tft.setCursor(255, 194);  tft.print("ATSP");

  tft.setCursor(10, 216);   tft.print("MAIN");
  tft.setCursor(112, 216);  tft.print("RES");
  tft.setCursor(190, 216);  tft.print("BAND");
}

void drawHeaderRadioField() {
  clearField(194, 16, 55, 10);
  deselectRadio();
  tft.setTextSize(1);
  tft.setCursor(194, 16);

  if (!radioInitOk) {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO INIT");
  } else if (lastRadioSendOk) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("TX OK");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("TX FAIL");
  }
}

void drawHeaderRunField() {
  char buf[9];
  formatTimeHHMMSS((millis() - bootMs) / 1000UL, buf);

  clearField(254, 16, 58, 10);
  deselectRadio();
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(254, 16);
  tft.print(buf);
}

void drawTankHeaderDynamic(int tankIndex, int x, int y) {
  bool isSelected = (selectedTank == tankIndex);
  uint16_t borderColor = isSelected ? ST77XX_YELLOW : ST77XX_WHITE;
  uint16_t titleFill   = isSelected ? (setMode ? ST77XX_CYAN : ST77XX_MAGENTA) : ST77XX_BLUE;

  // redraw only header/top area + border so selection/mode style updates
  deselectRadio();
  tft.drawRoundRect(x, y, CARD_W, CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, y + 1, CARD_W - 2, 20, 8, titleFill);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, titleFill);
  tft.setCursor(x + 8, y + 4);
  if (isSelected) tft.print(setMode ? "* " : "> ");
  else            tft.print("  ");
  tft.print(tankIndex == 0 ? "MAIN" : "RES");
}

void drawTankValues(int x, int y, const TankState &t) {
  float err = t.setpoint - t.filteredTemp;
  float sd  = computeStdDev(t);

  // Big temperature
  clearField(x + 10, y + 28, 70, 40);
  deselectRadio();
  if (isValidTemp(t.rawTemp)) {
    tft.setTextSize(4);
    tft.setTextColor(getStatusColor(t));
    tft.setCursor(x + 10, y + 30);
    tft.print(t.filteredTemp, 1);
  } else {
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(x + 10, y + 34);
    tft.print("ERR");
  }

  tft.setTextSize(1);

  // SP
  clearField(x + 28, y + 86, 48, 9);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x + 28, y + 86);
  tft.print(t.setpoint, 1);

  // dT
  clearField(x + 28, y + 98, 48, 9);
  tft.setCursor(x + 28, y + 98);
  tft.setTextColor(ST77XX_WHITE);
  if (err >= 0) tft.print("+");
  tft.print(err, 1);

  // RAW
  clearField(x + 34, y + 110, 42, 9);
  tft.setCursor(x + 34, y + 110);
  if (isValidTemp(t.rawTemp)) {
    tft.setTextColor(ST77XX_WHITE);
    tft.print(t.rawTemp, 1);
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("ERR");
  }

  // SD
  clearField(x + 24, y + 122, 52, 9);
  tft.setCursor(x + 24, y + 122);
  tft.setTextColor(ST77XX_WHITE);
  if (isnan(sd)) tft.print("--");
  else tft.print(sd, 2);

  // Trend
  clearField(x + 116, y + 86, 28, 9);
  tft.setCursor(x + 116, y + 86);
  tft.setTextColor(getTrendColor(t));
  tft.print(getTrendText(t));

  // Status
  clearField(x + 108, y + 98, 36, 9);
  tft.setCursor(x + 108, y + 98);
  tft.setTextColor(getStatusColor(t));
  tft.print(getStatusText(t));
}

void drawFooterFields() {
  char atspBuf[9];
  formatTimeHHMMSS(timeAtSetpointSec, atspBuf);

  // BTN
  clearField(10, 204, 88, 10);
  deselectRadio();
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 204);
  tft.print(lastButtonEvent);

  // SEL
  clearField(112, 204, 60, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(112, 204);
  tft.print(selectedTank == 0 ? "MAIN" : "RES");

  // MODE
  clearField(190, 204, 48, 10);
  tft.setCursor(190, 204);
  tft.setTextColor(setMode ? ST77XX_CYAN : ST77XX_RED);
  tft.print(setMode ? "SET" : "VIEW");

  // ATSP
  clearField(255, 204, 55, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(255, 204);
  tft.print(atspBuf);

  // MAIN temp footer
  clearField(10, 226, 80, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 226);
  if (isValidTemp(mainTank.rawTemp)) tft.print(mainTank.filteredTemp, 1);
  else tft.print("ERR");

  // RES temp footer
  clearField(112, 226, 70, 10);
  tft.setCursor(112, 226);
  if (isValidTemp(resTank.rawTemp)) tft.print(resTank.filteredTemp, 1);
  else tft.print("ERR");

  // BAND
  clearField(190, 226, 48, 10);
  tft.setCursor(190, 226);
  tft.setTextColor(atSetpoint ? ST77XX_GREEN : ST77XX_RED);
  tft.print(atSetpoint ? "YES" : "NO");
}

void updateDynamicUI() {
  drawHeaderRadioField();
  drawHeaderRunField();

  drawTankHeaderDynamic(0, MAIN_X, CARD_Y);
  drawTankHeaderDynamic(1, RES_X,  CARD_Y);

  drawTankValues(MAIN_X, CARD_Y, mainTank);
  drawTankValues(RES_X,  CARD_Y, resTank);

  drawFooterFields();
}

// ============================================================================
// Tasks
// ============================================================================
void readSensors() {
  mainTank.rawTemp = thermocoupleMain.readCelsius();
  resTank.rawTemp  = thermocoupleRes.readCelsius();

  if (isValidTemp(mainTank.rawTemp)) {
    mainTank.filteredTemp = EMA_ALPHA * mainTank.rawTemp + (1.0f - EMA_ALPHA) * mainTank.filteredTemp;
    pushHistory(mainTank, mainTank.filteredTemp);
  }

  if (isValidTemp(resTank.rawTemp)) {
    resTank.filteredTemp = EMA_ALPHA * resTank.rawTemp + (1.0f - EMA_ALPHA) * resTank.filteredTemp;
    pushHistory(resTank, resTank.filteredTemp);
  }
}

void buttonTask() {
  unsigned long now = millis();

  // SET button: single click vs double click
  if (updateButtonPressed(btnSet)) {
    if (pendingSetSingleClick && (now - lastSetClickMs <= DOUBLE_CLICK_MS)) {
      pendingSetSingleClick = false;
      selectedTank = (selectedTank + 1) % 2;
      setButtonEvent("SET DBL");
    } else {
      pendingSetSingleClick = true;
      lastSetClickMs = now;
    }
  }

  // single click = toggle SET/VIEW
  if (pendingSetSingleClick && (now - lastSetClickMs > DOUBLE_CLICK_MS)) {
    pendingSetSingleClick = false;
    setMode = !setMode;
    setButtonEvent("SET SGL");
  }

  if (updateButtonPressed(btnUp)) {
    if (setMode) {
      if (selectedTank == 0) setTankSetpoint(0, mainTank.setpoint + BUTTON_SP_STEP_C);
      else                   setTankSetpoint(1, resTank.setpoint  + BUTTON_SP_STEP_C);
    }
    setButtonEvent("UP");
  }

  if (updateButtonPressed(btnDown)) {
    if (setMode) {
      if (selectedTank == 0) setTankSetpoint(0, mainTank.setpoint - BUTTON_SP_STEP_C);
      else                   setTankSetpoint(1, resTank.setpoint  - BUTTON_SP_STEP_C);
    }
    setButtonEvent("DOWN");
  }

  if ((strcmp(lastButtonEvent, "NONE") != 0) &&
      (millis() - lastButtonEventAt > BTN_MSG_HOLD_MS)) {
    lastButtonEvent = "NONE";
    displayDirty = true;
  }
}

void sensorTask() {
  unsigned long now = millis();

  if (!sensorsReady) {
    if (now >= sensorsReadyAt) {
      sensorsReady = true;
      readSensors();

      if (!isValidTemp(mainTank.rawTemp)) mainTank.filteredTemp = 37.0f;
      if (!isValidTemp(resTank.rawTemp))  resTank.filteredTemp  = 37.0f;

      seedHistory(mainTank, mainTank.filteredTemp);
      seedHistory(resTank,  resTank.filteredTemp);

      lastAtSetpointCheck = now;
      updateAtSetpointTimer();
      displayDirty = true;
    }
    return;
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    updateAtSetpointTimer();
    displayDirty = true;
  }
}

void radioTask() {
  unsigned long now = millis();
  if (!radioInitOk || !sensorsReady) return;
  if (now - lastRadioSend < RADIO_INTERVAL) return;

  lastRadioSend = now;

  txPayload.mainTempC = mainTank.filteredTemp;
  txPayload.mainSetpointC = mainTank.setpoint;
  txPayload.resTempC = resTank.filteredTemp;
  txPayload.resSetpointC = resTank.setpoint;
  txPayload.selectedTank = (uint8_t)selectedTank;
  txPayload.setMode = setMode ? 1 : 0;
  txPayload.counter = txCounter++;

  deselectTFT();
  lastRadioSendOk = radio.write(&txPayload, sizeof(txPayload));
  displayDirty = true;
}

void serialTask() {
  unsigned long now = millis();
  if (now - lastSerialPrint < SERIAL_INTERVAL) return;
  lastSerialPrint = now;

  Serial.print("MAIN T=");
  Serial.print(mainTank.filteredTemp, 2);
  Serial.print(" SP=");
  Serial.print(mainTank.setpoint, 2);
  Serial.print(" SD=");
  {
    float sd0 = computeStdDev(mainTank);
    if (isnan(sd0)) Serial.print("NA");
    else Serial.print(sd0, 3);
  }

  Serial.print(" | RES T=");
  Serial.print(resTank.filteredTemp, 2);
  Serial.print(" SP=");
  Serial.print(resTank.setpoint, 2);
  Serial.print(" SD=");
  {
    float sd1 = computeStdDev(resTank);
    if (isnan(sd1)) Serial.print("NA");
    else Serial.print(sd1, 3);
  }

  Serial.print(" | RADIO_INIT=");
  Serial.print(radioInitOk ? "YES" : "NO");
  Serial.print(" | TX=");
  Serial.println(lastRadioSendOk ? "OK" : "FAIL");
}

void displayTask() {
  unsigned long now = millis();
  if (!displayDirty) return;
  if (now - lastDisplayUpdate < DISPLAY_INTERVAL) return;

  lastDisplayUpdate = now;
  updateDynamicUI();
  displayDirty = false;
}

// ============================================================================
// Setup / Loop
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_SET_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  SPI.begin();

  tft.init(240, 320);
  tft.setRotation(1);   // landscape
  tft.setTextWrap(false);

  mainTank.setpoint = 37.0f;
  resTank.setpoint  = 37.0f;

  mainTank.rawTemp = NAN;
  resTank.rawTemp = NAN;
  mainTank.filteredTemp = 37.0f;
  resTank.filteredTemp  = 37.0f;

  seedHistory(mainTank, 37.0f);
  seedHistory(resTank,  37.0f);

  bootMs = millis();
  lastAtSetpointCheck = bootMs;

  drawStaticUI();
  updateDynamicUI();

  sensorsReadyAt = millis() + MAX6675_STARTUP_MS;

  deselectTFT();
  radioInitOk = radio.begin();
  if (radioInitOk) {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.setRetries(5, 15);
    radio.openWritingPipe(RADIO_ADDRESS);
    radio.stopListening();
  }

  displayDirty = true;
  Serial.println("Remote TFT dashboard ready");
}

void loop() {
  buttonTask();
  sensorTask();
  radioTask();
  serialTask();
  displayTask();
}