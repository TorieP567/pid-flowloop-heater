#include <Arduino.h>
#line 1 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\firmware\\remote_box\\remote_box.ino"
// =============================================================================
// REMOTE BOX
// Arduino Nano handheld UI / sensor / radio node for the dual-tank heater.
//
// Responsibilities:
//   - read two MAX6675 thermocouples
//   - manage three buttons with responsive short/long press handling
//   - render the ST7789 TFT UI
//   - send temperatures + requested setpoints to the main box over nRF24L01
//   - receive authoritative controller status back from the main box
//
// Important:
//   - this board is NOT the final heater-control authority
//   - the main-box Uno R4 remains responsible for both SSR heater outputs
//   - this firmware reuses the shared packet contract via the local system_packets.h
//
// Hardware notes:
//   - the nRF24, ST7789, and MAX6675 modules share SPI-related lines
//   - always keep unselected chip-select lines HIGH
//   - place a 10 uF capacitor across nRF24 VCC/GND close to the radio module
//
// Pin map:
//   Radio:   CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13
//   MAX6675: MAIN CS=D6, RES CS=D7, SO=D12, SCK=D13
//   ST7789:  CS=D4, DC=D5, RST=D8, MOSI=D11, SCK=D13
//   Buttons: UP=A0, SET=A1, DOWN=A2, all wired to GND with INPUT_PULLUP
// =============================================================================

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <max6675.h>

#include "system_packets.h"

namespace {

#ifndef REMOTE_BOX_ENABLE_SERIAL_LOG
#define REMOTE_BOX_ENABLE_SERIAL_LOG 0
#endif

constexpr uint8_t TANK_COUNT = 2;
constexpr uint8_t TANK_MAIN = 0;
constexpr uint8_t TANK_RES = 1;

constexpr uint8_t RADIO_CE_PIN = 9;
constexpr uint8_t RADIO_CSN_PIN = 10;

constexpr uint8_t THERMO_SCK_PIN = 13;
constexpr uint8_t THERMO_SO_PIN = 12;
constexpr uint8_t THERMO_CS_MAIN_PIN = 6;
constexpr uint8_t THERMO_CS_RES_PIN = 7;

constexpr uint8_t TFT_CS_PIN = 4;
constexpr uint8_t TFT_DC_PIN = 5;
constexpr uint8_t TFT_RST_PIN = 8;

constexpr uint8_t BTN_UP_PIN = A0;
constexpr uint8_t BTN_SET_PIN = A1;
constexpr uint8_t BTN_DOWN_PIN = A2;

constexpr float DEFAULT_SETPOINT_C = 37.0f;
constexpr float MIN_SETPOINT_C = 20.0f;
constexpr float MAX_SETPOINT_ALLOWED_C = 39.0f;
constexpr float SETPOINT_STEP_C = 0.1f;

constexpr float MIN_REASONABLE_TEMP_C = -20.0f;
constexpr float MAX_REASONABLE_TEMP_C = 120.0f;
constexpr float WARN_TEMP_C = 39.0f;
constexpr float HARD_MAX_TEMP_C = 41.0f;

constexpr unsigned long MAX6675_STARTUP_MS = 500UL;
constexpr unsigned long SENSOR_INTERVAL_MS = 250UL;
constexpr unsigned long RADIO_TX_INTERVAL_MS = 150UL;
constexpr unsigned long DISPLAY_INTERVAL_MS = 100UL;
constexpr unsigned long LOG_INTERVAL_MS = 500UL;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30UL;
constexpr unsigned long SET_LONG_PRESS_MS = 700UL;
constexpr unsigned long SCREEN_TOGGLE_HOLD_MS = 900UL;
constexpr unsigned long STATUS_DEGRADED_MS = 600UL;
constexpr unsigned long STATUS_TIMEOUT_MS = 1500UL;

constexpr uint16_t COLOR_BG = ST77XX_BLACK;
constexpr uint16_t COLOR_PANEL = 0x1084;
constexpr uint16_t COLOR_PANEL_ALT = 0x18C6;
constexpr uint16_t COLOR_DIM = 0x8410;
constexpr uint16_t COLOR_ACCENT = ST77XX_CYAN;
constexpr uint16_t COLOR_OK = ST77XX_GREEN;
constexpr uint16_t COLOR_WARN = ST77XX_YELLOW;
constexpr uint16_t COLOR_FAULT = ST77XX_RED;
constexpr uint16_t COLOR_LOCAL = 0x7D7C;
constexpr uint16_t COLOR_TEXT = ST77XX_WHITE;

constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

constexpr int HEADER_X = 0;
constexpr int HEADER_Y = 0;
constexpr int HEADER_W = SCREEN_W;
constexpr int HEADER_H = 30;

constexpr int CARD_W = 148;
constexpr int CARD_H = 138;
constexpr int CARD_Y = 34;
constexpr int MAIN_X = 8;
constexpr int RES_X = 164;

constexpr int FOOT_Y = 180;
constexpr int FOOT_H = 60;

enum ScreenMode : uint8_t {
  SCREEN_MODE_MAIN = 0,
  SCREEN_MODE_DEBUG = 1
};

enum LinkState : uint8_t {
  LINK_STATE_NO_RADIO = 0,
  LINK_STATE_WAITING = 1,
  LINK_STATE_OK = 2,
  LINK_STATE_DEGRADED = 3,
  LINK_STATE_TIMEOUT = 4
};

enum SourceLabel : uint8_t {
  SOURCE_LABEL_ERR = 0,
  SOURCE_LABEL_LOCAL = 1,
  SOURCE_LABEL_LAST = 2,
  SOURCE_LABEL_AUTH = 3
};

struct ButtonTracker {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
  unsigned long pressedAtMs;
  bool longHandled;
};

struct TankLocalState {
  float rawTempC;
  bool valid;
  float requestedSetpointC;
};

struct TextFieldCache {
  char text[32];
  uint16_t color;
  bool valid;
};

RF24 radioHw(RADIO_CE_PIN, RADIO_CSN_PIN);
Adafruit_ST7789 tft(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
MAX6675 thermocoupleMain(THERMO_SCK_PIN, THERMO_CS_MAIN_PIN, THERMO_SO_PIN);
MAX6675 thermocoupleRes(THERMO_SCK_PIN, THERMO_CS_RES_PIN, THERMO_SO_PIN);

TankLocalState localTanks[TANK_COUNT];

ButtonTracker btnUp = {BTN_UP_PIN, HIGH, HIGH, 0, 0, false};
ButtonTracker btnSet = {BTN_SET_PIN, HIGH, HIGH, 0, 0, false};
ButtonTracker btnDown = {BTN_DOWN_PIN, HIGH, HIGH, 0, 0, false};

ScreenMode screenMode = SCREEN_MODE_MAIN;
uint8_t selectedTank = TANK_MAIN;
bool editMode = false;
bool displayNeedsFullRedraw = true;

bool radioInitOk = false;
bool lastTxOk = false;
bool haveMainStatus = false;
bool sensorsReady = false;
bool screenToggleComboHandled = false;
unsigned long screenToggleComboStartMs = 0;

RemoteToMainPacket lastOutboundPacket = {};
MainToRemotePacket latestMainStatus = {};

uint16_t txSequence = 0;
uint16_t lastStatusSequence = 0;
uint8_t pendingButtonFlags = 0;

unsigned long bootMs = 0;
unsigned long sensorsReadyAtMs = 0;
unsigned long lastSensorReadMs = 0;
unsigned long lastRadioTxMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastStatusRxMs = 0;

unsigned long txOkCount = 0;
unsigned long txFailCount = 0;
unsigned long rxOkCount = 0;
unsigned long rxBadCount = 0;

#if REMOTE_BOX_ENABLE_SERIAL_LOG
constexpr unsigned long LOG_INTERVAL_MS = 500UL;
unsigned long lastLogMs = 0;
LinkState lastLoggedLinkState = LINK_STATE_WAITING;
#endif

TextFieldCache mainLinkCache = {};
TextFieldCache mainModeCache = {};
TextFieldCache mainFaultCache = {};
TextFieldCache mainInfoCache = {};
TextFieldCache mainTempCache[TANK_COUNT] = {};
TextFieldCache mainSourceCache[TANK_COUNT] = {};
TextFieldCache mainReqCache[TANK_COUNT] = {};
TextFieldCache mainActCache[TANK_COUNT] = {};
TextFieldCache mainOutCache[TANK_COUNT] = {};

TextFieldCache debugLineCache[10] = {};
int8_t mainFrameSelectedTank = -1;
bool mainFrameEditMode = false;

void copyFlashString(char* out, size_t outLen, PGM_P text) {
  if (outLen == 0) return;
  strncpy_P(out, text, outLen - 1);
  out[outLen - 1] = '\0';
}

PGM_P tankLabelProgmem(uint8_t tank) {
  return (tank == TANK_MAIN) ? PSTR("MAIN") : PSTR("RES");
}

PGM_P linkStateTextProgmem(LinkState state) {
  switch (state) {
    case LINK_STATE_NO_RADIO: return PSTR("NO RADIO");
    case LINK_STATE_WAITING: return PSTR("WAITING");
    case LINK_STATE_OK: return PSTR("LINK OK");
    case LINK_STATE_DEGRADED: return PSTR("DEGRADED");
    case LINK_STATE_TIMEOUT: return PSTR("TIMEOUT");
    default: return PSTR("UNKNOWN");
  }
}

PGM_P sourceLabelTextProgmem(SourceLabel label) {
  switch (label) {
    case SOURCE_LABEL_AUTH: return PSTR("AUTH");
    case SOURCE_LABEL_LAST: return PSTR("LAST");
    case SOURCE_LABEL_LOCAL: return PSTR("LOCAL");
    case SOURCE_LABEL_ERR:
    default: return PSTR("ERR");
  }
}

PGM_P onOffTextProgmem(bool on) {
  return on ? PSTR("ON") : PSTR("OFF");
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool isReasonableTemp(float tempC) {
  return !isnan(tempC) && tempC > MIN_REASONABLE_TEMP_C && tempC < MAX_REASONABLE_TEMP_C;
}

void prepareForRadio() {
  digitalWrite(TFT_CS_PIN, HIGH);
  digitalWrite(THERMO_CS_MAIN_PIN, HIGH);
  digitalWrite(THERMO_CS_RES_PIN, HIGH);
}

void prepareForTft() {
  digitalWrite(RADIO_CSN_PIN, HIGH);
  digitalWrite(THERMO_CS_MAIN_PIN, HIGH);
  digitalWrite(THERMO_CS_RES_PIN, HIGH);
}

void prepareForSensors() {
  digitalWrite(RADIO_CSN_PIN, HIGH);
  digitalWrite(TFT_CS_PIN, HIGH);
}

void invalidateField(TextFieldCache& cache) {
  cache.text[0] = '\0';
  cache.color = 0;
  cache.valid = false;
}

void invalidateAllCaches() {
  invalidateField(mainLinkCache);
  invalidateField(mainModeCache);
  invalidateField(mainFaultCache);
  invalidateField(mainInfoCache);
  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    invalidateField(mainTempCache[tank]);
    invalidateField(mainSourceCache[tank]);
    invalidateField(mainReqCache[tank]);
    invalidateField(mainActCache[tank]);
    invalidateField(mainOutCache[tank]);
  }
  for (uint8_t index = 0; index < 10; ++index) {
    invalidateField(debugLineCache[index]);
  }
  mainFrameSelectedTank = -1;
  mainFrameEditMode = false;
}

void updateTextField(TextFieldCache& cache, const char* text, uint16_t color,
                     int16_t x, int16_t y, int16_t w, int16_t h,
                     uint8_t textSize, uint16_t bgColor = COLOR_BG) {
  if (w <= 0 || h <= 0) return;

  if (!cache.valid || cache.color != color || strncmp(cache.text, text, sizeof(cache.text)) != 0) {
    prepareForTft();
    tft.fillRect(x, y, w, h, bgColor);
    tft.setTextSize(textSize);
    tft.setTextColor(color, bgColor);
    tft.setCursor(x, y);
    tft.print(text);

    strncpy(cache.text, text, sizeof(cache.text) - 1);
    cache.text[sizeof(cache.text) - 1] = '\0';
    cache.color = color;
    cache.valid = true;
  }
}

void formatFixed1(char* out, size_t outLen, float value) {
  char temp[16];
  dtostrf(value, 0, 1, temp);

  char* start = temp;
  while (*start == ' ') ++start;

  strncpy(out, start, outLen - 1);
  out[outLen - 1] = '\0';
}

void formatTemp(char* out, size_t outLen, float tempC, bool valid) {
  if (!valid || isnan(tempC)) {
    copyFlashString(out, outLen, PSTR("ERR"));
    return;
  }

  char temp[12];
  formatFixed1(temp, sizeof(temp), tempC);
  snprintf_P(out, outLen, PSTR("%s C"), temp);
}

void formatShortTemp(char* out, size_t outLen, float tempC, bool valid) {
  if (!valid || isnan(tempC)) {
    copyFlashString(out, outLen, PSTR("--.-"));
    return;
  }
  formatFixed1(out, outLen, tempC);
}

void formatTimeHHMMSS(char* out, size_t outLen, unsigned long totalSec) {
  if (totalSec > 359999UL) totalSec = 359999UL;
  unsigned long hours = totalSec / 3600UL;
  unsigned long mins = (totalSec % 3600UL) / 60UL;
  unsigned long secs = totalSec % 60UL;
  snprintf_P(out, outLen, PSTR("%02lu:%02lu:%02lu"), hours, mins, secs);
}

bool statusPacketFresh(unsigned long nowMs) {
  return haveMainStatus && ((nowMs - lastStatusRxMs) <= STATUS_TIMEOUT_MS);
}

bool statusPacketHealthy(unsigned long nowMs) {
  return haveMainStatus && ((nowMs - lastStatusRxMs) <= STATUS_DEGRADED_MS);
}

LinkState currentLinkState(unsigned long nowMs) {
  if (!radioInitOk) return LINK_STATE_NO_RADIO;
  if (!haveMainStatus) return LINK_STATE_WAITING;
  unsigned long age = nowMs - lastStatusRxMs;
  if (age <= STATUS_DEGRADED_MS) return LINK_STATE_OK;
  if (age <= STATUS_TIMEOUT_MS) return LINK_STATE_DEGRADED;
  return LINK_STATE_TIMEOUT;
}

uint16_t linkStateColor(LinkState state) {
  switch (state) {
    case LINK_STATE_OK: return COLOR_OK;
    case LINK_STATE_DEGRADED: return COLOR_WARN;
    case LINK_STATE_WAITING: return COLOR_WARN;
    case LINK_STATE_NO_RADIO:
    case LINK_STATE_TIMEOUT:
    default: return COLOR_FAULT;
  }
}

float activeSetpointFromMain(uint8_t tank, bool& valid) {
  if (!haveMainStatus) {
    valid = false;
    return NAN;
  }
  valid = true;
  return decodeTempCx100(tank == TANK_MAIN ? latestMainStatus.mainSetpointCx100
                                           : latestMainStatus.resSetpointCx100);
}

float outputPctFromMain(uint8_t tank, bool& valid) {
  if (!haveMainStatus) {
    valid = false;
    return NAN;
  }
  valid = true;
  return decodeOutputPermille(tank == TANK_MAIN ? latestMainStatus.mainOutputPermille
                                                : latestMainStatus.resOutputPermille);
}

bool heaterOnFromMain(uint8_t tank) {
  if (!haveMainStatus) return false;
  uint8_t bit = (tank == TANK_MAIN) ? HEATER_FLAG_MAIN_ON : HEATER_FLAG_RES_ON;
  return (latestMainStatus.heaterFlags & bit) != 0U;
}

uint16_t tankFaultMaskInvalid(uint8_t tank) {
  return (tank == TANK_MAIN) ? FAULT_MAIN_SENSOR_INVALID : FAULT_RES_SENSOR_INVALID;
}

uint16_t tankFaultMaskWarn(uint8_t tank) {
  return (tank == TANK_MAIN) ? FAULT_MAIN_WARN : FAULT_RES_WARN;
}

uint16_t tankFaultMaskCut(uint8_t tank) {
  return (tank == TANK_MAIN) ? FAULT_MAIN_OVERTEMP : FAULT_RES_OVERTEMP;
}

float displayTempForTank(uint8_t tank, unsigned long nowMs, bool& valid, uint16_t& color, SourceLabel& sourceLabel) {
  if (haveMainStatus) {
    float filteredTemp = decodeTempCx100(tank == TANK_MAIN ? latestMainStatus.mainFilteredCx100
                                                           : latestMainStatus.resFilteredCx100);
    if (!isnan(filteredTemp)) {
      if (statusPacketHealthy(nowMs)) {
        sourceLabel = SOURCE_LABEL_AUTH;
        if ((latestMainStatus.faultFlags & tankFaultMaskCut(tank)) != 0U) color = COLOR_FAULT;
        else if ((latestMainStatus.faultFlags & tankFaultMaskWarn(tank)) != 0U) color = COLOR_WARN;
        else color = COLOR_OK;
        valid = true;
        return filteredTemp;
      }

      if (statusPacketFresh(nowMs)) {
        sourceLabel = SOURCE_LABEL_LAST;
        color = COLOR_WARN;
        valid = true;
        return filteredTemp;
      }
    }
  }

  if (localTanks[tank].valid) {
    sourceLabel = SOURCE_LABEL_LOCAL;
    color = COLOR_LOCAL;
    valid = true;
    return localTanks[tank].rawTempC;
  }

  sourceLabel = SOURCE_LABEL_ERR;
  color = COLOR_FAULT;
  valid = false;
  return NAN;
}

void logEvent(PGM_P text) {
#if REMOTE_BOX_ENABLE_SERIAL_LOG
  Serial.println(reinterpret_cast<const __FlashStringHelper*>(text));
#else
  (void)text;
#endif
}

void logLinkTransitions() {
#if REMOTE_BOX_ENABLE_SERIAL_LOG
  LinkState link = currentLinkState(millis());
  if (link != lastLoggedLinkState) {
    lastLoggedLinkState = link;
    switch (link) {
      case LINK_STATE_OK: logEvent(PSTR("Link restored")); break;
      case LINK_STATE_DEGRADED: logEvent(PSTR("Link degraded")); break;
      case LINK_STATE_TIMEOUT: logEvent(PSTR("Link timeout")); break;
      case LINK_STATE_WAITING: logEvent(PSTR("Waiting for main-box status")); break;
      case LINK_STATE_NO_RADIO: logEvent(PSTR("Radio hardware not initialized")); break;
      default: break;
    }
  }
#endif
}

void initButtons() {
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_SET_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
}

void initSpiChipSelects() {
  pinMode(TFT_CS_PIN, OUTPUT);
  pinMode(RADIO_CSN_PIN, OUTPUT);
  pinMode(THERMO_CS_MAIN_PIN, OUTPUT);
  pinMode(THERMO_CS_RES_PIN, OUTPUT);

  digitalWrite(TFT_CS_PIN, HIGH);
  digitalWrite(RADIO_CSN_PIN, HIGH);
  digitalWrite(THERMO_CS_MAIN_PIN, HIGH);
  digitalWrite(THERMO_CS_RES_PIN, HIGH);
}

void initDisplay() {
  prepareForTft();
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(COLOR_BG);
  tft.setTextWrap(false);
  displayNeedsFullRedraw = true;
}

void initRadio() {
  prepareForRadio();
  radioInitOk = radioHw.begin();
  if (!radioInitOk) {
    lastTxOk = false;
    return;
  }

  radioHw.setPALevel(RF24_PA_LOW);
  radioHw.setDataRate(RF24_250KBPS);
  radioHw.setChannel(SYSTEM_RADIO_CHANNEL);
  radioHw.setAutoAck(false);
  radioHw.openWritingPipe(SYSTEM_PIPE_REMOTE_TO_MAIN);
  radioHw.openReadingPipe(1, SYSTEM_PIPE_MAIN_TO_REMOTE);
  radioHw.startListening();
}

bool updateButtonState(ButtonTracker& button, bool& pressedEvent, bool& releasedEvent) {
  pressedEvent = false;
  releasedEvent = false;

  bool reading = digitalRead(button.pin);
  unsigned long nowMs = millis();

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeMs = nowMs;
  }

  if ((nowMs - button.lastChangeMs) > BUTTON_DEBOUNCE_MS && reading != button.stableState) {
    button.stableState = reading;
    if (button.stableState == LOW) {
      button.pressedAtMs = nowMs;
      button.longHandled = false;
      pressedEvent = true;
    } else {
      releasedEvent = true;
    }
    return true;
  }

  return false;
}

void toggleScreenMode() {
  screenMode = (screenMode == SCREEN_MODE_MAIN) ? SCREEN_MODE_DEBUG : SCREEN_MODE_MAIN;
  displayNeedsFullRedraw = true;
  if (screenMode == SCREEN_MODE_MAIN) logEvent(PSTR("Screen: MAIN"));
  else logEvent(PSTR("Screen: DEBUG"));
}

void adjustSelectedSetpoint(float deltaC) {
  uint8_t tank = selectedTank;
  float current = localTanks[tank].requestedSetpointC;
  float updated = clampFloat(current + deltaC, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);
  if (fabsf(updated - current) >= 0.001f) {
    localTanks[tank].requestedSetpointC = updated;
    displayNeedsFullRedraw = false;
  }
}

void handleButtons() {
  bool upPressed = false, upReleased = false;
  bool setPressed = false, setReleased = false;
  bool downPressed = false, downReleased = false;

  updateButtonState(btnUp, upPressed, upReleased);
  updateButtonState(btnSet, setPressed, setReleased);
  updateButtonState(btnDown, downPressed, downReleased);

  unsigned long nowMs = millis();
  bool comboActive = (btnUp.stableState == LOW) && (btnDown.stableState == LOW);
  if (comboActive) {
    if (screenToggleComboStartMs == 0) screenToggleComboStartMs = nowMs;
    if (!screenToggleComboHandled && (nowMs - screenToggleComboStartMs) >= SCREEN_TOGGLE_HOLD_MS) {
      screenToggleComboHandled = true;
      toggleScreenMode();
    }
  } else {
    screenToggleComboStartMs = 0;
    screenToggleComboHandled = false;
  }

  if (btnSet.stableState == LOW && !btnSet.longHandled && (nowMs - btnSet.pressedAtMs) >= SET_LONG_PRESS_MS) {
    btnSet.longHandled = true;
    editMode = !editMode;
    pendingButtonFlags |= REMOTE_BUTTON_SET;
    displayNeedsFullRedraw = false;
    logEvent(editMode ? PSTR("Edit mode ON") : PSTR("Edit mode OFF"));
  }

  if (setReleased && !btnSet.longHandled) {
    selectedTank = (selectedTank == TANK_MAIN) ? TANK_RES : TANK_MAIN;
    pendingButtonFlags |= REMOTE_BUTTON_SET;
    displayNeedsFullRedraw = false;
    logEvent(selectedTank == TANK_MAIN ? PSTR("Selected tank: MAIN") : PSTR("Selected tank: RES"));
  }

  if (!comboActive && editMode) {
    if (upPressed && btnDown.stableState != LOW) {
      adjustSelectedSetpoint(SETPOINT_STEP_C);
      pendingButtonFlags |= REMOTE_BUTTON_UP;
    }
    if (downPressed && btnUp.stableState != LOW) {
      adjustSelectedSetpoint(-SETPOINT_STEP_C);
      pendingButtonFlags |= REMOTE_BUTTON_DOWN;
    }
  }
}

void readSensors() {
  unsigned long nowMs = millis();
  if (!sensorsReady) {
    if (nowMs >= sensorsReadyAtMs) {
      sensorsReady = true;
    } else {
      return;
    }
  }

  if ((nowMs - lastSensorReadMs) < SENSOR_INTERVAL_MS) return;
  lastSensorReadMs = nowMs;

  prepareForSensors();
  float mainTemp = thermocoupleMain.readCelsius();
  prepareForSensors();
  float resTemp = thermocoupleRes.readCelsius();

  localTanks[TANK_MAIN].valid = isReasonableTemp(mainTemp);
  localTanks[TANK_MAIN].rawTempC = localTanks[TANK_MAIN].valid ? mainTemp : NAN;

  localTanks[TANK_RES].valid = isReasonableTemp(resTemp);
  localTanks[TANK_RES].rawTempC = localTanks[TANK_RES].valid ? resTemp : NAN;
}

void pollRadioDownlink() {
  if (!radioInitOk) return;

  MainToRemotePacket packet = {};
  bool sawPacket = false;

  prepareForRadio();
  while (radioHw.available()) {
    radioHw.read(&packet, sizeof(packet));
    sawPacket = true;
  }

  if (!sawPacket) return;

  if (!validatePacket(packet)) {
    ++rxBadCount;
    return;
  }

  latestMainStatus = packet;
  haveMainStatus = true;
  lastStatusRxMs = millis();
  lastStatusSequence = packet.statusSequence;
  ++rxOkCount;
}

void sendRadioUplink() {
  unsigned long nowMs = millis();
  if (!radioInitOk) return;
  if ((nowMs - lastRadioTxMs) < RADIO_TX_INTERVAL_MS) return;
  lastRadioTxMs = nowMs;

  RemoteToMainPacket packet = {};
  packet.sequence = ++txSequence;
  packet.remoteMillis = nowMs;
  packet.mainTempCx100 = encodeTempCx100(localTanks[TANK_MAIN].rawTempC);
  packet.resTempCx100 = encodeTempCx100(localTanks[TANK_RES].rawTempC);
  packet.mainSetpointCx100 = encodeTempCx100(localTanks[TANK_MAIN].requestedSetpointC);
  packet.resSetpointCx100 = encodeTempCx100(localTanks[TANK_RES].requestedSetpointC);
  packet.validFlags = 0;
  if (localTanks[TANK_MAIN].valid) packet.validFlags |= REMOTE_VALID_MAIN_TEMP;
  if (localTanks[TANK_RES].valid) packet.validFlags |= REMOTE_VALID_RES_TEMP;
  packet.uiFlags = editMode ? REMOTE_UI_SET_MODE : 0U;
  if (selectedTank == TANK_RES) packet.uiFlags |= REMOTE_UI_SELECTED_TANK_RES;
  packet.buttonFlags = pendingButtonFlags;
  finalizePacket(packet);

  lastOutboundPacket = packet;

  prepareForRadio();
  radioHw.stopListening();
  bool ok = radioHw.write(&packet, sizeof(packet));
  radioHw.startListening();
  lastTxOk = ok;

  if (ok) ++txOkCount;
  else ++txFailCount;

  pendingButtonFlags = 0;
}

void drawMainStatic() {
  prepareForTft();
  tft.fillScreen(COLOR_BG);

  tft.fillRect(HEADER_X, HEADER_Y, HEADER_W, HEADER_H, COLOR_PANEL);
  tft.drawFastHLine(0, HEADER_H, SCREEN_W, COLOR_DIM);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT);
  tft.setCursor(10, 7);
  tft.print(F("REMOTE BOX"));
  tft.setTextSize(1);
  tft.setTextColor(COLOR_DIM);
  tft.setCursor(220, 4);
  tft.print(F("LINK"));
  tft.setCursor(220, 17);
  tft.print(F("MODE"));

  tft.drawRoundRect(MAIN_X, CARD_Y, CARD_W, CARD_H, 8, COLOR_DIM);
  tft.drawRoundRect(RES_X, CARD_Y, CARD_W, CARD_H, 8, COLOR_DIM);
  tft.drawRoundRect(8, FOOT_Y, SCREEN_W - 16, FOOT_H - 8, 6, COLOR_DIM);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_DIM);
  tft.setCursor(16, FOOT_Y + 34);
  tft.print(F("SET short=tank  long=edit  UP+DOWN hold=screen"));

  invalidateAllCaches();
}

void drawTankFrame(uint8_t tank) {
  int x = (tank == TANK_MAIN) ? MAIN_X : RES_X;
  bool selected = (selectedTank == tank);

  uint16_t borderColor = selected ? ST77XX_YELLOW : COLOR_DIM;
  uint16_t titleFill = selected ? (editMode ? COLOR_ACCENT : ST77XX_BLUE) : COLOR_PANEL_ALT;

  prepareForTft();
  tft.fillRoundRect(x, CARD_Y, CARD_W, CARD_H, 8, COLOR_BG);
  tft.drawRoundRect(x, CARD_Y, CARD_W, CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, CARD_Y + 1, CARD_W - 2, 22, 8, titleFill);
  tft.fillRect(x + 2, CARD_Y + 18, CARD_W - 4, CARD_H - 20, COLOR_BG);
  tft.drawFastHLine(x + 8, CARD_Y + 78, CARD_W - 16, COLOR_DIM);

  char title[16];
  title[0] = selected ? (editMode ? '*' : '>') : ' ';
  title[1] = ' ';
  copyFlashString(title + 2, sizeof(title) - 2, tankLabelProgmem(tank));

  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT, titleFill);
  tft.setCursor(x + 8, CARD_Y + 5);
  tft.print(title);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_DIM);
  tft.setCursor(x + 10, CARD_Y + 86);
  tft.print(F("Req"));
  tft.setCursor(x + 10, CARD_Y + 102);
  tft.print(F("Act"));
  tft.setCursor(x + 10, CARD_Y + 118);
  tft.print(F("Out"));
  tft.setCursor(x + 98, CARD_Y + 118);
  tft.print(F("Src"));
}

void ensureMainFramesCurrent() {
  if (mainFrameSelectedTank != static_cast<int8_t>(selectedTank) || mainFrameEditMode != editMode) {
    drawTankFrame(TANK_MAIN);
    drawTankFrame(TANK_RES);
    mainFrameSelectedTank = static_cast<int8_t>(selectedTank);
    mainFrameEditMode = editMode;

    for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
      invalidateField(mainTempCache[tank]);
      invalidateField(mainSourceCache[tank]);
      invalidateField(mainReqCache[tank]);
      invalidateField(mainActCache[tank]);
      invalidateField(mainOutCache[tank]);
    }
  }
}

void buildFaultSummary(char* out, size_t outLen, uint16_t& colorOut) {
  LinkState link = currentLinkState(millis());
  uint16_t faults = haveMainStatus ? latestMainStatus.faultFlags : 0U;

  if (link == LINK_STATE_NO_RADIO) {
    copyFlashString(out, outLen, PSTR("Radio init failed"));
    colorOut = COLOR_FAULT;
  } else if (link == LINK_STATE_TIMEOUT) {
    copyFlashString(out, outLen, PSTR("Comm timeout: retrying main box"));
    colorOut = COLOR_FAULT;
  } else if (link == LINK_STATE_WAITING) {
    copyFlashString(out, outLen, PSTR("Waiting for main-box status"));
    colorOut = COLOR_WARN;
  } else if ((faults & FAULT_LOCAL_BRIDGE) != 0U) {
    copyFlashString(out, outLen, PSTR("Main-box bridge fault"));
    colorOut = COLOR_FAULT;
  } else if ((faults & FAULT_REMOTE_COMM) != 0U) {
    copyFlashString(out, outLen, PSTR("Remote comm fault at main box"));
    colorOut = COLOR_FAULT;
  } else if ((faults & FAULT_MAIN_OVERTEMP) != 0U) {
    copyFlashString(out, outLen, PSTR("MAIN overtemp cutoff"));
    colorOut = COLOR_FAULT;
  } else if ((faults & FAULT_RES_OVERTEMP) != 0U) {
    copyFlashString(out, outLen, PSTR("RES overtemp cutoff"));
    colorOut = COLOR_FAULT;
  } else if ((faults & FAULT_MAIN_SENSOR_INVALID) != 0U) {
    copyFlashString(out, outLen, PSTR("MAIN sensor invalid"));
    colorOut = COLOR_FAULT;
  } else if ((faults & FAULT_RES_SENSOR_INVALID) != 0U) {
    copyFlashString(out, outLen, PSTR("RES sensor invalid"));
    colorOut = COLOR_FAULT;
  } else if ((faults & (FAULT_MAIN_WARN | FAULT_RES_WARN)) != 0U) {
    copyFlashString(out, outLen, PSTR("Temperature warning active"));
    colorOut = COLOR_WARN;
  } else if ((localTanks[TANK_MAIN].valid && localTanks[TANK_MAIN].rawTempC >= HARD_MAX_TEMP_C) ||
             (localTanks[TANK_RES].valid && localTanks[TANK_RES].rawTempC >= HARD_MAX_TEMP_C)) {
    copyFlashString(out, outLen, PSTR("Local sensor over hard max"));
    colorOut = COLOR_FAULT;
  } else if ((localTanks[TANK_MAIN].valid && localTanks[TANK_MAIN].rawTempC >= WARN_TEMP_C) ||
             (localTanks[TANK_RES].valid && localTanks[TANK_RES].rawTempC >= WARN_TEMP_C)) {
    copyFlashString(out, outLen, PSTR("Local sensor warning"));
    colorOut = COLOR_WARN;
  } else if (!localTanks[TANK_MAIN].valid || !localTanks[TANK_RES].valid) {
    copyFlashString(out, outLen, PSTR("Check thermocouple inputs"));
    colorOut = COLOR_WARN;
  } else {
    copyFlashString(out, outLen, PSTR("System OK"));
    colorOut = COLOR_OK;
  }
}

void updateMainScreen() {
  unsigned long nowMs = millis();
  ensureMainFramesCurrent();

  LinkState link = currentLinkState(nowMs);
  char line[32];
  char linkText[10];
  char modeText[5];
  char tankText[5];
  copyFlashString(linkText, sizeof(linkText), linkStateTextProgmem(link));
  copyFlashString(modeText, sizeof(modeText), editMode ? PSTR("EDIT") : PSTR("VIEW"));
  copyFlashString(tankText, sizeof(tankText), tankLabelProgmem(selectedTank));

  updateTextField(mainLinkCache, linkText, linkStateColor(link),
                  220, 4, 94, 10, 1, COLOR_PANEL);

  snprintf_P(line, sizeof(line), PSTR("%s %s"), modeText, tankText);
  updateTextField(mainModeCache, line, editMode ? COLOR_ACCENT : COLOR_TEXT,
                  220, 17, 94, 10, 1, COLOR_PANEL);

  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    int x = (tank == TANK_MAIN) ? MAIN_X : RES_X;
    bool tempValid = false;
    uint16_t tempColor = COLOR_TEXT;
    SourceLabel sourceLabel = SOURCE_LABEL_ERR;
    float tempC = displayTempForTank(tank, nowMs, tempValid, tempColor, sourceLabel);
    char tempText[16];
    char sourceText[6];
    formatTemp(tempText, sizeof(tempText), tempC, tempValid);
    copyFlashString(sourceText, sizeof(sourceText), sourceLabelTextProgmem(sourceLabel));
    updateTextField(mainTempCache[tank], tempText, tempColor,
                    x + 10, CARD_Y + 34, 128, 36, 3, COLOR_BG);

    updateTextField(mainSourceCache[tank], sourceText,
                    sourceLabel == SOURCE_LABEL_AUTH ? COLOR_OK :
                    sourceLabel == SOURCE_LABEL_LAST ? COLOR_WARN :
                    sourceLabel == SOURCE_LABEL_LOCAL ? COLOR_LOCAL : COLOR_FAULT,
                    x + 98, CARD_Y + 118, 40, 10, 1, COLOR_BG);

    formatFixed1(line, sizeof(line), localTanks[tank].requestedSetpointC);
    updateTextField(mainReqCache[tank], line, COLOR_TEXT,
                    x + 36, CARD_Y + 86, 48, 10, 1, COLOR_BG);

    bool actValid = false;
    float actSetpoint = activeSetpointFromMain(tank, actValid);
    if (!haveMainStatus) {
      copyFlashString(line, sizeof(line), PSTR("--.-"));
    } else {
      formatShortTemp(line, sizeof(line), actSetpoint, actValid);
    }
    uint16_t actColor = statusPacketHealthy(nowMs) ? COLOR_TEXT :
                        statusPacketFresh(nowMs) ? COLOR_WARN : COLOR_DIM;
    if (!haveMainStatus) actColor = COLOR_DIM;
    updateTextField(mainActCache[tank], line, actColor,
                    x + 36, CARD_Y + 102, 48, 10, 1, COLOR_BG);

    bool outValid = false;
    float outputPct = outputPctFromMain(tank, outValid);
    if (!outValid) {
      copyFlashString(line, sizeof(line), PSTR("--.-%"));
    } else {
      char pctText[12];
      char onOffText[4];
      formatFixed1(pctText, sizeof(pctText), outputPct);
      copyFlashString(onOffText, sizeof(onOffText), onOffTextProgmem(heaterOnFromMain(tank)));
      snprintf_P(line, sizeof(line), PSTR("%s%% %s"), pctText, onOffText);
    }
    line[sizeof(line) - 1] = '\0';
    updateTextField(mainOutCache[tank], line,
                    heaterOnFromMain(tank) ? COLOR_ACCENT : COLOR_TEXT,
                    x + 36, CARD_Y + 118, 58, 10, 1, COLOR_BG);
  }

  uint16_t faultColor = COLOR_TEXT;
  buildFaultSummary(line, sizeof(line), faultColor);
  updateTextField(mainFaultCache, line, faultColor,
                  16, FOOT_Y + 10, 288, 12, 1, COLOR_BG);

  char timeBuf[16];
  char footerBuf[32];
  unsigned long atSetpointSec = haveMainStatus ? latestMainStatus.atSetpointSeconds : 0UL;
  formatTimeHHMMSS(timeBuf, sizeof(timeBuf), atSetpointSec);
  snprintf_P(footerBuf, sizeof(footerBuf), PSTR("Ack:%u OK:%lu At:%s"),
             lastStatusSequence, txOkCount, timeBuf);
  updateTextField(mainInfoCache, footerBuf, COLOR_DIM,
                  16, FOOT_Y + 22, 288, 12, 1, COLOR_BG);
}

void drawDebugStatic() {
  prepareForTft();
  tft.fillScreen(COLOR_BG);
  tft.fillRect(0, 0, SCREEN_W, 24, COLOR_PANEL);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT, COLOR_PANEL);
  tft.setCursor(10, 5);
  tft.print(F("REMOTE DEBUG"));
  tft.setTextSize(1);
  tft.setTextColor(COLOR_DIM);
  tft.setCursor(12, 224);
  tft.print(F("UP+DOWN hold toggles screen"));

  for (uint8_t index = 0; index < 10; ++index) {
    invalidateField(debugLineCache[index]);
  }
}

void updateDebugLine(uint8_t index, const char* text, uint16_t color) {
  if (index >= 10) return;
  int y = 34 + index * 18;
  updateTextField(debugLineCache[index], text, color, 10, y, 300, 12, 1, COLOR_BG);
}

void updateDebugScreen() {
  unsigned long nowMs = millis();
  char line[48];
  char tempA[12];
  char tempB[12];
  char linkText[10];
  char screenText[6];
  char tankText[5];
  char editText[4];

  copyFlashString(linkText, sizeof(linkText), linkStateTextProgmem(currentLinkState(nowMs)));
  copyFlashString(screenText, sizeof(screenText),
                  screenMode == SCREEN_MODE_MAIN ? PSTR("MAIN") : PSTR("DEBUG"));

  snprintf_P(line, sizeof(line), PSTR("Link:%s  Screen:%s"), linkText, screenText);
  updateDebugLine(0, line, linkStateColor(currentLinkState(nowMs)));

  copyFlashString(tankText, sizeof(tankText), tankLabelProgmem(selectedTank));
  copyFlashString(editText, sizeof(editText), editMode ? PSTR("ON") : PSTR("OFF"));
  snprintf_P(line, sizeof(line), PSTR("Sel:%s  Edit:%s  Btn:0x%02X"),
             tankText, editText, pendingButtonFlags);
  updateDebugLine(1, line, COLOR_TEXT);

  formatShortTemp(tempA, sizeof(tempA), localTanks[TANK_MAIN].rawTempC, localTanks[TANK_MAIN].valid);
  formatShortTemp(tempB, sizeof(tempB), localTanks[TANK_RES].rawTempC, localTanks[TANK_RES].valid);
  snprintf_P(line, sizeof(line), PSTR("Raw M:%s  R:%s"), tempA, tempB);
  updateDebugLine(2, line, (!localTanks[TANK_MAIN].valid || !localTanks[TANK_RES].valid) ? COLOR_WARN : COLOR_TEXT);

  formatFixed1(tempA, sizeof(tempA), localTanks[TANK_MAIN].requestedSetpointC);
  formatFixed1(tempB, sizeof(tempB), localTanks[TANK_RES].requestedSetpointC);
  snprintf_P(line, sizeof(line), PSTR("Req M:%s  R:%s"), tempA, tempB);
  updateDebugLine(3, line, COLOR_TEXT);

  if (haveMainStatus) {
    formatShortTemp(tempA, sizeof(tempA), decodeTempCx100(latestMainStatus.mainSetpointCx100), true);
    formatShortTemp(tempB, sizeof(tempB), decodeTempCx100(latestMainStatus.resSetpointCx100), true);
    snprintf_P(line, sizeof(line), PSTR("Act M:%s  R:%s"), tempA, tempB);
  } else {
    copyFlashString(line, sizeof(line), PSTR("Act M:--.-  R:--.-"));
  }
  updateDebugLine(4, line, statusPacketFresh(nowMs) ? COLOR_TEXT : COLOR_WARN);

  snprintf_P(line, sizeof(line), PSTR("TX seq:%u  RX seq:%u"),
             lastOutboundPacket.sequence, haveMainStatus ? latestMainStatus.statusSequence : 0U);
  updateDebugLine(5, line, COLOR_TEXT);

  snprintf_P(line, sizeof(line), PSTR("RX age:%lu ms  Ctrl age:%u"),
             haveMainStatus ? (nowMs - lastStatusRxMs) : 65535UL,
             haveMainStatus ? latestMainStatus.controllerAgeMs : 65535U);
  updateDebugLine(6, line, statusPacketFresh(nowMs) ? COLOR_TEXT : COLOR_WARN);

  if (haveMainStatus) {
    formatFixed1(tempA, sizeof(tempA), decodeOutputPermille(latestMainStatus.mainOutputPermille));
    formatFixed1(tempB, sizeof(tempB), decodeOutputPermille(latestMainStatus.resOutputPermille));
    snprintf_P(line, sizeof(line), PSTR("Out M:%s  R:%s  Heat:%02X"),
               tempA, tempB, latestMainStatus.heaterFlags);
  } else {
    copyFlashString(line, sizeof(line), PSTR("Out M:--.-  R:--.-  Heat:00"));
  }
  updateDebugLine(7, line, COLOR_TEXT);

  snprintf_P(line, sizeof(line), PSTR("Fault:0x%04X  Link:0x%02X"),
             haveMainStatus ? latestMainStatus.faultFlags : 0U,
             haveMainStatus ? latestMainStatus.linkFlags : 0U);
  updateDebugLine(8, line, (haveMainStatus && latestMainStatus.faultFlags != 0U) ? COLOR_WARN : COLOR_TEXT);

  snprintf_P(line, sizeof(line), PSTR("TX ok:%lu fail:%lu  RX ok:%lu bad:%lu"),
             txOkCount, txFailCount, rxOkCount, rxBadCount);
  updateDebugLine(9, line, COLOR_DIM);
}

void updateDisplay() {
  unsigned long nowMs = millis();
  if ((nowMs - lastDisplayMs) < DISPLAY_INTERVAL_MS) return;
  lastDisplayMs = nowMs;

  if (displayNeedsFullRedraw) {
    if (screenMode == SCREEN_MODE_MAIN) drawMainStatic();
    else drawDebugStatic();
    displayNeedsFullRedraw = false;
  }

  if (screenMode == SCREEN_MODE_MAIN) updateMainScreen();
  else updateDebugScreen();
}

void logSummary() {
#if REMOTE_BOX_ENABLE_SERIAL_LOG
  unsigned long nowMs = millis();
  char linkText[10];
  char tankText[5];
  char editText[4];
  char screenText[6];
  if ((nowMs - lastLogMs) < LOG_INTERVAL_MS) return;
  lastLogMs = nowMs;

  copyFlashString(linkText, sizeof(linkText), linkStateTextProgmem(currentLinkState(nowMs)));
  copyFlashString(tankText, sizeof(tankText), tankLabelProgmem(selectedTank));
  copyFlashString(editText, sizeof(editText), editMode ? PSTR("ON") : PSTR("OFF"));
  copyFlashString(screenText, sizeof(screenText),
                  screenMode == SCREEN_MODE_MAIN ? PSTR("MAIN") : PSTR("DEBUG"));

  Serial.print(F("RAW:"));
  Serial.print(localTanks[TANK_MAIN].rawTempC, 2);
  Serial.print('/');
  Serial.print(localTanks[TANK_RES].rawTempC, 2);
  Serial.print(F(",REQ:"));
  Serial.print(localTanks[TANK_MAIN].requestedSetpointC, 1);
  Serial.print('/');
  Serial.print(localTanks[TANK_RES].requestedSetpointC, 1);
  Serial.print(F(",TX:"));
  Serial.print(lastOutboundPacket.sequence);
  Serial.print(',');
  Serial.print(lastTxOk ? F("OK") : F("FAIL"));
  Serial.print(F(",RX:"));
  Serial.print(haveMainStatus ? latestMainStatus.statusSequence : 0U);
  Serial.print(F(",AGE:"));
  Serial.print(haveMainStatus ? (nowMs - lastStatusRxMs) : 65535UL);
  Serial.print(F(",LINK:"));
  Serial.print(linkText);
  Serial.print(F(",SEL:"));
  Serial.print(tankText);
  Serial.print(F(",EDIT:"));
  Serial.print(editText);
  Serial.print(F(",SCR:"));
  Serial.print(screenText);
  Serial.print(F(",FLT:0x"));
  Serial.print(haveMainStatus ? latestMainStatus.faultFlags : 0U, HEX);
  Serial.print(F(",OUT:"));
  Serial.print(haveMainStatus ? decodeOutputPermille(latestMainStatus.mainOutputPermille) : 0.0f, 1);
  Serial.print('/');
  Serial.print(haveMainStatus ? decodeOutputPermille(latestMainStatus.resOutputPermille) : 0.0f, 1);
  Serial.println();
#endif
}

}  // namespace

#line 1099 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\firmware\\remote_box\\remote_box.ino"
void setup();
#line 1126 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\firmware\\remote_box\\remote_box.ino"
void loop();
#line 1099 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\firmware\\remote_box\\remote_box.ino"
void setup() {
#if REMOTE_BOX_ENABLE_SERIAL_LOG
  Serial.begin(115200);
#endif

  localTanks[TANK_MAIN].requestedSetpointC = DEFAULT_SETPOINT_C;
  localTanks[TANK_RES].requestedSetpointC = DEFAULT_SETPOINT_C;
  localTanks[TANK_MAIN].rawTempC = NAN;
  localTanks[TANK_RES].rawTempC = NAN;
  localTanks[TANK_MAIN].valid = false;
  localTanks[TANK_RES].valid = false;

  initSpiChipSelects();
  SPI.begin();
  initButtons();
  initDisplay();
  initRadio();

  bootMs = millis();
  sensorsReadyAtMs = bootMs + MAX6675_STARTUP_MS;
#if REMOTE_BOX_ENABLE_SERIAL_LOG
  lastLoggedLinkState = currentLinkState(bootMs);
#endif

  logEvent(PSTR("Remote box ready"));
}

void loop() {
  handleButtons();
  readSensors();
  pollRadioDownlink();
  sendRadioUplink();
  updateDisplay();
  logLinkTransitions();
  logSummary();
}

