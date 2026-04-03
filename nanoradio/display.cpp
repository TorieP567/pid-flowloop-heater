#include "display.h"

#include <avr/pgmspace.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#include "config.h"
#include "radio.h"
#include "sensors.h"

namespace {

template <size_t N>
struct TextFieldCache {
  char text[N];
  uint16_t color;
  bool valid;
};

enum DisplayPhase : uint8_t {
  PHASE_IDLE = 0,
  PHASE_HEADER,
  PHASE_TANK0_HEADER,
  PHASE_TANK0_VALUES,
  PHASE_TANK1_HEADER,
  PHASE_TANK1_VALUES,
  PHASE_FOOTER
};

struct TankCache {
  float temp;
  int8_t statusCode;
  float setpoint;
  float error;
  float rawTemp;
  float stddev;
  int8_t trendCode;
  bool isSelected;
  bool inEditMode;
};

struct CachedFields {
  int8_t radioStatus;
  unsigned long runtimeSec;
  TankCache tanks[config::TANK_COUNT];
  const char* buttonEvent;
  uint8_t selectedTank;
  bool editMode;
  unsigned long atSpSec;
  bool atSetpoint;
  float footerMainTemp;
  float footerResTemp;
};

Adafruit_ST7789 tft(
    config::pins::TFT_CS_PIN,
    config::pins::TFT_DC_PIN,
    config::pins::TFT_RST_PIN);

DisplayPhase phase = PHASE_IDLE;
CachedFields cache = {};
TextFieldCache<40> debugLineCache[10] = {};

bool floatChanged(float a, float b, float tolerance = 0.05f) {
  if (isnan(a) != isnan(b)) return true;
  return fabsf(a - b) > tolerance;
}

void clearField(int x, int y, int w, int h) {
  config::prepareForTft();
  tft.fillRect(x, y, w, h, config::color::COLOR_BG);
}

uint16_t deltaColor(float errorC) {
  const float magnitude = fabsf(errorC);
  if (magnitude <= 0.5f) return config::color::COLOR_OK;
  if (magnitude <= 2.0f) return config::color::COLOR_WARN;
  return 0xFD20;
}

template <size_t N>
void invalidateField(TextFieldCache<N>& cacheField) {
  cacheField.text[0] = '\0';
  cacheField.color = 0;
  cacheField.valid = false;
}

void invalidateMainCache() {
  cache.radioStatus = 127;
  cache.runtimeSec = 0xFFFFFFFFUL;
  for (uint8_t index = 0; index < config::TANK_COUNT; ++index) {
    TankCache& tank = cache.tanks[index];
    tank.temp = NAN;
    tank.statusCode = 127;
    tank.setpoint = NAN;
    tank.error = NAN;
    tank.rawTemp = NAN;
    tank.stddev = NAN;
    tank.trendCode = 127;
    tank.isSelected = (index != 0);
    tank.inEditMode = true;
  }
  cache.buttonEvent = nullptr;
  cache.selectedTank = 255;
  cache.editMode = true;
  cache.atSpSec = 0xFFFFFFFFUL;
  cache.atSetpoint = true;
  cache.footerMainTemp = NAN;
  cache.footerResTemp = NAN;
}

void invalidateDebugCache() {
  for (uint8_t index = 0; index < 10; ++index) {
    invalidateField(debugLineCache[index]);
  }
}

template <size_t N>
void updateTextField(TextFieldCache<N>& cacheField, const char* text, uint16_t color,
                     int16_t x, int16_t y, int16_t w, int16_t h,
                     uint8_t textSize, uint16_t bgColor = config::color::COLOR_BG) {
  if (w <= 0 || h <= 0) return;

  if (!cacheField.valid || cacheField.color != color ||
      strncmp(cacheField.text, text, sizeof(cacheField.text)) != 0) {
    clearField(x, y, w, h);
    config::prepareForTft();
    tft.setTextSize(textSize);
    tft.setTextColor(color, bgColor);
    tft.setCursor(x, y);
    tft.print(text);

    strncpy(cacheField.text, text, sizeof(cacheField.text) - 1);
    cacheField.text[sizeof(cacheField.text) - 1] = '\0';
    cacheField.color = color;
    cacheField.valid = true;
  }
}

void formatTimeHHMMSS(unsigned long totalSec, char* out, size_t outLen) {
  if (totalSec > 359999UL) totalSec = 359999UL;
  const uint8_t h = totalSec / 3600UL;
  const uint8_t m = (totalSec % 3600UL) / 60UL;
  const uint8_t s = totalSec % 60UL;
  snprintf(out, outLen, "%02u:%02u:%02u", h, m, s);
}

void formatFixed1(char* out, size_t outLen, float value) {
  char temp[16];
  dtostrf(value, 0, 1, temp);

  char* start = temp;
  while (*start == ' ') ++start;

  strncpy(out, start, outLen - 1);
  out[outLen - 1] = '\0';
}

void formatShortTemp(char* out, size_t outLen, float tempC, bool valid) {
  if (!valid || isnan(tempC)) {
    strncpy(out, "--.-", outLen - 1);
    out[outLen - 1] = '\0';
    return;
  }
  formatFixed1(out, outLen, tempC);
}

void drawTankFrame(const DashboardState& state, uint8_t tankIndex, int x, int y) {
  const bool isSelected = (state.selectedTank == tankIndex);
  const uint16_t borderColor = isSelected ? ST77XX_YELLOW : ST77XX_WHITE;
  const uint16_t titleFill = isSelected ? (state.editMode ? ST77XX_CYAN : ST77XX_MAGENTA) : ST77XX_BLUE;

  config::prepareForTft();
  tft.drawRoundRect(x, y, config::layout::CARD_W, config::layout::CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, y + 1, config::layout::CARD_W - 2, 20, 8, titleFill);
  tft.fillRect(x + 2, y + 16, config::layout::CARD_W - 4, config::layout::CARD_H - 18, config::color::COLOR_BG);

  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(x + 8, y + 4);
  if (isSelected) tft.print(state.editMode ? "* " : "> ");
  else tft.print("  ");
  tft.print(tankIndex == config::TANK_MAIN ? "MAIN" : "RES");

  tft.setTextSize(1);
  tft.setTextColor(0xB596);

  tft.setCursor(x + 8,  y + 86);  tft.print("SP");
  tft.setCursor(x + 8,  y + 98);  tft.print("dT");
  tft.setCursor(x + 8,  y + 110); tft.print("RAW");
  tft.setCursor(x + 8,  y + 122); tft.print("SD");

  tft.setCursor(x + 86, y + 86);  tft.print("TRD");
  tft.setCursor(x + 86, y + 98);  tft.print("ST");

  tft.drawFastHLine(x + 6, y + 78, config::layout::CARD_W - 12, 0x4208);
}

void drawStaticUI(const DashboardState& state) {
  config::prepareForTft();
  tft.fillScreen(config::color::COLOR_BG);
  tft.setTextWrap(false);

  tft.fillRoundRect(config::layout::HEADER_X, config::layout::HEADER_Y,
                    config::layout::HEADER_W, config::layout::HEADER_H, 6, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(12, 9);
  tft.print("REMOTE DASH");

  tft.setTextSize(1);
  tft.setTextColor(0xB596);
  tft.setCursor(195, 8);  tft.print("RADIO");
  tft.setCursor(255, 8);  tft.print("RUN");

  drawTankFrame(state, config::TANK_MAIN, config::layout::MAIN_X, config::layout::CARD_Y);
  drawTankFrame(state, config::TANK_RES, config::layout::RES_X, config::layout::CARD_Y);

  tft.drawRoundRect(config::layout::FOOT_X, config::layout::FOOT_Y,
                    config::layout::FOOT_W, config::layout::FOOT_H, 6, 0x4208);
  tft.setTextSize(1);
  tft.setTextColor(0xB596);

  tft.setCursor(10, 194);   tft.print("BTN");
  tft.setCursor(112, 194);  tft.print("SEL");
  tft.setCursor(190, 194);  tft.print("MODE");
  tft.setCursor(255, 194);  tft.print("ATSP");

  tft.setCursor(10, 216);   tft.print("MAIN");
  tft.setCursor(112, 216);  tft.print("RES");
  tft.setCursor(190, 216);  tft.print("BAND");
}

void drawHeaderRadioField(const DashboardState& state) {
  clearField(194, 16, 55, 10);
  config::prepareForTft();
  tft.setTextSize(1);
  tft.setCursor(194, 16);

  if (!state.radioInitOk) {
    tft.setTextColor(config::color::COLOR_FAULT);
    tft.print("NO INIT");
  } else if (state.lastTxOk) {
    tft.setTextColor(config::color::COLOR_OK);
    tft.print("TX OK");
  } else {
    tft.setTextColor(config::color::COLOR_FAULT);
    tft.print("TX FAIL");
  }
}

void drawHeaderRunField(const DashboardState& state) {
  char buf[9];
  formatTimeHHMMSS((millis() - state.bootMs) / 1000UL, buf, sizeof(buf));

  clearField(254, 16, 58, 10);
  config::prepareForTft();
  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(254, 16);
  tft.print(buf);
}

void drawHeaderDirty(const DashboardState& state) {
  int8_t currentRadioStatus;
  if (!state.radioInitOk) currentRadioStatus = -1;
  else if (state.lastTxOk) currentRadioStatus = 1;
  else currentRadioStatus = 0;

  if (currentRadioStatus != cache.radioStatus) {
    drawHeaderRadioField(state);
    cache.radioStatus = currentRadioStatus;
  }

  const unsigned long currentRuntime = (millis() - state.bootMs) / 1000UL;
  if (currentRuntime != cache.runtimeSec) {
    drawHeaderRunField(state);
    cache.runtimeSec = currentRuntime;
  }
}

void drawTankHeaderDirty(const DashboardState& state, uint8_t tankIndex, int x, int y) {
  TankCache& tankCache = cache.tanks[tankIndex];
  const bool isSelected = (state.selectedTank == tankIndex);
  const bool inEditMode = state.editMode;

  if (isSelected != tankCache.isSelected || inEditMode != tankCache.inEditMode) {
    drawTankFrame(state, tankIndex, x, y);
    tankCache.isSelected = isSelected;
    tankCache.inEditMode = inEditMode;
  }
}

void drawTankValuesDirty(int x, int y, const TankLocalState& tank, TankCache& tankCache) {
  const float error = tank.requestedSetpointC - tank.filteredTempC;
  const float stddev = sensors::computeStdDev(tank);
  const int8_t currentStatus = sensors::getStatusCode(tank);
  const int8_t currentTrend = sensors::getTrendCode(tank);
  const bool statusChanged = (currentStatus != tankCache.statusCode);

  if (floatChanged(tank.filteredTempC, tankCache.temp) || statusChanged) {
    clearField(x + 8, y + 28, 134, 40);
    config::prepareForTft();
    if (tank.valid) {
      tft.setTextSize(4);
      tft.setTextColor(sensors::getStatusColor(tank));
      tft.setCursor(x + 10, y + 30);
      tft.print(tank.filteredTempC, 1);
    } else {
      tft.setTextSize(3);
      tft.setTextColor(config::color::COLOR_FAULT);
      tft.setCursor(x + 10, y + 34);
      tft.print("ERR");
    }
    tankCache.temp = tank.filteredTempC;
  }

  tft.setTextSize(1);

  if (floatChanged(tank.requestedSetpointC, tankCache.setpoint)) {
    clearField(x + 26, y + 86, 50, 9);
    config::prepareForTft();
    tft.setTextColor(config::color::COLOR_TEXT);
    tft.setCursor(x + 26, y + 86);
    tft.print(tank.requestedSetpointC, 1);
    tankCache.setpoint = tank.requestedSetpointC;
  }

  if (floatChanged(error, tankCache.error)) {
    clearField(x + 26, y + 98, 50, 9);
    config::prepareForTft();
    tft.setCursor(x + 26, y + 98);
    tft.setTextColor(deltaColor(error));
    if (error >= 0.0f) tft.print('+');
    tft.print(error, 1);
    tankCache.error = error;
  }

  if (floatChanged(tank.rawTempC, tankCache.rawTemp)) {
    clearField(x + 32, y + 110, 44, 9);
    config::prepareForTft();
    tft.setCursor(x + 32, y + 110);
    if (tank.valid) {
      tft.setTextColor(config::color::COLOR_TEXT);
      tft.print(tank.rawTempC, 1);
    } else {
      tft.setTextColor(config::color::COLOR_FAULT);
      tft.print("ERR");
    }
    tankCache.rawTemp = tank.rawTempC;
  }

  if (floatChanged(stddev, tankCache.stddev)) {
    clearField(x + 26, y + 122, 52, 9);
    config::prepareForTft();
    tft.setCursor(x + 26, y + 122);
    tft.setTextColor(config::color::COLOR_TEXT);
    if (isnan(stddev)) tft.print("--");
    else tft.print(stddev, 2);
    tankCache.stddev = stddev;
  }

  if (currentTrend != tankCache.trendCode) {
    clearField(x + 110, y + 86, 34, 9);
    config::prepareForTft();
    tft.setCursor(x + 110, y + 86);
    tft.setTextColor(sensors::getTrendColor(tank));
    tft.print(sensors::getTrendText(tank));
    tankCache.trendCode = currentTrend;
  }

  if (statusChanged) {
    clearField(x + 110, y + 98, 34, 9);
    config::prepareForTft();
    tft.setCursor(x + 110, y + 98);
    tft.setTextColor(sensors::getStatusColor(tank));
    tft.print(sensors::getStatusText(tank));
    tankCache.statusCode = currentStatus;
  }
}

void drawFooterDirty(const DashboardState& state) {
  config::prepareForTft();
  tft.setTextSize(1);

  if (state.lastButtonEvent != cache.buttonEvent) {
    clearField(10, 204, 88, 10);
    config::prepareForTft();
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 204);
    tft.print(state.lastButtonEvent);
    cache.buttonEvent = state.lastButtonEvent;
  }

  if (state.selectedTank != cache.selectedTank) {
    clearField(112, 204, 60, 10);
    config::prepareForTft();
    tft.setTextColor(config::color::COLOR_TEXT);
    tft.setCursor(112, 204);
    tft.print(state.selectedTank == config::TANK_MAIN ? "MAIN" : "RES");
    cache.selectedTank = state.selectedTank;
  }

  if (state.editMode != cache.editMode) {
    clearField(190, 204, 48, 10);
    config::prepareForTft();
    tft.setCursor(190, 204);
    tft.setTextColor(state.editMode ? config::color::COLOR_ACCENT : config::color::COLOR_OK);
    tft.print(state.editMode ? "SET" : "VIEW");
    cache.editMode = state.editMode;
  }

  if (state.timeAtSetpointSec != cache.atSpSec) {
    char atspBuf[9];
    formatTimeHHMMSS(state.timeAtSetpointSec, atspBuf, sizeof(atspBuf));
    clearField(255, 204, 55, 10);
    config::prepareForTft();
    tft.setTextColor(config::color::COLOR_TEXT);
    tft.setCursor(255, 204);
    tft.print(atspBuf);
    cache.atSpSec = state.timeAtSetpointSec;
  }

  if (floatChanged(state.localTanks[config::TANK_MAIN].filteredTempC, cache.footerMainTemp)) {
    clearField(10, 226, 80, 10);
    config::prepareForTft();
    tft.setTextColor(config::color::COLOR_TEXT);
    tft.setCursor(10, 226);
    if (state.localTanks[config::TANK_MAIN].valid) tft.print(state.localTanks[config::TANK_MAIN].filteredTempC, 1);
    else tft.print("ERR");
    cache.footerMainTemp = state.localTanks[config::TANK_MAIN].filteredTempC;
  }

  if (floatChanged(state.localTanks[config::TANK_RES].filteredTempC, cache.footerResTemp)) {
    clearField(112, 226, 70, 10);
    config::prepareForTft();
    tft.setCursor(112, 226);
    tft.setTextColor(config::color::COLOR_TEXT);
    if (state.localTanks[config::TANK_RES].valid) tft.print(state.localTanks[config::TANK_RES].filteredTempC, 1);
    else tft.print("ERR");
    cache.footerResTemp = state.localTanks[config::TANK_RES].filteredTempC;
  }

  if (state.atSetpoint != cache.atSetpoint) {
    clearField(190, 226, 48, 10);
    config::prepareForTft();
    tft.setCursor(190, 226);
    tft.setTextColor(state.atSetpoint ? config::color::COLOR_OK : config::color::COLOR_FAULT);
    tft.print(state.atSetpoint ? "YES" : "NO");
    cache.atSetpoint = state.atSetpoint;
  }
}

void drawDebugStatic() {
  config::prepareForTft();
  tft.fillScreen(config::color::COLOR_BG);
  tft.fillRect(0, 0, config::layout::SCREEN_W, 24, config::color::COLOR_PANEL);
  tft.setTextSize(2);
  tft.setTextColor(config::color::COLOR_TEXT, config::color::COLOR_PANEL);
  tft.setCursor(10, 5);
  tft.print(F("REMOTE DEBUG"));
  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_DIM);
  tft.setCursor(12, 224);
  tft.print(F("UP+DOWN hold toggles screen"));
  invalidateDebugCache();
}

void updateDebugLine(uint8_t index, const char* text, uint16_t color) {
  if (index >= 10) return;
  const int y = 34 + index * 18;
  updateTextField(debugLineCache[index], text, color, 10, y, 300, 12, 1, config::color::COLOR_BG);
}

void updateDebugScreen(const DashboardState& state) {
  const unsigned long nowMs = millis();
  char line[40];
  char tempA[12];
  char tempB[12];

  snprintf(line, sizeof(line), "Link:%s Scr:%s",
           radio::linkStateText(radio::currentLinkState(state, nowMs)),
           state.screenMode == SCREEN_MODE_MAIN ? "MAIN" : "DEBUG");
  updateDebugLine(0, line, radio::linkStateColor(radio::currentLinkState(state, nowMs)));

  snprintf(line, sizeof(line), "Sel:%s Edit:%s Btn:%02X",
           state.selectedTank == config::TANK_MAIN ? "M" : "R",
           state.editMode ? "ON" : "OFF",
           state.pendingButtonFlags);
  updateDebugLine(1, line, config::color::COLOR_TEXT);

  formatShortTemp(tempA, sizeof(tempA),
                  state.localTanks[config::TANK_MAIN].rawTempC,
                  state.localTanks[config::TANK_MAIN].valid);
  formatShortTemp(tempB, sizeof(tempB),
                  state.localTanks[config::TANK_RES].rawTempC,
                  state.localTanks[config::TANK_RES].valid);
  snprintf(line, sizeof(line), "Raw M:%s R:%s", tempA, tempB);
  updateDebugLine(2, line,
                  (!state.localTanks[config::TANK_MAIN].valid || !state.localTanks[config::TANK_RES].valid)
                      ? config::color::COLOR_WARN
                      : config::color::COLOR_TEXT);

  formatFixed1(tempA, sizeof(tempA), state.localTanks[config::TANK_MAIN].requestedSetpointC);
  formatFixed1(tempB, sizeof(tempB), state.localTanks[config::TANK_RES].requestedSetpointC);
  snprintf(line, sizeof(line), "Req M:%s R:%s", tempA, tempB);
  updateDebugLine(3, line, config::color::COLOR_TEXT);

  if (state.haveMainStatus) {
    formatShortTemp(tempA, sizeof(tempA), decodeTempCx100(state.latestMainStatus.mainSetpointCx100), true);
    formatShortTemp(tempB, sizeof(tempB), decodeTempCx100(state.latestMainStatus.resSetpointCx100), true);
    snprintf(line, sizeof(line), "Act M:%s R:%s", tempA, tempB);
  } else {
    snprintf(line, sizeof(line), "Act M:--.- R:--.-");
  }
  updateDebugLine(4, line,
                  radio::statusPacketFresh(state, nowMs) ? config::color::COLOR_TEXT : config::color::COLOR_WARN);

  snprintf(line, sizeof(line), "Tx:%u Rx:%u",
           state.lastOutboundPacket.sequence,
           state.haveMainStatus ? state.latestMainStatus.statusSequence : 0U);
  updateDebugLine(5, line, config::color::COLOR_TEXT);

  snprintf(line, sizeof(line), "Age:%lu Ctrl:%u",
           state.haveMainStatus ? (nowMs - state.lastStatusRxMs) : 65535UL,
           state.haveMainStatus ? state.latestMainStatus.controllerAgeMs : 65535U);
  updateDebugLine(6, line,
                  radio::statusPacketFresh(state, nowMs) ? config::color::COLOR_TEXT : config::color::COLOR_WARN);

  if (state.haveMainStatus) {
    formatFixed1(tempA, sizeof(tempA), decodeOutputPermille(state.latestMainStatus.mainOutputPermille));
    formatFixed1(tempB, sizeof(tempB), decodeOutputPermille(state.latestMainStatus.resOutputPermille));
    snprintf(line, sizeof(line), "Out M:%s R:%s", tempA, tempB);
  } else {
    snprintf(line, sizeof(line), "Out M:--.- R:--.-");
  }
  updateDebugLine(7, line, config::color::COLOR_TEXT);

  snprintf(line, sizeof(line), "Heat:%02X Flt:%04X",
           state.haveMainStatus ? state.latestMainStatus.heaterFlags : 0U,
           state.haveMainStatus ? state.latestMainStatus.faultFlags : 0U);
  updateDebugLine(8, line,
                  (state.haveMainStatus && state.latestMainStatus.faultFlags != 0U)
                      ? config::color::COLOR_WARN
                      : config::color::COLOR_TEXT);

  snprintf(line, sizeof(line), "Ok:%lu/%lu Rx:%lu/%lu",
           state.txOkCount, state.txFailCount, state.rxOkCount, state.rxBadCount);
  updateDebugLine(9, line, config::color::COLOR_DIM);
}

}  // namespace

namespace display {

void init(DashboardState& state) {
  config::prepareForTft();
  tft.init(240, 320);
  tft.setRotation(1);
  tft.setTextWrap(false);
  drawStaticUI(state);
  invalidateMainCache();
  invalidateDebugCache();
  phase = PHASE_HEADER;
  state.displayNeedsFullRedraw = false;
}

void update(DashboardState& state) {
  const unsigned long nowMs = millis();
  if ((nowMs - state.lastDisplayMs) < config::timing::DISPLAY_INTERVAL_MS) return;
  state.lastDisplayMs = nowMs;

  if (state.screenMode == SCREEN_MODE_DEBUG) {
    if (state.displayNeedsFullRedraw) {
      drawDebugStatic();
      state.displayNeedsFullRedraw = false;
    }
    updateDebugScreen(state);
    phase = PHASE_IDLE;
    return;
  }

  if (state.displayNeedsFullRedraw) {
    drawStaticUI(state);
    invalidateMainCache();
    state.displayNeedsFullRedraw = false;
    phase = PHASE_HEADER;
  }

  switch (phase) {
    case PHASE_IDLE:
      phase = PHASE_HEADER;
      return;
    case PHASE_HEADER:
      drawHeaderDirty(state);
      phase = PHASE_TANK0_HEADER;
      return;
    case PHASE_TANK0_HEADER:
      drawTankHeaderDirty(state, config::TANK_MAIN, config::layout::MAIN_X, config::layout::CARD_Y);
      phase = PHASE_TANK0_VALUES;
      return;
    case PHASE_TANK0_VALUES:
      drawTankValuesDirty(config::layout::MAIN_X, config::layout::CARD_Y,
                          state.localTanks[config::TANK_MAIN], cache.tanks[config::TANK_MAIN]);
      phase = PHASE_TANK1_HEADER;
      return;
    case PHASE_TANK1_HEADER:
      drawTankHeaderDirty(state, config::TANK_RES, config::layout::RES_X, config::layout::CARD_Y);
      phase = PHASE_TANK1_VALUES;
      return;
    case PHASE_TANK1_VALUES:
      drawTankValuesDirty(config::layout::RES_X, config::layout::CARD_Y,
                          state.localTanks[config::TANK_RES], cache.tanks[config::TANK_RES]);
      phase = PHASE_FOOTER;
      return;
    case PHASE_FOOTER:
      drawFooterDirty(state);
      phase = PHASE_IDLE;
      return;
  }
}

}  // namespace display
