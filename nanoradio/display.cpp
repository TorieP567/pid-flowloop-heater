#include "display.h"

#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#include "config.h"
#include "sensors.h"

namespace {

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

bool floatChanged(float a, float b, float tolerance = 0.05f) {
  if (isnan(a) != isnan(b)) return true;
  return fabsf(a - b) > tolerance;
}

void drawBootDisplayTest() {
#if NANORADIO_ENABLE_BOOT_DISPLAY_TEST
  config::prepareForTft();
  tft.fillScreen(ST77XX_RED);
  delay(120);

  config::prepareForTft();
  tft.fillScreen(ST77XX_GREEN);
  delay(120);

  config::prepareForTft();
  tft.fillScreen(ST77XX_BLUE);
  delay(120);
#endif
}

void clearField(int x, int y, int w, int h) {
  config::prepareForTft();
  tft.fillRect(x, y, w, h, config::color::COLOR_BG);
}

void formatTimeHHMMSS(unsigned long totalSec, char* out) {
  if (totalSec > 359999UL) totalSec = 359999UL;

  const uint8_t h = totalSec / 3600UL;
  const uint8_t m = (totalSec % 3600UL) / 60UL;
  const uint8_t s = totalSec % 60UL;

  out[0] = '0' + (h / 10);
  out[1] = '0' + (h % 10);
  out[2] = ':';
  out[3] = '0' + (m / 10);
  out[4] = '0' + (m % 10);
  out[5] = ':';
  out[6] = '0' + (s / 10);
  out[7] = '0' + (s % 10);
  out[8] = '\0';
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
  if (isSelected) tft.print(state.editMode ? F("* ") : F("> "));
  else tft.print(F("  "));
  tft.print(tankIndex == config::TANK_MAIN ? F("MAIN") : F("RES"));

  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(x + 8,  y + 86);  tft.print(F("SP:"));
  tft.setCursor(x + 8,  y + 98);  tft.print(F("dT:"));
  tft.setCursor(x + 8,  y + 110); tft.print(F("RAW:"));
  tft.setCursor(x + 8,  y + 122); tft.print(F("SD:"));
  tft.setCursor(x + 86, y + 86);  tft.print(F("TRD:"));
  tft.setCursor(x + 86, y + 98);  tft.print(F("ST:"));
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
  tft.print(F("REMOTE DASH"));

  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(195, 8);  tft.print(F("RADIO"));
  tft.setCursor(255, 8);  tft.print(F("RUN"));

  drawTankFrame(state, config::TANK_MAIN, config::layout::MAIN_X, config::layout::CARD_Y);
  drawTankFrame(state, config::TANK_RES, config::layout::RES_X, config::layout::CARD_Y);

  tft.drawRoundRect(config::layout::FOOT_X, config::layout::FOOT_Y,
                    config::layout::FOOT_W, config::layout::FOOT_H, 6, ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(10, 194);   tft.print(F("BTN"));
  tft.setCursor(112, 194);  tft.print(F("SEL"));
  tft.setCursor(190, 194);  tft.print(F("MODE"));
  tft.setCursor(255, 194);  tft.print(F("ATSP"));
  tft.setCursor(10, 216);   tft.print(F("MAIN"));
  tft.setCursor(112, 216);  tft.print(F("RES"));
  tft.setCursor(190, 216);  tft.print(F("BAND"));
}

void drawHeaderRadioField(const DashboardState& state) {
  clearField(194, 16, 55, 10);
  config::prepareForTft();
  tft.setTextSize(1);
  tft.setCursor(194, 16);

  if (!state.radioInitOk) {
    tft.setTextColor(config::color::COLOR_FAULT);
    tft.print(F("NO INIT"));
  } else if (state.lastTxOk) {
    tft.setTextColor(config::color::COLOR_OK);
    tft.print(F("TX OK"));
  } else {
    tft.setTextColor(config::color::COLOR_FAULT);
    tft.print(F("TX FAIL"));
  }
}

void drawHeaderRunField(const DashboardState& state) {
  char buf[9];
  formatTimeHHMMSS((millis() - state.bootMs) / 1000UL, buf);

  clearField(254, 16, 58, 10);
  config::prepareForTft();
  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(254, 16);
  tft.print(buf);
}

void drawTankHeaderDynamic(const DashboardState& state, uint8_t tankIndex, int x, int y) {
  const bool isSelected = (state.selectedTank == tankIndex);
  const uint16_t borderColor = isSelected ? ST77XX_YELLOW : ST77XX_WHITE;
  const uint16_t titleFill = isSelected ? (state.editMode ? ST77XX_CYAN : ST77XX_MAGENTA) : ST77XX_BLUE;

  config::prepareForTft();
  tft.drawRoundRect(x, y, config::layout::CARD_W, config::layout::CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, y + 1, config::layout::CARD_W - 2, 20, 8, titleFill);

  tft.setTextSize(2);
  tft.setTextColor(config::color::COLOR_TEXT, titleFill);
  tft.setCursor(x + 8, y + 4);
  if (isSelected) tft.print(state.editMode ? F("* ") : F("> "));
  else tft.print(F("  "));
  tft.print(tankIndex == config::TANK_MAIN ? F("MAIN") : F("RES"));
}

void drawHeaderDirty(const DashboardState& state) {
  int8_t currentRadio;
  if (!state.radioInitOk) currentRadio = -1;
  else if (state.lastTxOk) currentRadio = 1;
  else currentRadio = 0;

  if (currentRadio != cache.radioStatus) {
    drawHeaderRadioField(state);
    cache.radioStatus = currentRadio;
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

  if (isSelected != tankCache.isSelected || state.editMode != tankCache.inEditMode) {
    drawTankHeaderDynamic(state, tankIndex, x, y);
    tankCache.isSelected = isSelected;
    tankCache.inEditMode = state.editMode;
  }
}

void drawTankValuesDirty(int x, int y, const TankLocalState& tank, TankCache& tankCache) {
  const float error = tank.requestedSetpointC - tank.filteredTempC;
  const float stddev = sensors::computeStdDev(tank);
  const int8_t currentStatus = sensors::getStatusCode(tank);
  const int8_t currentTrend = sensors::getTrendCode(tank);
  const bool statusChanged = (currentStatus != tankCache.statusCode);

  if (floatChanged(tank.filteredTempC, tankCache.temp) || statusChanged) {
    clearField(x + 10, y + 28, 70, 40);
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
      tft.print(F("ERR"));
    }
    tankCache.temp = tank.filteredTempC;
  }

  tft.setTextSize(1);

  if (floatChanged(tank.requestedSetpointC, tankCache.setpoint)) {
    clearField(x + 28, y + 86, 48, 9);
    config::prepareForTft();
    tft.setTextColor(config::color::COLOR_TEXT);
    tft.setCursor(x + 28, y + 86);
    tft.print(tank.requestedSetpointC, 1);
    tankCache.setpoint = tank.requestedSetpointC;
  }

  if (floatChanged(error, tankCache.error)) {
    clearField(x + 28, y + 98, 48, 9);
    config::prepareForTft();
    tft.setCursor(x + 28, y + 98);
    tft.setTextColor(config::color::COLOR_TEXT);
    if (error >= 0.0f) tft.print(F("+"));
    tft.print(error, 1);
    tankCache.error = error;
  }

  if (floatChanged(tank.rawTempC, tankCache.rawTemp)) {
    clearField(x + 34, y + 110, 42, 9);
    config::prepareForTft();
    tft.setCursor(x + 34, y + 110);
    if (tank.valid) {
      tft.setTextColor(config::color::COLOR_TEXT);
      tft.print(tank.rawTempC, 1);
    } else {
      tft.setTextColor(config::color::COLOR_FAULT);
      tft.print(F("ERR"));
    }
    tankCache.rawTemp = tank.rawTempC;
  }

  if (floatChanged(stddev, tankCache.stddev)) {
    clearField(x + 24, y + 122, 52, 9);
    config::prepareForTft();
    tft.setCursor(x + 24, y + 122);
    tft.setTextColor(config::color::COLOR_TEXT);
    if (isnan(stddev)) tft.print(F("--"));
    else tft.print(stddev, 2);
    tankCache.stddev = stddev;
  }

  if (currentTrend != tankCache.trendCode) {
    clearField(x + 116, y + 86, 28, 9);
    config::prepareForTft();
    tft.setCursor(x + 116, y + 86);
    tft.setTextColor(sensors::getTrendColor(tank));
    tft.print(sensors::getTrendText(tank));
    tankCache.trendCode = currentTrend;
  }

  if (statusChanged) {
    clearField(x + 108, y + 98, 36, 9);
    config::prepareForTft();
    tft.setCursor(x + 108, y + 98);
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
    tft.print(state.selectedTank == config::TANK_MAIN ? F("MAIN") : F("RES"));
    cache.selectedTank = state.selectedTank;
  }

  if (state.editMode != cache.editMode) {
    clearField(190, 204, 48, 10);
    config::prepareForTft();
    tft.setCursor(190, 204);
    tft.setTextColor(state.editMode ? ST77XX_CYAN : ST77XX_RED);
    tft.print(state.editMode ? F("SET") : F("VIEW"));
    cache.editMode = state.editMode;
  }

  if (state.timeAtSetpointSec != cache.atSpSec) {
    char atspBuf[9];
    formatTimeHHMMSS(state.timeAtSetpointSec, atspBuf);
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
    else tft.print(F("ERR"));
    cache.footerMainTemp = state.localTanks[config::TANK_MAIN].filteredTempC;
  }

  if (floatChanged(state.localTanks[config::TANK_RES].filteredTempC, cache.footerResTemp)) {
    clearField(112, 226, 70, 10);
    config::prepareForTft();
    tft.setCursor(112, 226);
    tft.setTextColor(config::color::COLOR_TEXT);
    if (state.localTanks[config::TANK_RES].valid) tft.print(state.localTanks[config::TANK_RES].filteredTempC, 1);
    else tft.print(F("ERR"));
    cache.footerResTemp = state.localTanks[config::TANK_RES].filteredTempC;
  }

  if (state.atSetpoint != cache.atSetpoint) {
    clearField(190, 226, 48, 10);
    config::prepareForTft();
    tft.setCursor(190, 226);
    tft.setTextColor(state.atSetpoint ? ST77XX_GREEN : ST77XX_RED);
    tft.print(state.atSetpoint ? F("YES") : F("NO"));
    cache.atSetpoint = state.atSetpoint;
  }
}

void snapshotCache(const DashboardState& state) {
  if (!state.radioInitOk) cache.radioStatus = -1;
  else if (state.lastTxOk) cache.radioStatus = 1;
  else cache.radioStatus = 0;

  cache.runtimeSec = (millis() - state.bootMs) / 1000UL;

  for (uint8_t index = 0; index < config::TANK_COUNT; ++index) {
    const TankLocalState& tank = state.localTanks[index];
    TankCache& tankCache = cache.tanks[index];
    tankCache.temp = tank.filteredTempC;
    tankCache.statusCode = sensors::getStatusCode(tank);
    tankCache.setpoint = tank.requestedSetpointC;
    tankCache.error = tank.requestedSetpointC - tank.filteredTempC;
    tankCache.rawTemp = tank.rawTempC;
    tankCache.stddev = sensors::computeStdDev(tank);
    tankCache.trendCode = sensors::getTrendCode(tank);
    tankCache.isSelected = (state.selectedTank == index);
    tankCache.inEditMode = state.editMode;
  }

  cache.buttonEvent = state.lastButtonEvent;
  cache.selectedTank = state.selectedTank;
  cache.editMode = state.editMode;
  cache.atSpSec = state.timeAtSetpointSec;
  cache.atSetpoint = state.atSetpoint;
  cache.footerMainTemp = state.localTanks[config::TANK_MAIN].filteredTempC;
  cache.footerResTemp = state.localTanks[config::TANK_RES].filteredTempC;
}

}  // namespace

namespace display {

void init(DashboardState& state) {
  config::prepareForTft();
  tft.init(240, 320);
  tft.setRotation(1);
  tft.setTextWrap(false);
  drawBootDisplayTest();

  drawStaticUI(state);
  drawHeaderRadioField(state);
  drawHeaderRunField(state);
  drawTankValuesDirty(config::layout::MAIN_X, config::layout::CARD_Y,
                      state.localTanks[config::TANK_MAIN], cache.tanks[config::TANK_MAIN]);
  drawTankValuesDirty(config::layout::RES_X, config::layout::CARD_Y,
                      state.localTanks[config::TANK_RES], cache.tanks[config::TANK_RES]);
  drawFooterDirty(state);
  snapshotCache(state);
  state.displayNeedsFullRedraw = false;
  phase = PHASE_IDLE;
}

void update(DashboardState& state) {
  const unsigned long nowMs = millis();
  if ((nowMs - state.lastDisplayMs) < config::timing::DISPLAY_INTERVAL_MS) return;
  state.lastDisplayMs = nowMs;

  if (state.displayNeedsFullRedraw) {
    drawStaticUI(state);
    drawHeaderRadioField(state);
    drawHeaderRunField(state);
    drawTankHeaderDynamic(state, config::TANK_MAIN, config::layout::MAIN_X, config::layout::CARD_Y);
    drawTankHeaderDynamic(state, config::TANK_RES, config::layout::RES_X, config::layout::CARD_Y);
    drawTankValuesDirty(config::layout::MAIN_X, config::layout::CARD_Y,
                        state.localTanks[config::TANK_MAIN], cache.tanks[config::TANK_MAIN]);
    drawTankValuesDirty(config::layout::RES_X, config::layout::CARD_Y,
                        state.localTanks[config::TANK_RES], cache.tanks[config::TANK_RES]);
    drawFooterDirty(state);
    snapshotCache(state);
    phase = PHASE_IDLE;
    state.displayNeedsFullRedraw = false;
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
