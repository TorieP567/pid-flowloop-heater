#include "display.h"
#include "config.h"
#include "sensors.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

static Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ============================================================================
// Phase state machine + dirty-field cache
// ============================================================================

enum DisplayPhase {
  PHASE_IDLE,
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
  bool inSetMode;
};

struct CachedFields {
  int8_t radioStatus;         // -1=NO_INIT, 0=FAIL, 1=OK
  unsigned long runtimeSec;
  TankCache tanks[2];
  const char* buttonEvent;
  uint8_t selectedTank;
  bool setMode;
  unsigned long atSpSec;
  bool atSetpoint;
  float footerMainTemp;
  float footerResTemp;
};

static DisplayPhase phase = PHASE_IDLE;
static CachedFields cache;

static bool floatChanged(float a, float b, float tol = 0.05f) {
  if (isnan(a) != isnan(b)) return true;
  return fabsf(a - b) > tol;
}

// ============================================================================
// Static helpers (file-scope only) — used during boot init
// ============================================================================

static void clearField(int x, int y, int w, int h) {
  deselectRadio();
  tft.fillRect(x, y, w, h, ST77XX_BLACK);
}

static void drawTankFrame(const DashboardState& state, int tankIndex, int x, int y) {
  bool isSelected = (state.selectedTank == tankIndex);
  uint16_t borderColor = isSelected ? ST77XX_YELLOW : ST77XX_WHITE;
  uint16_t titleFill   = isSelected ? (state.setMode ? ST77XX_CYAN : ST77XX_MAGENTA) : ST77XX_BLUE;

  deselectRadio();

  tft.drawRoundRect(x, y, CARD_W, CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, y + 1, CARD_W - 2, 20, 8, titleFill);
  tft.fillRect(x + 2, y + 16, CARD_W - 4, CARD_H - 18, ST77XX_BLACK);

  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x + 8, y + 4);
  if (isSelected) tft.print(state.setMode ? "* " : "> ");
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

static void drawStaticUI(const DashboardState& state) {
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
  drawTankFrame(state, 0, MAIN_X, CARD_Y);
  drawTankFrame(state, 1, RES_X,  CARD_Y);

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

// ============================================================================
// Original draw primitives — used for boot and by dirty-check functions
// ============================================================================

static void drawHeaderRadioField(const DashboardState& state) {
  clearField(194, 16, 55, 10);
  deselectRadio();
  tft.setTextSize(1);
  tft.setCursor(194, 16);

  if (!state.radioInitOk) {
    tft.setTextColor(ST77XX_RED);
    tft.print("NO INIT");
  } else if (state.lastTxOk) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("TX OK");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("TX FAIL");
  }
}

static void drawHeaderRunField(const DashboardState& state) {
  char buf[9];
  formatTimeHHMMSS((millis() - state.bootMs) / 1000UL, buf);

  clearField(254, 16, 58, 10);
  deselectRadio();
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(254, 16);
  tft.print(buf);
}

static void drawTankHeaderDynamic(const DashboardState& state, int tankIndex, int x, int y) {
  bool isSelected = (state.selectedTank == tankIndex);
  uint16_t borderColor = isSelected ? ST77XX_YELLOW : ST77XX_WHITE;
  uint16_t titleFill   = isSelected ? (state.setMode ? ST77XX_CYAN : ST77XX_MAGENTA) : ST77XX_BLUE;

  // redraw only header/top area + border so selection/mode style updates
  deselectRadio();
  tft.drawRoundRect(x, y, CARD_W, CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, y + 1, CARD_W - 2, 20, 8, titleFill);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, titleFill);
  tft.setCursor(x + 8, y + 4);
  if (isSelected) tft.print(state.setMode ? "* " : "> ");
  else            tft.print("  ");
  tft.print(tankIndex == 0 ? "MAIN" : "RES");
}

static void drawTankValues(int x, int y, const TankState &t) {
  float err = t.setpoint - t.filteredTemp;
  float sd  = sensors::computeStdDev(t);

  // Big temperature
  clearField(x + 10, y + 28, 70, 40);
  deselectRadio();
  if (isValidTemp(t.rawTemp)) {
    tft.setTextSize(4);
    tft.setTextColor(sensors::getStatusColor(t));
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
  tft.setTextColor(sensors::getTrendColor(t));
  tft.print(sensors::getTrendText(t));

  // Status
  clearField(x + 108, y + 98, 36, 9);
  tft.setCursor(x + 108, y + 98);
  tft.setTextColor(sensors::getStatusColor(t));
  tft.print(sensors::getStatusText(t));
}

static void drawFooterFields(const DashboardState& state) {
  char atspBuf[9];
  formatTimeHHMMSS(state.timeAtSetpointSec, atspBuf);

  // BTN
  clearField(10, 204, 88, 10);
  deselectRadio();
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 204);
  tft.print(state.lastButtonEvent);

  // SEL
  clearField(112, 204, 60, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(112, 204);
  tft.print(state.selectedTank == 0 ? "MAIN" : "RES");

  // MODE
  clearField(190, 204, 48, 10);
  tft.setCursor(190, 204);
  tft.setTextColor(state.setMode ? ST77XX_CYAN : ST77XX_RED);
  tft.print(state.setMode ? "SET" : "VIEW");

  // ATSP
  clearField(255, 204, 55, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(255, 204);
  tft.print(atspBuf);

  // MAIN temp footer
  clearField(10, 226, 80, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 226);
  if (isValidTemp(state.main.rawTemp)) tft.print(state.main.filteredTemp, 1);
  else tft.print("ERR");

  // RES temp footer
  clearField(112, 226, 70, 10);
  tft.setCursor(112, 226);
  if (isValidTemp(state.res.rawTemp)) tft.print(state.res.filteredTemp, 1);
  else tft.print("ERR");

  // BAND
  clearField(190, 226, 48, 10);
  tft.setCursor(190, 226);
  tft.setTextColor(state.atSetpoint ? ST77XX_GREEN : ST77XX_RED);
  tft.print(state.atSetpoint ? "YES" : "NO");
}

static void updateDynamicUI(const DashboardState& state) {
  drawHeaderRadioField(state);
  drawHeaderRunField(state);

  drawTankHeaderDynamic(state, 0, MAIN_X, CARD_Y);
  drawTankHeaderDynamic(state, 1, RES_X,  CARD_Y);

  drawTankValues(MAIN_X, CARD_Y, state.main);
  drawTankValues(RES_X,  CARD_Y, state.res);

  drawFooterFields(state);
}

// ============================================================================
// Dirty-check draw functions (used by phase state machine)
// ============================================================================

static void drawHeaderDirty(const DashboardState& state) {
  // Radio status
  int8_t curRadio;
  if (!state.radioInitOk) curRadio = -1;
  else if (state.lastTxOk) curRadio = 1;
  else curRadio = 0;

  if (curRadio != cache.radioStatus) {
    drawHeaderRadioField(state);
    cache.radioStatus = curRadio;
  }

  // Runtime
  unsigned long curRuntime = (millis() - state.bootMs) / 1000UL;
  if (curRuntime != cache.runtimeSec) {
    drawHeaderRunField(state);
    cache.runtimeSec = curRuntime;
  }
}

static void drawTankHeaderDirty(const DashboardState& state, int tankIndex, int x, int y) {
  TankCache& tc = cache.tanks[tankIndex];
  bool isSelected = (state.selectedTank == tankIndex);
  bool inSetMode = state.setMode;

  if (isSelected != tc.isSelected || inSetMode != tc.inSetMode) {
    drawTankHeaderDynamic(state, tankIndex, x, y);
    tc.isSelected = isSelected;
    tc.inSetMode = inSetMode;
  }
}

static void drawTankValuesDirty(int x, int y, const TankState& t, TankCache& tc) {
  float err = t.setpoint - t.filteredTemp;
  float sd  = sensors::computeStdDev(t);
  int8_t curStatus = sensors::getStatusCode(t);
  int8_t curTrend  = sensors::getTrendCode(t);
  bool statusChanged = (curStatus != tc.statusCode);

  // Big temperature — redraws if temp or status color changed
  if (floatChanged(t.filteredTemp, tc.temp) || statusChanged) {
    clearField(x + 10, y + 28, 70, 40);
    deselectRadio();
    if (isValidTemp(t.rawTemp)) {
      tft.setTextSize(4);
      tft.setTextColor(sensors::getStatusColor(t));
      tft.setCursor(x + 10, y + 30);
      tft.print(t.filteredTemp, 1);
    } else {
      tft.setTextSize(3);
      tft.setTextColor(ST77XX_RED);
      tft.setCursor(x + 10, y + 34);
      tft.print("ERR");
    }
    tc.temp = t.filteredTemp;
  }

  tft.setTextSize(1);

  // SP
  if (floatChanged(t.setpoint, tc.setpoint)) {
    clearField(x + 28, y + 86, 48, 9);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(x + 28, y + 86);
    tft.print(t.setpoint, 1);
    tc.setpoint = t.setpoint;
  }

  // dT
  if (floatChanged(err, tc.error)) {
    clearField(x + 28, y + 98, 48, 9);
    tft.setCursor(x + 28, y + 98);
    tft.setTextColor(ST77XX_WHITE);
    if (err >= 0) tft.print("+");
    tft.print(err, 1);
    tc.error = err;
  }

  // RAW
  if (floatChanged(t.rawTemp, tc.rawTemp)) {
    clearField(x + 34, y + 110, 42, 9);
    tft.setCursor(x + 34, y + 110);
    if (isValidTemp(t.rawTemp)) {
      tft.setTextColor(ST77XX_WHITE);
      tft.print(t.rawTemp, 1);
    } else {
      tft.setTextColor(ST77XX_RED);
      tft.print("ERR");
    }
    tc.rawTemp = t.rawTemp;
  }

  // SD
  if (floatChanged(sd, tc.stddev)) {
    clearField(x + 24, y + 122, 52, 9);
    tft.setCursor(x + 24, y + 122);
    tft.setTextColor(ST77XX_WHITE);
    if (isnan(sd)) tft.print("--");
    else tft.print(sd, 2);
    tc.stddev = sd;
  }

  // Trend
  if (curTrend != tc.trendCode) {
    clearField(x + 116, y + 86, 28, 9);
    tft.setCursor(x + 116, y + 86);
    tft.setTextColor(sensors::getTrendColor(t));
    tft.print(sensors::getTrendText(t));
    tc.trendCode = curTrend;
  }

  // Status text
  if (statusChanged) {
    clearField(x + 108, y + 98, 36, 9);
    tft.setCursor(x + 108, y + 98);
    tft.setTextColor(sensors::getStatusColor(t));
    tft.print(sensors::getStatusText(t));
    tc.statusCode = curStatus;
  }
}

static void drawFooterDirty(const DashboardState& state) {
  deselectRadio();
  tft.setTextSize(1);

  // BTN
  if (state.lastButtonEvent != cache.buttonEvent) {
    clearField(10, 204, 88, 10);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 204);
    tft.print(state.lastButtonEvent);
    cache.buttonEvent = state.lastButtonEvent;
  }

  // SEL
  if (state.selectedTank != cache.selectedTank) {
    clearField(112, 204, 60, 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(112, 204);
    tft.print(state.selectedTank == 0 ? "MAIN" : "RES");
    cache.selectedTank = state.selectedTank;
  }

  // MODE
  if (state.setMode != cache.setMode) {
    clearField(190, 204, 48, 10);
    tft.setCursor(190, 204);
    tft.setTextColor(state.setMode ? ST77XX_CYAN : ST77XX_RED);
    tft.print(state.setMode ? "SET" : "VIEW");
    cache.setMode = state.setMode;
  }

  // ATSP
  if (state.timeAtSetpointSec != cache.atSpSec) {
    char atspBuf[9];
    formatTimeHHMMSS(state.timeAtSetpointSec, atspBuf);
    clearField(255, 204, 55, 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(255, 204);
    tft.print(atspBuf);
    cache.atSpSec = state.timeAtSetpointSec;
  }

  // MAIN temp footer
  if (floatChanged(state.main.filteredTemp, cache.footerMainTemp)) {
    clearField(10, 226, 80, 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 226);
    if (isValidTemp(state.main.rawTemp)) tft.print(state.main.filteredTemp, 1);
    else tft.print("ERR");
    cache.footerMainTemp = state.main.filteredTemp;
  }

  // RES temp footer
  if (floatChanged(state.res.filteredTemp, cache.footerResTemp)) {
    clearField(112, 226, 70, 10);
    tft.setCursor(112, 226);
    if (isValidTemp(state.res.rawTemp)) tft.print(state.res.filteredTemp, 1);
    else tft.print("ERR");
    cache.footerResTemp = state.res.filteredTemp;
  }

  // BAND
  if (state.atSetpoint != cache.atSetpoint) {
    clearField(190, 226, 48, 10);
    tft.setCursor(190, 226);
    tft.setTextColor(state.atSetpoint ? ST77XX_GREEN : ST77XX_RED);
    tft.print(state.atSetpoint ? "YES" : "NO");
    cache.atSetpoint = state.atSetpoint;
  }
}

// ============================================================================
// Cache snapshot helper
// ============================================================================

static void snapshotCache(const DashboardState& state) {
  if (!state.radioInitOk) cache.radioStatus = -1;
  else if (state.lastTxOk) cache.radioStatus = 1;
  else cache.radioStatus = 0;
  cache.runtimeSec = (millis() - state.bootMs) / 1000UL;

  for (int i = 0; i < 2; i++) {
    const TankState& t = (i == 0) ? state.main : state.res;
    TankCache& tc = cache.tanks[i];
    tc.temp = t.filteredTemp;
    tc.statusCode = sensors::getStatusCode(t);
    tc.setpoint = t.setpoint;
    tc.error = t.setpoint - t.filteredTemp;
    tc.rawTemp = t.rawTemp;
    tc.stddev = sensors::computeStdDev(t);
    tc.trendCode = sensors::getTrendCode(t);
    tc.isSelected = (state.selectedTank == i);
    tc.inSetMode = state.setMode;
  }

  cache.buttonEvent = state.lastButtonEvent;
  cache.selectedTank = state.selectedTank;
  cache.setMode = state.setMode;
  cache.atSpSec = state.timeAtSetpointSec;
  cache.atSetpoint = state.atSetpoint;
  cache.footerMainTemp = state.main.filteredTemp;
  cache.footerResTemp = state.res.filteredTemp;
}

// ============================================================================
// Public API
// ============================================================================

void display::init(DashboardState& state) {
  tft.init(240, 320);
  tft.setRotation(1);   // landscape
  tft.setTextWrap(false);

  drawStaticUI(state);
  updateDynamicUI(state);
  snapshotCache(state);
}

void display::update(DashboardState& state) {
  switch (phase) {
    case PHASE_IDLE:
      if (state.displayDirty) phase = PHASE_HEADER;
      return;
    case PHASE_HEADER:
      drawHeaderDirty(state);
      phase = PHASE_TANK0_HEADER;
      return;
    case PHASE_TANK0_HEADER:
      drawTankHeaderDirty(state, 0, MAIN_X, CARD_Y);
      phase = PHASE_TANK0_VALUES;
      return;
    case PHASE_TANK0_VALUES:
      drawTankValuesDirty(MAIN_X, CARD_Y, state.main, cache.tanks[0]);
      phase = PHASE_TANK1_HEADER;
      return;
    case PHASE_TANK1_HEADER:
      drawTankHeaderDirty(state, 1, RES_X, CARD_Y);
      phase = PHASE_TANK1_VALUES;
      return;
    case PHASE_TANK1_VALUES:
      drawTankValuesDirty(RES_X, CARD_Y, state.res, cache.tanks[1]);
      phase = PHASE_FOOTER;
      return;
    case PHASE_FOOTER:
      drawFooterDirty(state);
      state.displayDirty = false;
      phase = PHASE_IDLE;
      return;
  }
}
