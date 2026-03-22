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
  int8_t radioStatus;
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
// Drawing helpers
// ============================================================================

// Label color — visible but subdued (light grey)
#define LABEL_COLOR 0xB596

static void clearField(int x, int y, int w, int h) {
  deselectRadio();
  tft.fillRect(x, y, w, h, ST77XX_BLACK);
}

// Color for error-from-setpoint: green=close, yellow=medium, orange=far
static uint16_t dtColor(float err) {
  float a = fabsf(err);
  if (a <= 0.5f) return ST77XX_GREEN;
  if (a <= 2.0f) return ST77XX_YELLOW;
  return 0xFD20;  // orange
}

// ============================================================================
// Static UI (drawn once at boot)
// ============================================================================

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

  tft.setTextSize(1);
  tft.setTextColor(LABEL_COLOR);

  tft.setCursor(x + 8,  y + 86);  tft.print("SP");
  tft.setCursor(x + 8,  y + 98);  tft.print("dT");
  tft.setCursor(x + 8,  y + 110); tft.print("RAW");
  tft.setCursor(x + 8,  y + 122); tft.print("SD");

  tft.setCursor(x + 86, y + 86);  tft.print("TRD");
  tft.setCursor(x + 86, y + 98);  tft.print("ST");

  // Separator between big temp and detail rows
  tft.drawFastHLine(x + 6, y + 78, CARD_W - 12, 0x4208);
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
  tft.setTextColor(LABEL_COLOR);
  tft.setCursor(195, 8);  tft.print("RADIO");
  tft.setCursor(255, 8);  tft.print("RUN");

  // Cards
  drawTankFrame(state, 0, MAIN_X, CARD_Y);
  drawTankFrame(state, 1, RES_X,  CARD_Y);

  // Footer
  tft.drawRoundRect(FOOT_X, FOOT_Y, FOOT_W, FOOT_H, 6, 0x4208);
  tft.setTextSize(1);
  tft.setTextColor(LABEL_COLOR);

  tft.setCursor(10, 194);   tft.print("BTN");
  tft.setCursor(112, 194);  tft.print("SEL");
  tft.setCursor(190, 194);  tft.print("MODE");
  tft.setCursor(255, 194);  tft.print("ATSP");

  tft.setCursor(10, 216);   tft.print("MAIN");
  tft.setCursor(112, 216);  tft.print("RES");
  tft.setCursor(190, 216);  tft.print("BAND");
}

// ============================================================================
// Dynamic draw primitives
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

// ============================================================================
// Dirty-check draw functions (phase state machine)
// ============================================================================

static void drawHeaderDirty(const DashboardState& state) {
  int8_t curRadio;
  if (!state.radioInitOk) curRadio = -1;
  else if (state.lastTxOk) curRadio = 1;
  else curRadio = 0;

  if (curRadio != cache.radioStatus) {
    drawHeaderRadioField(state);
    cache.radioStatus = curRadio;
  }

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

  // Big temperature
  if (floatChanged(t.filteredTemp, tc.temp) || statusChanged) {
    clearField(x + 8, y + 28, 134, 40);
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
    clearField(x + 26, y + 86, 50, 9);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(x + 26, y + 86);
    tft.print(t.setpoint, 1);
    tc.setpoint = t.setpoint;
  }

  // dT — color-coded by distance from setpoint
  if (floatChanged(err, tc.error)) {
    clearField(x + 26, y + 98, 50, 9);
    tft.setCursor(x + 26, y + 98);
    tft.setTextColor(dtColor(err));
    if (err >= 0) tft.print('+');
    tft.print(err, 1);
    tc.error = err;
  }

  // RAW
  if (floatChanged(t.rawTemp, tc.rawTemp)) {
    clearField(x + 32, y + 110, 44, 9);
    tft.setCursor(x + 32, y + 110);
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
    clearField(x + 26, y + 122, 52, 9);
    tft.setCursor(x + 26, y + 122);
    tft.setTextColor(ST77XX_WHITE);
    if (isnan(sd)) tft.print("--");
    else tft.print(sd, 2);
    tc.stddev = sd;
  }

  // Trend
  if (curTrend != tc.trendCode) {
    clearField(x + 110, y + 86, 34, 9);
    tft.setCursor(x + 110, y + 86);
    tft.setTextColor(sensors::getTrendColor(t));
    tft.print(sensors::getTrendText(t));
    tc.trendCode = curTrend;
  }

  // Status
  if (statusChanged) {
    clearField(x + 110, y + 98, 34, 9);
    tft.setCursor(x + 110, y + 98);
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
    tft.setTextColor(state.setMode ? ST77XX_CYAN : ST77XX_GREEN);
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
// Invalidate cache so dirty-check redraws everything on first pass
// ============================================================================

static void invalidateCache() {
  cache.radioStatus = 127;
  cache.runtimeSec = 0xFFFFFFFF;
  for (int i = 0; i < 2; i++) {
    TankCache& tc = cache.tanks[i];
    tc.temp = NAN;
    tc.statusCode = 127;
    tc.setpoint = NAN;
    tc.error = NAN;
    tc.rawTemp = NAN;
    tc.stddev = NAN;
    tc.trendCode = 127;
    tc.isSelected = !!(i);
    tc.inSetMode = true;
  }
  cache.buttonEvent = nullptr;
  cache.selectedTank = 255;
  cache.setMode = true;
  cache.atSpSec = 0xFFFFFFFF;
  cache.atSetpoint = true;
  cache.footerMainTemp = NAN;
  cache.footerResTemp = NAN;
}

// ============================================================================
// Public API
// ============================================================================

void display::init(DashboardState& state) {
  tft.init(240, 320);
  tft.setRotation(1);
  tft.setTextWrap(false);

  drawStaticUI(state);
  invalidateCache();
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
