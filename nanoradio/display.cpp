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

namespace {

template <size_t N>
struct TextFieldCache {
  char text[N];
  uint16_t color;
  bool valid;
};

Adafruit_ST7789 tft(
    config::pins::TFT_CS_PIN,
    config::pins::TFT_DC_PIN,
    config::pins::TFT_RST_PIN);

TextFieldCache<16> mainLinkCache = {};
TextFieldCache<16> mainModeCache = {};
TextFieldCache<40> mainFaultCache = {};
TextFieldCache<40> mainInfoCache = {};
TextFieldCache<16> mainTempCache[config::TANK_COUNT] = {};
TextFieldCache<8> mainSourceCache[config::TANK_COUNT] = {};
TextFieldCache<8> mainReqCache[config::TANK_COUNT] = {};
TextFieldCache<8> mainActCache[config::TANK_COUNT] = {};
TextFieldCache<16> mainOutCache[config::TANK_COUNT] = {};

TextFieldCache<48> debugLineCache[10] = {};

int8_t mainFrameSelectedTank = -1;
bool mainFrameEditMode = false;

void copyFlashString(char* out, size_t outLen, PGM_P text) {
  if (outLen == 0) return;
  strncpy_P(out, text, outLen - 1);
  out[outLen - 1] = '\0';
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

  const unsigned long hours = totalSec / 3600UL;
  const unsigned long mins = (totalSec % 3600UL) / 60UL;
  const unsigned long secs = totalSec % 60UL;
  snprintf_P(out, outLen, PSTR("%02lu:%02lu:%02lu"), hours, mins, secs);
}

template <size_t N>
void invalidateField(TextFieldCache<N>& cache) {
  cache.text[0] = '\0';
  cache.color = 0;
  cache.valid = false;
}

void invalidateAllCaches() {
  invalidateField(mainLinkCache);
  invalidateField(mainModeCache);
  invalidateField(mainFaultCache);
  invalidateField(mainInfoCache);

  for (uint8_t tank = 0; tank < config::TANK_COUNT; ++tank) {
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

template <size_t N>
void updateTextField(
    TextFieldCache<N>& cache,
    const char* text,
    uint16_t color,
    int16_t x,
    int16_t y,
    int16_t w,
    int16_t h,
    uint8_t textSize,
    uint16_t bgColor = config::color::COLOR_BG) {
  if (w <= 0 || h <= 0) return;

  if (!cache.valid || cache.color != color || strncmp(cache.text, text, sizeof(cache.text)) != 0) {
    config::prepareForTft();
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

uint16_t tankFaultMaskWarn(uint8_t tank) {
  return (tank == config::TANK_MAIN) ? FAULT_MAIN_WARN : FAULT_RES_WARN;
}

uint16_t tankFaultMaskCut(uint8_t tank) {
  return (tank == config::TANK_MAIN) ? FAULT_MAIN_OVERTEMP : FAULT_RES_OVERTEMP;
}

float displayTempForTank(
    const DashboardState& state,
    uint8_t tank,
    unsigned long nowMs,
    bool& valid,
    uint16_t& color,
    const char*& sourceText) {
  if (state.haveMainStatus) {
    const float filteredTemp = decodeTempCx100(
        tank == config::TANK_MAIN ? state.latestMainStatus.mainFilteredCx100
                                  : state.latestMainStatus.resFilteredCx100);

    if (!isnan(filteredTemp)) {
      if (radio::statusPacketHealthy(state, nowMs)) {
        sourceText = "AUTH";
        if ((state.latestMainStatus.faultFlags & tankFaultMaskCut(tank)) != 0U) {
          color = config::color::COLOR_FAULT;
        } else if ((state.latestMainStatus.faultFlags & tankFaultMaskWarn(tank)) != 0U) {
          color = config::color::COLOR_WARN;
        } else {
          color = config::color::COLOR_OK;
        }
        valid = true;
        return filteredTemp;
      }

      if (radio::statusPacketFresh(state, nowMs)) {
        sourceText = "LAST";
        color = config::color::COLOR_WARN;
        valid = true;
        return filteredTemp;
      }
    }
  }

  if (state.localTanks[tank].valid) {
    sourceText = "LOCAL";
    color = config::color::COLOR_LOCAL;
    valid = true;
    return state.localTanks[tank].rawTempC;
  }

  sourceText = "ERR";
  color = config::color::COLOR_FAULT;
  valid = false;
  return NAN;
}

void drawMainStatic() {
  config::prepareForTft();
  tft.fillScreen(config::color::COLOR_BG);

  tft.fillRect(
      config::layout::HEADER_X,
      config::layout::HEADER_Y,
      config::layout::HEADER_W,
      config::layout::HEADER_H,
      config::color::COLOR_PANEL);
  tft.drawFastHLine(0, config::layout::HEADER_H, config::layout::SCREEN_W, config::color::COLOR_DIM);

  tft.setTextSize(2);
  tft.setTextColor(config::color::COLOR_TEXT);
  tft.setCursor(10, 7);
  tft.print(F("REMOTE BOX"));

  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_DIM);
  tft.setCursor(220, 4);
  tft.print(F("LINK"));
  tft.setCursor(220, 17);
  tft.print(F("MODE"));

  tft.drawRoundRect(
      config::layout::MAIN_X,
      config::layout::CARD_Y,
      config::layout::CARD_W,
      config::layout::CARD_H,
      8,
      config::color::COLOR_DIM);
  tft.drawRoundRect(
      config::layout::RES_X,
      config::layout::CARD_Y,
      config::layout::CARD_W,
      config::layout::CARD_H,
      8,
      config::color::COLOR_DIM);
  tft.drawRoundRect(8, config::layout::FOOT_Y, config::layout::SCREEN_W - 16, config::layout::FOOT_H - 8, 6,
                    config::color::COLOR_DIM);

  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_DIM);
  tft.setCursor(16, config::layout::FOOT_Y + 34);
  tft.print(F("SET short=tank  long=edit  UP+DOWN hold=screen"));

  invalidateAllCaches();
}

void drawTankFrame(const DashboardState& state, uint8_t tank) {
  const int x = (tank == config::TANK_MAIN) ? config::layout::MAIN_X : config::layout::RES_X;
  const bool selected = (state.selectedTank == tank);

  const uint16_t borderColor = selected ? ST77XX_YELLOW : config::color::COLOR_DIM;
  const uint16_t titleFill = selected
                                 ? (state.editMode ? config::color::COLOR_ACCENT : ST77XX_BLUE)
                                 : config::color::COLOR_PANEL_ALT;

  config::prepareForTft();
  tft.fillRoundRect(x, config::layout::CARD_Y, config::layout::CARD_W, config::layout::CARD_H, 8,
                    config::color::COLOR_BG);
  tft.drawRoundRect(x, config::layout::CARD_Y, config::layout::CARD_W, config::layout::CARD_H, 8, borderColor);
  tft.fillRoundRect(x + 1, config::layout::CARD_Y + 1, config::layout::CARD_W - 2, 22, 8, titleFill);
  tft.fillRect(x + 2, config::layout::CARD_Y + 18, config::layout::CARD_W - 4, config::layout::CARD_H - 20,
               config::color::COLOR_BG);
  tft.drawFastHLine(x + 8, config::layout::CARD_Y + 78, config::layout::CARD_W - 16, config::color::COLOR_DIM);

  char title[16];
  if (selected) {
    snprintf(title, sizeof(title), "%c %s", state.editMode ? '*' : '>', tank == config::TANK_MAIN ? "MAIN" : "RES");
  } else {
    snprintf(title, sizeof(title), "  %s", tank == config::TANK_MAIN ? "MAIN" : "RES");
  }

  tft.setTextSize(2);
  tft.setTextColor(config::color::COLOR_TEXT, titleFill);
  tft.setCursor(x + 8, config::layout::CARD_Y + 5);
  tft.print(title);

  tft.setTextSize(1);
  tft.setTextColor(config::color::COLOR_DIM);
  tft.setCursor(x + 10, config::layout::CARD_Y + 86);
  tft.print(F("Req"));
  tft.setCursor(x + 10, config::layout::CARD_Y + 102);
  tft.print(F("Act"));
  tft.setCursor(x + 10, config::layout::CARD_Y + 118);
  tft.print(F("Out"));
  tft.setCursor(x + 98, config::layout::CARD_Y + 118);
  tft.print(F("Src"));
}

void ensureMainFramesCurrent(const DashboardState& state) {
  if (mainFrameSelectedTank != static_cast<int8_t>(state.selectedTank) || mainFrameEditMode != state.editMode) {
    drawTankFrame(state, config::TANK_MAIN);
    drawTankFrame(state, config::TANK_RES);
    mainFrameSelectedTank = static_cast<int8_t>(state.selectedTank);
    mainFrameEditMode = state.editMode;

    for (uint8_t tank = 0; tank < config::TANK_COUNT; ++tank) {
      invalidateField(mainTempCache[tank]);
      invalidateField(mainSourceCache[tank]);
      invalidateField(mainReqCache[tank]);
      invalidateField(mainActCache[tank]);
      invalidateField(mainOutCache[tank]);
    }
  }
}

void buildFaultSummary(const DashboardState& state, char* out, size_t outLen, uint16_t& colorOut) {
  const LinkState link = radio::currentLinkState(state, millis());
  const uint16_t faults = state.haveMainStatus ? state.latestMainStatus.faultFlags : 0U;

  if (link == LINK_STATE_NO_RADIO) {
    copyFlashString(out, outLen, PSTR("Radio init failed"));
    colorOut = config::color::COLOR_FAULT;
  } else if (link == LINK_STATE_TIMEOUT) {
    copyFlashString(out, outLen, PSTR("Comm timeout: retrying main box"));
    colorOut = config::color::COLOR_FAULT;
  } else if (link == LINK_STATE_WAITING) {
    copyFlashString(out, outLen, PSTR("Waiting for main-box status"));
    colorOut = config::color::COLOR_WARN;
  } else if ((faults & FAULT_LOCAL_BRIDGE) != 0U) {
    copyFlashString(out, outLen, PSTR("Main-box bridge fault"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((faults & FAULT_REMOTE_COMM) != 0U) {
    copyFlashString(out, outLen, PSTR("Remote comm fault at main box"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((faults & FAULT_MAIN_OVERTEMP) != 0U) {
    copyFlashString(out, outLen, PSTR("MAIN overtemp cutoff"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((faults & FAULT_RES_OVERTEMP) != 0U) {
    copyFlashString(out, outLen, PSTR("RES overtemp cutoff"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((faults & FAULT_MAIN_SENSOR_INVALID) != 0U) {
    copyFlashString(out, outLen, PSTR("MAIN sensor invalid"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((faults & FAULT_RES_SENSOR_INVALID) != 0U) {
    copyFlashString(out, outLen, PSTR("RES sensor invalid"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((faults & (FAULT_MAIN_WARN | FAULT_RES_WARN)) != 0U) {
    copyFlashString(out, outLen, PSTR("Temperature warning active"));
    colorOut = config::color::COLOR_WARN;
  } else if ((state.localTanks[config::TANK_MAIN].valid &&
              state.localTanks[config::TANK_MAIN].rawTempC >= config::temperature::HARD_MAX_TEMP_C) ||
             (state.localTanks[config::TANK_RES].valid &&
              state.localTanks[config::TANK_RES].rawTempC >= config::temperature::HARD_MAX_TEMP_C)) {
    copyFlashString(out, outLen, PSTR("Local sensor over hard max"));
    colorOut = config::color::COLOR_FAULT;
  } else if ((state.localTanks[config::TANK_MAIN].valid &&
              state.localTanks[config::TANK_MAIN].rawTempC >= config::temperature::WARN_TEMP_C) ||
             (state.localTanks[config::TANK_RES].valid &&
              state.localTanks[config::TANK_RES].rawTempC >= config::temperature::WARN_TEMP_C)) {
    copyFlashString(out, outLen, PSTR("Local sensor warning"));
    colorOut = config::color::COLOR_WARN;
  } else if (!state.localTanks[config::TANK_MAIN].valid || !state.localTanks[config::TANK_RES].valid) {
    copyFlashString(out, outLen, PSTR("Check thermocouple inputs"));
    colorOut = config::color::COLOR_WARN;
  } else {
    copyFlashString(out, outLen, PSTR("System OK"));
    colorOut = config::color::COLOR_OK;
  }
}

void updateMainScreen(const DashboardState& state) {
  const unsigned long nowMs = millis();
  ensureMainFramesCurrent(state);

  char line[40];
  const LinkState link = radio::currentLinkState(state, nowMs);

  updateTextField(mainLinkCache, radio::linkStateText(link), radio::linkStateColor(link), 220, 4, 94, 10, 1,
                  config::color::COLOR_PANEL);

  snprintf_P(line, sizeof(line), PSTR("%s %s"), state.editMode ? "EDIT" : "VIEW",
             state.selectedTank == config::TANK_MAIN ? "MAIN" : "RES");
  updateTextField(mainModeCache, line, state.editMode ? config::color::COLOR_ACCENT : config::color::COLOR_TEXT, 220,
                  17, 94, 10, 1, config::color::COLOR_PANEL);

  for (uint8_t tank = 0; tank < config::TANK_COUNT; ++tank) {
    const int x = (tank == config::TANK_MAIN) ? config::layout::MAIN_X : config::layout::RES_X;

    bool tempValid = false;
    uint16_t tempColor = config::color::COLOR_TEXT;
    const char* sourceText = "ERR";
    const float tempC = displayTempForTank(state, tank, nowMs, tempValid, tempColor, sourceText);

    char tempText[16];
    formatTemp(tempText, sizeof(tempText), tempC, tempValid);
    updateTextField(mainTempCache[tank], tempText, tempColor, x + 10, config::layout::CARD_Y + 34, 128, 36, 3,
                    config::color::COLOR_BG);

    const uint16_t sourceColor = strcmp(sourceText, "AUTH") == 0   ? config::color::COLOR_OK
                                 : strcmp(sourceText, "LAST") == 0  ? config::color::COLOR_WARN
                                 : strcmp(sourceText, "LOCAL") == 0 ? config::color::COLOR_LOCAL
                                                                    : config::color::COLOR_FAULT;
    updateTextField(mainSourceCache[tank], sourceText, sourceColor, x + 98, config::layout::CARD_Y + 118, 40, 10, 1,
                    config::color::COLOR_BG);

    formatFixed1(line, sizeof(line), state.localTanks[tank].requestedSetpointC);
    updateTextField(mainReqCache[tank], line, config::color::COLOR_TEXT, x + 36, config::layout::CARD_Y + 86, 48, 10,
                    1, config::color::COLOR_BG);

    bool actValid = false;
    const float actSetpoint = radio::activeSetpointFromMain(state, tank, actValid);
    if (!state.haveMainStatus) {
      copyFlashString(line, sizeof(line), PSTR("--.-"));
    } else {
      formatShortTemp(line, sizeof(line), actSetpoint, actValid);
    }
    uint16_t actColor = radio::statusPacketHealthy(state, nowMs) ? config::color::COLOR_TEXT
                        : radio::statusPacketFresh(state, nowMs)  ? config::color::COLOR_WARN
                                                                  : config::color::COLOR_DIM;
    if (!state.haveMainStatus) actColor = config::color::COLOR_DIM;
    updateTextField(mainActCache[tank], line, actColor, x + 36, config::layout::CARD_Y + 102, 48, 10, 1,
                    config::color::COLOR_BG);

    bool outValid = false;
    const float outputPct = radio::outputPctFromMain(state, tank, outValid);
    if (!outValid) {
      copyFlashString(line, sizeof(line), PSTR("--.-%"));
    } else {
      char pctText[12];
      formatFixed1(pctText, sizeof(pctText), outputPct);
      snprintf_P(line, sizeof(line), PSTR("%s%% %s"), pctText, radio::heaterOnFromMain(state, tank) ? "ON" : "OFF");
    }
    line[sizeof(line) - 1] = '\0';
    updateTextField(mainOutCache[tank], line,
                    radio::heaterOnFromMain(state, tank) ? config::color::COLOR_ACCENT : config::color::COLOR_TEXT,
                    x + 36, config::layout::CARD_Y + 118, 58, 10, 1, config::color::COLOR_BG);
  }

  uint16_t faultColor = config::color::COLOR_TEXT;
  buildFaultSummary(state, line, sizeof(line), faultColor);
  updateTextField(mainFaultCache, line, faultColor, 16, config::layout::FOOT_Y + 10, 288, 12, 1,
                  config::color::COLOR_BG);

  char timeBuf[16];
  char footerBuf[40];
  const unsigned long atSetpointSec = state.haveMainStatus ? state.latestMainStatus.atSetpointSeconds : 0UL;
  formatTimeHHMMSS(timeBuf, sizeof(timeBuf), atSetpointSec);
  snprintf_P(footerBuf, sizeof(footerBuf), PSTR("Ack:%u OK:%lu At:%s"), state.lastStatusSequence, state.txOkCount,
             timeBuf);
  updateTextField(mainInfoCache, footerBuf, config::color::COLOR_DIM, 16, config::layout::FOOT_Y + 22, 288, 12, 1,
                  config::color::COLOR_BG);
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

  for (uint8_t index = 0; index < 10; ++index) {
    invalidateField(debugLineCache[index]);
  }
}

void updateDebugLine(uint8_t index, const char* text, uint16_t color) {
  if (index >= 10) return;
  const int y = 34 + index * 18;
  updateTextField(debugLineCache[index], text, color, 10, y, 300, 12, 1, config::color::COLOR_BG);
}

void updateDebugScreen(const DashboardState& state) {
  const unsigned long nowMs = millis();
  char line[48];
  char tempA[12];
  char tempB[12];

  snprintf_P(line, sizeof(line), PSTR("Link:%s  Screen:%s"), radio::linkStateText(radio::currentLinkState(state, nowMs)),
             state.screenMode == SCREEN_MODE_MAIN ? "MAIN" : "DEBUG");
  updateDebugLine(0, line, radio::linkStateColor(radio::currentLinkState(state, nowMs)));

  snprintf_P(line, sizeof(line), PSTR("Sel:%s  Edit:%s  Btn:0x%02X"),
             state.selectedTank == config::TANK_MAIN ? "MAIN" : "RES", state.editMode ? "ON" : "OFF",
             state.pendingButtonFlags);
  updateDebugLine(1, line, config::color::COLOR_TEXT);

  formatShortTemp(tempA, sizeof(tempA), state.localTanks[config::TANK_MAIN].rawTempC,
                  state.localTanks[config::TANK_MAIN].valid);
  formatShortTemp(tempB, sizeof(tempB), state.localTanks[config::TANK_RES].rawTempC,
                  state.localTanks[config::TANK_RES].valid);
  snprintf_P(line, sizeof(line), PSTR("Raw M:%s  R:%s"), tempA, tempB);
  updateDebugLine(2, line,
                  (!state.localTanks[config::TANK_MAIN].valid || !state.localTanks[config::TANK_RES].valid)
                      ? config::color::COLOR_WARN
                      : config::color::COLOR_TEXT);

  formatFixed1(tempA, sizeof(tempA), state.localTanks[config::TANK_MAIN].requestedSetpointC);
  formatFixed1(tempB, sizeof(tempB), state.localTanks[config::TANK_RES].requestedSetpointC);
  snprintf_P(line, sizeof(line), PSTR("Req M:%s  R:%s"), tempA, tempB);
  updateDebugLine(3, line, config::color::COLOR_TEXT);

  if (state.haveMainStatus) {
    formatShortTemp(tempA, sizeof(tempA), decodeTempCx100(state.latestMainStatus.mainSetpointCx100), true);
    formatShortTemp(tempB, sizeof(tempB), decodeTempCx100(state.latestMainStatus.resSetpointCx100), true);
    snprintf_P(line, sizeof(line), PSTR("Act M:%s  R:%s"), tempA, tempB);
  } else {
    copyFlashString(line, sizeof(line), PSTR("Act M:--.-  R:--.-"));
  }
  updateDebugLine(4, line,
                  radio::statusPacketFresh(state, nowMs) ? config::color::COLOR_TEXT : config::color::COLOR_WARN);

  snprintf_P(line, sizeof(line), PSTR("TX seq:%u  RX seq:%u"), state.lastOutboundPacket.sequence,
             state.haveMainStatus ? state.latestMainStatus.statusSequence : 0U);
  updateDebugLine(5, line, config::color::COLOR_TEXT);

  snprintf_P(line, sizeof(line), PSTR("RX age:%lu ms  Ctrl age:%u"),
             state.haveMainStatus ? (nowMs - state.lastStatusRxMs) : 65535UL,
             state.haveMainStatus ? state.latestMainStatus.controllerAgeMs : 65535U);
  updateDebugLine(6, line,
                  radio::statusPacketFresh(state, nowMs) ? config::color::COLOR_TEXT : config::color::COLOR_WARN);

  if (state.haveMainStatus) {
    formatFixed1(tempA, sizeof(tempA), decodeOutputPermille(state.latestMainStatus.mainOutputPermille));
    formatFixed1(tempB, sizeof(tempB), decodeOutputPermille(state.latestMainStatus.resOutputPermille));
    snprintf_P(line, sizeof(line), PSTR("Out M:%s  R:%s  Heat:%02X"), tempA, tempB, state.latestMainStatus.heaterFlags);
  } else {
    copyFlashString(line, sizeof(line), PSTR("Out M:--.-  R:--.-  Heat:00"));
  }
  updateDebugLine(7, line, config::color::COLOR_TEXT);

  snprintf_P(line, sizeof(line), PSTR("Fault:0x%04X  Link:0x%02X"),
             state.haveMainStatus ? state.latestMainStatus.faultFlags : 0U,
             state.haveMainStatus ? state.latestMainStatus.linkFlags : 0U);
  updateDebugLine(8, line,
                  (state.haveMainStatus && state.latestMainStatus.faultFlags != 0U) ? config::color::COLOR_WARN
                                                                                    : config::color::COLOR_TEXT);

  snprintf_P(line, sizeof(line), PSTR("TX ok:%lu fail:%lu  RX ok:%lu bad:%lu"), state.txOkCount, state.txFailCount,
             state.rxOkCount, state.rxBadCount);
  updateDebugLine(9, line, config::color::COLOR_DIM);
}

}  // namespace

namespace display {

void init(DashboardState& state) {
  config::prepareForTft();
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(config::color::COLOR_BG);
  tft.setTextWrap(false);
  state.displayNeedsFullRedraw = true;
}

void update(DashboardState& state) {
  const unsigned long nowMs = millis();
  if ((nowMs - state.lastDisplayMs) < config::timing::DISPLAY_INTERVAL_MS) return;
  state.lastDisplayMs = nowMs;

  if (state.displayNeedsFullRedraw) {
    if (state.screenMode == SCREEN_MODE_MAIN) {
      drawMainStatic();
    } else {
      drawDebugStatic();
    }
    state.displayNeedsFullRedraw = false;
  }

  if (state.screenMode == SCREEN_MODE_MAIN) {
    updateMainScreen(state);
  } else {
    updateDebugScreen(state);
  }
}

}  // namespace display
