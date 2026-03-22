// nanoradio/sensors.cpp
#include "sensors.h"
#include "config.h"
#include <max6675.h>
#include <Adafruit_ST7789.h>  // for ST77XX color constants

// ---------------- Hardware ----------------
static MAX6675 thermocoupleMain(THERMO_SCK, THERMO_CS_MAIN, THERMO_SO_MAIN);
static MAX6675 thermocoupleRes(THERMO_SCK, THERMO_CS_RES, THERMO_SO_RES);

// ---------------- Timing state ----------------
static bool sensorsReady = false;
static unsigned long sensorsReadyAt = 0;
static unsigned long lastSensorRead = 0;
static unsigned long lastAtSetpointCheck = 0;

// ---------------- Static helpers ----------------
static void seedHistory(TankState &t, float value) {
  t.historyIndex = 0;
  t.historyCount = 0;
  for (int i = 0; i < HISTORY_LEN; i++) {
    t.history[i] = value;
  }
}

static void pushHistory(TankState &t, float value) {
  t.history[t.historyIndex] = value;
  t.historyIndex = (t.historyIndex + 1) % HISTORY_LEN;
  if (t.historyCount < HISTORY_LEN) t.historyCount++;
}

static void readSensors(DashboardState& state) {
  state.main.rawTemp = thermocoupleMain.readCelsius();
  state.res.rawTemp  = thermocoupleRes.readCelsius();

  if (isValidTemp(state.main.rawTemp)) {
    state.main.filteredTemp = EMA_ALPHA * state.main.rawTemp + (1.0f - EMA_ALPHA) * state.main.filteredTemp;
    pushHistory(state.main, state.main.filteredTemp);
  }

  if (isValidTemp(state.res.rawTemp)) {
    state.res.filteredTemp = EMA_ALPHA * state.res.rawTemp + (1.0f - EMA_ALPHA) * state.res.filteredTemp;
    pushHistory(state.res, state.res.filteredTemp);
  }
}

static bool bothAtSetpoint(DashboardState& state) {
  if (!isValidTemp(state.main.rawTemp) || !isValidTemp(state.res.rawTemp)) return false;

  bool mainOK = fabs(state.main.filteredTemp - state.main.setpoint) <= AT_SETPOINT_BAND_C;
  bool resOK  = fabs(state.res.filteredTemp  - state.res.setpoint)  <= AT_SETPOINT_BAND_C;

  return mainOK && resOK &&
         sensors::getStatusCode(state.main) != 2 &&
         sensors::getStatusCode(state.res)  != 2;
}

static void updateAtSetpointTimer(DashboardState& state) {
  unsigned long now = millis();
  bool nowInBand = bothAtSetpoint(state);

  if (nowInBand) {
    unsigned long elapsed = now - lastAtSetpointCheck;
    if (elapsed >= 1000UL) {
      unsigned long secs = elapsed / 1000UL;
      state.timeAtSetpointSec += secs;
      if (state.timeAtSetpointSec > 359999UL) state.timeAtSetpointSec = 359999UL;
      lastAtSetpointCheck = now - (elapsed % 1000UL);
    }
    state.atSetpoint = true;
  } else {
    state.timeAtSetpointSec = 0;
    lastAtSetpointCheck = now;
    state.atSetpoint = false;
  }
}

// ---------------- Public API ----------------
namespace sensors {

void init(DashboardState& state) {
  sensorsReadyAt = millis() + MAX6675_STARTUP_MS;
}

bool isReady() {
  return sensorsReady;
}

void update(DashboardState& state) {
  unsigned long now = millis();

  if (!sensorsReady) {
    if (now >= sensorsReadyAt) {
      sensorsReady = true;
      readSensors(state);

      if (!isValidTemp(state.main.rawTemp)) state.main.filteredTemp = 37.0f;
      if (!isValidTemp(state.res.rawTemp))  state.res.filteredTemp  = 37.0f;

      seedHistory(state.main, state.main.filteredTemp);
      seedHistory(state.res,  state.res.filteredTemp);

      lastAtSetpointCheck = now;
      updateAtSetpointTimer(state);
      state.displayDirty = true;
    }
    return;
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors(state);
    updateAtSetpointTimer(state);
    state.displayDirty = true;
  }
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

} // namespace sensors
