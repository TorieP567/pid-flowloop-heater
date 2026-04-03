#include "sensors.h"

#include <math.h>

#include <max6675.h>

#include "config.h"

namespace {

MAX6675 thermocoupleMain(
    config::pins::THERMO_SCK_PIN,
    config::pins::THERMO_CS_MAIN_PIN,
    config::pins::THERMO_SO_MAIN_PIN);

MAX6675 thermocoupleRes(
    config::pins::THERMO_SCK_PIN,
    config::pins::THERMO_CS_RES_PIN,
    config::pins::THERMO_SO_RES_PIN);

unsigned long lastAtSetpointCheckMs = 0;

void seedHistory(TankLocalState& tank, float value) {
  tank.historyIndex = 0;
  tank.historyCount = 0;
  for (uint8_t index = 0; index < config::ui::HISTORY_LEN; ++index) {
    tank.history[index] = value;
  }
}

void pushHistory(TankLocalState& tank, float value) {
  tank.history[tank.historyIndex] = value;
  tank.historyIndex = (tank.historyIndex + 1) % config::ui::HISTORY_LEN;
  if (tank.historyCount < config::ui::HISTORY_LEN) {
    ++tank.historyCount;
  }
}

void readSensors(DashboardState& state) {
  config::prepareForSensors();
  const float mainTemp = thermocoupleMain.readCelsius();

  config::prepareForSensors();
  const float resTemp = thermocoupleRes.readCelsius();

  state.localTanks[config::TANK_MAIN].valid = config::isReasonableTemp(mainTemp);
  state.localTanks[config::TANK_MAIN].rawTempC =
      state.localTanks[config::TANK_MAIN].valid ? mainTemp : NAN;

  state.localTanks[config::TANK_RES].valid = config::isReasonableTemp(resTemp);
  state.localTanks[config::TANK_RES].rawTempC =
      state.localTanks[config::TANK_RES].valid ? resTemp : NAN;

  if (state.localTanks[config::TANK_MAIN].valid) {
    state.localTanks[config::TANK_MAIN].filteredTempC =
        config::ui::EMA_ALPHA * state.localTanks[config::TANK_MAIN].rawTempC +
        (1.0f - config::ui::EMA_ALPHA) * state.localTanks[config::TANK_MAIN].filteredTempC;
    pushHistory(state.localTanks[config::TANK_MAIN], state.localTanks[config::TANK_MAIN].filteredTempC);
  }

  if (state.localTanks[config::TANK_RES].valid) {
    state.localTanks[config::TANK_RES].filteredTempC =
        config::ui::EMA_ALPHA * state.localTanks[config::TANK_RES].rawTempC +
        (1.0f - config::ui::EMA_ALPHA) * state.localTanks[config::TANK_RES].filteredTempC;
    pushHistory(state.localTanks[config::TANK_RES], state.localTanks[config::TANK_RES].filteredTempC);
  }
}

bool bothAtSetpoint(const DashboardState& state) {
  if (!state.localTanks[config::TANK_MAIN].valid || !state.localTanks[config::TANK_RES].valid) {
    return false;
  }

  const bool mainOk =
      fabsf(state.localTanks[config::TANK_MAIN].filteredTempC -
            state.localTanks[config::TANK_MAIN].requestedSetpointC) <= config::ui::AT_SETPOINT_BAND_C;
  const bool resOk =
      fabsf(state.localTanks[config::TANK_RES].filteredTempC -
            state.localTanks[config::TANK_RES].requestedSetpointC) <= config::ui::AT_SETPOINT_BAND_C;

  return mainOk && resOk &&
         sensors::getStatusCode(state.localTanks[config::TANK_MAIN]) != 2 &&
         sensors::getStatusCode(state.localTanks[config::TANK_RES]) != 2;
}

void updateAtSetpointTimer(DashboardState& state) {
  const unsigned long nowMs = millis();
  const bool nowInBand = bothAtSetpoint(state);

  if (nowInBand) {
    const unsigned long elapsedMs = nowMs - lastAtSetpointCheckMs;
    if (elapsedMs >= 1000UL) {
      const unsigned long elapsedSec = elapsedMs / 1000UL;
      state.timeAtSetpointSec += elapsedSec;
      if (state.timeAtSetpointSec > 359999UL) {
        state.timeAtSetpointSec = 359999UL;
      }
      lastAtSetpointCheckMs = nowMs - (elapsedMs % 1000UL);
    }
    state.atSetpoint = true;
  } else {
    state.timeAtSetpointSec = 0;
    lastAtSetpointCheckMs = nowMs;
    state.atSetpoint = false;
  }
}

}  // namespace

namespace sensors {

void init(DashboardState& state) {
  state.sensorsReady = false;
  state.sensorsReadyAtMs = millis() + config::timing::MAX6675_STARTUP_MS;
  state.lastSensorReadMs = 0;
  lastAtSetpointCheckMs = millis();

  for (uint8_t tank = 0; tank < config::TANK_COUNT; ++tank) {
    state.localTanks[tank].filteredTempC = config::setpoint::DEFAULT_SETPOINT_C;
    seedHistory(state.localTanks[tank], state.localTanks[tank].filteredTempC);
  }

  state.timeAtSetpointSec = 0;
  state.atSetpoint = false;
}

void update(DashboardState& state) {
  const unsigned long nowMs = millis();

  if (!state.sensorsReady) {
    if (nowMs >= state.sensorsReadyAtMs) {
      state.sensorsReady = true;
      state.lastSensorReadMs = nowMs;
      readSensors(state);

      for (uint8_t tank = 0; tank < config::TANK_COUNT; ++tank) {
        if (!state.localTanks[tank].valid) {
          state.localTanks[tank].filteredTempC = config::setpoint::DEFAULT_SETPOINT_C;
        }
        seedHistory(state.localTanks[tank], state.localTanks[tank].filteredTempC);
      }

      lastAtSetpointCheckMs = nowMs;
      updateAtSetpointTimer(state);
    }
    return;
  }

  if ((nowMs - state.lastSensorReadMs) < config::timing::SENSOR_INTERVAL_MS) {
    return;
  }

  state.lastSensorReadMs = nowMs;
  readSensors(state);
  updateAtSetpointTimer(state);
}

float computeStdDev(const TankLocalState& tank) {
  if (tank.historyCount < 2) return NAN;

  float mean = 0.0f;
  for (uint8_t index = 0; index < tank.historyCount; ++index) {
    mean += tank.history[index];
  }
  mean /= static_cast<float>(tank.historyCount);

  float sumSq = 0.0f;
  for (uint8_t index = 0; index < tank.historyCount; ++index) {
    const float delta = tank.history[index] - mean;
    sumSq += delta * delta;
  }

  return sqrtf(sumSq / static_cast<float>(tank.historyCount - 1));
}

int8_t getTrendCode(const TankLocalState& tank) {
  if (tank.historyCount < config::ui::HISTORY_LEN) return 0;

  const uint8_t oldestIndex = tank.historyIndex;
  const uint8_t newestIndex = (tank.historyIndex + config::ui::HISTORY_LEN - 1) % config::ui::HISTORY_LEN;
  const float delta = tank.history[newestIndex] - tank.history[oldestIndex];

  if (delta > config::ui::TREND_THRESHOLD_C) return 1;
  if (delta < -config::ui::TREND_THRESHOLD_C) return -1;
  return 0;
}

const char* getTrendText(const TankLocalState& tank) {
  const int8_t trend = getTrendCode(tank);
  if (trend > 0) return "UP";
  if (trend < 0) return "DN";
  return "FLAT";
}

uint16_t getTrendColor(const TankLocalState& tank) {
  const int8_t trend = getTrendCode(tank);
  if (trend > 0) return config::color::COLOR_OK;
  if (trend < 0) return config::color::COLOR_ACCENT;
  return config::color::COLOR_TEXT;
}

int8_t getStatusCode(const TankLocalState& tank) {
  if (!tank.valid || isnan(tank.rawTempC)) return 2;
  if (tank.filteredTempC >= config::temperature::HARD_MAX_TEMP_C) return 2;
  if (tank.filteredTempC >= config::temperature::WARN_TEMP_C) return 1;
  return 0;
}

const char* getStatusText(const TankLocalState& tank) {
  const int8_t status = getStatusCode(tank);
  if (status == 2) return "ERR";
  if (status == 1) return "WRN";
  return "OK";
}

uint16_t getStatusColor(const TankLocalState& tank) {
  const int8_t status = getStatusCode(tank);
  if (status == 2) return config::color::COLOR_FAULT;
  if (status == 1) return config::color::COLOR_WARN;
  return config::color::COLOR_OK;
}

}  // namespace sensors
