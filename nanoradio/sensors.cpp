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
}

}  // namespace

namespace sensors {

void init(DashboardState& state) {
  state.sensorsReady = false;
  state.sensorsReadyAtMs = millis() + config::timing::MAX6675_STARTUP_MS;
  state.lastSensorReadMs = 0;
}

void update(DashboardState& state) {
  const unsigned long nowMs = millis();

  if (!state.sensorsReady) {
    if (nowMs >= state.sensorsReadyAtMs) {
      state.sensorsReady = true;
    } else {
      return;
    }
  }

  if ((nowMs - state.lastSensorReadMs) < config::timing::SENSOR_INTERVAL_MS) {
    return;
  }

  state.lastSensorReadMs = nowMs;
  readSensors(state);
}

}  // namespace sensors
