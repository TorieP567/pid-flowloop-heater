#line 1 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\nanoradio\\serial_log.cpp"
#include "serial_log.h"

#include <math.h>

#include "config.h"
#include "radio.h"

namespace {

bool firstSensorReportDone = false;

void logEvent(const __FlashStringHelper* text) {
#if NANORADIO_ENABLE_SERIAL_LOG
  Serial.println(text);
#else
  (void)text;
#endif
}

void printMaybeFloat(float value, uint8_t decimals = 2) {
#if NANORADIO_ENABLE_SERIAL_LOG
  if (isnan(value)) {
    Serial.print(F("nan"));
  } else {
    Serial.print(value, decimals);
  }
#else
  (void)value;
  (void)decimals;
#endif
}

void logBootBanner(const DashboardState& state) {
#if NANORADIO_ENABLE_SERIAL_LOG
  Serial.println(F("==== REMOTE HW CHECK ===="));
  Serial.println(F("[BOOT] No buttons is OK; D2/D3/D4 use pullups."));
  Serial.println(F("[BOOT] TC MAIN D5/D6/D7"));
  Serial.println(F("[BOOT] TC RES  D5/D8/A0"));
  Serial.println(F("[BOOT] RF24 CE9 CSN10 SPI11/12/13"));
  Serial.println(F("[BOOT] TFT  A1/A2/A3 SPI11/13 BL5V"));
  Serial.println(F("[BOOT] TFT should flash RGB, then show REMOTE DASH."));

  if (state.radioInitOk) {
    Serial.println(F("[RADIO] RF24 SPI OK; waiting for main-box."));
  } else {
    Serial.println(F("[RADIO] RF24 FAIL. Check 3V3, CE9, CSN10, SPI, 10uF."));
  }

  Serial.println(F("[SENSORS] Wait 500 ms for MAX6675 startup..."));
#else
  (void)state;
#endif
}

void logSensorVerdict(const __FlashStringHelper* label,
                      const TankLocalState& tank,
                      const __FlashStringHelper* pinHint) {
#if NANORADIO_ENABLE_SERIAL_LOG
  Serial.print(label);
  if (tank.valid) {
    Serial.print(F(" OK "));
    printMaybeFloat(tank.rawTempC);
    Serial.println(F(" C"));
    return;
  }

  Serial.print(F(" FAIL sample="));
  printMaybeFloat(tank.lastSampleC);
  Serial.print(F(" C, check "));
  Serial.println(pinHint);

  if (isnan(tank.lastSampleC)) {
    Serial.println(F("          nan = open TC, bad chip, or bad power."));
  } else {
    Serial.println(F("          sample outside expected range."));
  }
#else
  (void)label;
  (void)tank;
  (void)pinHint;
#endif
}

void logFirstSensorReport(const DashboardState& state) {
#if NANORADIO_ENABLE_SERIAL_LOG
  if (firstSensorReportDone || !state.sensorsReady) return;
  firstSensorReportDone = true;

  Serial.println(F("[SENSORS] First MAX6675 readings:"));
  logSensorVerdict(F("  MAIN:"), state.localTanks[config::TANK_MAIN], F("D5/D6/D7, 5V, GND, thermocouple #1"));
  logSensorVerdict(F("  RES :"), state.localTanks[config::TANK_RES], F("D5/D8/A0, 5V, GND, thermocouple #2"));

  if (!state.localTanks[config::TANK_MAIN].valid &&
      !state.localTanks[config::TANK_RES].valid) {
    Serial.println(F("[SENSORS] BOTH fail -> check shared D5, 5V, GND."));
  } else if (!state.localTanks[config::TANK_MAIN].valid) {
    Serial.println(F("[SENSORS] MAIN fail -> check D6, D7, TC1."));
  } else if (!state.localTanks[config::TANK_RES].valid) {
    Serial.println(F("[SENSORS] RES fail -> check D8, A0, TC2."));
  }
#else
  (void)state;
#endif
}

void logLinkTransitions(DashboardState& state) {
#if NANORADIO_ENABLE_SERIAL_LOG
  const LinkState link = radio::currentLinkState(state, millis());
  if (link != state.lastLoggedLinkState) {
    state.lastLoggedLinkState = link;
    switch (link) {
      case LINK_STATE_OK: logEvent(F("Link restored")); break;
      case LINK_STATE_DEGRADED: logEvent(F("Link degraded")); break;
      case LINK_STATE_TIMEOUT: logEvent(F("Link timeout")); break;
      case LINK_STATE_WAITING: logEvent(F("Waiting for main-box status")); break;
      case LINK_STATE_NO_RADIO: logEvent(F("Radio hardware not initialized")); break;
      default: break;
    }
  }
#else
  (void)state;
#endif
}

void logSummary(DashboardState& state) {
#if NANORADIO_ENABLE_SERIAL_LOG
  const unsigned long nowMs = millis();
  if ((nowMs - state.lastLogMs) < config::timing::LOG_INTERVAL_MS) return;
  state.lastLogMs = nowMs;

  Serial.print(F("[H] M="));
  Serial.print(state.localTanks[config::TANK_MAIN].valid ? F("OK ") : F("FAIL "));
  printMaybeFloat(state.localTanks[config::TANK_MAIN].lastSampleC);
  Serial.print(F(" R="));
  Serial.print(state.localTanks[config::TANK_RES].valid ? F("OK ") : F("FAIL "));
  printMaybeFloat(state.localTanks[config::TANK_RES].lastSampleC);
  Serial.print(F(" RF="));
  if (!state.radioInitOk) {
    Serial.print(F("NOINIT"));
  } else {
    Serial.print(radio::linkStateText(radio::currentLinkState(state, nowMs)));
    Serial.print(F("/TX"));
    Serial.print(state.lastTxOk ? F("OK") : F("FAIL"));
  }
  Serial.print(F(" TFT TX="));
  Serial.print(state.lastOutboundPacket.sequence);
  Serial.print(F(" RX="));
  Serial.print(state.haveMainStatus ? state.latestMainStatus.statusSequence : 0U);
  Serial.print(F(" AGE="));
  Serial.print(state.haveMainStatus ? (nowMs - state.lastStatusRxMs) : 65535UL);
  Serial.println();
#else
  (void)state;
#endif
}

}  // namespace

namespace serial_log {

void init(DashboardState& state) {
  firstSensorReportDone = false;
  state.lastLoggedLinkState = radio::currentLinkState(state, millis());
  logBootBanner(state);
}

void update(DashboardState& state) {
  logFirstSensorReport(state);
  logLinkTransitions(state);
  logSummary(state);
}

}  // namespace serial_log
