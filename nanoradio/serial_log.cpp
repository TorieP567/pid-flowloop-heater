#include "serial_log.h"

#include "config.h"
#include "radio.h"

namespace {

void logEvent(const __FlashStringHelper* text) {
#if NANORADIO_ENABLE_SERIAL_LOG
  Serial.println(text);
#else
  (void)text;
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

  Serial.print(F("RAW:"));
  Serial.print(state.localTanks[config::TANK_MAIN].rawTempC, 2);
  Serial.print('/');
  Serial.print(state.localTanks[config::TANK_RES].rawTempC, 2);
  Serial.print(F(",REQ:"));
  Serial.print(state.localTanks[config::TANK_MAIN].requestedSetpointC, 1);
  Serial.print('/');
  Serial.print(state.localTanks[config::TANK_RES].requestedSetpointC, 1);
  Serial.print(F(",TX:"));
  Serial.print(state.lastOutboundPacket.sequence);
  Serial.print(',');
  Serial.print(state.lastTxOk ? F("OK") : F("FAIL"));
  Serial.print(F(",RX:"));
  Serial.print(state.haveMainStatus ? state.latestMainStatus.statusSequence : 0U);
  Serial.print(F(",AGE:"));
  Serial.print(state.haveMainStatus ? (nowMs - state.lastStatusRxMs) : 65535UL);
  Serial.print(F(",LINK:"));
  Serial.print(radio::linkStateText(radio::currentLinkState(state, nowMs)));
  Serial.print(F(",SEL:"));
  Serial.print(state.selectedTank == config::TANK_MAIN ? "MAIN" : "RES");
  Serial.print(F(",EDIT:"));
  Serial.print(state.editMode ? "ON" : "OFF");
  Serial.print(F(",SCR:"));
  Serial.print(state.screenMode == SCREEN_MODE_MAIN ? "MAIN" : "DEBUG");
  Serial.print(F(",FLT:0x"));
  Serial.print(state.haveMainStatus ? state.latestMainStatus.faultFlags : 0U, HEX);
  Serial.print(F(",OUT:"));
  Serial.print(state.haveMainStatus ? decodeOutputPermille(state.latestMainStatus.mainOutputPermille) : 0.0f, 1);
  Serial.print('/');
  Serial.print(state.haveMainStatus ? decodeOutputPermille(state.latestMainStatus.resOutputPermille) : 0.0f, 1);
  Serial.println();
#else
  (void)state;
#endif
}

}  // namespace

namespace serial_log {

void init(DashboardState& state) {
  state.lastLoggedLinkState = radio::currentLinkState(state, millis());
  logEvent(F("Remote box ready"));
}

void update(DashboardState& state) {
  logLinkTransitions(state);
  logSummary(state);
}

}  // namespace serial_log
