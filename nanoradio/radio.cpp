#include "radio.h"

#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "config.h"

namespace {

RF24 radioHw(config::pins::RADIO_CE_PIN, config::pins::RADIO_CSN_PIN);

void pollRadioDownlink(DashboardState& state) {
  if (!state.radioInitOk) return;

  MainToRemotePacket packet = {};
  bool sawPacket = false;

  config::prepareForRadio();
  while (radioHw.available()) {
    radioHw.read(&packet, sizeof(packet));
    sawPacket = true;
  }

  if (!sawPacket) return;

  if (!validatePacket(packet)) {
    ++state.rxBadCount;
    return;
  }

  state.latestMainStatus = packet;
  state.haveMainStatus = true;
  state.lastStatusRxMs = millis();
  state.lastStatusSequence = packet.statusSequence;
  ++state.rxOkCount;
}

void sendRadioUplink(DashboardState& state) {
  const unsigned long nowMs = millis();
  if (!state.radioInitOk) return;
  if ((nowMs - state.lastRadioTxMs) < config::timing::RADIO_TX_INTERVAL_MS) return;

  state.lastRadioTxMs = nowMs;

  RemoteToMainPacket packet = {};
  packet.sequence = ++state.txSequence;
  packet.remoteMillis = nowMs;
  packet.mainTempCx100 = encodeTempCx100(state.localTanks[config::TANK_MAIN].rawTempC);
  packet.resTempCx100 = encodeTempCx100(state.localTanks[config::TANK_RES].rawTempC);
  packet.mainSetpointCx100 = encodeTempCx100(state.localTanks[config::TANK_MAIN].requestedSetpointC);
  packet.resSetpointCx100 = encodeTempCx100(state.localTanks[config::TANK_RES].requestedSetpointC);
  packet.validFlags = 0;
  if (state.localTanks[config::TANK_MAIN].valid) packet.validFlags |= REMOTE_VALID_MAIN_TEMP;
  if (state.localTanks[config::TANK_RES].valid) packet.validFlags |= REMOTE_VALID_RES_TEMP;
  packet.uiFlags = state.editMode ? REMOTE_UI_SET_MODE : 0U;
  if (state.selectedTank == config::TANK_RES) packet.uiFlags |= REMOTE_UI_SELECTED_TANK_RES;
  packet.buttonFlags = state.pendingButtonFlags;
  finalizePacket(packet);

  state.lastOutboundPacket = packet;

  config::prepareForRadio();
  radioHw.stopListening();
  const bool ok = radioHw.write(&packet, sizeof(packet));
  radioHw.startListening();

  state.lastTxOk = ok;
  if (ok) {
    ++state.txOkCount;
  } else {
    ++state.txFailCount;
  }

  state.pendingButtonFlags = 0;
}

}  // namespace

namespace radio {

void init(DashboardState& state) {
  config::prepareForRadio();
  state.radioInitOk = radioHw.begin();
  state.lastTxOk = false;

  if (!state.radioInitOk) {
    return;
  }

  radioHw.setPALevel(RF24_PA_LOW);
  radioHw.setDataRate(RF24_250KBPS);
  radioHw.setChannel(config::pins::RADIO_CHANNEL);
  radioHw.setAutoAck(false);
  radioHw.openWritingPipe(SYSTEM_PIPE_REMOTE_TO_MAIN);
  radioHw.openReadingPipe(1, SYSTEM_PIPE_MAIN_TO_REMOTE);
  radioHw.startListening();
}

void update(DashboardState& state) {
  pollRadioDownlink(state);
  sendRadioUplink(state);
}

bool statusPacketFresh(const DashboardState& state, unsigned long nowMs) {
  return state.haveMainStatus &&
         ((nowMs - state.lastStatusRxMs) <= config::timing::STATUS_TIMEOUT_MS);
}

bool statusPacketHealthy(const DashboardState& state, unsigned long nowMs) {
  return state.haveMainStatus &&
         ((nowMs - state.lastStatusRxMs) <= config::timing::STATUS_DEGRADED_MS);
}

LinkState currentLinkState(const DashboardState& state, unsigned long nowMs) {
  if (!state.radioInitOk) return LINK_STATE_NO_RADIO;
  if (!state.haveMainStatus) return LINK_STATE_WAITING;

  const unsigned long ageMs = nowMs - state.lastStatusRxMs;
  if (ageMs <= config::timing::STATUS_DEGRADED_MS) return LINK_STATE_OK;
  if (ageMs <= config::timing::STATUS_TIMEOUT_MS) return LINK_STATE_DEGRADED;
  return LINK_STATE_TIMEOUT;
}

const char* linkStateText(LinkState state) {
  switch (state) {
    case LINK_STATE_NO_RADIO: return "NO RADIO";
    case LINK_STATE_WAITING: return "WAITING";
    case LINK_STATE_OK: return "LINK OK";
    case LINK_STATE_DEGRADED: return "DEGRADED";
    case LINK_STATE_TIMEOUT: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}

uint16_t linkStateColor(LinkState state) {
  switch (state) {
    case LINK_STATE_OK: return config::color::COLOR_OK;
    case LINK_STATE_DEGRADED: return config::color::COLOR_WARN;
    case LINK_STATE_WAITING: return config::color::COLOR_WARN;
    case LINK_STATE_NO_RADIO:
    case LINK_STATE_TIMEOUT:
    default: return config::color::COLOR_FAULT;
  }
}

float activeSetpointFromMain(const DashboardState& state, uint8_t tank, bool& valid) {
  if (!state.haveMainStatus) {
    valid = false;
    return NAN;
  }

  valid = true;
  return decodeTempCx100(
      tank == config::TANK_MAIN ? state.latestMainStatus.mainSetpointCx100
                                : state.latestMainStatus.resSetpointCx100);
}

float outputPctFromMain(const DashboardState& state, uint8_t tank, bool& valid) {
  if (!state.haveMainStatus) {
    valid = false;
    return NAN;
  }

  valid = true;
  return decodeOutputPermille(
      tank == config::TANK_MAIN ? state.latestMainStatus.mainOutputPermille
                                : state.latestMainStatus.resOutputPermille);
}

bool heaterOnFromMain(const DashboardState& state, uint8_t tank) {
  if (!state.haveMainStatus) return false;

  const uint8_t mask = (tank == config::TANK_MAIN) ? HEATER_FLAG_MAIN_ON : HEATER_FLAG_RES_ON;
  return (state.latestMainStatus.heaterFlags & mask) != 0U;
}

}  // namespace radio
