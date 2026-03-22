#include "radio.h"
#include "config.h"
#include "sensors.h"
#include "radio_payload.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ---------------- File-scope statics ----------------
static RF24 radioHw(RADIO_CE_PIN, RADIO_CSN_PIN);

static const byte RADIO_ADDRESS[6] = "00001";

static RadioPayload txPayload;
static uint8_t txCounter = 0;
static unsigned long lastRadioSend = 0;

// ---------------- Public API ----------------
namespace radio {

void init(DashboardState& state) {
  deselectTFT();
  state.radioInitOk = radioHw.begin();
  if (state.radioInitOk) {
    radioHw.setPALevel(RF24_PA_LOW);
    radioHw.setDataRate(RF24_250KBPS);
    radioHw.setChannel(108);
    radioHw.setAutoAck(false);
    radioHw.openWritingPipe(RADIO_ADDRESS);
    radioHw.stopListening();
  }
}

void update(DashboardState& state) {
  unsigned long now = millis();
  if (!state.radioInitOk || !sensors::isReady()) return;
  if (now - lastRadioSend < RADIO_INTERVAL) return;

  lastRadioSend = now;

  txPayload.main.rawTempC = state.main.rawTemp;
  txPayload.main.setpointC = state.main.setpoint;
  txPayload.res.rawTempC = state.res.rawTemp;
  txPayload.res.setpointC = state.res.setpoint;
  txPayload.validMask = 0;
  if (isValidTemp(state.main.rawTemp)) txPayload.validMask |= RADIO_VALID_MAIN;
  if (isValidTemp(state.res.rawTemp))  txPayload.validMask |= RADIO_VALID_RES;
  txPayload.counter = txCounter++;

  deselectTFT();
  bool ok = radioHw.writeFast(&txPayload, sizeof(txPayload));
  radioHw.txStandBy();
  state.lastTxOk = ok;
  state.displayDirty = true;
}

} // namespace radio
