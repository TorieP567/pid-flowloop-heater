#include "radio.h"
#include "config.h"
#include "sensors.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ---------------- File-scope statics ----------------
static RF24 radioHw(RADIO_CE_PIN, RADIO_CSN_PIN);

static const byte RADIO_ADDRESS[6] = "00001";

struct Payload {
  float mainTempC;
  float mainSetpointC;
  float resTempC;
  float resSetpointC;
  uint8_t selectedTank;
  uint8_t setMode;
  uint8_t counter;
};

static Payload txPayload;
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

  txPayload.mainTempC = state.main.filteredTemp;
  txPayload.mainSetpointC = state.main.setpoint;
  txPayload.resTempC = state.res.filteredTemp;
  txPayload.resSetpointC = state.res.setpoint;
  txPayload.selectedTank = (uint8_t)state.selectedTank;
  txPayload.setMode = state.setMode ? 1 : 0;
  txPayload.counter = txCounter++;

  deselectTFT();
  bool ok = radioHw.writeFast(&txPayload, sizeof(txPayload));
  radioHw.txStandBy();
  state.lastTxOk = ok;
  state.displayDirty = true;
}

} // namespace radio
