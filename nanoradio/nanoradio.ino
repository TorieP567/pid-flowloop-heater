// =============================================================================
// REMOTE BOX
// Arduino Nano handheld UI / sensor / radio node for the dual-tank heater.
//
// Responsibilities:
//   - read two MAX6675 thermocouples
//   - manage three buttons with responsive short/long press handling
//   - render the ST7789 TFT UI
//   - send temperatures + requested setpoints to the main box over nRF24L01
//   - receive authoritative controller status back from the main box
//
// Important:
//   - this board is NOT the final heater-control authority
//   - the main-box Uno R4 remains responsible for both SSR heater outputs
//   - the packet contract mirrors firmware/shared/system_packets.h
//   - place a 10 uF capacitor across nRF24 VCC/GND close to the radio module
//
// Hardware notes:
//   - the nRF24, ST7789, and MAX6675 modules share SPI-related lines
//   - always keep unselected chip-select lines HIGH
//   - initialize chip-select pins before starting SPI traffic
//
// Pin map:
//   Radio:     CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13
//   MAX6675 #1 MAIN: SCK=D5, SO=D7, CS=D6
//   MAX6675 #2 RES:  SCK=D5, SO=A0, CS=D8
//   ST7789:    CS=A1, DC=A2, RST=A3, MOSI=D11, SCK=D13
//   Buttons:   UP=D2, SET=D3, DOWN=D4, all wired to GND with INPUT_PULLUP
// =============================================================================

#include <math.h>
#include <SPI.h>

#include "config.h"
#include "types.h"
#include "buttons.h"
#include "sensors.h"
#include "radio.h"
#include "serial_log.h"
#include "display.h"

DashboardState state = {};

namespace {

void initializeState(DashboardState& dashboard) {
  for (uint8_t tank = 0; tank < config::TANK_COUNT; ++tank) {
    dashboard.localTanks[tank].rawTempC = NAN;
    dashboard.localTanks[tank].valid = false;
    dashboard.localTanks[tank].requestedSetpointC = config::setpoint::DEFAULT_SETPOINT_C;
  }

  for (uint8_t index = 0; index < 3; ++index) {
    dashboard.buttons[index].pin = 0;
    dashboard.buttons[index].stableState = HIGH;
    dashboard.buttons[index].lastReading = HIGH;
    dashboard.buttons[index].lastChangeMs = 0;
    dashboard.buttons[index].pressedAtMs = 0;
    dashboard.buttons[index].longHandled = false;
  }

  dashboard.selectedTank = config::TANK_MAIN;
  dashboard.editMode = false;
  dashboard.screenMode = SCREEN_MODE_MAIN;
  dashboard.displayNeedsFullRedraw = true;

  dashboard.radioInitOk = false;
  dashboard.lastTxOk = false;
  dashboard.haveMainStatus = false;
  dashboard.sensorsReady = false;
  dashboard.screenToggleComboHandled = false;
  dashboard.screenToggleComboStartMs = 0;

  dashboard.lastOutboundPacket = RemoteToMainPacket();
  dashboard.latestMainStatus = MainToRemotePacket();
  dashboard.txSequence = 0;
  dashboard.lastStatusSequence = 0;
  dashboard.pendingButtonFlags = 0;

  dashboard.bootMs = 0;
  dashboard.sensorsReadyAtMs = 0;
  dashboard.lastSensorReadMs = 0;
  dashboard.lastRadioTxMs = 0;
  dashboard.lastDisplayMs = 0;
  dashboard.lastLogMs = 0;
  dashboard.lastStatusRxMs = 0;

  dashboard.txOkCount = 0;
  dashboard.txFailCount = 0;
  dashboard.rxOkCount = 0;
  dashboard.rxBadCount = 0;
  dashboard.lastLoggedLinkState = LINK_STATE_WAITING;
}

}  // namespace

void setup() {
#if NANORADIO_ENABLE_SERIAL_LOG
  Serial.begin(115200);
#endif

  initializeState(state);

  config::initSpiChipSelects();
  SPI.begin();

  state.bootMs = millis();

  buttons::init(state);
  sensors::init(state);
  display::init(state);
  radio::init(state);
  serial_log::init(state);
}

void loop() {
  buttons::update(state);
  sensors::update(state);
  radio::update(state);
  serial_log::update(state);
  display::update(state);
}
