// Legacy remote sketch reference.
// Active repo architecture is documented in firmware/ and docs/, and the remote must match firmware/shared/system_packets.h.

#include <SPI.h>
#include "config.h"
#include "types.h"
#include "buttons.h"
#include "sensors.h"
#include "radio.h"
#include "serial_log.h"
#include "display.h"

DashboardState state;

void setup() {
  Serial.begin(115200);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  pinMode(RADIO_CSN_PIN, OUTPUT);
  digitalWrite(RADIO_CSN_PIN, HIGH);

  SPI.begin();

  buttons::init(state);
  sensors::init(state);

  state.main.setpoint = 37.0f;
  state.res.setpoint  = 37.0f;
  state.main.rawTemp = NAN;
  state.res.rawTemp = NAN;
  state.main.filteredTemp = 37.0f;
  state.res.filteredTemp  = 37.0f;
  state.bootMs = millis();
  state.lastButtonEvent = "NONE";
  state.lastButtonEventAt = 0;
  state.selectedTank = 0;
  state.setMode = false;
  state.timeAtSetpointSec = 0;
  state.atSetpoint = false;

  display::init(state);
  radio::init(state);
  serial_log::init(state);

  state.displayDirty = true;
  Serial.println("Remote TFT dashboard ready");
}

void loop() {
  buttons::update(state);
  sensors::update(state);
  radio::update(state);
  serial_log::update(state);
  display::update(state);
}
