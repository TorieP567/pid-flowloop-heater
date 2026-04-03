#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

#include "config.h"
#include "system_packets.h"

// Screen pages rendered by the remote UI.
enum ScreenMode : uint8_t {
  SCREEN_MODE_MAIN = 0,
  SCREEN_MODE_DEBUG = 1
};

// Health summary for the remote -> main -> remote status path.
enum LinkState : uint8_t {
  LINK_STATE_NO_RADIO = 0,
  LINK_STATE_WAITING = 1,
  LINK_STATE_OK = 2,
  LINK_STATE_DEGRADED = 3,
  LINK_STATE_TIMEOUT = 4
};

// Debounce and press-duration tracking for one INPUT_PULLUP button.
struct ButtonTracker {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
  unsigned long pressedAtMs;
  bool longHandled;
};

// Local tank data gathered on the remote before the main box acknowledges it.
struct TankLocalState {
  float rawTempC;
  bool valid;
  float requestedSetpointC;
};

// Cross-module runtime state for the production remote implementation.
struct DashboardState {
  TankLocalState localTanks[config::TANK_COUNT];
  // Local thermocouple readings plus the operator-requested setpoint for MAIN
  // and RES.

  ButtonTracker buttons[3];
  // Debounce state for UP, SET, and DOWN in that order.

  uint8_t selectedTank;
  // UI-selected tank, using config::TANK_MAIN or config::TANK_RES.

  bool editMode;
  // True while UP/DOWN should change the selected tank setpoint.

  ScreenMode screenMode;
  // Which screen the display module should render.

  bool displayNeedsFullRedraw;
  // Requests a full static redraw, used when switching screens or recovering
  // from boot.

  bool radioInitOk;
  // Result of RF24 initialization.

  bool lastTxOk;
  // Result of the most recent uplink transmit attempt.

  bool haveMainStatus;
  // True once a valid MainToRemotePacket has been received.

  bool sensorsReady;
  // True after the MAX6675 startup delay has elapsed.

  bool screenToggleComboHandled;
  // Latches the UP+DOWN combo so the debug screen toggles only once per hold.

  unsigned long screenToggleComboStartMs;
  // Timestamp when UP+DOWN began being held together.

  RemoteToMainPacket lastOutboundPacket;
  // Last packet transmitted toward the main box.

  MainToRemotePacket latestMainStatus;
  // Most recent validated status packet returned by the main box.

  uint16_t txSequence;
  // Monotonic uplink sequence number.

  uint16_t lastStatusSequence;
  // Last accepted downlink status sequence number.

  uint8_t pendingButtonFlags;
  // Button-event bits to include in the next uplink packet.

  unsigned long bootMs;
  // millis() snapshot captured during setup.

  unsigned long sensorsReadyAtMs;
  // Time when MAX6675 readings can be trusted after startup.

  unsigned long lastSensorReadMs;
  // Last local thermocouple sampling time.

  unsigned long lastRadioTxMs;
  // Last uplink transmit attempt time.

  unsigned long lastDisplayMs;
  // Last display refresh time.

  unsigned long lastLogMs;
  // Last serial summary emission time.

  unsigned long lastStatusRxMs;
  // Time when the latest valid downlink packet arrived.

  unsigned long txOkCount;
  // Count of successful uplink writes.

  unsigned long txFailCount;
  // Count of failed uplink writes.

  unsigned long rxOkCount;
  // Count of validated status packets received from the main box.

  unsigned long rxBadCount;
  // Count of received status packets rejected by version or CRC.

  LinkState lastLoggedLinkState;
  // Most recent link state already reported on the serial console.
};

#endif  // TYPES_H
