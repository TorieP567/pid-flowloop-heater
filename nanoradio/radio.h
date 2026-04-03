#ifndef RADIO_H
#define RADIO_H

#include "types.h"

// Bidirectional nRF24 transport for the remote box.
namespace radio {

// Initializes the RF24 device and enters listening mode for downlink status.
void init(DashboardState& state);

// Polls for MainToRemotePacket updates and periodically transmits the current
// RemoteToMainPacket uplink.
void update(DashboardState& state);

// Returns true when the latest validated downlink packet is still inside the
// timeout window.
bool statusPacketFresh(const DashboardState& state, unsigned long nowMs);

// Returns true when the latest validated downlink packet is still inside the
// healthy, non-degraded window.
bool statusPacketHealthy(const DashboardState& state, unsigned long nowMs);

// Computes the user-facing link state from radio init status and downlink age.
LinkState currentLinkState(const DashboardState& state, unsigned long nowMs);

// Human-readable label for the current link state.
const char* linkStateText(LinkState state);

// UI color associated with the current link state.
uint16_t linkStateColor(LinkState state);

// Reads the authoritative setpoint returned by the main box for the requested
// tank and reports whether the field is currently valid.
float activeSetpointFromMain(const DashboardState& state, uint8_t tank, bool& valid);

// Reads the authoritative output percentage returned by the main box for the
// requested tank and reports whether the field is currently valid.
float outputPctFromMain(const DashboardState& state, uint8_t tank, bool& valid);

// Returns whether the main box reports the heater output as ON for the tank.
bool heaterOnFromMain(const DashboardState& state, uint8_t tank);

}  // namespace radio

#endif  // RADIO_H
