#ifndef SENSORS_H
#define SENSORS_H

#include "types.h"

// Local MAX6675 sensor acquisition for the remote box.
namespace sensors {

// Starts the MAX6675 warm-up timer and resets local sensor readiness state.
void init(DashboardState& state);

// Samples both thermocouples on the configured interval and updates the local
// tank readings used by the radio and display modules.
void update(DashboardState& state);

}  // namespace sensors

#endif  // SENSORS_H
