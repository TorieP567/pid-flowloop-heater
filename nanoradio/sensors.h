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

// Local stability metric used by the original nanoradio dashboard.
float computeStdDev(const TankLocalState& tank);

// Local temperature trend summary used by the original nanoradio dashboard.
int8_t getTrendCode(const TankLocalState& tank);
const char* getTrendText(const TankLocalState& tank);
uint16_t getTrendColor(const TankLocalState& tank);

// Local temperature health summary used by the original nanoradio dashboard.
int8_t getStatusCode(const TankLocalState& tank);
const char* getStatusText(const TankLocalState& tank);
uint16_t getStatusColor(const TankLocalState& tank);

}  // namespace sensors

#endif  // SENSORS_H
