// nanoradio/sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include "types.h"

namespace sensors {
  void init(DashboardState& state);
  void update(DashboardState& state);
  bool isReady();

  // Query functions used by display module
  float computeStdDev(const TankState& t);
  int getTrendCode(const TankState& t);
  const char* getTrendText(const TankState& t);
  uint16_t getTrendColor(const TankState& t);
  int getStatusCode(const TankState& t);
  const char* getStatusText(const TankState& t);
  uint16_t getStatusColor(const TankState& t);
}

#endif // SENSORS_H
