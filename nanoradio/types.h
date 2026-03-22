// nanoradio/types.h
#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include "config.h"

struct TankState {
  float rawTemp;
  float filteredTemp;
  float setpoint;
  float history[HISTORY_LEN];
  int historyIndex;
  int historyCount;
};

struct ButtonState {
  int pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
};

struct DashboardState {
  TankState main;
  TankState res;

  uint8_t selectedTank;       // 0=MAIN, 1=RES
  bool setMode;

  bool radioInitOk;
  bool lastTxOk;

  unsigned long bootMs;
  unsigned long timeAtSetpointSec;
  bool atSetpoint;

  const char* lastButtonEvent;
  unsigned long lastButtonEventAt;

  bool displayDirty;
};

#endif // TYPES_H
