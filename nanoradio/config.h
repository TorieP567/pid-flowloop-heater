// nanoradio/config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ---------------- LCD ----------------
#define TFT_CS   A1
#define TFT_DC   A2
#define TFT_RST  A3

// ---------------- Radio ----------------
#define RADIO_CE_PIN   9
#define RADIO_CSN_PIN  10

// ---------------- MAX6675 ----------------
const int THERMO_SCK      = 5;
const int THERMO_CS_MAIN  = 6;
const int THERMO_SO_MAIN  = 7;
const int THERMO_CS_RES   = 8;
const int THERMO_SO_RES   = A0;

// ---------------- Buttons ----------------
const int BTN_UP_PIN   = 2;
const int BTN_SET_PIN  = 3;
const int BTN_DOWN_PIN = 4;

// ---------------- Constants ----------------
const float EMA_ALPHA               = 0.30f;
const float MIN_SETPOINT_C          = 20.0f;
const float MAX_SETPOINT_ALLOWED_C  = 39.0f;
const float BUTTON_SP_STEP_C        = 0.1f;

const float WARN_TEMP_C             = 39.0f;
const float MAX_TEMP_C              = 41.0f;
const float AT_SETPOINT_BAND_C      = 1.0f;
const float TREND_THRESHOLD_C       = 0.20f;

const unsigned long MAX6675_STARTUP_MS  = 500UL;
const unsigned long SENSOR_INTERVAL     = 300UL;
const unsigned long RADIO_INTERVAL      = 300UL;
const unsigned long SERIAL_INTERVAL     = 500UL;
const unsigned long BUTTON_DEBOUNCE_MS  = 30UL;
const unsigned long DOUBLE_CLICK_MS     = 350UL;
const unsigned long BTN_MSG_HOLD_MS     = 1200UL;

const int HISTORY_LEN = 8;

// ---------------- Layout ----------------
const int SCREEN_W = 320;
const int SCREEN_H = 240;

const int HEADER_X = 4;
const int HEADER_Y = 4;
const int HEADER_W = 312;
const int HEADER_H = 24;

const int CARD_W   = 150;
const int CARD_H   = 146;
const int CARD_Y   = 34;
const int MAIN_X   = 6;
const int RES_X    = 164;

const int FOOT_X   = 4;
const int FOOT_Y   = 186;
const int FOOT_W   = 312;
const int FOOT_H   = 48;

// ---------------- SPI Arbitration ----------------
inline void deselectRadio() { digitalWrite(RADIO_CSN_PIN, HIGH); }
inline void deselectTFT()   { digitalWrite(TFT_CS, HIGH); }

// ---------------- Shared Helpers ----------------
inline float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

inline bool isValidTemp(float t) {
  return (!isnan(t) && t > -50.0f && t < 500.0f);
}

inline void formatTimeHHMMSS(unsigned long totalSec, char* buf) {
  if (totalSec > 359999UL) totalSec = 359999UL;
  uint8_t h = totalSec / 3600UL;
  uint8_t m = (totalSec % 3600UL) / 60UL;
  uint8_t s = totalSec % 60UL;
  buf[0] = '0' + h / 10; buf[1] = '0' + h % 10; buf[2] = ':';
  buf[3] = '0' + m / 10; buf[4] = '0' + m % 10; buf[5] = ':';
  buf[6] = '0' + s / 10; buf[7] = '0' + s % 10; buf[8] = '\0';
}

#endif // CONFIG_H
