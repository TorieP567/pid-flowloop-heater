#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <math.h>

#include "system_packets.h"

#ifndef NANORADIO_ENABLE_SERIAL_LOG
#define NANORADIO_ENABLE_SERIAL_LOG 0
#endif

namespace config {

// -----------------------------------------------------------------------------
// Pin map
// -----------------------------------------------------------------------------
namespace pins {
constexpr uint8_t RADIO_CE_PIN = 9;
constexpr uint8_t RADIO_CSN_PIN = 10;
constexpr uint8_t RADIO_CHANNEL = SYSTEM_RADIO_CHANNEL;

constexpr uint8_t THERMO_SCK_PIN = 5;
constexpr uint8_t THERMO_SO_MAIN_PIN = 7;
constexpr uint8_t THERMO_SO_RES_PIN = A0;
constexpr uint8_t THERMO_CS_MAIN_PIN = 6;
constexpr uint8_t THERMO_CS_RES_PIN = 8;

constexpr uint8_t TFT_CS_PIN = A1;
constexpr uint8_t TFT_DC_PIN = A2;
constexpr uint8_t TFT_RST_PIN = A3;

constexpr uint8_t BTN_UP_PIN = 2;
constexpr uint8_t BTN_SET_PIN = 3;
constexpr uint8_t BTN_DOWN_PIN = 4;
}  // namespace pins

// -----------------------------------------------------------------------------
// Tank identifiers
// -----------------------------------------------------------------------------
constexpr uint8_t TANK_COUNT = 2;
constexpr uint8_t TANK_MAIN = 0;
constexpr uint8_t TANK_RES = 1;

// -----------------------------------------------------------------------------
// Setpoint defaults and clamp limits
// -----------------------------------------------------------------------------
namespace setpoint {
constexpr float DEFAULT_SETPOINT_C = 37.0f;
constexpr float MIN_SETPOINT_C = 20.0f;
constexpr float MAX_SETPOINT_ALLOWED_C = 39.0f;
constexpr float SETPOINT_STEP_C = 0.1f;
}  // namespace setpoint

// -----------------------------------------------------------------------------
// Temperature thresholds
// -----------------------------------------------------------------------------
namespace temperature {
constexpr float MIN_REASONABLE_TEMP_C = -20.0f;
constexpr float MAX_REASONABLE_TEMP_C = 120.0f;
constexpr float WARN_TEMP_C = 39.0f;
constexpr float HARD_MAX_TEMP_C = 41.0f;
}  // namespace temperature

// -----------------------------------------------------------------------------
// Runtime timing intervals
// -----------------------------------------------------------------------------
namespace timing {
constexpr unsigned long MAX6675_STARTUP_MS = 500UL;
constexpr unsigned long SENSOR_INTERVAL_MS = 250UL;
constexpr unsigned long RADIO_TX_INTERVAL_MS = 150UL;
constexpr unsigned long DISPLAY_INTERVAL_MS = 100UL;
constexpr unsigned long LOG_INTERVAL_MS = 500UL;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30UL;
constexpr unsigned long SET_LONG_PRESS_MS = 700UL;
constexpr unsigned long SCREEN_TOGGLE_HOLD_MS = 900UL;
constexpr unsigned long STATUS_DEGRADED_MS = 900UL;
constexpr unsigned long STATUS_TIMEOUT_MS = 2000UL;
}  // namespace timing

// -----------------------------------------------------------------------------
// Display colors
// -----------------------------------------------------------------------------
namespace color {
constexpr uint16_t COLOR_BG = 0x0000;       // black
constexpr uint16_t COLOR_PANEL = 0x1084;
constexpr uint16_t COLOR_PANEL_ALT = 0x18C6;
constexpr uint16_t COLOR_DIM = 0x8410;
constexpr uint16_t COLOR_ACCENT = 0x07FF;   // cyan
constexpr uint16_t COLOR_OK = 0x07E0;       // green
constexpr uint16_t COLOR_WARN = 0xFFE0;     // yellow
constexpr uint16_t COLOR_FAULT = 0xF800;    // red
constexpr uint16_t COLOR_LOCAL = 0x7D7C;
constexpr uint16_t COLOR_TEXT = 0xFFFF;     // white
}  // namespace color

// -----------------------------------------------------------------------------
// Display layout
// -----------------------------------------------------------------------------
namespace layout {
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;

constexpr int HEADER_X = 0;
constexpr int HEADER_Y = 0;
constexpr int HEADER_W = SCREEN_W;
constexpr int HEADER_H = 30;

constexpr int CARD_W = 148;
constexpr int CARD_H = 138;
constexpr int CARD_Y = 34;
constexpr int MAIN_X = 8;
constexpr int RES_X = 164;

constexpr int FOOT_Y = 180;
constexpr int FOOT_H = 60;
}  // namespace layout

// -----------------------------------------------------------------------------
// Shared helpers
// -----------------------------------------------------------------------------
inline float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

inline bool isReasonableTemp(float tempC) {
  return !isnan(tempC) &&
         tempC > temperature::MIN_REASONABLE_TEMP_C &&
         tempC < temperature::MAX_REASONABLE_TEMP_C;
}

inline void initSpiChipSelects() {
  pinMode(pins::TFT_CS_PIN, OUTPUT);
  pinMode(pins::RADIO_CSN_PIN, OUTPUT);
  pinMode(pins::THERMO_CS_MAIN_PIN, OUTPUT);
  pinMode(pins::THERMO_CS_RES_PIN, OUTPUT);

  digitalWrite(pins::TFT_CS_PIN, HIGH);
  digitalWrite(pins::RADIO_CSN_PIN, HIGH);
  digitalWrite(pins::THERMO_CS_MAIN_PIN, HIGH);
  digitalWrite(pins::THERMO_CS_RES_PIN, HIGH);
}

inline void prepareForRadio() {
  digitalWrite(pins::TFT_CS_PIN, HIGH);
  digitalWrite(pins::THERMO_CS_MAIN_PIN, HIGH);
  digitalWrite(pins::THERMO_CS_RES_PIN, HIGH);
}

inline void prepareForTft() {
  digitalWrite(pins::RADIO_CSN_PIN, HIGH);
  digitalWrite(pins::THERMO_CS_MAIN_PIN, HIGH);
  digitalWrite(pins::THERMO_CS_RES_PIN, HIGH);
}

inline void prepareForSensors() {
  digitalWrite(pins::RADIO_CSN_PIN, HIGH);
  digitalWrite(pins::TFT_CS_PIN, HIGH);
}

}  // namespace config

#endif  // CONFIG_H
