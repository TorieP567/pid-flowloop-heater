# Nanoradio — Remote Temperature Dashboard

Arduino Nano-based remote temperature monitoring dashboard for a dual-tank hot water system. Displays real-time temperatures, setpoints, trends, and status on a TFT screen with wireless telemetry via nRF24L01+.

## Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| Microcontroller | Arduino Nano (ATmega328P) | — |
| Display | Adafruit ST7789 TFT (320x240) | Hardware SPI |
| Radio | nRF24L01+ | Hardware SPI (shared) |
| Thermocouples (x2) | MAX6675 + K-type | Bit-banged SPI |
| Buttons (x3) | Momentary push | Digital (INPUT_PULLUP) |

### Pin Map

```
TFT Display          Radio (nRF24L01+)     Thermocouples (MAX6675)
  CS  = A1             CE  = 9               SCK  = 5 (shared)
  DC  = A2             CSN = 10              Main CS = 6, SO = 7
  RST = A3             SPI = 11,12,13        Res  CS = 8, SO = A0

Buttons (INPUT_PULLUP)
  UP  = 2    SET = 3    DOWN = 4
```

The TFT and radio share the hardware SPI bus (pins 11-13). The thermocouples use bit-banged SPI on separate pins with a shared clock line.

## Architecture

The codebase is split into focused modules that communicate through a single shared `DashboardState` struct:

```
nanoradio/
├── nanoradio.ino       54 lines   Setup + loop wiring
├── config.h            92 lines   Pins, constants, layout, SPI helpers
├── types.h             44 lines   TankState, ButtonState, DashboardState
├── sensors.h/.cpp     203 lines   Thermocouple reads, filtering, statistics
├── buttons.h/.cpp     108 lines   Debounce, click detection, setpoint control
├── radio.h/.cpp        73 lines   nRF24 fire-and-forget transmit
├── serial_log.h/.cpp   57 lines   Non-blocking drip-feed serial output
└── display.h/.cpp     596 lines   Phase state machine + dirty-field renderer
```

### Module Pattern

Every module exposes `init()` and `update()` in a namespace:

```cpp
namespace modulename {
  void init(DashboardState& state);   // called once from setup()
  void update(DashboardState& state); // called every loop()
}
```

Each module keeps its own timing variables and hardware instances as file-scope statics. Only shared observable state lives in `DashboardState`. No module calls another — all coordination flows through the shared state.

### Data Flow

```
loop() {
  buttons::update(state)     →  reads pins, mutates selectedTank/setMode/setpoints
  sensors::update(state)     →  reads thermocouples, filters, computes trends
  radio::update(state)       →  transmits payload every 300ms
  serial_log::update(state)  →  drip-feeds debug output to serial
  display::update(state)     →  advances one display phase, paints changed fields
}
```

### SPI Bus Arbitration

The TFT and radio share the SPI bus. Before any SPI operation, the active module deselects the other device:

```cpp
// config.h
inline void deselectRadio() { digitalWrite(RADIO_CSN_PIN, HIGH); }
inline void deselectTFT()   { digitalWrite(TFT_CS, HIGH); }
```

This works because `loop()` is single-threaded — no concurrent SPI access is possible.

## Non-Blocking Performance

The main loop targets 1-10 kHz in steady state. Every operation is designed to complete quickly and yield back to the loop:

| Operation | Strategy | Worst Case |
|-----------|----------|------------|
| **Display** | 7-phase state machine + dirty-field tracking | ~2-8ms per phase |
| **Radio** | `writeFast()` + `txStandBy()`, auto-ACK off | ~0.5ms |
| **Serial** | Buffer built once, drip-fed via `availableForWrite()` | ~0.01ms |
| **Sensors** | Bit-banged SPI read (inherently fast) | ~0.2ms |
| **Buttons** | Digital pin read + debounce | ~0.05ms |

### Display State Machine

The display is the most complex module. Instead of redrawing all ~25 fields at once (which would block for 30-80ms), updates are spread across 7 loop iterations:

```
IDLE → HEADER → TANK0_HEADER → TANK0_VALUES → TANK1_HEADER → TANK1_VALUES → FOOTER → IDLE
```

Each phase runs in one `display::update()` call and processes only 2-4 SPI operations. Combined with per-field dirty tracking (each field compares its current value to a cached previous value), a stable system redraws almost nothing — typically just the runtime counter once per second.

### Radio Fire-and-Forget

Auto-ACK is disabled so `writeFast()` loads the TX FIFO and returns immediately. `txStandBy()` flushes the FIFO. There are no retries or ACK waits. At 300ms intervals, a missed packet is immediately replaced by the next one.

### Serial Drip-Feed

The debug message is built into a 128-byte buffer using `dtostrf()` (since AVR `snprintf` doesn't support `%f`). Each loop iteration writes only as many bytes as the hardware TX buffer can accept (`Serial.availableForWrite()`), preventing any blocking.

## Sensor Pipeline

Raw thermocouple readings go through a multi-stage processing chain:

```
MAX6675 raw read (every 300ms)
  ↓
Validity check (not NaN, -50°C to 500°C range)
  ↓
EMA filter (α=0.30: 30% new reading, 70% previous)
  ↓
Circular history buffer (8 samples)
  ├── Trend detection (newest - oldest > ±0.20°C → UP/DN/FLAT)
  ├── Standard deviation (sample stddev over buffer)
  └── Status code (OK < 39°C, WRN ≥ 39°C, ERR ≥ 41°C)
  ↓
At-setpoint timer (both tanks within ±1°C of setpoint)
```

## Button Controls

| Button | Action | Effect |
|--------|--------|--------|
| **SET** (single click) | Toggle mode | Switches between VIEW and SET mode |
| **SET** (double click) | Switch tank | Toggles selected tank (MAIN ↔ RES) |
| **UP** | Adjust setpoint | +0.1°C (only in SET mode) |
| **DOWN** | Adjust setpoint | -0.1°C (only in SET mode) |

Double-click detection uses a 350ms window. Button events display in the footer for 1.2 seconds. All buttons use INPUT_PULLUP with 30ms software debounce.

## Display Layout

```
┌─────────────────── REMOTE DASH ───── RADIO ── RUN ──┐
│                                      TX OK   00:05:23│
├──────────────────┬──────────────────────────────────-─┤
│  ┌─ > MAIN ────┐ │ ┌──── RES ─────┐                  │
│  │   37.2      │ │ │   36.8       │                  │
│  │             │ │ │              │                  │
│  │ SP: 37.0    │ │ │ SP: 37.0    │                  │
│  │ dT: -0.2   │ │ │ dT: +0.2    │                  │
│  │ RAW: 37.1  │ │ │ RAW: 36.9   │                  │
│  │ SD: 0.15   │ │ │ SD: 0.08    │  TRD: FLAT       │
│  └─────────────┘ │ └─────────────┘  ST:  OK         │
├──────────────────┴───────────────────────────────────┤
│ BTN: NONE    SEL: MAIN  MODE: VIEW  ATSP: 00:02:15  │
│ MAIN: 37.2   RES: 36.8  BAND: YES                   │
└──────────────────────────────────────────────────────┘
```

- **Header**: Title, radio status (TX OK/FAIL/NO INIT), runtime counter
- **Tank cards**: Selected tank has yellow border and colored title bar (magenta in VIEW, cyan in SET mode)
- **Big temperature**: Displayed in status color (green=OK, yellow=WRN, red=ERR)
- **Footer**: Button events, selected tank, mode, at-setpoint timer

## Radio Payload

Transmitted every 300ms at 250kbps on channel 108:

```cpp
struct Payload {
  float mainTempC;        // Main tank filtered temperature
  float mainSetpointC;    // Main tank setpoint
  float resTempC;         // Reservoir filtered temperature
  float resSetpointC;     // Reservoir setpoint
  uint8_t selectedTank;   // 0=MAIN, 1=RES
  uint8_t setMode;        // 0=VIEW, 1=SET
  uint8_t counter;        // Rolling packet counter
};
```

Address: `"00001"` — must match receiver.

## Serial Output

At 115200 baud, every 500ms:

```
MAIN T=37.20 SP=37.00 | RES T=36.80 SP=37.00 | TX=OK
```

## Building

Requires the Arduino IDE or `arduino-cli` with these libraries:
- `Adafruit GFX Library`
- `Adafruit ST7735 and ST7789 Library`
- `RF24`
- `MAX6675`

```bash
arduino-cli compile --fqbn arduino:avr:nano nanoradio/
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano nanoradio/
```

## Key Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `EMA_ALPHA` | 0.30 | Filter responsiveness (higher = more responsive, noisier) |
| `SENSOR_INTERVAL` | 300ms | Thermocouple read rate |
| `RADIO_INTERVAL` | 300ms | Packet transmit rate |
| `SERIAL_INTERVAL` | 500ms | Debug log rate |
| `WARN_TEMP_C` | 39.0°C | Yellow warning threshold |
| `MAX_TEMP_C` | 41.0°C | Red error threshold |
| `AT_SETPOINT_BAND_C` | ±1.0°C | Band for "at setpoint" detection |
| `HISTORY_LEN` | 8 | Samples in trend/stddev buffer |
