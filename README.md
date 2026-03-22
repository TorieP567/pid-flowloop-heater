# PID Flowloop Heater

This repository now contains both sides of the dual-tank heater system:

- `nanoradio/`: the remote Nano dashboard/transmitter with TFT, buttons, and MAX6675 K-type inputs
- `V03_QuickPID/`: the main heater controller that receives remote temperatures and setpoints over nRF24, then drives two SSR outputs with QuickPID plus adaptive thresholding

The main controller no longer uses local buttons, OLED, or local MAX6675 inputs. Those functions stay on the remote box.

## System Layout

```text
Remote Box (nanoradio)                          Main Controller (V03_QuickPID)
-------------------------------------------     -----------------------------------------
MAX6675 main/res thermocouples                  nRF24L01+ receiver
Buttons + TFT dashboard                         QuickPID loop for MAIN SSR
nRF24L01+ transmitter            -------->      QuickPID loop for RES SSR
Sends rawTemp + setpoint payload                1 s time-proportional SSR drive
                                                Radio timeout and overtemp interlocks
```

## Repository Layout

```text
nanoradio/
  nanoradio.ino
  config.h
  types.h
  sensors.h / sensors.cpp
  buttons.h / buttons.cpp
  radio.h / radio.cpp
  radio_payload.h
  serial_log.h / serial_log.cpp
  display.h / display.cpp

V03_QuickPID/
  V03_QuickPID.ino
  radio_payload.h
```

`radio_payload.h` is duplicated in both sketch folders so each sketch opens cleanly in Arduino IDE without cross-folder include issues.

## Remote Box

The remote box is still the operator-facing unit. It:

- reads the two MAX6675 thermocouples
- filters readings with an EMA
- allows setpoint edits from buttons
- renders the TFT dashboard
- transmits the latest raw temperatures and setpoints to the main controller

### Remote Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| Microcontroller | Arduino Nano (ATmega328P) | - |
| Display | Adafruit ST7789 TFT (320x240) | Hardware SPI |
| Radio | nRF24L01+ | Hardware SPI |
| Thermocouples (x2) | MAX6675 + K-type | Bit-banged SPI |
| Buttons (x3) | Momentary push | Digital INPUT_PULLUP |

### Remote Pin Map

```text
TFT Display          Radio (nRF24L01+)     Thermocouples (MAX6675)
  CS  = A1             CE  = 9               SCK  = 5 (shared)
  DC  = A2             CSN = 10              Main CS = 6, SO = 7
  RST = A3             SPI = 11,12,13        Res  CS = 8, SO = A0

Buttons (INPUT_PULLUP)
  UP  = 2    SET = 3    DOWN = 4
```

## Main Controller

`V03_QuickPID/V03_QuickPID.ino` is the heater-side controller. It:

- receives nRF24 packets from the remote box
- takes `rawTempC` and `setpointC` for both tanks from the packet
- filters the incoming temperatures locally with an EMA
- runs two independent QuickPID controllers
- applies adaptive thresholding to adjust effective `Kp`, `Ki`, and `Kd` based on error and temperature rise rate
- drives two SSR outputs using a 1 second time-proportional window
- forces heater shutdown on radio loss, invalid sensor data, or overtemperature

### Main Controller Hardware

| Component | Interface |
|-----------|-----------|
| Microcontroller | Arduino-compatible board running `V03_QuickPID` |
| Radio | nRF24L01+ |
| SSR outputs | 2 digital outputs |

### Main Controller Pins

Current pin assignments in code:

```text
SSR MAIN = 7
SSR RES  = 6
Radio CE = 9
Radio CSN = 10
Radio SPI = 11,12,13
```

If your original V03 wiring differs, update the constants near the top of [V03_QuickPID/V03_QuickPID.ino](V03_QuickPID/V03_QuickPID.ino).

## Radio Link

Both sketches use the same nRF24 settings:

- channel: `108`
- data rate: `250 kbps`
- auto-ack: disabled
- address: `"00001"`

### Radio Payload

The remote box now transmits raw sensor values and setpoints, not filtered dashboard values or UI state:

```cpp
struct RadioTankPayload {
  float rawTempC;
  float setpointC;
};

struct RadioPayload {
  RadioTankPayload main;
  RadioTankPayload res;
  uint8_t validMask;
  uint8_t counter;
};
```

`validMask` uses:

- bit `0x01` for valid MAIN temperature
- bit `0x02` for valid RES temperature

## Control Strategy

The main controller keeps the quick PID path but moves the measurement source to the remote radio packet.

### Temperature Path

```text
Remote MAX6675 raw read
  ->
radio packet
  ->
main controller packet receive
  ->
EMA filtering
  ->
QuickPID input
  ->
SSR time-proportional output
```

### Adaptive Thresholding

Each tank starts from a base tuning set:

- MAIN: `Kp=10.0`, `Ki=0.04`, `Kd=16.0`
- RES: `Kp=12.0`, `Ki=0.05`, `Kd=18.0`

Effective gains are adjusted at runtime from:

- temperature error magnitude
- whether the tank is below setpoint, near setpoint, or above setpoint
- filtered rate of temperature rise near setpoint

This keeps the controller more aggressive when far below target and more conservative when approaching or exceeding target.

### Safety Interlocks

The controller forces output to zero if:

- no valid radio packet has arrived within `1500 ms`
- the remote sensor marks a tank invalid
- the received temperature is invalid
- filtered temperature reaches `41.0 C`

Warning state starts at `39.0 C`.

## Serial Output

### Remote Box

The remote Nano still logs dashboard-side status at 115200 baud.

### Main Controller

`V03_QuickPID` logs packet age, temperatures, setpoints, outputs, effective gains, and status for both tanks. Example:

```text
PKT:42,AGE:104,MAINRaw:36.82,MAINTemp:36.74,MAINSP:37.00,MAINOut:54.2,MAINKp:10.50,MAINKi:0.032,MAINKd:12.00,MAINSt:OK,RESRaw:36.55,RESTemp:36.48,RESSP:37.00,RESOut:61.8,RESKp:12.60,RESKi:0.040,RESKd:13.50,RESSt:OK
```

## Building

### Remote Sketch

Libraries:

- `Adafruit GFX Library`
- `Adafruit ST7735 and ST7789 Library`
- `RF24`
- `MAX6675`

Example compile:

```bash
arduino-cli compile --fqbn arduino:avr:nano nanoradio/
```

### Main Controller Sketch

Libraries:

- `RF24`
- `QuickPID`

Compile with the FQBN for the board used in your main controller. Example:

```bash
arduino-cli compile --fqbn <your-board-fqbn> V03_QuickPID/
```

## Key Constants

### Shared Behavior

| Constant | Value | Description |
|----------|-------|-------------|
| `EMA_ALPHA` | `0.30` | Temperature smoothing factor |
| `MAX_TEMP_C` | `41.0 C` | Hard heater cutoff |
| `WARN_TEMP_C` | `39.0 C` | Warning threshold |

### Remote Box

| Constant | Value | Description |
|----------|-------|-------------|
| `SENSOR_INTERVAL` | `300 ms` | MAX6675 read interval |
| `RADIO_INTERVAL` | `300 ms` | Packet transmit interval |
| `SERIAL_INTERVAL` | `500 ms` | Serial log interval |
| `HISTORY_LEN` | `8` | Trend/stddev buffer length |

### Main Controller

| Constant | Value | Description |
|----------|-------|-------------|
| `CONTROL_INTERVAL_MS` | `500 ms` | QuickPID update interval |
| `WINDOW_SIZE_MS` | `1000 ms` | SSR time-proportional window |
| `RADIO_TIMEOUT_MS` | `1500 ms` | Fail-safe timeout for packet loss |
| `D_FILTER_ALPHA` | `0.25` | Rise-rate smoothing factor |
