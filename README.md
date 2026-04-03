# PID Flowloop Heater

This repo now targets a split main-box architecture for the dual-tank heater system:

- the `remote box` owns sensors, buttons, and the display
- the `main-box Nano` owns the nRF24 radio and bridges data
- the `main-box Uno R4 Minima` is the final heater-control authority

The new canonical firmware lives under [`firmware/`](./firmware). The older `V03_QuickPID/` and `nanoradio/` folders are kept as legacy reference material for the original single-main-MCU design and the earlier remote sketch.

## Current Architecture

```text
REMOTE BOX
  Nano + 2x MAX6675 + 2x K-type + 3 buttons + LED/TFT + nRF24
        |
        |  nRF24 wireless packets
        v
MAIN BOX NANO
  Arduino Nano + nRF24
  radio / packet validation / I2C bridge
        |
        |  I2C local bridge
        v
MAIN BOX UNO R4 MINIMA
  adaptive/self-tuning dual-tank controller
  SSR MAIN on D7
  SSR RES  on D6
```

## What Changed

- The main box is now a two-MCU system.
- The Nano in the main box is only a communications bridge.
- The Uno R4 in the main box keeps the adaptive/self-tuning heater logic and safety interlocks.
- Temperatures, setpoints, and UI-derived commands are now received over `remote -> main Nano -> main Uno`.
- Controller status is returned over `main Uno -> main Nano -> remote`.

## Canonical Firmware Targets

```text
firmware/
  shared/
    system_packets.h
  main_box_uno/
    main_box_uno.ino
  main_box_nano_bridge/
    main_box_nano_bridge.ino
  remote_box/
    remote_box.ino
```

### `firmware/main_box_uno/main_box_uno.ino`

- Runs on the main-box `Arduino Uno R4 Minima`
- Preserves the legacy controller behavior as closely as practical:
  - EMA smoothing
  - trend tracking
  - online RLS model identification
  - periodic adaptive retuning
  - adaptive PID-style control law
  - SSR time-proportional windowing
  - overtemp and invalid-sensor shutdown
- Accepts bridged sensor/setpoint data over I2C
- Drives only the SSR outputs

### `firmware/main_box_nano_bridge/main_box_nano_bridge.ino`

- Runs on the main-box `Arduino Nano`
- Owns the `nRF24L01`
- Receives remote packets
- Validates packet version/checksum/freshness
- Forwards command frames to the Uno over I2C
- Polls Uno status back over I2C
- Relays controller status back to the remote box

### `firmware/shared/system_packets.h`

- Shared packet definitions for:
  - remote-to-main radio packets
  - Nano-to-Uno I2C command packets
  - Uno-to-Nano I2C status packets
  - main-to-remote radio packets
- Defines versioning, compact fixed-point encoding, and CRC16 checksums

### `firmware/remote_box/remote_box.ino`

- Runs on the remote `Arduino Nano`
- Reads both MAX6675 thermocouples
- Handles three buttons with:
  - short `SET` press = switch selected tank
  - long `SET` press = enter/exit edit mode
  - `UP` / `DOWN` = adjust requested setpoint while editing
  - `UP+DOWN` hold = toggle main/debug screens locally
- Sends requested setpoints plus local temperatures to the main box
- Receives authoritative controller state back from the main box
- Displays both requested and active setpoints so pending UI changes are obvious

## Key Safety Behavior

- If remote wireless packets stop arriving, the Uno fails safe and forces both SSR outputs off.
- If the local Nano-to-Uno bridge stops updating, the Uno fails safe and forces both SSR outputs off.
- If one tank sensor becomes invalid or unrealistic, that tank is shut down and the other tank may continue if healthy.
- If one tank reaches the hard max temperature, that tank is shut down immediately and the other tank may continue if healthy.
- Incoming setpoints are clamped to a safe range before control is applied.
- Scheduling is `millis()`-based and non-blocking throughout the new firmware.

## Documentation

- [`docs/ARCHITECTURE.md`](./docs/ARCHITECTURE.md)
- [`docs/HARDWARE_AND_PINOUT.md`](./docs/HARDWARE_AND_PINOUT.md)
- [`docs/COMMUNICATION_PROTOCOL.md`](./docs/COMMUNICATION_PROTOCOL.md)
- [`docs/BUILD_AND_UPLOAD.md`](./docs/BUILD_AND_UPLOAD.md)
- [`docs/BENCH_TEST_AND_FAILURE_CHECKLIST.md`](./docs/BENCH_TEST_AND_FAILURE_CHECKLIST.md)
- [`docs/REMOTE_BOX_UI.md`](./docs/REMOTE_BOX_UI.md)

## Build / Upload

Use Arduino IDE 2.x or your normal CLI workflow and upload each target separately:

1. Open `firmware/main_box_uno/main_box_uno.ino`, select `Arduino Uno R4 Minima`, and upload to the main-box Uno.
2. Open `firmware/main_box_nano_bridge/main_box_nano_bridge.ino`, select `Arduino Nano`, and upload to the main-box Nano.
3. Open `firmware/remote_box/remote_box.ino`, select `Arduino Nano`, and upload to the remote box.

Library requirements:

- Main-box Uno:
  - `Wire` (core)
- Main-box Nano bridge:
  - `Wire` (core)
  - `SPI` (core)
  - `RF24`
- Remote box:
  - `SPI` (core)
  - `RF24`
  - `Adafruit GFX`
  - `Adafruit ST7735 and ST7789 Library`
  - `max6675`

## Legacy Reference Material

These folders are retained as baseline/reference code and are not the active architecture:

- `V03_QuickPID/`
- `nanoradio/`

The richer adaptive/self-tuning behavior in the new Uno sketch was derived from the legacy controller logic in the repo, but the active deployment path is now the `firmware/` tree above.
