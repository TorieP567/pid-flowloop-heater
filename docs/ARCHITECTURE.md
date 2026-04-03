# Architecture

## System Split

The heater system is now intentionally split across three nodes:

1. `Remote box`
   - Arduino Nano
   - 2x MAX6675 thermocouple interfaces
   - 2x K-type thermocouples
   - 3 buttons
   - ST7789 TFT display
   - nRF24 radio
   - local UI state for requested setpoints

2. `Main-box Nano`
   - Arduino Nano
   - nRF24 radio endpoint in the main box
   - validates radio packets
   - bridges structured messages to the main-box Uno over I2C

3. `Main-box Uno R4 Minima`
   - final heater-control authority
   - dual-tank adaptive/self-tuning control logic
   - heater safety interlocks
   - SSR output control only

## Data Flow

```text
Remote sensors/buttons/UI
  -> RemoteToMainPacket
  -> nRF24
  -> main-box Nano
  -> NanoToUnoCommandPacket
  -> I2C
  -> main-box Uno
  -> UnoToNanoStatusPacket
  -> I2C
  -> main-box Nano
  -> MainToRemotePacket
  -> nRF24
  -> remote box display/logging
```

## Control Authority

The Uno R4 remains the only board allowed to make final heater decisions.

- The Nano bridge never owns final SSR output decisions.
- The Nano bridge can report packet freshness and bridge faults.
- The Uno decides whether heaters are allowed to run.
- The remote box can request setpoints but only displays acknowledged controller state.

## Remote UI Behavior

The remote firmware now runs from `nanoradio/nanoradio.ino`.

- `SET` short press switches the selected tank
- `SET` long press enters/exits edit mode
- `UP` and `DOWN` change the selected tank setpoint while editing
- `UP + DOWN` hold toggles between main and debug screens locally

The remote transmits local raw temperatures and requested setpoints, then displays the main-box status packet as the authoritative control state.

The older monolithic sketch under `firmware/remote_box/` is deprecated and no longer the canonical implementation.

## Preserved Legacy Controller Behavior

The new Uno sketch keeps the legacy dual-tank behavior wherever practical:

- EMA smoothing of incoming temperatures
- short-term trend history
- recursive least-squares online model identification
- periodic adaptive retuning
- adaptive control law with aggressive/fine/overshoot regimes
- 1-second SSR time-proportional output windowing
- serial debug telemetry

## Fault Domains

### Global faults

These force both heaters off:

- remote wireless timeout
- missing or stale Nano-to-Uno bridge updates

### Per-tank faults

These only force the affected tank off:

- invalid or unrealistic tank temperature
- hard overtemperature cutoff

If only one tank faults, the other tank remains allowed to run when its own sensor and command path are still healthy.

## Timing Model

The new firmware uses `millis()`-based scheduling only.

- no blocking delay loops in normal runtime
- controller update cadence remains periodic
- SSR outputs are time-proportional within a fixed window
- packet age is tracked explicitly rather than inferred from blocking loops
