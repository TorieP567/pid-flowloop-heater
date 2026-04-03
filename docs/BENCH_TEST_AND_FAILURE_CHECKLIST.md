# Bench Test And Failure Checklist

## Bench Setup

Use a safe no-load bench setup first:

1. Main-box Uno powered and connected to the SSR control inputs only.
2. Main-box Nano powered and connected to the Uno over I2C.
3. Remote box powered with live or simulated thermocouple data.
4. Serial monitors open on all three active boards at `115200`.
5. If possible, place indicators or meters on the SSR control lines before energizing real heaters.

## Basic Bring-Up

1. Power the Uno and confirm it boots with heaters off.
2. Power the Nano bridge and confirm I2C traffic begins.
3. Power the remote box and confirm valid remote packets are received.
4. Verify the Uno logs non-stale bridge age and remote age.
5. Verify both setpoints and temperatures appear correctly in Uno logs.
6. Verify SSR outputs begin time-proportional switching only when temperatures are below setpoint and no faults are present.
7. Verify the remote main screen shows both `Req` and `Act` setpoints.
8. Verify the remote debug screen opens with `UP+DOWN` hold and shows packet ages / fault flags.

## Functional Checklist

- MAIN tank responds to MAIN temperature/setpoint only.
- RES tank responds to RES temperature/setpoint only.
- Both setpoints are clamped to the configured safe range.
- Output percentages match expected heating demand.
- `atSetpointSeconds` starts counting only when both tanks are healthy and within band.
- remote `SET` short press changes the selected tank without double-click lag
- remote `SET` long press toggles edit mode
- remote `UP` / `DOWN` only change setpoints while edit mode is active

## Failure-Mode Tests

### 1. Wireless timeout

Test:

- turn off the remote box or stop its radio packets

Expected:

- Nano bridge keeps sending command frames with remote timeout flagged
- Uno sets remote comm fault
- both SSR outputs go low
- Uno log shows comm fault
- remote screen shows degraded / timeout communication state while remaining usable

### 2. Local Nano-to-Uno bridge failure

Test:

- disconnect SDA or SCL between the main-box Nano and Uno

Expected:

- Uno times out on bridge updates
- both SSR outputs go low
- Nano reports local bridge fault / stale status
- remote should receive a local-bridge fault indication once downlink status becomes synthetic

### 3. Single invalid sensor

Test:

- force only one remote temperature invalid or NaN-equivalent in the remote firmware

Expected:

- affected tank heater disables
- unaffected tank may continue operating if healthy
- per-tank sensor-invalid fault bit is set

### 4. Hard overtemperature cutoff

Test:

- simulate one filtered temperature at or above the configured hard max

Expected:

- that heater turns off immediately
- overtemperature fault bit sets
- other tank remains allowed if healthy

### 5. Out-of-range setpoint

Test:

- send a setpoint outside the allowed range

Expected:

- Uno clamps the setpoint
- clamped-setpoint fault bit sets
- control uses the clamped value only

### 6. Bad remote packet checksum

Test:

- inject a packet with the wrong checksum

Expected:

- Nano discards the packet
- Nano notes a dropped/invalid packet
- Uno continues using the last valid remote data until freshness expires
- when freshness expires, Uno enters remote comm fail-safe

## Before Real Heater Power

- verify SSR outputs are low on startup
- verify SSR outputs are low during any bridge or comm fault
- verify one-tank faults do not unexpectedly enable the other tank
- verify hard max shutdown happens before any unsafe heater run-on
- verify grounds are common and the nRF24 supply is stable
