# Remote Box UI

## Purpose

The remote box is the operator-facing node.

Canonical firmware path: [`nanoradio/nanoradio.ino`](../nanoradio/nanoradio.ino)

It owns:

- the two thermocouple inputs
- the three buttons
- the ST7789 display
- the outbound requested setpoints

It does not own final heater control.

## Button Behavior

- short press `SET`: switch the selected tank between `MAIN` and `RES`
- long press `SET`: enter or exit setpoint edit mode
- press `UP`: raise the selected tank setpoint while edit mode is active
- press `DOWN`: lower the selected tank setpoint while edit mode is active
- hold `UP + DOWN`: toggle between the main screen and debug screen

Notes:

- button handling is debounced
- short/long press detection is non-blocking
- there is no double-click delay

## Main Screen

The main screen is meant to be readable at a glance.

For each tank it shows:

- large temperature
- requested setpoint from the remote UI
- active setpoint acknowledged by the main box
- heater output percentage and ON/OFF state
- source tag for the displayed temperature:
  - `AUTH` when the returned controller temperature is fresh
  - `LAST` when the last returned controller temperature is stale but not timed out
  - `LOCAL` when the screen falls back to the local raw thermocouple reading

The footer shows:

- link status
- warning/fault summary
- at-setpoint timer from the main box when available

## Debug Screen

The debug screen shows transport and controller details:

- local raw temperatures
- requested setpoints
- active setpoints from the main box
- transmit sequence and received status sequence
- received status age
- controller age field returned by the main box
- output percentages
- heater flags
- fault flags and link flags
- packet success/failure counters

## Requested vs Active Setpoints

The remote keeps the user-requested setpoints locally for editing.

The main box remains authoritative, so the UI shows both:

- `Req`: what the remote is currently asking for
- `Act`: what the main box says it is actually using

This makes it easy to see when a new request is still waiting on radio/bridge confirmation or when the controller has clamped a setpoint for safety.

## Communication-Loss Behavior

If the remote stops hearing back from the main box:

- the UI stays responsive
- requested setpoints can still be edited locally
- the link state moves through `DEGRADED` to `TIMEOUT`
- the main screen fault area shows a communication problem
- the remote keeps transmitting periodic uplink packets to re-establish the link
