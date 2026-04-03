# Communication Protocol

The canonical packet definitions live in [`firmware/shared/system_packets.h`](../firmware/shared/system_packets.h).

## Design Goals

- fit within the `32-byte` nRF24 payload limit
- fit within the `32-byte` AVR `Wire` buffer limit
- include explicit packet versioning
- include a lightweight integrity check
- carry freshness/heartbeat information
- keep the Nano as a bridge, not a controller

## Shared Constants

- I2C address for the Uno: `0x42`
- nRF24 channel: `108`
- remote-to-main pipe: `"R2M01"`
- main-to-remote pipe: `"M2R01"`
- checksum: `CRC16-CCITT`

## Remote -> Main Radio Packet

Struct: `RemoteToMainPacket`

| Field | Type | Meaning |
|---|---|---|
| `version` | `uint8_t` | protocol version |
| `sequence` | `uint16_t` | increments every remote packet |
| `remoteMillis` | `uint32_t` | remote heartbeat / uptime snapshot |
| `mainTempCx100` | `int16_t` | MAIN temperature in `degC * 100` |
| `resTempCx100` | `int16_t` | RES temperature in `degC * 100` |
| `mainSetpointCx100` | `int16_t` | MAIN setpoint in `degC * 100` |
| `resSetpointCx100` | `int16_t` | RES setpoint in `degC * 100` |
| `validFlags` | `uint8_t` | temp validity bits |
| `uiFlags` | `uint8_t` | UI mode bits |
| `buttonFlags` | `uint8_t` | button-event bits |
| `checksum` | `uint16_t` | CRC16 over the packed struct |

### `validFlags`

- `0x01` = MAIN temp valid
- `0x02` = RES temp valid

### `uiFlags`

- `0x01` = setpoint edit mode active
- `0x02` = RES tank currently selected in the UI

### `buttonFlags`

- `0x01` = UP event
- `0x02` = SET event
- `0x04` = DOWN event
- `0x08` = double-click event

Current remote firmware behavior:

- `SET` short press changes the selected tank and sends `SET`
- `SET` long press toggles edit mode and sends `SET`
- `UP` / `DOWN` send events only when they actually change a requested setpoint
- the remote does not use double-click, so `0x08` is currently left clear

## Main Nano -> Main Uno I2C Command Packet

Struct: `NanoToUnoCommandPacket`

This is the bridge-side command frame forwarded to the Uno.

| Field | Type | Meaning |
|---|---|---|
| `version` | `uint8_t` | protocol version |
| `bridgeSequence` | `uint16_t` | increments every Nano-to-Uno write |
| `remoteSequence` | `uint16_t` | last valid remote packet sequence |
| `remoteAgeMs` | `uint16_t` | age of the last valid remote packet |
| `mainTempCx100` | `int16_t` | forwarded MAIN temp |
| `resTempCx100` | `int16_t` | forwarded RES temp |
| `mainSetpointCx100` | `int16_t` | forwarded MAIN setpoint |
| `resSetpointCx100` | `int16_t` | forwarded RES setpoint |
| `validFlags` | `uint8_t` | forwarded validity bits |
| `uiFlags` | `uint8_t` | forwarded UI bits |
| `buttonFlags` | `uint8_t` | forwarded button event bits |
| `bridgeFlags` | `uint8_t` | Nano bridge freshness/health flags |
| `checksum` | `uint16_t` | CRC16 |

### `bridgeFlags`

- `0x01` = at least one valid remote packet has been received
- `0x02` = remote packet stream is currently fresh
- `0x04` = last stored remote packet passed validation
- `0x08` = one or more remote packets were dropped as invalid since the last good one
- `0x10` = remote packet stream is timed out/stale

This split is important:

- if radio packets stop but I2C still works, the Nano still sends command frames
- the Uno can then distinguish `remote comm fault` from `local bridge fault`

## Main Uno -> Main Nano I2C Status Packet

Struct: `UnoToNanoStatusPacket`

| Field | Type | Meaning |
|---|---|---|
| `version` | `uint8_t` | protocol version |
| `statusSequence` | `uint16_t` | increments for each Uno status publish |
| `bridgeSequenceEcho` | `uint16_t` | last bridge command sequence seen by the Uno |
| `commandAgeMs` | `uint16_t` | age of the last command frame seen by the Uno |
| `mainFilteredCx100` | `int16_t` | MAIN filtered temperature |
| `resFilteredCx100` | `int16_t` | RES filtered temperature |
| `mainSetpointCx100` | `int16_t` | MAIN setpoint in use |
| `resSetpointCx100` | `int16_t` | RES setpoint in use |
| `mainOutputPermille` | `uint16_t` | MAIN output demand in `0.1%` |
| `resOutputPermille` | `uint16_t` | RES output demand in `0.1%` |
| `faultFlags` | `uint16_t` | controller fault/warning bits |
| `heaterFlags` | `uint8_t` | heater ON/OFF bits |
| `atSetpointSeconds` | `uint16_t` | time both tanks have stayed in band |
| `checksum` | `uint16_t` | CRC16 |

## Main -> Remote Radio Packet

Struct: `MainToRemotePacket`

This is what the main-box Nano sends back to the remote box.

It mirrors the Uno status and adds bridge/radio-link context.

| Field | Type | Meaning |
|---|---|---|
| `version` | `uint8_t` | protocol version |
| `statusSequence` | `uint16_t` | downlink status sequence |
| `remoteSequenceEcho` | `uint16_t` | latest remote sequence incorporated |
| `controllerAgeMs` | `uint16_t` | age of controller status / last command |
| `mainFilteredCx100` | `int16_t` | MAIN filtered temperature |
| `resFilteredCx100` | `int16_t` | RES filtered temperature |
| `mainSetpointCx100` | `int16_t` | MAIN setpoint in use |
| `resSetpointCx100` | `int16_t` | RES setpoint in use |
| `mainOutputPermille` | `uint16_t` | MAIN output demand |
| `resOutputPermille` | `uint16_t` | RES output demand |
| `faultFlags` | `uint16_t` | system/controller faults |
| `heaterFlags` | `uint8_t` | heater ON bits |
| `linkFlags` | `uint8_t` | radio/bridge freshness summary |
| `atSetpointSeconds` | `uint16_t` | time in band |
| `checksum` | `uint16_t` | CRC16 |

### `heaterFlags`

- `0x01` = MAIN heater ON
- `0x02` = RES heater ON

### `faultFlags`

- `0x0001` = remote comm fault
- `0x0002` = local bridge fault
- `0x0004` = MAIN sensor invalid
- `0x0008` = RES sensor invalid
- `0x0010` = MAIN overtemperature cutoff
- `0x0020` = RES overtemperature cutoff
- `0x0040` = MAIN setpoint clamped
- `0x0080` = RES setpoint clamped
- `0x0100` = MAIN forced off by safety
- `0x0200` = RES forced off by safety
- `0x0400` = MAIN warning temperature reached
- `0x0800` = RES warning temperature reached
- `0x1000` = invalid remote packet(s) seen
- `0x2000` = controller status stale

### `linkFlags`

- `0x01` = remote packet stream fresh
- `0x02` = local bridge healthy
- `0x04` = Uno status fresh
- `0x08` = status packet is synthetic/failsafe
- `0x10` = at least one valid remote packet received
- `0x20` = previous radio transmit succeeded

## Expected Remote Firmware Behavior

The remote box should:

1. Read the two thermocouples.
2. Clamp/edit requested setpoints locally in the UI.
3. Populate `RemoteToMainPacket`.
4. Update `sequence` every outbound packet.
5. Mark invalid sensors via `validFlags`.
6. Send packets on the `R2M01` pipe.
7. Listen for `MainToRemotePacket` updates on the `M2R01` pipe.

The implemented remote firmware in `nanoradio/nanoradio.ino` shows both:

- `Req`: the local requested setpoint
- `Act`: the authoritative setpoint reported back by the main box

That keeps UI intent separate from controller acknowledgement.

## Fixed-Point Encoding

- temperatures/setpoints use `degC * 100`
- output demand uses `0.1%` increments
- invalid temperature sentinel is `INT16_MIN`
