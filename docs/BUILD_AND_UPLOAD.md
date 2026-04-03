# Build And Upload

## Firmware Targets

Upload these sketches separately:

1. [`firmware/main_box_uno/main_box_uno.ino`](../firmware/main_box_uno/main_box_uno.ino)
2. [`firmware/main_box_nano_bridge/main_box_nano_bridge.ino`](../firmware/main_box_nano_bridge/main_box_nano_bridge.ino)
3. [`firmware/remote_box/remote_box.ino`](../firmware/remote_box/remote_box.ino)

## Board Selection

In Arduino IDE:

- `main_box_uno.ino` -> select `Arduino Uno R4 Minima`
- `main_box_nano_bridge.ino` -> select `Arduino Nano`
- `remote_box.ino` -> select `Arduino Nano`

## Required Libraries

### Main-box Uno

- `Wire` from the Arduino core

### Main-box Nano bridge

- `Wire` from the Arduino core
- `SPI` from the Arduino core
- `RF24`

### Remote box

- `SPI` from the Arduino core
- `RF24`
- `Adafruit GFX`
- `Adafruit ST7735 and ST7789 Library`
- `max6675`

## Recommended Upload Order

1. Upload the `main_box_uno` sketch first.
2. Open the serial monitor for the Uno at `115200`.
3. Upload the `main_box_nano_bridge` sketch second.
4. Open the serial monitor for the Nano bridge at `115200`.
5. Upload the `remote_box` sketch third.
6. Open the serial monitor for the remote box at `115200`.
7. Power the full system and verify the packet chain starts updating.

## Serial Debug Expectations

### Uno debug stream

The Uno logs:

- bridge sequence and remote sequence
- bridge age and remote age
- filtered temperatures and raw temperatures
- setpoints
- output percentages
- heater states
- current adaptive gains
- estimated tau from the online model when valid
- fault bitfield and per-tank status

### Nano bridge debug stream

The Nano logs:

- radio init state
- valid and invalid remote packet counts
- I2C write success/failure to the Uno
- I2C status poll success/failure from the Uno
- local bridge freshness
- downlink fault/link flags

### Remote box debug stream

The remote logs:

- raw MAX6675 readings
- requested setpoints
- uplink packet sequence and send result
- downlink status sequence and receive age
- link-state transitions
- selected tank
- edit-mode state
- active screen
- returned fault flags

## Notes

- Keep all active serial monitors at `115200 baud`.
- The Nano bridge and Uno must share ground.
- The nRF24 on the main-box Nano should have a local `10 uF` decoupling capacitor.
- The nRF24 on the remote box should also have a local `10 uF` decoupling capacitor.
