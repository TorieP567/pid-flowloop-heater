# Hardware And Pinout

## Main Box Uno R4 Minima

Role: final heater controller

| Signal | Pin | Notes |
|---|---:|---|
| SSR MAIN | `D7` | MAIN tank heater output |
| SSR RES | `D6` | RES tank heater output |
| I2C SDA | `SDA` | To main-box Nano `A4` |
| I2C SCL | `SCL` | To main-box Nano `A5` |
| GND | `GND` | Common ground with Nano and SSR logic reference |

## Main Box Nano

Role: radio + bridge

### nRF24L01

| Signal | Pin |
|---|---:|
| CE | `D9` |
| CSN | `D10` |
| MOSI | `D11` |
| MISO | `D12` |
| SCK | `D13` |
| VCC | `3.3V` |
| GND | `GND` |

### I2C link to Uno

| Signal | Pin |
|---|---:|
| SDA | `A4` |
| SCL | `A5` |
| GND | `GND` |

Important:

- Add a `10 uF` capacitor across the nRF24 `VCC` and `GND` close to the module.
- Keep the Nano, Uno, radio, and SSR control grounds common.

## Remote Box

Role: sensors + UI + radio

### Remote Nano nRF24L01

| Signal | Pin |
|---|---:|
| CE | `D9` |
| CSN | `D10` |
| MOSI | `D11` |
| MISO | `D12` |
| SCK | `D13` |
| VCC | `3.3V` |
| GND | `GND` |

Important:

- add a `10 uF` capacitor across `VCC` / `GND` close to the radio module

### MAX6675 #1

| Signal | Pin |
|---|---:|
| SCK | `D13` |
| SO | `D12` |
| CS | `D6` |
| VCC | `5V` |
| GND | `GND` |

### MAX6675 #2

| Signal | Pin |
|---|---:|
| SCK | `D13` |
| SO | `D12` |
| CS | `D7` |
| VCC | `5V` |
| GND | `GND` |

### ST7789 TFT

| Signal | Pin |
|---|---:|
| CS | `D4` |
| DC | `D5` |
| RST | `D8` |
| MOSI / DIN | `D11` |
| SCK | `D13` |
| VCC | `VCC` |
| GND | `GND` |
| BL | `VCC` |

### Buttons

| Button | Pin | Mode |
|---|---:|---|
| UP | `A0` | `INPUT_PULLUP` |
| SET | `A1` | `INPUT_PULLUP` |
| DOWN | `A2` | `INPUT_PULLUP` |

The remote uses one side of each button to the Nano pin and the other side to `GND`.

The remote box no longer owns final heater output decisions.
It owns sensor reads, button-driven requested setpoints, and the display.

## Wiring Summary

```text
Remote box nRF24  <----wireless---->  Main-box Nano nRF24
Remote box D6/D7  ----------shared--  MAX6675 chip selects
Remote box D4/D5/D8 --------shared--  ST7789 control pins
Remote box A0/A1/A2 --------local---  buttons
Main-box Nano A4  <------I2C------->  Main-box Uno SDA
Main-box Nano A5  <------I2C------->  Main-box Uno SCL
Main-box Uno D7   ----------------->  SSR outlet box #1 control
Main-box Uno D6   ----------------->  SSR outlet box #2 control
All grounds common
```
