# Hardware And Pinout

This document reflects the current canonical circuit wiring for the active
main-box firmware under [`firmware/`](../firmware/) and the active remote-box
firmware under [`nanoradio/`](../nanoradio/).

## Main Box

### Arduino Uno R4 Minima

Role: final heater controller

| Connection | Pin | Notes |
|---|---:|---|
| SSR outlet box #1 control input (+) | `D7` | MAIN tank heater output |
| SSR outlet box #1 control input (-) | `GND` | SSR control return |
| SSR outlet box #2 control input (+) | `D6` | RES tank heater output |
| SSR outlet box #2 control input (-) | `GND` | SSR control return |
| Main-box Nano `5V` | `5V` | Powers the Nano main board |
| Main-box Nano `GND` | `GND` | Common ground |
| Main-box Nano `A4` | `SDA` | I2C data to Nano main |
| Main-box Nano `A5` | `SCL` | I2C clock to Nano main |

### Arduino Nano Main

Role: radio + bridge

| Connection | Pin | Notes |
|---|---:|---|
| Uno `5V` | `5V` | Powered from the Uno |
| Uno `GND` | `GND` | Common ground with Uno and radio |
| Uno `SDA` | `A4` | I2C data back to Uno |
| Uno `SCL` | `A5` | I2C clock back to Uno |
| nRF24L01 `VCC` | `3.3V` | Radio power |
| nRF24L01 `GND` | `GND` | Radio ground |
| nRF24L01 `CE` | `D9` | |
| nRF24L01 `CSN` | `D10` | |
| nRF24L01 `MOSI` | `D11` | |
| nRF24L01 `MISO` | `D12` | |
| nRF24L01 `SCK` | `D13` | |

Important:

- Add a `10 uF` capacitor across the nRF24 `VCC` and `GND` close to the module.
- Keep the Uno, Nano, radio, and SSR control grounds common.

## Remote Box

Role: sensors + UI + radio

### Arduino Nano Remote to nRF24L01

| Connection | Pin |
|---|---:|
| nRF24L01 `VCC` | `3.3V` |
| nRF24L01 `GND` | `GND` |
| nRF24L01 `CE` | `D9` |
| nRF24L01 `CSN` | `D10` |
| nRF24L01 `MOSI` | `D11` |
| nRF24L01 `MISO` | `D12` |
| nRF24L01 `SCK` | `D13` |

Important:

- Add a `10 uF` capacitor across `VCC` / `GND` close to the remote radio module.

### MAX6675 #1

| Connection | Pin |
|---|---:|
| `SCK` | `D5` |
| `SO` | `D7` |
| `CS` | `D6` |
| `VCC` | `5V` |
| `GND` | `GND` |

### MAX6675 #2

| Connection | Pin |
|---|---:|
| `SCK` | `D5` |
| `SO` | `A0` |
| `CS` | `D8` |
| `VCC` | `5V` |
| `GND` | `GND` |

### ST7789 TFT

| Connection | Pin |
|---|---:|
| `VCC` | `5V` |
| `GND` | `GND` |
| `DIN` | `D11` |
| `CLK` | `D13` |
| `CS` | `A1` |
| `DC` | `A2` |
| `RST` | `A3` |
| `BL` | `5V` |

### Buttons

| Button | Pin | Mode |
|---|---:|---|
| Button 1 | `D2` | `INPUT_PULLUP` |
| Button 2 | `D3` | `INPUT_PULLUP` |
| Button 3 | `D4` | `INPUT_PULLUP` |

Each button connects between the Nano pin and `GND`.

The remote box does not own final heater output decisions.
It owns sensor reads, button-driven requested setpoints, and the display.

## Wiring Summary

```text
Main-box Uno D7  -----------------> SSR outlet box #1 control input (+)
Main-box Uno GND -----------------> SSR outlet box #1 control input (-)
Main-box Uno D6  -----------------> SSR outlet box #2 control input (+)
Main-box Uno GND -----------------> SSR outlet box #2 control input (-)

Main-box Uno 5V  -----------------> Main-box Nano 5V
Main-box Uno GND -----------------> Main-box Nano GND
Main-box Uno SDA -----------------> Main-box Nano A4
Main-box Uno SCL -----------------> Main-box Nano A5

Main-box Nano D9/D10/D11/D12/D13 -> nRF24L01 CE/CSN/MOSI/MISO/SCK
Remote Nano D9/D10/D11/D12/D13 --> nRF24L01 CE/CSN/MOSI/MISO/SCK

Remote Nano D5/D7/D6 -----------> MAX6675 #1 SCK/SO/CS
Remote Nano D5/A0/D8 -----------> MAX6675 #2 SCK/SO/CS
Remote Nano D11/D13/A1/A2/A3 ---> ST7789 DIN/CLK/CS/DC/RST
Remote Nano D2/D3/D4 -----------> Button 1 / Button 2 / Button 3

All board and peripheral grounds should be common.
```
