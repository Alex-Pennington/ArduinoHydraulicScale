# Controller Card — PLC #4 Tractor Scale

**Document:** 01 — Controller Card
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

## Board

| Field | Value |
|---|---|
| Controller | Arduino UNO R4 WiFi [1] |
| Sensor bus | Qwiic connector, the board's second two-wire bus, `Wire1` [1], [2] |
| Console | Serial over USB and over the radio module, both at once [2] |
| Radio module pins | `01A` §*Terminals* |
| Header pins used | Only those in §*Header pins* |

## Header pins

| Pin | Function | Spec | Wire | Connected to |
|---|---|---|---|---|
| D10 | CAN transmit, the CAN controller's pin [1] | 250 kbit/s [3] | | Not connected |
| D13 | CAN receive, the CAN controller's pin [1] | 250 kbit/s [3] | | CAN transceiver, P1 pin 2, `01B` §*Terminals* |
| GND | Board ground | | | Loop supply negative, `02` §*Connections*; CAN transceiver, P1 pin 3, `01B` §*Terminals* |
| Qwiic | Sensor bus trunk | 3.3 V [1] | | Multiplexer, `01C` §*Bus channels* |
| USB | Console | 115200 baud [2] | | Laptop, when servicing |

## CAN

| Item | Value |
|---|---|
| Controller | On-chip, 250 kbit/s, 29-bit identifiers [3], [4] |
| Mode | Normal. The controller acknowledges frames, and the transceiver's transmit input, tied recessive, keeps the node off the bus [3], `01B` §*Terminals* |
| Transmit | The firmware sends no frame and no request [3] |
| Hardware filter | Off at every power-up; `canf` turns it on [3], [5] |

## References

[1] Arduino, "Arduino UNO R4 WiFi," datasheet, SKU ABX00087, §10 *Qwiic I2C Connector* and §12.2 *Digital*. Filed in `docs/vendor/arduino/`

[2] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 25–46 and 1460–1461, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_can.h`, lines 23–26, 33 and 74–87, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[4] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_decode.h`, line 332, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[5] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, line 1583, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
