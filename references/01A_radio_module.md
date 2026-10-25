# Radio Module — PLC #4 Tractor Scale

**Document:** 01A — Radio Module
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

## Module

| Field | Value |
|---|---|
| Module | HC-05 serial radio [1] |
| Role | Field console, the same console as USB [1] |
| Serial port | `Serial1`, 9600 baud [1] |
| Commands | Every console verb, the same as over USB, `06` §*Commands* |
| Output | Every console line, mirrored to USB and to this module [1] |

## Terminals

| Terminal | Function | Wire | Connected to |
|---|---|---|---|
| Transmit output | Console data to the board | | Board D0, `01` §*Header pins* |
| Receive input | Console data from the board | | Board D1, `01` §*Header pins* |

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 25–46 and 1460, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
