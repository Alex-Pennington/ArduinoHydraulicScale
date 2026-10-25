# Radio Module — PLC #4 Tractor Scale

**Document:** 01A — Radio Module
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

## Module

| Field | Value |
|---|---|
| Module | HC-05 serial radio [1] |
| Role | Field console, the same console as USB [1] |
| Serial port | `Serial1` [1] |
| Commands | Every console verb, the same as over USB, `06` §*Commands* |
| Output | Every console line, mirrored to USB and to this module [1] |

## Terminals

Board pins only. The module side lands as `00` §*Wiring conventions* states.

| Pin | Function | Spec | Wire | Connected to |
|---|---|---|---|---|
| D0 | Serial receive, console [1], [2] | 9600 baud [1] | | Radio module transmit |
| D1 | Serial transmit, console [1], [2] | 9600 baud [1] | | Radio module receive |

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 25–46 and 1460, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[2] Arduino, "Arduino UNO R4 WiFi," datasheet, SKU ABX00087, §12.2 *Digital*. Filed in `docs/vendor/arduino/`
