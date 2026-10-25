# CAN Transceiver — PLC #4 Tractor Scale

**Document:** 01B — CAN Transceiver
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

## Module

| Field | Value |
|---|---|
| Board | Waveshare SN65HVD230 CAN Board [1] |
| Transceiver | TI SN65HVD230, 3.3 V supply [1], [2] |
| Bus | Tractor J1939 bus, `01` §*CAN* |

## Terminals

| Terminal | Transceiver pin | Function | Wire | Connected to |
|---|---|---|---|---|
| P1 pin 1 | D | Transmit data input; high is recessive [1], [2] | | Tied recessive, so the node cannot drive the bus [3] |
| P1 pin 2 | R | Receive data output [1], [2] | | Board D13, `01` §*Header pins* |
| P1 pin 3 | GND | Ground [1], [2] | | Board GND, `01` §*Header pins* |
| P1 pin 4 | VCC | 3.3 V supply [1], [2] | | |
| can1 pin 2 | CANH | CAN high [1], [2] | | Tractor J1939 bus, CAN high |
| can1 pin 1 | CANL | CAN low [1], [2] | | Tractor J1939 bus, CAN low |

## References

[1] Waveshare, "SN65HVD230 CAN Board," schematic. Filed in `docs/vendor/waveshare/`

[2] Texas Instruments, "SN65HVD23x 3.3-V CAN Bus Transceivers," datasheet SLOS346O, *Pin Functions*. Filed in `docs/vendor/texas-instruments/`

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_can.h`, lines 23–26, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
