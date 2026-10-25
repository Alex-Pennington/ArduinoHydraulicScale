# Qwiic Stack — PLC #4 Tractor Scale

**Document:** 01C — Qwiic Stack
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

## Trunk

| Field | Value |
|---|---|
| Bus | Board Qwiic connector, `01` §*Header pins* |
| Multiplexer | TCA9548A [1] or PCA9548A [2], eight channels, address `0x70`, the only device on the trunk [3] |

## Bus channels

| Channel | Address | Device | Used by the firmware | Wire |
|---|---|---|---|---|
| 0 | `0x67` | MCP9600 thermocouple amplifier [4] | no [3] | |
| 1 | `0x2A` | NAU7802 load-cell amplifier [5] | no [3] | |
| 2 | `0x40` | INA219 voltage and current monitor [6] | yes, diagnostics [3] | |
| 3 | `0x68` | MCP3421A0 pressure-loop converter [7], `02` §*Parts* | yes, the measurement [3] | |
| 7 | `0x27` | LCD2004A display, PCF8574 backpack [8], `01D` §*Display* | yes [3] | |

## References

[1] Texas Instruments, "TCA9548A low-voltage 8-channel I2C switch with reset," datasheet SCPS207H. Filed in `docs/vendor/texas-instruments/`

[2] Texas Instruments, "PCA9548A low voltage 8-channel I2C switch with reset," datasheet SCPS143G. Filed in `docs/vendor/texas-instruments/`

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `include/i2c_mux.h`, lines 5–39, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[4] Microchip Technology, "MCP960X/L0X/RL0X thermocouple EMF to temperature converter, ±1.5 °C maximum accuracy," datasheet DS20005426G. Filed in `docs/vendor/microchip/`

[5] Nuvoton Technology, "NAU7802 precision ADC with 2-wire control interface," datasheet Rev 2.6. Filed in `docs/vendor/nuvoton/`

[6] Texas Instruments, "INA219 zero-drift, bidirectional current/power monitor with I2C interface," datasheet SBOS448G. Filed in `docs/vendor/texas-instruments/`

[7] Microchip Technology, "MCP3421 18-bit analog-to-digital converter with I2C interface and on-board reference," datasheet DS22003E. Filed in `docs/vendor/microchip/`

[8] Texas Instruments, "PCF8574 remote 8-bit I/O expander for I2C bus," datasheet SCPS068K. Filed in `docs/vendor/texas-instruments/`
