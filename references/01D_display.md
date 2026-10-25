# Display — PLC #4 Tractor Scale

**Document:** 01D — Display
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

## Display

| Field | Value |
|---|---|
| Module | LCD2004A, 20 characters by 4 rows [1] |
| Interface | PCF8574 backpack [2] on the sensor bus, `01C` §*Bus channels* |
| Inputs | None. The display is read-only; operator actions arrive over MQTT, `03` §*Command* |

## Rows

| Row | Shows, first match wins |
|---|---|
| 1 | Weight in pounds and the tally count; the fault name when the reading cannot be trusted [3] |
| 2 | Pressure in psi and loop current in mA; dashes when the reading cannot be trusted [3] |
| 3 | `DROPPED` and the count of dropped weights; queued weights and the link; tally full; tally slots left when five or fewer remain; the link state [3]. Wording: `06` §*The tally* |
| 4 | The fault name when the reading cannot be trusted; the uncalibrated warning; the converter failure count; blank [4]. Wording: `06` §*What the display asks for* |

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `include/lcd2004.h`, lines 6–14, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[2] Texas Instruments, "PCF8574 remote 8-bit I/O expander for I2C bus," datasheet SCPS068K. Filed in `docs/vendor/texas-instruments/`

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 1672–1712, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[4] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 1658–1670, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
