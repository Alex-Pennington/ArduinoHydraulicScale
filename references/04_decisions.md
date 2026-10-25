# Decisions — PLC #4 Tractor Scale

**Document:** 04 — Decisions
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

What has been decided about this machine, and by whom. Rulings only.

## Platform

| Decision | Value | Decided by |
|---|---|---|
| Controller | Arduino UNO R4 WiFi; Opta not adopted | Alex |

## Measurement

| Decision | Value | Decided by |
|---|---|---|
| Front end | Two-wire 4-20 mA current loop | Alex |
| Transmitter | `02` §*Parts* | Alex |
| Loop supply | `02` §*Parts* | Alex |
| Supply negative | Bonded to board ground, `00` §*Wiring conventions* | derived |
| Burden resistor | `02` §*Parts* | Alex |
| Burden placement | `02` §*Connections* | Alex |
| Converter | `02` §*Parts* | Alex |
| Circuit pressure | Treated as exceeding 3000 psi in service | Alex |

## Scaling

| Decision | Value | Decided by |
|---|---|---|
| Source of truth | Milliamps. Counts are derived at compile time from the burden [1] | Alex |
| Hand-written counts | Not permitted anywhere | Alex |
| Live zero | 4 mA is zero pressure, not zero counts [1] | derived |
| Fault banding | `02` §*What each current means*. The high fault threshold is the NAMUR NE43 fault level [1]; the transmitter offers NE43-compliant output levels as an option [2] | derived |

## Reporting

| Decision | Value | Decided by |
|---|---|---|
| Topic root | `plc4/jd5055e` | Alex |
| Offline weights | Queue while the broker is out of reach and publish when it returns | Alex |
| Weight queue | RAM only, `03` §*Weight* | Alex |
| Buffered weights | Carry their age and a backlog flag | derived |
| Pressure and weight when not measuring | Publish null, never zero | derived |
| Status topic | Retained, so a late subscriber learns the machine's state | derived |
| Acknowledgement | QoS 0, so an ack can never stall the weighing loop | Alex |
| Operator interface | Phone app over MQTT; no buttons on the enclosure | Alex |
| Tractor bus | Listen only. The board transmits nothing, including requests for engine hours | Alex |
| Converter and loop faults | Reported as different faults, `06` §*What the display asks for* | derived |

## Operating

| Decision | Value | Decided by |
|---|---|---|
| Weighing position | `06` §*Weighing* | Alex |
| Calibration model | Single point against a known load, valid at one boom position | Alex |
| Tare | `06` §*Weighing* | Alex |
| Calibration store | A stored calibration from a different front end is rejected, forcing a recalibration | Alex |
| Filter on a fault | Frozen; the last reading is held, never smoothed toward the fault | Alex |

## See also

`03` §*Sensor states* for the six values and what each one means.

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `include/loop_scaling.h`, lines 28–59, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[2] Emerson Process Management, "Rosemount 1151 Pressure Transmitter," Product Data Sheet 00813-0100-4360, Rev. JB, ordering codes CN and C4. Filed in `docs/vendor/emerson-rosemount/`
