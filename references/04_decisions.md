# Decisions — PLC #4 Tractor Scale
## Decision Record

PHOENIX NEST LLC · PLC #4
**Document:** 04 — Decisions
**Firmware:** build `699e3c6c`
**Controller:** Arduino Uno R4 WiFi

What has been decided about this machine, and by whom. Rulings only. The arguments behind them
are in the repository history and are not reproduced here.

## Measurement

| Decision | Value | Decided by |
|---|---|---|
| Front end | Two-wire 4-20 mA current loop | Alex |
| Transmitter | Rosemount 1151, 0–3000 psi | Alex |
| Loop supply | Dedicated 24 V, separate from the board supply | Alex |
| Supply negative | Bonded to board ground | derived |
| Burden resistor | 68 Ω, 0.1 %, 25 ppm/°C or better | Alex |
| Burden placement | Across the converter's differential input, in the loop return leg | Alex |
| Converter | Own voltage reference, not the board's supply rail | Alex |
| Circuit pressure | Treated as exceeding 3000 psi in service | Alex |

The burden was 150 Ω while the board's own input did the reading, and 250 Ω before that. Both
were correct for the converter of their day and neither is correct now. Fit 68 Ω.

## Scaling

| Decision | Value | Decided by |
|---|---|---|
| Source of truth | Milliamps. Counts are derived at compile time from the burden | Alex |
| Hand-written counts | Not permitted anywhere | Alex |
| Live zero | 4 mA is zero pressure, not zero counts | derived |
| Fault banding | NAMUR NE43: dead below 3.5 mA, fault at 21 mA and above | derived |

Change the burden and every threshold follows from it. A count written by hand is a number that
stops being true the moment the hardware moves and says nothing when it does.

## Reporting

| Decision | Value | Decided by |
|---|---|---|
| Topic root | `plc4/jd5055e` | Alex |
| Offline weights | Queue while the broker is out of reach and publish when it returns | Alex |
| Weight queue | RAM only; a queued weight need not survive a power cycle | Alex |
| Buffered weights | Carry their age and a backlog flag | derived |
| Pressure and weight when not measuring | Publish null, never zero | derived |
| Status topic | Retained, so a late subscriber learns the machine's state | derived |
| Acknowledgement | QoS 0, so an ack can never stall the weighing loop | Alex |
| Operator interface | Phone app over MQTT; no buttons on the enclosure | Alex |
| Tractor bus | Listen only. The board transmits nothing, including requests for engine hours | Alex |

A dead converter and a dead loop are different faults with different first checks at the machine,
and are never reported as the same thing.

## Operating

| Decision | Value | Decided by |
|---|---|---|
| Weighing position | One boom mark. Tare, calibrate and weigh at it | Alex |
| Calibration model | Single point against a known load, valid at one boom position | Alex |
| Tare | At the mark before each weighing session. `+` has no sign guard | Alex |
| Calibration store | A stored calibration from a different front end is rejected, forcing a recalibration | Alex |
| Filter on a fault | Frozen; the last reading is held, never smoothed toward the fault | Alex |
| Commissioning | Alex tests. No session writes or revises a commissioning test | Alex |

## See also

`02` §*Parts* for the transmitter, the supply and the burden as fitted.
`03` §*Sensor states* for the six values and what each one means.
