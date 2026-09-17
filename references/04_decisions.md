# Decisions — PLC #4 Tractor Scale
## Decision Record

PHOENIX NEST LLC · PLC #4
**Document:** 04 — Decisions
**Firmware:** EEPROM signature PL4B
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
| Circuit pressure | Exceeds 3000 psi in service, above the loader plate figure | Alex |

The burden was 150 Ω while the board's own input did the reading, and 250 Ω before that. Both
were correct for the converter of their day and neither is correct now. Fit 68 Ω.

## Scaling

| Decision | Value | Decided by |
|---|---|---|
| Source of truth | Milliamps. Counts are derived at compile time from the burden | Alex |
| Hand-written counts | Not permitted anywhere | Alex |
| Live zero | 4 mA is zero pressure, not zero counts | derived |
| Fault banding | Live-zero convention: dead below 3.5 mA, fault at 21 mA and above | Alex |

Change the burden and every threshold follows from it. A count written by hand is a number that
stops being true the moment the hardware moves and says nothing when it does.

## Reporting

| Decision | Value | Decided by |
|---|---|---|
| Sensor states on the wire | Six values, fixed. A consumer schema depends on them | Alex |
| Staleness | A separate field, not a seventh sensor state | Alex |
| Pressure when not measuring | Publishes null, never zero | Alex |
| Buffered weights | Carry their age and a backlog flag | Alex |
| Status topic | Retained, so a late subscriber learns the machine's state | derived |

A dead converter and a dead loop are different faults with different first checks at the machine,
and are never reported as the same thing.

## Operating

| Decision | Value | Decided by |
|---|---|---|
| Weighing position | One boom mark. Tare, calibrate and weigh at it | Alex |
| Calibration store | Survives a power cycle, keyed to a signature | Alex |
| Recorded tally | Survives a power cycle | Alex |
| Full-scale setting | Not stored. Set it again after every reset | Alex |
| Commissioning | Alex tests. No session writes or revises a commissioning test | Alex |

## See also

`02` §*Parts* for the transmitter, the supply and the burden as fitted.
`03` §*Sensor states* for the six values and what each one means.
