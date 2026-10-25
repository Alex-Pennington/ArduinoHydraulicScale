# Known Defects — PLC #4 Tractor Scale

**Document:** 05 — Known Defects
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

What is known to be wrong with this machine, and what to do about each one until it is fixed.

## Open

### D4 — can_err can miss a driver error event

The CAN driver reports an error event only when the event carries exactly one flag. An event with
more than one flag set, such as a lost mailbox message together with an error, is not counted in
`can_err` [1].

**Do not read `can_err` 0 and `ring_full` 0 as proof that no frame was lost.** They prove only that
the board's own receive ring never filled.

Tracked as issue 53 [1].

### D5 — torque_pct can read -125 with the engine stopped

An engine torque byte of 0 decodes as -125 % and enters both `torque_pct` and the
`torque_pct_min`/`_max`/`_avg` window, whatever the engine speed [2]. A frame the ECU sends with a
torque byte of 0 at 0 rpm, such as at power-up, publishes `torque_pct` -125 and pulls
`torque_pct_min` to -125 for that window.

**Disregard a `torque_pct` or `torque_pct_min` of -125 in a window where `rpm` is 0 or absent.**

## Standing hazards

| Hazard | What to do |
|---|---|
| `+` records a negative weight when the tare is stale [3] | Tare at the boom mark, bucket empty, before each weighing session |
| The full-scale setting `sm` after a reset, `06` §*What survives a reset* | Set it again after every reset, or leave it at the built-in 3000 psi |

## See also

`03` §*Staleness is a separate question* for what a consumer must read.

## References

[1] A. Pennington, "can_err can miss a driver error event whose event word carries more than one bit," GitHub issue #53, *greybeard-plc4-tractor-scale*. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale/issues/53

[2] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_decode.h`, lines 338–350, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 1902–1918, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
