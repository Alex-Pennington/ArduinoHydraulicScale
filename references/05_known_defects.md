# Known Defects — PLC #4 Tractor Scale

**Document:** 05 — Known Defects
**Firmware:** build `699e3c6c`

What is known to be wrong with this machine, and what to do about each one until it is fixed.

## Open

### D4 — can_err can miss a driver error event

The CAN driver reports an error event only when the event carries exactly one flag. An event with
more than one flag set, such as a lost mailbox message together with an error, is not counted in
`can_err`.

**Do not read `can_err` 0 and `ring_full` 0 as proof that no frame was lost.** They prove only that
the board's own receive ring never filled.

Tracked as issue 53.

### D5 — torque_pct can read -125 with the engine stopped

In build `699e3c6c` an engine torque byte of 0 decodes as -125 % and enters both `torque_pct` and
the `torque_pct_min`/`_max`/`_avg` window, whatever the engine speed. A frame the ECU sends with a
torque byte of 0 at 0 rpm, such as at power-up, therefore publishes `torque_pct` -125 and pulls
`torque_pct_min` to -125 for that window.

**Disregard a `torque_pct` or `torque_pct_min` of -125 in a window where `rpm` is 0 or absent.**

Tracked as issue 57.

## Closed in this build

### D2 — Telemetry weight during a fault

Telemetry `w` publishes null whenever `psi` does: when the loop is not measuring or the converter
has stopped answering.

Tracked as issue 5.

### D3 — Supply rail as a gain term

The converter carries its own voltage reference, so the board's supply rail does not enter the
measurement.

Tracked as issue 4.

## Standing hazards, not defects

Properties of the build. Nothing is broken and nothing is being fixed.

| Hazard | What to do |
|---|---|
| `+` records a negative weight when the tare is stale | Tare at the boom mark, bucket empty, before each weighing session |
| The full-scale setting `sm` is not stored | Set it again after every reset, or leave it at the built-in 3000 psi |
| The CAN hardware filter has not been proven on the board | If engine values stop arriving with `canf` on, turn it off again with `canf` |

## See also

`07` §*The full-scale setting does not persist* for what a lost `sm` does to the reading.
`03` §*Staleness is a separate question* for what a consumer must read.
