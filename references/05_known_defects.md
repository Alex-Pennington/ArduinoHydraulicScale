# Known Defects — PLC #4 Tractor Scale
## Defect Register

PHOENIX NEST LLC · PLC #4
**Document:** 05 — Known Defects
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

What is known to be wrong with this machine, and what to do about each one until it is fixed. A
register that cannot say what is open is not a register, so status is the subject here.

## Open

### D1 — A refused zero destroys the undo target

Setting zero is refused when the loop is not healthy, but the value the undo would have restored
is overwritten before the refusal. A later undo then returns nothing and reports success.

**Do not rely on undo after a refused zero.** Set zero again deliberately, with the bucket empty
at the boom mark, rather than undoing back to a previous one.

Tracked as issue 3.

### D2 — The live stream carries a weight during a fault

Telemetry publishes a weight field on every cycle, including while the reading is held rather
than current. The sensor state and the staleness flag beside it both say so, but the number is
still on the wire.

**A consumer must read the sensor state and the staleness flag, not the weight alone.** A weight
accompanied by anything other than a healthy state and a false staleness flag is not a live
reading. Recorded weights are unaffected: the machine refuses to record one while the reading
cannot be trusted, so a stale weight never reaches the record.

Tracked as issue 5.

## Closed by the current firmware

### D3 — The supply rail was a gain term on every reading

The board's own converter measured against its supply rail, so every pressure reading scaled with
a voltage that was never measured and that moves with load and with the source.

Closed. The converter now carries its own voltage reference and the supply rail no longer enters
the measurement.

Tracked as issue 4.

## Standing hazards, not defects

These are properties of the build. Nothing is broken and nothing is being fixed; they are here
because each one has cost time at the machine.

| Hazard | What to do |
|---|---|
| A burden of the wrong value reads plausibly and clips silently | Fit 68 Ω. Larger values clip below full scale and can exceed what the converter tolerates |
| A bus scan finding nothing means the trunk or the multiplexer | The converter and the display fall silent together and neither is at fault |
| The full-scale setting is not stored | Set it again after every reset, or the scale reads wrong with no sign on the display |
| A held reading looks steadier than a live one | The filter freezes on a fault, so a stale number is rock-steady rather than drifting |

## See also

`02` §*Connections* for the burden and where it belongs in the loop.
`03` §*Staleness is a separate question* for what a consumer must read.
