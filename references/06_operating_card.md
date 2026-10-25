# Operating Card — PLC #4 Tractor Scale

**Document:** 06 — Operating Card
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

How the scale is operated: the weighing procedure, the verbs that drive it, and what the
machine does when it will not do as it is asked. Every operator action arrives over MQTT from
the phone app. The console, over USB or the radio module, takes the same verbs and is the
service path that works with no broker in reach. Which verbs the app may send is in `03`
§*Command*.

## Weighing

**All three steps happen at the boom mark**, which is marked on the machine so the position
can be reproduced. Boom position changes the unloaded pressure as well as the loaded pressure: a
tare taken at one position and a weight read at another differ by a constant offset in every
reading.

| Step | Verb | Condition |
|---|---|---|
| 1. Tare | `tare` | Boom at the mark, bucket empty |
| 2. Span | `kv` | Boom at the mark, carrying the calibration load |
| 3. Weigh and record | `+` | Boom at the mark |

The calibration load is 530 lb, the figure `kv` calibrates against [1].

The tare is applied at every power-up without being announced, and it goes stale as the circuit
loses pressure while the machine stands. Tare at the mark before each weighing session,
`05` §*Standing hazards*.

## Commands

| Verb | Does |
|---|---|
| `tare` | Set the tare from the current reading |
| `kv` | Calibrate the span against the calibration load |
| `+` | Record the current weight to the tally and publish it |
| `-` | Remove the last recorded entry |
| `b` | Clear the tally, immediately and without confirmation |
| `c<value>` | Set the calibration factor directly, above 0 and up to 1000 [1] |
| `w<value>` | Set the smoothing factor, 0 up to but not including 1 [1] |
| `sm<value>` | Set the transducer full-scale pressure in psi, 1 to 20000 [1] |
| `sum` | List the recorded entries and their total |
| `read` | Print the current weight |
| `net` | Print link, queue, build and command counters |
| `i2c` | Scan the bus trunk and every multiplexer channel, and name what answers. Nothing found means the trunk or the multiplexer, not a device behind it [2] |
| `pwr` | Report the voltage and current monitor |
| `can` | Report the CAN link, its counters and the decoded engine values |
| `canf` | Toggle the CAN hardware filter and reopen the controller |
| `debug` | Toggle the diagnostic dump `read` prints |
| `hang` | Stop the board deliberately, to prove the watchdog resets it |

For the message shapes these verbs travel in, see `03` §*Command* and `03` §*Acknowledgement*.

## When an action is refused

`tare`, `+` and `kv` require the loop to be measuring within its band, state `ok`, and the
converter to be answering. The refusal names the state that caused it, on the console and in the
app alike. A reading at either end of its range is refused, `03` §*Sensor states*.

`+` is also refused with the tally full, `-` with it empty, and `kv` when the load is not above
the tare. `c`, `w` and `sm` refuse a missing, non-numeric or out-of-range value.

A refused command is still answered. It comes back marked as refused, with the reason.

## The tare cannot be stepped back

There is no undo. If the tare needs correcting, `tare` again at the mark.

## What the display asks for

The display is a read-only readout behind a clear coverplate; nothing on the machine responds
to a touch. Row 4 carries one fault at a time, worst first, and stays visible for as long as it
is true, `01D` §*Rows*.

Two of its messages look alike and lead to different first checks:

- **`NO SENSOR`** — the current loop itself is dead. Check the transmitter's wiring and the loop
  supply.
- **`NO ADC`** — the board cannot reach the converter that reads that loop. Check the bus cable
  first. It appears 2 s after the board stops hearing from the converter [1];
  until then the last good reading sits unmoving, which is what a loose connector looks like.

Either blanks the weight, and `tare`, `+` and `kv` are refused while one shows. So does
`XMTR FAULT`, a loop current at or above the fault threshold.

`CAL NOT SET!` shows when no valid stored calibration was found at power-up, and stands until
`kv` succeeds. A flash that changes the calibration layout produces it. Tare and calibrate at the
mark before trusting the scale.

`I2C ERR` with a count means converter reads have failed since the last reset. It shows only
when nothing worse does, and clears at the next reset [3].

## The tally

The tally holds 50 entries [1]. With five or fewer slots left, row 3 counts them down; at 50 it
reads `TALLY FULL` and `+` stops recording. Row 3 shows a dropped or queued weight ahead of
either, `01D` §*Rows*. Use `sum`, then `b`, before recording again.

## Out of radio range

The scale weighs normally with no network. Recorded weights are held and published when the
broker is reachable again, `03` §*Weight*. Row 3 shows how many are waiting, or `DROPPED` with
the count once the queue has overflowed, and `net` reports both.

## What survives a reset

| | Survives |
|---|---|
| The tare | yes, stored [1] |
| The calibration factor | yes, stored [1] |
| The smoothing factor | yes, stored, for a value above 0. `w0` reverts to 0.1 at the next reset [1] |
| The recorded tally | yes, stored as it is recorded [1] |
| Weights queued for upload | no, `03` §*Weight* |
| The `sm` full-scale setting | no; reverts to 3000 psi [1]. A transducer of another range that stays on the machine has its range set in the firmware |
| `debug` and the `canf` filter | no, both off [1], [4] |

`sum` is still right after a reset. If the queue matters, get back in range before powering down.

## See also

`01` §*Header pins* for the console's ports and rates, and `03` §*Command* for the contract the
app speaks.

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 9, 47, 71, 73–78, 144, 1490–1513, 1622–1640, 2088–2131, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[2] A. Pennington, *greybeard-plc4-tractor-scale*, `include/i2c_mux.h`, lines 27–30, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `include/mcp3421.h`, lines 25–26, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[4] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_can.h`, lines 38–40, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
