# Operating Card — PLC #4 Tractor Scale

**Document:** 07 — Operating Card
**Firmware:** build `699e3c6c`

How the scale is operated: the weighing procedure, the verbs that drive it, and what the
machine does when it will not do as it is asked. Every operator action arrives over MQTT from
the phone app. The console, over USB or the radio module, takes the same verbs and is the
service path that works with no broker in reach.

## Weighing

**All three steps happen at the boom mark**, which is marked on the machine so the position
can be reproduced.

| Step | Verb | Condition |
|---|---|---|
| 1. Tare | `tare` | Boom at the mark, bucket empty |
| 2. Span | `kv` | Boom at the mark, carrying the calibration load |
| 3. Weigh and record | `+` | Boom at the mark |

The calibration load is 530 lb, the figure `kv` calibrates against.

Boom position changes the unloaded pressure as well as the loaded pressure. A tare taken at one
position and a weight read at another differ by a constant offset that enters every reading.

The tare is stored and applied at every power-up without being announced, and it goes stale as
the circuit loses pressure while the machine stands. Tare at the mark before each weighing
session. A negative weight with the bucket empty at the mark means the tare is stale; `+` records
it anyway.

## Commands

| Verb | Does | From the app |
|---|---|---|
| `tare` | Set the tare from the current reading. Stored. | yes |
| `kv` | Calibrate the span against the calibration load. Stored. | yes |
| `+` | Record the current weight to the tally and publish it | yes |
| `-` | Remove the last recorded entry | yes |
| `b` | Clear the tally, immediately and without confirmation | yes |
| `c<value>` | Set the calibration factor directly, above 0 and up to 1000. Stored. | yes |
| `w<value>` | Set the smoothing factor, 0 up to but not including 1. Stored. | yes |
| `sm<value>` | Set the transducer full-scale pressure in psi, 1 to 20000. Not stored. | yes |
| `sum` | List the recorded entries and their total | console |
| `read` | Print the current weight | console |
| `net` | Print link, queue, build and command counters | console |
| `i2c` | Scan the bus trunk and every multiplexer channel, and name what answers | console |
| `pwr` | Report the voltage and current monitor | console |
| `can` | Report the CAN link, its counters and the decoded engine values | console |
| `canf` | Toggle the CAN hardware filter and reopen the controller. Off at every boot. | console |
| `debug` | Toggle the diagnostic dump `read` prints. Off at every boot. | console |
| `hang` | Stop the board deliberately, to prove the watchdog resets it | console |

`hang` is refused from the app by design: stopping a machine that is holding a load, from a
phone, is a denial of service rather than a diagnostic. `read`, `net` and `sum` are refused
because the telemetry and status topics already carry what they report. Every other console verb
is refused from the app as well.

For the message shapes these verbs travel in, see `03` §*Command* and `03` §*Acknowledgement*.

## When an action is refused

`tare`, `+` and `kv` require the loop to be measuring within its band and the converter to be
answering. The refusal names the state that caused it, on the console and in the app alike. A
reading pinned at the end of its range is no basis for a new tare or span, so it is refused even
though it is a real measurement — see `03` §*Sensor states* for that distinction.

`+` is also refused with the tally full, `-` with it empty, and `kv` when the load is not above
the tare. `c`, `w` and `sm` refuse a missing, non-numeric or out-of-range value.

A refused command is still answered. It comes back marked as refused, with the reason.

## The full-scale setting does not persist

**`sm` reverts to 3000 psi at every reset.** It is the only setting that looks permanent and is
not. With a transducer of another range fitted, a silent revert leaves the scale working and
reading wrong, with nothing on the display to say so. For a transducer that stays on the
machine the range belongs in the firmware, where it survives a reset without anyone remembering.

## The tare cannot be stepped back

There is no undo. If the tare needs correcting, `tare` again at the mark.

## What the display asks for

The display is a read-only readout behind a clear coverplate; nothing on the machine responds
to a touch. Row 4 carries one fault at a time, worst first, and stays visible for as long as it
is true.

Two of its messages look alike and lead to different first checks:

- **`NO SENSOR`** — the current loop itself is dead. Check the transmitter's wiring and the loop
  supply.
- **`NO ADC`** — the board cannot reach the converter that reads that loop. Check the bus cable
  first. It appears about two seconds after the board stops hearing from the converter;
  until then the last good reading sits unmoving, which is what a loose connector looks like.

Either blanks the weight, and `tare`, `+` and `kv` are refused while one shows. So does
`XMTR FAULT`, a loop current at or above the fault threshold.

`CAL NOT SET!` shows when no valid stored calibration was found at power-up, and stands until
`kv` succeeds. A flash that changes the calibration layout produces it on purpose: a stored
calibration from a different front end would restore cleanly and read wrong. Tare and calibrate
at the mark before trusting the scale.

`I2C ERR` with a count means converter reads have failed since power-on. It shows only when
nothing worse does, and clears at the next reset.

## The tally

The tally holds 50 entries. With five or fewer slots left the display counts them down; at 50
it reads `TALLY FULL` and `+` stops recording. Use `sum`, then `b`, before recording again.

## Out of radio range

The scale weighs normally with no network. Recorded weights are held and published when the
broker is reachable again; row 3 shows how many are waiting, and `net` reports it. The board holds
200; beyond that the oldest are dropped, and row 3 shows `DROPPED` with the count. Published
backlog is marked as such and carries its age, so it cannot be read as a live weight.

## What survives a reset

| | Survives |
|---|---|
| The tare | yes, stored |
| The calibration factor and the smoothing factor | yes, stored |
| The recorded tally | yes, stored as it is recorded |
| Weights queued for upload | no |
| The `sm` full-scale setting | no |
| `debug` and the `canf` filter | no, both off |

A reset costs the uploads that had not gone out, not the recorded numbers. `sum` is still
right. If the queue matters, get back in range before powering down.

## See also

`01` §*Console* for the console's ports and rates, and `03` §*Command* for the contract the app
speaks.
