# Operating Card — PLC #4 Tractor Scale

## Operating Card

PHOENIX NEST LLC · PLC #4
**Document:** 07 — Operating Card
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

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
| 3. Weigh | — | Boom at the mark |

The calibration load is 530, the figure the firmware calibrates against.

Boom position changes the unloaded pressure as well as the loaded pressure, because the boom's
own weight acts through the same moment arm. A tare taken at one position and a weight read at
another differ by the boom's contribution, and that difference enters every reading as a
constant offset. Tare and span are established at one position or they disagree.

The tare is stored and restored at every power-up, so the board applies the last stored tare
without announcing it. That is correct only while the position it was taken at is reproducible.
Re-tare at the mark rather than trusting a tare of unknown origin.

## Commands

| Verb | Does | From the app |
|---|---|---|
| `tare` | Set the tare from the current reading. Stored. | yes |
| `kv` | Calibrate the span against the calibration load. Stored. | yes |
| `+` | Record the current weight to the tally and publish it | yes |
| `-` | Remove the last recorded entry | yes |
| `b` | Clear the tally, immediately and without confirmation | yes |
| `c<value>` | Set the calibration factor directly. Stored. | yes |
| `w<value>` | Set the smoothing factor, 0 to 1. Stored. | yes |
| `sm<value>` | Set the transducer full-scale pressure in psi | yes |
| `sum` | List the recorded entries and their total | console |
| `read` | Print the current weight | console |
| `net` | Print link, queue and command counters | console |
| `i2c` | Scan the bus and name what answers | console |
| `pwr` | Report the voltage and current monitor | console |
| `debug` | Toggle verbose output. Off at every boot. | console |
| `hang` | Stop the board deliberately, to prove the watchdog resets it | console |

`hang` is refused from the app by design: stopping a machine that is holding a load, from a
phone, is a denial of service rather than a diagnostic. `read`, `net`, `sum` and `debug` are
refused from the app because the telemetry and status topics already carry what they report.

For the message shapes these verbs travel in, see `03` §*Command* and `03` §*Acknowledgement*.

## When an action is refused

`tare`, `+` and `kv` require the loop to be measuring within its band. The refusal names the
state that caused it, and the app, the console and the display all name it the same way. A
reading pinned at the end of its range is no basis for a new tare or span, so it is refused
even though it is a real measurement — see `03` §*Sensor states* for that distinction.

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

- **No sensor** — the current loop itself is dead. Check the transmitter's wiring and the loop
  supply.
- **No converter** — the board cannot reach the chip that reads that loop. Check the bus cable
  first. It appears about two seconds after the board stops hearing from the converter; until
  then the last good reading sits unmoving, which is what a loose connector looks like.

Either blanks the weight, and `tare`, `+` and `kv` are refused while one shows.

`CAL NOT SET!` stands until the scale is tared and calibrated. It follows any flash that moves
the counts-per-psi relationship, and it is deliberate: a stored calibration from a different
front end would restore cleanly and read wrong. Tare and calibrate at the mark before trusting
the scale after such a flash.

## The tally

The tally holds 50 entries. With five or fewer slots left the display counts them down; at 50
it reads `TALLY FULL` and `+` stops recording. Use `sum`, then `b`, before recording again.

## Out of radio range

The scale weighs normally with no network. Recorded weights are held and published when the
broker is reachable again, and `net` reports how many are waiting. Published backlog is marked
as such and carries its age, so it cannot be read as a live weight.

## What survives a reset

| | Survives |
|---|---|
| The tare | yes, stored |
| The calibration | yes, stored |
| The recorded tally | yes, stored as it is recorded |
| Weights queued for upload | no |
| The `sm` full-scale setting | no |

A reset costs the uploads that had not gone out, not the recorded numbers. `sum` is still
right. If the queue matters, get back in range before keying off.

## Mind the reset button

**The reset button is easy to catch with a glove and it restarts the board.** The tally is kept,
but anything queued for upload is lost and the scale takes a few seconds to come back and
re-associate. If it keeps being caught, tape over it.

## See also

`01` §*Console* for the console's ports and rates, and `03` §*Command* for the contract the app
speaks.
