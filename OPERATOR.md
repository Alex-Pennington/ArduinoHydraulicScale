# PLC #4 — operator notes

Tractor-mounted hydraulic loader scale, John Deere 5055E. The board announces itself
as `JD5055e OK` on the Bluetooth console at power-up.

## Weighing procedure — all three steps at the boom mark

The boom is physically marked so the position can be reproduced. **Every step below
must be done at that mark.**

1. **`tare`** — boom at the mark, bucket empty.
2. **`kv`** — boom at the mark, with the known calibration load (`calWeight`, currently 530).
3. **Weigh** — boom at the mark.

**Why all three, not just the calibration.** Boom position changes the unloaded
pressure as well as the loaded pressure, because the boom's own weight acts through
the same moment arm. A tare taken with the boom resting on the ground and a weigh
taken at the mark differ by the boom's own contribution, and that difference lands
in every reading as a constant offset. Zero and span have to be established at the
same position or they disagree.

**The mark is not optional.** The zero is stored and restored across power cycles, so
the board silently applies the last stored zero at every boot. That is correct only
while the position it was taken at is reproducible. If the mark is lost, painted over
or ignored, the stored zero becomes wrong without any indication that it is — so
re-tare at the mark rather than trusting a zero of unknown provenance.

## Commands

Every command in this table runs from the console — USB, or Bluetooth (HC-05, 9600 baud).
The ones marked **yes** in the MQTT column also run from the phone app; see *Phone app
(MQTT)* below. The console is the only path for the rest, and the only one that works with
no broker in range.

| Command | Does | Over MQTT |
|---|---|---|
| `tare` | Set zero from the current reading. Stored. **At the mark, empty.** | yes |
| `kv` | Calibrate span against `calWeight`. Stored. **At the mark, with the load.** Refuses if at or below tare. | yes |
| `+` | Record the current weight to the tally, and publish it | yes |
| `-` | Remove the last recorded entry | yes |
| `sum` | Total the recorded entries | console only |
| `b` | Clear the tally — immediately, no confirmation, over either path | yes |
| `read` | Print the current weight | console only |
| `net` | Print WiFi, MQTT, IP, queued and dropped counts, sensor state, and the MQTT command counters | console only |
| `debug` | Toggle verbose output. Not stored — off at every boot. | console only |
| `c<value>` | Set the calibration factor directly. Stored. Rejected if out of range. | yes |
| `w<value>` | Set the smoothing factor, 0 to 1. Stored. | yes |
| `sm<value>` | Set the transducer full-scale pressure in psi. **NOT stored — reverts to 3000 on every reset.** | yes |

### Service commands — not for the seat

These exist for diagnosing the board and are typed at the console. Neither is on the MQTT
allowlist, so neither reaches the board from the app.

| Command | Does |
|---|---|
| `hang` | **Deliberately stops the board.** It stops answering, the watchdog resets it after about 35 seconds, and it comes back reporting `WATCHDOG`. It exists to prove the watchdog still works. Nothing is lost — the tally and calibration are stored — but anything queued for upload goes. **Refused over MQTT** — stopping a machine that is holding a load, from a phone, is a denial of service rather than a diagnostic. |

**Removed 2026-09-18: the `keys` command.** It reported the raw keypad value and decoded
button, to check the button ladder against a replacement shield. There is no keypad left to
check, and the command is gone from the firmware with it.

**Note the renames:** `cal` became `kv` and `clear` became `b` on this firmware line.
The Bluetooth terminal's macro set in `serial_bluetooth_terminal_cfg.txt` (not the MQTT
app above) has been reissued to match — reload it on the phone, or two of the buttons in
that terminal app will do nothing.

**`tare`, `+` and `kv` all refuse unless the loop reads `ok`** — including when it
reads `SATURATED`. The refusal names the state. A reading pinned at full scale
understates the load, so it is not a safe basis for a new zero or span.

**If you ever change `sm`, you must set it again after every reset.** It is the only
setting that looks permanent and is not. If a transducer with a range other than the
built-in default is fitted and `sm` silently reverts, the scale keeps working and reads
wrong — with nothing on the display to say so.

For a sensor that is staying on the machine, do not rely on `sm` at all: the default
belongs in the firmware, so it survives every reset without anyone remembering.

## Phone app (MQTT)

**Added 2026-09-18.** The app sends `{"id":"<unique>","verb":"<verb>"}` — add
`"arg":"<value>"` too for `c`, `w` and `sm` — to `plc4/jd5055e/cmd`, and the board answers
on `plc4/jd5055e/ack`:

    {"id":"7f3a9c21","verb":"+","ok":true,"detail":"recorded","n":12,"total":19408}
    {"id":"7f3a9c22","verb":"tare","ok":false,"detail":"UNDER ZERO"}
    {"id":"7f3a9c21","verb":"+","ok":true,"detail":"recorded","dup":true}

A refusal's `detail` is the same state word the console and the LCD would show, so the app
and the machine can never disagree about why something was refused. A repeated `id` is
acked with `"dup":true` and not re-run, so retrying a command whose ack never arrived is
always safe. A command sent as a retained MQTT message is dropped and never acked — it
would otherwise re-run on every reconnect — and `net`'s `cmd_retained` count is how you'd
notice that happening.

Only the commands marked **yes** in *Commands* above reach the board this way. `hang` is
refused regardless — see *Service commands* above — and `read`, `net`, `sum` and `debug`
are refused too: the app already gets that data from the telemetry and status topics, so
there is nothing for those verbs to do remotely. A refused command still gets an ack, with
`"ok":false` and a `detail` naming why.

## Operator input

**Updated 2026-09-18: the five physical buttons are gone**, and with them the menu, the
confirm screen and the check-cal screen — all three were reachable only by button. The LCD
is now a read-only readout behind a clear coverplate; nothing on the machine responds to a
touch. Every action that used to be a key press is now a command from the phone app, over
MQTT — see *Phone app (MQTT)* and the MQTT column in *Commands*, both above. The console
(USB or Bluetooth) still takes every command it always did, as the service and recovery
path.

**UNDO TARE has no replacement.** It existed only in the button menu, never had a console
verb, and is not on the MQTT allowlist. If the zero needs correcting, `tare` again at the
mark — there is no way to step back to the previous one.

## What the display tells you

**The display shows one screen, and row 4 is a permanent status row on it.**
There is no menu and no calibration-check screen to navigate away to, so a
fault stays visible for as long as it is true. Row 4 shows worst first, one
thing at a time, in this order: the loop or ADC state (`NO SENSOR`,
`XMTR FAULT`, or `NO ADC`), then `CAL NOT SET!`, then an I2C error count, and
otherwise blank. If two of these are true at once, you only see the higher
one on the list; clearing it is what reveals the next.

- A number — the current weight.
- **`NO SENSOR`** — dead loop, below 3.5 mA: cut wire, no loop supply, or the
  transmitter removed. **This is not a zero reading; it is a fault.** Blanks the
  weight; the scale cannot weigh until it is fixed.
- **`XMTR FAULT`** — loop at or above 21 mA (NAMUR NE43). The transmitter is not
  giving a usable reading. Blanks the weight.
- **`NO ADC`** (new 2026-09-16) — the board has lost contact with the MCP3421
  chip that reads the loop, most likely a loose Qwiic connector. **This is not
  the same problem as `NO SENSOR`, and the first check at the machine is
  different for each:** `NO SENSOR` means the 4-20 mA loop itself is dead —
  check the transducer's wiring and loop supply. `NO ADC` means the board
  cannot talk to the chip that *reads* that loop — check the Qwiic cable
  first. It appears about 2 seconds after the board stops hearing from the
  chip; until then the last good reading just sits there, unmoving, which is
  what a loose connector actually looks like — not a jump to a fault message.
  Blanks the weight, same as `NO SENSOR`, and `tare`, `+` and calibration are
  all refused while it shows.
- **`BAD STATE`** — should never appear. It means a corrupted record, not a live
  fault. Report it if you see it.
- **`CAL NOT SET!`** (row 4, the status row) — shown after every flash, until the
  scale is tared and calibrated. Deliberate, not a fault: the stored calibration
  belongs to the old voltage front end and is wrong by about 1.67x, so the
  firmware refuses to use it rather than silently weighing wrong.
  **Updated 2026-09-16:** the same refusal fires again on this branch, for the
  same reason. The MCP3421 front end moved counts-per-psi again — the previous
  4-20 mA signature's 7864-count span became 4352 — so a calibration stored under
  that previous signature would restore cleanly and read wrong by roughly 1.8x.
  Tare and `kv` at the boom mark before trusting the scale after any flash that
  changes this message.
- **an I2C error count** (row 4, the status row) — `I2C ERR` followed by a
  count, shown once calibration is set if the Qwiic bus to the ADC has logged a
  failure. It does not blank the weight; treat a rising count as the warning to
  act on before the bus fails outright.
- **`TALLY n LEFT`** (new 2026-09-16, row 3, just above the status row) —
  shown once 5 or fewer tally slots remain, so you get warning before you hit
  the wall. At 50 recorded entries it changes to **`TALLY FULL`** and `+`
  stops recording — `sum` the tally, then `b` to clear it, before you can
  record another.
- `Buffer Full` — the one-time message on sending `+` after the tally is
  already full (the `TALLY FULL` line above is the standing warning; this is
  what you see if you send `+` anyway). Use `sum`, then `b`.

**Removed 2026-09-18: `KEY STUCK`.** It reported a button held past 10 seconds. There is no
keypad left to stick, and the message is gone from the firmware with it.

**`SATURATED` and `UNDER ZERO` never reach the LCD, and the status row does not
blank the weight for either of them.** Both are still real measurements — pinned
at full scale, or below the 4 mA live zero — and the weight stays on screen
through both, on purpose: blanking it because the transmitter was pinned was the
old defect this front end exists to fix. Don't wait for a display that is never
coming; `net` and the MQTT telemetry both carry the state if you need to know
which of the two you're in.

## Working out of radio range

The scale works normally with no network. Recorded weights (`+`) are held in memory
and published when the broker comes back into range — `net` shows how many are
waiting. Published backlog is marked as such with its age, so it cannot be mistaken
for a live reading.

## What survives a reset, and what does not

**Corrected 2026-09-01.** An earlier version of this page said the recorded weights
stayed in the tally through a power cycle. That was wrong at the time and has now been
made true: the tally is written to permanent memory as you record it.

**Removed 2026-09-16.** This page used to carry a second, duplicated copy of itself, ending
in a section headed "NOTHING YOU RECORD SURVIVES A POWER CYCLE." That section described the
firmware as it was before the tally was made persistent, and it contradicted the table below.
It has been deleted; it is in the repository history if anyone needs it (issue #2).
**If you are holding a printout that tells you your tally is lost, it is out of date — throw
it away and print this page again.**

| what | survives a reset or power cycle? |
|---|---|
| `tare` zero | **yes** — stored |
| `kv` calibration | **yes** — stored |
| the recorded tally (`+`, `sum`) | **yes** — stored as you go |
| weights queued for upload while out of range | **NO — lost** |
| `sm` full-scale setting | **NO — reverts to 3000 psi** |

So a reset costs you the *uploads* that had not gone out yet, not the numbers
themselves. `sum` will still be right. If the queue mattered, get back in radio range
before keying off — `net` and the display both show how many are waiting.

**If you ever change `sm`, set it again after every reset.** It is the only setting
that looks permanent and is not. With a transducer of a range other than 3000 psi
fitted, a silent revert leaves the scale working and reading wrong.

## Mind the RESET button

**There is still a RESET button, easy to catch with a glove, and it restarts the board.**
The five operator keys it used to sit alongside are gone, but the RESET button is not one
of them and it is still there.

That is survivable — the tally is kept — but it is not free: anything still queued for
upload is lost, and the scale takes a few seconds to come back and re-associate. If you
keep catching it, tape over it.
