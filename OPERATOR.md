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

Sent over the Bluetooth console (HC-05, 9600 baud), or from the LCD keypad.

| Command | Does |
|---|---|
| `tare` | Set zero from the current reading. Stored. **At the mark, empty.** |
| `kv` | Calibrate span against `calWeight`. Stored. **At the mark, with the load.** Refuses if at or below tare. |
| `+` | Record the current weight to the tally, and publish it |
| `-` | Remove the last recorded entry |
| `sum` | Total the recorded entries |
| `b` | Clear the tally |
| `read` | Print the current weight |
| `net` | Print WiFi, MQTT, IP, queued and dropped counts, and sensor state |
| `debug` | Toggle verbose output. Not stored — off at every boot. |
| `c<value>` | Set the calibration factor directly. Stored. Rejected if out of range. |
| `w<value>` | Set the smoothing factor, 0 to 1. Stored. |
| `sm<value>` | Set the transducer full-scale pressure in psi. **NOT stored — reverts to 3000 on every reset.** |

### Service commands — not for the seat

These exist for diagnosing the board and are typed, never on the keypad.

| Command | Does |
|---|---|
| `keys` | Reports the raw keypad value and decoded button for 15 s. Use it to check the button ladder against a replacement shield. |
| `hang` | **Deliberately stops the board.** It stops answering, the watchdog resets it after about 35 seconds, and it comes back reporting `WATCHDOG`. It exists to prove the watchdog still works. Nothing is lost — the tally and calibration are stored — but anything queued for upload goes. |

**Note the renames:** `cal` became `kv` and `clear` became `b` on this firmware line.
The Bluetooth macro set in `serial_bluetooth_terminal_cfg.txt` has been reissued to
match — reload it on the phone, or two of the buttons will do nothing.

**`tare`, `+` and `kv` all refuse unless the loop reads `ok`** — including when it
reads `SATURATED`. The refusal names the state. A reading pinned at full scale
understates the load, so it is not a safe basis for a new zero or span.

**If you ever change `sm`, you must set it again after every reset.** It is the only
setting that looks permanent and is not. If a transducer with a range other than the
built-in default is fitted and `sm` silently reverts, the scale keeps working and reads
wrong — with nothing on the display to say so.

For a sensor that is staying on the machine, do not rely on `sm` at all: the default
belongs in the firmware, so it survives every reset without anyone remembering.

## Keypad

| Key | Command |
|---|---|
| Right | `+` |
| Up | `sum` |
| Down | `b` |
| Left | `-` |
| Select | `tare` |

## What the display tells you

- A number — the current weight.
- **`NO SENSOR`** — dead loop, below 3.5 mA: cut wire, no loop supply, or the
  transmitter removed. **This is not a zero reading; it is a fault.** Blanks the
  weight; the scale cannot weigh until it is fixed.
- **`XMTR FAULT`** — loop at or above 21 mA (NAMUR NE43). The transmitter is not
  giving a usable reading. Blanks the weight.
- **`BAD STATE`** — should never appear. It means a corrupted record, not a live
  fault. Report it if you see it.
- **`CAL NOT SET!`** (row 1) — shown after every flash, until the scale is tared
  and calibrated. Deliberate, not a fault: the stored calibration belongs to the
  old voltage front end and is wrong by about 1.67x, so the firmware refuses to
  use it rather than silently weighing wrong.
- `Buffer Full` — the 50-entry tally is full. Use `sum`, then `b`.

**`SATURATED` and `UNDER ZERO` never reach the LCD.** Both are still real
measurements — pinned at full scale, or below the 4 mA live zero — and the weight
stays on screen through both, on purpose: blanking it because the transmitter was
pinned was the old defect this front end exists to fix. Don't wait for a display
that is never coming; `net` and the MQTT telemetry both carry the state if you
need to know which of the two you're in.

## Working out of radio range

The scale works normally with no network. Recorded weights (`+`) are held in memory
and published when the broker comes back into range — `net` shows how many are
waiting. Published backlog is marked as such with its age, so it cannot be mistaken
for a live reading.

## What survives a reset, and what does not

**Corrected 2026-09-01.** An earlier version of this page said the recorded weights
stayed in the tally through a power cycle. That was wrong at the time and has now been
made true: the tally is written to permanent memory as you record it.

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

The LCD shield has a **RESET button in the same row as the five operator keys.** It is
easy to catch with a glove, and it restarts the board.

That is now survivable — the tally is kept — but it is not free: anything still queued
for upload is lost, and the scale takes a few seconds to come back and re-associate.
If you keep catching it, tape over it.

## Commands

Sent over the Bluetooth console (HC-05, 9600 baud), or from the LCD keypad.

| Command | Does |
|---|---|
| `tare` | Set zero from the current reading. Stored. **At the mark, empty.** |
| `kv` | Calibrate span against `calWeight`. Stored. **At the mark, with the load.** Refuses if at or below tare. |
| `+` | Record the current weight to the tally, and publish it |
| `-` | Remove the last recorded entry |
| `sum` | Total the recorded entries |
| `b` | Clear the tally |
| `read` | Print the current weight |
| `net` | Print WiFi, MQTT, IP, queued and dropped counts, and sensor state |
| `debug` | Toggle verbose output. Not stored — off at every boot. |
| `c<value>` | Set the calibration factor directly. Stored. Rejected if out of range. |
| `w<value>` | Set the smoothing factor, 0 to 1. Stored. |
| `sm<value>` | Set the transducer full-scale pressure in psi. **NOT stored — reverts to 3000 on every reset.** |

**Note the renames:** `cal` became `kv` and `clear` became `b` on this firmware line.
The Bluetooth macro set in `serial_bluetooth_terminal_cfg.txt` has been reissued to
match — reload it on the phone, or two of the buttons will do nothing.

**If you ever change `sm`, you must set it again after every reset.** It is the only
setting that looks permanent and is not. If a transducer with a range other than 3000 psi
is fitted and `sm` silently reverts, the scale keeps working and reads wrong — with
nothing on the display to say so.

## Keypad

| Key | Command |
|---|---|
| Right | `+` |
| Up | `sum` |
| Down | `b` |
| Left | `-` |
| Select | `tare` |

## What the display tells you

- A number — the current weight.
- **`NO SENSOR`** — dead loop, below 3.5 mA: cut wire, no loop supply, or the
  transmitter removed. **This is not a zero reading; it is a fault.** Blanks the
  weight; the scale cannot weigh until it is fixed.
- **`XMTR FAULT`** — loop at or above 21 mA (NAMUR NE43). The transmitter is not
  giving a usable reading. Blanks the weight.
- **`BAD STATE`** — should never appear. It means a corrupted record, not a live
  fault. Report it if you see it.
- `Buffer Full` — the 50-entry tally is full. Use `sum`, then `b`.

**`SATURATED` and `UNDER ZERO` never reach the LCD.** Both are still real
measurements — pinned at full scale, or below the 4 mA live zero — and the weight
stays on screen through both, on purpose: blanking it because the transmitter was
pinned was the old defect this front end exists to fix. Don't wait for a display
that is never coming; `net` and the MQTT telemetry both carry the state if you
need to know which of the two you're in.

## Working out of radio range

The scale works normally with no network. Recorded weights (`+`) are held in memory
and published when the broker comes back into range — `net` shows how many are
waiting. Published backlog is marked as such with its age, so it cannot be mistaken
for a live reading.

## NOTHING YOU RECORD SURVIVES A POWER CYCLE

**Read this before you key off.**

An earlier version of this page said the recorded weights stay in the tally through a
power cycle and that only the upload was lost. **That was wrong.** Both the tally and
the queued uploads live in RAM:

| what | survives a reset or power cycle? |
|---|---|
| `tare` zero | **yes** — stored |
| `kv` calibration | **yes** — stored |
| the recorded tally (`+`, `sum`) | **NO — lost** |
| weights queued for upload while out of range | **NO — lost** |
| `sm` full-scale setting | **NO — reverts to 3000 psi** |

**So: get your weights off the machine before you key off.** Either `sum` them and write
the total down, or be in radio range long enough for the queue to publish — `net` shows
how many are still waiting. Once it queues to zero, they are gone from the machine and
safe on the broker.

Anything that resets the board does this, not just turning the key. A power blip, a loose
connector, or a firmware fault all clear the tally the same way and without warning.
