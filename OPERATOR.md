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
- **`NO SENSOR`** — the transducer reads below its live zero. Open circuit, cut or
  chafed wire, or a dead sensor. **This is not a zero reading; it is a fault.** The
  scale cannot weigh until it is fixed.
- **`SENSOR SHORT`** — the input is near the supply rail. Shorted wiring.
- `Buffer Full` — the 50-entry tally is full. Use `sum`, then `b`.

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
