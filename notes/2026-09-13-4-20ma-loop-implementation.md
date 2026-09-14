# 4-20 mA loop: what was built, what was found, what is left

Recorded 2026-09-13 on `feat/plc4-4-20ma-loop-impl`. Companion to
`notes/2026-09-13-4-20ma-loop-decisions.md` (the hardware decisions and why) and
`MA-LOOP-DESIGN.md` (the firmware design). **This file is the implementation record — read it
instead of a session transcript.**

**THE BRANCH OF RECORD IS `feat/plc4-4-20ma-loop-impl`.** Ruled by Alex 2026-09-13. It contains
every commit from `feat/plc4-4-20ma-loop`, which is a redundant pointer at one of its ancestors —
the work is not split across the two. A fast-forward to unify the names was approved but blocked
by a repo guard that reserves merges for the session owning the repository root; it was left
rather than worked around, because the only thing it would change is a branch name.

**Status: IMPLEMENTED AND COMPILED. NOT FLASHED. NO BOARD HAS BEEN TETHERED.** Every claim about
runtime behaviour here is read from source or from host arithmetic, and says so.

---

## 1. This repo compiles, and that was new

`CLAUDE.md` says nothing here has been compiled or flashed. **That is now stale for the first
half.** Saga carries `arduino-cli` 1.5.0 with `arduino:renesas_uno` 1.6.0, which is what the
estate flashes with. PlatformIO is NOT installed and is not needed despite the tracked
`platformio.ini`.

`tools/build-check.sh` bridges the layout mismatch — the repo is PlatformIO-shaped (`src/*.cpp`)
and `arduino-cli` wants a sketch directory containing `<name>.ino` — by assembling a throwaway
sketch in a scratch directory. **It compiles only. It never opens a port and takes no upload
flag.** Flashing stays Alex's, at the machine.

| | flash | RAM |
|---|---|---|
| baseline, `7e7f2f4` | 85008 (32%) | 11196 (34%) |
| this branch, `36c4226` | 85668 (32%) | 11216 (34%) |

The `library WDT claims to run on renesas architecture(s)` warning is **pre-existing** and appears
on the untouched baseline. Do not chase it.

`test/test_loop_scaling.cpp` runs the scaling arithmetic on the host with plain `g++` — the only
part of this firmware that can be executed anywhere in this workspace. 35 checks, every band edge
on both sides. **It proves arithmetic. It proves nothing about a transmitter or a tractor.**

---

## 2. Both defects the feature existed for are closed

Traced end to end by the final review, from `analogRead` to the wire and the LCD.

**Bottom.** An open loop sources 0 mA, develops 0 V across the 150 ohm burden, and lands at ~0
counts — `LOOP_DEAD`, `psi: null`, `psi_fault: "low"`, `NO SENSOR` on the display, filter frozen,
all three operator verbs refused. The old front end floated near mid-rail at ~4401 counts and
reported `sensor = ok` with a confident 632 psi.

**Top.** A 20.5 mA reading (10075 counts) classifies `LOOP_SATURATED`, increments `overrange`,
keeps feeding the filter, publishes a real numeric `psi` with `psi_fault: null`, keeps the weight
on the display, and still refuses `tare` / `+` / `kv`. The old front end classified the same
excursion as a **wiring fault** and did not even count it.

**The two predicates are the thing most likely to be broken by a later edit:**

    loopState != LOOP_OK        -> REFUSE AN OPERATOR ACTION   (tare, +, kv)
    loopIsMeasuring(loopState)  -> TRUST THE READING           (LCD, psi, filter)

They differ on `LOOP_UNDER` and `LOOP_SATURATED`. The final review checked every use site across
all 1290 lines: none wrong. **Do not collapse them into one.** `loopIsMeasuring` where refusal
belongs lets an operator calibrate against a pinned transmitter; the reverse blanks the weight on
routine over-range, which IS the defect this feature removed.

---

## 3. Alex's decisions, 2026-09-13

**`+` KEEPS REFUSING on `LOOP_SATURATED`.** His words: *"a pinned reading is not a weight."*
Asked rather than assumed, because this branch changed the behaviour without anyone intending it:
under `9e187d2` an over-range sample fell through to `SENSOR_OK` and `+` was **allowed**. Since
he reports the circuit goes over range routinely, `+` will now routinely refuse on a real
weighing. That is the accepted trade.

**FIX the two remaining false-success paths on this branch**, rather than filing them. Both were
pre-existing, and this repo's rule is that baseline defects are written up, not fixed, without a
specific approval — so they were put to him. He chose to fix.

---

## 4. Defects found DURING implementation, and what happened to them

### A regression this branch introduced, caught by a worker reading adjacent code

`queueFlush()` carried a private 3-entry state-name array indexed by the stored capture byte,
clamped `<= 2 ? idx : 0`. It was **correct** against the old 3-value enum. Wiring in the 5-value
`LoopState` broke it:

| captured | replayed as | |
|---|---|---|
| `LOOP_OK` | `ok` | correct |
| `LOOP_DEAD` | `NO SENSOR` | correct by coincidence |
| `LOOP_UNDER` | `SENSOR SHORT` | wrong — a string that must no longer exist |
| `LOOP_SATURATED` | `ok` | **wrong** |
| `LOOP_FAULT_HIGH` | `ok` | **wrong** |

**A weight buffered during a transmitter fault replayed to the broker claiming the sensor was
healthy.** `sensorAtCapture` exists precisely so validity is a historical fact; this turned it
into a fabricated one, and the consumer reads that field to decide whether to store the value.

Fixed (`a8a88d5`) by deleting the array and calling `loopStateName()`, the single source of truth.
`loopStateName()` gained an explicit `LOOP_OK` case and a `default: "BAD STATE"` in the same
change — that arm was cosmetic while only `classifyLoop` fed the function, and became
load-bearing the moment a raw byte from a RAM record did. **It compiled cleanly, `-Wformat` could
not see it, and a grep gate missed it because the defect was the STRING `"SENSOR SHORT"` while
the gate matched the IDENTIFIER `SENSOR_SHORT`.** Two independent sweeps since found no second
instance.

### A Critical that existed only between two files

`hmiPoll()` returns early per screen, and the sensor fault is painted in exactly one place —
`hmiDrawHome()`. So on `SCR_CHECKCAL` a faulted loop showed a held weight with nothing indicating
a problem, no timeout, and only Left or Select to exit.

**Freezing the filter made it worse.** Before that change a dead loop dragged the value visibly
toward the tare; afterwards it sits rock-steady, i.e. more convincing. Failure scenario: loop
supply fuse blows, operator opens Menu -> Check cal, reads `Should be 530 / Reads 528`, and signs
the scale off with the transmitter disconnected.

`MA-LOOP-DESIGN.md` §6.1 justified the freeze with *"The LCD writes the fault over row 0 either
way, so a held value is never presented as current."* **That sentence was false.** Fixed in
`3384221`, and the spec sentence corrected with it.

**The architectural lesson, which is bigger than the bug:** fault indication was a property of one
screen's renderer instead of the display layer, so every screen added later silently opted out.

### The false-success family

A refused verb printed its refusal to the console and the LCD, and a deferred render then painted
`Added <n>` / `Tare taken` / `Calibrated` over it. **The operator's actual interface confidently
reported a success that did not happen.** Fixed across five triggers:

- `+`, `tare`, `kv` refused on loop state — `68dc5a8`
- `kv` failing on `span <= 0` — `36c4226`. **The dangerous one.** `calFactor` is NOT written on
  that path, so the old factor stands and nothing is corrupted; the lie is purely in the report,
  which is what makes it insidious — there is no bad number to find later while the operator's
  belief about the machine is wrong.
- `+` on a full tally — `36c4226`

One member of the family remains and is filed, not fixed: see §6.

---

## 5. Parked, with reasons

**`w` publishes live while the filter is frozen.** On `LOOP_DEAD` / `LOOP_FAULT_HIGH` the payload
correctly carries `psi: null`, `psi_fault`, and the state — but a constant stale `w` with no null
and no hold flag. A consumer branching on `sensor != "ok"` fails safe and the known consumer does
exactly that. Changing `w`'s contract now would widen the consumer break already filed. Real,
deferred, deliberate.

**`OPERATOR.md`'s two duplicated display sections have diverged** — one gained a `CAL NOT SET!`
bullet. Superseded by issue #2, whose fix is to delete the second copy; patching a doomed copy is
wasted work.

**AVCC is an unmeasured direct gain term.** `ADC_VREF_V = 5.0f` is an assumption about a rail
that on USB power is typically 4.75-5.25 V. The old sensor was ratiometric so AVCC error
cancelled; with a burden it does not. **This is now a larger error term than the 0.1% burden
tolerance sitting beside it.** The two copies of the constant were collapsed into one
(`4618826`) so a future correction moves everything at once.

---

## 6. Filed, not fixed

- **`Alex-Pennington/greybeard-plc4-tractor-scale#2`** — `OPERATOR.md` carries its body twice and
  the two copies give opposite answers on whether the tally survives a power cycle. Settled from
  the firmware: **it does** (`tallySaveEntry` / `tallySaveIndex` per record, `tallyRestore` at
  boot behind a signature). The all-caps "NOTHING YOU RECORD SURVIVES" section is stale.
- **`Alex-Pennington/greybeard-plc4-tractor-scale#3`** — a refused tare assigns `tarePrev` before
  the verb runs, destroying the undo target. A later `Undo tare` silently does nothing. The last
  member of the false-success family, and a state bug rather than a message bug.
- **`Alex-Pennington/greybeard-plc-consumer#5`** — the consumer nulls `w` and `psi` on
  `sensor != "ok"`, which would **discard every `SATURATED` reading** — the exact excursions this
  feature exists to capture. Fixing the firmware and reintroducing the defect one layer down.

---

## 7. Before this is flashed

**THE GATE, ruled by Alex 2026-09-13: `greybeard-plc-consumer#5` IS UPDATED BEFORE THIS IS
FLASHED.** Not "should be" — he was asked about the ordering explicitly and chose it. That
consumer nulls both `w` and `psi` whenever `sensor != "ok"`, which now includes `SATURATED`, so
flashing first means the ingest silently discards the first stretch of over-range excursions —
the exact measurement this feature exists to produce, and the one nobody has ever had.
**As of this commit that work is NOT STARTED.**

**Acknowledged by Alex 2026-09-13: transmitter fitted FIRST, then flash.**


- **Do not flash current-mode firmware before the transmitter is physically fitted.** PLC #3's
  playbook §2 makes this a procedure, not a commit message; it cost them a morning.
- **`greybeard-plc-consumer#5` wants actioning at or before the flash**, or the first over-range
  excursions are silently dropped by the ingest.
- **The board will demand a full recalibration**: `EEPROM_SIGNATURE` moved to `PL4A`, so the
  voltage-era `tareValue` and `calFactor` are refused rather than silently used. `CAL NOT SET!`
  on the LCD is expected, not a fault.
- **Check the old harness for oil** before reuse — the previous transducer leaked through its
  cable entry and capillary action carries oil to whatever the far end is.
- **The burden resistor is load-bearing for the bottom-end fix.** With it open or unfitted, A2
  floats and the old mid-rail defect returns exactly as it was. Firmware cannot detect that.

---

## 8. Wrong turns, recorded so nobody repeats them

**The task plan's decomposition was wrong and a worker caught it.** Task 2 deleted the
`SENSOR_COUNTS_*` constants while three later tasks owned their remaining consumers, so the tree
could not compile at the end of Task 2 — and Task 2's own verification step already assumed
otherwise. A pre-flight scan checked that exact pair and recorded the wrong answer, because it
compared **line ranges** rather than **symbol dependencies**.

**A proposed stopgap was rejected.** The worker offered to rename the constants at the psi
conversion and leave the real fix for its owning task. That would have put one commit on the
branch carrying the unclamped-negative-psi bug the floor exists to prevent. On a machine whose
documented failure history is plausible wrong answers nobody notices, a knowingly-wrong
intermediate commit is the wrong trade even for one commit.

**A grep gate that could never pass.** Verification greps used the bare pattern `sensorState`,
which substring-matches the live function `sensorStateName` — 11 legitimate hits, so the check
could never return zero. The correct pattern is
`SENSOR_OK\b\|SENSOR_OPEN\|SENSOR_SHORT\|SENSOR_COUNTS`.

**Dispatches asserted the wrong working directory.** Workers were told the implementation
worktree "is your cwd". It is not — a subagent inherits the controller's. All of them resolved it
by absolute path and no stray edit occurred, but a worker that believed it and used relative
paths would have written into the controller's checkout, which is the shared-tree failure
`CLAUDE.md` exists to prevent.
