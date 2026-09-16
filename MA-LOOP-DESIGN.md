# 4-20 mA current loop — firmware design

For `feat/plc4-4-20ma-loop`, base `plc4/mqtt-port`. Written 2026-09-13.

**Status: DESIGN ONLY. No firmware edited.** Decisions, their owners and the reasoning behind
them are in `notes/2026-09-13-4-20ma-loop-decisions.md`; this file says what the code does, not
why the hardware was chosen. Read that one first if you are arriving cold.

**Out of scope, explicitly:** the commissioning procedure (Alex is the tester and writes his
own), transmitter mounting and installation (his ruling, 2026-09-13), and every defect in
`findings/` that this feature does not touch.

---

## 1. What is being fixed

Not scaling. Two diagnostic defects, at opposite ends of the range, that share one cause: the
three-state enum at `:80` cannot express what a sensor front end actually does.

- **Bottom.** A disconnected input floats near mid-rail, so `SENSOR_OPEN` never trips and a cut
  wire publishes a confident 632 psi. Demonstrated on the bench 2026-09-01 with nothing
  connected.
- **Top.** `:897` counts over-range only within 14745-15400; anything past 15400 is classified a
  wiring fault at `:902` and is **not** counted. Real pressure excursions therefore publish as
  `SENSOR SHORT` while `overrange` stays clean. Reported by Alex 2026-09-13.

The burden resistor closes the first by construction: 0 mA develops 0 V across 150 ohm, landing
at ~0 counts with the whole 0-4 mA band separating it from the live zero. NAMUR banding closes
the second by giving "saturated but measuring" a state of its own.

**UPDATED 2026-09-16.** The reasoning above still holds; only the number changed. 0 mA still
develops 0 V, now across 68 ohm (the MCP3421 has its own reference, not this board's AVCC), and
still lands at ~0 counts with the same separation from the live zero. See §2 below and
`include/loop_scaling.h`.

---

## 2. Constants

mA is the source of truth; counts are derived at compile time. This file already carries three
documented cases of a hand-derived constant left stale when the resolution changed (`:914-919`
names all three: the `map` in-max, the 10-bit filter seed, and the psi conversion). A count that
silently *means* 21.0 mA would be the fourth.

**SUPERSEDED 2026-09-16.** The whole constants block below is the AVCC-era, 10-bit-internal-ADC
design. It is retained only as the historical record of that design; the live constants are
`include/loop_scaling.h`. The block reproduces two defects this repo has since closed, and
copying it now would reintroduce both:

- `BURDEN_OHMS = 150.0f` clips at **13.65 mA (68.3% of the 20 mA span end)** against the
  MCP3421's 2.048 V reference. The live value is **68 ohm** — see `include/loop_scaling.h` and
  the 2026-09-16 marker in `notes/2026-09-13-4-20ma-loop-decisions.md` §2 for why 150 ohm was the
  right answer for the AVCC-referenced ADC and was retired by the hardware change, not by error.
- `ADC_FS = 16383` is wrong for the MCP3421: its signed 14-bit output tops out at **8191**, not
  16383. Anyone copying this block gets a gauge that is both clipped and roughly 2x wrong.

```cpp
// ---------------------------------------------------------------------------
// 4-20 mA current loop front end.
//
// The transmitter is a Rosemount 1151, 0-3000 psi, 2-wire, fed from a DEDICATED
// 24 V supply whose negative is bonded to this board's GND. The loop current is
// read as the voltage across a burden resistor from A2 to GND.
//
// BURDEN_OHMS is the only number here that describes hardware. Change it and
// every threshold below follows. Do not hand-write counts.
//
// WHY 150 OHM, in one line so nobody re-argues it on resolution: resolution binds
// on nothing (every candidate 100-250 ohm is 52-131x finer than the sensor's own
// +/-1.0% FS) and compliance binds on nothing at 24 V. What binds is that PLC #3
// metered a 29.7 mA hard-over; against 200 ohm that puts 5.94 V on this pin,
// 0.94 V above AVCC and into the ESD clamp. 150 ohm holds to 33.3 mA.
// Full trade table: notes/2026-09-13-4-20ma-loop-decisions.md §2.
//
// TOLERANCE IS NOW A CALIBRATION TERM AND IT WAS NOT BEFORE. A 1% burden is
// +/-30 psi at 3000 -- the entire sensor accuracy budget. Specify 0.1%,
// <=25 ppm/degC; at 100 ppm/degC a 60 degC enclosure swing adds another 18 psi.
//
// AND SO IS AVCC. The old sensor was RATIOMETRIC: its output tracked AVCC, so
// AVCC error cancelled. A burden does not cancel -- the reading is I*R/AVCC, so
// ADC_VREF_V below is a direct gain term. 5.0 is an assumption about a rail that
// on USB power is typically 4.75-5.25 V. UNRESOLVED; may want measuring.
//
// RESOLVED 2026-09-16, and moot rather than measured: the MCP3421 carries its
// OWN 2.048 V reference, so AVCC leaves the measurement as a gain term entirely.
// That is a large part of why the branch moved to it. See include/loop_scaling.h.
// ---------------------------------------------------------------------------
constexpr float BURDEN_OHMS = 150.0f;   // 0.1%, <=25 ppm/degC, A2 to GND
constexpr float ADC_VREF_V  = 5.0f;     // see the AVCC warning above
constexpr int   ADC_FS      = 16383;    // 14-bit full scale is 16383, not 16384

constexpr int mAToCounts(float mA) {
  return (int)(mA * BURDEN_OHMS / ADC_VREF_V / 1000.0f * ADC_FS + 0.5f);
}

// NAMUR NE43 and the estate's own settled figures. Every value here cites a
// source; none is invented locally.
constexpr float LOOP_DEAD_MA       =  3.5f;  // playbook §5; PLC #1 CURRENT_DEAD_LOOP_MA
constexpr float LOOP_ZERO_MA       =  4.0f;  // live zero
constexpr float LOOP_FS_MA         = 20.0f;  // span end
constexpr float LOOP_FAULT_HIGH_MA = 21.0f;  // NE43; see the 20.5 note below

constexpr int COUNTS_DEAD       = mAToCounts(LOOP_DEAD_MA);        // 1720
constexpr int COUNTS_ZERO       = mAToCounts(LOOP_ZERO_MA);        // 1966
constexpr int COUNTS_FS         = mAToCounts(LOOP_FS_MA);          // 9830
constexpr int COUNTS_FAULT_HIGH = mAToCounts(LOOP_FAULT_HIGH_MA);  // 10321
```

**Why 21.0 mA and not 20.5, because it will be questioned.** Taken from playbook §5, not invented.
NE43 puts the valid measuring range at 3.8-20.5 mA and the fault signal at >=21.0; 20.5-21.0 is
over-range-but-still-measuring and is deliberately **not** a fault. 21.0 also sits in the gap for
both Rosemount alarm configurations (standard saturates 20.8 / alarms >=21.75; NE43 option
saturates 20.5 / alarms >=21.5), so it discriminates correctly without anyone knowing the option
code. **A 20.5 threshold would be wrong rather than conservative: it would report a transmitter
legitimately pinned at full scale, measuring correctly and saying so, as failed — which is
precisely the defect in §1, reproduced in new units.**

`SENSOR_COUNTS_ZERO` / `_FS` / `_OPEN` / `_SHORT` (`:73-76`) are deleted. Nothing else in the file
hard-codes their values; their three consumers are handled in §5, §6 and §7.

---

## 3. Classification

Replaces `enum SensorState { SENSOR_OK, SENSOR_OPEN, SENSOR_SHORT }` (`:80`). `SENSOR_SHORT` names
a condition a current loop cannot produce and is retired.

```cpp
enum LoopState { LOOP_OK, LOOP_DEAD, LOOP_UNDER, LOOP_SATURATED, LOOP_FAULT_HIGH };
```

| counts | mA | state | measuring? |
|---|---|---|---|
| < 1720 | < 3.5 | `LOOP_DEAD` | no |
| 1720-1965 | 3.5-4.0 | `LOOP_UNDER` | yes, floored at 0 psi |
| 1966-9829 | 4.0-20.0 | `LOOP_OK` | yes |
| 9830-10320 | 20.0-21.0 | `LOOP_SATURATED` | **yes** |
| >= 10321 | >= 21.0 | `LOOP_FAULT_HIGH` | no |

Classification runs on the 10-sample mean (`sensorValue`), before conversion, exactly as `:897-903`
does today. Order matters: classify, then convert, then filter.

**`LOOP_SATURATED` must not suppress the reading.** That is the whole point of §1's top-end fix.

---

## 4. What consumes each state

This closes the gap named in `Alex-Pennington/phoenix-nest-plc1#156` — there, `LoopFault::LOW` is
produced at `sensorIO.h:75` and consumed nowhere, so a dead loop on PLC #1's saw circuit raises
nothing at all. **Classifying a dead loop is not the same as acting on it.** Every state below has
a named consumer.

| state | LCD row 0 | `psi` | `psi_fault` | filter | counter |
|---|---|---|---|---|---|
| `LOOP_OK` | weight | value | `null` | fed | — |
| `LOOP_UNDER` | weight | `0` | `null` | fed | `underrange` |
| `LOOP_SATURATED` | weight | value | `null` | fed | `overrange`, `overrange_max` |
| `LOOP_DEAD` | `NO SENSOR` | `null` | `"low"` | **frozen** | — |
| `LOOP_FAULT_HIGH` | `XMTR FAULT` | `null` | `"high"` | **frozen** | — |

`sensorAtCapture` on queued weights (`:423`) records the state as a historical fact at capture
time, unchanged in mechanism but now carrying five values rather than three.

The 3.5-4.0 mA dead band is playbook §8's known residual: it reads as a healthy 0 psi and is
**structurally unchanged by the swap**. What the swap eliminates is its dominant cause —
excitation sensitivity. A current loop has no excitation dependency, so a sagging supply or a
high-resistance joint can no longer move the reading. What can still land there is a transmitter
drifting below its own zero, a calibration problem rather than a wiring one. Counting it is the
mitigation, not a fix.

---

## 5. Pressure conversion

```cpp
pressureValue = map((int)smoothedValue, COUNTS_ZERO, COUNTS_FS, 0, sensorMaxPSI);
if (pressureValue < 0) pressureValue = 0;    // floor; playbook §4
```

`sensorMaxPSI` default becomes **3000** — unchanged in value, but its meaning now comes off a
Rosemount 1151's 0-3000 span rather than a 0.5-4.5 V assumption.

Span is 9830 - 1966 = **7864 counts**, i.e. **0.382 psi/count** — 79x finer than the sensor's own
+/-30 psi. Quantisation is not a limiting term and should not be re-argued as one.

The floor is new and it matters: `map()` does not clamp, so `LOOP_UNDER` would otherwise publish a
small negative pressure. Above `COUNTS_FS` `map()` extrapolates, which is correct and deliberate —
a saturating transmitter is proportional up to its clamp. **But per playbook §10, a reading above
the transmitter's rated maximum is a validity boundary, not a scale marking:** psi above
`sensorMaxPSI` should be read as "at least this much", and that sentence belongs in the comment
beside the constant, not only here.

The `sm` comment at `:920-928` is rewritten. Under a current loop a different psi range with the
same 4-20 mA output *is* rescalable by `sm` alone and the count endpoints do not move — cleaner
than the voltage case it describes. The live hazard is unchanged: **`sm` is not persisted** and
reverts to the compile-time default on every reset, so a non-default range wants the default
changed, not `sm` typed after each boot.

---

## 6. Weight path — two approved changes

Both approved by Alex 2026-09-13 as specific changes, per `CLAUDE.md`'s gate.

### 6.1 The filter freezes on fault (`:907`)

Today `smoothedValue = (smoothedValue*alpha) + (sensorValue*(1-alpha))` runs unconditionally, so a
dead loop drags the weight smoothly down to a plausible-looking number rather than stopping. On
`LOOP_DEAD` and `LOOP_FAULT_HIGH` the sample is discarded without touching `smoothedValue`, so the
channel holds its last reading. Playbook §6's rule, and what PLC #3 does.

**Corrected after the final whole-branch review.** This section originally justified the freeze
with "the LCD writes the fault over row 0 either way, so a held value is never presented as
current." That was false, and the freeze is what made it dangerous: `hmiPoll()` returns before
`hmiDrawHome()` whenever the screen is `SCR_CHECKCAL`, so Menu -> Check cal showed a frozen —
therefore rock-steady, therefore convincing — weight with no fault anywhere on the display, and
nothing times that screen out.

Both drawing paths now carry the fault themselves. `hmiDrawHome()` puts `sensorStateName()` on
row 0 of the home screen; `hmiDrawCheckCal()` replaces both rows with `sensorStateName()` and
`NO READING Lt=x` on the calibration-check screen. Neither prints a number that could be read as
a reading while `loopIsMeasuring()` is false, so the freeze cannot present a held value as
current on any screen.

### 6.2 `EEPROM_SIGNATURE` is bumped (`:176`)

`0x504C4334` ('PLC4') becomes `0x504C3441` ('PL4A').

Counts-per-psi moves by ~1.67x (13107 counts of span becomes 7864), so a voltage-era `tareValue`
and `calFactor` would restore cleanly, pass the bounds check at `:617`, and publish `"cal":"ok"`
(`:396`) over a scale wrong by that ratio. **That is the same class of failure as the 16256
leftover already documented at `:581-592` — which the signature was added to catch, and which this
one would miss.** Bumping it makes existing machinery do the right thing: `calibrationDefaulted`
goes true, the LCD shows `CAL NOT SET!` (`:780`), and the wire says `cal:DEFAULT`.

**Consequence, accepted: the board demands a full recalibration after flashing and will not weigh
correctly until it gets one.**

`:617`'s bounds move with it:

```cpp
if (tareValue < COUNTS_DEAD || tareValue > COUNTS_FS) { tareValue = COUNTS_ZERO; defaulted = true; }
```

`COUNTS_DEAD` rather than `COUNTS_ZERO` as the lower bound, so a tare taken slightly below the live
zero is not rejected as impossible.

---

## 7. MQTT contract

### Telemetry (`:948-952`)

```
{"w":%d,"psi":%s,"mA":%.2f,"counts":%d,"sensor":"%s","psi_fault":%s,"queued":%d,"id":"%s"}
```

- **`psi` publishes JSON `null`, never `0`, on `LOOP_DEAD` and `LOOP_FAULT_HIGH`.** Playbook §6:
  null is not zero, at every layer. A real zero and a dead sensor are different things.
- **`psi_fault`** is new: `null` / `"low"` / `"high"`. PLC #3's exact field name and values, so the
  estate reads the same way. Fail-low is a wiring question and fail-high is not; they lead to
  different first checks at the machine.
- **`mA`** is new. It is the unit a person standing at the machine compares against a meter, and
  playbook §10's lesson is that the out-of-range reading arrives on a meter, not through the
  firmware.

**`char payload[128]` (`:948`) must grow to 192.** Worst case today is ~101 bytes with
`CLIENT_ID = "plc4-jd5055e"` (`:239`); the new fields push past 128 and `snprintf` truncates
**silently into invalid JSON**.

The other two 128-byte payloads — `publishWeight()` (`:364`) and `queueFlush()` (`:432`) — do
**not** need growing. They carry `sensorStateName()` but none of the new fields, and the longest
new string (`UNDER ZERO`, `SATURATED`, `XMTR FAULT`) is shorter than the `SENSOR SHORT` it
replaces.

### Status (`:391-401`)

Adds `"underrange":%lu`. `char payload[320]` (`:390`) grows to 384.

### `sensor` string values — a breaking change, stated plainly

`sensorStateName()` (`:352-357`) feeds both the LCD and the wire.

| state | string | change |
|---|---|---|
| `LOOP_OK` | `ok` | unchanged |
| `LOOP_DEAD` | `NO SENSOR` | unchanged |
| `LOOP_UNDER` | `UNDER ZERO` | **new** |
| `LOOP_SATURATED` | `SATURATED` | **new** |
| `LOOP_FAULT_HIGH` | `XMTR FAULT` | **replaces `SENSOR SHORT`** |

Chosen to break as little as possible: `ok` and `NO SENSOR` keep their meanings and their
spellings. `SENSOR SHORT` disappears because it described a condition a current loop cannot
produce. Two new values appear, which is unavoidable — the whole point is that the old vocabulary
had one bucket too few. **Any consumer switching on `sensor` needs updating**; `greybeard-plc-consumer`
is the known one and is not this session's to edit.

---

## 8. LCD

`hmiDrawHome()` (`:772-790`). The fault line at `:775` currently reads `if (sensorState != SENSOR_OK)`,
which would now hide the weight during saturation — the exact behaviour §1 is fixing. It becomes:

```cpp
if (loopState == LOOP_DEAD || loopState == LOOP_FAULT_HIGH)
  snprintf(l0, sizeof(l0), "%-16s", sensorStateName());
else
  snprintf(l0, sizeof(l0), "%-11d n%-3d", outputValue, rTindex);
```

`UNDER ZERO` and `SATURATED` never take row 0. They are visible in telemetry and in the counters;
they are not reasons to withhold a weight from the operator.

All strings fit 16 columns.

---

## 9. Console

- `sensorVoltage` (`:906`) currently reports mV. The diagnostic unit is now mA; the debug line
  gains it. mV is retained — it is what a meter on the burden reads.
- `:1158`, `:1214` print the state name; they follow `sensorStateName()` and need no separate edit.
- `sm` (`:1103-1110`) is unchanged in behaviour. Its guard (`v <= 0 || v > 20000`) still holds.

---

## 10. A0 is untouched

The resistor-ladder keypad on A0 and the `analogReadResolution` flips around it at `:858`, `:884`
and `:1127` stay exactly as they are. The burden is on A2 (`:62`), which does not move.

---

## 11. What this does NOT fix

Recorded so the swap is not read as a clean sweep.

- **The 3.5-4.0 mA dead band** still reads as a healthy 0 psi. §4.
- **`psi_fault` does not latch.** A dropout shorter than a publish interval is invisible.
  Playbook §8's third bullet, inherited unchanged.
- **AVCC is an unmeasured gain term.** §2. It is now larger than the burden tolerance it sits
  beside, and the ratiometric cancellation that used to hide it is gone.
- **The `2828 psi` question from `notes/…-decisions.md` §3 is not answered**, only made
  measurable: `overrange_max` will say how far past 3000 psi the circuit actually goes.
- **Every defect in `findings/` that is not one of the two in §1.** Untouched by design.
