# 4-20 mA current loop: settled decisions and open questions

Recorded 2026-09-13, during brainstorming, on branch `feat/plc4-4-20ma-loop`
(cut from `plc4/mqtt-port` at 7e7f2f4).

**Status: NO CODE WRITTEN.** This is a decisions record, not a design spec. The design
is blocked on the unresolved range contradiction in §5. Nothing in `src/ArduinoHydraulicScale.cpp`
has been edited.

**Base branch is `plc4/mqtt-port`, not `main`.** `main` is stale upstream refactor history
and HEAD is thirty commits ahead of it. Never diff against or merge from `main`.

---

## 1. Why this feature exists, and it is not scaling

Two defects drive it. Re-scaling counts without closing both is the wrong outcome.

### T4 is broken at the bottom: a floating input reads `ok`

Bench demo 2026-09-01, **nothing connected to the transducer input**:

```
a = 4401 counts   v = 1343 mV   p = 632 psi   sensor = ok
```

A high-impedance CMOS input floats near mid-rail rather than falling to ground, so the
1311-count open-circuit threshold (`:75`) never trips. A cut wire on the tractor reads as
a valid 632 psi. Full write-up in `notes/2026-09-01-transducer-failure-and-selection.md`.

**A 4-20 mA loop retires this by construction rather than patching it.** The burden resistor
is a hard 150 ohm pull-down to GND -- two orders of magnitude stiffer than the 10k-47k the
2026-09-01 note proposed. An open loop sources 0 mA, develops 0 V across the burden, and lands
at ~0 counts. 0 mA sits unambiguously below the 4 mA live zero with the entire 0-4 mA band as
separation.

Residual failure mode, recorded so it is not a surprise: if the **burden itself** opens while
the loop is live, the loop drives a high-impedance pin and the protection diodes clamp it to
rail -- which reads as fault-high. Caught, but by a different path than intended. A burden open
*and* a dead loop together is a double fault that floats.

### T4 is also broken at the top: real pressure reads as a wiring fault

**Reported by Alex 2026-09-13:** the fitted sensor reads `SENSOR SHORT` whenever the circuit
goes overpressure, and that happens routinely.

```
:897   if (sensorValue > SENSOR_COUNTS_FS && sensorValue <= SENSOR_COUNTS_SHORT) overRangeCount++;
:902   else if (sensorValue > SENSOR_COUNTS_SHORT) sensorState = SENSOR_SHORT;
```

Over-range is counted only in the 14745-15400 band. Anything past 15400 is classified a
**wiring fault** and is **not** counted as over-range. So genuine pressure excursions are
reported as a short to rail, `overrange` in the status payload (`:400`) stays clean while it
happens, and the LCD shows `SENSOR SHORT` (`:775`).

This is the mirror image of the float defect: a false fault on a real reading, where the float
is a false `ok` on no reading. **The three-state enum at `:80` has one bucket above full scale,
so "saturated but measuring correctly" and "the transmitter has failed" are indistinguishable.**

NAMUR NE43 banding separates them. See §4.

---

## 2. Settled decisions

| Decision | Value | Decided by | When |
|---|---|---|---|
| Loop supply | Dedicated 24 V, separate supply | Alex | 2026-09-13 |
| Supply negative | Bonded to Arduino GND | derived | 2026-09-13 |
| Burden resistor | **150 ohm, 0.1%, <=25 ppm/degC**, A2 to GND | Alex | 2026-09-13 |
| Circuit exceeds 3000 psi | Assumed true, over the spec-sheet figure | Alex | 2026-09-13 |
| ADC input pin | A2, unchanged (`:62`) | already settled | 2026-09-01 |
| Approach | mA is source of truth, counts derived (§4) | Alex | 2026-09-13 |
| Transmitter | **Rosemount 1151, 0-3000 psi** -- see §5 | Alex | 2026-09-13 |
| `EEPROM_SIGNATURE` | Bump it; forces a recalibration (§6.1) | Alex | 2026-09-13 |
| Filter on fault | Freeze it; hold last reading (§6.2) | Alex | 2026-09-13 |

### Why 150 ohm

At AVCC 5.000 V, 14-bit, `counts = mA * R * 3.2766`:

| R | 3.5 mA | 4 mA | 20 mA | 21.0 mA | 25 mA | rail at | psi/ct @3000 | vs +/-30 psi | drop @20 mA |
|---|---|---|---|---|---|---|---|---|---|
| 100 ohm | 1147 | 1311 | 6553 | 6881 | 8192 | 50.0 mA | 0.572 | 52x | 2.00 V |
| 120 ohm | 1376 | 1573 | 7864 | 8257 | 9830 | 41.7 mA | 0.477 | 63x | 2.40 V |
| **150 ohm** | **1720** | **1966** | **9830** | **10322** | **12287** | **33.3 mA** | **0.382** | **79x** | **3.00 V** |
| 180 ohm | 2064 | 2359 | 11796 | 12386 | 14745 | 27.8 mA | 0.318 | 94x | 3.60 V |
| 200 ohm | 2294 | 2621 | 13106 | 13762 | 16383 | 25.0 mA | 0.286 | 105x | 4.00 V |
| 220 ohm | 2523 | 2883 | 14417 | 15137 | clips | 22.7 mA | 0.260 | 115x | 4.40 V |
| 250 ohm | 2867 | 3277 | 16383 | clips | clips | 20.0 mA | 0.229 | 131x | 5.00 V |

Three constraints, in order of how much they bind:

- **Resolution binds on nothing.** Every row is 52-131x finer than the sensor's own +/-1.0% FS
  (+/-30 psi on a 3000 psi unit). This confirms the 2026-09-01 note: ADC quantisation was never
  the limiting term. **Do not re-argue the burden on resolution grounds.**
- **Compliance binds on nothing**, given the 24 V supply. Even 250 ohm leaves 19 V at the
  transmitter, roughly double what a Rosemount-class part needs.
- **ADC headroom binds, and it is the whole decision.**

**The deciding argument is hardware survival, not measurement.** PLC #3's tension transmitter,
rated 4-20 mA, metered **29.7 mA** at motor stall -- 1.48x its rated maximum, on this estate,
with a meter (`references/16_4_20ma_transmitter_playbook.md` §10, in
`~/greybeard-saw-actuator-plc`). A hard-over transmitter is not politely bounded at 25 mA.
Against 200 ohm, 29.7 mA drives the pin to **5.94 V, 0.94 V above AVCC**, into the RA4M1's ESD
clamp, on a failure mode this estate has actually observed. 150 ohm tolerates 33.3 mA and keeps
the pin under the rail through it.

**250 ohm is rejected, and the reason does not depend on the supply:** 20 mA lands at exactly
16383, so NE43 fault-high at 21 mA clips into the same rail as a hard-over. It destroys the
diagnostic this whole feature exists to buy.

### A new error term the swap introduces

**The old sensor was ratiometric: its output tracked AVCC, so AVCC error cancelled.** A current
loop through a burden does not cancel -- the reading is `I * R / AVCC`, so AVCC error becomes
direct gain error. `vReference = 5.0` (`:86`) is a compile-time assumption about a rail that on
USB power is typically 4.75-5.25 V. That is up to +/-5% gain error against a sensor rated
+/-1.0% FS. **Unresolved.** It may want measuring rather than assuming.

Burden tolerance is the second new term: a 1% resistor is +/-30 psi at 3000, consuming the
entire sensor accuracy budget by itself. Hence 0.1%. At 100 ppm/degC a 60 degC enclosure swing
would add another 18 psi, hence <=25 ppm/degC.

---

## 3. The 2828 psi figure, and why it is not the answer

`notes/JD_520M_SpecSheet.pdf` gives **Rated Pressure 2828.0 psi**, Pump Capacity 11.0 gpm.

**That figure is not this tractor's.** The sheet's Tractor block reads
`Model - 5085E, 5090E, 5100E FT4`. Pump capacity and rated pressure are properties of the
*tractor's* hydraulic system, not the loader; the 520M is sold across several chassis and the
5055E is smaller than any of the three listed. It is the best figure anyone has and it is not a
5055E figure.

**It also contradicts the field observation.** 2828 psi is *below* the fitted sensor's 3000 psi
span, so that sensor should never reach full scale -- yet it rails routinely. Worse,
`SENSOR_COUNTS_SHORT = 15400` (`:76`) is **4.70 V**, above where a 0.5-4.5 V part is specified
to be able to drive at all. Three readings of that, leading to different machines:

1. The 5055E's circuit genuinely relieves well above 2828 psi and spikes carry it past 3000.
2. The fitted part is not the 0.5-4.5 V / 3000 psi unit the constants assume. `sm` (`:1103`)
   would not fix that -- a different **output voltage** range needs both count endpoints moved.
3. The reading is wrong for an electrical reason, which on this pin has precedent (`:55-61`,
   and the float defect in §1).

**Alex ruled 2026-09-13: assume the circuit really does exceed 3000 psi**, taking the field
observation as authoritative over a spec sheet for a different tractor. Recorded as a ruling,
not as a verified measurement.

Note: the sheet says `520M`; this workspace's notes say `M520`. Treated as the same loader.

---

## 4. Chosen approach: mA is the source of truth, counts are the runtime representation

Constants declared in milliamps against published standards; count thresholds derived at
compile time from the burden and AVCC.

```cpp
constexpr float BURDEN_OHMS = 150.0f;     // 0.1%, <=25 ppm/degC, A2 to GND
constexpr float ADC_VREF_V  = 5.0f;
constexpr int   ADC_FS      = 16383;      // 14-bit full scale is 16383, not 16384
constexpr int mAToCounts(float mA) { return (int)(mA*BURDEN_OHMS/ADC_VREF_V/1000.0f*ADC_FS + 0.5f); }

constexpr float LOOP_DEAD_MA       =  3.5f;  // playbook §5; PLC #1 CURRENT_DEAD_LOOP_MA
constexpr float LOOP_ZERO_MA       =  4.0f;
constexpr float LOOP_FS_MA         = 20.0f;
constexpr float LOOP_FAULT_HIGH_MA = 21.0f;  // NAMUR NE43; playbook §5
```

**Why derived and not hand-written counts.** This file already carries **three** documented
instances of the same failure -- a hand-derived scaling constant left behind when the resolution
changed. `:914-919` names all three: the `map` in-max, the 10-bit filter seed, and the psi
conversion itself. A count that silently *means* 21.0 mA is a fourth one waiting to happen the
moment the burden or AVCC changes.

### Why 21.0 mA and not 20.5

Taken from playbook §5, not invented here. NE43 puts the valid measuring range at 3.8-20.5 mA
and the fault signal at >=21.0; 20.5-21.0 is over-range-but-still-measuring and is deliberately
**not** a fault. 21.0 also sits in the gap for both Rosemount alarm configurations (standard
saturates 20.8 / alarms >=21.75; NE43 option saturates 20.5 / alarms >=21.5), so it
discriminates correctly without anyone knowing which option code the part carries.

**A 20.5 threshold would be wrong rather than merely conservative:** it would report a
transmitter legitimately pinned at full scale, measuring correctly and saying so, as failed.
**That is precisely the defect described in §1, reproduced in new units.**

### Bands, and what consumes each one

This closes the gap named in `Alex-Pennington/phoenix-nest-plc1#156` -- *classifying a dead loop
is not the same as acting on it*. There, `LoopFault::LOW` is produced at `sensorIO.h:75` and
consumed nowhere, so a dead loop on the saw circuit raises nothing at all. Every band below has
a named consumer.

| counts @150 ohm | mA | state | LCD | `psi` | `psi_fault` | filter |
|---|---|---|---|---|---|---|
| < 1720 | < 3.5 | `LOOP_DEAD` | `NO SENSOR` | `null` | `"low"` | frozen |
| 1720-1965 | 3.5-4.0 | `UNDER_ZERO` | weight | 0 | `null` | live, counted |
| 1966-9829 | 4.0-20.0 | `LOOP_OK` | weight | value | `null` | live |
| 9830-10321 | 20.0-21.0 | `SATURATED` | **weight** | value | `null` | live, counted |
| >= 10322 | >= 21.0 | `FAULT_HIGH` | `XMTR FAULT` | `null` | `"high"` | frozen |

`SATURATED` deliberately does **not** suppress the weight. The existing `overrange` /
`overrange_max` counters (`:259-260`, `:398-400`) are rebanded onto it, so how often the circuit
goes past full scale becomes a published number instead of an inference -- which is also how §3's
open question gets answered by measurement rather than argument.

The 3.5-4.0 mA dead band is playbook §8's known residual: it reads as a healthy 0 psi and is
**structurally unchanged by the swap**. What the swap does eliminate is its dominant cause,
excitation sensitivity -- a current loop has no excitation dependency, so a sagging supply or a
high-resistance joint can no longer move the reading. What can still land there is a transmitter
drifting below its own zero, which is a calibration problem rather than a wiring one. Counting
it is the mitigation.

### Rejected approaches

- **Re-derive the four constants in place**, keeping the three-state enum. Smallest diff, and it
  reproduces §1's false fault exactly. Also leaves `SENSOR_SHORT` (`:355`) as a name for
  something that cannot happen on a current loop.
- **Carry milliamps through the whole pipeline**, PLC #3 style. Cleanest estate match, but
  `outputValue = calFactor * (smoothedValue - tareValue)` (`:932`) is counts-based and tare is
  stored in counts, so it drags a rewrite of the machine's actual job into a sensor swap.
  Rejected on scope.

---

## 5. Sensor range: Rosemount 1151, 0-3000 psi -- AND AN UNRESOLVED CONTRADICTION

**Alex named the part 2026-09-13: a Rosemount 1151 at 0-3000 psi.**

### The contradiction, recorded rather than resolved

Four hours earlier, in the same brainstorm, Alex ruled (§3) **"assume the circuit really does
exceed 3000 psi"** and chose to size the new sensor **"with real margin above 3000"**, taking his
own field observation over the spec sheet. A 0-3000 psi transmitter has **no margin above 3000**.

Both statements are his and they do not reconcile. Neither has been withdrawn. **Do not read
the later answer as superseding the earlier one** -- a part number given in answer to a range
question is not obviously a retraction of a ruling about the circuit, and guessing which he meant
is exactly the class of silent assumption this file exists to prevent.

What follows from the pair, if both hold: the transmitter saturates routinely in normal loader
work. Under the §4 banding that is survivable -- it reports `SATURATED` and keeps reading, rather
than being mistaken for a wiring fault as it is today -- but **the scale is pinned and wrong for
the duration of every excursion.** The `overrange` counter measures how often.

### What the 1151 changes, and what must be confirmed before any of it is coded

**None of the following has been read from a datasheet or a calibration plate.** Per `CLAUDE.md`:
do not reason from a part number you have not seen. These are the questions the part raises, not
answers about it.

1. **Span comes from the calibration plate, not the model number.** The 1151 is a re-rangeable
   analog transmitter with zero and span adjustments, so its *calibrated* span is whatever it was
   last set to -- which on a second-hand or legacy unit is unknown. Playbook §4 is explicit and
   was written after this went wrong: *"Span comes from the calibration plate, not from a sibling
   repo... Span silently scales every pressure decision a machine makes; it is the last constant
   that should be copied."* **`sensorMaxPSI` must come off the plate.**
2. **Re-ranging may dissolve the contradiction.** If the unit's range code spans past 3000 psi,
   it can be re-spanned to put 20 mA above the circuit's actual peak. That would satisfy the §3
   ruling without buying another part. Needs the range code.
3. **NE43 applicability is unconfirmed for this variant.** A plain analog 1151 does not
   self-diagnose and will not deliberately drive a fault current. The §4 bands remain correct --
   `FAULT_HIGH` still means "current outside any valid range" -- but on an analog unit it catches
   a hard-over rather than a transmitter announcing its own failure. The threshold does not
   change; its *meaning* narrows. A 1151S / smart variant would differ.
4. **Overpressure rating is the question the failure history makes urgent.** The previous unit
   died by pressurised oil working through the sensor body and out of the cable entry -- element
   or internal seal failure, with overpressure and cycling fatigue the usual causes. The 1151's
   over-range and static pressure limits come from its range code. **If the circuit exceeds 3000
   psi routinely, the margin between "routine" and "rated" is the number that matters.**
5. **Loop supply is comfortable.** A 1151-class 2-wire transmitter wants roughly 12 V minimum at
   its terminals; 24 V less the 3.00 V burden drop leaves ~21 V. The 150 ohm choice is unaffected.
6. **Mounting and vibration are open.** The 1151 is a large cast-housing industrial transmitter
   with conduit entries, designed for pipe or panel mounting in a plant. A loader boom is a
   shock-and-vibration environment. Not a firmware question, but it is not nothing.

### The accuracy argument, retained because it will come up again

**The 2026-09-01 note's "resolution is not a criterion" conclusion is about ADC quantisation and
does not transfer to range selection.** Sensor accuracy is +/-1.0% **of full scale**, so it scales
directly with range:

| range | overload | accuracy | ~weight error* | behaviour above span |
|---|---|---|---|---|
| 3000 psi | 4500 | +/-30 psi | +/-35 lb | saturates; `SATURATED`, still reads |
| 3500 psi | 5250 | +/-35 psi | +/-41 lb | saturates; `SATURATED`, still reads |
| 5000 psi | 7500 | +/-50 psi | +/-58 lb | effectively never saturates |

\* scaled against ~3500 lb lift capacity from the sheet; indicative, not calibrated. Overload
figures are the 150% typical of the ceramic family in the 2026-09-01 note, **not** the 1151's --
see item 4 above.

3000 -> 3500 was the 5 psi difference the note rightly dismissed. **3000 -> 5000 is not** -- a 67%
increase in absolute error, not inside any noise floor. This is the real cost of buying margin,
and it is why the §3 ruling and the 0-3000 part are a genuine trade rather than a slip.

**Under a current loop, changing the psi range later is a ONE CONSTANT recompile.** The count
endpoints are set by the burden and do not move when the psi span does, so `sensorMaxPSI` alone
rescales it -- cleaner than the voltage case, where both endpoints would have to move too. The
live hazard is that `sm` (`:1103`) is **not persisted** and reverts on every reset, so a
non-default range wants the compile-time default changed, not `sm` typed after each boot.

---

## 6. Consequences requiring specific approval

Per `CLAUDE.md`: nothing in the `.ino` changes without Alex approving that specific change.

1. **APPROVED by Alex 2026-09-13.** **`EEPROM_SIGNATURE` (`:176`, `0x504C4334` 'PLC4') must change.** Counts-per-psi moves by
   ~1.67x, so a voltage-era `tareValue` and `calFactor` restore cleanly, pass the bounds check
   at `:617`, and publish `"cal":"ok"` (`:396`) over a scale wrong by that ratio -- the same
   class of failure as the 16256 leftover already documented at `:581-592`, which the signature
   was added to catch and would not. Bumping it makes existing machinery do the right thing:
   `calibrationDefaulted` -> `CAL NOT SET!` (`:780`). **Consequence: the board demands a full
   recalibration after flashing, and will not weigh correctly until it gets one.**
2. **APPROVED by Alex 2026-09-13.** **Freezing the filter on fault (`:907`) is a behaviour change to the weight path.** Today
   `smoothedValue` is fed unconditionally, so a dead loop drags the weight smoothly down to a
   plausible number rather than stopping. Playbook §6's rule is that a faulted read returns
   early without touching the filtered value, so the channel holds its last reading. This is
   outside the strict sensor block.
3. **`payload[128]` (`:948`) will overflow.** Worst case today is ~101 bytes with
   `CLIENT_ID = "plc4-jd5055e"` (`:239`); adding `psi_fault` and an `mA` field pushes past 128
   and `snprintf` truncates silently into invalid JSON. Grows to 192.
4. **`:617`'s tare bounds** (`SENSOR_COUNTS_OPEN` / `SENSOR_COUNTS_FS`) must move to the new
   count endpoints.
5. **The `sm` comment at `:920-928` must be rewritten.** Under a current loop a different psi
   range with the same 4-20 mA output *is* rescalable by `sm` alone and the count endpoints do
   not move -- cleaner than the voltage case. The live hazard that `sm` is **not persisted** and
   reverts to the compile-time default on every reset is unchanged.
6. **`sensorVoltage` (`:906`) reports mV.** The diagnostic unit a person at the machine compares
   against a meter is now mA.

**A0 is untouched.** The 10/14-bit `analogReadResolution` flips at `:858`, `:884` and `:1127`
stay exactly as they are.

---

## 7. Physical items, not firmware

- **The old harness may be oil-filled.** The previous transducer leaked through its cable entry,
  and capillary action along a conductor bundle carries oil to whatever the far end is. **Check
  the cable and its terminating enclosure before reuse.** Replacing the sensor and reusing an
  oil-filled cable moves the problem rather than fixing it. From
  `notes/2026-09-01-transducer-failure-and-selection.md`.
- **Do not flash current-mode firmware before the transmitter is physically fitted.** Playbook
  §2 makes this a deployment rule enforced by someone remembering, not by a commit message.
- **If a transmitter reads dead-loop, check for excitation before touching anything else.**
  Playbook §1: microamps on the loop means no drive. It is not the channel and not the firmware.
  This cost PLC #3 a morning.

---

## 8. Corrections made during this brainstorm

Recorded so the next reader does not redo the reasoning.

**The burden recommendation moved 150 -> 200 -> 150.** First 150 ohm on the grounds that it
works on a 12 V or a 24 V rail and so does not force the supply decision. When Alex settled on
24 V that justification evaporated and 200 ohm won as the largest burden that maps the whole
0-25 mA range without clipping. Then playbook §10's **measured 29.7 mA hard-over** made the
overvoltage argument decisive and returned it to 150 ohm. The final reason is unrelated to
either earlier one.

**The scope statement at `:70-71` is literally true and practically incomplete.** "Nothing else
in the file hard-codes these values" is correct -- grep confirms 1638/14745/1311/15400 appear
only at `:73-76`. But three consumers break anyway: `:617` (tare bounds in counts), `:932`
(weight is counts x a factor), and `:176` (the signature does not change when the front end
does). Treating that sentence as the scope of the change would have shipped a silently wrong
scale.
