# PLC #4 — field commissioning and the overnight hold test

**Date:** 2026-09-02
**Branch:** `plc4/mqtt-port`
**Machine:** John Deere 5055E with an M520 loader, transducer fitted, board on the tractor
**Broker:** farm1, `192.168.10.99:1883`, SSID FOXTROTBRAVO

First findings written against a real transducer on a real machine. Everything before this was
bench work on a floating input.

---

## 1. The transducer spec is confirmed against hardware

This was the assumption under every scaling constant in the audit, and it held.

| | |
|---|---|
| predicted live zero, 0.5 V at 0 psi | **1638 counts** |
| measured tare, boom at the mark, empty bucket | **1806 counts** (0.551 V) |
| difference | 168 counts = **38.5 psi** |

That 38.5 psi is the boom holding its own weight — exactly what an empty loader at height should
read. A wrong zero would not have landed 168 counts above the prediction; it would have landed
anywhere.

So **0-3000 psi over 0.5-4.5 V is correct for the fitted unit**, and the T-series arithmetic in
`findings/2026-09-01-plc4-baseline-audit.md` is no longer provisional.

**What this does NOT confirm: the span.** Single-point calibration absorbs a span error exactly
(audit C3/T6), so calibrated weights cannot test it. Only `psi` is exposed to it, and `psi` has
no independent reference on this machine. The part marking and a datasheet remain the only way
to close that, and neither exists yet.

---

## 2. The circuit reaches the ADC rail in normal work

During commissioning, two consecutive telemetry samples went above the assumed full scale:

```
19:18:45   14958 counts   4.565 V   psi 3041   sensor ok      <- past 4.5 V full scale
19:18:50   16383 counts   rail      psi null   SENSOR SHORT   <- pinned, w and psi nulled
```

Single excursion, not a run. The eight samples before it swing between 5168 and 13153 counts, so
the boom was being actively worked rather than held — consistent with a hydraulic spike on valve
closure, not with the span being mis-scaled.

Two things follow:

- **The short-detect path fired correctly on real data** for the first time, nulling `w` and
  `psi` while keeping `counts`. Keeping `counts` is what made the event readable at all.
- **The M520 circuit demonstrably reaches the rail**, with relief at 2828 psi and a 3000 psi
  sensor sitting at 94% of full scale in normal use. That is real support for fitting a
  **pressure snubber** on the transducer port. The previous transducer failed by internal
  leakage out of its electrical connection, which is the failure overpressure produces.

`status` now carries `overrange` and `overrange_max` (commit `b10ec94`) so this stops having to
be inferred from psi arithmetic. Deliberately not a sensor state: an over-range sample is still a
reading, and routing it through the T4 guards would block taring and calibrating in response to
a spike.

---

## 3. A calibration-validity bug that made the operator undo good work

**The worst defect found on this machine, because of its direction.**

On every reboot the board announced `CAL NOT SET!` with a perfectly good factor and zero still
loaded. The operator believed it and recalibrated.

Cause, two bugs compounding:

- `alpha` at `addr+8` was written **only** by the `w` verb, while `tare`, `kv`, `c` and the
  undo-tare path each wrote their own field **plus the signature** at `addr+12`. On any board
  where `w` had never been used — which is every board — the signature vouched for a slot that
  had never been written. It read back `0xFFFFFFFF`: NaN as a float.
- That NaN failed a range check which set the **same flag** used by `calFactor` and `tareValue`.
  A filter smoothing constant could condemn a calibration.

The published data proves the mechanism: across the 19:18:55 reboot, `cf` stayed 0.15455 and
`tare` stayed 7829 — so the EEPROM read *did* happen and the signature *did* match — while `cal`
went `ok` → `DEFAULT`. Six status messages over eight minutes reported every reading as
untrustworthy when they were fine, and the recalibration at 19:26:36 followed.

Fixed in `f7af2fc`: `saveCalibration()` writes all three values and *then* the signature, so the
signature can only vouch for slots that exist; and `alpha` defaults silently.

**The through-line, inverted.** Every other defect on this machine made a wrong reading look
right. This one made a right reading look wrong, and pushed a careful operator into destroying
the calibration it falsely condemned.

**And it echoed.** The dashboard built from this firmware's own output repeated the same false
claim on a second screen. Two independent-looking sources agreeing — because one derives from the
other — reads as corroboration when it is an echo. Worth recognising wherever it appears.

---

## 4. The overnight hold test — and a wrong conclusion, corrected

A load was left in the air overnight with telemetry captured to
`~/plc4-logs/plc4-overnight-20260901-2357.jsonl` (5,645 samples, 23:57-08:04 UTC).

### The valid window is only the first 72 minutes

Alex established afterwards that **the arm reached the ground at about 21:10 EDT (01:10 UTC)**.
Everything after that is the arm resting, not a load being held.

**In the air**, 19:57-21:10 EDT, 729 samples:

```
counts   8384 -> 8421     trend  +0.61 counts/min  (+0.14 psi/min)
weight    510 ->  513     range  508-523
```

**A held load is stable.** Flat to very slightly rising over 72 minutes — about 3 weight units,
barely outside the +/-2 noise floor. Thermal, if anything. **There is no meaningful leak-down at
height**, and the cylinders hold.

### The wrong conclusion, recorded because the mistake is the lesson

Reading the *whole* night's curve — counts falling 8384 to 4083 with a decaying rate, from
-25 counts/min at 01:00 to -1 by 07:00 — this was written up as a self-limiting cylinder
leak-down, with operational advice to "weigh immediately after lifting" because a held load
supposedly lost ~0.7 weight units per minute.

**That was wrong.** The decay was the arm resting on the ground and residual pressure bleeding
off. The advice derived from it was backwards: a held load is stable, and there is no need to
hurry.

The evidence was there and was misread. At 00:31 the trend was observed and reported as
**rising**; when the full night showed a fall, a leak-down story was built around it without
asking what had changed in between. **The fact that settled it was not in the data at all** — it
was Alex knowing where the arm ended up. A clean decay curve is a persuasive shape, and it
persuaded.

### What the transition looks like

Adjacent-sample excursions of 40-80 counts around the landing, against +/-24 while hanging. So a
reading taken while anything is moving or settling is not trustworthy — which is separate from,
and more important than, the drift claim that was withdrawn.

---

## 5. Open: the 3.6% gap

A 530-unit calibration weight reads 511. The arithmetic is self-consistent — `cf` = 0.07765 puts
the calibration point at 8631 counts, and the published `w` matches exactly — so it is physical.

With leak-down eliminated by section 4, **boom position is the leading candidate again**: the
weigh position differing slightly from the position used for `kv`, which is the moment-arm effect
behind the fixed-height ruling.

**The test:** lower fully, raise back to the mark, read. Back near 530 means position, and the
mark wants hitting more precisely or `kv` re-running where the weighing actually happens. Still
511 means the calibration point and the weigh point differ systematically.

Worth noting the tares observed during commissioning span 1806, 1830, 7829, 8003 and 9860 counts.
The high ones were taken with the boom loaded or raised. Only a tare taken at the mark with an
empty bucket is meaningful, and the current 1806 is one of those.

---

## 6. What `reset` can and cannot tell you — measured

| event | reported |
|---|---|
| watchdog bite | `WATCHDOG (r1=0002)` — **detected** |
| upload / software reset | `software (r1=0004)` — **detected** |
| USB power cycle | `none-flagged` — PORF not observed |
| shield RESET button | `none-flagged` — indistinguishable from a power cycle |

Both commissioning reboots read `none-flagged`, which is the **absence of information**, not
evidence of a power event. Neither has a confirmed cause; a bench RESET press during an active
session is entirely plausible and n=1 either way.

---

## Still open

- **The part marking** off the failed transducer. The only route to a datasheet, and the only way
  to confirm the span (section 1).
- **A pull-down resistor** on the transducer input. Without it a disconnected input floats
  mid-rail and reports `sensor: ok` — the fault detection cannot work (see the 2026-09-01
  post-flash review).
- **A pressure snubber**, now evidenced rather than precautionary (section 2).
- **The 3.6% gap** (section 5).
- **Units.** `calWeight` is 530 with no unit recorded anywhere. Carried unitless downstream by
  deliberate agreement.
- **Two fixes await a flash**: `f7af2fc` (calibration flag) and `b10ec94` (over-range counters).
  Until then the LCD wrongly reports `CAL NOT SET!` after every reboot — check `cf` before
  recalibrating.
