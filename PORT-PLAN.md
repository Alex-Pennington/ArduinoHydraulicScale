# PLC #4 — MQTT forward-port onto `origin/main`

**Status: implemented on `plc4/mqtt-port`, branched from `origin/main`. Not flashed — no
board is tethered.**

**Build verified.** `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi` against core
1.6.0 succeeds: **79,940 bytes flash (30%)**, **10,256 bytes RAM (31%)**, 22,512 bytes left for
locals. The 1.6 KB backlog queue costs what §1.3 predicted. PlatformIO is not installed on this
machine, so `pio run` was not the vehicle; the same sources build under arduino-cli with
`LiquidCrystal` 1.0.7 substituted for the `fmalpartida` fork named in `platformio.ini` (same
API for this usage). The core emits one benign warning — the bundled `WDT` library declares
architecture `renesas` against the board's `renesas_uno`.

### The one defect the toolchain found that review did not

`publishWeight()` was declared `void` and used in a boolean test. It would have compiled as
written nowhere, so it was caught immediately — but it sat through a full self-review first,
which is the argument for building rather than reading.

### The third instance of the unfinished resolution change

The coordinator's hypothesis — that the 10-bit-to-14-bit change was never followed through
anywhere, so a third instance would exist — was right, and it was in a value this port
**publishes**.

`pressureValue = map(smoothedValue, 0, 16384, 0, sensorMaxPSI)` mapped from **zero counts**
rather than from the sensor's 1638-count live zero:

| true pressure | reported |
|---|---|
| 0 psi | **299 psi** |
| 3000 psi | **2699 psi** |

Wrong at both ends — a +299 psi offset *and* a span of 2400 over a true 3000, i.e. 80%. Fixed
to map from `SENSOR_COUNTS_ZERO` to `SENSOR_COUNTS_FS`.

**Be precise about the blast radius: the weight was never affected.** Verified — `pressureValue`
is assigned at `:364`, published at `:387` and printed at `:639`, and appears nowhere else. The
weight comes from `outputValue = calFactor * (smoothedValue - tareValue)` at `:366`, straight
from counts, and `kv` calibrates against the same count span. The psi path is a leaf. **The
scale's own reading was sound; the published `psi` field would have been wrong.**

State it that way round rather than leaving it unqualified. The vague version invites a reader
to conclude the scale was broken and then to distrust the calibration path with it — which
would be wrong, and would cost someone a day re-verifying something that was never at fault.
The narrow claim is also the stronger one.

**It still matters, because it is wrong *plausibly*.** 299 psi on an idle loader does not look
like a fault; it reads as residual system pressure. A zero or a dead-loop value would be
noticed. Any consumer using that field to judge relief pressure or overload is misled, by 20% at
the top of the range, with nothing to signal it.

Running total for the same unfinished change: the `map()` in-max (C3), the 10-bit seed of the
14-bit filter, and now the psi conversion. The keypad ladder (A2) was a fourth, already fixed on
`main`. **Anything in this codebase that touches an ADC count should be assumed guilty until
checked against 14 bits.**

One asymmetry left deliberately, and flagged in the code: `sensorMaxPSI` is runtime-settable via
`sm`, but the count endpoints are compile-time constants derived from the 0.5-4.5 V spec.
Changing `sm` alone does **not** rescale a physically different transducer. If the part number
comes back as something else, `SENSOR_COUNTS_ZERO`/`_FS` must change with it.

### The through-line: not one of these defects announces itself

Worth stating once, because it is the strongest single generalisation in the whole
investigation and it is the argument behind several design choices here.

| Defect | What it looked like |
|---|---|
| T0 | ~9700 at no load — a number, not an error |
| C2 | weights published to `arduino/w`, unmatched by any subscription, no error anywhere |
| T5 + the boom mark | a stale zero silently applied at every boot |
| T4 | a shorted sensor reading high, and persisting into the span via `kv` |
| the psi conversion | 299 psi on an idle loader — reads as residual pressure |
| the watchdog conflict | correct in the yard; a reset loop only out of range |

**Every one of them produces a plausible wrong answer rather than a visible failure.** That is
why T4 refuses to record, tare or calibrate on a faulted sensor instead of returning a number,
why backlog carries `age_ms` rather than being published bare, and why `publishTo()` refuses a
truncated topic instead of shortening it. On this machine, the correct response to "I am not
sure this value is real" is to say so, never to emit the best guess.

### On review: a session reviewing its own fresh work is the weakest reviewer of it

Two instances today, both in code written minutes earlier and both missed by a careful
self-review:

- `publishWeight()` declared `void` and used in a boolean test — caught only by the compiler.
- The psi conversion — caught only because the coordinator asked whether a third instance of
  the resolution change existed. The field was *added by this port*, so the exposure was
  created and then reviewed by the same session, and survived.

Neither was subtle in hindsight. Both were invisible from inside. This is a better argument for
building the code and getting an outside prompt than any process assertion — and it is why the
A3 remedy, which was accepted from a peer without walking the boundary case, is the third
example of the same failure from the other direction.

### The defect that reading the core found, which a build would NOT have caught

**The watchdog and the WiFi library conflict, and the result is a reboot loop out of range —
the one situation the whole design exists to serve.** Verified against ArduinoCore-renesas
1.6.0:

- `WDT.cpp getMaxTimeout()` = `RL_16384 * PR_8192 / (PCLKB_Hz/1000)`. `bsp_clock_cfg.h` for
  UNOWIFIR4 sets ICLK /1 from 48 MHz and PCLKB /2, so PCLKB is 24 MHz and **the watchdog
  ceiling is 5592 ms**. There is no longer period available on this part.
- `CWifi::begin()` busy-waits `while (millis() - start < _timeout)` polling `status()`, with
  **no watchdog refresh inside the loop**, and `CWifi::CWifi()` defaults `_timeout` to
  **10000 ms** (`WiFiS3/WiFi.cpp:8`).

So a *failed* association — driving out of range — blocks for 10 s against a watchdog that
cannot be set beyond 5.6 s. The board would reset every retry, forever, while away from the
yard. It compiles and it is correct in the yard; it fails only where nobody is watching.

Fixed by calling `WiFi.setTimeout(3000)` and `mqttClient.setConnectionTimeout(3000)` in
`setup()` before the watchdog is armed. Both constants are documented at their definition with
the ceiling and the coupling, because raising one without the other reintroduces the loop.

**This is why F1-first mattered in a way that was not obvious when it was sequenced.** Arming
the watchdog before the networking is what made the conflict visible at all; had the networking
gone in first and the watchdog last, the reset loop would have arrived as a field symptom.

Date: 2026-09-01. Audit: `findings/2026-09-01-plc4-baseline-audit.md`. Context: `BRIEF.md`.

## The two rulings this plan serves

1. **Base is `origin/main`** (`dde9195`), not `r4_wifi_mqtt`. The MQTT layer moves onto it.
2. **Network loss: keep weighing, buffer, flush on reconnect.** The scale is fully usable
   offline. This is the design spine — neither branch has anything resembling it.

Ruling 2 is the larger change. Ruling 1 is mostly a matter of *not* dragging things back in;
ruling 2 is new engineering, and §1 below is where the open decisions actually are.

**Ruling 2 collapses the three terminal paths into a single design requirement, and that is the
spine of this port.** The audit lists `:152` (blocking WiFi retry), `:179` (`while (1);` on MQTT
failure) and `:219` (no reconnect) as three findings. Under ruling 2 they are not three fixes —
they are one requirement stated three times: **the weighing path may never wait on, or be
stopped by, the network.** Fixing them individually invites a port that makes the connect
non-blocking and still has no reconnect, or reconnects but still bricks on the first failure.
Treat it as one structural property to be satisfied, and verify it as one.

---

## 1. The buffering design

Ruling 2 says readings queue while offline and publish when the broker returns. That is three
separate design decisions, and **the first one collapses or creates the whole problem.**

### 1.0 RULED, 2026-09-01: RAM only

**A queued reading does not have to survive a power cycle.** Six loads lost to a lunchtime
key-off is acceptable — they remain on the LCD and in the `sum` tally. So the queue is SRAM and
the entire flash-endurance problem is **out of scope**: no wear budget, no erase-block layout,
no wear-levelling, no coexistence work against `main`'s existing `calFactor`/`tareValue`/`alpha`
addresses.

The two facts flagged as unverified in §1.2 — the erase block size in User's Manual §44, and
whether `EEPROM.put()` read-modify-writes a whole block — are **moot by ruling, not resolved.**
Nobody should spend time on either. If persistence is ever revisited, they become live again.

§1.1 and §1.2 are kept deliberately. They are what made "RAM only" a decision rather than a
guess: had the ruling gone the other way, the six-days number would have been the whole design.

### 1.1 The question that decided it: must the queue survive a power cycle?

If **no** — the queue lives in SRAM, and there is no endurance problem, no wear budget, and no
flash-layout work. The board has 32 KB SRAM (RA4M1 datasheet, Table 1.2 / product table). A
queue of a few hundred records at 8 bytes each is under 2 KB, which is affordable even beside
the existing `String` usage.

If **yes** — the queue must live in the 8 KB data flash, and the endurance arithmetic below
becomes binding.

This is Alex's call and we should not settle it. The thing worth putting to him is the
consequence, not the mechanism: **a tractor is keyed off between jobs.** If the queue is in RAM
and the operator weighs six loads out of radio range and then shuts down for lunch, those six
readings are gone. If that is acceptable — because the operator also has them on the LCD and in
the `sum` tally — RAM is obviously right and §1.2 is moot. If it is not acceptable, we pay for
flash.

### 1.2 If it must persist: the endurance budget, from the datasheet

Verified against the Renesas RA4M1 datasheet as hosted by Arduino
(`docs.arduino.cc/resources/datasheets/ra4m1-datasheet.pdf`), §2.15.2, Table 2.72 — not from
memory:

| Parameter | Figure |
|---|---|
| Data flash size | **8 KB** (Table 1.2) |
| Reprogram/erase cycles, data flash (`NDPEC`) | **100,000 min**, 1,000,000 typ |
| Data retention after 10,000 cycles | 20 years @ 85 °C |
| Data retention after **100,000** cycles | **5 years** @ 85 °C |
| Data retention after 1,000,000 cycles | 1 year typ @ 25 °C |
| Reprogram/erase cycles, **code** flash (`NPEC`) | **1,000** — two orders of magnitude worse |

Two constraints from the datasheet's own notes that shape the design more than the headline
number does:

- **The cycle count is erasures per block, not writes.** Writing different addresses inside a
  block costs nothing until the block is erased.
- **"Overwriting is prohibited"** (Table 2.72, Note 1) — the same address cannot be
  reprogrammed without erasing its block first.

Together those say: **an append-only log is nearly free; an in-place head/tail pointer is
ruinous.** The arithmetic, at the current 5 s telemetry cadence (17,280 records/day):

- Append-only, 8-byte records, erase once per 8 KB pass ≈ 1,024 records per erase cycle →
  ~102 million records before the 100,000-cycle minimum. Decades. Not a constraint.
- A head or tail pointer rewritten in place on every record → one erase per record →
  **100,000 cycles in under six days.** The board would wear out its data flash in a week.

If only operator-initiated `+` records are queued (say 50/day), even the naive design lasts
~5.5 years — but note the retention column: at 100,000 cycles the guaranteed hold time has
dropped to 5 years, so "it lasts as long as the endurance budget" and "the data is still there"
stop being the same statement.

**Not verified and to be checked before committing to a layout:** the data flash *erase block
size* is in the RA4M1 User's Manual §44, not the datasheet, and I did not read it. More
importantly, **how Arduino's `EEPROM` library on the R4 actually implements a byte write** —
whether it read-modify-writes a whole block per `put()` — is unverified, and it decides whether
the naive arithmetic above or the append-only arithmetic applies to `EEPROM.put()`. `main`
already uses `EEPROM` for `calFactor`, `tareValue` and `alpha` (`:65-67`, `:151`, `:223`,
`:237`) at `addr` 10/14/18, so any queue layout has to coexist with those and must not be
placed where a wear-levelling scheme would walk over them.

### 1.3 As built, and what is still Alex's

Implemented under the ruling, with the defaults below. Each is a one-line change if he wants
otherwise:

- **Only operator-recorded weights (`+`) are queued.** Periodic telemetry is a live sample; a
  stale one has no value, and queuing it at 5 s would fill any RAM queue in minutes. A recorded
  weight is a deliberate act with lasting value. **This is a judgement we made, not a ruling.**
- **Depth 200** (~4 working days at 50 buckets/day), 8 bytes per record, ~1.6 KB of 32 KB.
- **Overflow drops the OLDEST and counts the drops**, reported by the new `net` verb. Dropping
  the newest would silently discard the weight the operator just took, which is the worse
  failure — but this is a judgement about the work, not a technical necessity.
- **Backlog is marked**, per the constraint: every published reading carries `age_ms` and a
  `backlog` boolean, so a consumer can always tell stale from live.

### 1.4 Still open for Alex — not settled in code

- **Persistence:** must a queued reading survive a power cycle? (§1.1 — decides everything else)
- **Depth:** how many readings? This is really "how long a trip out of range must be covered".
- **Overflow policy:** drop oldest or drop newest? For a scale, dropping the *newest* silently
  discards the weight the operator just took, which is the worse failure — but that is a
  judgement about the work, not a technical one.
- **Timestamps:** does a buffered reading carry its own time? The board has no RTC and no NTP
  before the network returns, so the honest options are a millis()-since-boot delta stamped at
  capture and resolved at flush, or no timestamp at all and an explicit "these are backlog"
  marker on the topic. A buffered reading published bare, with no indication it is minutes or
  hours old, is worse than not buffering — a consumer cannot tell backlog from live.

---

## 2. Avoiding the 100 ms starvation (port constraint 1)

`main` samples by taking 10 readings with `delay(10)` between them, inside `loop()`
(`:82-88`), against `interval = 500` (`:30`). That is **100 ms of blocking in every 500 ms** —
a 20 % duty cycle during which nothing else in `loop()` runs.

Bolt an MQTT client on top unchanged and `mqttClient.poll()` is starved 100 ms at a time. That
does not reliably break anything; it produces the worse outcome of intermittent keepalive
misses and broker disconnects that nobody can reproduce.

**What the restructure has to look like** (stated as a constraint, not implemented):

- The sample burst must stop being a blocking `for` loop with `delay()`. The same 10-sample
  average is achievable by accumulating one reading per `loop()` pass on the existing millis
  scheduler and averaging when the count is reached — no delay, same result, and the whole of
  `loop()` keeps running at full rate.
- Once that is done, `mqttClient.poll()` and the reconnect state machine can run every pass.
- The existing `delay(2)` at the end of `r4_wifi_mqtt`'s `loop()` must not come across with the
  networking block; it is the same class of mistake at a smaller scale.
- **Only two failure states exist, and the code should not pretend otherwise.** Alex confirmed
  2026-09-01 that FOXTROTBRAVO is on the 192.168.10.0 LAN, so the board sits on the broker's own
  subnet with no routing hop at all. The reconnect machine therefore has to distinguish exactly
  two things: **out of WiFi range**, and **broker down**. There is no route-change, gateway-
  failure or inter-subnet case. Stated so nobody later adds defensive handling for hops that do
  not exist and then has to maintain it. (The earlier caveat that the target was unestablished
  from the board's vantage point is withdrawn — it is established, and in the simplest possible
  way.)
- **No blocking connect anywhere.** Ruling 2 makes this structural rather than stylistic: the
  weighing path must never wait on the network. That means the WiFi and MQTT connect attempts
  become a non-blocking state machine with backoff, called from `loop()` — not `while` loops in
  `setup()`.

---

## 3. Work order — the base-independent list

These survive the port because they are either in the code being ported or absent from both
branches. Ordered by what would hurt most in the field.

1. **F1 — watchdog.** Absent on both branches. It is the floor that makes every other failure
   recoverable rather than terminal, and it is the cheapest item here. It should land first, and
   it should land *before* the networking, so that a hang in new reconnect code is survivable.
2. **T4 — the NO SENSOR floor.** Still unbuilt on both branches; `main`'s map in-min is `0`, so
   the 1638-count live zero is computed nowhere. Port the *pattern* from PLC #1
   (`config.h:363 CURRENT_DEAD_LOOP_MA = 3.5f`), never the constant. Depends on the live zero
   being correct first. This is the difference between a chafed wire reading as "no load" and
   reading as "no sensor" — on a tractor, the likeliest hardware failure there is.
3. **A3 — out-of-bounds write in `-`.** Established from source on both branches, verified
   independently here; treat as settled, not as a claim to re-check.

   | | `+` guard | increment | OOB write |
   |---|---|---|---|
   | `r4_wifi_mqtt` | `:318` | `:323` | `:344` |
   | `main` | `:158` | `:164` | `:174` |

   The `+` guard `if (rTindex < rTsize)` passes at 49, writes `rT[49]`, increments to 50. The
   `-` guard is only `if (rTindex >= 0)`, true at 50, and writes `rT[50]` on `int rT[50]`.

   **It is reachable through the primary workflow, not through abuse.** `+` records a load and
   `sum` totals them; on a front-end loader, fifty buckets is a working day. This is the second
   finding whose severity changed once we knew the machine is a loader — it moves from
   "theoretical overflow" to "will happen".

   **The missing guard is the UPPER bound, not the lower.** An earlier revision of this plan
   said to change `rTindex >= 0` to `rTindex > 0`. **That does not fix it** — at `rTindex == 50`,
   `50 > 0` is true and `rT[50] = 0` still executes. `>= 0` was never the defect; it is merely
   useless, since `rTindex` is never negative. The defect is that the `-` path never checks
   `rTindex < rTsize` while the `+` path does.

   Correct minimal fix — bounded by construction, decrement before indexing:

   ```c
   if (rTindex > 0) { rTindex--; rT[rTindex] = 0; }
   ```

   **It also changes which element is zeroed, and that must be noted wherever the fix lands.**
   `sum` sets `rTsum = 0` and loops `i < rTindex` (`main :194-195`, `r4_wifi_mqtt :369-370`), so
   the removed entry is already excluded by the index alone. Today's `rT[rTindex] = 0` clears the
   *next free slot*, outside that loop — it does nothing as written. The reordered version clears
   the entry actually removed, which is equally cosmetic. Without this note a later reader will
   "restore" the old line believing it was a lost cleanup step.

   Also settled, so nobody re-derives it: `-` decrementing without adjusting a running total is
   correct. `rTsum` is recomputed on demand, not maintained incrementally.

   **Do not characterise the corruption.** Declaration order puts `rTindex` and `rTsum` after
   `rT[]`, but linker layout does not follow declaration order and nobody has established what
   sits there. Both candidates are self-masking: zeroing `rTindex` silently resets the buffer,
   zeroing `rTsum` silently zeroes the total, and either way the instrument returns a wrong
   number with no crash to notice. The fix is a two-line reorder, not a layout investigation.
4. **A7 — unguarded divide in calibration.** `main :222` still divides by
   `(smoothedValue - tareValue)` with no zero check, and `main` *adds* an unvalidated direct-set
   `c<value>` at `:226` that writes straight to EEPROM. Both want a range check.
5. **A6 — serial framing.** Survives verbatim (`main :284-294`). Back-to-back commands
   concatenate and are silently discarded.

---

## 4. Regression checklist — what the MQTT layer must not drag back in

`main` has already fixed C1, C3, C4, T0, T2, T3, T5, A1, A2, A4 and A8. Every one of those is a
defect a careless copy-paste from `r4_wifi_mqtt` could reintroduce. The ones actually at risk,
because the networking block touches them:

- **X1 — topic truncation. Two steps, in this order, not one commit.**
  1. Replace the length argument with `sizeof(topic_temp)` at **both** sites
     (`r4_wifi_mqtt :287` and `:330`), and size the buffers for the longest topic the naming
     convention will produce. Behaviour-preserving for `/debug`, restores `/weight`.
  2. *Then* rename the topic root as a separate change that can no longer truncate.
  Doing (2) first re-truncates at a *different* cut point rather than a shorter one, so
  subscribers break silently. Note the 30-byte buffers also cap the full topic at 29 characters.
- **C7 — topic root: `plc4/jd5055e/` (ruled by Alex, 2026-09-01).** Role prefix for estate
  consistency, equipment segment for the identity the author already chose at `main :74`.

  Checked against the buffers: `plc4/jd5055e/weight` is 19 characters and `plc4/jd5055e/debug`
  is 18, both inside the 29-character cap the 30-byte buffers impose — **no buffer resize is
  forced**. But **X1 step 1 is still required**: the literal `10` at `:330` truncates the weight
  topic to **`plc4/jd50`** (nine characters — not `plc4/jd55`; both topics share the same first
  nine). That stub is *not* under `plc4/jd5055e/`, so a correctly-written wildcard subscription
  on `plc4/jd5055e/#` would silently miss every weight publish while receiving debug normally.
  That is a worse failure than the current `arduino/w`, because it looks like the port worked.

  The client ID is still unset (`:167` commented out), so the library generates a fresh one every
  reboot. It should be derived from the same constant.

- **C5/C6 — no blocking connect, and a reconnect path that actually exists.** These are not
  "bugs to avoid copying"; under ruling 2 they are the feature being built.
- **A5 — topic semantics and payload.** Do not carry over publishing routine telemetry to
  `/debug` while the deliberate record goes elsewhere. Payload should carry units and identity;
  under §1.3 it may need a backlog marker.
- **A9 — plaintext credentials on port 1883.** Not a blocker, but it should be a conscious
  decision rather than an inherited default. `SECRET_MQTT_BROKER` must be the LAN form
  `192.168.10.99` — Saga's `farm1.internal` overlay address is not routable from the board.
- **`char oV[10]`** (`:289`/`:332`) — latent, not broken. The bound matches the buffer, but a
  32-bit `int` can render to 11 characters against a 9-character capacity. Size it properly
  while the file is open.

---

## 5. Deliverables beyond the firmware

- **Reissued `serial_bluetooth_terminal_cfg.txt`.** `main` renames `cal` → `kv` and `clear` →
  `b`, and adds `c<value>` and `w<value>`. Two of the seven committed macros no longer exist.
  The operator config has to ship with the firmware or the Bluetooth terminal stops matching
  the board — this is the terminal's own macro set, not the MQTT command path below.
- **`arduino_secrets.h.example`** already documents the five defines and the LAN-form broker
  requirement. It stays in step with whatever the port needs.
- **A commissioning check**, not a code change: confirm `FOXTROTBRAVO` routes to
  `192.168.10.0/24` (`BRIEF.md` question 2). If it does not, none of this reaches the broker.
- **The MQTT command contract, since built.** `plc4/jd5055e/cmd` (QoS 1, never retained)
  takes `{"id":"<unique>","verb":"<verb>"}`, an allowlisted subset of the console's own verbs
  (`tare + - b kv c w sm`) run through the same handler, and answers on `plc4/jd5055e/ack`
  (QoS 0, never retained) in the console's own success and refusal vocabulary. This is the
  replacement for the five physical buttons Alex is removing from the enclosure — see
  `OPERATOR.md` for the full contract and the allowlist's reasoning. `hang`, `read`, `net`,
  `sum` and `debug` stay console-only.

---

## What this plan does not decide

- The buffering questions in §1.3. Those are Alex's.
- The topic namespace (`BRIEF.md` question 3), pending the estate convention.
- The transducer part numbers (question 6). Every scaling number in the audit's T-series is
  spec-dependent; if the part comes back different, T0-T4 need re-deriving before T4's
  thresholds mean anything.
- ~~Whether the machine can be made accurate at all (question 5).~~ **CLOSED by Alex,
  2026-09-01: the loader is weighed at one fixed boom height, and the boom is physically marked
  so the operator reproduces the position every time.**

  **T5 and the mark are coupled, not two independent wins.** Without a reproducible position a
  persisted tare is *worse* than no persistence, because on boot it silently applies a stale zero
  instead of prompting for a fresh one. The mark is what makes the EEPROM write correct in the
  first place. Note that the ruling and the mechanism decay differently: citing "it is used at one
  fixed height" does not cover the mark being painted over or ignored, and if that happens the
  stored zero becomes actively harmful and does so silently.

  **Procedure consequence, now in `OPERATOR.md` and in the firmware comments: tare at the mark
  too, not only calibrate there.** Boom position changes the unloaded pressure as well as the
  loaded pressure, so a tare taken on the ground and a weigh taken at the mark differ by the
  boom's own contribution, and that offset lands in every reading.

  **An option for Alex, deliberately NOT implemented:** the firmware cannot detect whether the
  boom is at the mark, so it cannot validate a restored zero — but it could stop that zero being
  invisible. `main` already prints "Loader Scale" at boot; showing the restored `tareValue`
  beside it costs nothing and turns a silent stale zero into something an operator can notice.
  His call, and not in the work order.

  Superseded detail: The moment-arm concern is moot —
  pressure/load is evaluated at a single boom position, so a ratio-through-origin `calFactor` is
  a valid model and "repeatable but not accurate" does not apply. **Design the psi-to-weight path
  against single-point calibration.** This was the largest open risk in the investigation and it
  closed in the cheapest possible direction: the geometry answer does not change what the port
  should compute, so there is no reason to hold the conversion work.
