# PLC #4 baseline audit — `ArduinoHydraulicScale.ino`

**Date:** 2026-09-01
**Commit audited:** `c602655` ("update", 2024-10-29), branch `plc4/baseline` from `origin/r4_wifi_mqtt`
**Auditor:** PLC #4 worker session
**Revision 2** — 2026-09-01. Adds F2 (broker/SSID decisions) and T0; corrects the
PLC #1 sensor-interface claim and the topic-fix sequencing. Superseded text is marked.

**Revision 3** — 2026-09-01. No finding changed. Added: `origin/main` already fixes eleven of
the items below and has no networking at all, so this audit is best read as the **regression
checklist for a forward-port of the MQTT layer onto `main`** rather than as a fix list against
`r4_wifi_mqtt`. The partition — which items are already fixed on `main` versus which are
base-independent and survive any port — is in `BRIEF.md` open question 12. The items that live
inside the MQTT block (C2, C5, C6, C7, X1, A5, A9) are live regardless of base, because they
are the code being ported.

**Status:** audit only. **Nothing was fixed.** Firmware changes are a separate gate needing Alex's approval.

## Method and caveats

Every claim below is **read from source**. No board was tethered; nothing was compiled,
flashed, run, or scoped. Where I state runtime behaviour I am reasoning from the Arduino
core's documented implementations of `map()`, `String::toCharArray()` and the UNO R4
`Serial` object, not from observation. Line numbers are `c602655`.

Arithmetic uses the transducer spec relayed by the coordinator from Alex: **0-3000 psi,
0.5-4.5 V ratiometric**, no part number given. I did not read PLC #1's tree.

Two numbers in the relayed arithmetic are off by one count and I use my own: at 14-bit
resolution 4.5 V is `16383 x 4.5/5 = 14744.7` → **14745 counts**, not 14746, so the span
is **13107**, not 13108. The difference is 0.23 psi and changes no conclusion.

---

## 1. The governing fact: there is no watchdog

### F1 — no watchdog anywhere in the firmware — **CONFIRMED**

`grep -niE 'watchdog|wdt|iwdg'` over the file returns nothing (the only `reset` hit is the
comment at :232, "reset the debouncing timer"). The RA4M1 has a WDT and an IWDT; neither is
enabled, fed, or referenced. There is no `NVIC_SystemReset()` path either.

This is the fact that sets the severity of everything else, and I agree it belongs first.
The three blocking constructs below are not three peer defects — they are three entrances
to the same absorbing state, and F1 is what makes that state absorbing:

| Line | Construct | Waits for |
|---|---|---|
| 145 | `while (!Serial) { ; }` | a USB host to attach |
| 152 | `while (WiFi.begin(ssid, pass) != WL_CONNECTED)` | an AP to accept it |
| 179 | `while (1);` | nothing — terminal by construction |

On PLC #1/#2/#3 a blocking call ends in an Opta IWDG reset: bad, but self-announcing and
self-clearing. PLC #4 has no such floor. A hang is **permanent and silent**, and it is the
one machine in the estate that drives out of radio range with nobody watching a console.

All three sit in `setup()`, ahead of `lcd.begin()` at :191. So the failure signature for
every one of them is identical and uninformative: **a blank LCD and a dead machine**,
indistinguishable from a blown fuse. The operator cannot tell which stage failed, or that
firmware is running at all.

### F2 — the deployment decision makes those three failures certainties, not risks

**Added 2026-09-01 after Alex fixed the broker and SSID.** This is a design finding, not a
defect in a line of code, and it changes how severely §1 should be read.

- **Broker: the estate broker at `192.168.10.99:1883`.** A fixed address on the farm LAN.
- **SSID: `FOXTROTBRAVO`.** (PSK not recorded here and not in the tree.)

Relayed from the coordinator and **not verified by this session** (out of workspace scope, and
the host is unreachable from Saga in any case): a TCP connect to `192.168.10.99:1883` from
Saga fails and `ip route get` resolves only via the default gateway, so this is a local,
in-range-only broker rather than a cloud one. PLC #1's telemetry goes one-way to a cloud
broker; PLC #4 is being pointed at something **different in kind**.

PLC #4 is on a tractor. Driving out of range is not an adverse condition, it is ordinary use.
Cross that with the four paths above and each one stops being conditional:

| Path | Under a fixed local broker on a mobile machine |
|---|---|
| `:152` blocking `WiFi.begin` retry | Key-on out of range → loops in `setup()` forever. `lcd.begin()` is at `:191`, *after* it, so the operator gets a **blank screen**, not an error. |
| `:179` `while (1);` on MQTT failure | In WiFi range but broker unreachable → hard hang, same blank screen. |
| `:219` no reconnect in `loop()` | Drive away, drive back → never reconnects. Power cycle required. |
| F1 no watchdog | Every one of the above is permanent and silent. |

**As built, PLC #4 cannot function as a mobile machine against this broker.** The first trip
out of the yard ends with a blank LCD and a dead scale until somebody power-cycles it back in
range — and that is the expected outcome of normal operation, not a tail risk.

The design question this raises is in `BRIEF.md` open question 1 and is not ours to answer:
whether weighing should be independent of the network entirely. **No reconnect logic has been
written.** Fixes remain gated.

---

## 2. The seven original claims

### C1 — `isFluctuating()` assignment instead of comparison — **CONFIRMED**

`:97` `if (ScaleReadingsIndex = ( ScaleReadingsIndex_MAX - 1 )) {`

Assignment, not comparison. The value assigned is 99, which is truthy, so the condition is
**always true**, the `else` at :100 is dead code, and `ScaleReadingsIndex` is reset to 0 at
:98 on every call.

The consequence is more degenerate than "computed over mostly-zero data". Because :96 writes
`ScaleReadings[ScaleReadingsIndex]` *before* the reset, and the index is always 0 on entry,
**only `ScaleReadings[0]` is ever written**. Elements 1..99 hold their zero-initialised
values (`:91`) forever. The loop at :104-106 therefore sums:

```
i=1:      |ScaleReadings[1] - ScaleReadings[0]| = |0 - reading| = |reading|
i=2..99:  |0 - 0| = 0
```

so `averageDifference == |reading| / 99`, and `isFluctuating()` reduces to
`|outputValue| > 120 * 99`, i.e. **`|weight| > 11880`**. It is not a fluctuation detector at
all; it is a magnitude threshold on a single instantaneous sample. The LED at :261 that it
drives means nothing.

Secondary, for whoever takes the fix: even with `==` restored, the difference sum at
:104-106 spans the ring buffer's wrap seam, so one term per pass is the discontinuity
between newest and oldest rather than a real sample-to-sample step.

### C2 — `+` publish truncates the topic — **CONFIRMED**

`:326` `char topic_temp[30];` — 30 bytes, as you said.
`:330` `str_temp.toCharArray(topic_temp, 10);` — the literal is 10.

`String::toCharArray(buf, bufsize)` copies at most `bufsize - 1` characters and
NUL-terminates. `"arduino/weight"` is 14 characters, so the published topic is
**`arduino/w`** (9 characters). The `/debug` path at `:283-288` declares the same 30-byte
buffer and passes 30, and `"arduino/debug"` is 13 characters, so it publishes correctly.
The two publishes do disagree on topic. Confirmed exactly as reported.

The bug is the literal `10` at :330 alone — the buffer is adequate; only the length argument
is wrong. It looks copy-pasted from the `char oV[10]` payload buffer two lines below (:332).

### C3 — `map()` in-max of 1023 against 14-bit resolution — **CONFIRMED as fact; your stated consequence needs correcting**

`:258` `pValue = map(fValue, tareValue, 1023, 0, 3000);`
`:185` `analogReadResolution(14);` — range is 0..16383.

The mismatch is real and it is a **10-bit leftover**. I traced it: `origin/uno-r3-swserial-lcd`
carries the identical line `pValue = map(fValue, tareValue, 1023, 0, 3000);` where 1023 was
that board's correct full scale. `analogReadResolution(14)` arrived in `c00bafe`
(2024-10-27, "button1, fluctuations") — the AVR→R4 port — and the constant was not updated.

**Where I disagree: "the conversion is wrong by roughly 16x and every calibration is
absorbing that error" is half right, and the half that is wrong matters.**

- The factor is not 16x. With the default `tareValue = 241` (:22) the slope is
  `3000/(1023-241) = 3.836` psi/count against a correct `3000/13107 = 0.2289` psi/count —
  a ratio of **16.8x**. The 16x figure only holds if `tareValue` were 0; the offset moves it.
- **The gain error is fully cancelled by calibration, so it is not the live defect.**
  `cal` at :386 computes `calFactor = calWeight / pValue` — a single-point ratio through the
  origin — and :259 then computes `outputValue = calFactor * pValue`. Any constant gain
  error in `pValue` divides straight back out. A bench calibration produces a correct
  reading at the calibration point regardless of the in-max.
- **What survives calibration is the offset error, and it is large.** A ratio-through-origin
  calibration is only valid if `pValue` is zero at zero load. It is not: the sensor's live
  zero is 1638 counts (0.5 V) while the map's in-min is 241, so at zero load
  `pValue = (1638-241) * 3.836 = 5359` "psi". The calibrated reading at zero load is
  `530 * 1397 / (counts_cal - 241)`, which is nonzero and, for any modest calibration load,
  a large fraction of the calibration weight itself. I cannot put a number on it without the
  cylinder geometry (see open questions).

So: the constant is wrong, but the mechanism you should record is *offset survives, gain
cancels*. That is precisely the shape of a defect that passes on the bench and drifts in
service — see T6, where the coordinator reached the same conclusion independently.

Two further notes on this line:

- `map()` takes `long` arguments, so the `float fValue` produced by the EMA at :257 is
  **truncated to an integer at every call**. The sub-count smoothing the filter exists to
  provide is discarded before it is used.
- There is no `constrain()` anywhere in the file. Below tare, `pValue` goes negative and is
  reported as a negative weight with no fault indication.

### C4 — `while (!Serial);` blocks forever — **CONFIRMED, and it is the most severe single line in the file**

`:145-147`
```c
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
```

On the UNO R4 WiFi, `Serial` is the native USB CDC port and its `operator bool()` is false
until a host enumerates and opens it. With no USB host attached this never returns. It sits
ahead of WiFi (:152), MQTT (:175), `analogReadResolution` (:185) and `lcd.begin` (:191), so
on a tractor **the board does nothing, shows nothing, and never reaches `loop()`**.

This line did not exist before the port — `git log -S'while (!Serial)'` returns only
`c00bafe`. On the R3 branch `Serial` was the D0/D1 hardware UART, for which the guard is a
no-op; the comment at :146 ("Needed for native USB port only") is the author's own note that
this is boilerplate. Moving to the R4 turned the boilerplate into a brick-on-boot.

**On the PLC #3 comparison — I adopt the coordinator's sharper version, which is right and
changes what a fix looks like.** Same family (serial state gates the control path), opposite
trigger:

- PLC #3's N20 (and PLC #2 #15): a host **is attached and is not draining**, so a blocking
  write stalls once the TX buffer fills.
- PLC #4 `:145`: blocks waiting for a host to **attach at all** — and on a tractor there
  never will be one.

The PLC #3 fix shape (bound the write / drop when the sink is not draining) does not address
:145. This one is a deployment-configuration bug: the guard has no business in a firmware
that boots unattended.

### C5 — blocking WiFi retry and terminal MQTT failure — **CONFIRMED**

`:152-156`
```c
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
```
`:175-180`
```c
  if (!mqttClient.connect(broker, port)) {
    ...
    while (1);
  }
```

Both confirmed. Three amplifications:

- The effective retry period is longer than 5 s. `WiFi.begin()` on WiFiS3 is itself blocking
  with its own internal association timeout; the `delay(5000)` is added on top of it.
- `while (1);` at :179 is an empty statement with no watchdog kick (F1) and no reset.
  Recovery requires a physical power cycle. And MQTT is attempted **exactly once, ever** —
  there is no retry loop around :175 at all.
- Both are ahead of `lcd.begin()` (:191), so neither failure is distinguishable from the
  other, or from C4, or from dead hardware.

A tractor is mobile. Out of AP range at key-on, the machine cannot boot; broker unreachable
at key-on, the machine bricks until it is power-cycled somewhere else.

### C6 — no reconnect path in `loop()` — **CONFIRMED**

`loop()` spans :198-397 and contains `mqttClient.poll()` at :219 and nothing else network-
related. There is no `mqttClient.connected()` test, no `WiFi.status()` test, no reconnect,
no backoff.

Worth adding, because it changes how the failure presents: **the loss is silent, and the
operator is told the opposite.** `MqttClient::beginMessage()` returns 0 when the client is
not connected, and its return is ignored at both :288 and :331. After a dropped link a `+`
still prints the reading to serial (:320-322), still increments `rTindex` (:323), and still
prints **`"OK"`** at :337 — for a message that went nowhere. The recorded tally and the
published record diverge with no indication at either end.

### C7 — MQTT topic root is bare `"arduino"` — **CONFIRMED, and the client ID is worse**

`:75` `const char topic[] = "arduino";` — a literal with no machine identity, no MAC
suffix, no configuration hook.

Adding to it: **the client ID is also unset.** `:167` `// mqttClient.setId("clientId");` is
commented out, above the library's own warning at :166 that "Each client must have a unique
client ID". ArduinoMqttClient then defaults to `"Arduino-"` plus `millis()`. That is worse
than the topic collision on two counts: the ID **changes on every reboot**, littering the
broker with orphaned sessions; and any other Arduino on the estate using the same default
scheme can land on the same ID, in which case an MQTT broker is required to disconnect the
older client — producing a mutual reconnect flap that looks like a network fault.

See §4 for the constraint that binds C2 and C7 together.

---

## 3. Claims derived from the transducer spec

### T0 — the scale is wrong out of the box, before anyone tares it — **CONFIRMED**

**Revision 2. This supersedes the framing in T2, and it should be read first.** I originally
wrote this as "both states are broken" inside C3; the coordinator is right that it deserves to
lead, and right that the earlier "taring inverts the scale" framing under-claims it.

The sensor's live zero of 1638 counts (T1) **already exceeds the map's in-max of 1023** at
`:258`. So with the stock `tareValue = 241` (`:22`), on a board nobody has touched, at zero
load:

```
map(1638, 241, 1023, 0, 3000) = (1638-241) * 3000 / (1023-241) = 4191000 / 782 = 5359
outputValue = calFactor * pValue = 1.81 * 5359                              = 9699
```

**~9700 units displayed at no load, out of the box.** (The relayed ~5360/9700 are the
unrounded values; `map()` returns `long` and truncates, so the exact figures are 5359 and 9699.)

Taring does not introduce the fault. It changes its **sign**: `tareValue` becomes ~1638, in-min
then exceeds in-max, the denominator goes negative and the transfer function inverts (T2).

So there is no starting state and no operator action that produces a correct reading. That is a
stronger and much cheaper claim than the tare one: **it is testable by powering the board with
no load and reading the display.** No calibration, no tare, no rig.

Marked **INFERRED** in one respect: the 1638-count live zero assumes the R4's default analog
reference is VCC ≈ 5 V. `:187` `//analogReference(DEFAULT);` is commented out, so the core
default applies and I have not read the core sources to confirm what it is.

Confirming step — and it needs **no code change**: the `read` verb at `:351-367` already prints
raw `sensorValue`, and `debug_flag` defaults to `true` (`:7`). Issue `read` at a known 0 psi and
the raw count settles it. (The commented block at `:267-281` prints the same thing and would
also serve, but uncommenting it is unnecessary.) The catch is that reaching a console requires
C4 and A1 to be dealt with first — which is itself a finding about how undiagnosable this
machine is in the field.

### T1 — the correct scaling constants — **CONFIRMED (arithmetic re-derived, two counts corrected)**

At AVCC = 5 V and 14-bit resolution (`:185`), with no explicit `analogReference` call
(`:187` is commented out, so the R4 core default AVCC applies):

| | volts | counts |
|---|---|---|
| live zero (0 psi) | 0.5 | `16383 x 0.5/5` = **1638** |
| full scale (3000 psi) | 4.5 | `16383 x 4.5/5 = 14744.7` → **14745** |
| span | 4.0 | **13107** |

`psi = (counts - 1638) x 3000/13107 = (counts - 1638) x 0.22889`, i.e. **0.229 psi/count**.

Corrections to the relayed figures: 14745 not 14746, span 13107 not 13108. Both traceable to
using 16384 rather than 16383 as full scale. Immaterial (0.23 psi) but recorded for accuracy.

Confirming detail nobody has stated: **the map's out-max of 3000 at :258 equals the sensor's
full-scale psi.** The author's intent is legible — `pValue` is meant to be pressure in psi,
and the only wrong constants on that line are the in-min and in-max. That narrows the fix
and raises confidence in this reading of the code.

### T2 — `tare` inverts the scale — **CONFIRMED** (but see T0: it is not where the fault starts)

`:312` `tareValue = fValue;` writes the current filtered reading into the map's **in-min**
at :258, while the in-max stays the literal 1023.

At no load `fValue` is the live zero, ~1638 counts — **greater than 1023**. Arduino's `map()`
is `(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min` with no guard on
operand order, so the denominator becomes `1023 - 1638 = -615` and the slope goes negative:
**-4.878 psi/count**. Pressure rising makes the reading fall. At 3000 psi (14745 counts) the
map returns `(14745-1638) * 3000 / -615 = -63937`.

I re-derived this independently and get -63937 against the relayed "about -63,900" — same
number, and the discrepancy is the 14745/14746 count.

Severity: taring is the first thing anyone does to a scale, and the `tare` verb is macro `t`
in the shipped operator config (`serial_bluetooth_terminal_cfg.txt`, `pref_m1_value`). This
is not an edge case; it is the primary workflow.

**Revision 2:** I originally recorded this as the sharpest live defect. It is not — **T0** is.
The fault is present at boot with the stock tare; taring flips its sign rather than creating it.
T2 remains true and remains severe, but leading with it invites a fix that corrects the tare
path and leaves a board that still reads ~9700 at rest.

### T3 — default `tareValue = 241` is below the sensor's live zero — **CONFIRMED**

`:22` `int tareValue = 241;`. The sensor's zero is 1638 counts. 241 is not a valid zero for
this sensor on this board — it appears to be an R3-era empirical value, and it is the source
of the surviving offset described in C3.

Consequence the coordinator flags and I confirm from source: with the sensor unplugged (0
counts) the map yields `(0-241) * 3000/782 = -925` "psi" — reported as a negative weight, not
as a fault. There is no range check anywhere (`grep -n 'constrain\|isnan\|isinf'` → nothing).

### T4 — the 0.5 V live zero is free diagnostics that the firmware discards — **CONFIRMED as an absence**

`:256` reads the pin and `:257-258` filter and map it. Nothing tests `sensorValue` against
any bound at any point. There is no fault state, no fault topic, no fault display.

A 0.5-4.5 V transducer exists so that 0 V is distinguishable from 0 psi. The available and
unused signals are:

```
counts < ~1311  (<0.4 V)  open circuit / unplugged / cut wire / sensor dead
counts > ~15400 (>4.7 V)  short to rail
```

(0.4 V and 4.7 V converted at 16383 full scale: 1311 and 15400 — the relayed 1300/15400 are
the same thresholds rounded.)

**Prior art exists on PLC #1 — port the pattern, never the constant.** Per the coordinator
(citation theirs; I did not read that tree): `plc1 libraries/PhoenixPLC/src/config.h:363`
defines `CURRENT_DEAD_LOOP_MA = 3.5f`, below which the loop is reported as broken rather than
as zero. PLC #4 wants the voltage analogue of exactly that — a floor beneath the ~1638-count
live zero under which the reading is published as NO SENSOR, not as no load. The design is
already proven in the estate; only the constant is machine-specific.

I agree this is the most valuable thing the spec unlocks. Chafed wiring is a leading failure
mode on a tractor, and as written PLC #4 cannot distinguish "no load" from "no sensor" — it
reports a plausible number in both cases. But note it interacts with T3: implementing the
diagnostic requires the correct live zero, so T1/T3 must land first or the thresholds are
meaningless.

### T5 — calibration does not survive a reboot — **CONFIRMED**

`:387` `EEPROM.put(addr, calFactor);` in the `cal` verb.
`:190` `//EEPROM.get(addr, calFactor);` — **commented out** in `setup()`.

The EEPROM is write-only. Every boot reverts `calFactor` to the hard-coded `1.81` (`:23`).
`grep -n 'EEPROM'` returns exactly three lines — the include (:5), the dead get (:190), and
the put (:387) — so nothing else touches it.

`tareValue` is not persisted at all: `grep -n 'tareValue'` shows :22, :258, :273, :312, :358
and no EEPROM path. It reverts to 241 (T3).

So every power cycle loses **both zero and span**, on a machine that is power-cycled every
time the tractor is keyed off. This also means the C3 offset is not a one-time bench problem
that a good calibration fixes — it returns on every start.

### T6 — why this was never caught — **CONFIRMED, and I reached it independently**

`cal` at :386 is `calFactor = float(calWeight) / float(pValue)` — a single-point ratio
through the origin — and :259 multiplies it back in. Constant gain error cancels exactly;
offset error does not. I derived the same result in C3 before receiving this message, from
the algebra rather than from the history, which I take as reasonable corroboration.

The practical shape: it reads correctly at the calibration point on the bench, wrong
everywhere else, and the error looks like drift rather than like a bug.

### T7 — `pValue` integer width — **CONFIRMED as stated, but the framing overstates the hazard**

`:20` `int pValue = 0;` assigned from `map()`, which returns `long`.

On the R4 (RA4M1, 32-bit) `int` is 32 bits and the worst case fits: at 14745 counts with the
default tare, `(14745-241) * 3000/782 = 55641`. No overflow. I get 55641 against the relayed
55646 — again the 14745/14746 count.

Where I would soften it: **the `uno-r3-swserial-lcd` branch as it exists does not overflow.**
That branch has no `analogReadResolution(14)` — I checked — so its readings top out at 1023
and `pValue` maxes at 3000, comfortably inside a 16-bit `int`. The hazard is confined to a
hypothetical 14-bit build on an AVR, which nobody would produce. Worth one line in a porting
note, not a defect.

---

## 4. Constraints for whoever takes the fix gate

These are couplings between defects. Taking one without the other makes things worse.

### X1 — the topic rename is booby-trapped: C2 and C7 must be fixed in the same commit

**CONFIRMED from source.** Adding a machine identity to `topic[]` at :75 to resolve the
collision (C7) *fixes* the `/debug` path and *breaks* the `/weight` path further, because
:330 caps at 10 bytes:

| `topic[]` | `/debug` (30-byte cap) | `/weight` (10-byte cap) |
|---|---|---|
| `arduino` (today) | `arduino/debug` ✅ | `arduino/w` ❌ |
| `plc4/tractor-scale` | `plc4/tractor-scale/debug` ✅ | **`plc4/trac`** ❌ |

`plc4/tractor-scale` would publish weights to `plc4/trac`. **Changing :75 without changing
:331 in the same commit makes the collision worse, not better** — it moves the debug stream
to a namespaced topic while stranding the weight stream on a nine-character stub that no
subscriber will have been told about.

A second ceiling nobody has named: **the 30-byte buffer caps the full topic at 29
characters.** `phoenix-nest/plc4/tractor-scale/debug` (37 chars) truncates to
`phoenix-nest/plc4/tractor-sca`. If the estate convention uses a multi-level prefix, both
buffers at :283 and :326 need resizing, not just the length argument at :330.

**Revision 2 — this is a two-step sequence, not one atomic commit.** The bound at `:330` is
the literal `10`, not `sizeof(topic_temp)`, so it tracks neither the buffer nor the topic
string. A rename therefore does not merely truncate *shorter* — it truncates at a **different
cut point** (`arduino/w` becomes `plc4/trac`), so any subscriber built against the old stub
breaks silently rather than visibly. Correct order:

1. Replace the length argument with `sizeof(topic_temp)` at **both** `:287` and `:330`, and
   size the buffers for the longest topic the convention will produce. This is behaviour-
   preserving for `/debug` and restores `/weight`.
2. *Then* rename `topic[]` at `:75` as a separate change that can no longer truncate.

Doing (2) first, or both in one commit without (1)'s `sizeof` change, leaves the same class of
bug waiting for the next rename.

**The collision is now real, not hypothetical.** With the broker fixed at
`192.168.10.99:1883` (F2), PLC #4 shares a bus with PLC #1, which already publishes there.
PLC #4 currently publishes to bare `arduino/debug` and truncated `arduino/w` on that shared
bus. A machine-identity topic root is no longer a tidiness improvement; it is required — and
it is still booby-trapped, so `:75` and `:331` must change together.

Relayed and unverified by this session: that broker reportedly runs `allow_anonymous true`
with no `password_file` and no `acl_file`. If so, `setUsernamePassword` at `:170` is
decorative on it — PLC #4 connects regardless.

**Withdrawn, 2026-09-01:** an earlier revision said topic discipline was "the only thing
preventing collision with PLC #1". Overstated. PLC #1's only `subscribe()` is on the literal
`plc1/notify` with no wildcard (`mqtt.cpp:1091`, `config.h:205`, coordinator's citations), so
different roots cannot collide and PLC #4 cannot trigger it by accident. The accurate exposures
are (a) `arduino` is the default root in Arduino's own example sketches, so it collides with any
other stock sketch — a risk at machine five, not four; and (b) no ACL, so prefix discipline
protects nothing against a misbehaving publisher, which is an estate gap predating PLC #4.

**Corrected 2026-09-01 — the capture is available now.** An earlier revision said it had to
run on farm1. Wrong: the coordinator re-measured and `mosquitto_sub -h farm1.internal -t
'arduino/#' -v` runs from Saga today. It currently returns nothing, i.e. **the `arduino` root is
unclaimed at present** — so the truncation claim stays inferred only because no board is
publishing, not because the capture is out of reach.

Keep the inverse distinction, because this correction invites the opposite error: Saga reaches
the broker as `farm1.internal` (10.77.77.13, overlay); the board on FOXTROTBRAVO reaches the
same broker as `192.168.10.99` (farm LAN). `SECRET_MQTT_BROKER` must carry the LAN form.

### X2 — the 10-bit → 14-bit port left three constant sets stranded, not one

`c00bafe` (2024-10-27) added `analogReadResolution(14)` and updated none of the constants
calibrated against the old 10-bit scale. C3, T3 and A2 are three symptoms of one change.
A fix that addresses the pressure map (:258) and leaves the keypad ladder (:203-217) has
fixed half a port.

### X3 — T4 depends on T1/T3

The open-circuit and short-to-rail thresholds are expressed in counts against the sensor's
real live zero. Implementing diagnostics before correcting the scaling produces thresholds
that fire on healthy hardware.

---

## 5. Additional defects — not in either list

Ordered by severity. All confirmed from source at `c602655`.

### A1 — the committed operator interface cannot reach this firmware

`serial_bluetooth_terminal_cfg.txt` (added `2041e5b`, 2024-10-03) is an Android
*Serial Bluetooth Terminal* profile (`de.kai_morich.serial_bluetooth_terminal`) targeting a
device named **`HC-05`** at `00:22:09:01:01:EE`, with macros for `tare`, `debug`, `+`, `-`,
`sum`, `read`, `clear` — the exact verb set the firmware implements.

An HC-05 is a UART bridge. On the UNO R4 WiFi, D0/D1 is **`Serial1`**; `Serial` is the USB
CDC port. The firmware reads only `Serial` (:296). So every macro in the committed operator
config is unreachable, and C4 additionally prevents boot when no USB host is present.

This worked before the port: on `origin/uno-r3-swserial-lcd`, `Serial` *was* the D0/D1 UART
(that branch also carries a `SoftwareSerial swSerial(2, 3)` at 4800 baud for the LCD). The
AVR→R4 port silently moved the console from the Bluetooth adapter to a USB socket that is
not present in the field. `Serial.begin(9600)` at :144 still matches the HC-05 default baud —
a leftover that makes the intent unambiguous.

**Net effect: the machine as written has no working operator input path in the field.**
Unverified: whether an HC-05 is actually fitted to this build. That is an open question.

### A2 — the LCD keypad ladder is 10-bit constants read at 14-bit resolution

`:201-217`. The thresholds are 60/200/400/600/800. I confirmed via
`git show origin/uno-r3-swserial-lcd:ArduinoHydraulicScale.ino` that these are **byte-identical
to the R3 branch** — 10-bit ladder values — and `analogReadResolution(14)` (:185) applies to
A0 as well as A1.

Readings scale by ~16x, so:

| button | ~10-bit | ~14-bit | matches |
|---|---|---|---|
| Right | ~0 | ~0 | `x < 60` ✅ |
| Up | ~100 | ~1600 | nothing ❌ |
| Down | ~256 | ~4100 | nothing ❌ |
| Left | ~410 | ~6560 | nothing ❌ |
| Select | ~640 | ~10240 | nothing ❌ |
| none | ~1023 | ~16383 | nothing |

Only **Right** still registers. There is no final `else`, so when nothing matches, the LCD
holds whatever label was written last — including on release, so the display latches "Right"
indefinitely after one press. Same root cause as C3 (see X2).

### A3 — out-of-bounds array write in the `-` verb

`:26` `int rT[rTsize];` with `rTsize` = 50 (`:25`).
`:318` `if (rTindex < rTsize)` guards `+`, and `:323` increments — so after the 50th `+`,
`rTindex == 50`.
`:342-348`:
```c
    else if (inputString == "-") {
      if (rTindex >= 0 ) {
        rT[rTindex] = 0;
```

The guard `rTindex >= 0` is always true (`rTindex` is `int`, initialised 0 at :27, never
decremented below 0 by :345-347). With a full buffer, `rT[50] = 0` is a **write one element
past the end of the array**, corrupting whatever global follows it in the layout —
`rTindex` (:27) and `rTsum` (:28) are the adjacent candidates.

Separately, the logic is off by one even in bounds: it zeroes the slot *above* the last
entry, then decrements. The last recorded value is never actually cleared — it is merely
made unreachable until the next `+` overwrites it.

**Reachable through the primary workflow, and it survives onto `main` verbatim** (`main`
guard `:158`, increment `:164`, OOB write `:174` — same shape, same values). `+` records a load
and `sum` totals them, so on a front-end loader fifty buckets is a working day. This is the
second finding whose severity changed once `main :70` told us the machine is a loader: it moves
from theoretical overflow to something that will happen.

**The missing guard is the upper bound, not the lower.** `rTindex >= 0` was never the defect —
it is useless, since `rTindex` is never negative. The defect is that `-` never tests
`rTindex < rTsize` while `+` does. Note that simply tightening the test to `rTindex > 0` does
**not** fix it: at 50, `50 > 0` is true and `rT[50] = 0` still executes. The index must be
decremented *before* it is used. The remedy and its side effect are recorded in `PORT-PLAN.md`
§3; the short form is `if (rTindex > 0) { rTindex--; rT[rTindex] = 0; }`.

**Do not characterise the corruption.** Declaration order puts `rTindex` (`:27`) and `rTsum`
(`:28`) after `rT[]` (`:26`), but linker layout does not follow declaration order and nobody has
established what occupies that address. Both candidates are self-masking — zeroing `rTindex`
silently resets the buffer, zeroing `rTsum` silently zeroes the total — and in either case the
instrument returns a wrong number with no crash to notice. It is a two-line reorder, not a
layout investigation.

### A4 — `clear` does not clear

`:379-384` zeroes `rT[]` across all `rTsize` elements but never touches `rTindex`.
Consequences: `sum` (:368-378) still iterates `i < rTindex` and prints that many zero rows;
`+` resumes at the stale index rather than at 0; and if the buffer had been filled,
`rTindex` remains 50 and **`+` reports "Buffer Full" forever** (:339) with no way to recover
short of a reboot — which then loses the calibration (T5).

The one verb whose entire purpose is to reset the tally does not reset it.

### A5 — telemetry topic semantics are inverted, and the payload is bare

The periodic 5 s publish (`:265-294`) — the real telemetry stream — goes to **`arduino/debug`**.
The operator-initiated record (`:325-336`) — the deliberate act of recording a weight — goes
to `arduino/weight`, truncated to `arduino/w` (C2). The routine stream is labelled debug and
the significant event is labelled with a stub.

The payload in both cases is an unlabelled integer (`:290-292`, `:333-335`): no units, no
timestamp, no device identity, no sequence number. Neither publish sets retain and both are
QoS 0 by default. A subscriber cannot tell a fresh value from a stale one, which machine
sent it, or whether the number is pounds, kilograms or psi.

### A6 — serial command framing corrupts back-to-back commands

`:296-307` drains the entire RX buffer in one pass. `stringComplete` is set when `'\n'` is
seen (:299-300) but the loop **keeps appending subsequent bytes to the same `inputString`**;
nothing breaks out. Two commands arriving in one burst concatenate — `tare` + `read` becomes
`"tareread"` — and match no branch, so they are silently discarded (there is no `else` on the
comparison chain at :311-388).

This is not theoretical: the shipped config's `debug` macro is `pref_m2_value: "debug\n    "`
— four spaces *after* the newline. Depending on how the bytes land across loop iterations,
those spaces either make the command `"debug    "` (no match) or prepend to the next command
and poison it. Either way one of the seven documented macros does not work as configured.

Also here: `inputString` grows without bound. `:186` `inputString.reserve(200)` reserves but
does not cap, and `:305` appends unconditionally, so a long garbage line grows the String on
the heap until allocation fails.

### A7 — `cal` has no divide-by-zero or sanity guard

`:386` `calFactor = float(calWeight) / float(pValue);`

At or below tare, `pValue` is zero or negative. Zero gives `inf`, which is written to EEPROM
at :387 and then multiplied at :259 into `int outputValue` — a non-finite float converted to
`int` is undefined behaviour. A negative `pValue` silently inverts the scale factor and is
also persisted. There is no confirmation prompt, no readback, no plausibility range, and no
serial acknowledgement at all — `cal` is the only verb that prints nothing on success.

### A8 — the LCD never displays a weight

`setup()` writes the IP address and `"Press Key:"` (`:191-195`). `loop()` writes only the
keypad label at column 10, row 1 (`:202-217`). **Nothing anywhere writes `outputValue` to the
display.**

The brief describes a local LCD readout; the baseline does not implement one. This is a scope
gap rather than a defect, but its effect is that the machine has no local indication of weight
whatsoever — the LED at :261 shows only the degenerate `isFluctuating()` result from C1, and
the only weight output is `read` over a serial console that cannot be reached (A1, C4).

### A9 — MQTT credentials are sent in the clear from a mobile machine

`:74` `int port = 1883;` and `:170` `mqttClient.setUsernamePassword(username, password);`,
with `WiFiClient` (not `WiFiSSLClient`) at :70. The comment block at :64-68 documents the TLS
alternative and it was not taken.

The credentials themselves are not in the tree — `arduino_secrets.h` is gitignored (`.gitignore`)
and absent, which is correct. But a tractor is mobile and will associate to whatever AP it can
reach; the username and password go out in plaintext on every connection attempt, on whatever
network that turns out to be. This bears on the broker question, not on the fix gate.

### A10 — smaller items, grouped

- **`pin_BL` is declared and never used.** `:132` `const int pin_BL = 10;` — never passed to
  `pinMode` or `digitalWrite`. The backlight is uncontrolled; the pin is left floating.
- **The entire button1 debounce block is commented out** (`:222-251`), leaving `button1Pin`,
  `button1State`, `lastButton1State`, `lastDebounceTime1` and `debounceDelay` (:80-88) as dead
  globals, and `pinMode(button1Pin, INPUT)` at :137 configuring a floating input with no pull.
  The commented block is also the only thing that ever *cleared* the LED after a press.
- **`String` churn in the hot path.** `:284-291` and `:327-334` construct, assign and
  concatenate `String` objects every 5 s and on every `+`. Heap fragmentation over a long
  shift is a real risk on a 32 KB-RAM part with no allocator diagnostics.
- **`outputValue` is `int`** (`:21`) assigned from the float product at :259, truncating and
  with no clamp at either end.
- **`char oV[10]` payload buffer is latent, not broken.** `:289`/`:291` and `:332`/`:334` pass
  10 against a 10-byte buffer, so unlike :330 the bound is correct. But `outputValue` is a
  32-bit `int`, and `String()` of one can reach 11 characters against a 9-character capacity.
  Only the 4-5 digit physical range keeps it safe today — and T2's inverted readings
  (-63937) already consume 6 of those. Recorded as latent; verified by the coordinator, not
  re-derived here.
- **`delay(2)` at `:396`** blocks the whole loop including `mqttClient.poll()` (:219) and the
  serial drain (:296). Harmless at present rates; it becomes a constraint if a keepalive or a
  reconnect backoff is ever added.
- **`debug_flag` defaults to `true`** (`:7`), so an unattended board with no console attached
  is generating serial output nobody reads. Combined with C4 that is moot today; it matters
  once :145 is removed.

---

## 6. What I could not verify, and what would settle it

- **Nothing was compiled, flashed or observed.** Claims about `map()`, `String::toCharArray()`
  and the R4 `Serial` object rest on the documented core implementations, not on the installed
  core sources. A build against the real toolchain would settle C1-C3 and A3 cheaply.
- **The wire behaviour of the topics (C2, C7, X1)** is inferred. `mosquitto_sub -t 'arduino/#' -v`
  against `192.168.10.99` would settle it in seconds, but must be run **on farm1** — the host
  is not reachable from Saga. Not attempted from this session.
- **The broker's reachability and configuration (F2, X1)** were relayed by the coordinator.
  I did not probe the farm LAN and did not read the other PLC trees.
- **Whether `FOXTROTBRAVO` routes to `192.168.10.0/24`** cannot be established from this tree.
  If it does not, every network finding above is moot because the board never reaches the
  broker's subnet at all. Raised as a commissioning check in `BRIEF.md`.
- **Whether an HC-05 is physically fitted (A1)** cannot be determined from the repo.
- **The transducer's exact part marking** was not supplied — only the 0-3000 psi / 0.5-4.5 V
  spec, relayed from Alex. All T-series arithmetic is only as good as that spec.
- **That PLC #4 and PLC #1 share a transducer is now open, not given.** Alex said "same part
  PLC #1 uses". Per the coordinator, PLC #1 reads a **4-20 mA current loop** on an AFX00007
  (`config.h:360-363`) — a different physical interface, not a differently-scaled voltage — and
  `config.h:366` still records its pressure full scale as "TBD against the relief-valve
  set-point", so it is not established that PLC #1 has one fitted at all. A 0.5-4.5 V
  ratiometric unit and a 4-20 mA unit are different variants even within one family. This wants
  two real part numbers from Alex and must not be resolved by inference. Citations are the
  coordinator's; I did not read that tree.

  **The one thing genuinely shared is the engineering full scale**: `config.h:367`
  `PRESSURE_FULL_SCALE_PSI = 3000.0f` against this firmware's map out-max of 3000 at `:258`.
  Same number, unrelated means — and that coincidence is precisely what would make a careless
  copy of a conversion constant look correct.
- **The R4's default analog reference** (T0) is assumed to be VCC ≈ 5 V and was not confirmed
  against the core sources. The `read` verb settles it on real hardware with no code change.
- **The estate broker's configuration** (anonymous access, no ACL) was relayed by the
  coordinator and is recorded in `BRIEF.md` as an argument, not a verified fact. I did not and
  will not read the other PLC trees.
- **The hydraulic geometry** is unrecorded anywhere, so the offset magnitude in C3 cannot be
  quantified and the plausibility of single-point calibration cannot be assessed.

## Summary

| # | Claim | Verdict |
|---|---|---|
| F1 | No watchdog anywhere | **Confirmed** — governs C4/C5/C6 |
| F2 | Fixed local broker under a mobile machine | **Design finding** — makes C4/C5/C6 certainties, not risks |
| C1 | `isFluctuating()` assignment not comparison | **Confirmed** — degenerates to a magnitude threshold |
| C2 | `+` publish truncates to `arduino/w` | **Confirmed** |
| C3 | `map()` in-max 1023 vs 14-bit | **Confirmed as fact; consequence corrected** — gain cancels, offset survives; 16.8x not 16x |
| C4 | `while (!Serial)` blocks forever | **Confirmed** — most severe line; N20 analogy corrected |
| C5 | Blocking WiFi retry, terminal MQTT failure | **Confirmed** |
| C6 | No reconnect in `loop()` | **Confirmed** — and failure is silent, reported as "OK" |
| C7 | Bare `"arduino"` topic root | **Confirmed** — client ID is unset and worse |
| T1 | Scaling constants 1638 / 14745 / 0.229 psi-per-count | **Confirmed**, two counts corrected |
| T0 | Reads ~9700 at no load, out of the box | **Confirmed** — supersedes T2 as the lead; testable by powering the board |
| T2 | `tare` inverts the scale | **Confirmed** — flips the sign of T0's fault, does not create it |
| T3 | Default tare 241 below live zero | **Confirmed** |
| T4 | Live-zero diagnostics discarded | **Confirmed** as an absence |
| T5 | Calibration not restored from EEPROM | **Confirmed** — and tare not persisted at all |
| T6 | Single-point cal masks the gain error | **Confirmed**, independently derived |
| T7 | `pValue` integer width | **Confirmed; hazard overstated** — R3 branch does not overflow |
| X1 | Topic rename booby-trap | **Confirmed** — plus an unnamed 29-char ceiling; collision now real, not hypothetical |
| A1 | Committed HC-05 operator config unreachable | **New** |
| A2 | Keypad ladder broken by 14-bit port | **New** |
| A3 | Out-of-bounds write in `-` | **New** |
| A4 | `clear` does not clear | **New** |
| A5 | Topic semantics inverted, payload bare | **New** |
| A6 | Back-to-back commands corrupt | **New** |
| A7 | `cal` divide-by-zero unguarded | **New** |
| A8 | LCD never shows a weight | **New** (scope gap) |
| A9 | Plaintext credentials from a mobile machine | **New** |
| A10 | Assorted smaller items | **New** |

**Claims I judge you got wrong or loose:** C3's consequence (the 16x error is cancelled by
calibration; the offset is the live defect), C4's PLC #3 analogy (right family, wrong
mechanism — corrected per the coordinator), and T7's severity (the R3 branch does not
overflow). Everything else confirmed as reported.
