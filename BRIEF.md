# PLC #4 — tractor-mounted hydraulic scale

**Status, 2026-09-02: DEPLOYED AND RUNNING.** The firmware is on the tractor, associated to
FOXTROTBRAVO, publishing to farm1 with a fitted transducer and a real calibration. The audit
below describes the baseline it started from; the port, the HMI and the field results are in
`PORT-PLAN.md` and `findings/`.

**Superseded, 2026-09-18.** The two fixes named here — `f7af2fc` (calibration validity) and
`b10ec94` (over-range counters) — were flashed long ago and are ancestors of what is running.
The `cf` caveat no longer applies. **The board on the tractor is running `c601d792`**, which it
publishes as `build` in every status frame, so the running version is a thing to read rather
than a thing to remember.

**The transducer spec is confirmed against hardware** — see
`findings/2026-09-02-field-commissioning.md`. The scaling constants are no longer provisional.

**Original status:** baseline audited, nothing fixed. Firmware changes gated on Alex's approval.
**Baseline:** `c602655` on `plc4/baseline`, branched from `origin/r4_wifi_mqtt`.
**Caveat, added 2026-09-01:** `origin/main` already fixes eleven of the audited items and has
no networking at all. Read **open question 12 before any fix-gate planning** — the likely work
is a forward-port of the MQTT layer onto `main`, not a fix list against this baseline.
**Audit:** `findings/2026-09-01-plc4-baseline-audit.md`
**Port scope:** `PORT-PLAN.md` — approved base was `origin/main`. **Superseded, 2026-09-18: the
port is implemented and on `main`.** `PORT-PLAN.md` is kept as the record of what was decided
and why, not as outstanding work.

## Scope

PLC #4 reads a hydraulic pressure transducer as a weight, displays it locally, and publishes
it to MQTT over WiFi. It is the fourth machine in the Phoenix Nest estate; PLC #1 (HPU/VFD
coordinator), #2 (log splitter) and #3 (saw feed actuator) live in their own trees and are
coordinated from `~/greybeard-plc-coordination`.

What distinguishes PLC #4 from the other three, and drove most of what follows: **it is
mobile, it is unattended, and it has no watchdog.** The other three are fixed installations
on an Opta whose IWDG provides a recovery floor. This one drives out of radio range with
nobody watching a console, and a hang is permanent and silent.

**Superseded, 2026-09-18: it has a watchdog, and that sentence is the reason it got one.** The
risk framing above still describes the machine correctly — mobile, unattended, out of range —
but the recovery floor it says is missing has been built. `WDT.h` is armed at
`WDT_TIMEOUT_MS = 5000` (the part's ceiling is 5592), refreshed at eight sites, and crucially
**refreshed from a timer ISR only while `loop()` is still ticking** — a heartbeat watchdog
rather than the classic anti-pattern of an ISR that refreshes unconditionally and defeats the
whole mechanism. Reset cause is decoded from RSTSR0/RSTSR1 and published as `reset`, so a
watchdog bite is distinguishable from a brownout after the fact. See
`src/ArduinoHydraulicScale.cpp:146-232` and `:329-348`.

**Also superseded, 2026-09-18:** this gate covered orientation and a baseline audit only, and
at the time no firmware had been changed, nothing compiled and no board tethered. All three
are now false. The firmware has been through a forward-port, an ADC change and an HMI change;
it is built with `arduino-cli` and flashed via `farm1`; and the board is in service on the
tractor. **Claims about runtime behaviour can now be measured, and should be** — say which of
yours are read from source and which are read from the machine.

## Decisions (settled — not open parameters)

**Broker: the estate broker at `192.168.10.99:1883`.** Decided by Alex, 2026-09-01. A fixed
address on the farm LAN, not a cloud broker — so it is reachable only while the tractor is in
range.

**Two vantage points, one broker — do not confuse them.** The board, on FOXTROTBRAVO and on the
farm LAN, reaches it as **`192.168.10.99`**; that is what `SECRET_MQTT_BROKER` must contain.
Saga reaches the same broker as **`farm1.internal` (10.77.77.13)**, an overlay address the board
has no route to. Corrected by the coordinator 2026-09-01 — an earlier note in this workspace
said the capture had to be run on farm1, which was wrong. PLC #1 already publishes to it. Reportedly `allow_anonymous true` with no password file
and no ACL, which makes `setUsernamePassword` (:170) decorative there and leaves **topic
discipline as the only thing preventing collision with PLC #1**.

**Topic root: `plc4/jd5055e/`.** Decided by Alex, 2026-09-01. Role prefix for estate
consistency, equipment segment for the identity already in the source at `main :74`. Both
`/weight` (19 ch) and `/debug` (18 ch) fit inside the 29-character cap, so no buffer resize is
forced — but the `sizeof` fix at `:330` is still required or weights publish to `plc4/jd50`,
outside the new tree. See `PORT-PLAN.md` §4.

**Calibration model: single-point is valid — the loader is weighed at one fixed boom height.**
Decided by Alex, 2026-09-01. This closes the largest open risk in the investigation: the
moment-arm concern is moot, `calFactor` as a ratio through the origin is a correct model, and
the psi-to-weight path can be designed against single-point calibration. Supersedes the
geometry half of open question 5.

**PLC #4 has its own repository.** `Alex-Pennington/greybeard-plc4-tractor-scale`, private,
created 2026-09-01 — matching the five sibling PLC repos. `origin` now points there;
`upstream` is Alex's public `ArduinoHydraulicScale`. **Never push to `upstream`.** Branches
`plc4/baseline` and `plc4/mqtt-port` are pushed. `main` on the new repo is **not yet seeded** —
see open question 11. **Superseded, 2026-09-18: `main` is seeded and is the real mainline.**
Note also that the "never push to `upstream`" rule above is still correct, but the rule about
`origin` that once sat beside it was withdrawn — pushing to `origin` is fine. See `CLAUDE.md`,
which names what each remote points at, precisely because a repository does not version its
own remote names and a stale rule about one reads exactly like a live rule.

**Base: forward-port the MQTT layer onto `origin/main`.** Decided by Alex, 2026-09-01. Not 26
fixes to `r4_wifi_mqtt`. See `PORT-PLAN.md` and open question 12.

**Network loss: keep weighing, buffer, flush on reconnect.** Decided by Alex, 2026-09-01. The
scale stays fully usable offline; readings queue and publish when the broker returns. This
supersedes open question 1 and is the design spine of the port — neither branch has anything
like it.

**WiFi SSID: `FOXTROTBRAVO`.** Decided by Alex, 2026-09-01. The PSK is `<set>` — not recorded
here, not in any tracked file, and not requested. `arduino_secrets.h` stays gitignored;
`arduino_secrets.h.example` documents the five required defines with placeholders.

Together these two decisions raise the severity of the baseline rather than settling it. A
fixed local broker under a mobile machine turns the three blocking failure paths from risks
into certainties — see audit finding **F2**. As built, the first trip out of the yard ends
with a blank LCD and a dead scale until someone power-cycles it in range.

## The machine, and where the hardware actually is

**John Deere 5055E with an M520 front loader.** Recorded from Alex 2026-09-01. The firmware
already announced the tractor (`main :74`, `JD5055e OK`) and that it was a loader scale (`:70`),
but the loader model was written down nowhere until now.

**The hardware is NOT installed on the tractor. It is on the bench, with no transducer
connected.** Every reading taken so far is bench work on a floating analog input and none of it
is a hydraulic measurement.

**Superseded, 2026-09-18: it is on the tractor and the readings are real.** A Rosemount 4-20 mA
transmitter is fitted to the hydraulic circuit, the loop is live, and the calibration is stored
and reported as `cal:"ok"`. The pressure reading was checked against an independent gauge at
roughly 500 psi and agreed to about 0.13% of span. Every sentence in this section that begins
"on the bench" now describes history, not the machine.

**The previous transducer failed by leaking at the electrical connection** — oil worked through
the body and out of the cable entry, which is an internal element/seal failure rather than a
thread or installation fault. See `notes/2026-09-01-transducer-failure-and-selection.md`; oil
travels up a cable, so the harness and whatever it terminates in want checking before reuse.

**Superseded, 2026-09-18 — closed by the front end changing, not by fitting the resistor.**
The defect below is real for a *voltage* transducer on a high-impedance analog input, which is
what this machine had when it was written. It no longer has one. A 4-20 mA loop cannot float:
a broken loop carries no current, so the burden resistor reads 0 mA, which lands below
`LOOP_DEAD_MA` (3.5 mA) and is reported as a dead loop rather than as a valid pressure. That is
NAMUR NE43 live-zero banding and it is what a pull-down was trying to approximate. Do not fit
the resistor; do not read the paragraph below as outstanding work.

**Open defect, hardware: the transducer input needs a pull-down resistor or the NO SENSOR
detection does not work.** Demonstrated on the bench — with nothing connected, the input reads
4401 counts and the
firmware reports `sensor = ok`. A floating CMOS input does not sit at 0 V. Until a 10k-47k
pull-down to GND is fitted, `sensor = ok` means "in range", not "a sensor is present", and a cut
sensor wire on the tractor would read as a valid pressure.

## Hardware — verified

Read from the source at `c602655` unless marked otherwise.

**Board — Arduino UNO R4 WiFi.** Three independent signals agree: the `ARDUINO_UNOR4_WIFI` /
`WiFiS3` arm of the include block (:55-57), `analogReadResolution(14)` (:185), and `D2`/`D13`
pin naming (:80-81) which the AVR core does not define.

**Transducer — 0-3000 psi, 0.5-4.5 V ratiometric, on A2 as wired** (confirmed by Alex
2026-09-01; briefly recorded as A1 in error — `main` had A2 all along).** Spec direct from Alex via the
coordinator; **no part number supplied and no datasheet read.** Read with plain `analogRead`
(:256); no external conditioning is recorded anywhere in the repo. At AVCC = 5 V and 14-bit
resolution this gives a live zero of **1638 counts**, full scale of **14745 counts**, a span
of 13107, and **0.229 psi/count**. The firmware's map at :258 uses none of these numbers.

Alex described this as the same part PLC #1 uses. **Treat that as open, not given** — see
open question 6. Per the PLC coordinator (citation theirs; this session did not read that
tree), PLC #1 reads a **4-20 mA current loop** on an AFX00007, not a scaled voltage — a
different physical interface, not a different constant. What the two machines do share is the
engineering full scale of 3000 psi, and that coincidence is exactly what would make a careless
copy of a conversion constant look correct. **Do not assume the sensors interchange.**

**Superseded, 2026-09-13: the front end is a 4-20 mA current loop, not the 0.5-4.5 V
ratiometric transducer above.** Transmitter: **Rosemount 1151, 0-3000 psi, 2-wire 4-20 mA**.
Burden resistor: **150 ohm, 0.1%, <=25 ppm/degC, across A2 to GND**. Loop supply: a
**dedicated 24 V supply, negative bonded to the board's GND**. This also answers open
question 6 for PLC #4's own part number, independent of whatever PLC #1 turns out to use.
The reasoning behind each of these figures — why 150 ohm, why 0.1%, why a dedicated supply
rather than the board's own rail — is in `MA-LOOP-DESIGN.md` and
`notes/2026-09-13-4-20ma-loop-decisions.md`, not restated here.

**Superseded again, 2026-09-16: the ADC changed, not the reasoning above.** This branch moved the
loop reading from the board's 10-bit AVCC-referenced ADC to an MCP3421 with its own 2.048 V
reference. 150 ohm was the right burden for the AVCC-referenced ADC and stays right in that
context; it was retired by this hardware change, not by an error in the 2026-09-13 figure. Fit
**68 ohm, 0.1%, <=25 ppm/degC**, and note it is now **across the MCP3421's differential input,
not A2 to GND** — the ADC reading the loop is no longer the board's own. Live source of truth:
`include/loop_scaling.h`.

**Display — 16x2 LCD + keypad shield.** `LiquidCrystal(8,9,4,5,6,7)` (:126-133), backlight on
pin 10 (declared, never driven), button ladder on A0 (:201). The ladder thresholds are 10-bit
values read at 14-bit resolution, so only "Right" registers. The LCD never shows a weight.

**Superseded, 2026-09-18: the LCD and the operator interface both changed.** The 16x2
LCD-plus-keypad shield is gone. The current source carries a 20x4 LCD on the Qwiic bus,
always showing the weight, behind a clear coverplate as a read-only readout — see
`references/01_controller_card.md`. The board's discrete buttons are removed as well: the
outdoor enclosure needs no holes for a keypad, five holes being five places for water to get
in. Every operator action now arrives over MQTT from a phone app, on `plc4/jd5055e/cmd`,
through the same handler the console uses so the two paths cannot diverge. The serial and
Bluetooth console keeps every verb it had, as the service and recovery path with no broker
in reach. See `OPERATOR.md` for the full command table and the MQTT command contract.

**Network — MQTT over WiFi** to `192.168.10.99:1883` on SSID `FOXTROTBRAVO` (see Decisions),
`ArduinoMqttClient`, port 1883 plaintext (:74), credentials from
`arduino_secrets.h` (`SECRET_SSID`, `SECRET_PASS`, `SECRET_MQTT_BROKER`, `SECRET_MQTT_USER`,
`SECRET_MQTT_PASS`). That header is gitignored and absent from the tree, which is correct.
Topic root is the literal `"arduino"` (:75); the client ID is unset (:167 commented out).

**Operator interface — ANSWERED, see open question 4.** The repo commits an Android Bluetooth
terminal profile targeting an **HC-05** adapter with macros for all seven serial verbs. An
HC-05 is a UART device, which on the R4 is `Serial1`; at baseline the firmware read only
`Serial`, the USB CDC port, so whether an HC-05 was actually fitted was unknown. It is: the
board announces itself as `JD5055e OK` on the Bluetooth console at every power-up
(`OPERATOR.md`), and the console is now the service and recovery path beside the MQTT phone
app — see the superseded note above.

**Serial verbs:** `tare`, `cal`, `+`, `-`, `read`, `sum`, `clear`, `debug`. Calibration is a
single-point ratio against `#define calWeight 530` (:3), written to EEPROM at `addr = 10`
(:387) — and never read back, because the matching `EEPROM.get` at :190 is commented out.

## History worth knowing

`c00bafe` (2024-10-27, "button1, fluctuations") is the AVR→R4 port. It added WiFi, MQTT,
`analogReadResolution(14)` and `while (!Serial)` in one commit, and updated **none** of the
constants calibrated against the old 10-bit ADC. Three separate defect clusters trace to that
single omission. `origin/uno-r3-swserial-lcd` is the pre-port line and is useful as a control
when deciding whether a constant is a deliberate choice or a leftover.

## What is still open, 2026-09-02

**Superseded, 2026-09-18.** Items 1, 2 and 5 below are closed, and not by anyone doing the work
they ask for — they were closed by the front end changing. The transmitter is now a known part
with a known span (Rosemount, 4-20 mA, 0-3000 psi), so item 1's hunt for a part marking is
moot; a current loop cannot float, so item 2's pull-down is moot (see the superseded note under
*The machine*); and the units are settled — **`w` is pounds** once `cal` reads `ok`. Item 3, the
snubber, is a physical fitting and this file cannot tell you whether it was fitted. Item 4's
3.6% gap belonged to the old voltage front end; the current one was checked against an
independent gauge at ~500 psi and agreed to about 0.13% of span.

The original list, kept because the reasoning in it is still worth reading:

1. **The part marking** off the failed transducer — the only route to a datasheet, and the only
   way to confirm the *span*. The zero is confirmed; single-point calibration hides a span error
   entirely, so no amount of weighing can test it.
2. **A pull-down resistor** on the transducer input. Without it a disconnected input floats
   mid-rail and reports `sensor: ok`, so the fault detection cannot work.
3. **A pressure snubber** on the transducer port. Now evidenced rather than precautionary: the
   circuit was observed reaching the ADC rail during normal work, and the previous unit failed by
   internal leakage — the failure overpressure produces.
4. **The 3.6% calibration gap.** A 530-unit weight reads 511. Leak-down is eliminated; boom
   position is the leading candidate. Test: lower fully, raise to the mark, read.
5. **Units.** `calWeight` is 530 with no unit recorded anywhere. Carried unitless downstream.

## Open questions for Alex — as originally raised

Sharpest first. Nothing below is guessable from the repo, and several of them block work that
would otherwise be straightforward.

### 1. What should the machine do when the network is gone? — **ANSWERED, see Decisions**

This is the question that shapes the whole restructure, and it should be answered before any
of the individual network defects are fixed. A tractor is mobile and will spend most of its
working life out of AP range. When WiFi or the broker is unreachable, should PLC #4:

- keep weighing locally and buffer readings for later upload,
- keep weighing locally and drop them,
- or refuse to operate and indicate a fault?

**Alex ruled 2026-09-01: keep weighing, buffer, flush on reconnect.** The network is entirely
optional to the weighing path. What remains open under that ruling — queue location, depth,
overflow policy, and whether a buffered reading carries a timestamp — is scoped in
`PORT-PLAN.md` §1 and is for Alex, not for us.

### 2. Does `FOXTROTBRAVO` actually route to `192.168.10.0/24`? — **ANSWERED by deployment**

**The board associates and publishes, so the routing question is settled in the only way that
matters.** What has changed since this was written is which vantage point can see it: the board
reaches the broker as `farm1`, and **Saga can no longer reach the board's network directly** —
the VPN sees to that, deliberately. Saga reaches the same broker as `farm1.internal`. Whether
`mqtt.firewood.ltd`, `192.168.10.99` and `farm1.internal` are all one host is genuinely not
settled here and is tracked as issue #28; do not assume it from this file.

The original reasoning follows.

A commissioning check, and it gates everything else on the network side. The board must reach
the broker's subnet from that SSID. Nothing in this tree can establish it — we have no path to
the farm LAN and did not probe for one.

If it does not route, every network finding in the audit is moot in a way that matters: the
board would never reach the broker regardless of how the reconnect logic is written, and the
blocking `WiFi.begin` at :152 would hang on a *successful* association. Worth confirming before
any network fix work is scoped.

The confirming test for the topic collision — `mosquitto_sub -t 'arduino/#' -v` against
`192.168.10.99` — has the same constraint: it must run on farm1, not from Saga. Not attempted
from this session.

### 3. What is PLC #4's machine identity, and what topic convention do #1-#3 already use?

The topic root is a bare `"arduino"` with no machine identity, and the MQTT client ID is unset
so the library generates a fresh one on every reboot. Both need a convention before they can
be fixed. This is now urgent rather than tidy: PLC #1 publishes to the same broker, so the
collision is real, not hypothetical. And the fix must be **sequenced, not atomic**: the
length bound at :330 is a literal `10` that tracks neither the buffer nor the topic, so a
rename re-truncates at a *different* cut point rather than merely a shorter one. Fix the bound
to `sizeof` at both sites first, then rename (audit X1). We need the estate's existing naming
scheme, not one we invent.

**Correction, 2026-09-01 — the PLC #1 hazard is adjacent, not a collision.** An earlier note
here said topic discipline was the only thing preventing collision with PLC #1. That was
overstated and is now withdrawn. Measured by the PLC coordinator: `mqtt.cpp:1091` is the only
`subscribe()` in that file, on `TOPIC_NOTIFY = "plc1/notify"` (`config.h:205`) — a literal with
no wildcard. **PLC #4 cannot reach it by accident.** Different roots; they cannot collide.

The real exposures, stated precisely so they survive being checked:

- **`arduino` is the default root in Arduino's own example sketches.** It collides with any
  other stock sketch joining this broker. That is a live risk at machine five, not at four —
  but it is why the rename is worth doing now rather than later.
- **No ACL on the broker, so prefix discipline protects nothing against a misbehaving
  publisher.** This is an estate-level gap that predates PLC #4, and a fourth publisher does
  not change the mechanism. Ours to note, not ours to fix.

### 4. Is an HC-05 physically fitted to this build, and what is the intended field interface? — **ANSWERED**

At baseline the repo committed a complete Bluetooth terminal configuration for an HC-05, and
the firmware could not read it (audit A1) — so whether one was fitted, and what "fix the
operator interface" even meant, was open.

**It is fitted, and it is the console's radio path.** The board announces itself as
`JD5055e OK` on the Bluetooth console at every power-up (`OPERATOR.md`). The field interface
is now settled on two paths rather than one: the console (USB and Bluetooth together) as the
service and recovery path with every verb, and a phone app over MQTT as the normal operator
path, now that the five physical buttons are gone. See the superseded note under *Hardware —
verified* and `OPERATOR.md`.

### 5. What does "weight" mean on this hydraulic circuit? — **geometry half CLOSED, see Decisions**

Cylinder bore, rod diameter, lever geometry, and which side of the circuit is tapped. None of
it is recorded anywhere in the repo.

**Alex ruled 2026-09-01: the loader is weighed at one fixed boom position, so single-point
calibration is a valid model.** The moment-arm worry is closed. What remains open here is only
the units question below.

The single-point `cal` empirically absorbs the geometry into `calFactor`, so the
machine can be made to read correctly **without** knowing the bore. What the geometry is
needed for is different and still important: judging whether a single-point calibration
through the origin is even valid here (seal friction, hysteresis, and dead weight of the
implement all make the pressure/load relationship non-proportional), and sanity-checking a
reading against physics rather than against another reading.

Related and small: **is `calWeight 530` pounds or kilograms?** Nothing in the repo says, and
the MQTT payload carries no units either.

### 6. Two real part numbers, please — PLC #4's transducer and PLC #1's

You told us PLC #4's is "the same part PLC #1 uses". The coordinator has since established that
PLC #1's channel is a 4-20 mA current loop, not a 0.5-4.5 V ratiometric output. Those are
different variants even within one product family, and it is not established that PLC #1 has a
pressure transducer fitted at all.

**Corrected 2026-09-01:** an earlier note here read `config.h:366`'s "TBD" as meaning PLC #1
might have no transducer fitted. That was wrong — the TBD is about the *scaling*, not the
sensor. Per the coordinator, `plc1/sensors/analog` shows `saw_pressure_3000 valid:true` (and
`flow valid:false`, which is a working demonstration of exactly the dead-loop detection we want
to port). PLC #1 does have transducers fitted. **The part-number question stands unchanged** —
a 4-20 mA unit and a 0.5-4.5 V ratiometric unit are still different variants.

This is not academic. Every scaling number in the audit's T-series rests on 0.5-4.5 V
ratiometric being correct for the unit actually bolted to this tractor. A part marking off the
sensor body, and a datasheet fetched against it, moves the whole T-series from arithmetic on a
relayed spec to arithmetic on a datasheet. We will not resolve it by inference.

### 7. How is the transducer powered?

Ratiometric output only cancels supply variation if the sensor and the ADC share a reference.
If the sensor is fed from the Arduino's 5 V rail, tractor supply sag cancels out and the T1
constants hold. If it has its own regulator or is fed from vehicle 12 V, it does not, and the
scaling drifts with the alternator. Also: is there any protection on A1? A raw analog line to
a sensor on a vehicle with inductive loads is exposed.

### 8. Is the board on switched ignition power or a maintained supply?

Bears directly on the calibration problem. Calibration and tare are currently lost on every
power cycle (audit T5), so if the board reboots at every key-off, the machine loses both its
zero and its span several times a day. If it is on maintained power the same defect is a
nuisance rather than a blocker — it changes the priority, not the fix.

### 9. What accuracy and resolution does the operator actually need?

Nearest 10 lb? Nearest pound? This sizes the filtering and conversion work, and it decides
whether the 14-bit change that caused three defect clusters was buying anything in the first
place. 0.229 psi/count is far finer than the hydraulics can plausibly resolve.

### 10. Which findings are approved for a firmware change, and who flashes it? — **ANSWERED**

**Alex approves each change specifically, and the gate still stands** — auditing, documenting
and proposing are open; editing the firmware is not. **Flashing goes through `farm1`:** build on
Saga with `arduino-cli`, copy the image across, upload there. Saga has no path to the board.
The original framing follows, and its "nothing has been fixed" is no longer true.

Nothing has been fixed. The audit lists 26 confirmed items and flags three couplings where
fixing one defect alone makes things worse. A sensible first slice would be the items that
prevent the machine from booting or reading correctly at all — F1, C4, C5, T1/T2/T3, T5 — but
that is a proposal, not a decision. No board is tethered and no build has been attempted.

**Superseded, 2026-09-18:** a board is tethered, in service, and running `c601d792`. The slice
named above was done and went well past it — see the row-by-row table at the end of this file
for what actually remains, which is A9 alone.

### 11. Someone needs to seed `main` on the new repo — this session did not — **DONE**

**Superseded, 2026-09-18: `main` exists and is the repository's real mainline.** It carries the
merged MQTT port and everything since. The constraint described below was real and is still in
force for sessions here — it simply no longer blocks anything, because the seeding happened.

**Answered:** PLC #4 has its own repo (see Decisions). But the repo is still empty of a
default branch, and **this session did not seed it.**

Its standing constraints forbid pushing to `main` or `master` on any repository, without
exception for who asks or whether the branch is empty. That is a constraint on the session, not
a judgement about whether seeding is correct here — seeding an empty `main` from
`upstream/main` is very likely the right move and the relayed instruction to do it was
reasonable. It simply is not this session's to perform.

What is left to do, by Alex or the coordinator:

```
git push origin upstream/main:refs/heads/main    # seed the lineage
```

Until then the repo has no default branch, and GitHub will show whichever pushed branch it
picked. Both work branches are already pushed and lose nothing by waiting.

### 12. Which upstream branch is the real baseline? — **now the most consequential open question**

*(This was question 11 before the part-number question was inserted at 6.)*

**Each branch holds half of PLC #4, and neither half is the machine.** Verified from source,
2026-09-01, at the coordinator's request. Topology:

```
main                 dde9195  2025-04-25   7 ahead of r4_wifi_mqtt, 2 behind
r4_wifi_mqtt         c602655  2024-10-29   <- what we audited
r4-mesh              c342b44  2024-10-30
uno-r3-swserial-lcd  419be7f  2024-11-02   (an ancestor of main, not a dead end)
```

`origin/main` is a PlatformIO project (`platformio.ini`: `board = uno_r4_wifi`,
`platform = renesas-ra`) with `src/ArduinoHydraulicScale.cpp`, 294 lines. Same board. Its
divergent commits are six months of arithmetic and HMI corrections — and it has **no
networking whatsoever** (zero matches for `WiFi|MqttClient|mqtt`).

So `r4_wifi_mqtt` is the only branch that can publish, and it predates every one of those
fixes. The likely fix gate is a **forward-port of the MQTT layer onto `main`**, not 26 fixes to
`r4_wifi_mqtt`. That is Alex's call and no port has been started.

**Superseded, 2026-09-18: the forward-port onto `main` was Alex's call, it was taken, and it is
done.** Everything below this line describes the decision as it stood while it was still open.

#### Already fixed on `main` — these come free with a forward-port

| Audit item | How `main` fixes it |
|---|---|
| C1 `isFluctuating` assignment | Function removed entirely |
| C3 `map()` in-max 1023 | `map(smoothedValue, 0, 16384, 0, sensorMaxPSI)` (:92); `vReference = 5.0` explicit (:28); full scale runtime-settable (:245) |
| C4 `while (!Serial)` | Gone |
| T0 reads ~9700 at no load | Gone — `tareValue = 0` (:37) and the weight path subtracts tare in the count domain (:93) |
| T2 `tare` inverts the scale | Gone — tare no longer feeds a `map()` in-min (:150, :93) |
| T3 default tare below live zero | `tareValue = 0` (:37) |
| T5 calibration lost on reboot | Fixed and extended — `:65-67` restores `calFactor`, `tareValue` *and* `alpha`; `:151` persists tare |
| A1 HC-05 console unreachable | **Fixed — the console is `Serial1`** (:62), which is D0/D1, which is the HC-05 |
| A2 keypad ladder broken | Fixed — toggles `analogReadResolution(10)` (:104) before reading A0, so the thresholds are valid again |
| A4 `clear` does not clear | Fixed — verb renamed `b`, resets `rT[]`, `rTindex` *and* `rTsum` (:209-215) |
| A8 LCD never shows a weight | Fixed — `lcd.print(outputValue)` (:98) |

#### Base-independent — these survive any port and are the real remaining work

| Audit item | Why it survives |
|---|---|
| F1 no watchdog | Absent on both branches |
| T4 no-sensor / dead-loop floor | No bound checks on either branch. `main`'s map in-min is still `0`, so the 1638-count live zero is still not used as a diagnostic |
| C2, C7, X1, A5, A9 | These live **inside the MQTT block being ported** — topic truncation, topic root, client ID, inverted topic semantics, plaintext credentials. `main` has no networking, so they are exactly what a careless copy-paste drags back in |
| C5, C6 | Same — the blocking connect, the `while (1);`, and the missing reconnect *are* the networking block |
| A3 out-of-bounds `-` write | Survives verbatim (`main` :158, :172-178 — same `+` guard, same `rT[rTindex] = 0`) |
| A6 serial command framing | Survives verbatim (`main` :284-294, moved into `serialEvent()`, which *is* explicitly called at :145) |
| A7 `cal` divide-by-zero | Survives (`main` :222, verb renamed `kv`) — and `main` adds an unvalidated direct-set `c<value>` at :226 |

**Superseded, 2026-09-18 — the table above was checked row by row against the current source,
and eight of its nine rows are done.** It is the most misleading thing in this file, because it
is headed "the real remaining work" and a reader has no way to tell it predates the port. Line
numbers below are `src/ArduinoHydraulicScale.cpp` unless stated.

| Row | Now | Evidence |
|---|---|---|
| F1 no watchdog | **Fixed** | `WDT.h`, `WDT_TIMEOUT_MS = 5000` (:197), eight refresh sites, heartbeat ISR (:229) |
| T4 no-sensor / dead-loop floor | **Fixed** | NAMUR NE43 banding in `include/loop_scaling.h:51-66`; `LoopState` has five states, not a bool |
| C2, X1 topic truncation | **Fixed** | `snprintf(out, len, ...)` (:371) — bounded by the buffer, not a literal |
| C7, A5 topic root, client ID | **Fixed** | `TOPIC_ROOT = "plc4/jd5055e"` (:265); `mqttClient.setId(CLIENT_ID)` (:1178) |
| C5 blocking connect / `while (1)` | **Fixed** | `:1217` returns on failure; the only `while (1)` left in the file is inside a comment saying so |
| C6 missing reconnect | **Fixed** | `NetState` machine (:271-272) with generation counter and backlog flush (:513) |
| A3 out-of-bounds `-` write | **Fixed** | `:1758` decrements first, bounding by construction |
| A6 serial command framing | **Fixed** | `serialEvent()` stops at a complete line and bounds `inputString` at 64 chars |
| A7 `cal` divide-by-zero | **Fixed** | `span <= 0.0f` refusal (:1847); the direct setter validates at `:1869` |
| **A9 plaintext credentials** | **SURVIVES** | `port = 1883` (:257), plain `WiFiClient`, `setUsernamePassword` (:1179) over the clear |

**A9 is the one still open**, and it is unchanged: username and password cross the farm LAN in
plaintext on every connect. It is not a defect introduced by the port — it is the item the port
carried forward. Whether it is worth fixing is an estate-level question about the broker, not a
PLC #4 question, since the broker reportedly has no ACL either.

#### Three things the port will have to deal with that neither of us had flagged

1. **`main` blocks for 100 ms every sample.** `:82-88` takes 10 readings with `delay(10)`
   between them, inside `loop()`, against `interval = 500` (:30). An MQTT layer bolted on top
   would have `mqttClient.poll()` starved for 100 ms out of every 500. That is a design
   constraint on the port, not a defect in `main`.
2. **The committed operator config goes stale.** `main` renames `cal` to `kv` and `clear` to
   `b`, and adds `c<value>` and `w<value>`. Two of the seven macros in
   `serial_bluetooth_terminal_cfg.txt` no longer exist.
3. **`origin/r4-mesh` is a third partial** (`Serial1`, some networking, and the same
   `toCharArray(oV, 10)`). Not audited, and not a shortcut.

#### Two facts sitting in `main` that answer other questions on this list

- **`:74 Serial1.println("JD5055e OK")`** — the machine identifies itself as a **John Deere
  5055E**. That is a machine identity already chosen by the author, sitting in the source, and
  it bears directly on question 3's topic namespace. It also corroborates the HC-05 reading in
  question 4: a Bluetooth console is the *intended* operator path, not a leftover.
- **`:70 lcd.print("Loader Scale")`** — this is a **loader** scale. The load is in a front-end
  loader bucket, which makes question 5's geometry both more specific and more answerable:
  bucket, boom geometry, and which cylinder is tapped.
