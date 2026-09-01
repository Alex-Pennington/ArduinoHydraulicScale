# Post-flash review — two blocking defects in the ported MQTT layer

**Date:** 2026-09-01
**Reviewed:** `plc4/mqtt-port` @ `83cb2c5` (the source now running on the bench board)
**Found by:** an independent reviewer, relayed via the coordinator. Both confirmed here against
the library sources and against my own code.
**Status: R2 fixed. R1 fixed for the steady-state case and verified against a real network;
one narrower reset remains at first association, undiagnosed.** See "The fix" and the correction
that follows it.

Both defects are mine, introduced by this port.

---

## R1 — the watchdog fix does not cover `mqttClient.connect()`

**BLOCKING.** Consequence is a 30-second reboot cycle that destroys the backlog queue and the
operator's tally, in exactly the situation the queue exists to serve.

### Mechanism, verified from source

| evidence | what it shows |
|---|---|
| `MqttClient.cpp:846` `_client->connect(host, port)` | the TCP connect happens here |
| `MqttClient.cpp:227,247,349,395,938` | every `_connectionTimeout` loop is *after* it |
| `MqttClient.cpp:71` `_connectionTimeout(30 * 1000L)` | the default it replaces |
| `WiFiClient.h:251` `int _connectionTimeout = 0;` | and nothing in this firmware sets it |
| `Modem.h:12` `MODEM_TIMEOUT 10000` | so the TCP connect is capped only at 10 s |

`mqttClient.setConnectionTimeout(3000)` therefore bounds only the **CONNACK wait**, not the TCP
connect underneath it. It was not useless — it cut the 30 s default — but it is not what the
comment in `setup()` claims.

Compounding it: `netPoll()` skips its inner `WDT.refresh()` when WiFi is already associated, so
the last refresh before that 10 s block is at the top of the function, against a watchdog ceiling
of 5592 ms.

### Consequence

WiFi up, broker unreachable → `mqttClient.connect()` blocks ~10 s → watchdog resets the board.
After reset `lastNetAttemptMs = 0`, so the next attempt comes at ~30 s, blocks, and resets again.

**Every reset zeroes `queueBuf` and `rT[]`** — they are globals, and the queue is RAM by ruling.
So the machine loses both the publish backlog and the operator's recorded tally, on a 30-second
cycle, precisely when the broker is unreachable. That is worse than the original defect this
port set out to fix.

### What `wifiClient.setConnectionTimeout()` actually does

The reviewer called it necessary but not sufficient because `modem.write()` is still capped at
`MODEM_TIMEOUT`. Reading `WiFiClient.cpp:59-60` refines that: `_connectionTimeout` is passed **as
a parameter of the AT command** to the coprocessor, so it bounds how long the ESP32 spends
connecting and the reply comes back at that point. `MODEM_TIMEOUT` is the outer bound on the
coprocessor *answering at all*.

That distinction is the whole fix. With the timeouts set, a normal unreachable broker returns in
~2 s. The 10 s path is reached only when the coprocessor itself is wedged — and a reset is the
correct response to that, which is what a watchdog is for. The defect was that an **ordinary**
unreachable broker triggered it.

### It turned out not to need a design decision

Three branches were on the table. Two are closed by fact, not by preference:

- **Disarm the watchdog around the calls** — unavailable. The `WDT` class exposes `begin`,
  `refresh`, `getTimeout`, `getCounter` and no stop or end.
- **Refresh from a poll callback** — checked, and the hook is in the wrong place. `yield()` is
  weak in the core (`cores/arduino/time.cpp:17`) and ArduinoMqttClient calls it at 237, 253,
  357, 401 and 944 — but every one of those is inside a `_connectionTimeout` loop that was
  already bounded. WiFiS3 contains **zero** `yield()` calls across every `.cpp` and no weak
  symbols, and `Modem.cpp` has no callback or handler of any kind. The two loops that actually
  block — `ModemClass::buf_read` and `CWifi::begin`'s busy-wait — offer nothing to hook.

That leaves the third answer, which is not a trade-off but ordinary correctness: **bound every
blocking layer to less than the watchdog period, and refresh immediately before each one.** No
design authority was needed; the earlier framing as a design question came from my own fix being
incomplete rather than from a real fork.

---

## R2 — recorded weights are silently discarded when association drops

**BLOCKING.** No queue entry, no dropped count, nothing on the LCD, and `net` still reports
`mqtt = up`.

`netPoll()` consults `WiFi.status()` **only inside the `!mqttClient.connected()` branch**. If a
silent association loss leaves `mqttClient.connected()` still returning true, `netState` stays
`NET_ONLINE` for up to ~120 s. In that window:

- `publishTo()` returns true, because `WiFiClient::write()` reports success once bytes reach the
  coprocessor, and QoS 0 means nothing waits for an acknowledgement;
- so the `+` handler takes the "published live" branch and **never calls `queuePush`**.

Every weight the operator records in that window is lost. The tally still shows it, so `sum` is
right and the published record is short — the two silently disagree, which is the same divergence
the audit recorded as C6 in the original firmware.

---

## Why nothing is on fire

**R1 is not live on the bench board, and this was tested rather than assumed.**

The flashed build carries placeholder credentials, so association never succeeds and the MQTT
path is never reached (`net` reports `wifi = down`). To confirm the board was not reset-cycling:

```
cleared the tally, recorded three weights, waited 80 s, re-read
  before:  0=2777  1=2868  2=2783  sum=8428
  after:   0=2777  1=2868  2=2783  sum=8428   queued=3
```

A 30-second reset cycle would have zeroed both `rT[]` and `queueBuf` twice over. It did not.

**The gate is not the board being powered — it is `arduino_secrets.h` getting real values.**
Placeholder credentials are what keeps R1 latent, and that is load-bearing rather than
incidental. Nothing goes on that board with real credentials until R1 is resolved.

---

## The generalisation, which matters more than either defect

**A confident comment on a partial fix is an active instruction to skip the place the bug lives.**

R1 sits under a long comment block in `setup()` that states the interaction was verified against
the core sources. That block is worse than the bug. The bug is a missed call; the comment tells
the next reader there is nothing to find there.

What actually happened: I read `WDT.cpp` and `WiFiS3/WiFi.cpp` closely, correctly bounded the
call I had investigated, and then *assumed* the other blocking call was covered without opening
`MqttClient.cpp`. The investigation was real and the conclusion was written up as though it were
complete.

This is the third instance today of a fix receiving less scrutiny than the finding that prompted
it:

1. the A3 remedy, accepted from a peer without walking the boundary case — caught by a fourth reader;
2. `publishWeight()` declared `void` and used in a boolean test — caught by the compiler;
3. R1 — caught by an independent reviewer.

The pattern is not carelessness in the analysis. It is that **the analysis and the remedy get
different levels of scrutiny**, and the write-up then inherits the confidence of the analysis.
Fixes need the treatment findings get, and a comment asserting verification should name exactly
what was checked — here, `WDT.cpp` and `WiFi.cpp` — so that what was *not* checked is visible.

---

## What a value-sweep misses: the `calFactor` upper bound

Raised by the reviewer, and worth recording for its shape rather than its size.

`calFactor`'s validation accepts anything up to `1000.0f`. The expected value is around
**0.066**, so the bound admits roughly **15,000x** the plausible figure — it is a range check
that would pass almost any corruption.

The reason it survived the earlier sweep is the useful part. That sweep hunted for constants
that were wrong *for 14-bit resolution*, and it was substantially right that no fifth instance
affected a published value. But `1000.0f` is a **validator**, not a value: it produces no wrong
number itself, it merely fails to reject one. A search for wrong outputs cannot surface a bound
that is too generous, because a too-generous bound has no output.

The rule to carry: after a resolution or scaling change, re-derive the **limits** as well as the
values. Not fixed — `src/` is untouched pending the R1 design decision.

## Also from the bench, and also mine

**T4's open-circuit detection does not work.** With nothing connected the board reads 4401
counts and reports `sensor = ok`. A floating CMOS analog input does not sit at 0 V. Needs a
10k-47k pull-down to GND; no firmware change substitutes for it, because a floating reading is
not distinguishable from a real one. The PLC #1 dead-loop *pattern* crossed to this machine; its
electrical precondition did not, and the estate should know that.

See `notes/2026-09-01-transducer-failure-and-selection.md`.


---

## The fix

**R1** — three layers, which are sequential inside one `mqttClient.connect()` call because the
TCP connect happens before the CONNACK loop:

```
WIFI_CONNECT_TIMEOUT_MS  3000   CWifi::begin association wait
TCP_CONNECT_TIMEOUT_MS   2000   WiFiClient — THIS WAS MISSING, and it is the one that mattered
MQTT_CONNECT_TIMEOUT_MS  2000   CONNACK wait only
```

`wifiClient.setConnectionTimeout()` was never called, so `_connectionTimeout` sat at its
`WiFiClient.h:251` default of 0 and the TCP connect was bounded only by `MODEM_TIMEOUT = 10000`.
Worst normal case is now 2000 + 2000 = **4000 ms against a 5592 ms ceiling**. The two must sum to
less than `WDT_TIMEOUT_MS`; that constraint is written at their definitions.

The missing `WDT.refresh()` before `mqttClient.connect()` is restored. It was skipped whenever
WiFi was already associated — the common case — leaving the last refresh at the top of the
function.

**R2** — `WiFi.status()` is now checked first and unconditionally on entry to the `NET_ONLINE`
branch, instead of only inside `!mqttClient.connected()`. The loss window shrinks from ~120 s to
one loop iteration.

**Residual, stated plainly:** QoS 0 means a single in-flight message can still be lost if the
association drops between the check and the write. Closing that needs QoS 1, whose `endMessage()`
blocks for PUBACK and would have to be weighed against the same watchdog budget. Not done, and
not hidden.

**Verified on hardware:** flashed to the bench board and held for 70 s across two 30-second
net-retry windows with WiFi down. Tally intact (5563 before and after), `queued = 2`, no reset.


---

## CORRECTION: the first R1 fix was wrong, and I called it verified

The section above ("It turned out not to need a design decision") was written with confidence and
was wrong. Recording it rather than editing it away, because the mistake is the lesson.

**What I claimed:** that bounding every blocking layer was sufficient, that the design question
dissolved on closer reading, and that the fix was "verified on hardware".

**What the hardware said**, the moment real WiFi credentials existed: the board reset-looped every
30 seconds against a real SSID, wiping the tally each time. Four resets in 100 seconds.

**Why my verification did not catch it:** I tested with placeholder credentials, so association
never succeeded and `WiFi.begin()` failed fast. The test never entered the path. I even wrote, in
the same report, that "nothing has exercised the live path because no WiFi credentials exist" —
and still called the defect fixed. **A test that cannot fail is not evidence.**

**What was actually wrong:** the same error as the original R1, one layer over. `CWifi::begin()`
has *three* blocking stages, and `WiFi.setTimeout()` bounds only the last:

```
WiFi.cpp  modem.write(_MODE)      capped only by MODEM_TIMEOUT = 10000 ms
WiFi.cpp  modem.write(_BEGINSTA)  capped only by MODEM_TIMEOUT = 10000 ms
WiFi.cpp  while (millis() - start < _timeout)   <-- the only one WiFi.setTimeout bounds
```

Worst case 23 s against a ceiling of 5592 ms that cannot be raised. **No set of timeouts fits** —
anything short enough is shorter than association itself needs, so the board would never connect.
The design question was real. I dissolved it by not looking one level further down, in exactly
the place I had already been caught not looking once.

## What actually fixed it: the heartbeat watchdog (branch 4)

A 2 Hz `FspTimer` ISR refreshes the hardware watchdog, **but only while `loop()` is still
ticking** — `loopHeartbeatMs` is stamped at the top of every pass and the ISR refreshes only if
it has moved within `HEARTBEAT_TOLERANCE_MS` (30 s, chosen to exceed the 23 s worst case).

A timer that refreshes unconditionally is the classic watchdog anti-pattern: it feeds the dog
while the program is dead. Gating on the heartbeat keeps hang detection and only widens it from
5 s to ~30 s — the price of a network stack whose blocking calls cannot be bounded under the
ceiling. If no timer is free the board still runs and says `heartbeat wdt = UNAVAILABLE` rather
than failing quietly.

### Verified, this time against the real failure

| test | result |
|---|---|
| WiFi UP, broker unreachable, 150 s, 5 connect attempts | **0 resets, tally intact (8324 before and after)** |

That is the exact scenario the reviewer described as a 30-second reboot cycle. It no longer
resets, and the board associated to a real network (`ip = 192.168.5.170`) while doing it.

### Still open, and not claimed as fixed

**One reset was observed during the first association**, before WiFi came up — 1 in 130 s, down
from ~4. Not diagnosed. Two candidates, and I have not distinguished them:

- a stall in first association exceeding the 30 s tolerance, which would point at raising
  `HEARTBEAT_TOLERANCE_MS` and accepting still-later hang detection;
- a brownout as the ESP32 radio powers up, drawing current through the USB debug chip — which is
  a bench power artefact and not a firmware fault at all.

The steady-state case is fixed. The first-association case is narrower, happens once at boot, and
costs a tally that is empty at that point. It should not be called fixed until someone knows which
of those two it is.

---

## Bench bring-up against a live broker — the publish path, exercised at last

Bench broker: Saga, `192.168.5.241:1883`, anonymous, same subnet as the board — deliberately
mirroring farm1 rather than rata, so the bench exercises the configuration that ships.

### The fix that stopped the board reaching any broker

`wifiClient.setConnectionTimeout()` — added as part of my R1 fix — **silently selects a different
AT command**, and that command does not work with a dotted-quad host on this core:

```
WiFiClient.cpp:53   _connectionTimeout == 0  ->  +CLIENTCONNECTNAME   works
                    _connectionTimeout != 0  ->  +CLIENTCONNECT       fails here
```

Symptom: `mqttClient.connect()` returned **-2 (MQTT_CONNECTION_REFUSED)** on every attempt while
`tcpdump` at the broker saw **zero packets**. The connect failed at the coprocessor before any TCP
was attempted. I first diagnosed this as a firewall problem — it was neither firewall nor network,
it was the fix.

Removed rather than worked around. Bounding the TCP connect is unnecessary now that the heartbeat
watchdog covers blocking network calls, which is what it exists for.

**The diagnostic that made it findable:** `mqtt_err` and `attempts` on the `net` verb. `-2` with
nothing on the wire is a fingerprint; without it this is indistinguishable from a firewall drop,
which is exactly where I looked first and wasted the time.

### Verified end to end

```
plc4/jd5055e/telemetry {"w":2794,"psi":639,"counts":4432,"sensor":"ok","queued":0,...}
plc4/jd5055e/weight    {"w":2795,"age_ms":0,"backlog":false,"sensor":"ok",...}
```

The topic arrives **complete and untruncated** at its full 19 characters. That closes X1: the
audit's truncation finding was inferred from source and is now observed on the wire, and the
`snprintf`-into-a-sized-buffer construction holds under the real root.

### Backlog replay semantics — measured, not assumed

Method: hold the subscriber connected and drop only the board with an `iptables` rule, so the
capture spans the whole outage. Record three weights offline, age them, unblock.

```
plc4/jd5055e/weight {"w":2772,"age_ms":74701,"backlog":true,"id":"plc4-jd5055e"}
plc4/jd5055e/weight {"w":2773,"age_ms":72119,"backlog":true,"id":"plc4-jd5055e"}
plc4/jd5055e/weight {"w":2769,"age_ms":69535,"backlog":true,"id":"plc4-jd5055e"}
```

**Answer to "original timestamp or replay time": neither — age at replay, plus an explicit flag.**

- `backlog: true` — a replayed sample is **never** indistinguishable from a live one. That is the
  failure that has had PLC #2 reading healthy for 39 hours on retained replays, and this design
  does not have it.
- `age_ms` is the delta from capture to flush, so a consumer recovers capture time as
  `receipt_time - age_ms` without the board needing a clock. It has no RTC and no NTP before the
  network returns, so an absolute stamp would have been a fiction.
- Order is FIFO, oldest first — ages descend, and the ~2.58 s gaps match the spacing at which the
  weights were recorded, so relative timing survives the outage intact.
- No `sensor` field on backlog records, by design: that state is current and the reading is not.

Queue depth reported live by `net` (`queued = 3` while offline, `0` after flush), so the operator
can see the backlog drain rather than having to trust it.

---

## The first-association reset: found, explained, fixed

Chased rather than left open, because "only at first association" is the signature of something
that fires in the field and not on the bench — and this tractor makes a first association every
time it is keyed on.

### Diagnosing it without guessing

The RA4M1 records why it last reset, so this did not have to be inferred. `RSTSR0` carries
power-on and brownout flags, `RSTSR1` carries watchdog and software. Reading and decoding them at
boot — and clearing them so the next boot reports its own cause — distinguishes a watchdog bite
from a brownout directly, which were the two candidates.

Now reported at boot and on the `net` verb: `reset: software (r0=00 r1=0004)`.

### The mechanism

`loopHeartbeatMs` is stamped **once per `loop()` pass**, so everything `netPoll()` does
accumulates against a single heartbeat. With both halves in the same pass:

```
WiFi.begin()          up to 23000 ms   (2 x modem.write @ 10000 + 3000 busy-wait)
mqttClient.connect()  up to 10000 ms   (MODEM_TIMEOUT)
                      -------------
                            33000 ms   against HEARTBEAT_TOLERANCE_MS = 30000
```

Past 30 s the ISR stops refreshing and the watchdog bites ~5 s later.

**That is why it appeared only at first association.** It needs *both* halves to run slow in the
*same* pass, and that combination only occurs when the board must associate and then reach a
broker in one go. Once associated, stage 1 is skipped and the pass costs 10 s at worst — which is
exactly the "stable afterwards" behaviour observed.

### The fix: split the stages

`netPoll()` now returns after association **even on success**, so the broker connect always runs
in a later pass with a fresh heartbeat. `STAGE_GAP_MS` (500 ms) means the broker attempt follows
shortly rather than waiting out the 30 s retry.

Worst case per pass is now bounded by the larger half alone — 23 s, against a 30 s tolerance.

Raising the tolerance would also have "worked" and would have been the wrong fix: it buys margin
by degrading hang detection, and leaves the accumulation problem in place for the next blocking
call anyone adds.

### Verified

Fresh boot with the broker blocked at the firewall, so the board had to associate and then fail
to reach a broker — the closest reproduction available on the bench:

| | |
|---|---|
| boots in 160 s | **1** (the flash's own software reset) |
| connect attempts | 5, all failing with `mqtt_err = -2` |
| resets | **none** |

**Stated honestly:** this does not reproduce the original 33 s condition, which needs a *slow*
association, and no AP under our control can force that. The assurance here is structural — the
split makes it arithmetically impossible for both halves to share a heartbeat — plus a passing
test of the reachable half. That is a different and weaker kind of evidence than reproducing the
failure and watching it stop, and it should be read that way.
