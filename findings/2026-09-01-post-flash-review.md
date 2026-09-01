# Post-flash review — two blocking defects in the ported MQTT layer

**Date:** 2026-09-01
**Reviewed:** `plc4/mqtt-port` @ `83cb2c5` (the source now running on the bench board)
**Found by:** an independent reviewer, relayed via the coordinator. Both confirmed here against
the library sources and against my own code.
**Status: NOT FIXED.** Both wait on a design decision from Alex. Neither is live on the bench
board — see "Why nothing is on fire" below.

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

### Not a certified remedy

`wifiClient.setConnectionTimeout(3000)` is necessary but **not sufficient** — the enclosing
`modem.write()` is still capped at `MODEM_TIMEOUT`. Capping via `modem.timeout()` leaves at best
~92 ms of margin through `CWifi::begin()`, which is not engineering, it is luck.

**This is a design question for Alex, with three branches on the table:**

1. Should the watchdog span the network calls at all, or should it be disarmed around them?
2. Should the queue survive a reset — which reopens the flash-persistence question that the
   RAM-only ruling closed?
3. Unexplored: refresh the watchdog from inside a poll callback, covering the network calls
   without widening the window — *if* the library exposes a hook that runs during the blocking
   call. Nobody has checked whether it does.

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

## Also from the bench, and also mine

**T4's open-circuit detection does not work.** With nothing connected to A1 the board reads 4401
counts and reports `sensor = ok`. A floating CMOS analog input does not sit at 0 V. Needs a
10k-47k pull-down to GND; no firmware change substitutes for it, because a floating reading is
not distinguishable from a real one. The PLC #1 dead-loop *pattern* crossed to this machine; its
electrical precondition did not, and the estate should know that.

See `notes/2026-09-01-transducer-failure-and-selection.md`.
