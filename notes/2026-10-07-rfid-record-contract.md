# RFID record and tally — MQTT contract proposal (2026-10-07)

**NOT APPROVED.** This is a proposal. Nothing in it is in the firmware, and nothing changes on the
tractor until Alex approves a specific firmware change. The binder card
`references/03_mqtt_contract.md` follows once a change is approved and flashed. Until then it
describes the running firmware, not this note.

The design this specifies is settled in `notes/rfid-inventory-discussion.md`. Nothing recorded
there as decided is reopened here. The pure logic behind this contract is in
`include/rfid_record.h`, and `test/test_rfid_record.cpp` tests it on the host. Nothing in `src/`
includes the header.

Citations of the form `src:N` refer to `src/ArduinoHydraulicScale.cpp` at the base of this
branch. `MqttClient.cpp:N` refers to ArduinoMqttClient 0.1.8 as the build uses it
(`~/Arduino/libraries/ArduinoMqttClient`). Anything marked **inference** is reasoned from that
source, not measured.

## Ruled by Alex (2026-10-07)

Ruled in the coordinator's dialog and relayed to this worker by Ymir. These are closed.

- **Platform:** a new UNO R4 WiFi with a screw-terminal shield. It has the same MCU and the same
  32 KB of RAM as the board in service, so the RAM sums below are for that part.
- **Records are taken only within WiFi range.** In his words: *"Range only is fine."* Out of
  range there is no recording, and that is accepted.
- **No GPS fix: the record is accepted with no position.** This was this note's proposal, and he
  ruled it.

## Messages at a glance

| Message | Topic | Direction | New or changed |
|---|---|---|---|
| Record command (`+`) | `plc4/jd5055e/cmd` | app → board | Changed: tally, phone time, GPS, optional chosen tag |
| Void command (`-`) | `plc4/jd5055e/cmd` | app → board | Changed: names the record it voids |
| Tally-end command (`end`) | `plc4/jd5055e/cmd` | app → board | New verb. It replaces `b` |
| Ack | `plc4/jd5055e/ack` | board → app | Changed: `rec`, `tally`, `tag`, `w`, `tags`, `tags_seen`. `n` and `total` are dropped |
| Record event | `plc4/jd5055e/weight` | board → consumer | Changed: `kind`, `rec`, `tally`, `tag`, `t_ms`, `lat`, `lon`, `acc`. `n` and `total` are dropped |
| Void event | `plc4/jd5055e/weight` | board → consumer | New |
| Tally-end event | `plc4/jd5055e/weight` | board → consumer | New |

Retain and the dedup rules stay as `03` states them today. Commands go in at QoS 1 and are never
retained. Acks go out at QoS 0. **The weight topic changes from QoS 0 to QoS 1**, not retained.
An event leaves the board's queue only when the broker has acknowledged it; see *Delivery*.

## Commands on `plc4/jd5055e/cmd`

The envelope does not change. `id` and `verb` are read and checked as they are today: same
charset, 40-character cap, the eight-deep dedup ring, and a repeat replays the stored verdict
(`src:1140-1170`, `src:1147-1152`). All new fields are flat members of the same object.

### Record — verb `+`

```json
{"id":"3f2a9c21-7b4e-4c1d-9a0e-5d6f7a8b9c0d","verb":"+","tally":"T-day-1007",
 "t_ms":1791398400123,"lat":44.1234567,"lon":-72.7654321,"acc":5}
```

The retry after a `MULTIPLE TAGS` refusal uses a **new `id`** and names the tag the operator
tapped:

```json
{"id":"8e1b44d0-2c55-4a1f-b1e3-0a9d6c7e2f11","verb":"+","tally":"T-day-1007",
 "t_ms":1791398406410,"lat":44.1234567,"lon":-72.7654321,"acc":5,
 "tag":"E28011700000020F1A2B3C01"}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `id` | string | yes | As today. **It becomes the record ID if the record is accepted** (see *Record ID*) |
| `verb` | `"+"` | yes | |
| `tally` | string, the `id` charset, 1–40 chars | yes | The open tally this record goes to. The app picks it when a daily log and a delivery are both open |
| `t_ms` | integer | yes | Phone time: UTC milliseconds since the Unix epoch |
| `lat`, `lon` | number, decimal degrees | all three, or none | Phone fix. The board keeps 1e-7 degree and truncates extra decimals. Exponent form (`1e-7`) is refused |
| `acc` | number, metres | all three, or none | Phone fix accuracy. The board rounds it **up** to whole metres, 0–65535 |
| `tag` | string, upper-case hex EPC | no | The chosen tag, on the retry only |

GPS is optional: the phone may have no fix, and Alex ruled that such a record is accepted with no
position. Time is required.

**A retry must carry a new `id`.** A repeated `id` is not run again: the board replays the stored
verdict (`src:1147-1152`). If a retry reused the `id` of the refused attempt, it would get the
refusal back without the board reading the tags again.

### Void — verb `-`

```json
{"id":"v-5b0c","verb":"-","rec":"3f2a9c21-7b4e-4c1d-9a0e-5d6f7a8b9c0d",
 "t_ms":1791398460000,"lat":44.1234567,"lon":-72.7654321,"acc":5}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `rec` | string, the `id` charset | yes | The record ID being voided. The app sends the last record of the tally the operator is in |
| `t_ms`, `lat`, `lon`, `acc` | as for `+` | as for `+` | |

The board does not hold the tally, so it cannot check that `rec` exists. It checks only the
syntax. Matching the void to a record is the consumer's job.

### Tally end — verb `end`

```json
{"id":"e-77a1","verb":"end","tally":"T-day-1007","t_ms":1791420000000}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `tally` | string, the `id` charset | yes | The tally being closed |
| `t_ms`, `lat`, `lon`, `acc` | as for `+` | as for `+` | |

The verb name `end` is this proposal's choice. The discussion names the button "end tally" and
says it replaces `b`.

## Acknowledgement on `plc4/jd5055e/ack`

The shape is the same as today: `id`, `verb`, `ok`, `detail`, and `dup` on a replay. The fields
below are added. `n` and `total` are not sent any more.

| Outcome | `ok` | `detail` | Extra fields |
|---|---|---|---|
| Record accepted | true | `recorded` | `rec`, `tally`, `tag`, `w` |
| No tag in view | false | `NO TAG` | none |
| Several tags, none chosen | false | `MULTIPLE TAGS` | `tags` (strongest first), `tags_seen` |
| Chosen tag not in this read | false | `TAG NOT IN VIEW` | `tags` (what *is* in view), `tags_seen` |
| No tally ID / malformed | false | `NO TALLY` / `BAD TALLY` | none |
| No phone time / malformed | false | `NO TIME` / `BAD TIME` | none |
| GPS partial, malformed or out of range | false | `BAD GPS` | none |
| Chosen tag malformed | false | `BAD TAG` | none |
| Reader not answering | false | `NO READER` | none |
| Event queue full (any of `+`, `-`, `end`) | false | `QUEUE FULL` | none. Nothing already accepted is dropped; see *Backlog* |
| Loop not `ok` or reading stale | false | the sensor state, or `NO ADC` | none. Unchanged from today (`src:1902-1909`) |
| Void accepted | true | `voided` | `rec` |
| Void with no `rec` / malformed | false | `NO RECORD ID` / `BAD RECORD ID` | none |
| Tally end accepted | true | `ended` | `tally` |
| Replay of an accepted record | true | `recorded` | `dup:true`, `rec`. Since the record ID *is* the command `id`, a replay can name the record without storing anything more |

```json
{"id":"3f2a9c21-…","verb":"+","ok":true,"detail":"recorded","rec":"3f2a9c21-…",
 "tally":"T-day-1007","tag":"E28011700000020F1A2B3C01","w":1834}
{"id":"7f3a0001","verb":"+","ok":false,"detail":"MULTIPLE TAGS",
 "tags":[{"epc":"E28011700000020F1A2B3C02","rssi":-45},
         {"epc":"E28011700000020F1A2B3C01","rssi":-70}],"tags_seen":2}
{"id":"7f3a0002","verb":"+","ok":false,"detail":"TAG NOT IN VIEW","tags":[],"tags_seen":0}
{"id":"v-5b0c","verb":"-","ok":true,"detail":"voided","rec":"3f2a9c21-…"}
{"id":"e-77a1","verb":"end","ok":true,"detail":"ended","tally":"T-day-1007"}
```

The checks run in this order, and the first failure is the one reported: tally, then time, then
GPS, then the chosen tag's syntax, then queue space, then the loop state, then the reader, then
the tag decision. The cheap checks run before the loop check and the tag read. For `-` and `end`
the order is the field checks, then queue space.

**Tag list rules** (`rfid::decide`):

- A tag the reader reports several times in one read counts once, at its strongest RSSI.
- The list is sorted strongest first. Tags with equal RSSI sort by EPC, so the order does not
  depend on the order the reader reported them in.
- The board keeps the 8 strongest, and the ack lists those 8. `tags_seen` counts every distinct
  tag in the read, so the app can tell when the list is cut short.
- A chosen tag is checked against the whole read, not only the 8 it keeps. It is accepted if it
  is in this read, whatever else is also in view.
- A replayed `MULTIPLE TAGS` ack has no `tags`. The ring stores only `detail`
  (`src:742-747`), and an old tag list would be stale anyway. Send a new `id` to read again.

All `detail` strings fit the 20-character field the ack path has today (`src:2228`). Each refusal
string is also printed on the console, as the rule at `src:2231-2235` requires.

## Events on `plc4/jd5055e/weight`

The topic now carries three kinds of event, told apart by `kind`. All three go on the one topic
so the consumer gets them **in the order the board took them**, from one source. That is the
point of the discussion's "one complete record" decision.

### Record

```json
{"kind":"record","rec":"3f2a9c21-…","tally":"T-day-1007","w":1834,
 "tag":"E28011700000020F1A2B3C01","t_ms":1791398400123,"lat":44.1234567,
 "lon":-72.7654321,"acc":5,"age_ms":0,"backlog":false,"sensor":"ok","cal":"ok",
 "id":"plc4-jd5055e"}
```

| Field | Status | Meaning |
|---|---|---|
| `kind` | new | `"record"` |
| `rec` | new | Record ID, the accepted command's `id` |
| `tally` | new | Tally ID from the command |
| `w` | kept (`src:451-454`) | Weight in pounds |
| `tag` | new | The one EPC recorded, upper-case hex |
| `t_ms` | new | Phone time from the command |
| `lat`, `lon`, `acc` | new | Phone fix from the command, or all three `null` when the phone sent none |
| `age_ms`, `backlog` | kept (`src:451-454`) | `age_ms`: milliseconds from capture to this publish attempt, as today. `backlog`: true when an earlier attempt to publish this event went unacknowledged, so it was held and retried. Every event now passes through the queue, so "came from the queue" no longer separates anything. `t_ms` now answers "when" |
| `sensor`, `cal` | kept (`src:451-454`, at-capture rule `src:524-530`) | Loop state and calibration at capture |
| `id` | kept | `plc4-jd5055e` |
| `n`, `total` | **removed** | See *What retires* |

### Void

```json
{"kind":"void","rec":"3f2a9c21-…","t_ms":1791398460000,"lat":44.1234567,
 "lon":-72.7654321,"acc":5,"age_ms":0,"backlog":false,"id":"plc4-jd5055e"}
```

### Tally end

```json
{"kind":"tally_end","tally":"T-day-1007","t_ms":1791420000000,"lat":null,"lon":null,
 "acc":null,"age_ms":0,"backlog":false,"id":"plc4-jd5055e"}
```

Voids and tally ends carry no `w`, `sensor` or `cal`: they record no reading.

## Record ID

**Recommendation: the record ID is the `id` of the record command the board accepted. The app
generates it, and it must be unique for all time and across every phone. A UUID does this, and
the `id` field already admits one (`src:743`).**

Why:

1. **The board holds no ID state.** Nothing has to survive a reboot, and nothing is written to
   flash at boot. The firmware already turned down a persisted boot counter for this reason: *"an
   EEPROM write on every boot, and a board in a reset loop would then wear the flash out while
   failing"* (`src:470-472`).
2. **The app knows the ID before it sends the command.** So a lost ack (QoS 0) costs nothing: the
   app can still void the record by ID, and it can match the record when it arrives on `weight`.
   If the board generated the ID, a lost ack would leave the app not knowing which ID to void.
3. **A duplicate cannot hide.** The dedup ring is RAM, eight deep. After a reboot, an app resend
   of a command the board already executed runs again (the test shows this). Under this scheme the
   second record carries the **same** `rec`, so the consumer can see the duplicate and drop it.
   Under a board-side `boot:seq` scheme the duplicate would get a fresh ID and count twice without
   anyone seeing it.
4. **The backlog needs nothing extra.** The ID is stored in the queue entry with the rest of the
   event. A retry after a lost PUBACK carries the same `rec`, so a double delivery is visible too.

The alternative was a persisted boot counter plus a RAM sequence. It was rejected for reasons 1–3.
A RAM-only sequence with no persistence collides across a reboot; `test_rfid_record.cpp` shows
this with a model of the board.

What the app must do: generate a globally unique `id` for every record command, and a new one for
every retry. `rfid::recordIdFor()` only checks and copies the ID; the board does not check
uniqueness beyond the eight-deep ring.

## Backlog, RAM and delivery

### Today

The backlog is a RAM ring of 200 `WeightRecord`s (`src:320-554`; struct `src:328-333`, depth
`src:343`). When full it drops the oldest and counts it (`src:515-520`). The flush sends the
oldest first and stops at the first failed publish (`src:534-554`). It runs on reconnect
(`src:1394`) and on every telemetry cycle while anything is queued (`src:1824`). `+` publishes
live whenever the board is online, even with records still waiting (`src:1919-1924`).

### Range-only makes the queue a short reconnect buffer

Commands reach the board only through the broker. A command captured on a connection that has
since dropped is thrown away (`src:1119-1123`). So the board can accept a record, void or tally
end only while it is connected, which is the range-only rule Alex ruled. **The queue cannot grow
while the board is out of reach.** It grows only while commands arrive but publishes go
unacknowledged, or for the event or two in flight when the link drops. The USB console cannot
record in this design, because it has no tally.

### Proposed queue

- **One FIFO for all three kinds** (`rfid::EventQueue`, entry `rfid::Event`). A void must never
  publish before the record it cancels, and a tally end must never publish before that tally's
  last record.
- **Every accepted event is queued; nothing publishes from the verb handler.** The flush sends the
  oldest event, at most one per loop pass, at the end of the pass after the ack
  (`rfid::flushOne`). That removes today's live-publish path, which can let a void overtake its
  record (`src:1919-1924`).
- **Depth 16** (`rfid::QUEUE_DEPTH`). A full queue **refuses** the new event with `QUEUE FULL`
  instead of dropping the oldest. Drop-oldest suited recording out of range with nobody told.
  Now every event is answered by an ack to an operator standing at the machine, and dropping the
  oldest could silently lose a void or a tally end. `status.dropped` stays in the payload, and
  this firmware never increments it.
- At-capture `sensor`/`cal` (`src:524-530`) and `age_ms` stay. `backlog` is redefined; see the
  record field table.
- **Why 16:** at 2,448 B it costs what today's 200 × 12 B queue costs (2,400 B), so the depth is
  not traded against RAM. It holds four full capture rings' worth of accepted events (the ring
  is four deep, `src:952-958`). Since the queue cannot grow out of range, 16 covers any stretch
  of acknowledgements failing in range during which an operator is still pressing record. If it
  ever fills, the operator is told and nothing accepted is lost.

The queue is RAM only, as Alex ruled for today's queue (`src:320-322`). A reset with events
queued still loses them, so the consumer must still tolerate a void with no record (item 7
below).

### RAM

Measured on this branch's base with `tools/build-check.sh` on 2026-10-07: *"Global variables use
17792 bytes (54%) of dynamic memory, leaving 14976 bytes for local variables. Maximum is 32768
bytes."* That corrects this note's earlier figure. The 11,196 B in the comment at
`tools/build-check.sh:17-18` is a baseline from before the 4-20 mA work, not today's build.

The sizes below are from the host test, compiled with host `g++`. That host is not the board, but
every member is a fixed-width integer or a `char` array, so the sizes should match on the
Cortex-M4 (inference). The deltas are arithmetic. Nothing here is a firmware build.

| Change to statically allocated RAM | Bytes |
|---|---|
| Event queue, 16 × 152 B (`sizeof(EventQueue<16>)`), replacing `queueBuf` 200 × 12 B | +48 |
| Command capture ring, 4 × `body` 192 → 256 (`src:952-958`) | +256 |
| Tally retires: `rT[50]` of 4-byte `int` (`src:73-74`), `rTindex`, `rTsum` | −208 |
| **Net** | **+96** |

That puts static RAM near 17,888 B and leaves about 14,880 B for stack and heap. That figure is
arithmetic, not a build. Any RAM the reader's driver adds is not determinable until the driver is
chosen. A build of the wired change answers it.

| Stack buffers (locals, not globals) | Today | Proposed | Worst case measured by the test |
|---|---|---|---|
| Ack payload (`src:912`) | 192 | 576 (`rfid::ACK_BUF`) | 559, eight 128-bit EPCs and a 40-char id |
| Weight-topic payload (`src:440`) | 160 | 352 (`rfid::EVENT_BUF`) | 339 |
| `rfid::Decision`, record path only | — | 336 | |

The handler, the ack block and the flush run one after another in `loop()`, not inside each
other. So the peak stack grows by the largest of these, not their sum: under 1 KB, plus however
the driver buffers one raw read. Today's peak stack is not measured. That margin is the reason
for the 14.9 KB figure above, not a substitute for measuring it.

**Binary EPCs and IDs: not worth it at this depth.** Packing the EPC into 16 bytes and the
record and tally IDs into 16 bytes each saves about 80 B per entry, so about 1.3 KB across 16
entries. That is under 9% of the free RAM. The cost is a contract restriction, because IDs would
have to be UUIDs where today any `id`-charset token up to 40 characters is allowed. It also adds
pack and unpack code on both the publish and the dup-replay paths. Revisit it only if the depth
must grow past about 64 entries (64 × 152 B = 9.7 KB, inference).

### Delivery: QoS 1, removed from the queue only on PUBACK

**The problem.** The weight topic publishes at QoS 0 today. `publishTo()` calls the sized
`beginMessage(topic, size)` with the library's default QoS 0 (`src:407`), and at QoS 0
`endMessage()` returns 1 without any acknowledgement (`MqttClient.cpp:222`, `:259`). The
firmware notes that this success means only that the bytes reached the radio coprocessor
(`src:902-905`). A void or tally end lost on the wire leaves no trace. A lost void overcounts a
delivery, and a lost tally end leaves a truckload open.

**What the library supports.** ArduinoMqttClient 0.1.8 publishes at QoS 1 when `beginMessage`
is given `qos = 1` (`MqttClient.cpp:174`). `endMessage()` then spins on `poll()` until the
PUBACK for its packet id arrives (`MqttClient.cpp:244-254`, matched at `:500-507`). It returns 1
on a PUBACK and 0 on timeout or disconnect. The timeout is `setConnectionTimeout()`, which this
firmware sets to `MQTT_CONNECT_TIMEOUT_MS` = 2,000 ms (`src:200`, `src:1552`). The library
tracks one outstanding packet id and has no non-blocking PUBACK API. The session is clean
(`src:1370`), so neither side resends anything after a reconnect.

**The fix: the smallest one that holds.**

1. Publish weight-topic events at QoS 1, not retained. Records go at QoS 1 too: a lost record
   undercounts just as a lost void overcounts, and one path is smaller than two.
2. Remove an event from the queue only when `endMessage()` returns 1, which means the PUBACK
   arrived. On 0 the event stays at the front, marked `retried`, and is sent again. Nothing
   behind it moves first (`rfid::flushOne`).
3. Use the pattern this board already runs: `publishNextDtc()` publishes at QoS 1, one per pass,
   marks published only on `endMessage() == 1`, and backs off after a failure (`src:632-670`).
   It sits at the end of `loop()`, after the ack, because a PUBACK spin there cannot age the
   reading under a command waiting to run (`src:2247-2250`). The event flush goes in the same
   place.
4. After an unacknowledged attempt, wait one telemetry period (5 s) before the next. Retry at
   once on reconnect, as today (`src:1394`). This bounds what a broker that has stopped
   acknowledging costs the loop.

**What this holds against, and what it does not:**

- It holds on this broker: it already acknowledges QoS 1 from this board for `status` and the
  DTC topics (`src:510`, `src:664-669`). A delivered PUBACK proves the broker has the event.
- **A PUBACK lost after the broker took the message means a second delivery.** The consumer must
  treat a repeat `rec`, or a repeat tally end, as the same event. The record-ID scheme makes that
  possible.
- **The cost is a wait on every event.** On the healthy path the wait is one LAN round trip; its
  length on this network is not measured. On a lost PUBACK it is up to 2 s, which equals
  `READING_STALE_MS` (`src:144`). **Alex moved the acks from QoS 1 to QoS 0 for exactly this reason**
  (`src:892-897`): a held reading refuses the next command `NO ADC`. So putting the weight topic
  at QoS 1 needs his approval made knowing this. Placing the flush after the
  ack, one event per pass with backoff, keeps that hazard to the one pass that failed. `status`
  and DTC publishes already carry the same risk.
- **The broker-to-consumer leg is the consumer's.** The board's PUBACK says nothing about the
  subscriber. The consumer must subscribe at QoS 1, and must use a persistent session if events
  published while it is down are to be held for it (consumer impact, item 11).

## What retires

| Item | Where today | Note |
|---|---|---|
| The board's tally `rT[]`, `rTindex`, `rTsum`, `tallyTotal()` | `src:73-97` | The tally lives in the app and consumer (discussion, *Tallies*) |
| The tally's EEPROM block and its save/restore | `src:192-195`, `src:1629-1650` | |
| `b` (clear) | `src:2002-2010`, allowed at `src:769-773` | Replaced by `end` |
| `sum` | `src:1990-2000` | Console-only today |
| `-` as "remove last entry" | `src:1934-1955` | Becomes void-by-ID |
| `TALLY FULL`, `TALLY EMPTY`, the LCD tally warnings | `src:1926-1931`, `src:1950-1955`, `src:1700-1702` | LCD shows weight only (discussion) |
| `n`, `total` on `weight` | `src:451-455` | **Consumer contract change, owned by the consumer coordinator** |
| `n`, `total` on `ack` | `src:920-926`, `src:2244` | **Consumer contract change, owned by the consumer coordinator** (the app reads acks) |
| `n`, `total` on `status` | `src:496`, `src:502` | Inference: retires with the tally it reports. Same owner |
| Drop-oldest on a full queue | `src:515-520` | Replaced by `QUEUE FULL` refusal |
| Live publish from the `+` handler, and QoS 0 on `weight` | `src:1919-1924`, `src:407` | All events through the queue, QoS 1. **The QoS change is a consumer contract change, owned by the consumer coordinator** |
| The Bluetooth console (HC-05 on `Serial1`) | `src:26-44`, `src:1460`, `src:2302-2307` | Mothballed (discussion). The reader takes D0/D1. Inference from the discussion: the USB console stays as the service path, without `+`, `-`, `b` or `sum` |

`tare`, `kv`, `c`, `w` and `sm` do not change.

## Consumer impact

What the consumer must change. The consumer is the estate handler for PLC #4; `src:334-342`
names `/opt/plc1-monitoring` as where it lives. It belongs to the consumer coordinator.

1. Read `kind` on every `weight` message. A message with no `kind` is from firmware before this
   change, and should be handled as today.
2. **Stop reading `n` and `total`** on `weight`, `ack` and `status`. Compute tally totals from
   records minus voids.
3. Key each record by `rec`, and make the insert idempotent. A second record with a `rec`
   already seen is either an app resend after a board reboot, or a QoS 1 redelivery after a lost
   PUBACK. Drop it and log it. Voids are idempotent on `rec` and tally ends on `tally` in the
   same way.
4. Take "when" from `t_ms`, which is phone time. `backlog: true` now means the board's first
   attempt to publish the event went unacknowledged.
5. Take "where" from `lat`/`lon`/`acc`, which may all be `null`. The yard zone is derived from
   these downstream (discussion, *Decided*).
6. Record the move against `tag`, the one EPC.
7. **Void:** reverse the move whose `rec` matches. A void naming a `rec` never received must be
   kept or flagged, not ignored. The queue no longer drops anything, but a board reset still
   loses whatever was queued in RAM.
8. **Tally end:** close the tally named in `tally`. A record arriving for a closed tally means the
   app and consumer disagree. Flag it.
9. **Tally start and attribution are app and consumer work, not the board's.** The app opens a
   tally with an ID, and attribution is set at start: a daily log, or a delivery with its
   customer/order (discussion, *Tallies*). No message on the board's topics carries either. The
   consumer coordinator owns how the app delivers them to the consumer. The board only stamps the
   tally ID on records and publishes `tally_end`.
10. The backlog depth changes from 200 to 16, and full now refuses instead of dropping. Update the
    consumer's documented bound (`src:334-342`).
11. **Subscribe to `plc4/jd5055e/weight` at QoS 1**, so the broker-to-consumer leg matches the
    board's. If events published while the consumer is down must be held for it, use a
    persistent session with a fixed client id. What the broker holds for an offline subscriber
    depends on its configuration, which this repo cannot see.

## Open questions only Alex can answer

None remain in this note. The three earlier questions are answered: two by his rulings above, and
the tally-start question is now consumer item 9. The proposal as a whole still waits for his
approval as a firmware change.

## Implementation notes for the gated firmware change (not done)

- Size the command capture buffer (`src:953`), the ack buffer (`src:912`) and the weight payload
  buffer (`src:440`) to `rfid::CMD_BUF`, `ACK_BUF` and `EVENT_BUF`.
- The existing `jsonField` reads quoted strings only (`src:844-864`). `t_ms`, `lat`, `lon` and
  `acc` are JSON numbers; `rfid::numberToken` reads them.
- The verb allowlist (`src:769-773`) gains `end` and loses `b`.
- The dup-replay path (`src:1147-1152`) adds `rec` when the stored verb is `+` and `ok`.
- Replace `queuePush`/`queueFlush`, the live publish in `+` (`src:1919-1924`), and the flush
  calls at `src:1394` and `src:1824` with the single QoS 1 flush at the end of `loop()`, placed
  next to `publishNextDtc()`. See *Delivery*.
- No signing, nonces, keys or replay defence (Alex, 2026-10-07: "this is a firewood buisiness not
  norad").
