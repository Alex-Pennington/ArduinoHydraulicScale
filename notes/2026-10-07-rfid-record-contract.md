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
branch. Anything marked **inference** is reasoned from that source, not measured.

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

QoS, retain and the dedup rules stay as `03` states them today. Commands go in at QoS 1 and are
never retained. Acks go out at QoS 0. The weight topic is QoS 0 and not retained.

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

GPS is optional: the phone may have no fix. Time is required. Which of the two must be present is
a call for Alex; see *Open questions*.

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
GPS, then the chosen tag's syntax, then the loop state, then the reader, then the tag decision.
The cheap field checks run before the loop check and the tag read.

**Tag list rules** (`rfid::decide`):

- A tag the reader reports several times in one read counts once, at its strongest RSSI.
- The list is sorted strongest first. Tags with equal RSSI sort by EPC, so the order does not
  depend on the order the reader reported them in.
- The board keeps the 16 strongest. The ack lists the first 8. `tags_seen` counts every distinct
  tag in the read, so the app can tell when the list is cut short.
- A chosen tag is checked against the whole read, not the 16 it keeps. It is accepted if it is in
  this read, whatever else is also in view.
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
| `age_ms`, `backlog` | kept (`src:451-454`) | Same meaning as today. `t_ms` now answers "when"; these still say whether the board held the event before publishing |
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
4. **Offline buffering needs nothing extra.** The ID is stored in the backlog entry with the rest
   of the event.

The alternative was a persisted boot counter plus a RAM sequence. It was rejected for reasons 1–3.
A RAM-only sequence with no persistence collides across a reboot; `test_rfid_record.cpp` shows
this with a model of the board.

What the app must do: generate a globally unique `id` for every record command, and a new one for
every retry. `rfid::recordIdFor()` only checks and copies the ID; the board does not check
uniqueness beyond the eight-deep ring.

## Backlog and ordering

Today (`src:320-554`), the backlog is a RAM ring of 200 `WeightRecord`s (`src:328-333`,
`src:343`). When full it drops the oldest and counts it (`src:515-520`). The flush sends the
oldest first and stops at the first failed publish (`src:534-554`). It runs on reconnect
(`src:1394`) and on every telemetry cycle while anything is queued (`src:1824`).

Proposed:

- **One FIFO for all three kinds** (`rfid::EventQueue`, entry `rfid::Event`). A void must never
  publish before the record it cancels, and a tally end must never publish before that tally's
  last record.
- **The ordering rule** (`rfid::mustQueue`): an event is published live only when the queue is
  empty. Otherwise it goes to the back of the queue. Today `+` publishes live whenever the board
  is online, even with records still waiting (`src:1919-1924`). That reorders weights harmlessly,
  but it would let a void overtake its record.
- Drop-oldest, the `dropped` count, at-capture `sensor`/`cal`, `age_ms` and `backlog` all stay.
- A dropped event can leave a void with no record, or a record with no void. The consumer must
  handle both (see *Consumer impact*). `dropped` on `status` already says this has happened.

**Inference: the backlog matters much less in this design.** Commands reach the board only through
the broker. A command captured on a connection that has since dropped is thrown away
(`src:1119-1123`). So the board can accept a record, void or tally end only while it is online.
The queue then holds only events whose live publish failed in that same pass, and anything queued
behind them. Under the BT console today, `+` worked out of range and filled the queue. The USB
console cannot record in this design, because it has no tally.

### Sizes, measured by the host test

| Item | Size | Today |
|---|---|---|
| Worst-case record event | 339 bytes | `publishWeight` buffer 160 (`src:440`) |
| Worst-case record command | 224 bytes | capture buffer 191 usable (`CmdCapture.body[192]`, `src:953`). An oversize command is dropped with no ack (`src:1138`) |
| Worst-case multi-tag ack (8 × 128-bit EPCs, 40-char id) | 559 bytes | ack buffer 192 (`src:912`) |
| One backlog entry, `sizeof(rfid::Event)` | 152 bytes | 12 bytes (`WeightRecord`) |
| 200 entries | 30,400 bytes | 2,400 bytes |
| 32 entries | 4,864 bytes | |

`tools/build-check.sh:17-18` records a baseline of 11,196 bytes of RAM as 34% on the UNO R4 (taken before the 4-20 mA work, so today's figure is not stated there), which is about
32 KB in all. **A 200-entry event queue does not fit on that board.** Given the inference above, a
much shallower queue would do, but how deep is a decision for Alex. The consumer documents the
200 bound independently (`src:334-342`), so a change there must be told to the consumer's owner.
The platform is not chosen yet, so this table is for sizing whatever is chosen.

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
| The Bluetooth console (HC-05 on `Serial1`) | `src:26-44`, `src:1460`, `src:2302-2307` | Mothballed (discussion). The reader takes D0/D1. Inference from the discussion: the USB console stays as the service path, without `+`, `-`, `b` or `sum` |

`tare`, `kv`, `c`, `w` and `sm` do not change.

## Consumer impact

What the consumer must change. The consumer is the estate handler for PLC #4; `src:334-342`
names `/opt/plc1-monitoring` as where it lives. It belongs to the consumer coordinator.

1. Read `kind` on every `weight` message. A message with no `kind` is from firmware before this
   change, and should be handled as today.
2. **Stop reading `n` and `total`** on `weight`, `ack` and `status`. Compute tally totals from
   records minus voids.
3. Key each record by `rec`, and make the insert idempotent: a second record with a `rec` already
   seen is a resend after a reboot. Drop it and log it.
4. Take "when" from `t_ms`, which is phone time. Keep `age_ms` and `backlog` as the board's own
   statement that it held the event.
5. Take "where" from `lat`/`lon`/`acc`, which may all be `null`. The yard zone is derived from
   these downstream (discussion, *Decided*).
6. Record the move against `tag`, the one EPC.
7. **Void:** reverse the move whose `rec` matches. A void naming a `rec` never received (its record
   was dropped from a full queue) must be kept or flagged, not ignored.
8. **Tally end:** close the tally named in `tally`. A record arriving for a closed tally means the
   app and consumer disagree. Flag it.
9. The tally's attribution (daily log or delivery, and the customer/order) is not carried by any
   message here; see *Open questions*.
10. If the backlog depth changes from 200, update the consumer's documented bound
    (`src:334-342`).

## Open questions only Alex can answer

1. **Where does a tally's start and its attribution reach the consumer?** The discussion settles
   that attribution is set at start in the app, but not how it travels. The options are a board
   `start` verb published on `weight` like `end`, or the app talking to the consumer directly.
   This proposal publishes only `end`, because that is what the discussion records.
2. **Is a record refused when the phone has no GPS fix?** This proposal accepts it with `null`
   position. It requires phone time.
3. **Backlog depth on whatever platform is chosen.** 200 entries of the new event (30.4 KB) do not
   fit in the UNO R4's RAM.

## Implementation notes for the gated firmware change (not done)

- Grow the command capture buffer (`src:953`) and the ack buffer (`src:912`) to at least the
  measured worst cases above. Grow the weight payload buffer (`src:440`) too.
- The existing `jsonField` reads quoted strings only (`src:844-864`). `t_ms`, `lat`, `lon` and
  `acc` are JSON numbers; `rfid::numberToken` reads them.
- The verb allowlist (`src:769-773`) gains `end` and loses `b`.
- The dup-replay path (`src:1147-1152`) adds `rec` when the stored verb is `+` and `ok`.
- The weight topic is QoS 0 (`src:407`). A void or tally end can be lost on the wire with no
  trace, the same as a record today (`src:902-905`). This proposal does not change that.
- No signing, nonces, keys or replay defence (Alex, 2026-10-07: "this is a firewood buisiness not
  norad").
