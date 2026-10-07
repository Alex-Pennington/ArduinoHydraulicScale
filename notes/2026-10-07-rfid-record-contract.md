# RFID record and tally — MQTT contract proposal (2026-10-07)

**Approved by Alex 2026-10-07 for consumer hand-off. Board firmware is gated, not approved.**
Nothing in this note is in the firmware, and nothing changes on the tractor until Alex approves a
specific firmware change. The binder card `references/03_mqtt_contract.md` follows once a change
is approved and flashed. Until then the card describes the running firmware, not this note.

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
- **Lost voids and tally ends: "Consumer confirms, app resends".** He chose this over two
  alternatives: QoS 1 with a short wait on the board, and QoS 1 for voids and tally ends only.
  The option was put to him as: *"No waiting on the board. The consumer confirms each record and
  end and the app re-sends what goes unconfirmed. Work for the consumer coordinator, not the
  board."* That is the coordinator's option text, not his sentence. The weight topic stays at
  QoS 0. See *Confirmation*.
- **Hand-off: "Approve and hand off".** The spec, with that fix, is approved for handing to the
  consumer coordinator.

## Messages at a glance

| Message | Topic | Direction | New or changed |
|---|---|---|---|
| Record command (`+`) | `plc4/jd5055e/cmd` | app → board | Changed: tally, phone time, GPS, optional chosen tag |
| Void command (`-`) | `plc4/jd5055e/cmd` | app → board | Changed: names the record it voids and that record's tally |
| Tally-end command (`end`) | `plc4/jd5055e/cmd` | app → board | New verb. It replaces `b` |
| Ack | `plc4/jd5055e/ack` | board → app | Changed: `rec`, `tally`, `tag`, `w`, `tags`, `tags_seen`. `n` and `total` are dropped |
| Record event | `plc4/jd5055e/weight` | board → consumer | Changed: `kind`, `rec`, `tally`, `tag`, `t_ms`, `lat`, `lon`, `acc`. `n` and `total` are dropped |
| Void event | `plc4/jd5055e/weight` | board → consumer | New. Carries `rec` and `tally` |
| Tally-end event | `plc4/jd5055e/weight` | board → consumer | New |
| Confirmation | `plc4/jd5055e/confirm` | consumer → app | New. **Consumer and app own it; the board takes no part** |

QoS, retain and the dedup rules stay as `03` states them today. Commands go in at QoS 1 and are
never retained. Acks go out at QoS 0. The weight topic stays at QoS 0 and is not retained
(`src:407`). End-to-end delivery is confirmed by the consumer and the app, on a new topic that the
board neither publishes nor subscribes to; see *Confirmation*.

## Commands on `plc4/jd5055e/cmd`

The envelope does not change. `id` and `verb` are read and checked as they are today: same
charset, 40-character cap, the eight-deep dedup ring, and a repeat replays the stored verdict
(`src:1140-1170`, `src:1147-1152`). All new fields are flat members of the same object.

### Record — verb `+`

```json
{"id":"3f2a9c21-7b4e-4c1d-9a0e-5d6f7a8b9c0d","verb":"+","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3",
 "t_ms":1791398400123,"lat":44.1234567,"lon":-72.7654321,"acc":5}
```

The retry after a `MULTIPLE TAGS` refusal uses a **new `id`** and names the tag the operator
tapped:

```json
{"id":"8e1b44d0-2c55-4a1f-b1e3-0a9d6c7e2f11","verb":"+","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3",
 "t_ms":1791398406410,"lat":44.1234567,"lon":-72.7654321,"acc":5,
 "tag":"E28011700000020F1A2B3C01"}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `id` | string | yes | As today. **It becomes the record ID if the record is accepted** (see *Record ID*) |
| `verb` | `"+"` | yes | |
| `tally` | string, an app-generated UUID | yes | The open tally this record goes to. The app picks it when a daily log and a delivery are both open. Must be unique; see *Tallies*. The board checks only the `id` charset and the 40-character cap |
| `t_ms` | integer | yes | The phone's clock at the moment of the press: UTC milliseconds since the Unix epoch. See *Phone time* |
| `lat`, `lon` | number, decimal degrees | all three, or none | Phone fix. The board keeps 1e-7 degree and truncates extra decimals. Exponent form (`1e-7`) is refused |
| `acc` | number, metres | all three, or none | Phone fix accuracy. The board rounds it **up** to whole metres, 0–65535 |
| `tag` | string, upper-case hex EPC | no | The chosen tag, on the retry only |

GPS is optional: the phone may have no fix, and Alex ruled that such a record is accepted with no
position. Time is required.

**Phone time.** GPS and time come from the phone (`notes/rfid-inventory-discussion.md:28-29`), so
`t_ms` is the phone's clock when the operator pressed the button. A resend repeats it unchanged
(*Rules for the app*). The board has no clock of its own (`notes/rfid-inventory-discussion.md:78`),
so it checks only the shape. An absent `t_ms` is refused `NO TIME` (`include/rfid_record.h`,
`readTimeAndFix()`). A string, a fraction, an exponent, zero, a negative value or more than 15
digits is refused `BAD TIME` (`parseEpochMs()`, same header). A positive integer from a phone
whose clock is wrong is **accepted and published as sent**. The board cannot tell, and judging
plausibility (against the broker receive time, for example) is the consumer's job.

**A retry must carry a new `id`.** A repeated `id` is not run again: the board replays the stored
verdict (`src:1147-1152`). If a retry reused the `id` of the refused attempt, it would get the
refusal back without the board reading the tags again.

### Void — verb `-`

```json
{"id":"v-5b0c","verb":"-","rec":"3f2a9c21-7b4e-4c1d-9a0e-5d6f7a8b9c0d",
 "tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3","t_ms":1791398460000,"lat":44.1234567,
 "lon":-72.7654321,"acc":5}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `rec` | string, the `id` charset | yes | The record ID being voided. The app sends the last record of the tally the operator is in |
| `tally` | as for `+` | yes | The tally the voided record went to. The app knows it, because it chose the tally when it sent the record |
| `t_ms`, `lat`, `lon`, `acc` | as for `+` | as for `+` | |

The board does not hold the tally, so it cannot check that `rec` exists or that it belongs to
`tally`. It checks only the syntax: `rec` first, then `tally`, then time and GPS. Matching the
void to a record is the consumer's job.

### Tally end — verb `end`

```json
{"id":"e-77a1","verb":"end","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3","t_ms":1791420000000}
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
| No tally ID / malformed (`+`, `-`, `end`) | false | `NO TALLY` / `BAD TALLY` | none |
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
 "tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3","tag":"E28011700000020F1A2B3C01","w":1834}
{"id":"7f3a0001","verb":"+","ok":false,"detail":"MULTIPLE TAGS",
 "tags":[{"epc":"E28011700000020F1A2B3C02","rssi":-45},
         {"epc":"E28011700000020F1A2B3C01","rssi":-70}],"tags_seen":2}
{"id":"7f3a0002","verb":"+","ok":false,"detail":"TAG NOT IN VIEW","tags":[],"tags_seen":0}
{"id":"v-5b0c","verb":"-","ok":true,"detail":"voided","rec":"3f2a9c21-…"}
{"id":"e-77a1","verb":"end","ok":true,"detail":"ended","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3"}
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
{"kind":"record","rec":"3f2a9c21-…","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3","w":1834,
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
| `age_ms`, `backlog` | kept (`src:451-454`) | Same meaning as today: `backlog` is true when the event was held on the machine and published later, and `age_ms` is the time between capture and publication. `t_ms` now answers "when" |
| `sensor`, `cal` | kept (`src:451-454`, at-capture rule `src:524-530`) | Loop state and calibration at capture |
| `id` | kept | `plc4-jd5055e` |
| `n`, `total` | **removed** | See *What retires* |

### Void

```json
{"kind":"void","rec":"3f2a9c21-…","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3",
 "t_ms":1791398460000,"lat":44.1234567,"lon":-72.7654321,"acc":5,"age_ms":0,
 "backlog":false,"id":"plc4-jd5055e"}
```

### Tally end

```json
{"kind":"tally_end","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3","t_ms":1791420000000,"lat":null,"lon":null,
 "acc":null,"age_ms":0,"backlog":false,"id":"plc4-jd5055e"}
```

Voids and tally ends carry no `w`, `sensor` or `cal`: they record no reading. Every event
carries `tally`.

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
3. **A duplicate cannot hide.** The app never resends a record (*Rules for the app*), but a
   QoS 1 redelivery of a record command can reach the board twice; see *Repeated record ID*. If
   the dedup ring has lost the `id` by then, the board runs the command again. Under this scheme
   the second record carries the **same** `rec`, so the consumer sees the duplicate and keeps the
   first. Under a board-side `boot:seq` scheme the duplicate would get a fresh ID and count twice
   without anyone seeing it.
4. **The backlog and the confirmation need nothing extra.** The ID is stored in the queue entry
   with the rest of the event. A confirmation names the same `rec` the app already holds, and a
   resent void carries it too, so a double delivery is visible.

The alternative was a persisted boot counter plus a RAM sequence. It was rejected for reasons 1–3.
A RAM-only sequence with no persistence collides across a reboot; `test_rfid_record.cpp` shows
this with a model of the board.

What the app must do: generate a globally unique `id` for every record command, and a new one for
every retry. `rfid::recordIdFor()` only checks and copies the ID; the board does not check
uniqueness beyond the eight-deep ring.

### Repeated record ID

A second record with a `rec` already seen does not come from the app, which never resends a
record. It comes from a QoS 1 redelivery of the same record command, which the board then runs a
second time. Read from source:

- Every command `id` goes into the dedup ring with its verdict: refusals at `src:1178`, everything
  else at `src:2243`, through `cmdRemember()` (`src:877-885`). A repeated `id` is acked from the
  ring and **not run** (`src:1147-1152`).
- The ring is RAM and eight deep (`src:742-751`). It forgets an `id` after a reboot, or once
  eight newer command ids have pushed it out (`src:883-884`). The test shows both.
- The board connects with a clean session, the library default (`MqttClient.cpp:69`,
  `:900-901`; nothing in `src/` calls `setCleanSession`). It PUBACKs an inbound command as soon as
  the payload is read (`MqttClient.cpp:735`). **Inference:** the broker keeps nothing for the
  board across a reconnect, so a redelivery reaches the board only from the app's side, as a QoS 1
  PUBLISH the app's client sent again and the broker forwarded twice.

So a redelivery that arrives while the ring still holds the `id` is a dedup replay and records
nothing. A second run happens only after a reboot, or after eight newer commands. That second run
reads the scale and the reader again, so its `w`, and possibly its `tag`, can differ from the
first. **The consumer keeps the first** and drops the repeat (consumer item 3).

## Tallies

**A tally ID must be unique for all time and across every phone, like a record ID. The app
generates it as a UUID.** This is a hard requirement, not a recommendation. A reused tally ID
merges two loads, or two days, without trace. A UUID is 36 characters and fits the 40-character
cap that tally IDs share with command IDs (`rfid::ID_MAX`, `src:743`), so the cap does not
change. The board validates only the shape (`rfid::idValid()`, the `id` charset) and the length.
It cannot detect a reused tally ID.

Every event names its tally: records and tally ends always did, and voids now do too.

**A record or void that arrives after its tally's `tally_end` still applies.** The consumer
counts it and flags it. The ordering rule makes this rare, because one FIFO and one topic deliver
in the order the board took them (*Proposed queue*). It can still happen. For example, a void
whose first send was lost can have its resend arrive after the tally end, or the operator can act
on a tally the app has already ended.

**Consumer-side rulings.** Alex ruled the following on 2026-10-07. They are relayed by the
consumer coordinator, not his words recorded here:

- The app publishes tally start on its own topic, and the consumer confirms it.
- A delivery's customer and order come from Odoo.
- Attribution cannot be corrected after a tally starts. End the tally and start a new one.

None of these touches the board. The design is in greybeard-plc-consumer, branch
`docs/plc4-rfid-consumer-design`, `docs/design/plc4-rfid-consumer.md`.

## Backlog and RAM

### Today

The backlog is a RAM ring of 200 `WeightRecord`s (`src:320-554`; struct `src:328-333`, depth
`src:343`). When full it drops the oldest and counts it (`src:515-520`). The flush sends the
oldest first and stops at the first failed publish (`src:534-554`). It runs on reconnect
(`src:1394`) and on every telemetry cycle while anything is queued (`src:1824`). `+` publishes
live whenever the board is online, even with records still waiting (`src:1919-1924`).

### Range-only makes the queue a short buffer

Commands reach the board only through the broker. A command captured on a connection that has
since dropped is thrown away (`src:1119-1123`). So the board can accept a record, void or tally
end only while it is connected, which is the range-only rule Alex ruled. **The queue cannot grow
while the board is out of reach.** It holds only the events whose publish failed as the link
dropped, plus anything queued behind them. The USB console cannot record in this design, because
it has no tally.

### Proposed queue

- **One FIFO for all three kinds** (`rfid::EventQueue`, entry `rfid::Event`). A void must never
  publish before the record it cancels, and a tally end must never publish before that tally's
  last record.
- **The ordering rule** (`rfid::mustQueue`): an event is published live only when the board is
  online and the queue is empty. Otherwise it goes to the back of the queue. That closes today's
  live-publish-past-a-backlog path (`src:1919-1924`), which would let a void overtake its record.
  The flush keeps today's rules and call sites (`src:534-554`, `src:1394`, `src:1824`).
- **Depth 16** (`rfid::QUEUE_DEPTH`). A full queue **refuses** the new event with `QUEUE FULL`
  instead of dropping the oldest. Drop-oldest suited recording out of range with nobody told.
  Now every event is answered by an ack to an operator standing at the machine, and dropping the
  oldest could silently lose a void or a tally end. `status.dropped` stays in the payload, and
  this firmware never increments it.
- At-capture `sensor`/`cal` (`src:524-530`), `age_ms` and `backlog` stay as they are today.
- **Why 16:** at 2,448 B it costs what today's 200 × 12 B queue costs (2,400 B), so the depth is
  not traded against RAM. It holds four full capture rings' worth of accepted events (the ring
  is four deep, `src:952-958`), which is more than can be in flight when a link drops. If it ever
  fills, the operator is told and nothing accepted is lost.

The void's `tally` costs no RAM: `rfid::Event` already held a tally for records and tally ends,
so the entry stays 152 B. The worst-case void command (two 40-character IDs) is 232 B against
`CMD_BUF` 256, and the worst-case void event is 242 B against `EVENT_BUF` 352. Both are measured
by the host test. No buffer constant changes.

The queue is RAM only, as Alex ruled for today's queue (`src:320-322`). A reset with events
queued loses them, and a QoS 0 publish can be lost on the wire after `publishTo()` reports
success (`src:902-905`). Both cases are what the confirmation leg catches.

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

| Stack buffers (locals) | Today | Proposed | Worst case measured by the test |
|---|---|---|---|
| Ack payload (`src:912`) | 192 | 576 (`rfid::ACK_BUF`) | 559, eight 128-bit EPCs and a 40-char id |
| Weight-topic payload (`src:440`) | 160 | 352 (`rfid::EVENT_BUF`) | 339 |
| `rfid::Decision`, record path only | — | 336 | |

**On a tag refusal the `Decision` and the ack buffer are live together**, because the ack's tag
list (`Ack.tags`) points into the `Decision`. That is 336 + 576 = **912 B** at once, which is the
peak, plus however the driver buffers one raw read. The weight payload is used separately, in the
handler or the flush. Today's peak stack is not measured, and the margin above is not a
substitute for measuring it.

Inference: today the ack is written at the end of `loop()` (`src:2218-2244`), after the handler
has returned. If the wired change keeps that structure, the `Decision` cannot live on the
handler's stack. It moves to static RAM, which adds 336 B to the net above (+432 in all). Writing
the refusal ack from the handler would avoid that. That choice is for the firmware change.

**Binary EPCs and IDs: not worth it at this depth.** Packing the EPC into 16 bytes and the
record and tally IDs into 16 bytes each saves about 80 B per entry, so about 1.3 KB across 16
entries. That is under 9% of the free RAM. The cost is a contract restriction, because IDs would
have to be UUIDs where today any `id`-charset token up to 40 characters is allowed. It also adds
pack and unpack code on both the publish and the dup-replay paths. Revisit it only if the depth
must grow past about 64 entries (64 × 152 B = 9.7 KB, inference).

## Confirmation — PROPOSAL FOR THE CONSUMER COORDINATOR

**The consumer and the app own this leg. The board takes no part:** it neither publishes nor
subscribes to the confirmation topic, and it subscribes only to `cmd` (`src:1387-1388`). Alex
ruled this design (*Ruled* above). What follows is the board side's proposal for the consumer
coordinator to adopt or change.

### Why it exists

The weight topic is QoS 0 (`src:407`). At QoS 0 `endMessage()` returns 1 with no
acknowledgement (`MqttClient.cpp:222`, `:259`), and the firmware notes that this success means
only that the bytes reached the radio coprocessor (`src:902-905`). A void or tally end lost on
the wire leaves no trace. A lost void overcounts a delivery, and a lost tally end leaves a
truckload open. The same is true of a record and of anything queued in RAM when the board resets.

The same holds for the consumer. A record published while the consumer is restarting is lost,
because nothing on a QoS 0, unretained topic waits for a subscriber that is not there. The board
cannot see this. What reveals it is the app's unconfirmed-record mark (*Rules for the app*): the
record gets no confirmation, and the operator is shown that. The consumer coordinator reports
that it will time its deploys for when the scale is idle.

The board's QoS 1 machinery cannot be leaned on instead. `publishStatus()` discards
`endMessage()`'s result (`src:512`). `publishNextDtc()` reads it (`src:664-669`), but nothing in
this repo records whether those publishes are actually acknowledged. **Nothing here shows that
this broker returns PUBACKs to this board.**

### Topic and payload

`plc4/jd5055e/confirm`, published by the consumer at QoS 1, not retained. The app subscribes.

```json
{"kind":"record","rec":"3f2a9c21-…","by":"<consumer client id>"}
{"kind":"void","rec":"3f2a9c21-…","by":"<consumer client id>"}
{"kind":"tally_end","tally":"c71e0b52-9d4a-4f3e-8a26-1b7f04d9e8a3","by":"<consumer client id>"}
```

| Field | Meaning |
|---|---|
| `kind` | The kind of the event being confirmed, as on `weight` |
| `rec` | For `record` and `void`: the record ID |
| `tally` | For `tally_end`: the tally ID |
| `by` | The consumer's client id. It is not `id`, so the payload cannot be confused with a board message, which uses `id` for `plc4-jd5055e` |

The topic sits under the machine's root because it is about this machine's events. Under the
reportedly ACL-free broker (`BRIEF.md`), topic discipline is the only separation, and the board
never subscribes outside `cmd`.

### Rules for the consumer

1. Confirm only after the move is stored durably. A confirmation is a promise that the event
   counts.
2. **Confirm every receipt, duplicates included.** If the first confirmation was lost, only a
   confirmation of the resend closes the loop. A duplicate is confirmed and dropped, never
   applied twice.
3. A void whose `rec` the consumer has never seen is kept and flagged as an orphan (consumer
   impact, item 7). It is still confirmed, because the app must stop resending it.

### Rules for the app

**Voids and tally ends are resent until confirmed.**

- After an accepted void (`voided`) or tally end (`ended`), wait **10 s** for its confirmation.
  That is two telemetry periods (`TELEMETRY_MS` = 5,000, `src:306`). An event queued on the board
  goes out at the latest on the next telemetry-cycle flush (`src:1824`), and the consumer needs
  time to store it. The 10 s is a starting value for the consumer coordinator to tune.
- If no confirmation comes, resend, again every 10 s while the board answers. Show the event as
  unconfirmed from the first timeout.
- **A resend must carry a NEW command `id`.** This is the trap. The board's dedup ring replays the
  stored verdict for a repeated `id` and does not run the verb again (`src:1147-1152`). A resend
  under the old `id` would come back `voided` and publish nothing.
- **A resend repeats the original `rec` or `tally`, `t_ms` and GPS unchanged.** `t_ms` is when the
  operator pressed the button, not when the app retried. The board then publishes an event
  identical to the first, apart from `age_ms` and `backlog`, and the consumer matches it as a
  duplicate.
- A resend refused `QUEUE FULL`, or one with no ack at all, is simply retried on the next interval
  with yet another new `id`.

**Records are not resent.** A resent `+` would read the scale again: a second weighing, not a
resend. The consumer still confirms records, and the app uses that to show each record as
confirmed or unconfirmed. For a record still unconfirmed after the same 10 s, the app marks it
unconfirmed and leaves the operator to act. The operator can wait, since a backlog flush may
still deliver it, or void it and weigh again. If the record did arrive, the void cancels it. If
it never arrived, the consumer receives a void for an unknown `rec`, keeps it and confirms it
(rule 3 above).

### What the board already does for this

Nothing new is needed on the board. A void or tally end is published each time it is accepted
under a new `id`. The board keeps no record of which `rec` it has voided or which tally it has
ended, because the tally lives in the app and consumer. `test_rfid_record.cpp` models the board's
command path to show this. A repeat under the same `id` is a dedup replay and publishes nothing.
A repeat under a new `id` publishes again, with the same payload. Two resends while the board is
offline both queue and flush in order with `backlog: true`. A model of the consumer's rules then
counts a record once after a duplicate record, a duplicate void and a duplicate tally end, and
keeps the first of two records with the same `rec`. It keeps, flags and confirms an orphan void,
and it flags a record and a void that arrive after their tally end while still counting both.

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
| Live publish past a waiting backlog | `src:1919-1924` | Live only when the queue is empty (`rfid::mustQueue`) |
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
   already seen is a QoS 1 redelivery of the record command that the board ran again after a
   reboot or a ring eviction (*Repeated record ID*). Its `w` and `tag` may differ. **Keep the
   first,** drop the repeat and log it. Voids are idempotent on `rec`, and tally ends on `tally`,
   because the app resends both until they are confirmed.
4. Take "when" from `t_ms`, the phone's clock at the press. The board does not check it for
   plausibility (*Phone time*). `age_ms` and `backlog` mean what they mean today.
5. Take "where" from `lat`/`lon`/`acc`, which may all be `null`. The yard zone is derived from
   these downstream (discussion, *Decided*).
6. Record the move against `tag`, the one EPC.
7. **Void:** reverse the move whose `rec` matches. The void names the record's `tally` too. A
   void naming a `rec` never received is kept, flagged as an orphan and confirmed, never
   ignored. The queue no longer drops anything, but a board reset still loses whatever was
   queued in RAM.
8. **Tally end:** close the tally named in `tally`. A record or void arriving after it still
   applies. Count it and flag it (*Tallies*).
9. **Tally start and attribution are app and consumer work, not the board's.** The app opens a
   tally with a UUID, and attribution is set at start: a daily log, or a delivery with its
   customer/order (discussion, *Tallies*). No message on the board's topics carries either. The
   consumer-side rulings and the design pointer are under *Tallies*. The board only stamps the
   tally ID on records and voids, and publishes `tally_end`.
10. The backlog depth changes from 200 to 16, and full now refuses instead of dropping. Update the
    consumer's documented bound (`src:334-342`).
11. **Build the confirmation leg** (see *Confirmation*): publish confirmations on
    `plc4/jd5055e/confirm`, and have the app resend unconfirmed voids and tally ends under a new
    command `id`, and flag unconfirmed records. The consumer and the app own all of it.

## App impact — for whoever owns greybeard-plc4-pwa

These are app-side and not this repo's to fix. They come from the consumer coordinator's review
of this contract (2026-10-07, relayed by Ymir). They were not verified here, because that repo
is outside this workspace.

- **Confirmations can be lost (C5).** The app connects with `clean_session` and a random client
  id, so a confirmation published while it is disconnected is lost. Its pending resends are held
  in RAM. It needs a third publish topic and a subscription to `plc4/jd5055e/confirm`.
- **No tally total on the phone (C6).** With `n` and `total` retired, the board no longer reports
  a total. The app sums its confirmed records.
- **New frames refused (C8).** The app's `parse_weight` requires `w`, `sensor` and `cal` and
  validates `n`/`total`. So `void` and `tally_end` frames, and records without `n`/`total`, would
  be refused once the board is flashed.
- **Resend id rule (reported by the consumer coordinator, citing greybeard-plc4-pwa
  `commands.mjs:9-10`).** The PWA's rule is to resend under the SAME id, never a new one. This
  contract requires a NEW command `id` on every resend of a void or tally end. Under the PWA's
  rule those resends are deduplicated as replays (`src:1147-1152`), and nothing is published. The
  same-id rule stays correct for a `+` retry after a lost ack, since the board replays the stored
  verdict and the record is not weighed again. Only void and tally-end resends need a new `id`.
  The retry after a `MULTIPLE TAGS` refusal also needs one (*Record*), because it must read
  the tags again.

## Open questions only Alex can answer

None remain in this note. The earlier questions are answered: by his rulings above, and by
moving tally start to consumer item 9. The spec is approved for consumer hand-off. The board
firmware change still needs his separate approval.

## Implementation notes for the gated firmware change (not done)

- Size the command capture buffer (`src:953`), the ack buffer (`src:912`) and the weight payload
  buffer (`src:440`) to `rfid::CMD_BUF`, `ACK_BUF` and `EVENT_BUF`.
- The existing `jsonField` reads quoted strings only (`src:844-864`). `t_ms`, `lat`, `lon` and
  `acc` are JSON numbers; `rfid::numberToken` reads them.
- The verb allowlist (`src:769-773`) gains `end` and loses `b`.
- The dup-replay path (`src:1147-1152`) adds `rec` when the stored verb is `+` and `ok`.
- Replace `WeightRecord` and `queuePush` with `rfid::Event` and `rfid::EventQueue`, refusing when
  full. Gate the live publishes for `+`, `-` and `end` on `rfid::mustQueue`. `queueFlush` keeps
  its rules and its call sites. The weight topic stays at QoS 0.
- No signing, nonces, keys or replay defence (Alex, 2026-10-07: "this is a firewood buisiness not
  norad").
