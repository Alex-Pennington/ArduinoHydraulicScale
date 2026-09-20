# MQTT Contract — PLC #4 Tractor Scale
## Cabinet Reference Card

PHOENIX NEST LLC · PLC #4
**Document:** 03 — MQTT Contract
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

Five topics under one root. A consumer that reads only the weight topic gets every deliberate
record; a consumer that reads only telemetry gets a live stream and no record of what the operator
chose to keep.

## Topics

| Topic | Direction | Retain | Quality | Published when |
|---|---|---|---|---|
| `plc4/jd5055e/weight` | Outbound | no | 0 | The operator records a weight, and again for each queued weight on reconnect |
| `plc4/jd5055e/telemetry` | Outbound | no | 0 | On a repeating cycle while connected |
| `plc4/jd5055e/status` | Outbound | yes | 1 | On a slower cycle, by the machine |
| `plc4/jd5055e/status` | Outbound | yes | 0 | Once, by the broker, if the machine drops without disconnecting |
| `plc4/jd5055e/cmd` | Inbound | no | 1 | Whenever the app or the console sends a command |
| `plc4/jd5055e/ack` | Outbound | no | 0 | Once per received command, including a repeat |

The root is one constant and every topic is built from it. A topic that would not fit its buffer is
refused rather than published to a truncated stub.

## Weight

A deliberate record. The operator chose to keep it, by a command from the app or the console; this
is what the machine kept.

```json
{"w":1834,"age_ms":0,"backlog":false,"sensor":"ok","cal":"ok",
 "n":12,"total":19408,"id":"..."}
```

| Field | Meaning |
|---|---|
| `w` | Weight in pounds |
| `age_ms` | Milliseconds between capture and publication. Live readings carry 0 |
| `backlog` | True when this was held on the machine and published later |
| `sensor` | Loop state at the moment of capture, not at the moment of publication |
| `cal` | `ok`, or `DEFAULT` when the machine was uncalibrated at capture |
| `n` | Entries in the tally after this one |
| `total` | Running total of the tally after this one |

> **The running total arrives with the record that moved it.** A consumer never has to add the
> weights up itself, and never races a message it has not seen yet. The status topic carries the
> same figure, but only once a minute.

> **A replayed weight carries no total.** The figure would be today's, standing beside a weight
> captured before the link dropped — a true number in a position that makes it read as the total
> at the time of capture.

> **Treat `backlog` and `age_ms` as part of the reading.** A buffered weight published bare is
> worse than not buffering at all, because the consumer cannot tell it from a fresh one. A record
> captured during a fault carries the fault state it was captured under, so a later publication
> can never claim the sensor was healthy.

## Telemetry

The live stream. Nothing here was chosen by the operator.

```json
{"w":1834,"psi":2850,"mA":19.21,"counts":5231,"sensor":"ok",
 "stale":false,"psi_fault":null,"queued":0,"id":"..."}
```

| Field | Meaning |
|---|---|
| `w` | Weight in pounds, or null when the reading cannot be trusted |
| `psi` | Pressure, or null when the reading cannot be trusted |
| `mA` | Loop current, the unit a meter at the machine reads |
| `counts` | Raw converter output |
| `sensor` | Loop state |
| `stale` | True when the reading is held rather than current |
| `psi_fault` | `low`, `high`, or null — the direction of a loop fault |
| `queued` | Weights waiting to be published |

`w` and `psi` both publish null and never zero when the loop is not measuring. A real zero and a
dead transmitter are different facts and are reported differently at every layer. The two fields
carry the same convention so a consumer cannot read one as live while the other declines.

Null here means the loop is not measuring or the converter has stopped answering. It does not
mean over-range: a reading below the live zero or at full scale is a real measurement and both
still publish a number.

## Status

The machine's standing answer to how it is. Retained, so a subscriber that connects later receives
the last one published.

```json
{"up":true,"cal":"ok","sensor":"ok","stale":false,"tare":412,"cf":0.51230,
 "n":12,"total":19408,"q":0,"dropped":0,"uptime_s":8130,"rssi":-58,"overrange":0,
 "overrange_max":0,"underrange":0,"i2c_err":0,"vbus":13.120,
 "shunt_mv":-12.710,"ina_err":0,"build":"d57002a1","reset":"...","id":"..."}
```

| Field | Meaning |
|---|---|
| `up` | True while running. The will publishes false |
| `cal` | `ok`, or `DEFAULT` when no stored calibration was accepted |
| `tare`, `cf` | The stored zero and the calibration factor |
| `n`, `total` | Entries in the tally, and their running total |
| `q`, `dropped` | Weights waiting to publish, weights lost |
| `uptime_s`, `rssi`, `reset` | Seconds running, radio strength, why the board last restarted |
| `build` | The source commit this firmware was compiled from |
| `overrange`, `overrange_max`, `underrange` | Counts of readings past each end of the band |
| `i2c_err`, `ina_err` | Sensor-bus failures since power-on. Neither resets |
| `vbus`, `shunt_mv` | Supply volts and shunt millivolts, or null when unread |

> **`build` identifies the source, not the deployment.** It names the commit the image was
> compiled from, never the commit the repository stood at when the board was flashed. A trailing
> `+dirty` means the image contains work that is in no commit, so the identifier beside it does
> not fully describe it. `UNKNOWN` means provenance was never captured — deliberately not shaped
> like an identifier, so it cannot be mistaken for one.

The last will is armed before the connection, so the broker publishes it if the machine drops
without disconnecting cleanly. It lands on the same topic and is retained like the cyclic status,
but at the lower quality of service: a subscriber sees both arrive on `status`, and the one that
says the machine is down was sent by the broker rather than by the machine.

```json
{"up":false,"id":"..."}
```

## Command

The board's remote write path. A command runs the same action the console runs, reaches the board
over MQTT instead of a wire, and is answered on `ack`.

```json
{"id":"7f3a9c21","verb":"+"}
{"id":"7f3a9c22","verb":"c","arg":"3000"}
```

| Field | Meaning |
|---|---|
| `id` | Opaque token the sender generates. Echoed on `ack`, and the key the dedup ring matches a repeat against |
| `verb` | The action to run, the same string the console accepts, run through the same handler |
| `arg` | Optional. Carries the argument `c`, `w` and `sm` take |

| | |
|---|---|
| Allowed | `tare` `+` `-` `b` `kv` `c` `w` `sm` |
| Refused, as a safety measure | `hang`. It deliberately stops the board to prove the watchdog; over the network that is a denial of service against a machine holding a load |
| Refused, as redundant | `read`, `net`, `sum`. The app already has that data from `telemetry` and `status` |

> **Never retained.** A retained command would re-run on every reconnect, re-taring or clearing the
> tally each time the board reassociated. A retained command is dropped, counted, and never
> executed or acknowledged.

> **A repeat `id` is acked but not re-executed.** The board keeps a ring of recent command ids and
> matches a repeat against it.

## Acknowledgement

Every command received gets exactly one reply, published on `ack`.

```json
{"id":"7f3a9c21","verb":"+","ok":true,"detail":"recorded","n":12,"total":19408}
{"id":"7f3a9c22","verb":"tare","ok":false,"detail":"UNDER ZERO"}
{"id":"7f3a9c21","verb":"+","ok":true,"detail":"recorded","dup":true}
```

| Field | Meaning |
|---|---|
| `id` | Echoes the command's `id` |
| `verb` | Echoes the command's `verb` |
| `ok` | True when the action ran |
| `detail` | The firmware's own state string naming the result or the refusal, never an invented one |
| `n`, `total` | Present on a successful `+`: the tally entry count and running total, matching `weight` |
| `dup` | True when this ack replays a stored verdict rather than re-executing |

> **A refusal detail is the firmware's own state string, never an invented one**, so the app and
> the LCD can never disagree about why something was refused.

> **A duplicate replays the stored verdict.** A repeat of a refused command reports `ok:false`
> again; it can never claim a success that never happened.

> **QoS 0 is deliberate.** A QoS 1 ack spins waiting for a PUBACK, which would stall the weighing
> loop and age the reading to stale. A lost ack is cheap: the dedup ring makes retrying safe.

## Sensor states

Six values. Three carry a real measurement and three do not, and that split is not the same as
the split between acting and not acting.

| Value | Real measurement | Meaning |
|---|---|---|
| `ok` | yes | Measuring, within the band |
| `UNDER ZERO` | yes | Below the live zero, still measuring |
| `SATURATED` | yes | At or past full scale, still measuring |
| `NO SENSOR` | no | Loop below the dead threshold — cut wire, no supply, transmitter removed |
| `XMTR FAULT` | no | At or above the fault threshold |
| `BAD STATE` | no | A corrupted record. Never produced by a live reading |

> **Do not discard the two over-range states.** Both carry a real reading, and throwing them away
> loses the excursions this front end was fitted to capture — the circuit goes past full scale in
> normal service. A consumer that treats everything but `ok` as invalid is not failing safe, it is
> dropping data.

The machine keeps a second and narrower rule for itself: it refuses a tare, a record or a
calibration unless the state is exactly `ok`, because a reading at the end of its range is no
basis for setting a zero. That rule is about permitting an action. The table above is about
trusting a number. They are different questions and collapsing them loses data one way or refuses
valid work the other.

## Staleness is a separate question

`stale` is orthogonal to `sensor`, and both must be read.

> **`{"sensor":"ok","stale":true}` is a normal combination.** It means the loop was last seen
> healthy and this reading is not current — the converter has stopped answering. A consumer
> reading only `sensor` banks a frozen weight as live.

The retained status topic is where this matters most: a subscriber connecting after the converter
has failed receives that state on its first delivery, describing a fault that began before it was
listening. Do not infer staleness from `i2c_err`, which counts from power-on and never clears.

## See also

`01` §*Bus channels* for the converter and the bus whose failure sets `stale`.
