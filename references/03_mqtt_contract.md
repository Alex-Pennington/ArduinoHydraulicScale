# MQTT Contract — PLC #4 Tractor Scale
## Cabinet Reference Card

PHOENIX NEST LLC · PLC #4
**Document:** 03 — MQTT Contract
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

Three topics under one root. A consumer that reads only the weight topic gets every deliberate
record; a consumer that reads only telemetry gets a live stream and no record of what the operator
chose to keep.

## Topics

| Topic | Retain | Quality | Published when |
|---|---|---|---|
| `plc4/jd5055e/weight` | no | 0 | The operator records a weight, and again for each queued weight on reconnect |
| `plc4/jd5055e/telemetry` | no | 0 | On a repeating cycle while connected |
| `plc4/jd5055e/status` | yes | 1 | On a slower cycle, by the machine |
| `plc4/jd5055e/status` | yes | 0 | Once, by the broker, if the machine drops without disconnecting |

The root is one constant and every topic is built from it. A topic that would not fit its buffer is
refused rather than published to a truncated stub.

## Weight

A deliberate record. The operator pressed the key; this is what the machine kept.

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
| `w` | Weight in pounds |
| `psi` | Pressure, or null when the reading cannot be trusted |
| `mA` | Loop current, the unit a meter at the machine reads |
| `counts` | Raw converter output |
| `sensor` | Loop state |
| `stale` | True when the reading is held rather than current |
| `psi_fault` | `low`, `high`, or null — the direction of a loop fault |
| `queued` | Weights waiting to be published |

`psi` publishes null and never zero when the loop is not measuring. A real zero pressure and a
dead transmitter are different facts and are reported differently at every layer.

## Status

The machine's standing answer to how it is. Retained, so a subscriber that connects later receives
the last one published.

```json
{"up":true,"cal":"ok","sensor":"ok","stale":false,"tare":412,"cf":0.51230,
 "n":12,"total":19408,"q":0,"dropped":0,"uptime_s":8130,"rssi":-58,"overrange":0,
 "overrange_max":0,"underrange":0,"i2c_err":0,"vbus":13.120,
 "shunt_mv":-12.710,"ina_err":0,"reset":"...","id":"..."}
```

| Field | Meaning |
|---|---|
| `up` | True while running. The will publishes false |
| `cal` | `ok`, or `DEFAULT` when no stored calibration was accepted |
| `tare`, `cf` | The stored zero and the calibration factor |
| `n`, `total` | Entries in the tally, and their running total |
| `q`, `dropped` | Weights waiting to publish, weights lost |
| `uptime_s`, `rssi`, `reset` | Seconds running, radio strength, why the board last restarted |
| `overrange`, `overrange_max`, `underrange` | Counts of readings past each end of the band |
| `i2c_err`, `ina_err` | Sensor-bus failures since power-on. Neither resets |
| `vbus`, `shunt_mv` | Supply volts and shunt millivolts, or null when unread |

The last will is armed before the connection, so the broker publishes it if the machine drops
without disconnecting cleanly. It lands on the same topic and is retained like the cyclic status,
but at the lower quality of service: a subscriber sees both arrive on `status`, and the one that
says the machine is down was sent by the broker rather than by the machine.

```json
{"up":false,"id":"..."}
```

## Sensor states

Six values, and a consumer treating anything other than `ok` as invalid fails safe on all of them.

| Value | Meaning |
|---|---|
| `ok` | Measuring, within the band |
| `NO SENSOR` | Loop below the dead threshold — cut wire, no supply, transmitter removed |
| `UNDER ZERO` | Below the live zero but still measuring |
| `SATURATED` | At or past full scale, still measuring |
| `XMTR FAULT` | At or above the fault threshold |
| `BAD STATE` | A corrupted record. Never produced by a live reading |

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
