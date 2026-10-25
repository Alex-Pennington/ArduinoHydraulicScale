# MQTT Contract — PLC #4 Tractor Scale

**Document:** 03 — MQTT Contract
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

Seven topics under one root. A consumer that reads only the weight topic gets every deliberate
record; a consumer that reads only telemetry gets a live stream and no record of what the operator
chose to keep. Engine data and fault codes come off the tractor's J1939 bus and never affect the
weight.

## Topics

| Topic | Direction | Retain | Quality | Published when |
|---|---|---|---|---|
| `plc4/jd5055e/weight` | Outbound | no | 0 | The operator records a weight; queued weights on reconnect and on each telemetry cycle while any remain |
| `plc4/jd5055e/telemetry` | Outbound | no | 0 | Every 5 s while connected |
| `plc4/jd5055e/status` | Outbound | yes | 1 | On connect, then every 60 s |
| `plc4/jd5055e/status` | Outbound | yes | 0 | Once, by the broker, if the machine drops without disconnecting |
| `plc4/jd5055e/engine` | Outbound | no | 0 | Every 15 s while connected |
| `plc4/jd5055e/dtc/<sa>` | Outbound | yes | 1 | When one engine control unit's fault state changes, and for every unit on reconnect |
| `plc4/jd5055e/cmd` | Inbound | no | 1 | Whenever the app sends a command |
| `plc4/jd5055e/ack` | Outbound | no | 0 | Once per command received with a usable `id`, including a repeat |

The root is one constant and every topic is built from it. A topic that would not fit its buffer is
refused rather than published to a truncated stub [1]. Every outbound payload except the ack
carries `id`, the machine's client identifier `plc4-jd5055e` [1]. The ack's `id` is the command's
`id`, §*Acknowledgement*.

## Weight

A deliberate record. The operator chose to keep it, by a command from the app or the console; this
is what the machine kept.

```json
{"w":1834,"age_ms":0,"backlog":false,"sensor":"ok","cal":"ok",
 "n":12,"total":19408,"id":"plc4-jd5055e"}
```

A replayed weight, held on the machine and published later:

```json
{"w":1834,"age_ms":42235,"backlog":true,"sensor":"ok","cal":"ok","id":"plc4-jd5055e"}
```

| Field | Meaning |
|---|---|
| `w` | Weight in pounds |
| `age_ms` | Milliseconds between capture and publication. Live readings carry 0 |
| `backlog` | True when this was held on the machine and published later |
| `sensor` | Loop state at the moment of capture, not at the moment of publication |
| `cal` | `ok`, or `DEFAULT` when the machine was uncalibrated at capture |
| `n` | Entries in the tally after this one. Live records only |
| `total` | Running total of the tally after this one. Live records only |

> **The running total arrives with the record that moved it.** A consumer never has to add the
> weights up itself, and never races a message it has not seen yet. The status topic carries the
> same figure, but only once a minute.

> **A replayed weight carries no `n` and no `total`.** The figures would be today's, standing
> beside a weight captured before the link dropped — a true number in a position that makes it
> read as the total at the time of capture.

> **Treat `backlog` and `age_ms` as part of the reading.** A buffered weight published bare is
> worse than not buffering at all, because the consumer cannot tell it from a fresh one. A record
> captured during a fault carries the fault state it was captured under, so a later publication
> can never claim the sensor was healthy.

The machine holds up to 200 weights in RAM while the broker is out of reach. A 201st drops the
oldest and counts it in `dropped`. The queue does not survive a reset. On replay the weights go out
back to back with no pacing, oldest first, and replay stops at the first failed publish.

## Telemetry

The live stream. Nothing here was chosen by the operator.

```json
{"w":1834,"psi":2850,"mA":19.21,"counts":5231,"sensor":"ok",
 "stale":false,"psi_fault":null,"queued":0,"id":"plc4-jd5055e"}
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
 "shunt_mv":-12.710,"ina_err":0,"build":"...","reset":"...","id":"plc4-jd5055e"}
```

| Field | Meaning |
|---|---|
| `up` | True while running. The will publishes false |
| `cal` | `ok`, or `DEFAULT` when no stored calibration was accepted |
| `sensor`, `stale` | Loop state, and whether the reading is held rather than current |
| `tare`, `cf` | The stored zero in converter counts, and the calibration factor |
| `n`, `total` | Entries in the tally, and their running total |
| `q`, `dropped` | Weights waiting to publish, weights lost from a full queue |
| `uptime_s`, `rssi`, `reset` | Seconds running, radio strength, why the board last restarted |
| `build` | The source commit this firmware was compiled from |
| `overrange`, `overrange_max`, `underrange` | Readings at or past full scale, the highest such reading in counts, readings below the live zero |
| `i2c_err` | Converter read failures since the last reset [2] |
| `vbus`, `shunt_mv` | The voltage monitor's bus volts and shunt millivolts, or null when unread |
| `ina_err` | Voltage monitor read failures since the last reset [3] |

> **`build` identifies the source, not the deployment.** It names the commit the image was
> compiled from, never the commit the repository stood at when the board was flashed. A trailing
> `+dirty` means the image contains work that is in no commit, so the identifier beside it does
> not fully describe it. `UNKNOWN` means provenance was never captured — deliberately not shaped
> like an identifier, so it cannot be mistaken for one.

`reset` names the cause flags the processor recorded, then the raw registers: `WATCHDOG` after a
watchdog reset, `software` after an upload. A power cycle and the reset button both read
`none-flagged`, so the firmware cannot tell them apart.

The last will is armed before the connection, so the broker publishes it if the machine drops
without disconnecting cleanly. It lands on the same topic and is retained like the cyclic status,
but at the lower quality of service: a subscriber sees both arrive on `status`, and the one that
says the machine is down was sent by the broker rather than by the machine.

```json
{"up":false,"id":"plc4-jd5055e"}
```

## Engine

The tractor's engine values, read off the J1939 bus. Published every 15 s while connected.

```json
{"rpm":1402,"load_pct":31,"torque_pct":24,"coolant_c":82.0,"oil_temp_c":88.4,
 "oil_kpa":320,"fuel_lph":6.35,"keysw_v":13.85,"hours":156.35,"fuel_level_pct":35.6,
 "hyd_oil_c":41,"rpm_min":1398,"rpm_max":1410,"rpm_avg":1403,...,
 "frames":6975,"unknown":4120,"ring_full":0,"ring_high":9,"can_err":0,"can_last":0,
 "drains":3000,"can_up":true,"filt":false,"id":"plc4-jd5055e"}
```

| Key | Unit | Source on the bus |
|---|---|---|
| `rpm` | rpm | EEC1, PGN 61444 [5] |
| `torque_pct` | % actual engine torque | EEC1, SPN 513 [5] |
| `load_pct` | % load at current speed | EEC2, PGN 61443, SPN 92 [5] |
| `coolant_c` | °C | ET1, PGN 65262 [5] |
| `oil_temp_c` | °C | ET1, PGN 65262 [5] |
| `oil_kpa` | kPa | EFLP1, PGN 65263 [5] |
| `fuel_lph` | litres per hour | LFE, PGN 65266, SPN 183 [5] |
| `batt_v` | V | VEP1, PGN 65271, SPN 168 [5] |
| `keysw_v` | V, keyswitch battery potential | VEP1, PGN 65271, SPN 158 [5] |
| `hours` | engine hours | PGN 65253, SPN 247 [5] |
| `fuel_level_pct` | % | DD, PGN 65276, SPN 96 [5] |
| `hyd_oil_c` | °C, hydraulic oil | VF, PGN 65128, SPN 1638 [5] |
| `rpm_min`, `rpm_max`, `rpm_avg` | rpm | Every valid `rpm` sample since the last message |
| `load_pct_min`, `_max`, `_avg` | % | Every valid `load_pct` sample since the last message |
| `torque_pct_min`, `_max`, `_avg` | % | Every valid `torque_pct` sample since the last message |

> **An absent key means no value, not zero.** A value is omitted when the bus has not carried it
> in the last 5 s, and a window's three keys are omitted when the window saw no valid sample. The
> J1939 not-available and error codes are never published as values. This is the opposite of
> telemetry's null convention: write the consumer to the omission rule.

`hours` is sent only in answer to another unit's request, so it stays in the message for 90 s
after the last answer rather than 5 s. The board never requests it.

The windows are taken every 15 s whether or not the broker is reachable, so a window never spans
an outage; a window that falls in one is discarded.

The CAN counters are always present:

| Key | Meaning |
|---|---|
| `frames` | Frames taken off the receive ring. Stops moving when the bus is quiet, as with the key off |
| `unknown` | Frames on groups this firmware does not decode |
| `ring_full`, `ring_high` | Drains that found the 32-frame ring full, so frames were lost; the deepest the ring has been |
| `can_err`, `can_last` | Driver error events, and the last error code |
| `drains` | Times the drain has run. Climbing with `frames` at 0 means nothing is arriving at the board's CAN receive pin, `01` §*Header pins* |
| `can_up` | False when the CAN controller did not open |
| `filt` | True when the hardware acceptance filter is in force |

## Fault codes

Active diagnostic trouble codes, one retained topic per engine control unit, keyed by its J1939
source address in decimal. The retained message is that unit's current fault state.

```json
{"sa":3,"n":0,"mil":0,"rsl":0,"awl":0,"pl":1,"dtc":[],"trunc":false,"id":"plc4-jd5055e"}
```

| Field | Meaning |
|---|---|
| `sa` | Source address of the unit reporting |
| `n` | Codes the board kept for this unit |
| `mil`, `rsl`, `awl`, `pl` | Malfunction, red stop, amber warning and protect lamp status, each 0 to 3 |
| `dtc` | One object per code: `spn` suspect parameter number, `fmi` failure mode identifier, `oc` occurrence count [1] |
| `trunc` | True when the unit sent more codes than the board keeps (12) or than fit the message |

> **A lamp can be on with an empty `dtc` list.** A unit that raises a lamp without naming a code
> sends a not-available record, and the board drops it rather than publishing a code that does not
> exist. Read the lamp fields as well as the list.

A message goes out when a unit's lamps or codes change, at most one per loop pass, 2 s apart, and
30 s apart after a failed publish. On every reconnect each unit's state is published again. The
board tracks up to six units; a seventh is ignored.

## Command

The board's remote write path. A command runs the same action the console runs, reaches the board
over MQTT instead of a wire, and is answered on `ack`.

```json
{"id":"7f3a9c21","verb":"+"}
{"id":"7f3a9c22","verb":"sm","arg":"3000"}
```

| Field | Meaning |
|---|---|
| `id` | Opaque token the sender generates, up to 40 letters, digits and `- _ . : +`. Echoed on `ack` |
| `verb` | The action to run, the same string the console accepts, run through the same handler |
| `arg` | Optional. Carries the argument `c`, `w` and `sm` take |

| | |
|---|---|
| Allowed | `tare` `+` `-` `b` `kv` `c` `w` `sm` |
| Refused | `hang`, which stops the board to prove the watchdog |
| Refused | `read`, `net`, `sum`, whose data `telemetry` and `status` already carry |
| Refused | Every other verb, including the console diagnostics `debug`, `i2c`, `pwr`, `can` and `canf` |

> **Never retained.** A retained command would re-run on every reconnect, re-taring or clearing the
> tally each time the board reassociated. A retained command is dropped, counted, and never
> executed or acknowledged.

> **A repeat `id` is acked but not re-executed.** The board remembers the last eight command ids
> and replays the stored verdict for a repeat.

No ack is sent for a command with no usable `id`, a retained command, a command whose payload had
not fully arrived when it was read, a command that arrived before a reconnect, or one that arrived
while four were already waiting [1]. Resend it with the same
`id`.

## Acknowledgement

Apart from the cases above, every command gets exactly one reply, published on `ack`.

```json
{"id":"7f3a9c21","verb":"+","ok":true,"detail":"recorded","n":12,"total":19408}
{"id":"7f3a9c22","verb":"tare","ok":false,"detail":"UNDER ZERO"}
{"id":"7f3a9c21","verb":"+","ok":true,"detail":"recorded","dup":true}
```

| Field | Meaning |
|---|---|
| `id` | Echoes the command's `id` |
| `verb` | Echoes the command's `verb`, or empty when the verb was unusable |
| `ok` | True when the action ran |
| `detail` | The result, or the reason for a refusal |
| `n`, `total` | Present on a successful `+` that is not a repeat: the tally entry count and running total, matching `weight` |
| `dup` | True when this ack replays a stored verdict rather than re-executing |

| `detail` | When |
|---|---|
| `recorded`, `removed`, `tared`, `cleared`, `calibrated`, `set` | The action ran |
| A sensor state, or `NO ADC` | `tare`, `+` or `kv` refused because the loop is not `ok` or the converter has stopped answering |
| `TALLY FULL`, `TALLY EMPTY` | `+` with 50 entries held, `-` with none |
| `at or below tare` | `kv` with no load above the tare |
| `c REJECTED`, `w REJECTED`, `sm REJECTED` | The argument is missing, not a number, or out of range |
| `CONSOLE ONLY` | A `c` whose argument would make it the console-only `can` or `canf` |
| `NOT ALLOWED` | A verb outside the allowed list |
| `BAD COMMAND` | No verb, or a verb with characters outside the allowed set |

> **A refusal because of the loop names the state the console prints.** The app and the console
> never disagree about which state stopped a tare, a record or a calibration. `NOT ALLOWED` and
> `BAD COMMAND` exist only on `ack`.

> **A duplicate replays the stored verdict.** A repeat of a refused command reports `ok:false`
> again; it can never claim a success that never happened.

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
| `BAD STATE` | no | A corrupted queued record. Never produced by a live reading |

Keep readings in every state that carries a real measurement [4]. The machine acts on a tare, a
record or a calibration only in state `ok`, `06` §*When an action is refused*.

## Staleness is a separate question

`stale` is orthogonal to `sensor`, and both must be read.

> **`{"sensor":"ok","stale":true}` is a normal combination.** It means the loop was last seen
> healthy and this reading is not current — the converter has stopped answering. A consumer
> reading only `sensor` banks a frozen weight as live.

The retained status topic is where this matters most: a subscriber connecting after the converter
has failed receives that state on its first delivery, describing a fault that began before it was
listening. Do not infer staleness from `i2c_err`, which counts every failure since the last reset.

## See also

`01C` §*Bus channels* for the converter and the bus whose failure sets `stale`.
`01` §*CAN* for the bus the engine and fault-code topics are read from.

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, lines 260–267, 388–410, 640–660, 1030–1066 and 1319–1344, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[2] A. Pennington, *greybeard-plc4-tractor-scale*, `include/mcp3421.h`, lines 25–26, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[3] A. Pennington, *greybeard-plc4-tractor-scale*, `include/ina219.h`, lines 41–42, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[4] A. Pennington, *greybeard-plc4-tractor-scale*, `include/loop_scaling.h`, lines 71–77, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[5] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_decode.h`, lines 31–40 and 338–420, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
