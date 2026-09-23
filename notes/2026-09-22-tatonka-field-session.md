# 2026-09-22 — tatonka field session at the JD 5055E

Closeout record for the onsite session, written by the Claude session on **tatonka** (the Mint
live-USB field laptop). Alex was at the machine throughout. The PLC #4 coordinator
(`ymir-plc4-coordinator-0922-0001`) built the firmware on Saga. The consumer coordinator
(`ymir-plc-consumer-coordinator-0914-0005`) did the ingest.

Each item says whether it was **measured** here or **reported** by another session.

**Times:** tatonka timestamps are UTC (the laptop clock). The consumer reports in EDT (UTC−4). For example, the −600 `+` at 20:33:11 UTC is 16:33:11 EDT.

## Where the data is

`saga:~/plc4-logs/`

- `census-2026-09-22/`: RH-02 candump logs, `RESULTS.md` (census numbers and corrections),
  and `j1939_census_decode.py` (the host decoder used for the signal proposal).
- `field-2026-09-22/`: MQTT and USB-console captures of each firmware build on the board,
  plus this note.

The firmware images are still in `saga:~/plc4-firmware/<build>/`. They were not copied again
because production secrets are compiled in.

## 1. PGN census (RH-02, SocketCAN, listen-only, 250k) — measured

The full numbers are in `census-2026-09-22/RESULTS.md`. Headlines:

- **The bus runs at 250k.** 500k was skipped on Alex's decision, because 250k gave clean frames
  with static error counters.
- **Nodes:** 5 source addresses (0x00, 0x03, 0x06, 0x17, 0x47).
- **Traffic:** ~40 PGNs, ~60% proprietary.

| Phase | Sustained fps | Peak fps (100 ms bins) |
|---|---|---|
| key on, engine off | 459 | 550 |
| crank/start | 800 | 1590 |
| idle ~900 rpm | 465 | 560 |

- **Ring budget, 32 frames:** ~57 ms steady-state. 20 ms to ride through the crank burst.
- **Key-on vs engine-running:** both send the same PGN set. Only start-up one-shots differ
  (address claim, requests, TP, DM1).
- **Hydraulic pressure: none.** No standard J1939 PGN on this bus carries it. VF 65128 (SA 0x47)
  carries **hydraulic oil temperature** only (18–19 °C). The scale's transducer is the only
  hydraulic pressure available.
- **Invalid logs:**
  - `_174629` is empty (tractor off).
  - `_175648` and `_175843` are partial.
  - `_181316`, `_181633` and `_181818` are empty (wiring fault at the time).
  - None of these are census data.

## 2. Firmware on the board — builds flashed from tatonka, each at Alex's go

The toolchain on tatonka matches Saga: arduino-cli 1.5.0 (Saga's binary) and
arduino:renesas_uno 1.6.0. The port was `/dev/ttyACM0`. `mint` was added to `dialout`.

| Build | Source | Result |
|---|---|---|
| `19f0c022` | merge, feat/plc4-can-fold-in | Flashed 19:19. CAN received on the R4 WiFi D10/D13 for the first time (**issue #21 answered**, 19:40, after Alex fixed wiring). Freshness bug: rpm, load, torque and hours were dropped. |
| `e82bcb7d` | diagnostics | Pulled and verified. **Not flashed.** |
| `705bf3c9` | freshness fix | Pulled and verified. **Not flashed:** Alex held until all known bugs were fixed. |
| `72c6c3a4` | release candidate | Flashed 20:30. **All key-on and running acceptance passed** (below). |
| `699e3c6c` | 72c6c3a4 + top picks | Flashed ~20:47. **Running at close.** Engine-running acceptance **passed on stored rows** (REPORTED by consumer, see §2a). |

### 72c6c3a4 acceptance — measured over MQTT (`72c6c3a4-mqtt-*.txt`)

- **Key on, engine off:** rpm 0 present. load, torque, fuel_lph, coolant 46 °C, keysw 12.75 V and
  hours 598.70 all present.
- **Running at 20:33, 4 messages, every field in each:**
  - rpm 899–901, load 32–35 %, torque 30–32 %
  - oil 376–380 kPa, fuel 1.5–1.6 L/h, keysw 14.25–14.30 V, hours 598.70
  - ring_full 0, ring_high 9/32, can_err 0, drains +~3000 per 15 s
  - 462.5 fps (census idle 464.9)
- **DTC:** 3, 6, 23 and 71 all clear, lamps 0, trunc false.
- **Phone `+`:** acked ok.

### 699e3c6c — as of 20:53, measured

- **status:** build `699e3c6c`, cal ok, sensor ok, i2c_err 0, ina_err 0, n 2, total 717. The
  EEPROM survived.
- **engine:** frames 0 (key off), drains +3000 per message, can_err 0.
- **Still to verify with the engine running:**
  - fuel_level_pct ~35.6 and hyd_oil_c present
  - rpm/load/torque min ≤ avg ≤ max
  - every 72c6c3a4 field present

### 2a. 699e3c6c engine-running acceptance — REPORTED by the consumer coordinator, not measured on tatonka

The tractor ran on 699e3c6c after the consumer went live at 17:19 EDT. From farm1 `plc4_engine`,
engine running 17:25:20–17:32:05 EDT (21:25–21:32 UTC):
- 28 rows carry the window stats. 26 carry fuel_level_pct and hyd_oil_c.
- fuel_level_pct: 35.6 parked, 25.2–38.4 under load. It swings with load (slosh, most likely).
- hyd_oil_c: 21 → 26 °C.
- rpm: 900 at idle, 1704 peak. coolant 43 → 71 °C, oil 316–408 kPa, keysw 14.25–14.3 V.
- **ZERO `plc4/jd5055e/engine` rows in `ingest_unmapped`.**
- DM1: all four ECUs published live at key-on (17:25:20–26), all clear.
- The first key-on window read **torque_pct_min −125**. That is the J1939 offset floor, raw 0,
  sent before the ECU settled. It is worth a firmware guard: treat raw 0 at key-on as not valid.

The min ≤ avg ≤ max ordering was not independently checked by tatonka.

## 3. Open items

1. ~~699e3c6c running acceptance~~ **Passed on stored rows (§2a).** Still open from it: the
   torque_pct_min −125 key-on artefact, and fuel_level swinging under load.
2. **Negative tally entry.** At 20:33:11 a phone `+` recorded **w = −600** (scale read ~−600,
   167 psi, 4.9 mA). The board tally went **1317 → 717** (n 1 → 2).
   - The firmware accepts a negative weight as a load.
   - The consumer stored it as a weight capture only; no inventory move was ingested
     (reported by the consumer coordinator).
   - The correction has to be made **on the board**. Not yet done.
3. **Key-off capture never done.** Wanted: 2 min of the engine/dtc topics after key-off, to see
   whether the board stays powered. Indirect evidence: 699e3c6c kept publishing status with
   vbus 13.19 V while CAN was silent, key position not confirmed.
4. **PTO census.** Alex: "Yes, next time it's running." Procedure:
   - RH-02 listen-only at 250k, R120 switch OFF.
   - ~2 min with the PTO engaged and the tractor moving.
   - Note the observed PTO rpm and ground speed.
   - Put the logs and a RESULTS note in `saga:~/plc4-logs/`.

   This confirms the UNVERIFIED signals: rear PTO 65091 (SA 0x47), CCVS wheel speed, ETC2 gear,
   and 65129, 64892 and 64947.
5. **Rear PTO decode.** It was in the proposal's top picks but is not in 699e3c6c. Alex said
   "top picks only". Whether PTO is included is still open, pending the PTO census.
6. **SA 0x03 fault.** DTC **SPN 2825 FMI 7** was active with pl 1, seen in the retained
   `dtc/3` around 19:54 (`19f0c022-mqtt-5.txt`). It had cleared by 20:33.
7. **Transceiver D pin.** The transceiver's D pin is on D10, so the board ACKs frames on the
   tractor bus. `findings/2026-09-20-two-node-bench.md` recommends tying D to the transceiver's
   3.3 V for guaranteed silence. That is hardware, and it has not been done.
8. **`canf` filter trial.** Not run. It needs the USB console at the tractor.
9. **Saga checkout.** `~/greybeard-plc4-tractor-scale` main was 53 commits behind origin when this
   session started.
   *Checked by the PLC #4 coordinator on 2026-09-22 at wrap: `saga/main`, `origin/main` and the
   local `main` were all at the same commit, so nothing was behind by then.*

## 4. Session environment notes (tatonka)

- **SSH.** The live user is `mint`, not the `linux` that `tatonka/README.txt` says. The
  provisioned SSH key was in `/home/linux/.ssh`. It was copied to `~/.ssh`, so
  `ssh saga|farm1|farm2|hub` works as `rayven` over wg0.
- **Tools installed:** can-utils, mosquitto-clients, arduino-cli (`~/.local/bin`).
- **Broker.** `192.168.10.99` accepts anonymous connections on FOXTROTBRAVO.
- **Saga's network.** Saga is on 192.168.5.x and cannot reach the broker directly.
- **Dashboard.** http://192.168.5.241:3001/d/plc4-tractor-engine
