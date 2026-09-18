# PLC #4 side quest — J1939 CAN off the JD 5055E into the existing stack

Research and planning only. Nothing here was flashed, deployed or applied. Branch
`feat/plc4-j1939-can`, worktree `.claude/worktrees/can`.

**Verdict: GO on the UNO R4 WiFi, with two conditions.** The board already on the
tractor has the CAN peripheral free, the pins free, and 74% of its flash unused. The
conditions are a 5 V transceiver on D10/D13 and a decision about ACK on a live
agricultural bus. A dedicated ESP32 is not needed and would add a second thing to
power, mount and keep on the WiFi.

The two conditions are the real work. Everything below the fold is either verified on
this machine or flagged as a bench unknown.

## 1. What is already on the tractor

`src/ArduinoHydraulicScale.cpp`, 1553 lines, PlatformIO `board = uno_r4_wifi`.
Deployed 2026-09-02 from `plc4/mqtt-port` @ `83cb2c5`. It weighs a hydraulic
transducer and publishes three topics under `plc4/jd5055e/`.

What matters for CAN:

| | |
|---|---|
| Pins in use | D0/D1 (HC-05 serial), **D4–D8 (keypad: Right, Up, Down, Left, Select)** |
| Pins free | **D10 and D13 — the two the R4 WiFi needs for CAN** |
| I2C | Qwiic/Wire1 only, behind a TCA9548A at 0x70 (ch2 INA219, ch3 MCP3421, ch7 LCD2004) |
| Loop | 500 ms nominal, no `delay()` in the weighing path |
| Build | 91,232 B flash (35%), 13,472 B RAM (41%) |
| Watchdog | Present — WDT, 5 s, refreshed from an FspTimer ISR on a loop heartbeat |
| Network | Two-stage `netPoll()`: WiFi and broker never connected in the same pass, 30 s minimum retry |

I verified the pin question directly rather than taking it on report: the only
`pinMode()` calls in the firmware are `KEY_PINS[5] = {4,5,6,7,8}` at
`src/ArduinoHydraulicScale.cpp:787`. There is no SPI use and no `LED_BUILTIN` use, so
D10 and D13 are genuinely unclaimed.

**Correction to the brief's premise.** The brief says the R4's CAN is on D4 (TX) and
D5 (RX). That is the **UNO R4 Minima** mapping. On the **WiFi** it is D10 and D13:

- `variants/UNOWIFIR4/pins_arduino.h:155-156` — `PIN_CAN0_TX (10)`, `PIN_CAN0_RX (13)`
- `variants/MINIMA/pins_arduino.h:151-152` — `PIN_CAN0_TX (4)`, `PIN_CAN0_RX (5)`

This is load-bearing. On the D4/D5 premise the answer would have been "the keypad is in
the way, move the buttons or use a second board". On the correct pinout there is no
conflict at all. D13 is the board's LED pin and the classic SPI SCK pin, so anything
later wanting SPI collides — worth knowing, nothing today uses it.

Nothing in the repository mentions CAN, J1939, OBD or Service ADVISOR. This is new work.

## 2. The stack it must land in

There is no Telegraf and no Node-RED. Ingestion is a custom Python consumer.

- **Broker.** Consumer connects to `mqtt.firewood.ltd:1883`, plain TCP, username/password
  (`/opt/plc1-monitoring/.env`). The board's own source uses `192.168.10.99:1883`, and
  `BRIEF.md` records a third name, `farm1.internal (10.77.77.13)`, for Saga's vantage.
  Three names, probably one broker, and I did not confirm that — see unknowns.
- **Consumer.** `/opt/plc1-monitoring/consumer/plc1_consumer.py`, ~2400 lines, paho-mqtt
  + psycopg2, subscribed to `plc1/#` … `plc4/#` at QoS 1. `dispatch()` routes by **exact
  topic string**, so a new subtopic needs a handler — the wildcard subscription alone
  does nothing.
- **Postgres.** Database `plc1`. PLC #4 already owns three tables (`plc4_telemetry`,
  `plc4_weight`, `plc4_status`) from migration 026, extended by 037, 061 and 066.
  Migrations run to **066**; mine is proposed as 067.
- **Grafana.** `plc1-grafana` on port 3001, datasource uid `plc1-postgres`, dashboards
  provisioned from JSON files in `grafana/provisioning/dashboards/`.
- **Ritual.** `tests/run-ritual.sh` gates changes, and every test file must be listed in
  `docs/DESIGN-STANDARD.md` §4 or the ritual jams.

Topic and payload conventions, from the code, not from habit:

- `plc4/jd5055e/telemetry` — 5 s, QoS 0, **not** retained (an event)
- `plc4/jd5055e/status` — 60 s, QoS 1, **retained** (a state)
- Validity: PLC1/PLC3 **omit** value fields when invalid; PLC4 **always populates** and
  puts validity in sibling strings (`sensor`, `cal`). Two conventions live side by side.

The CAN work follows the **PLC1/PLC3 omission** convention, deliberately, and §7 says why.

## 3. Existing work — none

`gh repo list Alex-Pennington` matches only `greybeard-plc4-tractor-scale` for
can/tractor/j1939/deere. `gh search issues --owner Alex-Pennington "CAN bus J1939"`
returns nothing. The six open issues on the PLC4 repo (1, 3, 5, 16, 17, 18) are all
scale-side. Nothing to duplicate.

## 4. The R4 WiFi as a CAN node — verified against the installed core

Core `arduino:renesas_uno 1.6.0`, `arduino-cli 1.5.0`, both already installed. The
`Arduino_CAN` library ships with the core; no dependency to add.

**Good:**

- 250 kbit/s is a first-class enum value (`CanBitRate::BR_250k`), and `begin(uint32_t)`
  also derives arbitrary rates from the 24 MHz CAN clock — so an unexpected bus speed
  is a one-line change, not a redesign.
- 29-bit extended IDs are native: `CanMsg::CAN_EFF_FLAG`, `isExtendedId()`,
  `getExtendedId()`, and the RX mailboxes are configured `CAN_ID_MODE_EXTENDED` by default.
- Hardware filtering exists: `setFilterMask_Extended(mask)` plus
  `setFilterId_Extended(mailbox, id)`.

**Constraints, all of which shape the design:**

1. **Eight extended filter slots, sharing one mask.** `CAN_MAX_NO_EXTENDED_MAILBOXES = 8`,
   and `setFilterMask_Extended()` writes a single mask across both mailbox groups. We
   want ten PGNs (seven data + DM1 + the two transport PGNs). Either drop two, or use a
   coarser mask and filter the remainder in software. Filtering matters: it is the
   difference between handling every frame on the bus and handling ~50/s.
2. **Two stacked silent-loss stages, no counter on either.** The hardware mailboxes run
   `CAN_MESSAGE_MODE_OVERWRITE` (loses the **oldest**); `CanMsgRingbuffer::enqueue()`
   returns early when its 32-slot ring is full (loses the **newest**). Neither
   increments anything. A frame counter in firmware counts what arrived, never what was
   missed.
3. **32 frames of ring buffer.** At 250 kbit/s a full extended frame is ~0.5 ms, so a
   busy bus can fill that ring in well under 50 ms. The scale's 500 ms loop is far too
   slow to be the only drain point — CAN must be drained several times per pass, or
   filtered hard enough that it cannot fill.
4. **An "unreachable code" comment sits on the receive path.** In
   `libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp`, the `CAN_EVENT_RX_COMPLETE` case carries
   `// Currently driver don't support this. This is unreachable code for now.` The RX
   interrupt *is* registered (`IRQManager.cpp:886` wires `can_rx_isr`), so the comment
   is probably stale — but "probably" is not a plan. **This is the first bench test and
   it needs no tractor**: `CAN.enableInternalLoopback()`, send a frame, see whether
   `CAN.available()` ever becomes non-zero. If it does not, the R4 path is dead on
   arrival and the answer is an ESP32 after all.
5. **No listen-only through the library.** The RA4M1 supports it —
   `CAN_TEST_MODE_LISTEN = 3` in `r_can_api.h:102` — but `Arduino_CAN` exposes only the
   two loopback modes. Getting true silence means vendoring a patched copy of the
   library into `lib/` with a six-line `enableListenOnly()` mirroring
   `enableInternalLoopback()`. Cheap, and see §5 for why it might be worth it.

**Transceiver.** Use a **5 V** transceiver: TJA1050 or MCP2551 breakout. The R4 runs 5 V
logic, and a 3.3 V part like the SN65HVD230 drives RXD to 3.3 V, which is marginal
against a 5 V input threshold — it may work on the bench and fail hot. Wiring is four
wires: D10→TXD, D13→RXD, 5 V, GND, plus CANH/CANL to the tractor. **Do not fit a 120 Ω
termination resistor** — the tractor bus is already terminated at both ends and a third
resistor degrades it. Many cheap TJA1050 modules have one fitted; check and remove it.

**The Qwiic/I2C CAN adapter path is a no.** It is attractive because the board already
has a Qwiic trunk with a free mux channel — but an I2C-bridged CAN controller polls over
a 100 kHz bus that is already carrying the ADC, the INA219 and a 20×4 LCD through a mux.
J1939 at 250 kbit/s will outrun it, the losses land in the adapter's own buffer where
firmware cannot see them, and it shares a bus whose timing the weighing path depends on.
The native peripheral is free. Use it.

## 5. The tractor's bus

This is the weakest part of the report and I want to be plain about that. I found no
John Deere document, and the 5055E-specific evidence is from a university reverse
engineering project, not from Deere.

What the sources support:

- The tractor has a **round diagnostic connector, low and to the right of the driver**,
  exposing CAN high, CAN low and ground. The Tractor Hacking project connects to exactly
  this on a 5055E and names **pins C and D** as CAN high and low.
- The bus is **250 kbit/s** and uses **29-bit extended identifiers** — i.e. standard
  SAE J1939 framing, not a proprietary link layer.
- Deere uses **proprietary PGNs** heavily alongside the standard ones. Those are
  undocumented, but they are undocumented *contents* inside standard frames; they do not
  stop standard PGNs being decodable.

What the sources do **not** support, and which I am not going to assert:

- The exact connector part number and pin lettering for a 2021 5055E.
- That the diagnostic connector is on the same bus segment as the engine ECU. On many
  machines the diagnostic port, the implement bus and the powertrain bus are separate. If
  the connector sits on an implement bus, engine PGNs may simply never appear — which
  would be a "no data" result on a correctly built node, not a fault to debug.
- The typical bus load on this machine.

**On ACK, which is the second condition.** A normal CAN node acknowledges every frame it
receives — one dominant bit. That is participation, not writing, and on a bus with
several ECUs it is unremarkable. The failure mode worth naming: if our node is the only
listener for a talker, a non-ACKing configuration makes that ECU retransmit and go error
passive. So the choice is between *normal ACKing* (safe on a populated bus, the usual
answer for sniffing) and *true listen-only* (needs the vendored library patch from §4.5).
My recommendation is to fit the listen-only patch and use it for the first capture, then
decide. It costs an hour and it removes the whole question from the first time anyone
plugs into a running tractor.

## 6. J1939 decode table

Byte positions are **1-based** as J1939-71 numbers them; the sketches index from 0.
Multi-byte values are little-endian on the wire.

| PGN | Name | Parameter | SPN | Bytes | Resolution | Offset | Rate |
|---|---|---|---|---|---|---|---|
| 61444 | EEC1 | Engine speed | 190 | 4–5 | 0.125 rpm/bit | 0 | ~20 ms |
| 61444 | EEC1 | Actual engine percent torque | 513 | 3 | 1 %/bit | −125 | ~20 ms |
| **61443** | **EEC2** | **Engine percent load at current speed** | **92** | **3** | 1 %/bit | 0 | ~50 ms |
| 65262 | ET1 | Engine coolant temperature | 110 | 1 | 1 °C/bit | −40 | 1 s |
| 65262 | ET1 | Engine oil temperature | 175 | 3–4 | 0.03125 °C/bit | −273 | 1 s |
| 65263 | EFL/P1 | Engine oil pressure | 100 | 4 | 4 kPa/bit | 0 | 500 ms |
| 65253 | HOURS | Engine total hours of operation | 247 | 1–4 | 0.05 h/bit | 0 | **on request** |
| 65266 | LFE | Engine fuel rate | 183 | 1–2 | 0.05 L/h per bit | 0 | 100 ms |
| 65271 | VEP1 | Battery potential / power input 1 | 168 | 5–6 | 0.05 V/bit | 0 | 1 s |
| 65271 | VEP1 | Keyswitch battery potential | 158 | 7–8 | 0.05 V/bit | 0 | 1 s |
| 65226 | DM1 | Active DTCs | — | 1–2 lamps, then 4/DTC | — | — | 1 s |

**Two traps in that table.**

*Load is not in EEC1.* The brief asks for "EEC1 (61444: rpm, load)". SPN 92, engine
percent load at current speed, is in **EEC2 (61443) byte 3**, offset 0. EEC1 byte 3 is
*actual percent torque*, SPN 513, offset **−125**. Decoding one as the other yields a
number that looks like a percentage and is wrong by 125.

*Hours is probably request-only.* J1939-71 lists PGN 65253 as on-request for most
engines. If the sniffer never shows it, reading hours means **transmitting** a request
(PGN 59904) onto the tractor bus. That is a genuinely different act from listening and
should be decided on its own, not smuggled in as "one more PGN". The dashboard panel is
built to show "No data" honestly rather than pretend.

**Provenance.** Engine speed and oil pressure I confirmed against a public J1939 DBC
(`arthithadee/J1939-DBC-Database`): `EngineSpeed : 24|16@1+ (0.125,0) "rpm"` and
`EngineOilPressure : 24|8@1+ (4,0) "kPa"`. Byte positions for 65253, 65262, 65266 and
65271 are corroborated by two independent sources. **The scale and offset for coolant,
hours, fuel rate and battery came from J1939-71 as reproduced in secondary sources, not
from the Digital Annex itself** — they are the conventional values and I could not
confirm them against the standard. They are all checkable by plausibility on the first
capture: battery ~13.8 V running, coolant ~85 °C warm, oil ~350 kPa. The consumer
handler enforces exactly those bounds and logs a decode fault rather than storing
nonsense.

**DM1.** Bytes 1–2 are lamp status (two bits each: MIL, red stop, amber warning,
protect; 0 off, 1 on, 3 not available — **3 is not "on"**). Then 4 bytes per DTC: SPN
low, SPN mid, then a byte holding SPN bits 16–18 in its top three bits and FMI in its
low five, then a byte holding the conversion method in bit 7 and the occurrence count in
bits 0–6. More than one active DTC pushes DM1 past 8 bytes and it goes out over **BAM**:
TP.CM on PGN **60416** (control byte 0x20, total size, packet count, embedded PGN), then
TP.DT on PGN **60417**, 7 data bytes per packet, sequence numbers from 1. `can_mqtt.ino`
implements this for a single session.

## 7. Integration shape, and the one contract decision

**Topics** — following the existing split exactly:

- `plc4/jd5055e/engine` — 15 s, QoS 0, **not** retained. Numbers are events.
- `plc4/jd5055e/dtc` — on change, QoS 1, **retained**. Fault state is state, and a
  subscriber connecting later must see it.

**Validity: omit absent keys.** This is PLC1/PLC3's convention and *not* PLC4's, and the
difference is principled. PLC4's existing telemetry always populates `w`/`psi`/`counts`
and puts validity in sibling strings, because an ADC always returns a count — a reading
can be present and untrustworthy at once. J1939 is not like that: a parameter the engine
does not broadcast is absent, and one broadcast as `0xFF…` says "not available" in the
standard itself. So an absent key means absent, and the consumer must not log a missing
`rpm` as a defect the way it logs a missing `w`. Carrying the PLC4 reflex across would
make every 15 s row on a parked tractor raise a warning.

**Firmware shape.** The CAN reader becomes a module inside
`src/ArduinoHydraulicScale.cpp`, reusing the existing `netPoll()`, will and client id —
not a second MQTT client. Two clients means two `up` flags that can disagree. The drain
must be called more often than once per 500 ms loop pass (see §4.3).

## 8. Unknowns to verify on the bench

Ordered so that each one can kill the next cheaply. The first two need no tractor.

1. **Does `Arduino_CAN` receive at all on core 1.6.0?** Internal loopback, one frame,
   check `CAN.available()`. Settles §4.4. *No tractor, no transceiver.*
2. **Does the transceiver wiring work?** Two R4s, or one R4 and a USB-CAN dongle, at
   250 kbit/s on the bench. Confirms D10/D13 and the 5 V part. *No tractor.*
3. **Is the diagnostic connector's bus live, and at what rate?** `can_sniff` at 250k. No
   frames could mean wrong rate, wrong pins, or the wrong bus segment — try 500k and
   250k before concluding anything.
4. **Which PGNs actually appear, at what rate, and from which source addresses?** The
   census table in `can_sniff` answers this directly. Specifically: does 65253 (hours)
   ever appear unrequested?
5. **What is the real frame rate?** Decides whether hardware filtering is mandatory or
   merely prudent, and whether the 32-frame ring can be drained from the scale's loop.
6. **Do the scalings produce plausible numbers?** Battery ~13.8 V, coolant ~85 °C, oil
   ~350 kPa, rpm matching the tachometer.
7. **Is `mqtt.firewood.ltd` the same broker the board reaches as `192.168.10.99`?** Three
   names appear across the estate for what is probably one broker. Cheap to confirm, and
   embarrassing to get wrong.
8. **Connector pinout on this specific machine.** Meter it before fabricating a lead.
9. **Power and grounding.** The board's supply and ground reference relative to the
   tractor's is undocumented (`BRIEF.md` open question 8 — switched or maintained power
   is unresolved). A ground offset between the board and the bus is how transceivers die.

## 9. Scope estimate

Rough, and assumes the answers come out as expected. Each phase can stop the next.

| Phase | Work | Estimate |
|---|---|---|
| Bench proof | Loopback RX test, two-node bench link, confirm the library works | 0.5 day |
| Hardware | Transceiver breakout, remove its termination, fabricate the connector lead, mount and fuse | 1 day + parts |
| Bus discovery | `can_sniff` on the tractor, capture at two bitrates, PGN census, decide filters | 0.5 day |
| Decode + publish firmware | Finish `can_mqtt`, fold into the scale firmware, filters, drain cadence, flash | 2–3 days |
| Ingestion | Migration 067, two consumer handlers, handler tests, register in DESIGN-STANDARD §4, run the ritual | 1 day |
| Dashboard + alerts | Provision the dashboard, alert on DM1 red-stop and on CAN liveness | 0.5 day |

**≈ 5.5–6.5 days**, excluding parts lead time. The firmware phase carries the risk: it
edits deployed, gated firmware that currently weighs loads for real, on a machine that
drives out of radio range.

Parts: one 5 V CAN transceiver breakout (~£3), a mating diagnostic connector or a
flying-lead adapter, fuse and wire. Under £30 unless the connector is a Deere-specific
part.

## 10. Deliverables in this branch

| Path | What | Verified how |
|---|---|---|
| `sketches/can_sniff/can_sniff.ino` | Bus discovery: prints every frame, keeps a PGN census with rates and source addresses | Compiles: 57,296 B flash (21%), 9,384 B RAM (28%) |
| `sketches/can_mqtt/can_mqtt.ino` | Decode skeleton: the PGNs above, BAM reassembly for DM1, 15 s summary, DTC on change | Compiles: 69,404 B flash (26%), 9,244 B RAM (28%) |
| `ingest/067_add_plc4_engine.sql` | Proposed migration: `plc4_engine`, `plc4_dtc_snapshot`, `plc4_dtc_code`, `v_plc4_dtc_active` | Applied clean to a throwaway PG16 cluster; the view's no-fault and two-fault behaviour tested |
| `ingest/consumer_plc4_engine.py` | Proposed consumer handlers with plausibility bounds and lamp handling | Driven through 7 cases with a stub cursor; all behaved as documented |
| `ingest/plc4_engine_dashboard.json` | Proposed Grafana dashboard, 8 panels | JSON parses; every panel's SQL `EXPLAIN`s against the proposed schema |

Neither sketch has seen a tractor, and no scaling constant in either has been confirmed
against this machine. The throwaway Postgres cluster was created under the job temp
directory and is not the production database.

Compile them yourself with:

```
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/can_sniff
cp arduino_secrets.h.example sketches/can_mqtt/arduino_secrets.h   # gitignored
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi sketches/can_mqtt
```

## 11. Proposed GitHub issues — not filed

Per the standing rule, dev follow-ups are GitHub issues and I do not open them without
your word. These are the ones I would file on `Alex-Pennington/greybeard-plc4-tractor-scale`:

1. **Prove `Arduino_CAN` receives on core 1.6.0 before anything else is built** — internal
   loopback test; the RX-complete handler carries an "unreachable code" comment. Blocks
   every other CAN issue.
2. **Vendor a patched `Arduino_CAN` exposing `CAN_TEST_MODE_LISTEN`** — six lines; makes
   the first live capture provably silent on the tractor bus.
3. **Fit and bench-test a 5 V CAN transceiver on D10/D13** — including removing the
   breakout's termination resistor.
4. **Capture a PGN census from the 5055E diagnostic connector** — `can_sniff` at 250k and
   500k; record which PGNs appear, their rates, source addresses, and whether 65253
   appears unrequested.
5. **Decide whether to transmit PGN requests for engine hours** — a decision about writing
   to the tractor bus, not a coding task.
6. **Fold the CAN reader into the scale firmware** — shared `netPoll()`, hardware filters,
   drain cadence faster than the 500 ms loop.
7. **Hand migration 067 and the two consumer handlers to the consumer coordinator** —
   including a handler test registered in `docs/DESIGN-STANDARD.md` §4 so the ritual does
   not jam.
8. **Alert on DM1 red-stop lamp and on CAN liveness going to zero with the key on.**
9. **Correct `BRIEF.md`: the "no watchdog" characterisation is out of date** — the firmware
   now runs a 5 s WDT refreshed from an FspTimer ISR. Found while surveying; unrelated to
   CAN, not fixed here.
10. **Confirm `mqtt.firewood.ltd`, `192.168.10.99` and `farm1.internal` are one broker** —
    and record the answer where both the board and the consumer can be read against it.

## Sources

- [Tractor Hacking — user manual](https://tractorhacking.github.io/usage/) and
  [connecting to a live system](https://tractorhacking.github.io/documentation/ConnectingToALiveSystem.html)
- [Monitoring John Deere J1939 networks — JCOM1939](https://jcom1939.com/monitoring-john-deere-j1939-networks-technical-challenges-and-considerations/)
- [J1939 connector pinout — Simma Software](https://simmasoftware.com/j1939-connector-pinout/)
- [J1939-73 DM1 diagnostics — CSS Electronics](https://www.csselectronics.com/pages/j1939-73-dm1-diagnostic-message-dtc)
- [J1939 DBC database (engine speed, oil pressure scaling)](https://github.com/arthithadee/J1939-DBC-Database)
- [J1939 PGN parsing reference (byte positions)](https://gist.github.com/philbegg/462dd78a0aa78df7073dd1c0b073cf4b)
- Local, and the strongest evidence here: `arduino:renesas_uno` core 1.6.0 under
  `~/.arduino15/packages/arduino/hardware/renesas_uno/1.6.0/`
