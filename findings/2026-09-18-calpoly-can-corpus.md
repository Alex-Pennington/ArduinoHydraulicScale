# The Cal Poly 5055E CAN corpus, checked against our decode

**2026-09-18.** Alex pointed out that someone had open-sourced CAN work on this specific
tractor model. They had. It corrects three things I had wrong and answers four questions
that were on the bench list.

**Attribution, because I got it wrong first.** This is **Cal Poly** — California
Polytechnic State University's Capstone I/II class, sponsored by iFixit — not Berkeley.
Repository `github.com/tractorhacking/CAN-data`, **MIT licensed**, checked at upstream
commit `db5c894d8810835403a7204238fe7d0498783173` (2019-03-22). Their README states the
data was "collected from a John Deere 5055E tractor". Site content is CC-BY-4.0; the
software is MIT.

**What it is.** 44 captures, 330,653 frames, recorded with their PolyCAN tool in
2018–19 and already split into PGN, priority, source, destination and data bytes. Capture
names describe what was being done: `idle10sec`, `highRPM1..4`, `start1..4`,
`inNeutralForStartUp`, `highBeamFuse1..4`, `noStartInGear1..4`, transmission-in-gear runs.

**What it is not.** It is not our tractor. Different machine, different year, different
hours (theirs reads 138–156 h), different options, captured seven years ago. Everything
below is "this is what a 5055E did", never "this is what ours will do". It replaces
nothing on the bench list — it reorders it, and it caught bugs that the bench would have
charged a day to find.

Reproduce with `tools/j1939_corpus_check.py`:

    git clone https://github.com/tractorhacking/CAN-data /tmp/CAN-data
    python3 tools/j1939_corpus_check.py /tmp/CAN-data

## Three things I had wrong

### 1. TP.DT is PGN 60160, not 60417

I took 60417 from a published source. It is wrong, and the corpus is unambiguous:

| PGN | | frames |
|---|---|---|
| 60416 | 0xEC00, TP.CM | 417 |
| **60160** | **0xEB00, TP.DT** | **1491** |
| 60417 | 0xEC01 | **0** |

The 60160 payloads carry leading sequence bytes `01`, `02`, `03`, `04` — textbook TP.DT.
Had this shipped, every multi-packet message would have been announced and then never
reassembled, and the failure would have looked like "DM1 with multiple faults silently
does nothing". Fixed in `sketches/can_mqtt/can_mqtt.ino`.

### 2. DM1 is per source address, and four ECUs publish it

| Source | frames | payload | lamps |
|---|---|---|---|
| 23 | 69 | `00 FF 00 00 00 00 FF FF` | all clear |
| 6 | 61 | `00 FF 00 00 00 00 FF FF` | all clear |
| 71 | 61 | `00 FF 00 00 00 00 FF FF` | all clear |
| 3 | 55 | `00 FF 00 00 00 00 FF FF` | all clear |
| **3** | **22** | `01 FF F4 FE FF 00 FF FF` | **protect lamp ON** |

They disagree: SA 3 raises the protect lamp while the other three report nothing. My
skeleton kept one global DM1 store, so at 1 Hz it would have flapped between four ECUs
and reported whichever spoke last — and a retained MQTT topic would have frozen on an
arbitrary one. Now tracked per source address, published to `plc4/jd5055e/dtc/<sa>`, one
retained topic per ECU.

### 3. The not-available DTC record is not 0xFF padding

The interesting frame is `01 FF F4 FE FF 00 FF FF`: protect lamp on, and a DTC record of
`F4 FE FF 00`. Reassembled under conversion method 0 that is **SPN 524020, FMI 31,
OC 0** — an SPN in the reserved region, the not-available FMI, and zero occurrences.

It is not a fault numbered 524020. It is the ECU saying "a lamp is on and I am not
telling you which code". My padding guard only skipped `FF FF FF`, so this would have
become a permanent phantom DTC on the dashboard the first time a protect lamp lit. Now
guarded on `spn >= 0x7F000` and on `fmi == 31 && oc == 0`.

Note what this also means: **a lit lamp here carries no readable code**, so the lamp
columns matter independently of the DTC list. The dashboard already shows both.

## Four bench questions answered early

### Engine hours is broadcast — nothing needs to be transmitted

This was the big open one, because the alternative was sending PGN requests onto a live
tractor bus. PGN 65253 appears in **39 of 44 captures**, unprompted, from source 0.

Values across sessions: 138.15, 138.2, 138.25, 147.55, 147.6, 156.35 h. Monotonically
increasing in capture order, which independently confirms the **0.05 h/bit** scaling —
wrong scaling would not advance plausibly across three sessions.

**Issue #23 (whether to transmit requests for hours) is answered: we do not need to.**

### Battery voltage is SPN 158, not SPN 168

Across 1206 VEP1 frames:

- **SPN 168** (bytes 5–6, "battery potential / power input 1") — not-available in **all
  1206**. The payloads are literally `FF FF FF FF FF FF xx xx`.
- **SPN 158** (bytes 7–8, keyswitch battery potential) — available in **all 1206**,
  reading 7.75 to 14.35 V.

And the range is itself a validation: ~12.4 V key-on engine-off, up to 14.2–14.35 V with
the engine running (alternator charging), dipping to 7.75–9.0 V during cranking and to
9.35 V under a deliberate high-beam load in `highBeamFuse1`. That is exactly what a 12 V
system does, which means the 0.05 V/bit scaling is right.

Consequence: a dashboard whose "battery" panel reads SPN 168 would be **permanently
empty**. The firmware decodes both; the panel and the schema now lead with SPN 158.

### DM1 never goes multipacket on this machine

All 358 transport sessions carry PGN 65251 (Engine Configuration) — 240 BAM, 59 RTS/CTS
pairs, 4 aborts. **Zero** announce DM1.

So the BAM reassembly path is unexercised here. It stays in the firmware: J1939-73
requires a DM1 with two or more active faults to arrive that way, and that is precisely
the case we would most want to read. But it is untested code, and should be labelled as
such rather than trusted.

### The bus is a third proprietary traffic

| PGN | share | what |
|---|---|---|
| 61184 | 33.0% | Proprietary A (sources 0, 6, 71) |
| 61444 | 18.1% | EEC1 |
| 61443 | 18.1% | EEC2 |
| 126720 | 8.4% | proprietary (sources 3, 71) |
| 65266 | 3.6% | LFE fuel rate |

EEC1 and EEC2 together are 36% of frames and both come from source 0 (the engine ECU),
as do every other PGN we want except DM1. **Over 41% of the bus is proprietary Deere
traffic we cannot decode and do not need** — which is the strongest argument yet for
hardware filtering: we can discard roughly half the bus in silicon before it ever reaches
the 32-frame ring.

## Decode plausibility, end to end

Our constants against their data. Nothing here was tuned to fit.

| capture | rpm | coolant °C | oil kPa | fuel L/h | battery V (SPN 158) | hours |
|---|---|---|---|---|---|---|
| `idle10sec` | 0 throughout | 34 | 0 | 0 | 12.4–12.6 | 138.2 |
| `inNeutralForStartUp` | median 683, max 1014 | 38 | median 396, max 408 | median 1.6 | 9.0–14.2 | 138.2 |
| `highRPM1` | max 1259 | 30–31 | max 448 | max 6.4 | 9.8–14.1 | 147.6 |
| `highBeamFuse1` | max 1013 | 38 | max 408 | max 5.4 | 9.3–13.9 | 147.6 |

Every figure is physically sensible: oil pressure 396–448 kPa (57–65 psi) with the engine
turning and 0 with it stopped; coolant 30–38 °C on a cold start; fuel rate rising with
rpm; cranking voltage dips paired with running charge voltage.

Note `idle10sec` shows **rpm 0 in all 1078 EEC1 frames** — despite the name, the engine
is not running in that capture. Worth stating because it is the trap in this corpus: the
capture names describe what the operator was doing, not always what the engine was doing.

**Also worth knowing: rpm 0 is a real value, not a sentinel.** The firmware publishes it
and the consumer's plausibility bound accepts it, which is correct — a stopped engine is
data.

## What this does not settle

The bench list in `REPORT.md` §8 is shortened, not retired:

- **Their connector is not necessarily wired like ours**, and none of this proves which
  bus segment our diagnostic port exposes. Still meter it.
- **Nothing here tests our hardware.** `Arduino_CAN` receiving on core 1.6.0 (§8 item 1)
  and the transceiver wiring (item 2) are untouched by this corpus.
- **Bus load is theirs, not ours.** A different options list changes the frame rate.
- **A 2021 machine may differ from a 2018-19 one** in PGN set and in which SPNs are
  populated. SPN 168 being dead on theirs is suggestive, not binding.
