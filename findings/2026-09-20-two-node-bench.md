# Two-node bench: the R4 works against a foreign node over a real bus

**2026-09-20. PASS in both directions, zero frame loss up to ~1000 frames/s.**

This is the claim #19 could not make. The loopback proof showed the controller
receives a frame it transmitted itself, through the pads, over a wire. This shows
the R4 exchanging frames with an **independent node** across a **transceiver** and
a **real differential bus** — which is what the tractor actually is.

**This is NOT issue #21.** #21 is specifically the R4 WiFi on D10/D13 and still
waits for that board. This is a separate bench proof on a Minima.

## Rig

| | |
|---|---|
| Node A | Arduino UNO R4 **Minima**, `/dev/ttyACM0`, `sketches/can_twonode` |
| Transceiver | **Waveshare SN65HVD230 CAN Board**, powered from the Minima's **3V3** |
| Node B | **Jhoinrch RH-02** USB-CAN (CANable/candleLight), Linux SocketCAN `can0` |
| Bit rate | 250 kbit/s both ends |
| RH-02 mode | **normal** — `loopback off listen-only off`, verified with `ip -details` |

The RH-02 must be in normal mode. In listen-only it cannot ACK, the Minima
retransmits forever and reports transmit failures, and the whole thing reads as
"the R4 cannot transmit" when it is actually an interface setting.

## The wiring fault, and its signature

The first run failed completely — nothing passed either way. **`CAN_H` and
`CAN_L` were swapped.** Once Alex corrected them it worked immediately and has
worked since.

The signature is worth recording, because a swapped pair is silent and looks
like a dead stack:

- **On the RH-02:** state walked to `ERROR-PASSIVE`, with `error-warn` and
  `error-pass` incrementing but **`bus-errors` staying at zero**.
- **On the Minima:** `CAN.write()` returned **`rc = -60003`** on every attempt
  after the first. The first succeeds because it is accepted into the mailbox;
  it then never completes, because nothing acknowledges it, so the mailbox stays
  occupied and every later write is refused.

**Error-warn and error-pass climbing while bus-errors stays zero means: we are
transmitting and nobody is acknowledging.** That is distinct from bus-errors
climbing, which means the bits themselves are being misread — a wrong bit rate or
a corrupted signal. Two different faults, two different counter signatures.

That distinction was flagged as unmeasured in
`notes/2026-09-20-rh02-capture-procedure.md` §6. Half of it is now measured: the
no-acknowledgement case. The wrong-bit-rate case is still inferred.

## Direction 1 — Minima transmits, RH-02 captures

```
 (000.000000)  can0  18FF0100   [8]  00 00 00 23 C0 FF EE 00
 (000.999372)  can0  18FF0100   [8]  00 00 00 24 C0 FF EE 00
 (000.999830)  can0  18FF0100   [8]  00 00 00 25 C0 FF EE 00
 (000.999975)  can0  18FF0100   [8]  00 00 00 26 C0 FF EE 00
 (001.000023)  can0  18FF0100   [8]  00 00 00 27 C0 FF EE 00
```

Sequence numbers contiguous (`23`…`2E`, no gaps), interval 1.000 s as programmed,
payload intact. The R4 puts well-formed extended frames on a real bus.

## Direction 2 — RH-02 transmits, Minima receives — THE ONE THAT MATTERS

```
RX  ext 0CF00400  PGN  61444  SA   0  dlc 8  FFFF6812FFFFFFFF
RX  ext 18FECA03  PGN  65226  SA   3  dlc 8  01FFF4FEFF00FFFF
```

Frames originated by a foreign node, arriving intact through a transceiver:
29-bit ID preserved, DLC correct, all eight payload bytes correct. The on-board
J1939 decode resolved them to **PGN 61444 (EEC1) from SA 0** and **PGN 65226
(DM1) from SA 3** — the same shapes seen in the Cal Poly 5055E corpus.

## Frame loss under load — none found

| Burst | Gap | Sent | Received | Lost |
|---|---|---|---|---|
| 1 | 5 ms | 200 | 200 | **0** |
| 2 | 1 ms | 500 | 500 | **0** |

Cumulative receive counter reconciles exactly: `rx 703` = 3 + 200 + 500.

This was with the sketch printing **every** received frame to serial. The
115200 setting is cosmetic over USB CDC — actual throughput is USB speed — so
serial was not the limiting factor and this is a genuine ~1000 frame/s arrival
rate with per-frame formatting in the loop.

**What that does and does not tell us.** The driver and the 32-frame ring keep up
with bursts well above typical J1939 rates; at 250 kbit/s the theoretical ceiling
is roughly 2000 frames/s. It does **not** clear the integration risk in
`REPORT.md` §4.3: the scale firmware's main loop is 500 ms, and a ring that
absorbs a burst inside a tight loop will still overflow if it is only drained
every half second. The drain cadence problem is unchanged.

## The transceiver — it worked, and that is not an endorsement

The SN65HVD230 is a **3.3 V** part and the UNO R4 is fixed at **5 V** logic. This
pairing is out of specification in both directions, **and both claims are from the
datasheets rather than from recollection** — they were checked after Alex rightly
pushed back on them being asserted:

**Direction 1 — the Arduino overdrives the transceiver's input.**
TI SN65HVD230 datasheet (**SLOS346O**, March 2001 rev. April 2018), §8.1 *Absolute Maximum Ratings*:

> Digital Input and Output voltage, VI (D or R): **–0.5 to VCC + 0.5 V**

§8.3 gives VCC as 3.0–3.6 V. At 3.3 V the ceiling is **3.8 V**; the Minima drives
`D` at 5 V, exceeding the absolute maximum by about 1.2 V. Absolute maximum is a
damage limit, not a performance limit.

**Direction 2 — the transceiver underdrives the Arduino's input.**
Renesas RA4M1 datasheet, Table 2.4 *I/O VIH, VIL*:

> Other peripheral input pins excluding IIC: **VIH = VCC × 0.8**

At VCC = 5 V that requires **4.0 V** for a guaranteed logic high. TI §8.6 specifies
the receiver output as **VOH min 2.4 V** (at IO = –8 mA); unloaded it sits near the
3.3 V rail. Either figure is below 4.0 V.

On the Minima, `D4` is port **P103** and `D5` is **P102** (core `MINIMA/variant.cpp`),
which are peripheral input/output pins covered by that row.

**Why it worked anyway.** Those are guaranteed limits across process and
temperature, not typical values. A real RA4M1 sample switches nearer VCC/2 than
its specified 4.0 V, and a lightly loaded SN65HVD230 drives `R` close to 3.3 V.
Neither manufacturer guarantees that margin, and nothing about this bench run
establishes it holds on another sample, at temperature, or on longer leads.

**It nonetheless worked perfectly here** — every test above passed with this
part. That is precisely the outcome warned about: *it can pass on a bench and
fail hot.* Short leads, room temperature, a quiet two-node bus and a part being
driven outside its ratings is the easiest possible case.

**Do not read this as clearance to put an SN65HVD230 on the tractor.** What this
bench proves is the *CAN stack and the firmware*, not the transceiver choice.

**CORRECTION, 2026-09-20 — the replacement parts first named here were wrong.**
This section originally said "use a 5 V transceiver — TJA1050 or MCP2551 — which
is in spec against a 5 V R4 and needs no argument." That is wrong on the receive
direction and the confidence was misplaced. Both publish their high-level output
only at **0.7 × VCC = 3.5 V**, which is *under* the RA4M1's 4.0 V threshold —
MCP2551 gives VOH min `0.7 VDD` at IOH = 8 mA, and TJA1050 publishes **no VOH
minimum at all**, only IOH −2 mA at `VRXD = 0.7 VCC`. They fix the overdrive and
leave the underdrive exactly where it was. Additionally **MCP2551 is marked "Not
Recommended for New Designs"** on page 1 of its own datasheet.

The part that closes **both** directions on guaranteed numbers is a plain
**`TJA1051T`**: TXD VIH min 0.7·VIO = 3.5 V with an absolute maximum of
VIO + 0.3 = 5.3 V, so it accepts a 5 V drive; and RXD guaranteed to
VIO − 0.4 = **4.6 V**, clearing the 4.0 V threshold. Its ±58 V bus rating also
suits a 12 V automotive harness. Specify the plain `T`, **not** the `T/3` — with
a 5 V MCU the VIO pin buys nothing, costs specified loop delay, and creates a
path to wire VIO to 3.3 V that would silently restore this very defect while
still passing a bench test. Lifecycle and stock were not confirmable from this
host and must be checked at a distributor before ordering. Full analysis in
`notes/2026-09-20-can-level-interface-plan.md`.

**CORRECTION — "5 V would have destroyed it" overstated the datasheet.** VCC was
on **3V3**, not 5V, which is correct and is what it should stay on. But SLOS346O
§8.1 gives `Supply voltage, VCC` an absolute maximum of **−0.3 to 6 V**, so 5 V
is *below* the damage limit. The real objection is that the part is simply
**unspecified** outside the §8.3 recommended range of 3.0–3.6 V: behaviour there
is not characterised, not that destruction is assured. Keep it on 3V3 — but for
the documented reason, not an invented one.

## The bench rig runs unprotected, deliberately

**Decided by Alex, 2026-09-20: leave it, run it, replace it if it degrades.** No
divider is fitted and `D4` stays wired directly to the transceiver's `D` input.
That is a considered choice for a desk rig, not an oversight, and it is recorded
here so nobody "fixes" it without knowing.

The reasoning that informed it, and one correction to a tempting assumption:

**Not transmitting does NOT avoid the overdrive.** CAN's idle state is recessive
= logic high, so once `CAN.begin()` hands the pin to the peripheral, `CANTX` sits
at **5 V continuously**, whether or not a frame is ever sent. The stress on the
transceiver's input is a **DC condition**, not a per-frame one. A listener node
is overdriven just as hard as a talker.

**What failure will look like.** An overdriven CMOS input rarely fails cleanly.
Expect gradual degradation presenting as intermittent frame loss weeks later,
rather than a part that stops working today. On a bench that is fine — it gets
noticed and swapped. On a tractor it would present as an intermittent CAN fault,
which is among the worst things to diagnose in a field.

**For the tractor there is a free fix that is better than a divider.** If the
node never transmits — which is the design, and which Alex ruled on 2026-09-18
for the first live capture — then do not connect the MCU's CANTX to the
transceiver at all. **Tie the transceiver's `D` pin to its own 3.3 V rail.**
`D` high is recessive, so the transceiver physically cannot drive the bus:

- no overdrive, because the 5 V pin connects to nothing
- no parts, no resistors
- a **hardware** guarantee of silence, which is a stronger form of the
  listen-only property that issue #20 proposed to obtain by vendoring a patched
  `Arduino_CAN` — a ~400 line fork, since closed

The node cannot ACK in that configuration, which is harmless on a bus with
several healthy ECUs and is the same trade-off listen-only always carried. Keep
`D` wired on the bench so two-node transmit tests still work; unwire it for the
machine.

## What is now established

1. The R4 transmits well-formed extended frames onto a real differential bus.
2. **The R4 receives frames from a foreign node**, intact, through a transceiver.
3. No frame loss to ~1000 frames/s, well above expected J1939 rates.
4. On-board PGN and source-address decode is correct against real received frames.

Still open: the R4 **WiFi** on D10/D13 (#21), the tractor's actual bit rate and
PGN set, and the drain cadence once this is folded into the 500 ms scale loop.
