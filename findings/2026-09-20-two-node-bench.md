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
pairing is out of specification in both directions:

- Minima CANTX at 5 V into the transceiver's `D` input, whose absolute maximum is
  about 3.6 V.
- Transceiver `R` output at 3.3 V into the Minima's input, against an RA4M1
  input-high threshold of roughly 0.8 × VCC = 4.0 V.

**It nonetheless worked perfectly here** — every test above passed with this
part. That is precisely the outcome warned about: *it can pass on a bench and
fail hot.* Short leads, room temperature, a quiet two-node bus and a part being
driven outside its ratings is the easiest possible case.

**Do not read this as clearance to put an SN65HVD230 on the tractor.** For the
permanent node use a 5 V transceiver — TJA1050 or MCP2551 — which is in spec
against a 5 V R4 and needs no argument. What this bench proves is the *CAN stack
and the firmware*, not the transceiver choice.

VCC was on **3V3**, not 5V. On 5V the part would likely have been destroyed.

## What is now established

1. The R4 transmits well-formed extended frames onto a real differential bus.
2. **The R4 receives frames from a foreign node**, intact, through a transceiver.
3. No frame loss to ~1000 frames/s, well above expected J1939 rates.
4. On-board PGN and source-address decode is correct against real received frames.

Still open: the R4 **WiFi** on D10/D13 (#21), the tractor's actual bit rate and
PGN set, and the drain cadence once this is folded into the 500 ms scale loop.
