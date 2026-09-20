# Soak test: the 500 ms loop loses two thirds of the bus

**2026-09-20.** Sustained receive testing on the desk rig, with loss measured by
**sequence gaps** rather than by comparing totals. Two results matter:

1. **`REPORT.md` §4.3 is answered and it is worse than feared.** Folding the CAN
   reader into the scale firmware's 500 ms loop loses **67.4%** of frames at a
   modest 200 frame/s bus load.
2. **The bit-rate-mismatch signature in the capture procedure was wrong**, and is
   now measured. `bus-errors` is not the discriminator — on this adapter it can
   never be anything but zero.

Rig as before: UNO R4 Minima + Waveshare SN65HVD230 (on 3V3, unprotected by
decision) against the RH-02 on SocketCAN `can0`, 250 kbit/s.

Sketch `sketches/can_soak`, sender `tools/can_soak_send.py`. The sender puts a
32-bit big-endian counter in the payload so a dropped frame is detected by its
**absence** — `cangen` randomises payloads and cannot support this.

## The drain cadence measurement

`can_soak` takes a `d<ms>` command that inserts a deliberate blocking delay in
`loop()`, imitating a main loop with other work to do. All runs at 200 frames/s.

| Loop delay | Sent | Received | Lost | Loss | Ring high-water |
|---|---|---|---|---|---|
| 0 ms | 11,996 | 11,996 | 0 | **0 ppm** | 1 |
| 5 ms | 6,000 | 6,000 | 0 | **0 ppm** | 2 |
| 150 ms | 4,000 | 4,000 | 0 | **0 ppm** | **30** |
| **500 ms** | 5,998 | 1,938 | **4,009** | **674,121 ppm (67.4%)** | **32 (full)** |

**The arithmetic is exact, which is what makes it trustworthy.** At 200 frames/s
a 500 ms window admits 100 frames. The ring holds 32. So 68 of every 100 are
lost — predicted 68%, measured 67.4%.

At 150 ms the ring peaks at **30 of 32** — right at the edge, and still lossless.
That is the threshold being approached from below, exactly where theory puts it
(32 ÷ 200 = 160 ms).

### The rule that follows

> **Maximum tolerable loop period = ring depth ÷ frame rate = 32 ÷ fps**

At 200 frames/s that is **160 ms**. At a busier 500 frames/s it is **64 ms**. The
scale firmware's loop is 500 ms, so it is over budget by 3× at a load the tractor
will comfortably exceed.

### What this means for issue #24

Folding the CAN reader into the existing 500 ms loop **as-is does not work**. The
options, and none of them is a small edit:

- **Drain CAN several times per loop pass** — call the drain from inside the
  sampling wait rather than once at the top. Cheapest change, but it means every
  long-running section in the firmware becomes a place a frame can be lost.
- **Hardware filtering to cut the arrival rate.** With 8 extended filter slots
  and the corpus showing over 40% of the bus is proprietary traffic we do not
  decode, this is worth doing regardless — but filtering to under 64 frames/s
  would mean discarding PGNs we want.
- **Shorten the main loop.** Largest blast radius; the 500 ms cadence is load
  bearing elsewhere.

The loss is also **silent**: `CanMsgRingbuffer::enqueue()` drops the newest frame
when full and increments nothing. Without a sequence number in the payload — which
real J1939 traffic does not have — the firmware cannot tell a quiet bus from a
lost one. Any integration needs its own liveness signal, which is what the
`frames` counter in the telemetry payload is for.

## The bit-rate mismatch signature — measured, and it corrects the procedure

`notes/2026-09-20-rh02-capture-procedure.md` §6 said that a wrong bit rate on a
live bus makes **`bus-errors`** climb, and labelled that "reasoned, not
measured". It has now been measured and **it is wrong.**

Setup: Minima transmitting at 250 kbit/s, RH-02 brought up at **500 kbit/s**,
listen-only — the tractor configuration with the wrong rate.

| | re-started | bus-errors | arbit-lost | error-warn | error-pass | bus-off |
|---|---|---|---|---|---|---|
| start | 0 | **0** | 0 | 43 | 10,174 | 0 |
| +25 s | 0 | **0** | 0 | 43 | **86,732** | 0 |

`candump` captured **zero frames** — indistinguishable on screen from a dead bus.
`error-pass` climbed by **76,558 in 25 seconds, about 3,000/s**, while
`bus-errors` never moved.

### Why bus-errors is useless here

```
$ sudo ip link set can0 up type can bitrate 500000 ... berr-reporting on
RTNETLINK answers: Operation not supported
```

**The gs_usb driver does not implement bus-error reporting.** `bus-errors` is
structurally always zero on this adapter. The procedure told the reader to watch
a counter this hardware cannot produce.

### The discriminator that actually works

It is not *which* counter, it is **whether the counters are moving**:

| Situation | Frames | State | Counters |
|---|---|---|---|
| **Wrong bit rate, live bus** | none | `ERROR-PASSIVE` | `error-pass` climbing fast, ~3,000/s |
| **Dead bus / not connected** | none | `ERROR-ACTIVE` | all six static |
| **Correct bit rate** | flowing | `ERROR-WARNING`→normal | `error-pass` +2 in 8 s |

The contrast at the correct rate is the clincher: dropping back to 250 kbit/s
produced frames immediately and `error-pass` rose by **2 in 8 seconds** against
~3,000/s at the wrong rate. Three orders of magnitude.

**So at the tractor: take the counters, wait a minute, take them again.** Moving
fast means live bus, wrong rate. Static means you are not on a bus at all.

## Caveats

- 200 frames/s is a **conservative** stand-in for a real J1939 bus. The Cal Poly
  corpus implies a busier one, which makes the loop-period budget tighter, not
  looser.
- The `ERROR-WARNING` state persisting after recovery is a counter artefact of
  the earlier mismatch, not a live fault.
- The Minima's transmit attempts fail throughout, because a listen-only RH-02
  never acknowledges, so it retransmits the same frame. That is expected and is
  why every captured frame shows sequence 0.
- Still unmeasured: behaviour at temperature, and on leads longer than a desk.
