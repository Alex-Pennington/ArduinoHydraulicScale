# Taking a PGN census off the tractor with the RH-02

**What this is:** the sequence someone follows, in order, to get a candump log
worth decoding off the JD 5055E's CAN bus. Written from behaviour measured on
saga on 2026-09-20, not from the man page — the two traps in §4 and §6 are not
discoverable from documentation and both will silently ruin a capture.

**What this is not:** anything about the Arduino path, the transceiver bench test,
or the permanent on-tractor node. This is the instrument procedure only.

**You need root.** Every `ip link` command needs `sudo`. Find that out here rather
than standing at the tractor.

---

## 1. The instrument

Waveshare/Jhoinrch **RH-02**, a USB-to-CAN converter. It is a CANable clone
shipping candleLight firmware, so on Linux it is a *native SocketCAN interface* —
no serial protocol, no vendor tool. Verified on saga:

```
$ lsusb
Bus 001 Device 013: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter

$ ip -details link show can0
can0: <NOARP,ECHO> mtu 16 ... state DOWN
    can state STOPPED restart-ms 0
      gs_usb: tseg1 1..16 tseg2 1..8 sjw 1..4 brp 1..1024 brp_inc 1
      clock 48000000
```

If `can0` does not appear, the `gs_usb` driver has not bound. It is in-tree —
`modinfo gs_usb` confirms it on kernel 6.8.0-139 — so suspect the cable or the
port before suspecting the adapter.

`can-utils` provides `candump`, `cansniffer`, `cangen`, `cansend`.

## 2. The termination switch — OFF at the tractor

The RH-02 has an **R120** slide switch that fits a 120 Ω terminator across the bus.

- **Tapping the tractor: OFF.** The vehicle bus is already terminated at both
  ends. A third terminator drops the bus impedance and degrades signalling for
  every ECU on it, not just for us.
- **Two-node bench bus: ON.** An isolated pair needs termination at each end.

This is the opposite setting in the two situations, which is exactly why it gets
left wrong.

## 3. Wiring

Three connections at the diagnostic connector: **CAN_H**, **CAN_L**, **GND**.
The RH-02's terminal block is labelled. Ground is not optional — a floating
adapter reads noise.

On the 5055E the connector is round, low and to the right of the driver's seat.
Meter it before trusting any published pinout; Cal Poly names pins C and D as
CAN high and low on a 5055E, and that is a starting point, not an authority.

## 4. Bring the interface up — and the trap that makes a capture worthless

```
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000 loopback off listen-only on
```

**Write every flag explicitly, including the ones you want OFF.**

SocketCAN **retains ctrlmode flags across bring-ups**. If the adapter was used in
loopback earlier — which it will have been, if anyone bench-tested it — then
bringing it up with only `listen-only on` leaves loopback set:

```
# after an earlier loopback session, this is what you get:
$ sudo ip link set can0 up type can bitrate 250000 listen-only on
$ ip -details link show can0
    can <LOOPBACK,LISTEN-ONLY> state ERROR-ACTIVE      # <-- loopback survived
```

versus, with the flag cleared explicitly:

```
$ sudo ip link set can0 up type can bitrate 250000 loopback off listen-only on
$ ip -details link show can0
    can <LISTEN-ONLY> state ERROR-ACTIVE               # <-- correct
```

A loopback interface at the tractor captures its own echo and may capture nothing
from the bus at all. The log will look plausible and be worthless.

**So the verification step is not optional. Every time, before trusting a capture:**

```
ip -details link show can0
```

Read the flag list. It must say `<LISTEN-ONLY>` and nothing else.

**Why listen-only.** The adapter receives and never transmits, including never
sending an acknowledge bit, so it is electrically silent. Connecting a diagnostic
node cannot perturb a working machine. Note the trade-off for completeness: a
listen-only node does not ACK, which is harmless on a bus with several healthy
ECUs because the others acknowledge, but would matter if ours were the only
listener for a given talker.

## 5. Capture, at both bit rates

```
candump -l can0                 # writes candump-<timestamp>.log in the cwd
candump -td can0                # live, with deltas, for watching
cansniffer can0                 # live, grouped by id, for seeing what repeats
```

Take **at least a minute** at each rate with the engine running, and a second
capture with the key on and the engine off — the difference tells you which PGNs
are engine-sourced.

### RECORD THE FRAME RATE AS A RESULT, not just the PGN list

This is easy to skip because the log contains the data, so it feels captured. It
is not captured until it is written down, and **it cannot be recovered later
without going back to the tractor.**

The firmware's loop budget is driven by *rate*, not by which PGNs exist:

> maximum tolerable loop period = ring depth ÷ frame rate = **32 ÷ fps**

A census listing the PGNs without the rate answers half the question and leaves
issue #24 unable to choose between its options. See
`findings/2026-09-20-soak-and-drain-cadence.md`.

Write down **both sustained and peak**. Peak matters more: a burst that overruns
the ring loses frames even if the average is comfortable.

From a `candump -l` log:

```
# sustained frames/s over the whole capture
awk -F'[()]' '{print $2}' candump-*.log | sort -n | \
  awk 'NR==1{a=$1} {n++; b=$1} END{printf "sustained %.1f fps over %.1f s (%d frames)\n", n/(b-a), b-a, n}'

# peak frames/s, measured in 100 ms bins
awk -F'[()]' '{print int($2*10)}' candump-*.log | uniq -c | \
  awk '{if($1>m) m=$1} END{printf "peak %d fps (busiest 100 ms bin x10)\n", m*10}'
```

Record them beside the PGN census. Two numbers, and they decide the firmware
design.

**Both commands are tested, not merely plausible.** Run against a synthetic log
of 2,300 frames — 200 fps sustained for 10 s with a 300-frame burst packed into
500 ms — they returned `sustained 230.1 fps` and `peak 800 fps`, which are the
constructed values exactly (2,300 ÷ 10 s, and 200 + 600 during the burst). Shell
one-liners in a procedure are worth no more than any other untested claim, and
this file has already carried two that were wrong.

Capture at **250k first, then 500k**:

```
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000 loopback off listen-only on
ip -details link show can0      # verify again; flags persist
candump -l can0
```

## 6. Telling a wrong bit rate from a dead bus

This is the distinction that wastes an afternoon in a field, because both look
like "no data" in `candump`.

Check the interface statistics rather than the empty screen:

```
ip -details -statistics link show can0
```

The counters to read are on the line beginning `re-started`:

```
	  re-started bus-errors arbit-lost error-warn error-pass bus-off
	  0          0          0          0          0          0
```

- **Dead bus, or not connected — MEASURED on saga, disconnected, 250k,
  listen-only:** all six counters stay at zero and the state stays
  `ERROR-ACTIVE`. The controller is sitting in idle seeing nothing happen. The
  output above is that case, taken from a real run. Suspect wiring, ground, or
  the wrong connector.

- **Transmitting with nobody acknowledging — MEASURED 2026-09-20** on the
  two-node bench with `CAN_H`/`CAN_L` swapped, so the far node could not hear us:
  the state walks to `ERROR-PASSIVE` and `error-warn` / `error-pass` increment,
  while **`bus-errors` stays at zero**.

  ```
	  re-started bus-errors arbit-lost error-warn error-pass bus-off
	  0          0          0          4          1          0
  ```

  Zero bus-errors with climbing error-warn/error-pass means **the bits are fine
  and nothing is answering** — we are alone on the bus, or the pair is swapped,
  open, or landed on the wrong pins. It is a wiring fault, not a bit-rate fault.
  Note this only appears if you TRANSMIT; a listen-only capture never acks and
  never provokes it, which is another reason to bench-test in normal mode before
  going to the tractor. See `findings/2026-09-20-two-node-bench.md`.

- **Wrong bit rate on a live bus — NOW MEASURED, and the earlier text here was
  WRONG.** This note previously said `bus-errors` would climb and that this was
  the distinguishing counter. It does not, and it cannot:

  ```
  $ sudo ip link set can0 up type can bitrate 500000 ... berr-reporting on
  RTNETLINK answers: Operation not supported
  ```

  **The gs_usb driver does not implement bus-error reporting, so `bus-errors` is
  structurally always zero on the RH-02.** Do not wait for it to move.

  What actually happens, measured with a node transmitting at 250k while the
  RH-02 listened at 500k:

  ```
        re-started bus-errors arbit-lost error-warn error-pass bus-off
  start 0          0          0          43         10174      0
  +25s  0          0          0          43         86732      0
  ```

  `candump` shows **nothing** — identical on screen to a dead bus — while
  `error-pass` climbs by about **3,000 per second** and the state sits at
  `ERROR-PASSIVE`.

**THE DISCRIMINATOR IS WHETHER THE COUNTERS MOVE, NOT WHICH COUNTER.**

| Situation | Frames | State | Counters over a minute |
|---|---|---|---|
| Wrong bit rate, live bus | none | `ERROR-PASSIVE` | `error-pass` climbing fast (~3,000/s) |
| Dead bus / not connected | none | `ERROR-ACTIVE` | all six static |
| Correct bit rate | flowing | settles | `error-pass` ~+2 in 8 s |

Dropping back to the correct rate produced frames immediately and `error-pass`
rose by **2 in 8 seconds**, against ~3,000/s at the wrong rate — three orders of
magnitude apart, so this is not a subtle judgement call.

See `findings/2026-09-20-soak-and-drain-cadence.md`.

**Rising error counters mean the bus is alive and you are listening wrongly.**
Zero counters and zero frames mean you are not electrically on a bus at all.

**Take the counters BEFORE and AFTER a minute of listening and compare**, rather
than glancing once — "zero" is only meaningful against a previous zero.

If 250k gives errors and 500k gives clean frames, the bus is 500k — J1939 is
normally 250k but that is a convention, not a guarantee, which is why the census
is taken at both.

## 7. Afterwards

```
sudo ip link set can0 down
```

Leave the interface down. It was down before you started, and an interface left
up is a state change on somebody's machine.

Keep the `.log` files. `candump -l` format is canonical and replayable with
`canplayer`, and it is what any decode tooling should read.

## 8. A bench self-test, and the second trap

To prove the adapter and the tooling before going near a tractor, use controller
loopback — no bus, no second node:

```
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000 loopback on listen-only off
cansend can0 18FEE000#DEADBEEF01234567
candump -td can0
```

Measured result on saga, and the trap:

```
 (000.000000)  can0  18FEE000   [8]  DE AD BE EF 01 23 45 67
 (000.000578)  can0  18FEE000   [8]  DE AD BE EF 01 23 45 67
```

**Every transmitted frame appears TWICE** — once as SocketCAN's local echo to
other sockets, once as the hardware loopback return. That is correct behaviour
and it is not duplication on the wire.

It does not affect a real listen-only capture, where nothing is transmitted and
each line is one real frame. But anyone who bench-tests first and then reads a
tractor capture expecting the doubling — or, worse, sees doubling in a tractor
capture and assumes it is normal — will misread it. Doubling in a listen-only
capture means the interface is not actually in listen-only. See §4.

Remember to clear `loopback` before the real capture.

## 9. What to do with the log

`tools/j1939_corpus_check.py` currently reads the Cal Poly corpus's PolyCAN CSV
format, not candump. It does not read these logs yet. The decode table it
validates — PGN, source address, and the scaling constants — is in
`REPORT.md` §6 and `findings/2026-09-18-calpoly-can-corpus.md`.

Expect roughly 40% of frames to be proprietary Deere PGNs that nothing public
decodes. That is normal and was true of the Cal Poly captures too.
