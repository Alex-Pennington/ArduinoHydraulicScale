# Issue #19 answered: Arduino_CAN does receive on renesas_uno core 1.6.0

**2026-09-20. PASS, 7 of 7 checks, reproduced across two flashes.**

The comment in the core's driver is stale. The R4 CAN path is viable and every
downstream plan that depends on it is unblocked.

## What was in doubt

`libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp`, in the FSP callback:

```
case CAN_EVENT_RX_COMPLETE: // Currently driver don't support this. This
                            // is unreachable code for now.
```

That branch is the only thing that enqueues into the ring buffer which
`CAN.available()` and `CAN.read()` serve. If the comment were accurate,
`CAN.available()` would never become non-zero, no frame would ever reach the
sketch, and no amount of correct J1939 decoding would matter. The whole R4
approach rested on it.

Counter-evidence was circumstantial: `IRQManager.cpp:886` does register
`can_rx_isr` and enable it, and the Renesas FSP `r_can` driver emits
`CAN_EVENT_RX_COMPLETE` from that ISR. That justified testing it, not assuming it.

## How it was tested

`sketches/can_loopback/can_loopback.ino`, run on an **Arduino UNO R4 Minima** on
Alex's desk, `/dev/ttyACM0`, with a plain jumper wire between **D4 and D5**.
No transceiver, no bus, no tractor.

**The board matters and is recorded deliberately.** The result was taken on a
Minima, not on the R4 WiFi that goes on the tractor. What transfers is the
answer to the question asked — same RA4M1 silicon, same `renesas_uno` core 1.6.0,
same `Arduino_CAN` — because this is a driver-and-core question. **The pin numbers
do not transfer:** CAN is D4/D5 on the Minima and D10/D13 on the WiFi.

## Why a jumper was needed, which was not obvious

`enableInternalLoopback()` does not engage internal loopback:

```
int R7FA4M1_CAN::enableInternalLoopback() {
  R_CAN_ModeTransition(&_can_ctrl, CAN_OPERATION_MODE_NORMAL,
                       CAN_TEST_MODE_LOOPBACK_EXTERNAL);
}
```

`r_can_api.h:103-104` — `CAN_TEST_MODE_LOOPBACK_EXTERNAL = 5`,
`CAN_TEST_MODE_LOOPBACK_INTERNAL = 7`. The method named "internal" engages
**external** loopback, which drives the CTX pad and samples the CRX pad. The
signal leaves the chip and must come back.

Without the wire, no frame is received — **which is indistinguishable from the
failure mode under test.** The first run proved the point: it correctly refused
to conclude.

```
[FAIL] D4-D5 jumper   D5 did not follow D4 (low reads 0, high reads 0) -- NO JUMPER FITTED

STOPPING. Without the jumper this test cannot distinguish
"the driver never receives" from "the signal had nowhere to go",
and a FAIL here would wrongly condemn the whole R4 CAN path.
```

The sketch proves continuity by driving CANTX as a plain GPIO and reading CANRX
*before* the peripheral claims the pins. That pre-check is the part that makes a
FAIL meaningful.

## The result

With the jumper fitted:

```
board: UNO R4 Minima   CANTX = D4   CANRX = D5

  [PASS] D4-D5 jumper               D4 -> D5 follows (low reads 0, high reads 1)
  [PASS] CAN.begin                  CAN.begin(250000) returned true
  [PASS] enableInternalLoopback     returned 1 (1 = success)
  [PASS] extended-id round trip     id, dlc and all 8 data bytes intact
  [PASS] standard-id round trip     id, dlc and all 3 data bytes intact
  [PASS] multiple frames buffered   sent 4, CAN.available() reports 4
  [PASS] no CAN error events        no error events raised

  7 passed, 0 failed
```

Four findings worth separating out:

1. **The RX path is live.** `CAN.available()` goes non-zero and `CAN.read()`
   returns the frame. The "unreachable code" comment is wrong.
2. **29-bit extended IDs round-trip intact** — id, DLC and all eight data bytes.
   That is the J1939 case and it is the one that mattered.
3. **The 32-slot ring really buffers.** Four frames sent, four reported. It is
   not a one-deep latch.
4. **No error events** were raised during any of it.

## What this does NOT establish

- **Nothing about the transceiver or the wiring to a real bus.** Loopback never
  leaves the board. Issue #21 is untouched by this and still needs the WiFi board.
- **Nothing about the tractor.** No bit rate confirmed, no PGN observed.
- **Nothing about the R4 WiFi specifically.** Same silicon and same core, so the
  driver answer carries; the pin mapping does not.
- **It does not validate listen-only.** That is `CAN_TEST_MODE_LISTEN`, which
  Arduino_CAN does not expose (issue #20, since closed).

## Incidental: DFU uploads are flaky on this host

Two uploads succeeded, then `dfu-util` began failing repeatedly with
`Cannot set alternate interface zero: LIBUSB_ERROR_OTHER` / `exit status 74`,
with the board still enumerated and running.

**A 1200-baud touch on the port clears it every time** — open `/dev/ttyACM0` at
1200 baud and close it, which drops the R4 into its bootloader, then upload
normally. A double-tap of RESET does the same by hand. Worth knowing before
somebody concludes a board is bricked.
