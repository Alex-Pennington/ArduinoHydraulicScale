#!/usr/bin/env python3
"""Send sequence-numbered CAN frames for sketches/can_soak.

cangen randomises payloads, so a receiver cannot tell a dropped frame from a
quiet bus. This sends an incrementing 32-bit big-endian counter in the first
four payload bytes, which lets can_soak detect loss by the GAP rather than by
comparing totals — the difference between "we think some went missing" and
"frames 4471 to 4489 never arrived".

Uses SocketCAN through Python's stdlib socket module. No dependencies.

    sudo ip link set can0 up type can bitrate 250000 loopback off listen-only off
    python3 tools/can_soak_send.py --rate 200 --seconds 60

Arguments are deliberately explicit: a default rate that happened to work would
be the kind of unstated assumption this workstream keeps tripping over.
"""

import argparse
import socket
import struct
import sys
import time

CAN_EFF_FLAG = 0x80000000
SEQ_ID = 0x18FF0200


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--rate", type=float, required=True,
                    help="frames per second (target; actual is reported)")
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--start", type=int, default=0, help="first sequence number")
    args = ap.parse_args()

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.iface,))
    except OSError as exc:
        sys.exit("cannot open %s: %s (is the link up?)" % (args.iface, exc))

    period = 1.0 / args.rate
    deadline = time.time() + args.seconds
    seq = args.start
    sent = 0
    errors = 0
    t0 = time.time()
    next_tx = t0

    while time.time() < deadline:
        payload = struct.pack(">I", seq & 0xFFFFFFFF) + b"\xC0\xFF\xEE\x00"
        frame = struct.pack("=IB3x8s", SEQ_ID | CAN_EFF_FLAG, 8, payload)
        try:
            sock.send(frame)
            sent += 1
        except OSError:
            # ENOBUFS when the tx queue backs up: count it, do not pretend it went.
            errors += 1
        seq += 1

        next_tx += period
        sleep_for = next_tx - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tx = time.time()   # we are behind; stop accumulating debt

    elapsed = time.time() - t0
    sock.close()

    print("sent      %d frames in %.1f s" % (sent, elapsed))
    print("actual    %.1f frames/s (target %.1f)" % (sent / elapsed, args.rate))
    print("tx errors %d" % errors)
    print("last seq  %d" % (seq - 1))
    if errors:
        print("NOTE: tx errors mean frames never reached the wire. Subtract them")
        print("      before blaming the receiver for loss.")


if __name__ == "__main__":
    main()
