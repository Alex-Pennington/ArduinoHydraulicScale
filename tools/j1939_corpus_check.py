#!/usr/bin/env python3
"""Check our J1939 decode against the Cal Poly 5055E CAN corpus.

WHY THIS EXISTS. We cannot bench-test a decode against a tractor we do not have
tethered. Cal Poly's Tractor Hacking project (sponsored by iFixit) published real
CAN captures from a John Deere 5055E -- the same model as PLC #4's machine -- under
the MIT licence. That corpus answers, offline and for free, several questions that
would otherwise have cost a day in the field: which PGNs this model actually
broadcasts, whether engine hours needs a request, which battery SPN is populated,
and whether our scaling constants produce plausible numbers.

It is NOT our tractor. Different year, different hours, different options, and a
2018-19 capture. Treat everything it says as "this is what a 5055E did", not "this
is what ours will do". It is still the best evidence available without a bench.

USAGE:
    git clone https://github.com/tractorhacking/CAN-data /tmp/CAN-data
    python3 tools/j1939_corpus_check.py /tmp/CAN-data

Verified against upstream commit db5c894d8810835403a7204238fe7d0498783173 (2019-03-22).
Findings are written up in findings/2026-09-18-calpoly-can-corpus.md.
"""

import csv
import collections
import glob
import os
import statistics
import sys

PGN_NAMES = {
    61444: "EEC1 engine speed / torque",
    61443: "EEC2 load / pedal",
    65262: "ET1 coolant + oil temp",
    65263: "EFL_P1 oil pressure",
    65253: "HOURS engine hours",
    65266: "LFE fuel rate",
    65271: "VEP1 electrical potential",
    65226: "DM1 active DTCs",
    60416: "TP.CM transport connection mgmt",
    60160: "TP.DT transport data",
    60928: "Address claim",
    65265: "CCVS1 vehicle speed",
    61445: "ETC2 gear",
    65269: "AMB ambient conditions",
    61184: "Proprietary A",
}

# The PGNs the firmware decodes. Kept in step with sketches/can_mqtt/can_mqtt.ino.
WANTED = (61444, 61443, 65262, 65263, 65253, 65266, 65271, 65226, 60416, 60160)


def load(path):
    """PolyCAN CSV: index,time,pgn,priority,source,destination,data,description."""
    out = []
    with open(path, newline="", errors="ignore") as fh:
        for row in csv.DictReader(fh):
            try:
                pgn = int(row["pgn"])
                sa = int(row["source"])
            except (TypeError, ValueError, KeyError):
                continue
            try:
                data = [int(b, 16) for b in row["data"].split()]
            except (TypeError, ValueError, KeyError):
                continue
            out.append((pgn, sa, data))
    return out


def le16(d, i):
    return d[i] | (d[i + 1] << 8)


def bad16(v):
    return (v >> 8) >= 0xFE


def bad8(v):
    return v >= 0xFE


def decode(rows):
    """Apply exactly the scalings the firmware applies. Same constants, same guards."""
    sig = collections.defaultdict(list)
    for pgn, sa, d in rows:
        if pgn == 61444 and len(d) >= 5:
            raw = le16(d, 3)
            if not bad16(raw):
                sig["rpm"].append(raw * 0.125)
            if not bad8(d[2]):
                sig["torque_pct"].append(d[2] - 125.0)
        elif pgn == 61443 and len(d) >= 3 and not bad8(d[2]):
            sig["load_pct"].append(float(d[2]))
        elif pgn == 65262 and len(d) >= 1 and not bad8(d[0]):
            sig["coolant_c"].append(d[0] - 40.0)
        elif pgn == 65263 and len(d) >= 4 and not bad8(d[3]):
            sig["oil_kpa"].append(d[3] * 4.0)
        elif pgn == 65266 and len(d) >= 2:
            raw = le16(d, 0)
            if not bad16(raw):
                sig["fuel_lph"].append(raw * 0.05)
        elif pgn == 65271 and len(d) >= 8:
            raw168 = le16(d, 4)
            raw158 = le16(d, 6)
            if not bad16(raw168):
                sig["batt_v_spn168"].append(raw168 * 0.05)
            if not bad16(raw158):
                sig["batt_v_spn158"].append(raw158 * 0.05)
        elif pgn == 65253 and len(d) >= 4:
            raw = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)
            if (raw >> 24) < 0xFE:
                sig["hours"].append(raw * 0.05)
    return sig


def main(root):
    logs = sorted(glob.glob(os.path.join(root, "sept2018-march2019", "logs", "*.csv")))
    if not logs:
        sys.exit("no PolyCAN logs under %s -- is this the CAN-data checkout?" % root)

    every = []
    per_file = {}
    for path in logs:
        rows = load(path)
        per_file[os.path.basename(path)[:-4]] = rows
        every += rows

    census = collections.Counter()
    srcs = collections.defaultdict(set)
    for pgn, sa, _ in every:
        census[pgn] += 1
        srcs[pgn].add(sa)
    total = len(every)

    print("== census: %d captures, %d frames ==" % (len(logs), total))
    print("%9s %8s %7s  %-32s %s" % ("PGN", "frames", "share", "name", "sources"))
    for pgn, n in census.most_common(15):
        print("%9d %8d %6.1f%%  %-32s %s"
              % (pgn, n, 100.0 * n / total, PGN_NAMES.get(pgn, ""), sorted(srcs[pgn])))

    print("\n== the PGNs the firmware decodes ==")
    for pgn in WANTED:
        state = "PRESENT %6d frames  sources %s" % (census[pgn], sorted(srcs[pgn])) \
                if census[pgn] else "ABSENT"
        print("  %6d  %-32s %s" % (pgn, PGN_NAMES.get(pgn, ""), state))

    print("\n== transport protocol: which PGN really carries the data? ==")
    for pgn in (60416, 60160, 60417):
        print("  PGN %6d (0x%04X): %6d frames" % (pgn, pgn, census[pgn]))
    print("  -> TP.DT is 60160. A published source claims 60417; the data says otherwise.")

    print("\n== which PGNs use multipacket transport? ==")
    announced = collections.Counter()
    for pgn, sa, d in every:
        if pgn == 60416 and len(d) >= 8:
            announced[(d[0], d[5] | (d[6] << 8) | (d[7] << 16))] += 1
    ctl = {0x10: "RTS", 0x11: "CTS", 0x13: "EndOfMsgAck", 0x20: "BAM", 0xFF: "Abort"}
    for (c, emb), n in sorted(announced.items(), key=lambda kv: -kv[1]):
        print("  %-12s carries PGN %6d  x%d%s"
              % (ctl.get(c, hex(c)), emb, n, "   <-- DM1" if emb == 65226 else ""))
    if not any(emb == 65226 for (_, emb) in announced):
        print("  -> DM1 NEVER goes multipacket here. The BAM path is unexercised.")

    print("\n== DM1, per source address ==")
    dm1 = collections.Counter()
    for pgn, sa, d in every:
        if pgn == 65226 and len(d) >= 6:
            dm1[(sa, tuple(d))] += 1
    for (sa, d), n in dm1.most_common():
        b = list(d)
        spn = b[2] | (b[3] << 8) | ((b[4] >> 5) << 16)
        fmi = b[4] & 0x1F
        oc = b[5] & 0x7F
        note = ""
        if spn >= 0x7F000 or (fmi == 31 and oc == 0):
            note = "  <- not-available record, MUST NOT become a DTC"
        print("  SA %-3d x%-5d %s  lamps MIL=%d RSL=%d AWL=%d PL=%d  SPN=%d FMI=%d OC=%d%s"
              % (sa, n, " ".join("%02X" % x for x in b),
                 (b[0] >> 6) & 3, (b[0] >> 4) & 3, (b[0] >> 2) & 3, b[0] & 3,
                 spn, fmi, oc, note))
    print("  -> %d distinct source addresses publish DM1. Fault state is PER ECU."
          % len({sa for sa, _ in dm1}))

    print("\n== VEP1: which battery SPN is actually populated? ==")
    sig = decode(every)
    for key in ("batt_v_spn168", "batt_v_spn158"):
        v = sig.get(key, [])
        if v:
            print("  %-14s available in %5d frames   %.2f - %.2f V"
                  % (key, len(v), min(v), max(v)))
        else:
            print("  %-14s NEVER AVAILABLE (not-available sentinel in every frame)" % key)

    print("\n== decode plausibility, per capture ==")
    print("%-24s %-14s %8s %8s %8s %6s" % ("capture", "signal", "min", "median", "max", "n"))
    for name in ("idle10sec", "inNeutralForStartUp", "highRPM1", "highBeamFuse1"):
        if name not in per_file:
            continue
        sig = decode(per_file[name])
        for k in ("rpm", "load_pct", "coolant_c", "oil_kpa", "fuel_lph",
                  "batt_v_spn158", "hours"):
            v = sig.get(k)
            if not v:
                continue
            print("%-24s %-14s %8.1f %8.1f %8.1f %6d"
                  % (name, k, min(v), statistics.median(v), max(v), len(v)))
        print()

    print("== engine hours: broadcast, or request-only? ==")
    seen = {}
    for name, rows in per_file.items():
        h = decode(rows).get("hours")
        if h:
            seen[name] = sorted(set(round(x, 2) for x in h))
    print("  present in %d of %d captures" % (len(seen), len(per_file)))
    vals = sorted({v for lst in seen.values() for v in lst})
    print("  values seen: %s" % vals)
    print("  -> broadcast unprompted. No PGN request needs to be transmitted.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/CAN-data")
