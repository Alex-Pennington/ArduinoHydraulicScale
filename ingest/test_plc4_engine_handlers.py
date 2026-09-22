"""PROPOSED test for consumer_plc4_engine.py -- NOT the consumer's test.

Runs here, standalone, against a STUB cursor that records statements:

    python3 ingest/test_plc4_engine_handlers.py

A stub-cursor pass is not evidence the consumer will accept (#25 hand-over: "a
stub-cursor result will not be quoted as evidence"). This file is the case list to
port into the consumer's real-Postgres harness (scripts/run-consumer-tests.sh),
and the port must add its docs/DESIGN-STANDARD.md section 4 entry in the same
change or tests/run-ritual.sh jams.

The reboot case (counters decrease) is a VIEW behaviour, not a handler one: the
handler stores whatever arrives. Its SQL fixture is the last case here, returned
as text for the port to run against v_plc4_can_health.

Payload shapes are copied from publishEngine() in src/ArduinoHydraulicScale.cpp
at HEAD and at 19f0c02 (the image on the tractor since 2026-09-22). Values are
from the 19f0c022 field messages relayed 2026-09-22.
"""

import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import consumer_plc4_engine as h  # noqa: E402

LOG = logging.getLogger("test")
LOG.addHandler(logging.NullHandler())
LOG.propagate = False


class StubCursor:
    def __init__(self):
        self.calls = []
        self._next_id = 100

    def execute(self, sql, params=()):
        self.calls.append((" ".join(sql.split()), tuple(params)))

    def fetchone(self):
        self._next_id += 1
        return (self._next_id,)

    def inserts(self, table):
        return [c for c in self.calls if c[0].startswith("INSERT INTO %s " % table)
                or c[0].startswith("INSERT INTO %s(" % table)]

    def unmapped(self):
        return [c[1] for c in self.inserts("ingest_unmapped")]

    def engine_row(self):
        rows = self.inserts("plc4_engine")
        assert len(rows) == 1, rows
        sql, params = rows[0]
        cols = sql.split("(", 1)[1].split(")", 1)[0].split(", ")
        return dict(zip(cols, params))


ENGINE = h.PLC4_ENGINE_TOPIC

# 19f0c022 running row, 2026-09-22 19:41:35: no rpm (the freshness bug fixed
# after 705bf3c), no drains, no can_last.
P_19F0C022 = {
    "coolant_c": 26.0, "oil_kpa": 392, "fuel_lph": 1.80, "keysw_v": 14.25,
    "frames": 123456, "unknown": 98765, "ring_full": 0, "ring_high": 9,
    "can_err": 0, "can_up": True, "filt": False, "id": "plc4-jd5055e",
}

P_HEAD = dict(P_19F0C022, rpm=1500, hours=598.65, can_last=0, drains=40000)


def case_19f0c022_shape():
    cur = StubCursor()
    h.handle_plc4_engine(cur, ENGINE, dict(P_19F0C022), LOG)
    assert cur.unmapped() == [], cur.unmapped()
    row = cur.engine_row()
    assert row["drains"] is None and row["can_last"] is None
    assert row["rpm"] is None and row["hours"] is None
    assert row["frames"] == 123456 and row["unknown_frames"] == 98765
    assert row["ring_high"] == 9 and row["can_up"] is True and row["filt"] is False
    assert row["board_id"] == "plc4-jd5055e"


def case_head_shape():
    cur = StubCursor()
    h.handle_plc4_engine(cur, ENGINE, dict(P_HEAD), LOG)
    assert cur.unmapped() == []
    row = cur.engine_row()
    assert row["drains"] == 40000 and row["can_last"] == 0 and row["rpm"] == 1500.0


def case_key_off_counters_only():
    cur = StubCursor()
    p = {k: P_19F0C022[k] for k in ("frames", "unknown", "ring_full", "ring_high",
                                    "can_err", "can_up", "filt", "id")}
    h.handle_plc4_engine(cur, ENGINE, p, LOG)
    assert cur.unmapped() == []
    row = cur.engine_row()
    assert all(row[k] is None for k in h._PLC4_ENGINE_FIELDS)


def case_required_counter_missing():
    cur = StubCursor()
    p = dict(P_19F0C022)
    del p["ring_full"]
    h.handle_plc4_engine(cur, ENGINE, p, LOG)
    um = cur.unmapped()
    assert len(um) == 1 and um[0][1] == "field" and um[0][2] == "ring_full", um
    assert cur.engine_row()["ring_full"] is None   # row still stored


def case_wrong_types():
    cur = StubCursor()
    p = dict(P_HEAD, can_up=1, frames=True, drains="12", can_last=2.5, ring_high=33,
             rpm="fast", oil_kpa=5000, id=7)
    h.handle_plc4_engine(cur, ENGINE, p, LOG)
    fields = sorted(u[2] for u in cur.unmapped())
    assert fields == sorted(["can_up", "frames", "drains", "can_last", "ring_high",
                             "rpm", "oil_kpa", "id"]), fields
    row = cur.engine_row()
    assert all(row[c] is None for c in ("can_up", "frames", "drains", "can_last",
                                        "ring_high", "rpm", "oil_kpa", "board_id"))


def case_json_null_value_is_unusable_not_absent():
    cur = StubCursor()
    h.handle_plc4_engine(cur, ENGINE, dict(P_19F0C022, rpm=None), LOG)
    assert [u[2] for u in cur.unmapped()] == ["rpm"]


DTC = h.PLC4_DTC_TOPIC_PREFIX + "3"
D_CLEAR = {"sa": 3, "n": 0, "mil": 0, "rsl": 0, "awl": 0, "pl": 0, "dtc": [], "id": "x"}


def case_dtc_live_vs_retained():
    live, ret = StubCursor(), StubCursor()
    h.handle_plc4_dtc(live, DTC, dict(D_CLEAR), LOG)
    h.handle_plc4_dtc(ret, DTC, dict(D_CLEAR), LOG, retained=True)
    assert live.inserts("plc4_dtc_snapshot")[0][1][-1] == "live"
    assert ret.inserts("plc4_dtc_snapshot")[0][1][-1] == "retained"
    assert live.unmapped() == [] and ret.unmapped() == []


def case_dtc_bad_fields_go_unmapped():
    cur = StubCursor()
    p = dict(D_CLEAR, n=3, rsl=7, dtc=[{"spn": 600000, "fmi": 1, "oc": 1},
                                       {"spn": 100, "fmi": 40, "oc": 1},
                                       {"spn": 100, "fmi": 1, "oc": 200}])
    del p["awl"]
    h.handle_plc4_dtc(cur, DTC, p, LOG)
    fields = [u[2] for u in cur.unmapped()]
    assert fields == ["rsl", "awl", "dtc[0].spn", "dtc[1].fmi", "dtc[2].oc"], fields
    codes = cur.inserts("plc4_dtc_code")
    assert len(codes) == 1 and codes[0][1][-1] is None   # oc NULL, code kept


# For the port: run inside a transaction against the migrated schema.
REBOOT_SQL = """
INSERT INTO plc4_engine (recv_ts, frames, unknown_frames, ring_full, ring_high, can_err, can_up, filt)
VALUES (now() - interval '45 s', 20000, 15000, 0, 9, 0, true, false),
       (now() - interval '30 s', 26900, 20500, 0, 9, 0, true, false),
       (now() - interval '15 s',   500,   400, 0, 3, 0, true, false);
-- expect: last row reboot = true, d_frames IS NULL, fps IS NULL, state_code = 2
SELECT reboot, counter_reset, d_frames, fps, state_code
  FROM v_plc4_can_health ORDER BY recv_ts DESC LIMIT 1;
"""


CASES = [v for k, v in sorted(globals().items()) if k.startswith("case_")]

if __name__ == "__main__":
    failed = 0
    for c in CASES:
        try:
            c()
            print("PASS", c.__name__)
        except AssertionError as e:
            failed += 1
            print("FAIL", c.__name__, e)
    print("%d/%d passed (STUB CURSOR -- not the consumer's evidence)" % (len(CASES) - failed, len(CASES)))
    sys.exit(1 if failed else 0)
