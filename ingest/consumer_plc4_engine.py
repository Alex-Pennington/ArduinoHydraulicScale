"""PROPOSED handlers for plc4/jd5055e/engine and plc4/jd5055e/dtc.

NOT APPLIED. This is a proposal to hand to the consumer coordinator, who owns
`consumer/plc1_consumer.py` and `db/migrations/`. It is written to drop into
that file's existing shape (module-level topic constants, a handler per topic,
`dispatch()` routing by exact topic string, defensive key access, a cursor
passed in) rather than to run standalone. Pairs with 067_add_plc4_engine.sql.

THE ONE CONTRACT DIFFERENCE, AND IT IS DELIBERATE
-------------------------------------------------
Every other PLC #4 topic uses "always populate the number, put the validity in a
sibling string" -- because an ADC always hands you a count, so a reading can be
present and untrustworthy at the same time. J1939 is not like that. A parameter
the engine does not broadcast is simply not in the payload, and one it
broadcasts as 0xFF.. means "not available" in the standard itself. The board
therefore OMITS the key, and absent means absent.

So these handlers must NOT treat a missing key as a bug to log. `plc4_telemetry`
logs a missing `w`; `plc4_engine` must not log a missing `rpm`. Carrying the
older reflex over is how every 15 s row would raise a warning on a tractor whose
key is off.

THE #25 CONDITION (consumer coordinator, 2026-09-20)
---------------------------------------------------
Absent and unusable are opposite facts and a bare NULL makes them identical. So:

  * a J1939 value key that is ABSENT  -> NULL, no row anywhere (the omission contract);
  * a key that ARRIVES UNUSABLE       -> NULL in the column AND an ingest_unmapped
    row of kind 'field' naming it (wrong type, out of range, JSON null);
  * a REQUIRED key that is missing    -> the same ingest_unmapped row. Required means
    "every image the board has ever run sends it": frames, unknown, ring_full,
    ring_high, can_err, can_up, filt.
  * drains and can_last are NOT required. Image 19f0c022 -- the one on the tractor
    since 2026-09-22 -- predates them (they arrive with e82bcb7). A missing drains or
    can_last is therefore NULL with no unmapped row; a present-but-wrong one still
    goes to ingest_unmapped.

REVISED 2026-09-22 after the first field run of 19f0c022: CAN health counters and
board_id stored; silent coercion and log-and-NULL replaced with ingest_unmapped rows;
DM1 records whether the broker delivered it retained (ts_kind).
"""

# --- topic constants, alongside PLC4_TELEMETRY_TOPIC & co. -------------------

PLC4_ENGINE_TOPIC = "plc4/jd5055e/engine"

# DM1 is published per J1939 source address: plc4/jd5055e/dtc/<sa>. This is the one
# place these handlers need something dispatch() does not currently do, because
# dispatch() routes on an EXACT topic string and this leaf varies. See the
# registration note at the bottom.
PLC4_DTC_TOPIC_PREFIX = "plc4/jd5055e/dtc/"
PLC4_DTC_TOPIC = PLC4_DTC_TOPIC_PREFIX.rstrip("/")   # for log messages only

# Value columns of plc4_engine, in insert order. Every one is optional.
_PLC4_ENGINE_FIELDS = (
    "rpm",
    "load_pct",
    "torque_pct",
    "coolant_c",
    "oil_temp_c",
    "oil_kpa",
    "fuel_lph",
    "batt_v",
    "keysw_v",
    "hours",
)

# Plausibility bounds. A value outside these is stored as NULL with an
# ingest_unmapped row: it means the decode is wrong, not that the engine is
# remarkable. They are wide on
# purpose -- they catch a byte-order or scaling error, not a hot engine.
_PLC4_ENGINE_BOUNDS = {
    "rpm": (0.0, 5000.0),
    "load_pct": (0.0, 125.0),
    "torque_pct": (-125.0, 125.0),
    "coolant_c": (-40.0, 210.0),
    "oil_temp_c": (-40.0, 210.0),
    "oil_kpa": (0.0, 1000.0),
    "fuel_lph": (0.0, 500.0),
    "batt_v": (0.0, 60.0),
    "keysw_v": (0.0, 60.0),
    "hours": (0.0, 200000.0),
}

# --- ingest_unmapped ----------------------------------------------------------
#
# ASSUMED SHAPE -- NOT READ. The consumer already routes unknown topics into
# ingest_unmapped and (per the #25 hand-over) has a kind 'field'. This repository
# may not read /opt/plc1-monitoring, so the column list below is a placeholder.
# THE REVIEWER SHOULD REPLACE THIS FUNCTION'S BODY WITH THE CONSUMER'S EXISTING
# WRITER; every call site passes (topic, field, raw value, reason) and nothing else.
def _plc4_unmapped(cur, topic, field, raw, reason, log):
    log.warning("%s: %s=%r %s -- routed to ingest_unmapped", topic, field, raw, reason)
    cur.execute(
        """
        INSERT INTO ingest_unmapped (topic, kind, field, raw, reason)
        VALUES (%s, %s, %s, %s, %s)
        """,
        (topic, "field", field, repr(raw), reason),
    )


_MISSING = object()

UINT32_MAX = 0xFFFFFFFF

# Health keys. (payload key, column, kind, lo, hi, required)
#   kind "uint": a JSON integer (bool rejected -- True is an int in Python)
#   kind "sint": the same, signed
#   kind "bool": JSON true/false ONLY; 0/1 rejected, so a firmware that ever sends
#                an int here is caught rather than silently coerced
# Required = every image sends it, 19f0c022 included (git show 19f0c02, publishEngine).
_PLC4_ENGINE_HEALTH = (
    ("frames",    "frames",         "uint", 0, UINT32_MAX, True),
    ("unknown",   "unknown_frames", "uint", 0, UINT32_MAX, True),
    ("ring_full", "ring_full",      "uint", 0, UINT32_MAX, True),
    ("ring_high", "ring_high",      "uint", 0, 32,         True),   # ring is 32 slots
    ("can_err",   "can_err",        "uint", 0, UINT32_MAX, True),
    ("can_up",    "can_up",         "bool", None, None,    True),
    ("filt",      "filt",           "bool", None, None,    True),
    # From e82bcb7 on. 19f0c022 omits both: missing -> NULL, NO unmapped row.
    ("drains",    "drains",         "uint", 0, UINT32_MAX, False),
    ("can_last",  "can_last",       "sint", -2**31, 2**31 - 1, False),
)


def _plc4_health_value(cur, topic, payload, spec, log):
    key, _col, kind, lo, hi, required = spec
    raw = payload.get(key, _MISSING)
    if raw is _MISSING:
        if required:
            _plc4_unmapped(cur, topic, key, None, "required key missing", log)
        return None
    if kind == "bool":
        if isinstance(raw, bool):
            return raw
        _plc4_unmapped(cur, topic, key, raw, "is not JSON true/false", log)
        return None
    if isinstance(raw, bool) or not isinstance(raw, int):
        _plc4_unmapped(cur, topic, key, raw, "is not an integer", log)
        return None
    if not (lo <= raw <= hi):
        _plc4_unmapped(cur, topic, key, raw, "outside [%s, %s]" % (lo, hi), log)
        return None
    return raw


def _plc4_board_id(cur, topic, payload, log):
    raw = payload.get("id", _MISSING)
    if raw is _MISSING:
        return None
    if isinstance(raw, str) and raw:
        return raw
    _plc4_unmapped(cur, topic, "id", raw, "is not a non-empty string", log)
    return None


def _plc4_engine_number(cur, topic, payload, key, log):
    """Return a float for `key`, or None.

    None has two causes and they ARE distinguished (#25): the key was absent (the
    engine did not report it -- normal, nothing recorded), or it arrived unusable
    (not a number, or implausible, which means our decode is wrong) -- an
    ingest_unmapped kind 'field' row names it.
    """
    if key not in payload:
        return None
    raw = payload[key]
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        _plc4_unmapped(cur, topic, key, raw, "is not a number", log)
        return None
    val = float(raw)
    lo, hi = _PLC4_ENGINE_BOUNDS[key]
    if not (lo <= val <= hi):
        # A DECODE fault, not an engine fault: check the PGN byte offsets and scaling.
        _plc4_unmapped(cur, topic, key, raw,
                       "outside plausibility bounds [%s, %s] (decode fault)" % (lo, hi), log)
        return None
    return val


def handle_plc4_engine(cur, topic, payload, log):
    """plc4/jd5055e/engine -- 15 s, QoS 0, NOT retained.

    Absent keys are the normal case: with the key off, the engine ECU is not
    powered and the whole payload is counters only. That row is still worth
    storing, because `frames` not advancing is how we see the CAN side die.

    The row is always inserted, even when some keys went to ingest_unmapped: the
    other columns are still good, and a missing row would read as a publish gap.
    ts_kind stays 'unknown' -- this topic is never retained.
    """
    if not isinstance(payload, dict):
        log.warning("%s: payload is not an object (%r) -- dropped", topic, payload)
        return

    values = [_plc4_engine_number(cur, topic, payload, k, log) for k in _PLC4_ENGINE_FIELDS]
    health = [_plc4_health_value(cur, topic, payload, spec, log) for spec in _PLC4_ENGINE_HEALTH]
    board_id = _plc4_board_id(cur, topic, payload, log)

    cols = (*_PLC4_ENGINE_FIELDS, *(spec[1] for spec in _PLC4_ENGINE_HEALTH), "board_id", "ts_kind")
    cur.execute(
        "INSERT INTO plc4_engine (%s) VALUES (%s)"
        % (", ".join(cols), ",".join(["%s"] * len(cols))),
        (*values, *health, board_id, "unknown"),
    )


_LAMP_KEYS = ("mil", "rsl", "awl", "pl")


def _plc4_lamp(cur, topic, payload, key, log):
    """A J1939 lamp is two bits: 0 off, 1 on, 2 reserved, 3 not available.

    3 is stored as 3, never folded to "on" and never to NULL. An ECU that does
    not implement a lamp says 3 forever, and a dashboard that reads 3 as lit
    would show a permanent red stop lamp on a healthy tractor.

    Every image sends all four lamps, so a missing one is a firmware fault and is
    recorded in ingest_unmapped, as is anything outside 0-3 (#25).
    """
    raw = payload.get(key, _MISSING)
    if raw is _MISSING:
        _plc4_unmapped(cur, topic, key, None, "required lamp key missing", log)
        return None
    if isinstance(raw, bool) or not isinstance(raw, int) or not (0 <= raw <= 3):
        _plc4_unmapped(cur, topic, key, raw, "is not a 0-3 lamp state", log)
        return None
    return raw


def _plc4_dtc_sa(topic, payload, log):
    """Source address, from the topic leaf, cross-checked against the payload.

    The topic is authoritative: it is what the retained message is keyed on. A
    payload that disagrees is a firmware bug worth shouting about, not a reason to
    guess.
    """
    leaf = topic[len(PLC4_DTC_TOPIC_PREFIX):] if topic.startswith(PLC4_DTC_TOPIC_PREFIX) else ""
    try:
        sa = int(leaf)
    except ValueError:
        log.warning("%s: topic leaf %r is not a source address -- dropped", topic, leaf)
        return None
    if not (0 <= sa <= 255):
        log.warning("%s: source address %d out of range -- dropped", topic, sa)
        return None
    claimed = payload.get("sa")
    if isinstance(claimed, int) and not isinstance(claimed, bool) and claimed != sa:
        log.warning("%s: payload says sa=%d but the topic says %d -- trusting the topic",
                    topic, claimed, sa)
    return sa


def handle_plc4_dtc(cur, topic, payload, log, retained=False):
    """plc4/jd5055e/dtc/<sa> -- on change, QoS 1, RETAINED, ONE TOPIC PER ECU.

    `retained` is paho's msg.retain, passed through by dispatch(). It is stored in
    the EXISTING ts_kind column as 'retained' or 'live': a retained delivery is the
    broker replaying the last DM1 on subscribe/reconnect, not the ECU saying
    anything new, and without the flag every reconnect looks like a lamp event.

    DM1 is per source address. Four ECUs publish it on the 5055E and they disagree,
    so each gets its own retained topic and its own snapshot rows.

    A snapshot with n == 0 is stored like any other. It is the row that clears that
    ECU's fault display; skipping it as "empty" would leave a resolved fault on the
    dashboard until the next unrelated fault arrived.
    """
    if not isinstance(payload, dict):
        log.warning("%s: payload is not an object (%r) -- dropped", topic, payload)
        return

    sa = _plc4_dtc_sa(topic, payload, log)
    if sa is None:
        return

    codes = payload.get("dtc")
    if codes is None:
        codes = []
    if not isinstance(codes, list):
        _plc4_unmapped(cur, topic, "dtc", codes, "is not a list -- treated as no codes", log)
        codes = []

    n = payload.get("n", _MISSING)
    if n is _MISSING or not isinstance(n, int) or isinstance(n, bool) or n < 0:
        # n is NOT NULL in the schema, so the snapshot keeps len(codes); the
        # unmapped row is what says the ECU's own count never arrived.
        _plc4_unmapped(cur, topic, "n", None if n is _MISSING else n,
                       "is not a non-negative integer -- n derived from the code list", log)
        n = len(codes)
    elif n != len(codes):
        # n is the number of codes the BOARD KEPT, and it sends all of them, so a
        # mismatch is a malformed payload rather than a truncation. Truncation is
        # stated explicitly by `trunc`.
        log.warning("%s: n=%d but %d codes in the payload", topic, n, len(codes))

    trunc = payload.get("trunc", _MISSING)
    if trunc is _MISSING:
        trunc = None                      # images before the key existed
    elif not isinstance(trunc, bool):
        _plc4_unmapped(cur, topic, "trunc", trunc, "is not a boolean -- stored NULL", log)
        trunc = None

    lamps = [_plc4_lamp(cur, topic, payload, k, log) for k in _LAMP_KEYS]

    cur.execute(
        """
        INSERT INTO plc4_dtc_snapshot (sa, n, mil, rsl, awl, pl, trunc, ts_kind)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s)
        RETURNING id
        """,
        (sa, n, *lamps, trunc, "retained" if retained else "live"),
    )
    snapshot_id = cur.fetchone()[0]

    for i, code in enumerate(codes):
        if not isinstance(code, dict):
            _plc4_unmapped(cur, topic, "dtc[%d]" % i, code, "is not an object -- code not stored", log)
            continue
        spn, fmi, oc = code.get("spn"), code.get("fmi"), code.get("oc")
        # spn and fmi are NOT NULL: a code without them cannot be stored, so the
        # whole entry goes to ingest_unmapped rather than vanishing.
        if not isinstance(spn, int) or isinstance(spn, bool) or not (0 <= spn <= 524287):
            _plc4_unmapped(cur, topic, "dtc[%d].spn" % i, code,
                           "is not a 19-bit SPN -- code not stored", log)
            continue
        if not isinstance(fmi, int) or isinstance(fmi, bool) or not (0 <= fmi <= 31):
            _plc4_unmapped(cur, topic, "dtc[%d].fmi" % i, code,
                           "is not 0-31 -- code not stored", log)
            continue
        if not isinstance(oc, int) or isinstance(oc, bool) or not (0 <= oc <= 127):
            # occurrence count is the least important field; keep the code, record why oc is NULL
            _plc4_unmapped(cur, topic, "dtc[%d].oc" % i, oc, "is not 0-127 -- oc stored NULL", log)
            oc = None
        cur.execute(
            """
            INSERT INTO plc4_dtc_code (snapshot_id, spn, fmi, oc)
            VALUES (%s,%s,%s,%s)
            """,
            (snapshot_id, spn, fmi, oc),
        )


# --- registration ------------------------------------------------------------
#
# In dispatch(), alongside the existing PLC4 handlers. Order does not matter here:
# unlike `cal`, nothing in these two topics primes state the other needs.
#
#     elif topic == PLC4_ENGINE_TOPIC:
#         handle_plc4_engine(cur, topic, payload, log)
#     elif topic.startswith(PLC4_DTC_TOPIC_PREFIX):
#         handle_plc4_dtc(cur, topic, payload, log, retained=msg.retain)
#
# `msg.retain` is paho's MQTTMessage.retain. dispatch() must be handed it (or the
# msg) from on_message; if dispatch() currently receives only topic and payload,
# that signature gains one argument. The engine topic is never retained and does
# not need it.
#
# THE SECOND LINE IS NOT THE HOUSE PATTERN AND THAT IS DELIBERATE. Every other
# topic in this consumer dispatches on an exact string. DM1 cannot: the source
# address is in the leaf, the set of ECUs is discovered at runtime, and on the
# 5055E there are four of them. A prefix test is the smallest departure that
# works. If the reviewer would rather keep dispatch() exact, the alternative is to
# enumerate the known source addresses as constants -- which silently drops any
# ECU nobody thought of, on a machine we have not yet captured.
#
# No subscription change is needed: the consumer already subscribes plc4/#.
#
# RITUAL: tests/run-ritual.sh requires every file in tests/ to be listed in
# docs/DESIGN-STANDARD.md section 4, and every section 4 member to have a file.
# A test for these handlers must join section 4 in the same change, or the
# ritual jams. ingest/test_plc4_engine_handlers.py is the proposed test; it runs
# here against a stub cursor and needs porting to the consumer's real-Postgres
# harness (scripts/run-consumer-tests.sh) together with its section 4 entry.
