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

# Plausibility bounds. A value outside these is stored as NULL and logged: it
# means the decode is wrong, not that the engine is remarkable. They are wide on
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


def _plc4_engine_number(payload, key, log):
    """Return a float for `key`, or None.

    None has two causes and they are not distinguished here on purpose: the key
    was absent (the engine did not report it), or the value was implausible (our
    decode is wrong). The second is logged; the first is normal.
    """
    if key not in payload:
        return None
    raw = payload[key]
    if raw is None:
        return None
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        log.warning(
            "%s: %s=%r is not a number -- storing NULL", PLC4_ENGINE_TOPIC, key, raw
        )
        return None
    val = float(raw)
    lo, hi = _PLC4_ENGINE_BOUNDS[key]
    if not (lo <= val <= hi):
        log.warning(
            "%s: %s=%r outside [%s, %s] -- storing NULL. This is a DECODE fault, "
            "not an engine fault: check the PGN byte offsets and scaling.",
            PLC4_ENGINE_TOPIC,
            key,
            val,
            lo,
            hi,
        )
        return None
    return val


def handle_plc4_engine(cur, topic, payload, log):
    """plc4/jd5055e/engine -- 15 s, QoS 0, NOT retained.

    Absent keys are the normal case: with the key off, the engine ECU is not
    powered and the whole payload is counters only. That row is still worth
    storing, because `frames` not advancing is how we see the CAN side die.
    """
    if not isinstance(payload, dict):
        log.warning("%s: payload is not an object (%r) -- dropped", topic, payload)
        return

    values = [_plc4_engine_number(payload, k, log) for k in _PLC4_ENGINE_FIELDS]

    frames = payload.get("frames")
    unknown = payload.get("unknown")
    frames = int(frames) if isinstance(frames, int) and not isinstance(frames, bool) else None
    unknown = int(unknown) if isinstance(unknown, int) and not isinstance(unknown, bool) else None

    cur.execute(
        """
        INSERT INTO plc4_engine (
            rpm, load_pct, torque_pct, coolant_c, oil_temp_c,
            oil_kpa, fuel_lph, batt_v, keysw_v, hours,
            frames, unknown_frames, ts_kind
        ) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
        """,
        (*values, frames, unknown, "unknown"),
    )


_LAMP_KEYS = ("mil", "rsl", "awl", "pl")


def _plc4_lamp(payload, key, log):
    """A J1939 lamp is two bits: 0 off, 1 on, 2 reserved, 3 not available.

    3 is stored as 3, never folded to "on" and never to NULL. An ECU that does
    not implement a lamp says 3 forever, and a dashboard that reads 3 as lit
    would show a permanent red stop lamp on a healthy tractor.
    """
    raw = payload.get(key)
    if raw is None:
        return None
    if isinstance(raw, bool) or not isinstance(raw, int) or not (0 <= raw <= 3):
        log.warning("%s: %s=%r is not a 0-3 lamp state -- storing NULL", PLC4_DTC_TOPIC, key, raw)
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


def handle_plc4_dtc(cur, topic, payload, log):
    """plc4/jd5055e/dtc/<sa> -- on change, QoS 1, RETAINED, ONE TOPIC PER ECU.

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
        log.warning("%s: dtc is not a list (%r) -- treating as no codes", topic, codes)
        codes = []

    n = payload.get("n")
    if not isinstance(n, int) or isinstance(n, bool):
        log.warning("%s: n=%r is not an integer -- deriving from the code list", topic, n)
        n = len(codes)
    elif n != len(codes):
        # The board truncates at MAX_DTC. n is what the ECU reported; the list is
        # what fitted. Keep n as reported so the truncation is visible.
        log.warning(
            "%s: n=%d but %d codes in the payload -- the board truncated the list",
            topic,
            n,
            len(codes),
        )

    lamps = [_plc4_lamp(payload, k, log) for k in _LAMP_KEYS]

    cur.execute(
        """
        INSERT INTO plc4_dtc_snapshot (sa, n, mil, rsl, awl, pl, ts_kind)
        VALUES (%s,%s,%s,%s,%s,%s,%s)
        RETURNING id
        """,
        (sa, n, *lamps, "unknown"),
    )
    snapshot_id = cur.fetchone()[0]

    for code in codes:
        if not isinstance(code, dict):
            log.warning("%s: code entry %r is not an object -- skipped", topic, code)
            continue
        spn, fmi, oc = code.get("spn"), code.get("fmi"), code.get("oc")
        if not isinstance(spn, int) or isinstance(spn, bool) or not (0 <= spn <= 524287):
            log.warning("%s: spn=%r is not a 19-bit SPN -- code skipped", topic, spn)
            continue
        if not isinstance(fmi, int) or isinstance(fmi, bool) or not (0 <= fmi <= 31):
            log.warning("%s: fmi=%r is not 0-31 -- code skipped", topic, fmi)
            continue
        if not isinstance(oc, int) or isinstance(oc, bool) or not (0 <= oc <= 127):
            oc = None  # occurrence count is the least important field; keep the code
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
#         handle_plc4_dtc(cur, topic, payload, log)
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
# ritual jams.
