# ingest/ — proposals for the consumer side. Nothing here is deployed.

These files describe how J1939 engine telemetry from the tractor would land in
the existing stack. **None of them has been applied, and none of them is this
repository's to apply.** The consumer, its migrations and its dashboards live in
`/opt/plc1-monitoring` and belong to the consumer coordinator. This directory exists so
that hand-over is a review rather than a retelling.

**Handed over 2026-09-20 (issue #25).** The consumer renumbers the migration to **070** on
their side (067-069 are taken there). The file keeps the name `067` here so the hand-over
trail still matches.

| File | What it is | Where it goes |
|---|---|---|
| `067_add_plc4_engine.sql` | `plc4_engine`, `plc4_dtc_snapshot`, `plc4_dtc_code`, `plc4_alert_threshold`; views `v_plc4_dtc_active`, `v_plc4_can_health`, `v_plc4_engine_now`, `v_plc4_dm1_now` | `db/migrations/` as **070** |
| `consumer_plc4_engine.py` | Two handlers, the `dispatch()` lines that route to them, and an `ingest_unmapped` writer whose column list is **assumed** and must be swapped for the consumer's own | merged into `consumer/plc1_consumer.py` |
| `test_plc4_engine_handlers.py` | Eight handler cases on a stub cursor, plus a reboot SQL fixture for the views | ported into the consumer's tests **with** its `DESIGN-STANDARD.md` §4 entry |
| `plc4_engine_dashboard.json` | 23 panels: an operator band, trends, fault tables, and two collapsed diagnostic rows (CAN health, filter check) | `grafana/provisioning/dashboards/` |

## What was actually checked

Re-checked 2026-09-22 against the revised files. **All of it ran on scratch copies on
Saga. None of it ran against `plc1`, the consumer container or any field data.** The field
rows exist only as MQTT messages relayed by the field laptop; the fixtures below copy
their values by hand.

- **The migration** was applied to a throwaway PostgreSQL 16 cluster with
  `ON_ERROR_STOP=1` and ran to `COMMIT`. A stand-in `ingest_unmapped` table was created
  first. Its columns are a guess, because this repository may not read the consumer's.
- **The views were run against fixture rows shaped like the 19f0c022 field messages.**
  The rows were key-on (no rpm), crank (161 rpm, 0 kPa), two running rows (no rpm, 392 kPa,
  14.25 V), a counter reset, a quiet bus, `can_up` false, a 60 s gap, and then two
  e82bcb7-shaped rows. The per-row states came out as 2, 8, 5, 7, 7, 2, 3, 1, 2, 4.
  `running_confirmed` was true only on the second running row, so the crank row did not
  count. The deltas and rates were NULL on the first row, the reset row and the gap row.
  `fps_board` and `drains_ps` were NULL on the 19f0c022-shaped rows and 460 and 200 on
  the later ones. With an empty table `v_plc4_engine_now` still returned one row, reading
  BOARD SILENT. With `oil_kpa_low` set to 400 and two running rows at 392 kPa, it read
  `RUNNING - LOW OIL`. With the limit NULL, `oil_low_running` stayed NULL.
- **`v_plc4_dm1_now`** listed SA 3, 6, 23 and 71 when no DM1 had arrived. It added an
  unexpected SA 99 when that one sent DM1. It counted only lamp value 1 as lit.
- **Plans were checked on 100,000 synthetic rows.**
  `v_plc4_can_health WHERE recv_ts > now() - interval '5 min'` used
  `idx_plc4_engine_recv` for both the outer rows and the lateral predecessor. It touched
  19 rows in 0.3 ms. `v_plc4_engine_now` first seq-scanned to find the last hours value
  when none existed, so the migration now adds the partial index
  `idx_plc4_engine_hours_recv`.
- **All 24 dashboard queries were run** (23 panel targets plus the annotation), with
  Grafana's `$__timeFilter`, `$__timeFrom`, `$__timeTo` and `$__interval_ms`
  substituted. Every one returned rows of the expected shape. **No Grafana instance loaded
  the JSON.** Panel options, mappings and overrides were written from Grafana's
  schemaVersion 39 model and never rendered.
- **Handlers:** `python3 ingest/test_plc4_engine_handlers.py` passes 8 of 8 on a stub
  cursor. The INSERTs from three engine payloads and one malformed DM1 were then replayed
  into the scratch schema. Every column name and type was accepted. A 19f0c022-shaped
  payload stored `drains` and `can_last` as NULL and wrote no unmapped row. The reboot
  fixture produced `reboot` true, `d_frames` NULL and state 2. The consumer has said a
  stub-cursor result is not evidence on its side (#25). These cases are the list to port
  into `scripts/run-consumer-tests.sh`.
- **Filter prediction:** the host replay (`test/test_j1939_replay.cpp`) was re-run on
  `candump-2026-09-22_175858.log`. The filter would pass 10,477 of 35,793 frames (140 fps).
  8,556 of those are used by the decoder, which leaves 0.18 unused. It drops 0 used
  frames. That is a host prediction only. The board has never run with `filt` true.

## What changed after checking against real 5055E data

These files were revised once Cal Poly's MIT-licensed 5055E CAN corpus was checked
(`findings/2026-09-18-calpoly-can-corpus.md`). Three changes matter to the reviewer:

- **`plc4_dtc_snapshot` gained an `sa` column**, and the topic became
  `plc4/jd5055e/dtc/<sa>`. DM1 is per ECU: four source addresses publish it on this model
  and they disagree.
- **`v_plc4_dtc_active` uses `DISTINCT ON (sa)`**, not a single global latest row. The
  earlier version would have shown one ECU and silently dropped three.
- **`keysw_v` (SPN 158) is the battery voltage to plot**, not `batt_v` (SPN 168), which
  was not-available in all 1206 corpus frames. The dashboard panel leads with it.

## Changed 2026-09-22, after the first field run

Image 19f0c022 ran on the tractor on 2026-09-22 and published over MQTT. What it sent
changed these files. The design behind the changes was judged against three candidates.
The winner was the one that reads correctly on the image now on the tractor.

- **`plc4_engine` gained the CAN health counters the board already sends.** These are
  `ring_full`, `ring_high`, `can_err`, `can_up`, `filt` and `board_id` (the payload key
  `id`), plus `drains` and `can_last`. **The flashed image does not send `drains` or
  `can_last`** (`git show 19f0c02`, `publishEngine`). They arrive with e82bcb7, so they
  are NULL on every 19f0c022 row, and the handler treats them as optional. The other
  seven keys are required, because every image sends them.
- **#25's condition is now met.** A key that arrives unusable goes to `ingest_unmapped`
  as kind `field`, and every nullable column's comment says which kind of absence its
  NULL means. Two silent paths are gone: the old handler turned a non-integer `frames`
  into NULL, and logged-and-NULLed an implausible J1939 value. An absent J1939 key is
  still just NULL. The omission contract is unchanged.
- **The `frames` and `unknown_frames` comments were wrong.** They said "lifetime". Both
  are uint32 counts since boot. They reset on every reboot and wrap after about 107 days
  at the ~465 fps measured here. The views read a decrease as a reboot.
- **DM1 now records the MQTT retain flag in the existing `ts_kind` column** (`retained` or
  `live`). Without it, a broker replaying the retained DM1 on reconnect would look like a
  new lamp event. This needs `dispatch()` to pass `msg.retain`. If `ts_kind` has a CHECK
  constraint on the consumer side, the two values must be added to it. That constraint
  was not read.
- **There are three new views, so tiles and alerts share one definition.**
  `v_plc4_can_health` gives per-interval deltas, rates, reboot detection and per-row
  state. It finds the previous row with a LATERAL predecessor, not `LAG()`, so alert
  queries use the index. `v_plc4_engine_now` always returns exactly one row, and every
  stat tile and alert reads it. `v_plc4_dm1_now` returns one row per ECU, and the expected
  SAs are always listed.
- **`plc4_alert_threshold` is new, and all its limits ship NULL.** The limits are
  low oil, high coolant and the minimum charging voltage. No John Deere figure is known,
  and a NULL limit fails closed. The 100/110 C coolant lines and the 100 kPa oil colour
  step in the first dashboard had no source, so they are gone.
- **The dashboard was rebuilt.** The operator band tells three cases apart on every tile:
  board silent (NO DATA), not reported, and a real 0. A coverage gauge is the acceptance
  check after flashing the next flashed image (after 705bf3c). On 19f0c022, rpm appeared in 0 of 8 key-on messages,
  hours in 4 of 8 and fuel in 6 of 8. Two collapsed rows carry the CAN diagnostics and
  the evidence for the first `canf` trial. The uid and title are unchanged.
- **Alert queries for #26** are written as comments at the end of the migration. No alert
  rules were provisioned. The rules for red stop (one instance per ECU), board silent,
  CAN down, drain stalled, lossy and low oil can page now. The rule for "bus quiet with
  the key on" is written against `key_on_evidence`, which stays NULL until BRIEF Q8
  (switched or maintained board power) is answered. Until then it returns no rows,
  instead of paging at every shutdown.

**Still open:**

- BRIEF Q8, above. It has to go to Alex through the coordinator. Nothing here asks it.
- The state cut points (400 rpm, 100 kPa, 0.5 L/h, 13.5 V) are placeholders taken from
  values seen on this tractor. They are not JD figures.
- The "Faults" tile deviates from the design on one point. The design ranked
  "LAMP LIT, CODE WITHHELD" below MIL. That branch could never be reached, because a lit
  lamp always matches an earlier branch. It is shown instead as a `, CODE WITHHELD`
  suffix on the lamp state.
- `_plc4_unmapped()` guesses the `ingest_unmapped` column list. Swap it for the
  consumer's own writer.

## Things the reviewer should push back on if they disagree

1. **The validity contract is inverted relative to the other PLC4 topics.** `plc4_engine`
   omits absent keys (PLC1/PLC3 style) instead of always populating with validity in
   sibling strings (PLC4 style). The reasoning is in `REPORT.md` §7: a J1939 parameter the
   engine never broadcasts is absent, not suspect. If the estate would rather be
   internally consistent than locally correct, this is the decision to revisit — but the
   handlers would need rewriting, not just the schema.
2. **The migration number.** Settled: the consumer renumbers it to 070 (067-069 are taken
   there). It stays `067` in this directory.
3. **A handler test must join `docs/DESIGN-STANDARD.md` §4 in the same change**, or
   `tests/run-ritual.sh` jams on the coverage gate. `test_plc4_engine_handlers.py` here
   is the case list, not the test. The real one belongs in the consumer's tree and runs
   against their throwaway Postgres.
4. **The DTC handler needs a prefix match in `dispatch()`**, not the exact topic string
   every other handler uses, because the source address is in the topic leaf and the set
   of ECUs is discovered at runtime. The alternative — enumerating known source addresses
   as constants — silently drops any ECU nobody anticipated, on a machine we have not
   captured yet. Flagged because it is a real departure from the house pattern.
