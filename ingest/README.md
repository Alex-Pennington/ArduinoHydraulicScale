# ingest/ — proposals for the consumer side. Nothing here is deployed.

These three files describe how J1939 engine telemetry from the tractor would land in
the existing stack. **None of them has been applied, and none of them is this
repository's to apply.** The consumer, its migrations and its dashboards live in
`/opt/plc1-monitoring` and belong to the consumer coordinator. This directory exists so
that hand-over is a review rather than a retelling.

| File | What it is | Where it goes |
|---|---|---|
| `067_add_plc4_engine.sql` | `plc4_engine`, `plc4_dtc_snapshot`, `plc4_dtc_code`, `v_plc4_dtc_active` | `db/migrations/` |
| `consumer_plc4_engine.py` | Two handlers plus the `dispatch()` lines that route to them | merged into `consumer/plc1_consumer.py` |
| `plc4_engine_dashboard.json` | Eight panels against the tables above | `grafana/provisioning/dashboards/` |

## What was actually checked

The migration was applied to a **throwaway PostgreSQL 16 cluster** created under this
job's temp directory — not to `plc1`, and not to any container. It ran to `COMMIT` with
`ON_ERROR_STOP=1`. Against that scratch schema:

- `v_plc4_dtc_active` was exercised both ways: a no-fault snapshot yields one row with a
  NULL `spn`, and a two-code snapshot yields two rows. That is the behaviour the view's
  comment claims, so the dashboard can distinguish "no faults" from "no data".
- Every one of the eight dashboard panels' `rawSql` was run through `EXPLAIN` (with
  Grafana's `$__timeFilter` macro substituted). All eight planned.
- The handlers were driven through seven cases with a stub cursor: key-off with every
  value absent, a running engine, an implausible rpm, a no-fault DM1, a two-fault DM1, a
  truncated DTC list, and a malformed SPN. Each behaved as its docstring says.

The handlers were **not** run against a real database, because `psycopg2` is not
installed outside the consumer's container and installing it was out of scope.

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

## Three things the reviewer should push back on if they disagree

1. **The validity contract is inverted relative to the other PLC4 topics.** `plc4_engine`
   omits absent keys (PLC1/PLC3 style) instead of always populating with validity in
   sibling strings (PLC4 style). The reasoning is in `REPORT.md` §7: a J1939 parameter the
   engine never broadcasts is absent, not suspect. If the estate would rather be
   internally consistent than locally correct, this is the decision to revisit — but the
   handlers would need rewriting, not just the schema.
2. **The migration number is a guess about timing.** 066 was the highest on 2026-09-18.
   If anything lands first, renumber.
3. **A handler test must join `docs/DESIGN-STANDARD.md` §4 in the same change**, or
   `tests/run-ritual.sh` jams on the coverage gate. No test is written here, because it
   belongs in the consumer's tree with the rest of them.
4. **The DTC handler needs a prefix match in `dispatch()`**, not the exact topic string
   every other handler uses, because the source address is in the topic leaf and the set
   of ECUs is discovered at runtime. The alternative — enumerating known source addresses
   as constants — silently drops any ECU nobody anticipated, on a machine we have not
   captured yet. Flagged because it is a real departure from the house pattern.
