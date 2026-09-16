# 4-20 mA loop: deployment runbook

Written 2026-09-13 for **the agent that installs the firmware**, which Alex has said will also
perform the migrations. Everything here was verified read-only on 2026-09-13; anything that was
not is labelled.

Companion to `notes/2026-09-13-4-20ma-loop-implementation.md` (what was built and why).

**Nothing in this runbook has been executed.** No migration has been applied, no consumer
deployed, no board flashed.

---

## 0a. THE MACHINE IS BENCHED, AND THAT CHANGES THE URGENCY

**Alex, 2026-09-16: "I'll be installing the hardware in the next few days, currently the system
is unusable with the current sensor so we will be benching the plc4 for upgrades."**

So there is **no running machine to protect** during this deployment, and **no working scale to
regress** — the fitted 0.5-4.5 V transducer is already unusable. That is worth knowing in both
directions:

- It removes the time pressure. Nothing is weighing today, so getting the order right matters
  more than getting it done quickly.
- It is independent corroboration of the contradiction in §3 of the decisions note. The reason
  the machine is unusable is the same reason `2828 psi` and "it rails routinely" could not both
  be true: that sensor is not reporting usefully. Nothing here settles WHY — the new firmware's
  `overrange` / `overrange_max` counters are still what will.

The consumer-side precondition below is unchanged by this. A benched machine does not publish,
so nothing is being lost while it is down — but the moment it comes back, the first over-range
excursions are the measurement this whole change exists to produce, and they are discarded
silently if the consumer has not landed first.

---

## 0. THE ORDERING CONSTRAINT THAT BITES FIRST

**Deploying the new consumer BEFORE migration 061 makes the consumer refuse to start.** Not
degrade — refuse.

`consumer/plc1_consumer.py` builds its required-column set by PARSING its own INSERT statements
(`required_columns()`), and `Store.require_schema()` asserts at start-up that every one of them
exists, raising `RuntimeError` — explicitly "not a transient condition" — so `main()` exits
rather than ingesting. The new statements name the new columns:

    PLC4_TELEMETRY_SQL  ... (w, psi, psi_fault, ma, counts, sensor, queued, valid, ts_kind)
    PLC4_STATUS_SQL     ... (..., overrange, overrange_max, underrange, ts_kind)

So `psi_fault`, `ma` and `underrange` are **start-up requirements**, not optional extras.

This fails SAFE — a loud exit rather than a wrong ingest — but it takes the estate's only ingest
down until the migration lands. **Migration first, consumer second. Always.**

---

## 1. PRECONDITION: PR #11 must be merged

`Alex-Pennington/greybeard-plc-consumer` PR #11 carries both the consumer change and migration
061. **Alex ruled 2026-09-13 that it is merged BEFORE anything is applied to production**, so
that no host ever runs a migration existing in no merged commit.

This matters more than usual because `db/migrations/README.md` rule 2 forbids editing a migration
that has been applied ANYWHERE. Apply from an unmerged branch and a later review change can only
be corrected by a 062.

Pre-merge review returned DO NOT MERGE on three items; all were cleared in `7c1353e` and all four
gates pass. **Do not apply anything until the PR is merged.**

---

## 2. THE MIGRATION IS NAMED `061_add_plc4_six_state_sensor`

**It was renamed.** Earlier references — including the body of consumer issue #12 — say
`061_add_plc4_five_state_sensor`. That name is dead. The file, the ledger version string and the
postconditions manifest row all say **six**.

### Use `--only`, NEVER `--apply --expect N`

Measured 2026-09-13 with the script's own read-only plan mode:

| host | outstanding per plan | `/opt/plc1-monitoring` |
|---|---|---|
| Saga | **19** | git checkout, was at `6a41841` |
| farm1 | **26** | **root-owned, NOT a git repo** |

Those counts are not a backlog to work through. `scripts/apply-migrations.sh`'s own header
records why: the ledger begins at 021 and the pre-021 set was applied and never recorded. It
also records that on 2026-09-06 one migration was authorised and `--apply` would have run
twenty-one. **`--apply --expect 19` or `--expect 26` would be catastrophic and is not what is
wanted here.**

    cd /opt/plc1-monitoring
    ./scripts/apply-migrations.sh                                     # plan; changes nothing
    ./scripts/apply-migrations.sh --only 061_add_plc4_six_state_sensor

Run it **on each host** — the script drives `docker exec` against the LOCAL database container,
so it cannot be pointed at a remote host.

The migration is additive and idempotent: three `ADD COLUMN IF NOT EXISTS`, five `COMMENT ON`, one
`INSERT ... ON CONFLICT DO NOTHING`, all in one transaction. A reviewer applied it three times to
a scratch database: rc=0 each time, one ledger row.

### Verify, per host

    SELECT column_name FROM information_schema.columns
     WHERE table_name='plc4_telemetry' AND column_name IN ('psi_fault','ma');    -- expect 2 rows
    SELECT column_name FROM information_schema.columns
     WHERE table_name='plc4_status' AND column_name='underrange';                -- expect 1 row
    SELECT version FROM schema_migration WHERE version='061_add_plc4_six_state_sensor';

---

## 3. WIDEN THE REPLICATION SPEC — consumer issue #12

`scripts/replicate-host.sh` names plc4 columns **explicitly** (around `:349-351`) and is a
hardcoded per-table allowlist. Until it names `psi_fault`, `ma` and `underrange`, those columns
populate on farm1 and stay **silently NULL on Saga** — no error, no warning. The script's own
header describes that exact defect costing two days on `variant`.

Do this only AFTER step 2 has succeeded on BOTH hosts. A premature widening is no longer a run
failure — a preflight added since September turns it into a named per-table SKIP — but a skip
still stops plc4 replication until the columns exist.

---

## 4. REBUILD THE CONSUMER — a REBUILD, not a restart

    docker compose up -d --build consumer

`plc1_consumer.py` is **COPYed into the image**. A plain restart runs the OLD code against the NEW
schema, which is the one combination nobody wants: the old code still decides validity by string
equality to `"ok"` and would discard every `SATURATED` reading.

Verify it actually started, rather than assuming: `require_schema()` exits the process on a
missing column, so a container that is not running means step 2 did not land.

**UNVERIFIED BY ME:** whether farm1's consumer is managed by that same `docker compose`
invocation. farm1's `/opt/plc1-monitoring` is root-owned and is not a git checkout, so its deploy
mechanism is not the one Saga uses and I did not establish what it is. **Establish that before
running anything on farm1.** There is a `deploy-service.git` in `/opt/repos` on Saga that may be
the answer.

---

## 4a. THE NEW `stale` FIELD ON TELEMETRY — read this before subscribing to the live feed

Firmware commit `7e291cf` (branch `feat/plc4-mcp3421-lcd2004`, not yet the branch of record named
in §5 below) adds a bare JSON boolean `stale` to the periodic `telemetry` payload, sitting right
after `sensor`:

    {"w":1834,"psi":812,"mA":11.4,"counts":4021,"sensor":"ok","stale":true,"psi_fault":null,"queued":0,"id":"..."}

`stale` is true whenever the board has gone `READING_STALE_MS` (2000 ms) without a successful read
from the MCP3421 — a loose Qwiic connector, most likely. `sensor` keeps meaning the LOOP state
(`ok`/`NO SENSOR`/`UNDER ZERO`/`SATURATED`/`XMTR FAULT`/`BAD STATE`, unchanged, see below) and is
**not** overridden when the ADC goes stale, so `sensor` can read `"ok"` — the last loop state it
actually measured — while `stale` is `true` and every field around it (`w`, `psi`, `mA`, `counts`)
is a frozen value from before the ADC stopped answering, not a live one.

**A consumer that ignores `stale` will bank a frozen weight as live.** The firmware itself refuses
to let `+` record while stale, and blanks the weight on the LCD — but neither of those protections
reaches a consumer that is only subscribed to the `telemetry` stream. Any code that reads this
payload must check `stale` before treating `w` (or `psi`, or `mA`) as a current measurement, the
same way it already checks `sensor` before treating a reading as trustworthy.

**The six `sensor` wire values are UNCHANGED** — `ok`, `NO SENSOR`, `UNDER ZERO`, `SATURATED`,
`XMTR FAULT`, `BAD STATE` (`loopStateName()` in `include/loop_scaling.h`, untouched by this
branch). Migration 061's enum and the consumer's existing `sensor`-based validity check both keep
working with no migration and no consumer change required for this field alone — `stale` is
additive. (Landing it as a real, non-NULL column, rather than reading it only off the live MQTT
stream, is a schema change nobody has scoped yet — not covered by migration 061.)

The retained `status` payload now carries the same `stale` field, same form, same place — right
after `sensor`. It was the more urgent gap of the two: `status` is published RETAINED, so before
this fix a board whose ADC died could publish `"sensor":"ok"` once and the broker would hand that
claim to every subscriber that connected afterward, indefinitely, with no ADC fault visible except
by inferring it from `i2c_err`. **Because `status` is retained, `stale:true` there is what a
subscriber gets on connect if the board's ADC had already failed before it subscribed** — the
consumer must treat that the same way it treats `stale:true` on `telemetry`, not as a one-off.

---

## 5. ONLY THEN, THE FIRMWARE

- **Do not flash before the transmitter is physically fitted.** PLC #3's playbook §2 makes this a
  procedure rather than a commit message, because it cost them a morning.
- **The burden resistor is load-bearing.** With it open or unfitted, A2 floats and the old
  mid-rail defect returns exactly as it was. Firmware cannot detect that.
- **Updated 2026-09-16: the burden is 68 ohm, 0.1%, <=25 ppm/degC — not the 150 ohm of the
  original 4-20 mA design.** The loop is now read by an MCP3421 on its own 2.048 V reference,
  not the board's 5 V rail. **Do not fit 150 ohm here.** Against 2.048 V, 150 ohm clips at
  13.65 mA — BELOW the 20 mA span end — so the fault is silent: the transmitter simply looks
  like it never exceeds about two-thirds of full scale, with no error anywhere to point at the
  resistor. That is a hard failure mode to diagnose from the symptom alone.
- **Check the old harness for oil** before reuse — the previous transducer leaked through its
  cable entry, and capillary action carries oil to whatever the far end is.
- **The board will demand a full recalibration.** `EEPROM_SIGNATURE` moved to `PL4A`, so the
  voltage-era `tareValue` and `calFactor` are refused rather than silently used. `CAL NOT SET!` on
  the LCD is expected, not a fault — tare and `kv` at the boom mark before it will weigh.
- Firmware branch of record: **`feat/plc4-mcp3421-lcd2004`**, also on the `saga` remote at
  `/opt/repos/greybeard-plc4-tractor-scale.git`. Compiles at 91232 flash / 13472 RAM via
  `tools/build-check.sh`.

---

## 5a. BENCH CHECK FOR `NO ADC`

A minimal check that the ADC-staleness path (§4a) actually behaves as described, once the board
is flashed and running with the loop live:

1. With the transducer loop live and the board reading normally, unplug the Qwiic connector
   between the board and the MCP3421.
2. Confirm the weight on the LCD blanks and `NO ADC` appears on the status row, after a roughly
   2-second delay (not instantly).
3. Confirm `+` is refused while `NO ADC` is showing.
4. Reconnect the Qwiic connector and confirm the display recovers on its own.

This is a bench check of the firmware behavior only — it is not a commissioning test, and no
commissioning procedure is scoped here. That is Alex's to run.

---

## 6. WHAT NOT TO DO

- Do not `--apply --expect` anything on either host. See §2.
- Do not edit migration 061 after applying it anywhere — `db/migrations/README.md` rule 2. Write a
  062.
- Do not restart the consumer in place of rebuilding it. See §4.
- Do not treat the Grafana dashboard as correct — it still teaches the retired vocabulary and has
  `SENSOR SHORT` and `NO SENSOR` exactly backwards. Consumer issue #13.
- Do not skip §3 because the firmware works without it. It fails silently, which is the whole
  reason it is written down.
