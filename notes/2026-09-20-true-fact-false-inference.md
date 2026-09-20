# A true fact carrying a false inference

2026-09-20. Four times in one afternoon, between two coordinator sessions working
PLC #4 and the estate consumer, a **verified fact** was used to support a
**conclusion it did not support**. Written down because the pattern is more
useful than any of the four, and because none of them was caught the way people
assume errors are caught.

## The four

| # | Claim | The part that was true | The part that was not |
|---|---|---|---|
| 1 | 4,872 NULL rows prove the consumer's handler has survived a wire-null `w` | the count, exactly | the causation — the handler nulled them itself, and has never been handed a wire null |
| 2 | A fall in the counter is explained by `reset` and `uptime_s` in the same frame | true of `status`, which is why `queueDropped` is safe | inferred onto `telemetry`, which carries neither field |
| 3 | The 964 s maximum gap is the farm1 outage | it is the maximum gap | wrong event — it closed the previous day, and the current outage cannot appear at all |
| 4 | Fourteen days of weight rows show the replay path has never run | the window was accurate | quoted as all history; one backlog row from 2026-09-01 sat outside it |

Three were PLC #4's, one the consumer's.

## What they share

**Every one is a verified fact carrying an unverified inference.**

This is the failure that survives checking, because **checking the fact passes**.
The error is one level up, in the step from the fact to what it is taken to
mean, and nothing about re-reading the sentence exposes it. Claim 2 is the
clearest: the sentence was accurate as written, and would pass any review that
asked "is this true?"

## The half that is operationally useful

**Not one of the four was caught by its author re-reading. All four were caught
by the other party going and looking.**

That is an argument for the cross-check itself, not for either party being more
careful. Care is what produced the verified facts in the first place; it did not
and could not produce the correction. What corrected each one was somebody with
a different vantage point running a query the author had not thought to run.

## A structural sub-case worth knowing on its own

Claim 3 is not only a misattribution — it names a property of a whole class of
statistic.

**An open-ended interruption cannot appear in any statistic computed between two
rows.** A gap needs a row on both sides. `lag()` has nothing to subtract from
while the outage is still running, so the largest interruption in the data
contributes *nothing* to `max(gap)` — the statistic meant to find interruptions
is structurally blind to the one you are in. It appears only once it has ended.

The same shape had already appeared that morning in the CAN soak test: **gap
detection is blind to loss at the head and tail of a window**, for the same
reason, and it cost 51 frames invisibly.

**The immune form is `now() - max(recv_ts)`** — how old is the newest row — which
sees an outage you are still inside. The estate's alarm gates were measured and
all use it:

```
scripts/check-plc-staleness.sh       now() - max(recv_ts)
scripts/check-ingest-gaps.sh         now() - max(recv_ts), plus age_s
scripts/check-hostmon-staleness.sh   now() - max(recv_ts)
```

None computes an inter-row gap, so none has this blindness. That is why
`check-ingest-gaps.sh` returned exit 2 rather than a clean result during the
farm1 outage. Reported honestly by the consumer coordinator: the shape happens
to be right and nobody had stated why until it was generalised here.

`scripts/backfill-gaps.sh` does use `lag()`, and there **the blindness is
correct** — it repairs gaps that have closed, and a gap with no far side is not
something that can be backfilled.

## Determined, after the fact

The first version of this note left one thing open: **whether any Grafana panel
presents an interruption statistic as a health signal.** Seven dashboards use
`lag()` in panel SQL. It was recorded as a candidate check rather than a
finding, because claiming it would have been the error the note is about.

**The consumer coordinator then ran the check.** Reported by them and recorded
here on their measurement, not on one taken from this side — the dashboards are
in `/opt/plc1-monitoring`, which this repo does not own.

**No dashboard presents an interruption statistic as a health signal.** Every
`lag()` in the provisioned panels falls into one of four groups:

| Use | Panels | Why it is immune |
|---|---|---|
| Rate deltas | `host_health` 13, `nas_health` 22 | lag over value *and* timestamp to make a per-second rate |
| Reboot-transition detectors | `plc1` 13, `plc1_diagnostics` 13, `plc3_actuator_drift` 3–6 | they find a *fall* in an uptime counter; during an outage they count no reboots, which is correct — no reboot happened |
| Episode and cut boundaries | `plc2_splitter` 14, `plc3_saw` 14/16/111/115–117, `plc4_tractor_scale` 12 | transitions *within* data, not gaps *in* it |
| The one real instance | `plc3_retract_jam` 2–5 | requires 2-second row adjacency, so it needs a neighbouring row to see anything — already filed as issue #27 on that repo, about a prune rather than an outage, identical mechanism |

### The worked example worth copying

`plc3_saw` panel 36 — *"Limit arrivals in this window — movement, or its
absence"* — sums increments of boot counters inside a window. It could trivially
render `0 arrivals` during an outage and read as a machine that simply did not
move. It does not:

```sql
SELECT CASE WHEN t.n = 0 THEN -1 ELSE t.h END AS "HOME arrivals in window"
  -- n = count of rows with home_rises NOT NULL in the window
```

**If the window contains no rows at all it renders `-1`, a sentinel, rather than
zero.** So *"nothing moved"* and *"nothing was recorded"* are different values on
the face of the panel — the distinction the blindness destroys, made explicit by
someone who saw it coming.

### A fifth instance of the pattern, found in the act of closing the fourth

The coordinator's stated reason for running the check now is worth keeping:

> That candidate check was a check on **my** repo recorded only in **your**
> `notes/`. Durable, but in a place no session of mine will ever look.

**Recording something durably in the wrong place is its own failure**, and it is
this same shape one indirection over: the fact was written down, and the
inference that writing it down made it findable did not hold. Doing the check
was cheaper than arranging for someone to find the note asking for it.
