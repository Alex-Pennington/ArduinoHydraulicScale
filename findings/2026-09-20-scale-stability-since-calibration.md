# Scale stability since the last calibration

Analysis pass, 2026-09-20. Requested by Alex: *"look at the scale data since the last
calibration for stability - note the loader arms creep down overnight and that causes
some drift in the reading."*

Every number below is MEASURED, from `plc1-db` (`plc4_telemetry`, `plc4_status`) and
from a live MQTT capture on `farm1.internal:1883`. Where a question could not be
answered it says NOT DETERMINED and what would answer it.

## The reference points

| | |
|---|---|
| Last calibration | **2026-09-18 12:50** — `cf` 2.25943 → 0.99578, unchanged since |
| Last tare | **2026-09-19 08:38** — `tare` → 1940 |
| Trustworthy record | 2026-09-01 → 2026-09-20, 213,971 telemetry rows |

`cf` has not moved since 09-18 12:50, so everything after that is one calibration.

**Counts are the honest unit here.** At `cf` 0.99578 one ADC count is almost exactly one
unit of weight: live at `counts` 1658, `tare` 1940, the board published `w` -281, and
(1658-1940) x 0.99578 = -280.8. So a drift of -200 counts is a drift of ~200 in whatever
the operator reads as weight. Counts are also calibration-independent, which is what makes
nights either side of a recalibration comparable at all.

## Short-term stability is excellent

14 telemetry samples over 70 s, live:

```
-281 -281 -280 -281 -281 -282 -282 -281 -281 -282 -282 -281 -282 -282
min -282   max -280   spread 2   mean -281.4
```

Quiescent hourly standard deviation across the record runs 0.4 to 3.0 counts. **The
transducer is not noisy and it is not wandering.** Whatever is wrong is not the sensor.

## Overnight drift is real, and it is hydraulic

Half-hourly, the night of 09-19 into 09-20:

```
slot     w       counts   psi     mA
18:00   -104.3   1834.7   514.1   6.745
22:00   -100.9   1838.1   516.5   6.757
02:00   -102.3   1836.7   515.4   6.752      <- flat for eight hours
02:30   -109.6   1829.4   510.3   6.726      <- onset
04:00   -141.7   1797.3   488.2   6.608
05:00   -210.8   1728.1   440.5   6.354      <- peak rate
06:00   -278.8   1659.2   393.0   6.100
07:30   -341.8   1596.2   349.6   5.869
```

**`counts`, `psi` and `mA` all move together.** This is a genuine loss of hydraulic
pressure — 518 psi down to 350 — not an artefact of the weight calculation. The reading
did not drift; the machine did.

The shape is a transient, not a constant-rate leak: flat until ~02:00, fastest around
04:30-05:30 at roughly 70 counts/hour, decelerating by 07:30. It settles toward a new
level rather than bleeding steadily.

## It does not happen every night

Nine quiescent nights (windows 01:00-02:00 vs 05:00-06:00, nights with activity in either
window excluded by standard deviation):

| Night | Drift (counts) |
|---|---|
| 2026-09-06 | -3 |
| 2026-09-08 | -5 |
| 2026-09-18 | -9 |
| 2026-09-14 | +7 |
| 2026-09-15 | +20 |
| 2026-09-05 | **-111** |
| 2026-09-02 | **-143** |
| 2026-09-19 | **-158** |
| 2026-09-04 | **-258** |

**Bimodal.** Five nights are flat within ±20 counts. Four lose 111 to 258. Two further
nights (09-01 -541, 09-03 -420) show the same direction but had activity in a window and
are excluded from the table rather than counted.

So the overnight fall is common and sometimes large, but it is not every night, and a
tare taken yesterday is not reliably wrong this morning — it is a coin toss.

## What does NOT explain it

**The day's workload does not predict the night's drift.** Tested directly:

| Night | Daytime activity (sd of counts) | Overnight drift |
|---|---|---|
| 2026-09-02 | 1790 | -143 |
| 2026-09-14 | 917 | +7 |
| 2026-09-15 | 879 | +20 |
| 2026-09-04 | 630 | -258 |
| 2026-09-08 | 569 | -5 |
| 2026-09-19 | 419 | -158 |
| 2026-09-05 | 100 | -111 |

The two busiest days after 09-02 were followed by flat nights, and the quietest day on
the list was followed by a 111-count fall. A hot-fluid-cooling-overnight story is not
supported by this data and should not be repeated as if it were.

## What was not determined

**NOT DETERMINED: the cause.** Thermal contraction and mechanical leak-down both fit the
transient shape, and nothing in the record separates them.

What would answer it: a temperature log at the machine, aligned to these timestamps. The
board publishes no temperature. Home Assistant carries only an indoor thermostat
(`sensor.z_wave_thermostat_air_temperature`) and `weather.forecast_home`, which is a
forecast rather than a measurement at the tractor — neither is evidence about a machine
standing in a field.

The UNO R4's MCU has an on-die temperature sensor that the firmware does not currently
read. Publishing it would not give ambient, but it would give something that tracks
ambient at the enclosure, which is more than exists now.

## The consequence that matters

**Nothing on the board flags any of this.** At `w` -332 the status payload still read
`cal: ok`, `sensor: ok`, `stale: false`, `i2c_err: 0`. A stale zero and a real load are
indistinguishable to the firmware, so a morning operator sees a healthy machine reporting
a confident wrong number.

This is also the live precondition for #29 — `+` has no sign guard, so recording at -332
banks a negative weight and drives `total` negative. Alex ruled 2026-09-20 that the guard
stays unbuilt because the operator re-tares at the mark first; that ruling stands and this
file does not reopen it. It records that the precondition is real and was observed.

## A trap for anyone querying this data

**`stale` is NULL for every row before 2026-09-18**, because the field postdates that
firmware. A filter of `stale = false` silently discards 179,590 good rows — three
sigma of the record — and leaves two nights standing where there are twelve. Use
`stale IS NOT TRUE`.

This was not hypothetical: the first version of this analysis used `stale = false`,
found two nights, and concluded the overnight fall did not repeat. That conclusion was
wrong and was produced entirely by the filter.
