# Transducer: the failure, and choosing the replacement

Recorded 2026-09-01 from Alex, at the machine.

## The machine

**John Deere 5055E with an M520 front loader.** The firmware already announced the tractor
(`main :74` prints `JD5055e OK`) and that it was a loader scale (`:70` prints `Loader Scale`),
but the loader model was not recorded anywhere until now.

**The hardware is NOT attached to the tractor.** Everything to date — the flash, the console
session, every reading below — is bench work. No measurement taken so far is a hydraulic
measurement, and none of it has been validated against the M520's circuit.

## How the previous transducer failed

**It leaked at the electrical connection.** Not at the 1/8 NPT thread, and not as a weeping
fitting — pressurised oil worked through the sensor body and out of the cable entry.

That rules out the two cheap explanations and leaves the expensive one:

- **Not a sealing or installation fault.** Thread sealant and torque are not implicated, so a
  replacement installed the same way is not repeating someone's mistake.
- **The sense element or its internal seal failed**, letting oil into a cavity that is supposed
  to stay dry. On a pressure transducer that is the element giving up, and the usual causes are
  overpressure or pressure-cycling fatigue.

Two consequences worth acting on:

1. **Overpressure margin now matters more than accuracy.** See the selection note below.
2. **Oil travelled up the cable.** Capillary action along a conductor bundle carries oil to
   whatever the far end is. Before reusing that harness, check it — and check whatever enclosure
   it terminates in. Replacing the sensor and reusing an oil-filled cable moves the problem
   rather than fixing it.

## T4 DOES NOT WORK: an unconnected input reports `sensor = ok`

**Demonstrated on the bench, 2026-09-01, with NOTHING connected to A1:**

```
a = 4401 counts   v = 1343 mV   p = 632 psi   sensor = ok
```

Eight counts of peak-to-peak jitter — a stable, confident reading, from a pin with no sensor on
it at all. The firmware reports the sensor healthy and computes 632 psi from nothing.

**This is the exact failure T4 was written to catch, and T4 misses it.** The open-circuit
threshold is 1311 counts (0.4 V), on the assumption that a disconnected input reads near zero
volts. That assumption is wrong. A high-impedance CMOS analog input does not fall to ground when
it is disconnected — it floats on leakage and residual sample-and-hold charge, and it can sit
anywhere, here around mid-rail.

So the justification written into the firmware — "a chafed wire is the likeliest hardware failure
on a tractor, and it previously read as no load" — describes a defect the feature does not
actually fix. A cut sensor wire on the tractor would float exactly like this and be reported as
a valid pressure.

**The fix is hardware, not firmware: a pull-down resistor from A1 to GND.** Something in the
10k-47k range. With the sensor connected its low output impedance dominates and the error is
negligible; with the sensor disconnected the pin is pulled to 0 V, lands far below the 1311-count
threshold, and the open-circuit path works as designed. No firmware change can substitute for it
— the reading from a floating pin is not distinguishable from a real one.

**Until that resistor is fitted, treat `sensor = ok` as meaning "the reading is in range", not
"a sensor is present".**

This is the audit's through-line arriving in the hardware: a plausible wrong answer rather than a
visible failure. It is also the second time on this machine that a feature looked correct in
review and was wrong in fact — the first being the EEPROM validation, also found by flashing.

### Bench validation, still to run

With no sensor fitted, both fault paths can be proved with a jumper wire:

| test | expected raw | expected state |
|---|---|---|
| A1 jumpered to GND | ~0 counts | `NO SENSOR` |
| A1 jumpered to 5V | ~16383 counts | `SENSOR SHORT` |
| A1 floating | ~4400 counts | `ok` (the defect above) |

That exercises T4 end to end without a transducer, and confirms the thresholds and the ADC
independently of any sensor question.

## How the previous transducer failed — what it means for the replacement

## Selecting the replacement

Candidate: ceramic Al2O3 element, 304 SS housing, 1/8 NPT, IP65.

```
Output          0.5-4.5 V          Overload      150% FS
Supply          5-12 VDC           Burst         300% FS
Accuracy        +/-1.0% FS         Cycle life    500,000
Response        3 ms               Range option  300-3500 psi
```

**Firmware impact: none.** 0.5–4.5 V over 0–3000 psi is exactly what is coded —
`SENSOR_COUNTS_ZERO = 1638`, `SENSOR_COUNTS_FS = 14745`, `sensorMaxPSI = 3000`. The short-to-rail
diagnostic also survives, which it would not have with a 0–5 V output sensor.

**Correcting an argument made earlier in this workspace:** the 0–5000 psi alternative was
argued against on resolution grounds — 0.382 psi/count against 0.229. That reasoning was
irrelevant and should not be repeated. The sensor's own accuracy is ±1.0% FS = **±30 psi on a
3000 psi unit, 131× coarser than the ADC step**, and measured jitter on the bench is 8 counts
peak-to-peak (±0.9 psi) after the firmware's 10-sample average. Quantisation was never the
limiting term on either part. The real selection criteria are output range compatibility, supply
voltage, and overpressure margin.

**Range choice is the open one.** For a 3000 psi unit, overload is 4500 psi and burst 9000. If
the M520's circuit relieves near 2900 psi, normal operation sits at ~97% of full scale with 1.55×
to the overload limit — and hydraulic spikes on abrupt valve closure can consume that. Given the
previous unit failed by internal leakage rather than by installation, margin is the thing to buy.

The same family offers up to 3500 psi. At a 2900 psi working pressure that is ~83% FS with 5250
psi of overload margin, for ±35 psi accuracy instead of ±30 — a 5 psi difference that is inside
the noise floor already measured. **Pending the M520 circuit's actual relief pressure, which
nobody has recorded.**
