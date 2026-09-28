# Pressure Loop Wiring — PLC #4 Tractor Scale

**Document:** 02 — Pressure Loop Wiring
**Firmware:** build `699e3c6c`

One two-wire pressure transmitter on the loader hydraulic circuit, powered from its own 24 V supply,
read as the voltage across a burden resistor at the analog-to-digital converter.

## Parts

| | |
|---|---|
| Transmitter | Rosemount 1151, 0–3000 psi, two-wire 4-20 mA |
| Loop supply | Dedicated 24 V, separate from the board supply |
| Burden resistor | 68 Ω, 0.1 %, 25 ppm/°C or better |
| Converter | MCP3421A0, address `0x68`, 18-bit, used at 14 bit, own 2.048 V reference |

The burden value is fixed by the converter's 2.048 V reference, not chosen for resolution. A
150 Ω or 250 Ω burden is wrong for this converter: 150 Ω clips at 13.65 mA, below full scale.
Replace either with 68 Ω.

## Connections

Current leaves the supply, passes through the transmitter, crosses the burden, and returns. The
burden sits in the return leg and the converter measures across it.

```
  24 V (+) ──────────────────────►  Transmitter  (+)

                                    Transmitter  (−)
                                            │
                                            ├─────────────►  MCP3421  IN+
                                            │
                                        [  68 Ω  ]
                                            │
  24 V (−) ──────┬────────────────────────┴─────────────►  MCP3421  IN−
                 │
                 └──────────────────────────────────────►  Board GND
```

> **Fit the burden in the return leg, between the transmitter negative and the supply negative.**
> Wired that way the converter inputs sit no more than about two volts above ground. A burden
> fitted in the supply leg instead puts the full 24 V on those inputs and destroys the converter
> the moment the loop is energised.

Bond the 24 V negative to board ground.

## What each current means

| Loop current | Across the burden | Meaning |
|---|---|---|
| Below 3.5 mA | Below 0.238 V | Dead loop — cut wire, no supply, transmitter removed |
| 4.0 mA | 0.272 V | Live zero, 0 psi |
| 20.0 mA | 1.360 V | Full scale, 3000 psi |
| 21.0 mA and above | 1.428 V and above | Transmitter fault, by NAMUR NE43 |

A dead loop reads as an affirmative fault, not as zero weight. That distinction is the reason the
loop is a current loop and not a voltage output.

## Headroom

The converter reads 2.048 V at full scale. The table above uses 1.428 V at the top of the
measuring band, leaving room above it for an over-current condition to be seen rather than
clipped.

A transmitter of this class can drive 29.7 mA when it fails hard over. Against 68 Ω that is
2.02 V, still inside the converter's range, so a hard-over is reported as a fault rather than read
as a weight. A larger burden pushes that case past the reference and the fault
becomes indistinguishable from full scale.

## Supply

Give the loop its own 24 V supply, separate from the board supply.

Compliance is not tight at 24 V. The burden drops 1.36 V at full scale, leaving about 22.6 V at
the transmitter against the roughly 10.5 V it needs.

## See also

`01` §*Bus channels* for the converter's address and the channel it answers on.
