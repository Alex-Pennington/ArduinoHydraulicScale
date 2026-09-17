# Pressure Loop Wiring — PLC #4 Tractor Scale

Decision authority: Alex Pennington. Status: design of record.

One two-wire pressure transmitter on the loader lift circuit, powered from its own 24 V supply,
read as the voltage across a burden resistor at the analog-to-digital converter.

## Parts

| | |
|---|---|
| Transmitter | Rosemount 1151, 0–3000 psi, two-wire 4-20 mA |
| Loop supply | Dedicated 24 V, separate from the board supply |
| Burden resistor | 68 Ω, 0.1 %, 25 ppm/°C or better |
| Converter | MCP3421, 18-bit, used at 14 bit, own 2.048 V reference |
| Signal pair | Shielded, shield grounded at the enclosure end only |

The burden value is fixed by the converter's reference, not chosen for resolution. Do not
substitute a value from an earlier card.

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

> **Bond the 24 V negative to board ground.** Without that bond the converter inputs float
> against its own supply and drift outside the range it can read. A floating loop can produce a
> plausible number before it produces an obviously wrong one.

## What each current means

| Loop current | Across the burden | Meaning |
|---|---|---|
| Below 3.5 mA | Below 0.238 V | Dead loop — cut wire, no supply, transmitter removed |
| 4.0 mA | 0.272 V | Live zero, 0 psi |
| 20.0 mA | 1.360 V | Full scale, 3000 psi |
| 21.0 mA and above | 1.428 V and above | Transmitter fault, by the NAMUR live-zero convention |

A dead loop reads as an affirmative fault, not as zero weight. That distinction is the reason the
loop is a current loop and not a voltage output.

## Headroom

The converter reads 2.048 V at full scale. The table above uses 1.428 V at the top of the
measuring band, leaving room above it for an over-current condition to be seen rather than
clipped.

A transmitter of this class has been metered at 29.7 mA driving hard over on this estate. Against
68 Ω that is 2.02 V, still inside the converter's range, so a hard-over is reported as a fault
rather than read as a weight. A larger burden pushes that case past the reference and the fault
becomes indistinguishable from full scale.

## Supply

Give the loop its own 24 V supply. Sharing the board supply couples converter noise into the
measurement and removes the isolation that lets the loop survive a cabinet fault.

Compliance is not tight at 24 V. The burden drops 1.36 V at full scale, leaving well over what a
Rosemount-class transmitter needs at its terminals.

## See also

`04_controller_card.md` for the converter, the bus it sits on and its channel.
