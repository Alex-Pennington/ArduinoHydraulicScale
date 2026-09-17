# Purchase List — PLC #4 Tractor Scale
## Purchase List

PHOENIX NEST LLC · PLC #4
**Document:** 06 — Purchase List
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

What the machine is built from, by the characteristics that matter when ordering. The order-code
column is blank on purpose: it is where the part actually bought gets written, and a code nobody
has bought is worse than an empty cell.

## Measurement chain

| Part | What matters when ordering | Order code |
|---|---|---|
| Pressure transmitter | Rosemount 1151, 0–3000 psi, two-wire 4-20 mA | |
| Burden resistor | 68 Ω, 0.1 %, 25 ppm/°C or better | |
| Loop supply | 24 V, its own supply, sized for one transmitter | |
| Signal pair | Shielded, two conductor | |

> **The burden's temperature coefficient is the part of the specification that survives
> calibration.** Calibrating against a known load removes a constant error, so a loose tolerance
> costs nothing once the scale is set. Drift does not: a 100 ppm/°C part moves about four times
> as far as a 25 ppm/°C one across an enclosure that sits in the sun, and it moves after the
> calibration was taken.

## Controller and bus

| Part | What matters when ordering | Order code |
|---|---|---|
| Controller | Arduino Uno R4 WiFi | |
| Bus multiplexer | Eight channel, answering at `0x70` | |
| Loop converter | Delta-sigma, own voltage reference, answering at `0x68` | |
| Display | 20 by 4 character, with a backpack answering at `0x27` | |
| Power monitor | Bus voltage and shunt current, answering at `0x40` | |
| Qwiic leads | One per device, plus one trunk lead | |

> **Address variants are a purchasing decision, not a wiring one.** The converter ships in eight
> variants across `0x68` to `0x6F` and the display backpack in two, and they are otherwise
> identical parts. A variant bought by mistake is silent on the bus and reads as a dead device.
> Order the addresses in the table, or expect to change firmware to match what arrives.

## Operator interface

| Part | What matters when ordering | Order code |
|---|---|---|
| Keys | Five, momentary, normally open | |
| Console radio module | Serial, 9600 baud | |

## Fitted and not used by this machine

Two devices share the bus and belong to no function here. They are listed so that nobody buys a
second one believing the machine needs it, and so that nobody removes one believing it is spare.

| Part | Answers at | Status |
|---|---|---|
| Thermocouple amplifier | `0x67` | Fitted, unused |
| Load-cell amplifier | `0x2A` | Fitted, unused |

## See also

`01` §*Bus channels* for where each device sits on the bus.
`02` §*Parts* for the measurement chain as designed.
