# Controller Card — PLC #4 Tractor Scale
## Cabinet Reference Card

PHOENIX NEST LLC · PLC #4
**Document:** 01 — Controller Card
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

An Arduino Uno R4 WiFi carries the whole machine. Every sensor and the display reach it over one
Qwiic bus through an eight-channel multiplexer.

## Board

| | |
|---|---|
| Controller | Arduino Uno R4 WiFi |
| Sensor bus | Qwiic, the board's second two-wire bus |
| Multiplexer | Eight-channel, address `0x70`, on the bus trunk |
| Console | Serial over USB and over the radio module, both at once |
| D4-D8 | Unused |

## Bus channels

Nothing answers at its own address until its channel is selected. The multiplexer is the only
device on the trunk.

| Channel | Address | Device | Used here |
|---|---|---|---|
| 0 | `0x67` | Thermocouple amplifier | no |
| 1 | `0x2A` | Load-cell amplifier | no |
| 2 | `0x40` | Voltage and current monitor | yes, diagnostics |
| 3 | `0x68` | Pressure-loop converter | yes, the measurement |
| 7 | `0x27` | Display, 20 by 4 | yes |

> **A scan that finds nothing means the trunk or the multiplexer, never a device behind it.**
> The multiplexer sits ahead of every channel. When it cannot answer, the converter and the
> display both fall silent together and neither one is at fault. Check the trunk lead and the
> bus supply before opening anything downstream.

The thermocouple and the load cell are wired and belong to no function on this machine. Leave
them alone rather than assuming they are spare inputs.

## Console

The console answers on both the USB port and the radio module, and takes commands from either.
A technician with a laptop and an operator with a handset see the same output.

| Device | Terminal | Direction | Rate |
|---|---|---|---|
| Radio module | D0 | Board receives | 9600 baud |
| Radio module | D1 | Board transmits | 9600 baud |
| USB | — | Both directions | 115200 baud |

> **The radio pair crosses.** The module's transmit pin lands on D0 and its receive pin on D1.
> Wired straight through, the console is silent both ways with nothing else wrong and nothing to
> see: the board keeps weighing, so the fault looks like a dead module rather than a swapped pair.

Two commands report the bus directly. `i2c` scans the trunk and then every channel, naming what
answers. `pwr` reports the voltage and current monitor.

## Display

Four rows, 20 columns, one screen, read-only. Row 4 belongs to the display layer rather than to
the screen itself, so a screen added later cannot omit it — nothing but the display layer writes
it.

Every operator action reaches the board over MQTT from a phone app, never from the display or
from the board directly. The console, over USB or the radio module, is the service and recovery
path.

## See also

`02` §*Connections* for how the transmitter, the burden resistor and the supply join up.
