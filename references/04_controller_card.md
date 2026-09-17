# Controller Card — PLC #4 Tractor Scale

Decision authority: Alex Pennington. Status: design of record.

An Arduino Uno R4 WiFi carries the whole machine. Every sensor and the display reach it over one
Qwiic bus through an eight-channel multiplexer; the operator keys are the only direct wiring.

## Board

| | |
|---|---|
| Controller | Arduino Uno R4 WiFi |
| Sensor bus | Qwiic, the board's second two-wire bus |
| Multiplexer | Eight-channel, address `0x70`, on the bus trunk |
| Operator keys | Five, pins D4 to D8, each to ground |
| Console | Serial over USB and over the radio module, both at once |

The keys use the controller's internal pull-ups. A key reads low when pressed, so a disconnected
key is silent rather than stuck on.

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

Two commands report the bus directly. `i2c` scans the trunk and then every channel, naming what
answers. `pwr` reports the voltage and current monitor.

## Display

Four rows, 20 columns. Row 4 belongs to the display layer rather than to any screen, so it
carries fault text on every screen including the menus. A screen added later cannot omit it,
because no screen writes it.

## Keys

| Key | Action |
|---|---|
| Right | Record the current weight |
| Up | Total the recorded weights |
| Down | Clear the tally |
| Left | Remove the last recorded weight |
| Select | Set zero |

Press one key at a time and let it rise before the next. A key held while another is pressed is
treated as rollover and does nothing, so that releasing one key can never trigger the other.

A key held down for more than ten seconds is treated as stuck. It is ignored so the remaining
keys keep working, and the fault row says so until it is released.

## See also

`10_pressure_loop_wiring.md` for the transmitter, the burden resistor and the loop supply.
