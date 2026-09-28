# Controller Card — PLC #4 Tractor Scale
## Cabinet Reference Card

PHOENIX NEST LLC · PLC #4
**Document:** 01 — Controller Card
**Firmware:** build `699e3c6c`
**Controller:** Arduino Uno R4 WiFi

An Arduino Uno R4 WiFi carries the whole machine. Every sensor and the display reach it over one
Qwiic bus through an eight-channel multiplexer. The tractor's J1939 bus reaches it through a CAN
transceiver on D10 and D13.

## Board

| Field | Value |
|---|---|
| Controller | Arduino Uno R4 WiFi |
| Sensor bus | Qwiic connector, the board's second two-wire bus |
| Multiplexer | TCA9548A or PCA9548A, eight channels, address `0x70`, on the bus trunk |
| CAN | On-chip controller, 250 kbit/s, receive only |
| Console | Serial over USB and over the radio module, both at once |

## Header pins

| Pin | Function | Spec | Wire | Connected to |
|---|---|---|---|---|
| D0 | Console receive | 9600 baud | | Radio module transmit |
| D1 | Console transmit | 9600 baud | | Radio module receive |
| D10 | CAN transmit | 250 kbit/s | | Not connected; see §*CAN* |
| D13 | CAN receive | 250 kbit/s | | CAN transceiver receive output |
| Qwiic | Sensor bus trunk | 3.3 V | | Multiplexer |
| USB | Console | 115200 baud | | Laptop, when servicing |

No other header pin is used by the firmware.

## Bus channels

Nothing answers at its own address until its channel is selected. The multiplexer is the only
device on the trunk.

| Channel | Address | Device | Used by the firmware |
|---|---|---|---|
| 0 | `0x67` | MCP9600 thermocouple amplifier | no |
| 1 | `0x2A` | NAU7802 load-cell amplifier | no |
| 2 | `0x40` | INA219 voltage and current monitor | yes, diagnostics |
| 3 | `0x68` | MCP3421 pressure-loop converter | yes, the measurement |
| 7 | `0x27` | LCD2004A display, 20 by 4, PCF8574 backpack | yes |

> **A scan that finds nothing means the trunk or the multiplexer, never a device behind it.**
> The multiplexer sits ahead of every channel. When it cannot answer, the converter and the
> display both fall silent together and neither one is at fault. Check the trunk lead and the
> bus supply before opening anything downstream.

The thermocouple and the load cell belong to no function on this machine. Leave them in place
rather than treating them as spare inputs.

## CAN

The board reads the tractor's J1939 bus and never transmits on it: no frame, no request, and no
acknowledgement.

| Item | Value |
|---|---|
| Bit rate | 250 kbit/s, 29-bit identifiers |
| Board receive | D13, from the transceiver's receive output |
| Board transmit | D10, left unconnected |
| Transceiver transmit input | Tied recessive at the transceiver, so the node cannot drive the bus |
| Termination | None at this node. The tractor bus is terminated at its ends; a third 120 Ω resistor degrades it |
| Hardware filter | Off at every power-up; `canf` turns it on |

With no transceiver fitted, the scale weighs exactly as it does with one, and publishes no engine
data.

## Console

The console answers on both the USB port and the radio module, and takes commands from either.
A technician with a laptop and an operator with a handset see the same output.

| Device | Terminal | Direction | Rate |
|---|---|---|---|
| HC-05 radio module | D0 | Board receives | 9600 baud |
| HC-05 radio module | D1 | Board transmits | 9600 baud |
| USB | — | Both directions | 115200 baud |

> **The radio pair crosses.** The module's transmit pin lands on D0 and its receive pin on D1.
> Wired straight through, the console is silent both ways with nothing else wrong and nothing to
> see: the board keeps weighing, so the fault looks like a dead module rather than a swapped pair.

## Display

Four rows of 20 characters, one screen, read-only. Every operator action reaches the board over
MQTT from a phone app, never from the display. The console, over USB or the radio module, is the
service and recovery path.

| Row | Shows |
|---|---|
| 1 | Weight in pounds and the tally count, or the fault name when the reading cannot be trusted |
| 2 | Pressure in psi and loop current in mA, or dashes |
| 3 | Dropped weights, queued weights, tally full or nearly full, or the link state |
| 4 | The worst current fault, or blank |

## See also

`02` §*Connections* for how the transmitter, the burden resistor and the supply join up.
`07` §*Commands* for the console verbs, including `i2c`, `pwr`, `can` and `canf`.
`07` §*What the display asks for* for the fault messages on row 4.
