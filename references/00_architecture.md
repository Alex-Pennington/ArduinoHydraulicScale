# Architecture — PLC #4 Tractor Scale

**Document:** 00 — Architecture
**Firmware:** `uno_r4_wifi` build `699e3c6c`
**Updated:** 2026-10-25

PLC #4 weighs what the loader is carrying by measuring pressure in its hydraulic circuit. It
shows the weight to the operator at the machine and publishes it to the estate broker, with the
tractor's engine data and fault codes read off its J1939 bus.

## The machine

| | |
|---|---|
| Tractor | John Deere 5055E [1] |
| Measured quantity | Loader hydraulic circuit pressure, `02` §*Parts* |
| Reported quantity | Weight in the bucket, in pounds |
| Engine data | Read from the tractor's J1939 bus, `01` §*CAN* |

## The measurement chain

| Stage | Becomes | Card |
|---|---|---|
| Hydraulic circuit pressure | Loop current, 4-20 mA | `02` §*Parts* |
| Loop current | Voltage across the burden resistor | `02` §*Connections* |
| Burden voltage | Converter reading, in counts | `01C` §*Bus channels* |
| Converter reading | Weight, through the tare and the calibration factor | `06` §*Weighing* |
| Weight | Display rows and MQTT messages | `01D` §*Rows*, `03` §*Topics* |
| Tractor J1939 bus | Engine data and fault codes, through the CAN transceiver | `01B` §*Terminals*, `03` §*Engine* |

The engine data does not pass through the weighing chain and does not affect the weight. With no
CAN transceiver fitted the scale weighs the same and publishes no engine data [2].

## Faults

The front end is a current loop: a cut wire, a lost supply or a removed transmitter reads below
the live zero, as a fault and never as zero pressure (`02` §*What each current means*).

> **A fault is never shown as a weight.** When the loop, the converter or the bus cannot be
> trusted, the display blanks the weight and names the fault, and the machine refuses to zero,
> record or calibrate until it is fixed.

## What reaches the broker

The weight, telemetry, status, engine data and fault codes, on the topics in `03` §*Topics*.

## Wiring conventions

Each convention applies to every termination it names. A row elsewhere in the binder states only
what is specific to that termination.

| Convention | Applies to | Why |
|---|---|---|
| An output lands on an input: a module's transmit goes to the board's receive, and its receive to the board's transmit | The radio module, `01A` §*Terminals*; the CAN transceiver's receive output, `01B` §*Terminals* | A link wired straight through is silent both ways with nothing else wrong. The board keeps weighing, so the fault looks like a dead module |
| Every sensor-bus device lands on its own multiplexer channel, and only the multiplexer sits on the trunk. Fit the address variant the channel table names | Every Qwiic device, `01C` §*Bus channels* | A device answers only while its channel is selected. One on the wrong channel, or a variant at another address, reads as absent |
| Every supply that feeds a signal the board reads shares board ground | The 24 V loop supply negative; the CAN transceiver ground | A signal with no common return floats against the reader's supply and drifts out of the range it can read, and gives a plausible number before an obviously wrong one |

## Scope of this binder

This binder describes the design of PLC #4: the controller, the pressure loop, the connection to
the tractor's J1939 bus, the wiring between them, and the messages the machine publishes.

## References

[1] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`, line 1, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[2] A. Pennington, *greybeard-plc4-tractor-scale*, `include/j1939_can.h`, lines 89–91, at the stamped build. License: none stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale
