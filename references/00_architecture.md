# Architecture — PLC #4 Tractor Scale
## Cabinet Reference Card

PHOENIX NEST LLC · PLC #4
**Document:** 00 — Architecture
**Firmware:** build `699e3c6c`
**Controller:** Arduino Uno R4 WiFi

PLC #4 weighs what the loader is carrying by measuring pressure in its hydraulic circuit. It
shows the weight to the operator at the machine and publishes it to the estate broker, with the
tractor's engine data and fault codes read off its J1939 bus.

## The machine

| | |
|---|---|
| Tractor | John Deere 5055E |
| Loader | M520 / 520M |
| Measured quantity | Loader hydraulic circuit pressure |
| Reported quantity | Weight in the bucket, in pounds |
| Range | 0–3000 psi at the transmitter |
| Engine data | Read from the tractor's J1939 bus at 250 kbit/s; the board never transmits |

## The measurement chain

Pressure becomes a current, the current becomes a voltage across a known resistor, and that
voltage becomes a weight. Each stage is a separate card.

```
  hydraulic circuit  ──►  transmitter  ──►  4-20 mA loop  ──►  burden resistor
                                                                     │
                                                                     ▼
     display  ◄──  weight  ◄──  calibration  ◄──  converter reading
                     │
                     └──►  radio  ──►  estate broker
                                ▲
  tractor J1939 bus  ──►  CAN transceiver  ──►  engine data, fault codes
```

The engine data does not pass through the weighing chain and does not affect the weight.

A weight is only as good as the calibration behind it. The scale is tared empty and calibrated
against a known load, both at the same boom position, and both are stored.

## Why a current loop

A current loop states its own health. A cut wire, a lost supply or a removed transmitter all
produce a current below the live zero, which is an affirmative fault rather than a reading of
zero pressure.

A voltage output cannot do this. A disconnected voltage input floats near the middle of its
range and reports a confident, plausible, wrong weight.

> **A fault is never shown as a weight.** When the loop, the converter or the bus cannot be
> trusted, the display blanks the weight and names the fault, and the machine refuses to zero,
> record or calibrate until it is fixed.

## Boom position

Weigh at one boom position and mark it. Boom position changes the circuit pressure with the
bucket empty as well as loaded, so the same weight reads differently at different boom heights.
Tare, calibrate and weigh at that same mark.

## What reaches the broker

The weight, the pressure, the loop current, the converter reading and the sensor state, on a
repeating cycle, plus a retained status message describing the machine as a whole. The engine
values from the J1939 bus go out on their own slower cycle, and each engine control unit's
active fault codes go out on change, retained.

Recorded weights survive a loss of radio contact. They are held on the machine and published
when contact returns, marked with their age so that a late arrival is never mistaken for a
current reading.

## Scope of this binder

This binder describes the design of PLC #4: the controller, the pressure loop and the wiring
between them. Dated audit and commissioning records are kept outside it.

## See also

`01` §*Bus channels* for the sensor bus and what sits on each channel.
`03` §*Command* for how an operator action reaches the board.
`02` §*Parts* for the transmitter, the loop supply and the burden resistor.
