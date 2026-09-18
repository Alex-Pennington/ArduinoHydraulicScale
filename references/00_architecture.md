# Architecture — PLC #4 Tractor Scale
## Cabinet Reference Card

PHOENIX NEST LLC · PLC #4
**Document:** 00 — Architecture
**Firmware:** EEPROM signature PL4B
**Controller:** Arduino Uno R4 WiFi

PLC #4 weighs what the loader is carrying by measuring hydraulic pressure in the lift circuit.
It shows the weight to the operator at the machine and publishes it to the estate broker.

## The machine

| | |
|---|---|
| Tractor | John Deere 5055E |
| Loader | M520 / 520M |
| Measured quantity | Lift-circuit hydraulic pressure |
| Reported quantity | Weight in the bucket, in pounds |
| Range | 0–3000 psi at the transmitter |

## The measurement chain

Pressure becomes a current, the current becomes a voltage across a known resistor, and that
voltage becomes a weight. Each stage is a separate card.

```
  lift circuit  ──►  transmitter  ──►  4-20 mA loop  ──►  burden resistor
                                                                │
                                                                ▼
     display  ◄──  weight  ◄──  calibration  ◄──  converter reading
        │
        └──►  radio  ──►  estate broker
```

A weight is only as good as the calibration behind it. The scale is tared empty and calibrated
against a known load, both at the same boom position, and both are stored.

## Why a current loop

A current loop states its own health. A cut wire, a lost supply or a removed transmitter all
produce a current below the live zero, which is an affirmative fault rather than a reading of
zero pressure.

A voltage output cannot do this. A disconnected voltage input floats near the middle of its
range and reports a confident, plausible, wrong weight — which on a loader means a suspended
load reported as lighter than it is.

> **A fault is never shown as a weight.** When the loop, the converter or the bus cannot be
> trusted, the display blanks the weight and names the fault, and the machine refuses to zero,
> record or calibrate until it is fixed.

## Boom position

Weigh at one boom position and mark it. Hydraulic pressure in a lift circuit depends on geometry
as well as load, so the same weight reads differently at different boom heights.

Tare, calibrate and weigh at that same mark. A calibration taken at one position and used at
another is wrong by an amount that changes with the load.

## What reaches the broker

The weight, the pressure, the loop current, the converter reading and the sensor state, on a
repeating cycle, plus a retained status message describing the machine as a whole.

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
