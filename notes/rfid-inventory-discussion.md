# RFID inventory tracking — discussion notes (2026-10-07)

Discussion and design only. Nothing here is approved for firmware, purchase or install.

## The idea

Each face-cord bag (FCB) carries one UHF EPC Gen2 tag. PLC #4 gets a reader on the loader
frame. When the operator records a weight, the board reads the bag's tag, and the record
carries tag + weight + time + place into the existing consumer that turns weights into
inventory moves.

## Decided (Alex)

- One tag per bag. Outdoors. Bag life is mixed; reuse tags if the cost justifies it.
- Inventory answers: what moved, when, where, and its weight at the time.
- Where = GPS point plus a yard zone. Zone is derived from GPS downstream; the operator
  can pick or override it in the app.
- One bag per lift. The read is deliberate, done when the operator records the weight.
- Reader-to-tag range at record time under about 1 m, so UHF EPC Gen2.
- No read means the record is refused. 100% read reliability is not required.
- **More than one tag seen: the operator picks.** The app lists the tags seen, strongest
  signal first and preselected; the operator confirms with one tap or picks another. No
  printed marking on the tag is needed for this.
- **The pick happens in the phone app**, next to the zone pick. The board reports what it
  saw; it does not choose.

## Out of scope for this discussion (Alex)

- Bag material and where the tag sits on the bag.
- Where the reader, antenna and GPS mount.

## Bags per day

Not asked further. With one deliberate read per lift it barely affects the electronics
(inference: reader draw only peaks while reading). It matters for the tag-reuse cost case
and for how much time the operator-pick step costs per day.

## PLC #4 constraints (read from source 2026-10-07)

- UNO R4 WiFi. D0/D1 are the HC-05 console, D10/D13 are CAN: no spare hardware UART, no
  usable SPI (`references/01_controller_card.md`).
- I2C mux at 0x70; ch0-3 and ch7 occupied, ch4-6 free (`include/i2c_mux.h`).
- No RTC and no GPS on board today; records carry only `age_ms`.
- Power headroom: not determinable from the repo.

## Candidate parts (floated earlier; not re-verified in this session)

- SparkFun M7E Hecto UHF reader — UART only, up to ~1 A at 5 V, onboard antenna ~1-2 ft,
  so an external antenna is likely needed.
- NXP SC16IS750 UART-to-I2C bridge on mux ch5, to reach the reader without a UART.
- SparkFun SAM-M10Q GPS on Qwiic (0x42), mux ch4.

## Consequence of "operator picks" (inference)

The board's record has to carry every tag seen with its signal strength, not one tag. The
app does the choosing, so the board-side change is smaller than a board-side chooser.
