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
- **One press does it all** (Alex, 2026-10-07). The operator presses record in the app as
  today; the board reads the tags and the weight in that same step.
  - One tag in view: the record goes through on that press.
  - No tag: refused.
  - Several tags: the board refuses, and its reply lists the tags, strongest signal first.
    The operator taps the right one; the app sends record again naming that tag; the board
    reads again and accepts only if that tag is still in view.
- **GPS and time come from the phone.** The app adds them to every record command. No GPS on
  the board; the SAM-M10Q is dropped and mux ch4 stays free.
- **The board publishes the one complete record** on `plc4/jd5055e/weight`, as it does
  today: weight plus tag plus the phone's GPS and time. The consumer keeps reading one topic
  from one source and never has to join two messages. (Alex first answered "the app
  publishes the final record"; on seeing that the weight topic has no per-record ID to join
  on, he chose this flow.)
- **Reader power: the existing beefy off-board 5 V supply** (Alex). Its spare capacity for
  the reader's ~1 A is his statement, not measured.

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
- ~~SparkFun SAM-M10Q GPS on Qwiic (0x42), mux ch4.~~ Dropped: the phone supplies GPS and time.

## What this means for the board (inference, read against the repo)

- The record command (`plc4/jd5055e/cmd`) gains optional fields: phone GPS, phone time, and a
  chosen tag on the retry.
- The ack for a multi-tag refusal has to carry the tag list with signal strengths.
- The weight record gains the tag, GPS and time. The phone time also fixes "when" for
  replayed records, which today carry only `age_ms`.
- A tag read has to fit inside, or be split across, the 500 ms main loop.
- All of this is a gated firmware change; nothing here is approved.
