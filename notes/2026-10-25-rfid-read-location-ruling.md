# RFID read location: Alex's ruling (2026-10-25)

Session `rfid-read-location-1025`. Answers the question he opened in
`notes/2026-10-25-rfid-read-range-and-antennas.md` §6. The options he was shown are
`findings/2026-10-25-rfid-read-location-options.md`, adversarially reviewed before he saw them.

## Ruled

Asked "Where should the RFID read happen?" with options A (antenna on the guard), B (zip tie to the
box), C (handheld reader) and D (phone with a reader built in), he picked none of them and answered
in his own words:

> *"Disconnect the reader from the plc4 hardware and create a pendant style reader"*

## What that settles, as stated

- The reader comes off the PLC #4 hardware.
- The reader is a pendant-style unit, and it is created (built), not one of the off-the-shelf
  handhelds in the finding's route C or D.

## What it touches (inference, for the coordinator; not his words)

- `notes/case-v2-log.md` entry 6 is answered: the V2 case does not carry the reader. Entries 3-5
  (the external antenna, the bulkhead hole and reader position, the C610 on the front guard) were
  for a reader on the tractor. Whether any of them still applies to the pendant is his to say.
- The 10-07 design has the board reading the tag at the press ("one press does it all",
  `notes/rfid-inventory-discussion.md`). A reader off the board changes that. How the pendant's tag
  reaches the record is not decided here.
- The finding's §1 still holds for any route: today's weight message has no record ID
  (`src/ArduinoHydraulicScale.cpp:450-455`), so tying a tag to a weight needs the RFID record
  firmware, or a join that has not been designed.

## Not stated, so not decided

How the pendant connects (cable or wireless, to the phone, the board or the broker), what is inside
it, what powers it, and its range. Nothing here fills those in.
