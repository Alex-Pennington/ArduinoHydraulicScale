# RFID pendant: Alex's answers (2026-10-25)

Session `rfid-pendant-1025`. Follows `notes/2026-10-25-rfid-read-location-ruling.md`, where he
ruled: *"Disconnect the reader from the plc4 hardware and create a pendant style reader"*.

Everything under *Answered* is his, quoted or as the option he chose. Option labels and their
descriptions are this session's text, not his sentences.

## Answered

### What "pendant style" looks like

Asked which picture he had in mind, he answered with a link instead of an option:
<https://claude.ai/artifact/8roZXem4DLCWgNuhEPq2GF>, the "Pendant Case — Print Files" page. Read
from that page (version `1792339553-89d8`): a 72 × 174 × 30.4 mm PETG case around the Heltec WiFi
LoRa 32 V4 Expansion Kit (the V4 on Heltec's carrier with a 2.8″ touch screen and an 18650 cell),
four push buttons, a rotary encoder, a 915 MHz flex antenna, and a lanyard hole.

Asked how the RFID pendant relates to it, he chose **"Its own copy"**: *"A separate RFID pendant
built on the same kit and case style, with a reader in it."*

### How it is used, in his words

> *"we enter the scene with the bag filled at the splitter hanging off the loader the splitter
> operator grabs the pendant scans the tag and presses record. cut, next scene, we are prepairing
> for a delivery, i grab the pendant and loader and load a bag onto the forks, scan the tag, and
> press record for delivery. etc..."*

### Where RECORD is pressed

He chose **"On the pendant"**: *"Scan and record both happen on the pendant; nobody needs the phone
for it."*

### What else it does

Asked what else a person should be able to do from the pendant (options offered: pick the tally,
void last, tare, moisture entry), he answered in his own words:

> *"look at the heltec mesh code.  it already does the rest"*

Which code that names, and what "the rest" covers, is being read (read-only) before anything is
built on it.

### How many pendants

He chose **"Not decided"**: *"Design so either works."*

## What follows (inference, not his words)

- Two different people use it: the splitter operator at the splitter, and Alex at the loader
  for a delivery. So the pendant is carried between them, or each has one. NOT DETERMINABLE.
- "Press record for delivery" means the pendant must say which tally a record goes to. Today the
  phone app chooses (`notes/rfid-inventory-discussion.md`, *Tallies*; greybeard-plc4-pwa
  `api/rfid.py`).
- The weight still comes from PLC #4. Today's weight message has no record ID
  (`src/ArduinoHydraulicScale.cpp:450-455`), so a record pressed on the pendant needs board
  firmware that takes the tag in the record command, or a join nobody has designed.
- The case V2 log entries 3-5 (C610 antenna, bulkhead, front-guard placement) were for a reader on
  the tractor. Nothing above says whether they carry over. His to say.

## Not decided

How the pendant reaches the board (WiFi to the broker, WiFi to the app server, LoRa, or other),
which reader goes inside, how it is powered and charged, read range, and how tally and moisture
content are chosen on it.
