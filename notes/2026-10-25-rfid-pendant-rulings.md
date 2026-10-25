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

### The host: Heltec V4 or LilyGO T-Deck

Unprompted, later the same day:

> *"Note I mentioned the Heltech V4 Development Board that we use we also have a Liligo T deck
> Both of those will be an option and that the tea deck could be repurposed with its outputs into
> a RFID reader that I already have that would be a pretty cool test"*

So both the Heltec WiFi LoRa 32 V4 and the LilyGO T-Deck are candidate hosts. He suggests a test:
a T-Deck wired to a reader he already has. Asked which, he answered: *"NFC nodule V3 ELECHOUSE,
or frio-rc522"*, read as the Elechouse PN532 NFC module V3 or an MFRC522 ("RC522") module. Asked
whether that test comes before the design is written, he chose **"Test first"**: *"Wire the
T-Deck to the reader, scan a tie, show the tag on screen. Throwaway. What it teaches feeds the
design."*

Asked where the T-Deck would be flashed, he chose **"Plug into Saga"**. Then he said *"wait"*, so
nothing is flashed until he says go. Then: *"lets plan to use pin 21,22"*. Asked whether that is
the ESP32-C3 header labelled TX(21), RX(20) (LilyGO `images/c3_boot.jpg`, which shows RX as 20),
he chose **"Yes, TX21 / RX20"**. The plan is `spikes/tdeck-nfc/PLAN-c3-header.md`. Told it needs a 3.3 V USB-TTL adapter on a
2.0 mm header, he chose **"Use the Grove plug"** and wrote *"just use the othe pins"*: the test
uses the Grove build (S3 GPIO43/44) already on the branch.

### "The heltec mesh code" (read-only survey, not his words)

It is `/home/rayven/mesh`, env `mesh-handheld` on the Heltec V4 TFT, with a T-Deck sibling env
`mesh-handheld-tdeck`. Per the survey, it already runs PLC #4 TARE, RECORD into the daily log or a
selected delivery, UNDO LAST, CLEAR BOARD, DAILY LOG +, tally select and end, delivery start from
the director list, and the live weight. It sends them by HTTP to the PLC #4 PWA
(`firmware/src/modules/plc/core/Plc4Operator.h:33`), not MQTT. On the PWA's RFID build it disables
its PLC #4 buttons (`Plc4Operator.cpp:492-493`). It has no RFID reader code, no moisture entry and
no void. That repo belongs to the mesh coordinator; nothing in it is edited from here.

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
