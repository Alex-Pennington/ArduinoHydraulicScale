# Where the RFID read happens, and what ties tag to weight

**Date:** 2026-10-25. **Scope:** facts and options for Alex's open question. Nothing here is
decided, or approved for firmware or purchase. Session `rfid-read-location-1025`. Adversarially
reviewed against the primary sources the same day; the review's corrections are folded in.

**His question** (`notes/2026-10-25-rfid-read-range-and-antennas.md` §6): *"we could bring the tag
to the unit to scan it while we're loading the bags like I could bring the zip tie to the RFID unit
to scan it It really doesn't even need to be on the loader when I start thinking about it because
this RFID thing may actually want to be a handheld unit on its own or there may be an RFID scanner
device that we could use that's already built the loader does only has to tie the two pieces of
information together and not even the loader does that the PWA app can do that does that make
sense"*

**How to read the claims.** `file:line` cites a repo: PWA = `greybeard-plc4-pwa`, consumer =
`greybeard-plc-consumer`, `src:` = this repo's `src/ArduinoHydraulicScale.cpp`. A bracketed number
cites *References*. **Inference** marks a conclusion no source states. **NOT DETERMINABLE** marks
what no source here settles. Nothing was measured. Prices are what the cited page showed today;
none is a quote. **EU band** means the listed unit is 865–868 MHz and is not the part for use here
(US 902–928 MHz).

## 1. What changes when the read leaves the loader

**What ties tag to weight.**

- **Reader on the tractor, reading the bag where it sits** (today's ruling): a tag is in the RF
  field at the moment the weight is taken. The board can refuse on no tag, or on several
  (`notes/rfid-inventory-discussion.md`, "One press does it all"). It proves a tag was in the
  field, not that the tag is on that bag (a tag on a nearby bag could read).
- **Every other route:** the operator scans a tag, then presses RECORD. The link is the
  operator's act. Nothing catches a tag scanned for one bag and recorded against another
  (inference).

**Who does the join: the app cannot do it alone on today's firmware.** Today's weight message has
no record ID: `{w, age_ms, backlog, sensor, cal, n, total, id}`, and `id` is the board's client ID
(`src:450-455`). That is why Alex chose, on 2026-10-07, that the board publishes the one complete
record: "on seeing that the weight topic has no per-record ID to join on, he chose this flow"
(`notes/rfid-inventory-discussion.md:30-34`). The moisture message the app sends keyed by record ID
(PWA `README.md`, *Tally start and moisture content*) exists only on the RFID record build. So
every route, including a handheld, needs the RFID record firmware on the board, or a join by time
that nothing has designed (inference).

What does carry over: the consumer needs a tag on every record and does not care who read it
(consumer `consumer/plc1_consumer.py:3707`). The approved command already has an optional `tag`
field on RECORD (`notes/2026-10-07-rfid-record-contract.md:85`; PWA `api/commands.py:234-237`).

## 2. Bluetooth through the browser needs HTTPS

The app is served over plain HTTP: nginx `listen 80` (PWA `nginx/default.conf:2`), and no TLS in
front of it anywhere in the PWA repo. Web Bluetooth works only on a secure (HTTPS) page [1][2], as
do Web NFC [3] and Web Serial [4]. Web Bluetooth is also Chrome-on-Android only
(`findings/2026-10-14-rfid-cots-alternatives.md` §2.1).

**Two ways past it:**

- **Serve the app over HTTPS** (a certificate on farm1). That allows Web Bluetooth on Android
  Chrome, and it also gives the app the phone's GPS, which the 10-07 design wants
  (`notes/rfid-inventory-discussion.md:28`) and loses today only because of HTTP (PWA
  `api/commands.py:199-202`). Readers would still need their Bluetooth protocol written in
  JavaScript; CSL publishes theirs as MIT source (10-14 finding §2.1). Not designed.
- **A reader in Bluetooth keyboard (HID) mode.** It pairs as a keyboard and types the tag number.
  Keyboard input needs no secure context; only the Bluetooth, NFC and Serial APIs do [1][3][4].

## 3. The routes

| | Where the tag is read | Board side | Firmware | Reader hardware |
|---|---|---|---|---|
| **A. As ruled** | Reader in the case, Vulcan C610 on the front guard, reads the bag in place | M7E, C610, coax, IP67 bulkhead, u.FL rework | RFID record build, contract as approved | M7E $309.95 + C610 ~$129–149 + coax and bulkhead |
| **B. Zip tie to the box** | Reader in the case on its built-in antenna; tag held to the lid | M7E only | RFID record build, contract as approved | M7E $309.95 |
| **C. Handheld keyboard reader** | Reader paired to the phone; scan the tie, the number goes into the app | No reader | RFID record build, with the tag taken from the app instead of read | see §3C |
| **D. Phone with a reader built in** | Android handheld with UHF replaces the phone | No reader | As C | see §3D |

Every route also carries the board-side lines the 10-14 finding lists: a new R4 and shield if the
in-service board is not used ($42.45), the level interface and regulator, unpriced
(`findings/2026-10-14-rfid-cots-alternatives.md` §0, §2.3).

### A. As ruled

Unchanged from `notes/2026-10-25-rfid-read-range-and-antennas.md`. RF power is tuned at the
machine so stacked bags nearby do not read.

### B. Zip tie to the box

- **Range: SparkFun gives two figures.** The hookup guide: "reads tags reliably at only up to a few
  inches away at the highest read power", "away from any solid surfaces" [5]. The product page:
  "roughly one to two feet" [6]. Held to the lid, a few inches is enough (inference). At one to two
  feet, other ties or bags near the case could also answer, and the board would refuse on several.
- **RF exposure.** The M7E-HECTO user guide's FCC grant assumes at least 20 cm between the antenna
  and a person; closer than that, "module power should be reduced, or a lower gain antenna must be
  used" [7] (RF safety). B puts a hand within inches of the antenna. Turning power down shrinks a
  range that is already short. Whether a reduced power still reads the RSCT tag at the lid is NOT
  DETERMINABLE without a bench test.
- **Timing.** The board reads when the command reaches it, after phone → farm1 → broker → board.
  The tag must still be at the lid then. That delay was not measured (inference that it is under a
  second or two).
- **Placement.** The V1 lid holds the reader with no metal in front
  (`notes/2026-10-25-rfid-case-print.md:46-52`) and, by inference, faces where the LCD faces
  (`notes/2026-10-25-rfid-read-range-and-antennas.md` §2). Whether the case is in reach when you tie
  a bag is NOT DETERMINABLE here.

### C. Handheld reader in keyboard mode

Readers with a keyboard (HID) mode. Ranges are vendor figures, none with the RSCT cable-tie tag.

| Reader | Keyboard mode | Range | Sealing | Temp | Price, as listed |
|---|---|---|---|---|---|
| Unitech RP902 | "HID / SPP / BLE" listed; no setup steps found [8] | up to 2 m | IP65 | −10 to 50 °C | £254.59 ex VAT, **EU band**; US part not priced [8] |
| CAEN qIDmini, HID model (US: WR1170IUHIDP) | Yes, "native keyboard emulation" [9] | up to 90 cm | IP32 | −10 to 55 °C | **not found**. (The $409.10 listing is the EU Apple-profile model, not HID.) |
| CAEN skID7 R1287I | Yes, BLE and classic [10] | up to 3 m | IP65 | −10 to 55 °C | €522.50, **EU band**, out of stock; US part WR1287IXUSAA not priced [10] |
| Zebra RFD40 Premium | Yes, set once from a PC with 123RFID Desktop; no Zebra app at run time [11] | ~6 m | IP54 (resellers) | −10 to 50 °C (resellers) | US SKU seen at $1,583.80 and $1,974.50 |
| TSL 1166 | "Bluetooth HID (configurable)" on a preliminary datasheet [12] | up to 6 m | "TBD" on the datasheet; IP67 on resellers | −20 to 55 °C | not read |
| TSL 1128 | HID claimed; TSL's pages refused automated reads, **not verified** | up to 7 m (TSL) | IP54 | −10 to 40 °C for units since Nov 2021 (TSL) | $1,369 (10-14 finding) |

No keyboard mode was found for the CSL CS108/CS710S, Chainway R5/R6 or the Geenfc A7. That is
absence of evidence, not proof. A wired USB keyboard-mode reader on the phone's port was not
researched.

**What C costs beyond the reader:**

- **Firmware:** the whole RFID record build (record IDs, tallies, voids, the queue) minus the
  reader code. The `tag` field on RECORD becomes required and taken on trust
  (`findings/2026-10-14-rfid-cots-alternatives.md` §2.2).
- **App:** the page has no text box on purpose: it redraws every second and a box would lose its
  value and keyboard (PWA `docs/superpowers/specs/2026-10-21-moisture-content-design.md:92-93`,
  `web/js/render.mjs:1346`). Catching a keyboard reader's typing means new app work; not designed.
  With a keyboard paired, phones often hide the on-screen keyboard; TSL, Zebra and CAEN each add a
  setting for it.
- **Consumer:** code unchanged. The ack's `tags` and `tags_seen` columns and the NO TAG, MULTIPLE
  TAGS and TAG NOT IN VIEW refusals go permanently empty (consumer `consumer/plc1_consumer.py:4715`).
  The duplicate rule, "its w and tag may differ" (consumer `docs/design/plc4-rfid-consumer.md:48`),
  changes meaning when the app supplies the tag (inference).
- **Range cuts both ways (inference).** A 2–6 m reader scans from the seat and reads any bag it is
  pointed near; a 90 cm one means walking to the tie. Several can be set to read only on the
  trigger.

### D. A phone with the reader built in

- **Chainway C72:** €745.48 on rfid.it today (regular €828.31), **EU band**; IP65, −20 to 50 °C
  [13]. Its keyboard emulator's documented mode is an Android broadcast, which a web page cannot
  receive; whether its typing mode works into Chrome was not confirmed.
- **Zebra MC3330xR:** about €3,403 for one EU configuration (not verified). DataWedge can type RFID
  reads as keystrokes into the app in focus [14]. Not tested in Chrome.

The same app and firmware costs as C apply. Inference: D replaces the operator's phone with a
heavier device.

## 4. NOT DETERMINABLE here

- Where you stand and what is in your hands when the zip tie goes on, so which route costs fewest
  steps.
- Whether the case is in reach then (B), and whether the M7E at a power safe for a hand still reads
  the tag at the lid.
- Read range of anything here with the RSCT-U8-AN-10330 tag. Nothing was measured.
- Winter: no reader read is rated below −20 °C.
- US-band prices for the cheaper keyboard readers.
- Keyboard-mode behaviour in this app on your phone. Untested.

## References

[1] MDN, "Web Bluetooth API." https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API

[2] W3C Web Bluetooth CG, "Web Bluetooth," spec source.
https://raw.githubusercontent.com/WebBluetoothCG/web-bluetooth/main/index.bs

[3] W3C Web NFC CG, "Web NFC." https://w3c-cg.github.io/web-nfc/

[4] MDN, "Web Serial API." https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API

[5] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M7E Hecto: External Antenna."
https://docs.sparkfun.com/SparkFun_Simultaneous_RFID_Reader_M7E/external_antenna/

[6] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M7E Hecto, WRL-24738."
https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html

[7] JADAK (Novanta), "ThingMagic M7E-HECTO User Guide," Rev. 1.2, RF safety.
https://cdn.sparkfun.com/assets/8/5/2/4/d/M7E_HECTO_User_Guide.pdf

[8] ERS, "Unitech RP902 Bluetooth UHF RFID pocket reader."
https://www.ers-online.co.uk/p11510/unitech-rp902-bluetooth-uhf-rfid-pocket-reader

[9] CAEN RFID, "R1170I qIDmini keyfob reader," datasheet, via
cisper.com/datasheets/caen/Caen-R1170I_qIDmini_keyfob_reader_datasheet.pdf

[10] rfid.it, CAEN skID7 R1287I listing (URL slug names the qIDmini; the page is the skID7).
https://rfid.it/en/caen/73-r1170l-qldmini-bluetooth-rain-rfid-reader.html

[11] Zebra Technologies, "RFD40/RFD90 release notes SAAFKS00-004-R04" and "RFD40 Series Premium
Product Reference Guide."
https://www.zebra.com/content/dam/support-dam/en/documentation/unrestricted/guide/product/rfd40series-premium-prg-en.pdf

[12] Technology Solutions (UK) Ltd, "1166 Bluetooth Reader," preliminary datasheet, via
gatewayrfidstore.com/product_images//1166BluetoothReader.pdf

[13] Chainway, "C72," datasheet via cisper.com/datasheets/Chainway/Chainway_C72_EN_datasheet.pdf;
price, https://rfid.it/en/358-handheld-rfid-reader

[14] Zebra Technologies, "DataWedge RFID input" and "Keystroke output."
https://techdocs.zebra.com/datawedge/8-1/guide/input/rfid ;
https://techdocs.zebra.com/datawedge/11-4/guide/output/keystroke
