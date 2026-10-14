# RFID tally system: commercial off-the-shelf alternatives for PLC #4

**Date:** 2026-10-14. **Scope:** research and a document only. Nothing here is approved for
purchase, wiring or firmware.

**Why this exists.** Alex, 2026-10-14: *"the total cost of the hardware makes me wonder if we
should investigate cots solutions also"*. The custom build is
`findings/2026-10-07-rfid-hardware-recommendation.md`. The settled design is
`notes/rfid-inventory-discussion.md`, and the approved MQTT contract is
`notes/2026-10-07-rfid-record-contract.md`. This document compares against them and does not
reopen anything they record as ruled. Where an option would reopen a ruling, it says so.

**Sizing rule:** a firewood business, not NORAD (Alex, 2026-10-07). Robust and simple.

**How to read the claims.** A bracketed number cites a document under *References*. A
`file:line` cites this repository. **Inference** marks a conclusion drawn from cited facts but
not stated in any source. **NOT DETERMINABLE** marks something no source reached here settles.
Nothing was measured on hardware. Every price is the price on the cited page when this
document was written; none is a quote.

> **Revised 2026-10-14, second pass.** Alex ruled on the build (below), and an adversarial
> review corrected this document. What changed and why:
> - **Ruled build added** (§0): the M7E keeps its on-board antenna, with 200 RFIDsolution
>   RSCT-U8-AN-10330 cable-tie tags.
> - **§4 "no outdoor tag beats $1.86" is withdrawn.** It was false, and the Fujitsu row in the
>   same table already showed it. The RSCT tag is added.
> - **EPC uniqueness is a commissioning requirement, not a tag-selection criterion** (§4, §6).
> - **Option 1 risk added:** the priced SR691 listing shows no RS-232 and an 8–12 V supply
>   (§1.6).
>
> Superseded text stays in place, marked **Superseded 2026-10-14**, per Alex's standing rule.

## Ruled by Alex, 2026-10-14

In the coordinator's dialog, Alex chose **"Cheap tags, no antenna"** over "Cheap tags, keep
antenna" (the recommended option), "As designed, HID tags" and "Integrated reader". The option
was put to him as: *"$646. Reader's built-in antenna only, so the reader must sit within ~0.5 m
of the bag. Cheapest. Needs a bench test on the loader before committing."* **That wording is
the coordinator's option text, not his sentence.** His act was choosing it. This is closed.

## 0. The ruled build: the M7E's on-board antenna and cable-tie tags

**Current figures. Read this section first.**

| Line | Price |
|---|---|
| Arduino UNO R4 WiFi | $27.50 |
| Adafruit Proto-ScrewShield R3, product 196 | $14.95 |
| SparkFun M7E Hecto, WRL-24738, **on-board PCB antenna** | $309.95 |
| Pololu D36V28F5, only if the existing 5 V supply fails the M7E's ripple limits | $23.95 |
| **Hardware priced** | **$376.35** |
| 200 RFIDsolution RSCT-U8-AN-10330 tags (2 × 100 at $135) [28] | $270.00 |
| **Total priced** | **$646.35** |
| 10-pack outdoor trial of the same tag, before the 200 [28] | $51.00 |

- **Removed:** the Vulcan C610 antenna ($149.00) and the 12 ft coax ($57.00), saving $206.00.
  Also removed, and never priced: the Taoglas CAB.011 pigtail and its sealed bulkhead.
- **Range.** SparkFun gives the built-in PCB trace antenna "a read range of roughly one to two
  feet" [30], 0.3–0.6 m. So the reader must sit within about 0.5 m of where the bag hangs.
  Mounting is out of scope (`notes/rfid-inventory-discussion.md:65`); this is stated as a
  constraint only.
- **Still unpriced:** SN74AHCT125N, divider resistors, Mean Well DDR-30G-12, Vishay TVS, Hammond
  enclosure, cable glands, fuse and holder.
- **Open before committing:** a bench test of the on-board antenna's read range on the loader at
  the record position, through the enclosure. An RF-transparent enclosure face (polycarbonate,
  no metal) in front of the reader. The 10-pack outdoor tag trial. The tag's IP rating, which is
  not stated.
- **Requirement:** each tag gets a unique EPC written once before use (§4).

---

## Summary

**The baseline.** The custom build's priced lines total **$582.35**: UNO R4 WiFi $27.50,
screw shield $14.95, SparkFun M7E Hecto $309.95, Vulcan C610 antenna $149.00, coax $57.00,
Pololu 5 V regulator $23.95 (`findings/2026-10-07-rfid-hardware-recommendation.md:33-35`,
`:393-394`). The reader, antenna and coax are $515.95 of it.

**The tag price in that build is wrong.** The $1.86 Sentry Duo price is the **5,100-tag** tier.
At 150 tags the same page charges **$4.60 a tag ($690.00)**, and at 600 tags **$3.32 a tag
($1,992.00)** [22]. The earlier document left this open
(`findings/2026-10-07-rfid-hardware-recommendation.md:292`, `:424`). With 150 Duo tags the
custom build is **$1,272.35**, not $861.35. The tags, not the reader, are the largest line.

| # | Option | Priced hardware | With 150 tags at $690 | Keeps the ruled design? |
|---|---|---|---|---|
| 0 | Custom build (baseline) | $582.35 | $1,272.35 | Yes |
| 1 | Integrated outdoor reader on the board's serial port (Yanzeo SR691) | $359.44 + RS-232 transceiver | $1,049.44 + transceiver | Yes. Contract unchanged; reader driver changes |
| 2 | Bluetooth handheld or sled paired to the phone | $126.99 to $1,411.45, board reader removed | $816.99 to $2,101.45 | **No.** Reverses "one press does it all" |
| 3 | COTS loader-scale system | NOT DETERMINABLE (dealer quote only) | — | **No.** Replaces the in-service PLC #4 |
| 4 | Cheaper tags | — | $596.00 for 200 Xerafy Data Trak II; $244.00 for 200 Fujitsu WT-A543 | Yes |
| 0b | **Ruled 2026-10-14:** custom build on the M7E's on-board antenna, 200 RSCT-U8-AN-10330 tags | $376.35 | $646.35 with 200 RSCT tags ($270) | Yes, with the reader within ~0.5 m of the bag |

**Superseded 2026-10-14:** the recommendation below. Alex ruled "Cheap tags, no antenna"
(coordinator's option text, not his words); see *Ruled by Alex* and §0.

**Recommendation: keep the custom M7E build, and fix the tag line first.** The one COTS route
that keeps the ruled design (option 1) saves about $220 on priced lines, but it rests on vendor
documents that contradict themselves, a command set not published, no FCC ID on record, and a
reseller listing. Details and the conditions that would change this are in §6.

---

## 1. Option 1: an integrated outdoor reader on the board's serial port

The reader and antenna come in one sealed housing, powered from 12 V, talking RS-232, RS-485 or
Wiegand. It takes the place of the M7E, the C610, the coax, the U.FL pigtail and the 5 V
regulator.

### 1.1 Candidates read

| Reader | RF / antenna | Interfaces | Supply | Operating temp. | Sealing | Price | Source |
|---|---|---|---|---|---|---|---|
| **Chafon CF661** | 902–928 MHz (US); EPC Gen2; 0–26 dBm adjustable on the product page, "Maximum 27dbm (adjustable)" in the datasheet; 6 dBi circular; RSSI reported | RS-232, Wiegand 26, relay, trigger input. Optional TCP/IP, PoE, WiFi; TTL and RS-485 on request | 9–24 V (12 V 2 A adapter supplied) | **−10 to +55 °C** | IP65, ABS | NOT DETERMINABLE (vendor lists ¥0.00) | [1] |
| Chafon CF561 | As CF661, 6 dBi | RS-232 or USB, plus Wiegand 26/34 and trigger | 9–24 V, 0–3 A | −10 to +55 °C | IP65 | NOT DETERMINABLE | [2] |
| **Yanzeo SR691** | 902–928 MHz; EPC Gen2 and 18000-6B; 0–30 dBm adjustable; 9 dBi circular on the page, **8 dBi in its own spec PDF** | RS-232, Wiegand 26/34, RS-485; 1 trigger input, 2 GPIO | 12 V; "< 1W" in the spec PDF | −20 to +60 °C | **Title says IP67; body says "IP55 Splash Proof"** | **$316.99** (Newegg, third-party seller) | [6], [8] |
| Yanzeo SR681 | 902–928 MHz; 0–30 dBm; 8 dBi circular | RS-485, USB, Wiegand 26/34; RS-232 optional | **VCC min 8 V, typ 9 V, max 12 V**; 350 mA typ, 650 mA max | −20 to +50 °C | IP67 in the title only; no IP figure in its spec PDF | **$410.19** (Newegg, third-party seller, ships from Hong Kong) | [5], [8] |
| Yanzeo R781 | 902–928 MHz; 0–30 dBm; 8 dBi | USB, RS-232/RS-485, Wiegand 26/34 | "DC12V, 3A"; wiring sheet says DC+ 7–12 V | **−40 to +65 °C** (15 min warm-up advised in the cold) | IP67 in the title only | NOT DETERMINABLE ("Call for Price") | [7] |
| FEIG MRU400i | FCC 902–928 MHz; 1 W; integrated circular antenna plus one RP-TNC port | RS-485, Ethernet, USB OTG | **24 V or PoE+** | −40 to +70 °C | IP65 / IP67, aluminium | **$1,517.00** | [9] |
| Zebra FXR90 integrated | FCC; 33 dBm | **Ethernet and USB only** | Power supply or PoE | −40 to +65 °C | IP65 / IP67 | $3,190.00 | [10] |

**Set aside.** The Zebra FXR90 has no serial port the board can take, and costs more than five
times the baseline. The FEIG MRU400i wants 24 V or PoE+; FEIG's own supply kit takes
100–240 V AC input [9], so it does not run off the tractor as sold. Inference: it would need a
12-to-24 V boost converter, and at $1,517 it is far above the baseline anyway. Invelion's site
(`invelion.com`) presented an expired TLS certificate and was not read, so no YR-series reader
is in this table.

### 1.2 What carries over, and what changes

| Item | Custom build | With an integrated reader |
|---|---|---|
| Board, shield, J1939, loop, LCD, MQTT | §1 of the custom build | **Unchanged** |
| Reader | M7E Hecto, $309.95 | Integrated reader |
| Antenna, coax, U.FL pigtail, sealed bulkhead | C610 $149, coax $57, CAB.011 unpriced | **Gone.** One cable carries power and serial instead |
| Reader supply | 5 V with < 25 mV ripple; Pololu D36V28F5 if the existing supply fails | **12 V.** The 5 V ripple question goes away |
| Level interface on D0/D1 | SN74AHCT125N plus dividers | **An RS-232 transceiver** in their place (§1.3) |
| ENABLE line on D2 | Resets a hung module | Not available on the readers read. Inference: a hung reader is reset by switching its 12 V |
| Firmware reader driver | SparkFun library v1.2.0, MIT | **Written from the vendor's protocol,** which is not published (§1.4) |
| MQTT contract | As approved | **Unchanged,** provided the reader returns every tag in view with its RSSI in command mode |
| Design rulings | As ruled | **Unchanged** |

**Wiegand is not usable for this design (inference from the format).** Wiegand 26 carries 26
bits a frame and Wiegand 34 carries 34. A 96-bit EPC does not fit, and the output carries no
RSSI and no list of tags. The contract needs the full EPC
(`notes/2026-10-07-rfid-record-contract.md:210`) and a tag list strongest first on a multi-tag
refusal (`:144`, `:174`). Use the RS-232 or RS-485 port in command mode. Chafon calls it
"answer mode" [1]; Yanzeo calls it "Master-Slave Mode (Online Mode)" [5], [6].

**Multi-tag refusal carries over only if the reader reports each tag with RSSI.** The CF661
reports RSSI [1], and the SR681 claims ">50PCS" group reading [5]. Whether the SR691 or SR681
returns RSSI per tag on the serial port is NOT DETERMINABLE without its protocol document.

### 1.3 RS-232 on D0/D1: the transceiver

An RS-232 port swings to ± several volts and cannot go onto the R4's pins. A transceiver
converts it to logic levels.

**TI MAX3232, I temperature grade** (MAX3232I, −40 to +85 °C) [11]:

- Supply 4.5–5.5 V at the 5 V setting, 3.0–5.5 V overall. Up to 250 kbit/s [11] §6.3, §1.
- Receiver output VOH **≥ VCC − 0.6 V at −1 mA** [11] §6.7. The R4's D0 needs VIH ≥ 0.8 ×
  VCC (`findings/2026-10-07-rfid-hardware-recommendation.md:176-178`).
- **Power the MAX3232 from the R4's own 5 V pin.** Then both thresholds track one rail, and
  VCC − 0.6 V ≥ 0.8 × VCC holds for any VCC ≥ 3.0 V (inference, arithmetic). At 5.0 V it
  gives 4.4 V against the 4.0 V the board needs.
- Driver input VIH 2.4 V at 5 V, inputs to 5.5 V [11] §6.3. The R4's 5 V TX drives it
  directly.

This **replaces** the SN74AHCT125N and both dividers. No breakout board is named: none was
read. **RS-485** is the alternative for a long run. Inference: it is half-duplex and needs a
direction-control GPIO, which RS-232 does not. No RS-485 transceiver datasheet was read.

### 1.4 Protocol documentation

- **Chafon:** the datasheet is public [1]. The SDKs on the download page, including
  "CF661.691.6A1.6D1 SDK.rar", are behind a password; the request returned "Password is
  wrong" [3]. The command set is not published. An MIT-licensed third-party Python library
  speaks Chafon's older protocol and lists the CF-RU5106 and CF-RU5112 integrated readers [4].
  Whether the CF661 speaks the same protocol is NOT DETERMINABLE. Neither the CF-RU5106 nor the
  CF-RU5112 is in Chafon's current download list [3]; whether they are still sold is NOT
  DETERMINABLE.
- **Yanzeo:** the SR681 user manual (27 lines of text) and the series manual describe the
  Windows demo program, not the command set [5]. A 22 MB "Demo-SDK" is attached to the SR681
  page [5] and was not opened. Whether it holds a protocol document is NOT DETERMINABLE.
- **FEIG:** host APIs for Windows, Linux, Android, Raspberry Pi and Python are listed [9].

### 1.5 Cost

| Line | Custom build | Option 1 (SR691) |
|---|---|---|
| UNO R4 WiFi | $27.50 | $27.50 |
| Screw shield | $14.95 | $14.95 |
| Reader | $309.95 | $316.99 [8] |
| Antenna | $149.00 | — |
| Coax | $57.00 | — |
| Pololu 5 V regulator | $23.95 | — |
| **Priced total** | **$582.35** | **$359.44** |
| Unpriced, removed | CAB.011 pigtail, SN74AHCT125N, dividers | — |
| Unpriced, added | — | MAX3232I and its four capacitors, or a breakout |
| Unpriced, both | DDR-30G-12, TVS, fuse, enclosure, glands | Same |

Option 1 saves **$222.91** on priced lines. With the SR681 instead, the total is $452.64,
saving $129.71. Inference: the unpriced lines it removes outweigh the transceiver it adds, so
the real saving is somewhat larger.

### 1.6 Risks

- **The vendor documents contradict themselves.** The SR691 page title says IP67 and its body
  says IP55. Its page says 9 dBi and its spec PDF says 8 dBi [6]. The SR681 spec PDF gives the
  frequency range as "960MHz～960MHz" [5]. The R781 manual calls the antenna circular in one
  section and "horizontally polarized" in another [7]. Treat every figure as unconfirmed until
  the unit is in hand.
- **No FCC ID on record.** None of the Chafon or Yanzeo documents read states an FCC ID
  [1], [2], [5]–[7]. fccid.io, fcc.report and the FCC's own search page refused automated
  requests here (HTTP 403). Whether either reader holds a US grant is NOT DETERMINABLE. These
  are up-to-30 dBm transmitters with 8–9 dBi antennas.
- **Supply voltage on a 12 V tractor.** The SR681's 12 V maximum [5] and the R781's 7–12 V
  wiring note [7] sit below a charging tractor's bus. Inference: a running 12 V system sits
  around 14 V, so these must run from the regulated output of the DDR-30G-12, which the custom
  build already carries (`findings/2026-10-07-rfid-hardware-recommendation.md:327-330`). That
  converter gives 2.5 A in total and also feeds the board. The CF661 takes 9–24 V [1], but its
  actual draw is not published; Chafon ships a 2 A adapter with it.
- **Cold.** The CF661 and CF561 stop at −10 °C [1], [2], and the SR681 at −20 °C [5]. The
  M7E is rated to −40 °C (`findings/2026-10-07-rfid-hardware-recommendation.md:161`). The yard's
  winter low is not recorded here.
- **Stray bags.** The C610 was chosen for its 70° beam and 20 dB front-to-back
  (`findings/2026-10-07-rfid-hardware-recommendation.md:237-241`). No integrated reader read
  publishes a beamwidth or front-to-back ratio. Power is adjustable on all of them, and that is
  the only read-zone control. The antenna cannot be swapped on the Chafon or Yanzeo units read.
- **The priced SR691 listing does not match the maker's page.** Added 2026-10-14 from the
  adversarial review and checked against the listing. The Newegg listing [8] gives
  "Interfaces(USB, RS485, Wiegand26/34)" with **no RS-232**, and a supply of **VCC min 8, typ 9,
  max 12 V**. Yanzeo's own page lists RS-232 [6]. The $316.99 unit would need an RS-485
  transceiver, not the MAX3232 of §1.3, and a regulated 12 V supply like the SR681.
- **Supply chain.** Both priced listings are third-party Newegg sellers ("Cozylux anky", ships
  from Hong Kong; "Tactifans Outdoor Sport Goods") [8]. Returns and lead time were not read.
  The maker's own sites say "Call for Price" or ¥0.00.

---

## 2. Option 2: a Bluetooth reader paired to the phone

### 2.1 Candidates read

| Reader | Price | Sealing | Operating temp. | Host interface | Source |
|---|---|---|---|---|---|
| Geenfc A7 (pocket reader) | **$126.99** (Newegg, third-party) | not stated | not stated | USB-C, Bluetooth 5.0; Android, iOS, Windows. **0–18 dBm, 0–30 cm read** | [18] |
| CSL CS108 sled | **$1,184.00** | IP54 | −20 to +55 °C | CSL Bluetooth API; MIT-licensed SDKs for Android, iOS, Flutter, .NET [15] | [14] |
| CSL CS710S sled | $1,222.00 | IP54 | not read | CSL Bluetooth and USB byte-stream API | [14] |
| Zebra RFD40 Premium sled | $1,326.41 | IP54 | −10 to +50 °C | "Host API: Not published"; Bluetooth 5.3, paired to Zebra TC21/TC26 or EC50/EC55 | [13] |
| TSL 1128 | $1,369.00 | IP54 | −10 to +40 °C | ASCII 2.0 | [16] |
| Chainway R6 sled | NOT DETERMINABLE (no retailer reached) | IP54 | −20 to +50 °C | BLE 5.0; SDK for Android and iOS (Java, Objective-C); 1–30 dBm in 1 dB steps | [17] |

**Web Bluetooth.** Chrome on Android supports it from version 56. Safari, Safari on iOS and
Firefox do not [12]. So a web app can talk Bluetooth only on an Android phone in Chrome. The
vendor SDKs are native Android and iOS libraries. Inference: Web Bluetooth would mean
reimplementing a vendor's Bluetooth protocol in JavaScript. CSL publishes its implementations
as MIT source [15], which makes that possible for the CS108 and CS710S. Nobody was found to
have done it. The repository carries an Android Bluetooth terminal profile (`BRIEF.md:204`).
Inference: an Android phone has been used. Which phone and browser the app runs on today is
NOT DETERMINABLE from this repository.

### 2.2 What it changes

This option **reverses the ruled design**, so it needs Alex's ruling before anything else.

- **Who reads the tag.** The phone reads it, not the board. Alex ruled that "the board reads
  the tags and the weight in that same step" (`notes/rfid-inventory-discussion.md:21-23`). Here
  the read and the weighing are two acts, and the board cannot confirm that the scanned bag is
  the bag on the forks.
- **The operator's job.** Inference: the bag is on the loader and the operator is in the seat.
  A 30 cm pocket reader means getting down to the bag. A sled reading several metres reaches
  from the seat, and also reaches every other bag in the yard. Mounting and operating positions
  are out of scope (`notes/rfid-inventory-discussion.md:65`), so this stays an inference.
- **Multi-tag refusal.** The board can no longer refuse on several tags: it never sees them.
  The app would have to run the decision that `rfid::decide` runs on the board today
  (`notes/2026-10-07-rfid-record-contract.md:174`).
- **The MQTT contract.** `tag` on the record command stops being "the chosen tag, on the retry
  only" (`notes/2026-10-07-rfid-record-contract.md:85`). It becomes required on every record,
  and the board takes it on trust. The `NO TAG`, `MULTIPLE TAGS`, `TAG NOT IN VIEW` and
  `NO READER` refusals and the `tags` / `tags_seen` ack fields leave the board (`:143-150`).
  The record event keeps its shape: the board still publishes one complete record with the
  weight and the app's tag (`notes/rfid-inventory-discussion.md:30-34`). That is a contract
  change, and the contract is approved and handed to the consumer
  (`notes/2026-10-07-rfid-record-contract.md:3`).
- **The board.** The reader, antenna, coax, regulator and level interface all leave it. Whether
  a new R4 is still wanted at all, or the in-service board takes the firmware change, is Alex's
  platform ruling (`findings/2026-10-07-rfid-hardware-recommendation.md:44-48`), not this
  document's.

### 2.3 Cost

| Phone-side reader | Reader | Board side ($0 if the in-service board is kept; $42.45 for R4 + shield) | Total |
|---|---|---|---|
| Geenfc A7 | $126.99 | $0–42.45 | **$126.99–169.44** |
| CSL CS108 | $1,184.00 | $0–42.45 | $1,184.00–1,226.45 |
| Zebra RFD40 Premium | $1,326.41 | $0–42.45 | $1,326.41–1,368.86 |
| TSL 1128 | $1,369.00 | $0–42.45 | $1,369.00–1,411.45 |

### 2.4 Risks

- **It reopens a closed ruling** and an approved contract. That is a cost in itself.
- **The cheap end is a pocket reader, not a field tool.** The A7 listing gives no sealing or
  temperature rating and a 30 cm range [18]. The sleds with ratings are IP54 and all cost more
  than twice the custom build.
- **Web Bluetooth is Android-and-Chrome only** [12], and no vendor ships a Web Bluetooth SDK
  among those read.

---

## 3. Option 3: a COTS loader-scale system

- **Loadrite** sells loader scales (L5000, L3180 SmartScale) and tractor and forklift scales
  [19]. Its pages render client-side and gave no specifications or prices here.
- **Topcon (formerly RDS):** Weighlog α10 stores "up to 30 products, 30 customers and up to 5
  different recipes" and runs on 10–30 V DC [20]. Weighlog 300 runs on 11–28 V DC, with
  instrument sealing to IP67 [21]. Neither brochure mentions RFID.
- **Price class: NOT DETERMINABLE.** No maker page read publishes a price; they sell through
  dealers. Weigh-Tronix and Griffith Elder were not reached.

**What it would replace:** the weighing half of PLC #4, meaning the transducer reading, the
loop, the board and the LCD. That is the machine already in service with its calibration stored
(`CLAUDE.md`, *How work moves here*). Neither system read offers RFID, MQTT or the tally
contract. Inference: it would add a second weighing system and still leave the RFID problem
unsolved. Not pursued further.

---

## 4. Option 4: cheaper tags

Every price below is from the tier table on the product page. A tier is a pack size: 150 Duo
tags means the "150 Tags" option.

| Tag | Smallest tiers | 150–200 tags | 600 tags | Fixing | Temp. | EPC unique? | Source |
|---|---|---|---|---|---|---|---|
| **HID Sentry Duo** (baseline) | 150 @ $4.60; 600 @ $3.32; 2,100 @ $2.10; 5,100 @ $1.86 | **$690.00** (150) | **$1,992.00** | Adhesive or **zip-tie slot** | −50 to +85 °C | **"Not Guaranteed Unique"** | [22] |
| **Xerafy Data Trak II** | 200 @ $2.98; 1,000 @ $2.07; 5,000 @ $1.40 | **$596.00** (200) | $1,788.00 (3 × 200) | **Cable ties** or adhesive; hard tag | −40 to +85 °C | Unique | [23] |
| **Fujitsu WT-A543** | 25 @ $1.60; 100 @ $1.22; 500 @ $1.02 | **$244.00** (2 × 100) | $632.00 (500 + 100) | **Sewn,** heat-sealed or pouch | −20 to +50 °C | Unique, random | [24] |
| HID Sentry Slim | 150 @ $3.80; 600 @ $2.74 | $570.00 (150) | $1,644.00 | Adhesive only; for metal | −50 to +85 °C | Not guaranteed unique | [27] |
| Xerafy TEX silicone (the custom build's alternative) | 1,000 @ $1.45; 5,000 @ $0.98 | — (1,000 minimum) | — | Sewn | −40 to +85 °C | Unique | [25] |
| Xerafy Pod Trak 60 | 2,000 @ $1.27; 10,000 @ $1.01 | — (2,000 minimum) | — | Zip ties, rivets, screws | −40 to +55 °C | Unique | [26] |
| **RFIDsolution RSCT-U8-AN-10330** (added 2026-10-14; **ruled**) | 10 for $51; 100 for $135; 500 for $474; 1,000 for $790 | **$270.00** (2 × 100) | $609.00 (500 + 100) | **Cable tie, 400 mm, disposable**; ABS and nylon, 103 × 30 mm | −25 to +60 °C | 128-bit EPC; uniqueness not stated | [28] |

**RSCT-U8-AN-10330, from its datasheet [28]:** NXP UCODE 8, 860–960 MHz, EPC Gen2v2, read range
"Up to 8 m". The characteristics line reads "UV protection, suitable for outdoor use". The
description also calls it waterproof, but **no IP rating is given**. Mounting is "Tie onto
projects", for plastic, wood and liquid surfaces. The tag type is "Disposable": the tie is
single-use, so reuse means a new tie. Lead time is 15 working days [28].

**Findings.**

- **Superseded 2026-10-14:** the bullet below is false. The Fujitsu row in the table above
  ($1.22 at 100, $1.02 at 500) already beat $1.86, and so does the RSCT-U8-AN-10330 ($1.35 at
  100, $0.948 at 500) [28]. Found by the adversarial review.
- **No outdoor tag on the pages read beats $1.86 at 150–600.** The $1.86 Duo price, and the
  $0.98 Xerafy TEX price the custom build cites
  (`findings/2026-10-07-rfid-hardware-recommendation.md:304`), are both 5,000-plus tiers [22],
  [25].
- **The Xerafy Data Trak II is the like-for-like saving.** It is a hard tag that takes a cable
  tie, like the Duo, and costs $596.00 for 200 against $690.00 for 150. Its IP rating is not on
  the product page and its datasheet is image-only, so its sealing is NOT DETERMINABLE. Its
  off-metal read range is up to 1.8 m [23], which covers the ~1 m design range
  (`notes/rfid-inventory-discussion.md:19`).
- **The Fujitsu WT-A543 is the cheapest, if a sewn tag suits the bag.** It is built for
  laundries: 60-bar extraction, bleach, 200 washes. Its datasheet gives **"200 washing cycles…
  or 3 years from shipping date, whichever comes first"**, a −20 °C lower limit, and no UV or
  IP rating [24]. Reuse means unpicking it. Bag material is out of scope
  (`notes/rfid-inventory-discussion.md:64`).
- **The Duo's EPC is "Not Guaranteed Unique"** [22]. The contract records the EPC as the bag's
  identity (`notes/2026-10-07-rfid-record-contract.md:210`). The Data Trak II and the WT-A543
  ship with unique EPCs [23], [24]. This holds whatever reader is chosen.
- **Superseded 2026-10-14:** the bullet above treats EPC uniqueness as a reason to choose a
  tag. It is not one. The EPC is writable, and the board's reader library has `writeTagEPC()`,
  `readTID()` and `readUID()` (`SparkFun_UHF_RFID_Reader.h:138`, `:152-153`), plus an
  `Example3_Write_EPC` sketch [29]. The write cannot pick which tag hears it ("This function
  doesn't control which tag hears the command", `SparkFun_UHF_RFID_Reader.cpp:326`) [29], so
  tags are written one at a time.
- **Requirement, whatever tag is chosen:** each tag gets a unique EPC written once before use.

---

## 5. Comparison

| Option | Replaces | Priced hardware vs $582.35 | + tags (150 Duo $690 / 200 Data Trak II $596) | Firmware | Contract | Rulings | Main risk |
|---|---|---|---|---|---|---|---|
| 0 Custom M7E | — | $582.35 | $1,272.35 / $1,178.35 | Reader driver on the SparkFun library | As approved | As ruled | Unpriced lines; 5 V ripple question |
| 1 Integrated reader (Yanzeo SR691) | M7E, C610, coax, pigtail, regulator, buffer | **$359.44** (−$222.91) + transceiver | $1,049.44 / $955.44 | Driver from an unpublished protocol | **Unchanged** if RSSI per tag | **Unchanged** | Contradictory specs; no FCC ID; IP55 or IP67; third-party seller |
| 1 Integrated reader (Yanzeo SR681) | Same | $452.64 (−$129.71) + transceiver | $1,142.64 / $1,048.64 | Same | Same | Same | 12 V maximum on a 14 V bus; −20 °C |
| 1 Integrated reader (Chafon CF661) | Same | NOT DETERMINABLE | — | Driver from a password-gated SDK | Unchanged; RSSI is reported | Unchanged | −10 °C lower limit; price unknown |
| 1 Integrated reader (FEIG MRU400i) | Same | $1,559.45 + 24 V boost | $2,249.45 / $2,155.45 | Driver from published APIs | Unchanged | Unchanged | Cost; 24 V |
| 2 Phone sled (Geenfc A7 to TSL 1128) | All RFID on the board | $126.99–1,411.45 | $816.99–2,101.45 / $722.99–2,007.45 | Board stops reading tags | **Changes** | **Reverses "one press"** | Bag scanned is not proven to be the bag weighed; Web Bluetooth only on Android Chrome |
| 3 Loader scale | The weighing half of PLC #4 | NOT DETERMINABLE | — | Replaced, not changed | Lost | Lost | No RFID in either brochure read |
| 4 Cheaper tags | Tag line only | — | Saves $94 against 150 Duo | None | None | None | Data Trak II sealing not established; WT-A543 is sewn and rated to −20 °C |
| **0b Ruled 2026-10-14:** M7E on-board antenna, 200 RSCT tags | C610, coax, pigtail, bulkhead, Duo tags | **$376.35** (−$206.00) | **$646.35** (200 RSCT at $270) | Unchanged | As approved | As ruled; reader within ~0.5 m of the bag | Read range on the loader not yet bench-tested; tag IP rating not stated |

---

## 6. Recommendation

**Superseded 2026-10-14:** Alex ruled "Cheap tags, no antenna" (coordinator's option text, not
his words). The build is §0. The text below is the recommendation he was shown before that, kept
as the record. Where it says the Data Trak II "brings unique EPCs", read §4: uniqueness is a
one-time write, not a tag property to buy.

**Keep the custom M7E build, and price the tags at their real tier.** Reason in one line: the
only COTS route that keeps the ruled design saves about $220, and buys that with vendor
documents that contradict themselves, an unpublished command set and no FCC ID on record.

- **Tags first.** At 150 bags the tags cost more than the reader. Either budget $690 for 150
  Sentry Duo, or consider 200 Xerafy Data Trak II at $596 once its sealing is confirmed. The
  Data Trak II also brings unique EPCs.
- **Option 1 is the one worth reopening, on two conditions.** If Yanzeo or Chafon supplies its
  serial command set, showing per-tag RSSI in command mode, and an FCC ID, before purchase, an
  integrated reader becomes the simpler machine: one sealed box and one cable in place of
  reader, pigtail, bulkhead, coax and antenna, and no 5 V ripple question. It still needs a
  bench check of the read zone at ~1 m, since no beamwidth is published.
- **Option 2 only if Alex wants to reopen "one press does it all".** It is cheaper only at the
  pocket-reader end, and that end has no field rating.
- **Option 3 is not a fit.** It replaces a working, calibrated scale and adds no RFID.

---

## 7. What could not be established

- **Chafon prices** for the CF661 and CF561. The maker lists ¥0.00; Amazon, eBay, AliExpress and
  Alibaba refused automated requests or showed a captcha.
- **FCC IDs** for any Chafon or Yanzeo reader. No document read states one, and the FCC lookups
  returned HTTP 403.
- **Command sets.** Chafon's SDKs are password-protected. Yanzeo's demo SDK was not opened.
  Whether the CF661 speaks the protocol of the older CF-RU5106 / CF-RU5112 that the MIT library
  implements [4].
- **Per-tag RSSI** on the Yanzeo SR681 and SR691 serial ports.
- **True sealing of the Yanzeo units:** IP55 or IP67 for the SR691; no IP figure in the SR681
  spec PDF.
- **Beamwidth and front-to-back** of any integrated reader read, so how well each rejects a
  second bag at ~1 m.
- **Current draw** of the CF661, so whether it fits beside the board on the DDR-30G-12's 2.5 A.
- **Invelion / YR-series readers:** the site's TLS certificate had expired, and it was not read
  around that.
- **Chainway R6 price;** the Zebra RFD40 operating temperature beyond the reseller page.
- **Which phone and browser** the app runs on, which decides whether Web Bluetooth is possible.
- **Loader-scale prices** (dealer-only), and Weigh-Tronix and Griffith Elder, not reached.
- **Xerafy Data Trak II IP rating and UV life** (datasheet is image-only); UV life of every tag
  in §4.
- **Cheap Chinese zip-tie tags:** a "TF9620" cable-tie seal tag is listed at $51.71 on Newegg,
  but the listing states no pack quantity, chip or rating, so it is not costed.
- **The yard's winter low,** which decides whether −10 °C or −20 °C limits matter.
- **The on-board antenna's read range on the loader,** through the enclosure, at the record
  position. SparkFun's "one to two feet" [30] is a bench figure, not a field one. (Added
  2026-10-14.)
- **The RSCT-U8-AN-10330's IP rating** (none given [28]), its read range on a bag, and its winter
  performance below its −25 °C limit. (Added 2026-10-14.)

---

## 8. Alex's tag, SLON SA-223, checked 2026-10-14

**Why this exists.** On 2026-10-14 Alex said *"check out these tags"* and pasted a listing for the
SLON SA-223 "UHF Ties RFID Tags": Alien H3 chip, 860–960 MHz, −40 to +65 °C working,
">5m (reader Alien ALR9900+, EIRP 4W)". This section checks that listing against the ruled tag.
**It does not change the ruled build or any total in §0.** Whether to swap tags is Alex's call.

### 8.1 Sources and their limits

- **The maker's pages.** SLON's site carries the SA-223 on two product pages, each in two
  languages: a French page [31] and a Spanish page [32]. The English versions of both return
  **HTTP 410 Gone**. The site footer gives "SLon Technology Co.,Limited", Guangzhou [31].
  **The two pages give different prices for the same model** (table below).
- **The French page has a second product's specification mixed into it.** Under the SA-223
  price table it describes the "Tamper Proof RFID Tag SA-224", a paper windscreen label rated
  −20 to +85 °C, "anti-UV", with a "64位" UID [31]. **Those are SA-224 facts. None of them is
  used here for the SA-223.**
- **No SA-223 datasheet was found.** The product pages are the only SA-223 source. The chip
  facts come from Alien's Higgs-3 datasheet [33], its supplement [34] and its product page [35].
- **Inference:** Yanzeo's documents name "SLon Technology Co., Ltd." [5], and SLonParts, also in
  Guangzhou, sells Yanzeo readers. SLON and Yanzeo are probably one company or group. That
  matters because §1.6 found Yanzeo's documents contradicting themselves.

### 8.2 Comparison

| | SLON SA-223 [31], [32] | RSCT-U8-AN-10330 (ruled) [28] |
|---|---|---|
| Chip | Alien Higgs-3 [33] | NXP UCODE 8 |
| Price, 10 | $7.00 (FR page) or $13.00 (ES page) | $51.00 (10-pack) |
| Price, 200 | **$140.00 (FR) or $260.00 (ES)**: $0.70 or $1.30 a tag | **$270.00** (2 × 100), $1.35 a tag |
| Price, 500 | $350.00 (FR) or $650.00 (ES) | $474.00 |
| Price tiers | FR: $0.70 for 1–1,000, $0.50 to 5,000. ES: $1.30 for 1–1,000, $1.20 to 5,000 | 10 / 100 / 500 / 1,000 packs |
| MOQ | Tiers start at 1. No other minimum is stated | 10-pack |
| Lead time / shipping | "3 día" / "3 jour" against every tier. Site footer: "Global free shipping". Packed 500 a box | 15 working days |
| Operating temp. | **−40 to +65 °C**; storage −40 to +85 °C. Chip: −50 to +85 °C [33] | −25 to +60 °C |
| IP rating / sealing | **Not stated** | Not stated; described as waterproof |
| UV | **Not stated.** Body is "polypropylene" | **"UV protection, suitable for outdoor use"** |
| Tie | 290 × 24 × 8 mm overall; inlay 40 × 15 × 0.3 mm. **Whether it is one-use or releasable is not stated** | 103 × 30 mm tag, 400 mm tie, "Disposable" |
| EPC | Listing says "EPC 128". Chip: 96 bits by default, extensible to 480 [33], [35]. Writable NVM, 100,000 cycles [33] | 128 bits |
| TID / serial | **64-bit factory-programmed serial "that cannot be altered"** in TID bank words 2–5 (20h–5Fh), after a 32-bit class and model header [33] | 96-bit factory-locked TID [37]. Whether it carries a unique serial was not established here |
| User memory | 512 bits [33] | 0 bits (UCODE 8) [37] |
| Chip read sensitivity | −20 dBm with a 2 dBi dipole [33]; −18 dBm minimum communication power [34] | −23 dBm [37] (third-party; NXP's datasheet refused automated download) |
| Stated read range | ">5m", Alien ALR-9900+ reader at 4 W EIRP | "Up to 8 m", conditions not stated |
| Band | 860–960 MHz; ISO/IEC 18000-6C | 860–960 MHz; ISO/IEC 18000-63, Gen2v2 |

### 8.3 Reads on the ruled build

- **Protocol.** The M7E supports only EPC Gen2 (ISO 18000-6C) and says so [36] §8.5. Higgs-3
  meets EPCglobal Gen2 v1.2.0 and ISO/IEC 18000-6C [33]. The M7E runs Gen2 Read, Write, Lock,
  Kill, Block Write and Block Permalock [36] Table 11. **Inference:** the M7E reads and writes
  an H3 tag with standard Gen2 commands. Alien's custom LoadImage command [33] is not needed.
- **Writing the EPC.** `writeTagEPC()` writes the given bytes to EPC bank word 2 and stops
  there (`SparkFun_UHF_RFID_Reader.cpp:327-333`) [29]. It does not write the PC word, which
  holds the EPC length. **Inference:** if the tag ships with a 128-bit EPC as listed, a 12-byte
  write leaves the last 32 bits as shipped and the tag still reports 128 bits. Write the full
  16 bytes. The same holds for the RSCT tag, which is also 128-bit.
- **Reading the serial.** `readUID()` reads TID bank from word 2
  (`SparkFun_UHF_RFID_Reader.cpp:406-412`) [29]. On Higgs-3 that is where the unalterable
  64-bit serial sits [33]. **Inference:** the board can read each H3 tag's serial with no
  library change. The contract keys on the EPC, not the TID
  (`notes/2026-10-07-rfid-record-contract.md:210`), so this is useful for commissioning, not
  required.
- **Range.** The listing's ">5m" is at 4 W EIRP (36 dBm) with a fixed reader. The M7E puts out
  27 dBm at most [36] §9. JADAK quotes it at "over 4.5 meters" with a 6 dBi antenna at 33 dBm
  EIRP [36] §9. On the on-board PCB antenna SparkFun gives "roughly one to two feet" [30].
  The PCB antenna's gain is not published, so the M7E's EIRP on it is NOT DETERMINABLE.
  **Inference:** at 0.3–0.6 m the reader's antenna is the limit, not the tag, and neither
  tag's stated range predicts reads on this build. On chip sensitivity alone the H3 is 3 dB
  below the UCODE 8. For otherwise identical tags that is about 0.7 times the range
  (10^(−3/20)). The two tags' antennas differ, so this is a direction, not a prediction.
- **Higgs-3 quirks with the M7E.** The M7E user guide names no tag chip and documents no
  chip-specific behaviour [36]. SparkFun's write example sets 5 dBm read and write power and
  warns that higher values may brown out a USB port (`Example3_Write_EPC.ino:61`, `:64`) [29].
  That applies to any tag.
- **The bench test decides.** Put both tags in the on-board antenna bench test at the record
  position (§0, *Open before committing*).

### 8.4 Verdict

**Cheaper and rated colder on paper, but UV and sealing are unstated and the maker's own pages
disagree on price. It is a candidate for the outdoor trial, not a like-for-like swap.** At 200
tags it costs $140 or $260 against $270. It is rated to −40 °C where the RSCT tag stops at
−25 °C, and it carries a 64-bit serial. Against that, nothing on its pages says UV-stabilised,
waterproof or IP-rated. The only "anti-UV" text belongs to a different product. The tie is
shorter (290 mm overall against 400 mm). Whether it releases is not stated.

### 8.5 NOT DETERMINABLE

- **Which SA-223 price is current:** $0.70 or $1.30 a tag. Both pages are live, and the English
  pages are gone. Ask the seller to quote 200 delivered to the US.
- **IP rating, UV resistance and UV life** of the SA-223. None is stated.
- **Whether the tie is one-use or releasable.**
- **Shipping cost and terms to the US.** "Global free shipping" is site-wide footer text.
  Whether it covers this item is not stated.
- **US 902–928 MHz performance.** The listing gives 860–960 MHz and one range figure, without
  saying which region's band it was measured in. The tag antenna's tuning is not published.
- **Whether a shipped tag's EPC is 96 or 128 bits.** The listing says 128 and the chip
  defaults to 96.
- **The on-board antenna's EIRP**, and so the read range of either tag on this build.
- **Whether the UCODE 8's TID carries a unique serial.** NXP's datasheet was not read.

---

## References

[1] Shenzhen Chafon Technology, "PRIME CF661 UHF integrated reader," datasheet (CF661 EN), and
"CF661 Prime UHF 6dbi integrated reader," product page. [Online]. Available:
https://www.chafontech.com/filedownload/110660 ; https://www.chafontech.com/productinfo/1069318.html

[2] Shenzhen Chafon Technology, "CF561 UHF integrated reader," datasheet (CF561-EN), and
"CF561 Access UHF 6dbi integrated reader," product page. [Online]. Available:
https://www.chafontech.com/filedownload/110658 ; https://www.chafontech.com/productinfo/1069199.html

[3] Shenzhen Chafon Technology, "Download," SDK and datasheet index; SDK files require a
password. [Online]. Available: https://www.chafontech.com/Download

[4] wabson, *chafon-rfid: Read RFID data from Chafon UHF readers*, MIT License, `README.md`.
[Online]. Available: https://github.com/wabson/chafon-rfid

[5] Yanzeo (SLon Technology Co., Ltd.), "Yanzeo SR681 SR682 6m Long Range UHF RFID Reader," product
page; "Yanzeo-SR681-UHF-RFID-Reader-Specifications"; "Yanzeo-SR681-RFID-Reader-User-Manual";
"Yanzeo-UHF-RFID-Reader-Series-User-Manual." [Online]. Available:
https://www.yanzeo.com/yanzeo-sr681-sr682-6m-long-range-uhf-rfid-reader-outdoor-ip67-8dbi-antenna-rs232-rs485-wiegand-output-uhf-integrated-reader-network-port.html ;
https://www.yanzeo.com/amfile/file/download/file/41/product/63/ ;
https://www.yanzeo.com/amfile/file/download/file/42/product/63/ ;
https://www.yanzeo.com/amfile/file/download/file/43/product/63/

[6] Yanzeo, "Yanzeo SR691 SR692 UHF RFID Writer Reader 10M Long Distance," product page, and
"Yanzeo-SR691-UHF-RFID-Reader-Specifications." [Online]. Available:
https://www.yanzeo.com/yanzeo-sr691-sr692-uhf-rfid-writer-reader-10m-long-distance-outdoor-ip67-9dbi-antenna-rs232-rs485-wiegand-rj45-uhf-integrated-reader.html ;
https://www.yanzeo.com/amfile/file/download/file/47/product/135/

[7] Yanzeo Smart Co., Ltd., "R781/R782-8DB Long distance UHF electronic tag reader, an instruction
manual," and R781 product page. [Online]. Available:
https://www.yanzeo.com/amfile/file/download/file/32/product/221/ ;
https://www.yanzeo.com/r781-uhf-rfid-reader-6m-long-range-outdoor-ip67-8dbi-antenna-usb-rs232-rs485-wiegand-output-uhf-integrated-reader.html

[8] Newegg Marketplace, listings: "SR681 UHF RFID Reader 6m Long Range … UHF Integrated" and
"Yanzeo SR691 UHF RFID Reader 9m Long Distance Outdoor IP67 … UHF Integrated Reader," product
pages and prices. [Online]. Available: https://www.newegg.com/p/0Z2-06S3-00DN6 ;
https://www.newegg.com/p/3C6-0189-00390

[9] atlasRFIDstore, "FEIG MRU400i Mid-Range Integrated UHF RFID Reader," product page, price and
specifications, with the FEIG 24V-B Power Supply Kit. [Online]. Available:
https://www.atlasrfidstore.com/feig-mru400i-mid-range-integrated-uhf-rfid-reader/

[10] atlasRFIDstore, "Zebra FXR90 Ultra-Rugged Integrated RAIN RFID Reader, 4-Port," product page,
price and specifications. [Online]. Available:
https://www.atlasrfidstore.com/zebra-fxr90-ultra-rugged-integrated-rain-rfid-reader-4-port/

[11] Texas Instruments, "MAX3232 3-V to 5.5-V Multichannel RS-232 Line Driver and Receiver With
±15-kV ESD Protection," SLLS410O, Jan. 2000, rev. Jun. 2021, §1, §6.3, §6.7. [Online].
Available: https://www.ti.com/lit/ds/symlink/max3232.pdf

[12] MDN, *browser-compat-data*, `api/Bluetooth.json`, CC0. [Online]. Available:
https://github.com/mdn/browser-compat-data/blob/main/api/Bluetooth.json

[13] atlasRFIDstore, "Zebra RFD40 Premium UHF RFID Sled," product page, price and specifications.
[Online]. Available: https://www.atlasrfidstore.com/zebra-rfd40-premium-uhf-rfid-sled/

[14] atlasRFIDstore, "CSL CS108 Bluetooth UHF RFID Handheld Sled Reader" and "CSL CS710S Bluetooth
UHF RFID Handheld Sled Reader," product pages, prices and specifications. [Online]. Available:
https://www.atlasrfidstore.com/csl-cs108-bluetooth-uhf-rfid-handheld-sled-reader/ ;
https://www.atlasrfidstore.com/csl-cs710s-bluetooth-uhf-rfid-handheld-sled-reader/

[15] Convergence Systems Ltd. (CSL), GitHub organisation `cslrfid`: CS108 and CS710S SDKs and
demo apps for Android, iOS, Flutter and .NET, MIT License. [Online]. Available:
https://github.com/cslrfid

[16] atlasRFIDstore, "TSL 1128 Bluetooth UHF RFID Reader," product page, price and
specifications. [Online]. Available: https://www.atlasrfidstore.com/tsl-1128-bluetooth-uhf-rfid-reader/

[17] Chainway, "R6 UHF RFID Sled Reader," product page and specifications. [Online]. Available:
https://www.chainway.net/Products/Info/52

[18] Newegg Marketplace, "Geenfc A7 Portable UHF RFID Reader Writer, Type-C USB & Bluetooth 5.0
Interface," product page and price. [Online]. Available: https://www.newegg.com/p/3C6-00UH-010E7

[19] Loadrite, "Loader Scales," product page. [Online]. Available:
https://www.goloadrite.com/loader-scales

[20] Topcon Positioning Systems, "Weighlog α10," brochure 7010-2324. [Online]. Available:
https://www.topconpositioning.com/content/dam/topcon_digital_asset_hub/collateral/brochures/topcon_weighloga10_7010-2324_enUS23broc.pdf

[21] Topcon Positioning Systems, "Weighlog 300," brochure 7010-2335. [Online]. Available:
https://www.topconpositioning.com/content/dam/topcon_digital_asset_hub/collateral/brochures/topcon_weighlog300_7010-2335_enUS23broc.pdf

[22] atlasRFIDstore, "HID Sentry Duo UHF RFID Tag | M750," product page, quantity tiers and
specifications. [Online]. Available: https://www.atlasrfidstore.com/hid-sentry-duo-uhf-rfid-tag-m750/

[23] atlasRFIDstore, "Xerafy Data Trak II RFID Tag | Monza 4QT," product page, quantity tiers and
specifications; Xerafy, "Data Trak II M4QT," datasheet. [Online]. Available:
https://www.atlasrfidstore.com/xerafy-data-trak-ii-rfid-tag-monza-4qt/ ;
https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/Xerafy/ATLAS%20Xerafy%20Data%20Trak%20II%20M4QT%20Datasheet.pdf

[24] atlasRFIDstore, "Fujitsu WT-A543 UHF RFID Linen Tag," product page and quantity tiers;
Fujitsu, "UHF RFID Tag WT-A543," datasheet. [Online]. Available:
https://www.atlasrfidstore.com/fujitsu-wt-a543-uhf-rfid-linen-tag/ ;
https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/Fujitsu/ATLAS%20Fujitsu%20WT-A543%20UHF%20RFID%20Laundry%20Tag%20Data%20Sheet.pdf

[25] atlasRFIDstore, "Xerafy TEX TRAK UHF RFID Silicone Laundry Tag | 55 mm x 12 mm | NXP UCODE 9,"
product page and quantity tiers. [Online]. Available:
https://www.atlasrfidstore.com/xerafy-tex-trak-uhf-rfid-silicone-laundry-tag-55-mm-x-12-mm-nxp-ucode-9/

[26] atlasRFIDstore, "Xerafy Pod Trak 60 RFID Tag | NXP UCODE 9," product page and quantity tiers.
[Online]. Available: https://www.atlasrfidstore.com/xerafy-pod-trak-60-rfid-tag-nxp-ucode-9/

[27] atlasRFIDstore, "HID Sentry Slim UHF RFID Tag | M750," product page and quantity tiers.
[Online]. Available: https://www.atlasrfidstore.com/hid-sentry-slim-uhf-rfid-tag-m750/

[28] RFIDsolution, "UHF RFID Cable Tie Tag – ABS and Nylon – NXP UCODE 8 – 103×30 mm,
RSCT-U8-AN-10330," product page with quantity prices and lead time, and datasheet. [Online].
Available: https://rfidsolution.com/?p=2545 ;
https://rfidsolution.com/wp-content/uploads/2025/07/UHF-RFID-Cable-Tie-Tag-ABS-and-Nylon-NXP-UCODE-8-103-30-mm-datasheet-RSCT-U8-AN-10330.pdf

[29] SparkFun Electronics, *SparkFun Simultaneous RFID Tag Reader Library*, v1.2.0, MIT License,
`src/SparkFun_UHF_RFID_Reader.h`, `src/SparkFun_UHF_RFID_Reader.cpp`,
`examples/Example3_Write_EPC`. [Online]. Available:
https://github.com/sparkfun/SparkFun_Simultaneous_RFID_Tag_Reader_Library

[30] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M7E Hecto, WRL-24738," product
page. [Online]. Available: https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html

[31] SLon Technology Co., Limited (SLON), "Cravates UHF RFID tags SA-223," product page, French,
with price tiers and delivery time. The English page at the same path returns HTTP 410.
[Online]. Available: http://fr.slonrfid.com/pid14737347/product-detail.htm

[32] SLON Tecnología Co., Ltd (SLON), "UHF Ties Etiquetas RFID," SA-223 product page, Spanish,
with price tiers and delivery time. The English page at the same path returns HTTP 410.
[Online]. Available: http://es.slonrfid.com/pid14901762/product-detail.htm

[33] Alien Technology, "Higgs 3 EPC Class 1 Gen 2 RFID Tag IC," ALC-360 datasheet, Dec. 21, 2014.
[Online]. Available: http://www.alientechnology.com/wp-content/uploads/ALC-360%20Higgs3%202014-12-21.pdf

[34] Alien Technology, "Higgs 3 IC Datasheet Supplement," §3.1.4, Tables 4.2–4.3, 2006–2013.
Hosted by a third party, RFID Alliance. [Online]. Available:
https://www.rfid-alliance.com/RFIDshop/Alien-Technology-Higgs-3-IC-Datasheet.pdf

[35] Alien Technology, "Higgs 3," product page. [Online]. Available:
https://www.alientechnology.com/products/ic/higgs-3/

[36] JADAK (Novanta), "ThingMagic M7E-HECTO User Guide," Rev. 1.2, Nov. 2023, §8.5, §8.7 Table 11,
§9. [Online]. Available: https://cdn.sparkfun.com/assets/8/5/2/4/d/M7E_HECTO_User_Guide.pdf

[37] Roxtron, "NXP UCODE 8 Datasheet: Specifications, Memory, Protocol," rfid-datasheet.com.
A third-party summary. NXP's own datasheet (SL3S1205_15) refused automated download (HTTP 404)
and was not read. [Online]. Available: https://rfid-datasheet.com/chip/ucode-8
