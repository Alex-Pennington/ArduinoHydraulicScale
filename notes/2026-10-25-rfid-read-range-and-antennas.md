# PLC #4 RFID build: read range, mounting and antennas

**Date:** 2026-10-25. **Scope:** a hardware discussion with Alex (session `plc4-hardware-1025`).
Nothing here changes firmware, and nothing here is a purchase.

**How to read the claims.** A bracketed number cites *References*. *Read from the artifact* means
the "PLC #4 Case Layout" page, version `1792960451-5dfc`. **Inference** marks a conclusion no
source states. **NOT DETERMINABLE** marks something no source reached here settles. Nothing was
measured.

## Ruled by Alex, 2026-10-25

In this session's dialog he chose **"External antenna (Recommended)"** over "Bench test V1 first"
and "Reader in own box". The option was put to him as: *"Reverses the 10-14 'no antenna' choice.
Move one 0 Ω resistor on the reader, fit a sealed ~150 mm patch on the loader facing the bag
(RFMAX R9025, ~$171), run coax to the case. LCD stays facing you."* **That wording is this
session's option text, not his sentence.** His act was choosing it.

What follows from it (inference, for the V2 case and the BOM; nothing here is decided beyond the
ruling):

- The M7E's RF 0 Ω resistor moves to the u.FL position [2]. Hand rework.
- A U.FL-to-SMA(F) bulkhead pigtail passes through the case wall, sealed. The 10-07 finding named
  the Taoglas CAB.011 (`findings/2026-10-07-rfid-hardware-recommendation.md` §3.3); its sealing is
  not rated. V2 needs a hole for it, which V1 does not have (artifact §08: "No antenna passes
  through the wall").
- The reader no longer has to sit on the lid behind a no-metal zone. Where it goes in V2 is open.
- Coax: SMA(M)–SMA(M), the shortest that reaches. Length, routing past the loader pivots, and the
  antenna's bracket on the loader are NOT DETERMINABLE here.
- RF power is set low and tuned at the machine, so a second bag in view does not answer.

## 1. SparkFun's two pages disagree about the on-board antenna

| Source | What it says about the on-board PCB antenna |
|---|---|
| Product page [1] | "a read range of roughly one to two feet"; up to 16 ft (4.9 m) with an external antenna on the U.FL |
| Hookup guide, *External antenna* [2] | "reads tags reliably at only up to a few inches away at the highest read power", "with the PCB antenna positioned away from any solid surfaces" |

The 2026-10-14 ruling ("Cheap tags, no antenna") was put to Alex on the product page's figure
(`findings/2026-10-14-rfid-cots-alternatives.md` §0, "roughly one to two feet" [30] there). The
hookup guide was not cited in either RFID finding. **Inference:** the build needs about 0.5 m
(`findings/2026-10-14-rfid-cots-alternatives.md` §0), and the hookup guide's figure is several
times shorter than that, so the bench test §0 already requires is now the deciding test rather
than a confirmation. Mounting the reader inside a lid, beside a ground plane and other boards, also
breaks the guide's "away from any solid surfaces" condition (inference).

**Caveat (adversarial review, same day):** the guide's URL names the "M7E" reader, not "M7E Hecto".
Whether it describes the same SparkFun carrier and trace antenna as WRL-24738 was not established.
The product page calls its figure fit for "desktop testing and near-field applications" and states
no conditions. The other guide pages give no PCB-antenna range.

## 2. The V1 case points the reader where the LCD points

Read from the artifact:

- The reader screws to bosses on the inside of the **lid**, antenna side toward the lid (§04 step 6).
- The LCD is on the same lid, behind the clear window (§04 step 5).
- The case mounts by its **back** (the floor) to a ROPS post through the bracket (§04 steps 10-11,
  §06).
- §06: mount it "with the RFID face within about 0.5 m of where the bag hangs on the loader".

**Inference:** the lid faces the operator so the LCD can be read, so the reader faces the
operator too. The bag hangs on the loader in front of the operator. With the reader on the lid,
the case can face the bag or the operator, not both. Where on the 5055E the bag hangs at the
record position, and how far that is from any ROPS post, is NOT DETERMINABLE here.

## 3. The ways out

| | What changes | Keeps the 10-14 ruling? |
|---|---|---|
| A | **External antenna.** Move the M7E's RF 0 Ω resistor to the u.FL position [2]. A small sealed patch antenna mounts on the loader facing the bag; coax runs back to the case. The case stays where the operator reads the LCD | No: it reverses "no antenna" |
| B | **Reader in its own small box near the bag**, on its on-board antenna. 5 V and the UART run to it from the main case | Yes, but needs the range in §1 to hold and a UART run of unknown length |
| C | **Bench test the V1 case as printed** at the record position before deciding | Yes |

### A candidate antenna for A: RFMAX R9025-LPF-SSF [3]

902-928 MHz, RHCP, 5 dBic typical, 105° beamwidth both planes, front-to-back 10 dB, SMA female
on the side, 150 × 149.4 × 14.5 mm, 0.23 kg, ABS radome (UV resistant), IP68, −30 to +65 °C,
100 mm VESA holes. The datasheet claims "4 meter read range (typ.)" without naming the reader or
tag. Price: $170.67 at one US retailer [4] when read; that listing says 5.5 dBiC, not 5.

**Licensing (from [5]).** The module delivers up to +27 dBm (§5.1.2). §5.7 allows antennas up to
8.15 dBiL; §5.8 says the FCC modular grant (FCC ID QV5MERCURY7EP) needs no further testing for an
antenna of the same type as one in the filing, at equal or lower gain. Table 5 lists the
MTI-242043, a circular patch at up to 9.5 dBiC. **Inference:** a 5 dBic circular patch fits that
rule by type and gain. The FCC filing itself was not read. Keep people at least 20 cm from the
antenna face (§ RF safety).

Earlier candidates, from `findings/2026-10-07-rfid-hardware-recommendation.md` §3 (superseded by
the 10-14 ruling, kept as the record): Vulcan C610, 250 mm square, 70° beam, 20 dB front-to-back,
$149, plus $57 for 12 ft of coax; Vulcan S9025PL, 132 mm square, 100° beam.

**Inference:** with an external patch at a metre or less, too much range is the risk, not too
little: a second bag in view forces a multi-tag refusal. RF power is set in 0.5 dB steps and
would be turned down at the machine.

## 4. Antenna choice, and the feed from the reader to it

Alex asked for both on 2026-10-25, after the ruling above.

### 4.1 Antennas compared

**Ruled by Alex, 2026-10-25: the Vulcan C610.** In this session's dialog he chose "Vulcan C610
(Recommended)" over "RFMAX R9025". The option was put to him as: *"250 mm square, 0.69 kg.
Narrow 70° beam and 20 dB rejection behind it, so a second bag nearby is less likely to read.
Rated to −40 °C. About $129–149."* That wording is this session's option text, not his sentence.

The build's antenna chain is therefore: M7E RF resistor moved to u.FL → Siretta
ASMG010XD113S11 or ASMG015XD113S11 (SMA female bulkhead, IP67) → SMA male–male Times LMR-195-UF
jumper → Vulcan C610 (VUL-C610-FCC). Jumper length, the antenna's bracket on the loader, and the
pigtail length are NOT DETERMINABLE here.

| | RFMAX R9025-LPF-SSF [3] | Vulcan RFID C610 [6] | Vulcan RFID C912 [7], **unverified** |
|---|---|---|---|
| Size | 150 × 149.4 × 14.5 mm, 0.23 kg | 250 × 250 × 20 mm, 0.69 kg | not read |
| Gain | 5 dBic typ | 6 dBiC | 9.4 dBiC (retailer page) |
| Beam (3 dB) | 105° both planes | 70° both planes | "narrow" (datasheet excerpt) |
| Front-to-back | 10 dB | 20 dB | "excellent" (datasheet excerpt) |
| Polarisation | RHCP | circular | circular |
| Connector | SMA female, side | SMA female, waterproof | not read |
| Sealing | IP68 | IP67 | IP67 |
| Operating temp | −30 to +65 °C | −40 to +70 °C | not read |
| Mount | 100 mm VESA holes | 100 × 100 mm M4 VESA studs | 100 × 100 mm M6 VESA studs |
| Price | $170.67 [4] | $149.00 read 2026-10-07; a search snippet on 2026-10-25 showed $129.00, page not readable | $172.00 (retailer page, not re-read) |

C610 figures are from `findings/2026-10-07-rfid-hardware-recommendation.md` §3.1, read from its
datasheet then; they were not re-read today.

**Inference, for choosing:**

- **Too much range is the risk, not too little.** A second bag in view forces a multi-tag refusal
  (`notes/rfid-inventory-discussion.md`, "Several tags"). The C610's 70° beam and 20 dB
  front-to-back read less of what is beside and behind it than the R9025's 105° and 10 dB.
- **A 70° beam is wide enough at this distance.** Its 3 dB footprint is about 1.4 × the distance
  across: about 0.7 m wide at 0.5 m, 1.4 m at 1 m. One bag fits.
- **The C912 column is unverified.** Its datasheet could not be read, and search listings disagree
  with each other (5.5 vs 9.4 dBic, LHCP, differing beams and sizes). It is set aside for that
  reason, not on its figures.
- **The R9025's case is size and weight.** It is about a third of the C610's area and a third of
  its weight, which matters on a loader if the place it must go is tight. Its −30 °C floor is
  10 °C short of the C610's.
- **Licensing.** All three are circular patches. By the guide's rule (§ above, [5] §5.8) the R9025
  and C610 sit at or under the listed MTI-242043 (up to 9.5 dBiC), the C912 just under it. The
  FCC filing itself was not read.

### 4.2 The feed: reader → wall → coax → antenna

| Link | Part | Why |
|---|---|---|
| Reader U.FL to the case wall | **Siretta ASMG010XD113S11** or **ASMG015XD113S11**: I-PEX MHF I (U.FL-compatible) to SMA female rear-mount bulkhead, O-ring, IP67, 1.13 mm coax, 100 or 150 mm [8] | The 10-07 finding's Taoglas CAB.011 states no sealing; this one states IP67 and comes with its O-ring. **Order the `XD` (standard SMA), not the `YD` (reverse-polarity) [9]**: the antennas above are plain SMA |
| Through the wall | Hole **Ø6.5 mm min with a 6 mm flat** (D-hole), 1/4-36 UNS-2A thread, 8 mm hex nut [8] | From the XD drawing. (The YD drawing gives a 6.1 mm flat [9].) |
| Wall to antenna | **SMA male to SMA male, Times LMR-195-UF** (ultra-flex) jumper, shortest that reaches | 0.43 dB/m at 900 MHz; repeated bend radius 51 mm, install 12.7 mm; −40 to +85 °C; TPE jacket, outdoor rated [10] |

Losses (inference, arithmetic on the datasheets): the 1.13 mm pigtail is 2.2 dB/m at 1 GHz [9],
about 0.3 dB for 150 mm; 3 m of LMR-195-UF is about 1.3 dB. Neither matters at a metre, where RF
power is being turned down anyway.

**Fitting notes (inference):**

- Put the bulkhead in the **bottom wall** with the glands, so water runs off the joint rather
  than into it. Wrap the mated SMA joint at the antenna in self-amalgamating tape.
- Tie the 1.13 mm pigtail to the board near the U.FL so vibration never pulls on the U.FL.
- Across the boom pivot, run the LMR-195-UF along the hydraulic hoses with a slack loop, and keep
  every bend that flexes at 51 mm radius or more [10]. No flex-cycle rating was found.
- The reader no longer needs the lid. In V2 it can sit near the bulkhead so the 100–150 mm
  pigtail reaches.

## References

[1] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M7E Hecto, WRL-24738," product
page. [Online]. Available: https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html

[2] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M7E Hecto: External Antenna,"
hookup guide. [Online]. Available: https://docs.sparkfun.com/SparkFun_Simultaneous_RFID_Reader_M7E/external_antenna/

[3] RFMAX, "R9025-LPF-SSF Circularly Polarized RFID Antenna," datasheet. [Online]. Available:
https://www.ptsmobile.com/rfid/antennas/R9025-LPF-SSF_Series_Datasheet.pdf

[4] Barcode Factory, "RFMAX R9025-LPF-SSF," product page and price. [Online]. Available:
https://www.barcodefactory.com/rfmax/antennas/r9025/r9025-lpf-ssf

[5] JADAK (Novanta), "ThingMagic M7E-HECTO User Guide," Rev. 1.2, §5.1.2, §5.7, §5.8, Table 5.
[Online]. Available: https://cdn.sparkfun.com/assets/8/5/2/4/d/M7E_HECTO_User_Guide.pdf

[6] Vulcan RFID, "C610 Outdoor RFID Antenna, VUL-C610-FCC," product spec sheet v5. [Online].
Available: https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/Vulcan%20RFID%E2%84%A2/Vulcan%20RFIDTM%20C610%20Antenna%20Datasheet%20v5.pdf

[7] atlasRFIDstore, "Vulcan RFID C912 Outdoor RFID Antenna (FCC)," product page and price, and
"Vulcan RFID C912 Antenna Datasheet v6." [Online]. Available:
https://www.atlasrfidstore.com/vulcan-rfid-c912-outdoor-rfid-antenna-fcc/

[8] Siretta, "ASMGxxxXD113S11, I-PEX MHF I to SMA Female Rear Mount Bulkhead with O-ring,"
datasheet. [Online]. Available via https://docs.rs-online.com/d3d1/A700000015827780.pdf

[9] Siretta, "ASMGxxxYD113S11, I-PEX MHF I to RP-SMA Female Rear Mount Bulkhead with O-ring
Pigtail Cable Assembly," Rev 1.1. [Online]. Available: https://docs.rs-online.com/1a6f/A700000015827842.pdf

[10] Pasternack, "LMR-195-UF Low Loss Flexible Outdoor Rated Coax Cable," datasheet. [Online].
Available: https://www.pasternack.com/images/ProductPDF/LMR-195-UF.pdf

## 5. Where the C610 mounts (working basis: bag on pallet forks)

Alex answered "On pallet forks" to how a bag is carried at record time, then asked for the
question again; the answer is used here as the working basis, not as settled. Everything in this
section is **inference** unless cited. The M520's dimensions and the fork frame's shape were not
read.

**Mount the antenna with the loader, not the tractor.** The bag rides on the tines, so anything
fixed to the loader's front end keeps the same distance and angle to it at every boom height.
RF power is then tuned once.

| Place | For | Against |
|---|---|---|
| **A. Loader side of the quick-attach**, above the fork carriage, facing forward along the tines | Stays on the loader when the forks come off; no connector rides the attachment | Must sit above the carriage's backrest so no steel is in front of the radome |
| B. On the fork frame or backrest, facing forward | Closest to the bag; nothing in the way | Coax needs an SMA joint at the quick-attach, unplugged every time the forks come off |
| C. On the tractor (grille guard, hood) | Coax never flexes | Distance to the bag changes with boom height and the tines may sit well over a metre out |

**Recommended: A.** Steel *behind* a patch antenna is not a problem (it has its own ground plane);
steel *in front* of it is.

Coax route for A: from the antenna along the boom with the hydraulic lines, with a slack loop at
the quick-attach pivot and another at the boom's tower pivot, every flexing bend at ≥ 51 mm
radius [10], back to the case on the ROPS. Jumper length: measure that route and add slack; NOT
DETERMINABLE here.

**Setting it up at the machine:**

1. Start at low RF power. Raise it in 0.5 dB steps [5] until a bag on the tines reads every time.
2. With bags stacked nearby where they normally stand, check none of them reads. Back off if one
   does.
3. Keep people 20 cm or more from the antenna face [5].

## 6. Alex's answer on placement, and the question it opened (2026-10-25)

Asked where the C610 goes, he answered in his own words instead of picking an option:

> *"Will you see but you know I'm an RF engineer so I will definitely be able to get the antenna in
> a good spot i'm thinking it should go basically the tractor has like a brush guard or something
> like you'd you know a guard on the front of it and what we'll do is attach it to that guard It's
> in front of everything but behind the loader in front of the tractors engine and intake and all
> that well in front of it but behind the loader should be fine My big question here though um is
> we could bring the tag to the unit to scan it while we're loading the bags like I could bring the
> zip tie to the RFID unit to scan it It really doesn't even need to be on the loader when I start
> thinking about it because this RFID thing may actually want to be a handheld unit on its own or
> there may be an RFID scanner device that we could use that's already built the loader does only
> has to tie the two pieces of information together and not even the loader does that the PWA app
> can do that does that make sense I know this is a lot of question but we need to get it right and
> you should pass this information on to the coordinator"*

**Placement:** the C610 goes on the tractor's front guard, behind the loader and in front of the
engine and intake. He places it himself. §5's options A–C are superseded by this.

**The open question is his, and it is undecided:** whether the tag read leaves the loader
altogether, with a handheld or an existing reader scanning the tag and the PWA joining tag and
weight. This is option 2 of `findings/2026-10-14-rfid-cots-alternatives.md` (§2), researched on
2026-10-14: a Bluetooth reader paired to the phone, from a $126.99 pocket reader (0–30 cm) to IP54
sleds at $1,184–1,369. §2.2 there lists what it changes: it reverses "one press does it all"
(`notes/rfid-inventory-discussion.md`), the board can no longer refuse on several tags or confirm
the scanned tag is on the forks, the approved record contract changes, and Web Bluetooth works only
in Chrome on Android.
