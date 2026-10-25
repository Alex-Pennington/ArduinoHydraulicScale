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
