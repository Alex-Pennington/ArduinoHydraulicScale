# PLC #4 RFID build: the 3D-printed case

**Date:** 2026-10-25. **Scope:** a record of where the printable case lives and what state it is
in. Nothing here is a design change, and nothing here touches firmware.

**How to read the claims.** *Read from the artifact* means read from the published page or its
file bundle at the version named below. **Inference** marks a conclusion not stated in any source.
**NOT DETERMINABLE** marks something no source reached here settles. Nothing was measured.

## Where it lives

The case is the claude.ai artifact **"PLC #4 Case Layout"**,
https://claude.ai/artifact/HQZLr4c1tWyn3KSXsKvTSS, read at version `1792960451-5dfc`. Alex
pointed this session at it on 2026-10-25 (*"This repo created an artifact that should tell you
everything you need to know"*). The repository holds no copy of it: at `main` 288a417 there is no
STL, STEP, 3MF or SCAD file and no print discussion.

The page carries a downloadable bundle, `plc4-case-files.zip`, with:

| File | What it is | Size (mm) |
|---|---|---|
| `case-print/case-body.3mf` | Body: walls, floor, stand-offs, gasket groove, gland and USB-C holes | 200 × 165 × 75 |
| `case-print/case-lid.3mf` | Lid: LCD window and rebate, LCD and reader bosses, gasket tongue | 200 × 165 × 13.5 |
| `case-print/bracket-plate.3mf` | Clamp plate for a 40–80 mm ROPS post, 4 × M5 pillars | 130 × 130 × 28 |
| `case-print/README.md` | Settings, hardware, assembly, design defaults, checks | — |
| `case-print/preview-top.svg` | Top view with every part placed | — |
| `components/*.3mf` (12 files) + `README.md` | Fit-check models of the parts inside; not for printing | — |

Full hashes, from `sha256sum` on the unpacked bundle:

```
7deacf9ee7e8dfcc57dd46728a82daecb841b8789ea438e48844e018b7dd9229  case-print/case-body.3mf
3c2a38fa47c49888dade39fce54a611c4ac5389ad2bde0ce74e28dd061078a4e  case-print/case-lid.3mf
b910954cf08068e1586bb508bffbd3647e261c32037b8a8c59f684fb6abca347  case-print/bracket-plate.3mf
2e0438aed167298b77f11bcf39d9063f1741a8a1729d4d2b632b7653f15eaf11  case-print/README.md
d5d55cb0964d0404e60de7267b6cce579e660066750a88ae7487724bfcd5899d  case-print/preview-top.svg
```

Material is ASA (PETG fallback), 0.4 mm nozzle, 0.2 mm layers, no supports (read from the
artifact, §03).

## How the RFID reader sits in it

Read from the artifact (`case-print/README.md`, *Design defaults*, M7E reader):

- The SparkFun M7E Hecto (WRL-24738) screws to four plain-plastic bosses **on the inside of the
  lid**, antenna side toward the lid, with 4-40 thread-forming screws. No inserts and no metal in
  front of it; the lid is solid ASA through to the outer face, marked NO METAL inside and RFID
  outside.
- The boss pattern, 30.48 × 33.02 mm, comes from SparkFun's Eagle board file. The board fits one
  way only: USB-C edge toward the bottom (gland) wall, antenna end toward the case centre.
- The reader reads through the lid on its on-board antenna. No antenna cable passes through a wall
  (artifact §08).

This matches the ruled build in `findings/2026-10-14-rfid-cots-alternatives.md` (§0: M7E on its
on-board antenna) and the "no metal in front of the reader" rule in
`findings/2026-10-07-rfid-hardware-recommendation.md` §5.3. The case replaces the Hammond 1554
polycarbonate box that §5.3 named.

## State

- **V1 is printed** (Alex, in the artifact's comment thread, 2026-10-25: *"its already printed"*).
- **V1 has one known error.** The two DIN-rail pads run the TS35 rail along y, parallel to the
  Mean Well DDR-30G-12's 92 mm length, so the converter cannot clip on. Alex ruled, same thread:
  *"no, add the fix to V2 and well keep going. I'll use a different power supply here."* So the V1
  case takes a different supply at that position. Which supply is NOT DETERMINABLE here.
- **V2 change list** (artifact §09): run the rail along x across the converter's clip line near
  y 57, about 40 mm long (x 156–196) to clear the I²C block, with the two Ø10 × 4 pads moved to the
  rail's slot pitch (25 mm on standard slotted TS35, to be checked against the rail bought). Touches
  `case-body.3mf`, the README and the top view. No V2 file exists yet.

## Gaps

- **The generator is not in the bundle or this repository.** The print README names
  `case_params.py` and `build_case.py` (regenerate the files), and `verify_3mf.py` and
  `fit_check.py` (the checks). None is in the bundle. Where they are is NOT DETERMINABLE from this
  tree. Without them, V2 is a remodel rather than a parameter change.
- **Reader placement on the tractor — unreviewed inference, not a finding.** The reader is fixed
  behind the lid, so it faces wherever the LCD faces, and the whole case must sit within the ~0.5 m
  of the bag that the ruled build needs. The bracket clamps to a ROPS post, and the artifact says
  only to mount it "with the RFID face within about 0.5 m of where the bag hangs on the loader"
  (§06). Whether a ROPS post on the 5055E is that close to the bag, and whether the M7E's on-board
  antenna reads usefully to the side or back, is NOT DETERMINABLE here. The bench test on the loader
  that §0 of the COTS findings already calls for would answer it.
