# PLC #4 RFID case V2: inputs log

Kept by the PLC #4 coordinator. It collects every input to the V2 case, so the worker Alex
dispatches to lay out the build starts with all of it. **Nothing here is dispatched work.** Alex
said on 2026-10-25: *"when I tell you we will dispatch a worker to start figuring out where
everything goes on the build"*. Until then the coordinator only adds to this log.

Each entry says who it came from: **Alex** (his words or his dialog choice), **built** (merged
work), or **suggestion** (a session's inference, not his instruction). A suggestion is not scope.

## Where things live

- Case generator and print files: `greybeard-plc4-pwa` `design/case-print/` (`case_params.py`,
  `build_case.py`, `verify_3mf.py`, `fit_check.py`). Alex chose to keep the source there.
- Printed V1: the "PLC #4 Case Layout" artifact, https://claude.ai/artifact/HQZLr4c1tWyn3KSXsKvTSS,
  and greybeard-plc4-pwa `d27af90`. Record: `notes/2026-10-25-rfid-case-print.md`.
- **Alex, 2026-10-25:** the artifact is not updated while V2 is in development ("v2 is still in
  development").

## Inputs

### 1. DIN rail for the power supply: built

- **Alex**, 2026-10-25, on V1's rail running along the DDR-30G-12 instead of across it: *"no, add
  the fix to V2 and well keep going. I'll use a different power supply here."*
- **Built**, merged to greybeard-plc4-pwa `main` at `f494e58`: the TS35 rail runs along x, 40 mm,
  x 156-196, y 57, across the DDR-30. Pads 25 mm apart. Lid and bracket unchanged.
- **Open:** the 25 mm slot pitch is ASSUMED. Check it against the rail Alex bought.

### 2. Power supply connection: changing

- **Alex**, 2026-10-25: *"there's gonna be a change in how the power supply is connected"*. V1 takes
  a different supply from the DDR-30G-12 ("I'll use a different power supply here").
- **Open:** which supply, and how it connects. NOT DETERMINABLE from the record. Ask Alex.

### 3. External antenna: ruled

- **Alex**, 2026-10-25, in `plc4-hardware-1025`'s dialog: chose "External antenna (Recommended)",
  reversing the 10-14 "no antenna" build, then chose the Vulcan C610.
- Chain, from that session's note: the M7E's RF 0 Ω resistor moved to u.FL, then a Siretta
  ASMG010XD113S11 or ASMG015XD113S11 IP67 SMA-F bulkhead, then an SMA M-M jumper on LMR-195, then
  VUL-C610-FCC on the loader.
- Source: `notes/2026-10-25-rfid-read-range-and-antennas.md`.

### 4. Bulkhead hole and reader position: suggestion

- **Suggestion** (`plc4-hardware-1025`): one D-hole for the bulkhead, 6.5 mm with a 6 mm flat,
  "ideally in the bottom wall beside the glands". Since the antenna is external, the reader no
  longer has to sit on the lid.
- Not Alex's instruction. Where the reader and the bulkhead go is for the layout worker to work out
  with him.

### 5. Antenna placement: Alex

- **Alex**, 2026-10-25, in `plc4-hardware-1025`'s dialog: he places the C610 himself ("I'm an RF
  engineer"). If it stays on the tractor, it goes on the front guard, behind the loader and in front
  of the engine and intake. Source: `notes/2026-10-25-rfid-read-range-and-antennas.md` §6.

### 6. Does the RFID read belong on the tractor at all? OPEN, and it gates inputs 3-5

- **Alex**, 2026-10-25, same dialog, his question and not a decision: the tag could be brought to a
  reader instead, maybe a handheld or an existing reader, with the PWA app tying tag to weight.
  *"we need to get it right"*. Full quote in `notes/2026-10-25-rfid-read-range-and-antennas.md` §6.
- If the read leaves the tractor, the case may need no reader, antenna, bulkhead or u.FL rework
  (inference). Settle this before the layout worker places them.
- The background research is option 2 of `findings/2026-10-14-rfid-cots-alternatives.md` §2.
  `plc4-hardware-1025` is putting the question to him.

## When Alex says go

Dispatch one worker in a worktree of greybeard-plc4-pwa, briefed with this log and Alex's words,
to work out with him where everything goes. It asks him and does not fill in the design.
