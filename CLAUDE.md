# PLC #4 — tractor hydraulic scale

## What this is

Phoenix Nest machine #4. A hydraulic pressure transducer read as a weight, shown on a
local LCD, published to MQTT over WiFi. Arduino UNO R4 WiFi. It is the newest machine in
an estate that already runs PLC #1, #2 and #3.

Read `BRIEF.md` first. It carries the verified hardware picture, the settled decisions and
the open questions. `PORT-PLAN.md` scopes the approved forward-port of the MQTT layer onto
`origin/main` — scope only, no code written.

## Boundaries

This tree is the whole workspace. PLC #1, #2 and #3 live in their own trees and are
coordinated from `~/greybeard-plc-coordination` — do not read or edit them from here.

PLC #4 has **two remotes and both are intended** (Alex, 2026-09-14):

- `saga` — `/opt/repos/greybeard-plc4-tractor-scale.git`, the canonical bare repo, created
  2026-09-13. Matches the rest of the estate: Saga is canonical, GitHub is a mirror.
- `origin` — `github.com/Alex-Pennington/greybeard-plc4-tractor-scale`, the mirror and the
  issue tracker. **Pushing to it is fine.**

`upstream` is `ArduinoHydraulicScale`, the project this was forked from. Do not push there.

**The line this replaces said "`origin` is Alex's upstream GitHub repo, not ours. Never push
to it."** That was a MISCOMMUNICATION in the chat that produced this file (Alex, 2026-09-14),
not a rule — and it survived unedited from 2026-09-01 because it reads as one. It cost real
caution: a session held a whole feature off GitHub on the strength of it, and could not tell
from the repository whether the rule named `origin` or the remote now called `upstream`, since
git does not version remotes. If a rule here names a remote, name what it points at too.

## How work moves here

Firmware changes are gated. The baseline has defects — they are written up in
`findings/`, not fixed. Nothing in the `.ino` changes without Alex approving that
specific change. Auditing, documenting and proposing are open; editing is not.

No board is tethered. Nothing here has been compiled or flashed, so every claim about
runtime behaviour is read from source and should say so.

## When you do not know

The transducer's specifications, the hydraulic geometry behind the word "weight", and
which broker this machine talks to are all genuinely unknown. They are not inferable
from the source and guessing them produces confident wrong numbers. Ask via the
coordinator; do not reason from a part number you have not seen.

## Reporting

Questions, blockers and completion go to `plc4-tractor-hydraulic-mqtt` by SendMessage.
That session relays to Alex. Do not contact Alex directly and do not fire the
house-lights signal.

## Layout

- `findings/` — dated audit write-ups. One file per pass, evidence with line numbers.
- `notes/` — working context worth keeping between sessions.
- `PORT-PLAN.md` — the scoped forward-port. Not started; the open decisions in its §1 are
  Alex's, not ours.
