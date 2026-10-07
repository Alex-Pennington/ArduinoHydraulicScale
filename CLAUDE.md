# PLC #4 — tractor hydraulic scale

## What this is

Phoenix Nest machine #4. A hydraulic pressure transducer read as a weight, shown on a
local LCD, published to MQTT over WiFi. Arduino UNO R4 WiFi. It is the newest machine in
an estate that already runs PLC #1, #2 and #3.

Read `BRIEF.md` first. It carries the verified hardware picture, the settled decisions and
the open questions. `PORT-PLAN.md` scoped the forward-port of the MQTT layer; that work is
implemented and on `main`.

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

**A board is in service.** PLC #4 is on the tractor with its loop live and its calibration
stored. Flashing needs its USB on a host this session can reach. Build on Saga, copy the
image to that host, and upload with `arduino-cli` there. Which host it is plugged into
changes, so do not assume one: before any upload, read `/dev/serial/by-id` on the host and
confirm it names the UNO R4 WiFi. The firmware publishes the commit it was built from as
`build` on the status topic, so what is running can be asked rather than assumed.

That changes what a wrong claim costs. It is no longer a sentence in a file — it is a
machine in a field producing a weight somebody acts on. Say which claims are read from
source and which are measured, and measure where you can: the broker is reachable from
Saga and the board answers.

## When you do not know

The transducer's specifications, the hydraulic geometry behind the word "weight", and
which broker this machine talks to are all genuinely unknown. They are not inferable
from the source and guessing them produces confident wrong numbers. Ask via the
coordinator; do not reason from a part number you have not seen.

## Reporting

**Talk to Alex directly.** He drives this work himself and answers in the session. An
earlier version of this file routed everything through a relay session called
`plc4-tractor-hydraulic-mqtt`; that is not how this repo is worked and the relay is not
running.

Put anything he must answer or acknowledge in a question dialog rather than in prose. He
has asked for that twice: the scrollback is too long to re-read every time, and a question
at the foot of a long message is a question he never sees.

Still true and not a convenience: **never fire the house-lights signal.** Only the estate
coordinator may, and only when Alex has asked to be signalled.

## Layout

- `findings/` — dated audit write-ups. One file per pass, evidence with line numbers.
- `notes/` — working context worth keeping between sessions.
- `PORT-PLAN.md` — the forward-port it scoped is implemented and on `main`. Kept as the
  record of what was decided and why.
