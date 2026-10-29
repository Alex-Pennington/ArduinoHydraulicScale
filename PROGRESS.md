# PROGRESS

Append-only log of session wraps. Newest at the bottom. Never rewrite an earlier entry.

## 2026-10-29 — ymir-plc4-1029-0950

**Task (Alex, 2026-10-29):** "coordinate with @ymir-gleipnir-coordinator-1028-0d32 to add the same
rfid chip that the t-deck pendant uses.  that way an operator at the tracktor can scan a rfid tag.
add the nessisary buttons and menues to the design". Dialog choices: "PN532, as you said",
"Full pendant set", "Build all eight", "Hold until wired".

**What changed**
- `docs/design/2026-10-29-onboard-rfid-buttons-menus.md`: on-board PN532 on `Wire` (A4/A5, 0x24);
  four sealed lid buttons on D2-D5; 22 screens on the 20x4 LCD; the board sends the pendant's
  `POST /api/cmd` `"+"` body with `tag` and optional `mc`; FW-1 to FW-8.
- Firmware FW-1 to FW-8: `include/{pn532,hmi_buttons,hmi_menu,panel_client,cmd_id,mc_value}.h`, the
  wiring in `src/ArduinoHydraulicScale.cpp`, five new host tests in `test/`.
  - At most one WiFiClient call per `loop()` pass, gated under 2000 ms into the pass.
  - Ids `p4-<session>-<n>`, with a lazy EEPROM session counter at 1024.
  - A RECORD whose answer is lost is retried under the same id, never a new one.
  - Tare, undo, clear and the tally verbs read `/api/state` first and refuse on the RFID build.
  - The heartbeat timer is allocated before any new IRQ consumer, and `hw_fail` goes on status.
  - The status JSON never goes out truncated.
- Not flashed. The board in service keeps its previous build until the reader and buttons are
  fitted (Alex: "Hold until wired").

**Verified**
- `TMPDIR=$CLAUDE_JOB_DIR/tmp/wrap ./tools/build-check.sh`: exit 0. 128904 B flash (49%),
  21136 B RAM (64%). The only warning is the core CAN narrowing warning that main also has.
- `g++ -std=c++17 -Wall -Wextra -Iinclude -Itest test/test_<n>.cpp`, run from `test/`: exit 0 and 0 FAIL
  for can_event_mask, cmd_id, hmi_buttons, hmi_menu, loop_scaling, panel_client and pn532.
- `test_j1939_replay` not run at wrap: it needs a candump log that is not in the repo. The branch
  does not touch the J1939 code. The worker and the reviewer ran it against the 2026-09-22 log
  (21 ok).
- The design and the firmware each had an adversarial review. The firmware review's blocking
  double-record was reproduced by a test on the old code, and fixed. Mutation checks fail as
  expected.
- farm1 panel: `curl -D - http://farm1.internal:8081/api/health` returns 200 with a `Date`
  header. Gleipnir reports greybeard-plc4-pwa df172d2 deployed there.
- Board: the retained `plc4/jd5055e/status`, read from Saga via `farm1.internal`, is `{"up": false}`.
  The board is offline.

**What is next** (not filed as issues; Alex 2026-10-02, burn down rather than file)
- Flash once the reader and buttons are fitted. Confirm the UNO R4 WiFi in `/dev/serial/by-id` on
  the host first. Bench unknowns: a free third timer, two sockets on the WiFiS3 bridge, real stage
  times, heap headroom, the PN532 on silicon.
- Binder cards stale once flashed: `00` (:48), `01` (header pins), `01D` (:13 "Inputs: None"),
  `03` (`nfc`, `nfc_fw`, `hw_fail`, `p4-` ids), `06` (button flows).
- Other repos, owned by other sessions: the V2 case in greybeard-plc4-pwa `design/case-print/`
  (four 16.2 mm holes, PN532 bosses); the comment on greybeard-plc-consumer migration 129
  ("plc4 will never have rfid") is superseded.
- `pio run -e uno_r4_wifi` fails on main ("tallyRestore was not declared", `src:1525`).
  `tools/build-check.sh` is the build that counts.
- The datasheets read (PN532, Elechouse, SparkFun COM-11966) are not filed in `docs/vendor/`.

**Sessions started**
- Design worker (in-process scoped-worker subagent, branch `design/plc4-onboard-rfid`): finished, merged; already stopped.
- Design reviewer (in-process general-purpose subagent, read-only): finished; already stopped.
- Firmware worker (in-process scoped-worker subagent, branch `feat/plc4-onboard-rfid`): finished, merged; already stopped.
- Firmware reviewer (in-process general-purpose subagent, read-only): finished; already stopped.
- No `ymir-*` or `tyr-*` sessions started.
