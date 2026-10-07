# `can_err` misses multi-flag CAN error events: proposal for issue #53

**2026-10-07. Alex approved route C for build only, in the question dialog ("Approve, build
only"). Implemented at `434c2f38` on `docs/proposal-53-can-err-mask`. Not flashed** (see §6).
Read from source and from the shipped FSP binary; the CAN behaviour itself is not measured on
silicon. The test and the compiles below ran on Saga. Revised the same day after an adversarial
review; each review point is answered in §5.

## Recommendation

**Install a small callback hook from this repo's own `include/j1939_can.h`.** The hook sees every
CAN driver event word before Arduino_CAN does. It records any word that is an error, whatever
flags it carries, and passes every event on to the library unchanged. The library itself is not
copied and not patched.

- **The issue is confirmed, and it is worse than it reads.** The driver's error interrupt hands the
  callback a raw register byte, so the word can carry several flags at once. Of the 248 words that
  interrupt can deliver, the shipped code counts 5 and misses 243 [1][2].
- **It works with every build path.** The hook is ordinary sketch code, so a plain `arduino-cli`
  build, `tools/build-check.sh` and the farm1 flash path all carry it, and the `build` stamp the
  board publishes describes it.
- **It costs 200 bytes of flash and 8 of RAM** (104,080 to 104,280 bytes; 17,784 to 17,792),
  measured by compile.
- **If a future core breaks the lookup, the board falls back to today's behaviour.** The console
  `can` verb then shows `hook OFF`. It never counts less than today.
- **It rests on how this core wires its interrupts**, which is not a published API (§2). On any
  core upgrade, re-check the four anchors listed there.
- **Whether a mailbox overwrite is ever reported cannot be settled from software.** It depends on
  RA4M1 silicon behaviour the shipped files do not describe. The pre-ring loss check in §4 answers
  the question that matters without depending on it.

## 1. What the source says

**Read** means read directly from a file or from a disassembly listing. **Inferred** means
reasoned from what was read.

| Claim | Status | Evidence |
|---|---|---|
| `onCanCallback` switches on exact event values, one `case` per value | **Read** | [3] `R7FA4M1_CAN.cpp:274-308` |
| Event values are single bits: 2, 4, 8, 16, 32, 128, 256, 512, 1024, 2048, 4096, 8192 | **Read** | [2] `r_can_api.h:69-83` |
| Those values line up with the error-flag register EIFR, bit for bit (bits 1-5 and 7) | **Read** | [4] `R7FA4M1AB.h:13579-13594` |
| The error interrupt's event word is EIFR as read, and EIFR is then cleared | **Read** | [1] `can_error_isr`, `r_can.c:899`, `:901` (register 0x84D = EIFR, [4] `:3078`) |
| If status bit STR.NMLST is set, MAILBOX_MESSAGE_LOST (32) is OR'd into that word | **Read** | [1] `r_can.c:908-910` (0x842 = STR, [4] `:2966`; NMLST = bit 4, [4] `:13512`) |
| The error interrupt is enabled for EIFR bits 1-5 only (EIER = 0x3E) | **Read** | [1] `R_CAN_Open`, `r_can.c:418`; [4] `:13562-13577` |
| EIFR bit 0 (bus error) and bit 6 (overload) have no `can_event_t` name but ride along in the word | **Read** for the word; **inferred** that the hardware sets them while their interrupt is disabled | [1] `r_can.c:899` reads the whole byte; [2] has no value 1 or 64 |
| The receive interrupt always passes exactly 1024; the transmit interrupt exactly 2048 | **Read** | [1] `can_rx_isr`, `r_can.c:980`; `can_tx_isr`, `r_can.c:1071` |
| The receive interrupt never reports or clears a lost message | **Read** | [1] `can_rx_isr` writes MCTL = 0x44, `r_can.c:817`; no STR or MSGLOST test anywhere in it |
| The controller runs in overwrite mode (an unread mailbox is overwritten by the next frame) | **Read** | [3] `R7FA4M1_CAN.cpp:115`; [1] `R_CAN_Open` writes `message_mode` into CTLR.MLM, `r_can.c:302-304`; [4] `:13485` |
| In overwrite mode the error interrupt does not re-arm itself for a further lost mailbox | **Read** | [1] `r_can.c:930`: the re-pend is skipped when `message_mode` is 0 |
| `canDrainISR` counts only what `CAN.isError()` returns | **Read** | `include/j1939_can.h:59-60` |
| The drain timer and the CAN interrupts run at the same priority (12) and cannot preempt each other | **Read** | `include/j1939_can.h:101` takes the default; [3] `FspTimer.h:152` (default 12); [3] `IRQManager.cpp:24` (`CAN_PRIORITY 12`) |

**The answer to "does a mailbox overwrite raise MAILBOX_MESSAGE_LOST in this driver?":**

- It is never raised by the receive path.
- It is raised only from the error interrupt: when STR.NMLST is set at the moment that interrupt
  runs, or when EIFR.ORIF (overrun, also bit 5, value 32) is set.
- If the RA4M1 sets ORIF on an overwrite in overwrite mode, a lone overwrite arrives as exactly 32.
  The shipped code catches that word.
- If the RA4M1 sets only MSGLOST/NMLST, nothing interrupts. The lost-mailbox bit then waits for the
  next real error interrupt and arrives combined with it, for example 0x22. That is the word the
  shipped code misses.
- **Which of those two the silicon does is NOT DETERMINABLE here.** The only Renesas document
  vendored in `docs/vendor/renesas/` is the datasheet [5], and it has no CAN register descriptions
  (searched for ORIF, MSGLOST and "overwrite mode"; no hits). The RA4M1 User's Manual: Hardware,
  CAN chapter, ORIF and MSGLOST descriptions, would answer it.

A side note, not part of this fix: the comment at [3] `R7FA4M1_CAN.cpp:279`, which says the
receive-complete case is unreachable, is wrong. 1024 is what `can_rx_isr` sends for every frame.

## 2. Fix routes

| | **C. Hook from repo code (recommended)** | **B. Repo copy of the library, patched** |
|---|---|---|
| What changes | `include/j1939_can.h` and 2 console lines in `src/`, plus the new `include/can_event_mask.h` | `lib/Arduino_CAN/` (a copy of the library plus a 14-line `default:` arm), and `tools/build-check.sh` |
| Build paths | Every one. It is sketch code. | Only builds that pass `--library`. `tools/gen-provenance.sh:4` expects direct `arduino-cli` builds and `BRIEF.md:394` describes one. Those link the **unpatched** library under the **same** `build` SHA, so the fork would need an `#error` guard in the sketch. |
| Licence | No third-party code copied | Copies GPL-2.0/LGPL-2.1 files [3] into a repo with no LICENSE file. The headers must be kept and the change marked. |
| On a core upgrade | Re-check the four anchors below. If they break, the hook reports failure and the board behaves as today. | Re-diff the fork against the new library. A stale fork keeps an old driver without any warning. |
| PlatformIO | Same as any sketch code | Inferred to pick up `lib/` on its own. Not verified: the `renesas-ra` platform is not installed on Saga, and its framework may not be 1.6.0. |
| Main risk | Relies on core internals that are not a published API | Build-path dependence (above) |
| Measured | Compiles, `--warnings all` clean, 104,280 / 17,792 | Compiles, 104,096 / 17,784 |

Also considered and rejected:

- **A. Patch `~/.arduino15` in place:** unversioned, invisible to `build`, and undone by a core
  reinstall.
- **D. Poll controller registers in `canDrainISR`:** it sees error *state*, not error *events*, and
  the FSP clears MSGLOST in its own interrupt. It could come later as a separate change, not
  instead of C.

**The four anchors route C rests on**, all read in 1.6.0:

1. IRQManager links the CAN0 error event into an ICU slot and stores the driver's control block as
   that slot's ISR context. [3] `IRQManager.cpp:879-882`, `:1752-1755`; [2] `bsp_irq.h:104-120`.
2. `R_FSP_IsrContextGet` returns that context. [2] `bsp_irq.h:197`; event number 74,
   [2] `bsp_elc.h:123`; IELS field mask 0x1FF, [4] `:14889`.
3. `R_CAN_CallbackSet` is public FSP API. [2] `r_can.h:235-238`. In the binary it stores the memory
   and context fields first, then the callback, and returns 0 ([1] `r_can.c:700-706`). The context
   it is given (`&CAN`) and the null memory are the values already there ([3]
   `R7FA4M1_CAN.cpp:124`; [1] `r_can.c:284`). An interrupt that lands between the two stores
   therefore finds a consistent pair.
4. `onCanCallback` is public ([3] `R7FA4M1_CAN.h`, though commented "should NOT be called by the
   user").

**`R_CAN_Open` resets the callback from the config** ([1] `r_can.c:282`) **and zeroes the callback
memory** ([1] `r_can.c:284`). So every `CAN.begin()`, including the `canf` reopen in
`canSetFilter()`, drops the hook. The proposal re-installs it inside `canOpen()`, which every open
goes through.

**Residual risk.** A future core that stored a different object as the error slot's context would
make `R_CAN_CallbackSet` write into the wrong memory. The `static_assert`s in
`include/can_event_mask.h` guard the event numbering only, not this. Re-check anchor 1 on any core
upgrade.

**Origin.** Route C was proposed by the adversarial reviewer, whose sketch compiled against 1.6.0.
I verified each anchor above against the source myself and compiled the integrated version below.
**Neither has run on hardware.**

### The exact change (route C)

**1. New file `include/can_event_mask.h`**, included by `include/j1939_can.h`:

- `canEventIsError(event)`: true for any bit other than RX_COMPLETE and TX_COMPLETE.
- `canMergeError(...)`: what one drain reports. The hook's word wins, and the library's own flag
  still counts when the hook is not installed.
- When built for the board, `static_assert`s fail the build if a core renumbers the events.

**2. `include/j1939_can.h`:**

```diff
@@ -29,6 +29,7 @@
 #include <Arduino_CAN.h>
 #include "FspTimer.h"
 #include "j1939_decode.h"
+#include "can_event_mask.h"
@@ -49,6 +50,38 @@
 static bool              canTimerOn    = false;
 static FspTimer          canTimer;
 
+// ISSUE #53. Arduino_CAN's callback tests each event for ONE exact value, but the
+// FSP error ISR passes a register byte that can carry several flags at once, so
+// most error events were never reported (findings/2026-10-07-can-err-multibit-
+// proposal.md). The hook below sits between the FSP and that callback: it records
+// any word canEventIsError() calls an error, then forwards EVERY event unchanged,
+// so the receive path is exactly the library's.
+static volatile bool     canHooked     = false;  // the hook is installed on the open controller
+static volatile bool     canHookErr    = false;  // the hook saw an error word since the last drain
+static volatile uint32_t canHookLast   = 0;      // that word, every flag kept
+
+static void canEventHook(can_callback_args_t *a) {
+  const uint32_t ev = (uint32_t)a->event;
+  if (canEventIsError(ev)) { canHookLast = ev; canHookErr = true; }
+  CAN.onCanCallback(a);
+}
+
+// IRQManager links the CAN0 error event into an ICU slot and stores the driver's
+// control block as that slot's ISR context (IRQManager.cpp:879-882). Find the slot,
+// take the block, and point its callback at the hook. R_CAN_Open() resets the
+// callback, so this must run after EVERY CAN.begin(). If it cannot find the slot,
+// it returns false and the drain falls back to the library's own error flag,
+// which is today's behaviour.
+static bool canInstallHook() {
+  for (uint32_t i = 0; i < BSP_ICU_VECTOR_MAX_ENTRIES; i++) {
+    if ((R_ICU->IELSR[i] & R_ICU_IELSR_IELS_Msk) == (uint32_t)ELC_EVENT_CAN0_ERROR) {
+      can_ctrl_t *c = (can_ctrl_t *)R_FSP_IsrContextGet((IRQn_Type)i);
+      return c && R_CAN_CallbackSet(c, canEventHook, &CAN, nullptr) == FSP_SUCCESS;
+    }
+  }
+  return false;
+}
+
 static void canDrainISR(timer_callback_args_t *) {
@@ -56,8 +89,12 @@
-  int err = 0;
-  if (CAN.isError(err)) { canErrEvents++; canLastErr = err; CAN.clearError(); }
+  int libCode = 0, err = 0;
+  const bool libErr = CAN.isError(libCode);
+  if (canMergeError(libErr, libCode, canHookErr, canHookLast, err)) {
+    canHookErr = false;
+    canErrEvents++; canLastErr = err; CAN.clearError();
+  }
@@ -83,6 +120,7 @@
   const bool ok = CAN.begin(CAN_BITRATE);
   canFiltered = ok && filtered;
+  canHooked = ok && canInstallHook();
   return ok;
```

**3. `src/ArduinoHydraulicScale.cpp`, in the `can` console verb:**

```diff
-      Console.print(" last ");      Console.println(canLastErr);
+      Console.print(" last ");      Console.print(canLastErr);
+      Console.println(canHooked ? " hook on" : " hook OFF");
```

**Not proposed: publishing `canHooked` over MQTT.** It would add a key to the engine status
payload, a contract change, and the margin left in its `char p[704]` buffer
(`src/ArduinoHydraulicScale.cpp:599`) has not been measured. Add it only if Alex wants hook
failure visible remotely.

**What `can_last` means afterwards.** It now holds the whole word, so it reads as a bit set:
34 (0x22) means lost mailbox plus error warning. `references/03_mqtt_contract.md:193` ("the last
error code") would need to say so after a merge. The reviewer reports that the consumer
(`consumer_plc4_engine.py:145`) accepts any integer. That file is outside this repo and I have
not read it. `can_err` still counts 5 ms drains that found an error, not individual events. That
is the existing behaviour, left as it is.

## 3. Evidence

Host test, `test/test_can_event_mask.cpp`. It runs the **real**
`canEventIsError()` and `canMergeError()` from `include/can_event_mask.h`:

```
$ g++ -std=c++17 -Wall -Wextra -Iinclude test/test_can_event_mask.cpp -o /tmp/m && /tmp/m
  ok    reachable error-ISR words                            248 distinct words
  ok    proposed counts every one in can_err                 248 of 248
  ok    shipped counts only the single-flag words            5 of 248 (misses 243)
  ok    0x22 lost mailbox + error warning                    shipped MISSED, proposed counted
  ok    no word changes behaviour the shipped code had       0
  ok    hook not installed == shipped, every word            0 words differ
  ...
PASS: 0 failure(s)
```

- The test covers every word the error interrupt can build, the receive and transmit words, all
  65,536 16-bit words, and the hook-not-installed fallback.
- **Mutation check:** with `canEventIsError()` in a copy of the header forced to `return false`,
  the test reports `FAIL: 8 failure(s)` and exits 1.
- **Limit.** The library's own switch is still a transcription inside the test, because
  `R7FA4M1_CAN.cpp` needs the FSP and cannot build on the host. A core upgrade that changed that
  switch would go unnoticed by this test. The firmware build guards only the event numbering. The
  same holds for the FSP event-word model (§1), which comes from the 1.6.0 binary.

Existing host suite, unchanged, both green:

```
$ g++ -std=c++17 -Wall -Wextra -Iinclude test/test_loop_scaling.cpp -o /tmp/t && /tmp/t
all checks passed                      (35 ok, 0 FAIL)
$ g++ -std=c++17 -Wall -Wextra -Iinclude test/test_j1939_replay.cpp -o /tmp/r && \
  /tmp/r ~/plc4-logs/census-2026-09-22/candump-2026-09-22_175858.log
all passed                             (21 ok, 0 FAIL)
```

Cross-compiles on Saga, arduino-cli 1.5.0, `arduino:renesas_uno:unor4wifi`, with the sketch
assembled as `build-check.sh` assembles it, in `/tmp`:

| Build | Arduino_CAN compiled from | Flash | RAM |
|---|---|---|---|
| today | platform `libraries/Arduino_CAN` | 104,080 | 17,784 |
| route C, `--warnings all`, no warnings from the changed files | platform `libraries/Arduino_CAN`, unmodified | 104,280 | 17,792 |
| route B, `--library` | `/tmp/.../lib/Arduino_CAN` (from `compile_commands.json`) | 104,096 | 17,784 |

The route C build also compiled the `static_assert`s in `can_event_mask.h`, so the event numbering
it relies on holds in 1.6.0.

## 4. What stays unproven until silicon, and the smallest on-tractor check

**Unproven:**

- That the hook installs on the board: `hook on` at the console.
- Which EIFR bits the RA4M1 sets on a real fault.
- Whether a mailbox overwrite sets ORIF in overwrite mode.
- Whether frames are lost before the software ring at all.

On the last point, the inference is that it is possible. Overwrite mode keeps one unread frame per
mailbox. The 5 ms drain and the CAN receive interrupt share a priority, so a long drain delays the
receive interrupt. Whether the controller spreads incoming frames across the 8 accept-all mailboxes
or reuses the first one is NOT DETERMINABLE here. The field figures, 462.5 fps on the board against
464.9 in the census, were not taken over the same window
(`notes/2026-09-22-tatonka-field-session.md:72`).

**Smallest check, after Alex approves and flashes the change:**

1. **The hook is in.** The console `can` verb shows `hook on`. Run `canf` twice, then check again:
   it must still say `hook on`, which proves the re-install on reopen.
2. **No false positives.** Key on, engine at idle for 15 minutes: `can_err` stays 0 and the frame
   rate matches the ~462 fps seen before.
3. **Pre-ring loss, measured directly.** Run a `candump` capture on the tractor bus over the same
   window as the board, the way the 2026-09-22 census was taken, and compare the capture's frame
   count with the change in the board's `frames` over that window. With `ring_full` at 0, any
   shortfall is frames lost before the ring.

Do not provoke a bus fault on the tractor to exercise the new error path: that disturbs the
tractor's own bus. If the path needs to be seen working, the two-node bench rig
(`findings/2026-09-20-two-node-bench.md`) is where to do it.

## 5. Adversarial review, point by point

1. **"A repo-only route exists."** Accepted. I wrongly said it was impossible. Each anchor was
   verified (§2), the hook was compiled integrated, and it is now the recommendation. One addition
   to the reviewer's sketch: re-install after every `CAN.begin()`, because `R_CAN_Open` resets the
   callback ([1] `r_can.c:282`, `:284`).
2. **"Covered by build is false for the fork."** Accepted. `tools/gen-provenance.sh:4` and
   `BRIEF.md:394` describe direct `arduino-cli` builds, which would link the unpatched library.
   This is moot under route C. Had the fork stayed, it would need an `#error` guard.
3. **"The test covers a copy."** Partly fixed. The test now runs the real classification and
   merge functions from `include/can_event_mask.h`. The library's switch is still transcribed;
   that is stated as a limit in §3.
4. **"PlatformIO is unverified."** Accepted. `~/.platformio/platforms` holds only `espressif32`
   and `native`, and `pio` is not on PATH. Stated in §2.
5. **Minor points.** The extra R7FA6M5 files, licence headers and modification notice apply to
   route B only (§2). The `can_last` bit-set change is noted in §2. The consumer claim is
   recorded as the reviewer's; I have not verified it.

## 6. Implementation

Implemented at `434c2f38`, exactly as the diff in §2. The commit touches `include/j1939_can.h`,
`src/ArduinoHydraulicScale.cpp` (the two console lines), `include/can_event_mask.h` (comment
only) and `test/test_can_event_mask.cpp` (comment only). **Built, not flashed.**

`tools/build-check.sh`, run from the worktree, image stamped `434c2f38`, not dirty:

| Build | Flash | RAM |
|---|---|---|
| main `501b7dc8` | 104,120 | 17,784 |
| `434c2f38` | 104,328 (+208) | 17,792 (+8) |

Both were built by `build-check.sh` through the same scratch path, `/tmp/plc4-build-check`. Main
came from a `git archive` export, stamped with its own SHA so the provenance strings are the same
length. The `/tmp` builds in §3 used other scratch paths and read about 40 bytes lower. That
offset is inferred to be the build path embedded in the image, since the sketch files were
identical (`diff -rq`); it was not traced further.

Host suite on `434c2f38`, all rc=0:

- `test_can_event_mask`: 23 ok, 0 FAIL.
- `test_loop_scaling`: 35 ok, 0 FAIL.
- `test_j1939_replay` on the 2026-09-22 census log: 21 ok, 0 FAIL.
- **Mutation check:** the real header with `canEventIsError()` forced false gives
  `FAIL: 8 failure(s)`, exit 1.

**Still owed before the change describes the machine:**

- A flash, which is Alex's.
- The checks in §4.
- The binder follow-ups: `references/03_mqtt_contract.md:193` (`can_last` becomes a bit set) and
  `references/05_known_defects.md` D4.

## References

[1] Renesas Electronics, *Flexible Software Package (FSP) v4.0.0*, `r_can` driver, object `r_can.o`
in `libfsp.a`, Arduino build, line table to `ra/fsp/src/r_can/r_can.c`. Shipped in Arduino,
ArduinoCore-renesas 1.6.0, `variants/UNOWIFIR4/libs/libfsp.a`. Renesas proprietary licence (file
header of [2]). Read by disassembly with `arm-none-eabi-objdump -d -l` (gcc 7-2017q4).

[2] Renesas Electronics, FSP v4.0.0, `api/r_can_api.h`, `instances/r_can.h`, `bsp/mcu/all/bsp_irq.h`
and `bsp/mcu/ra4m1/bsp_elc.h`, shipped in ArduinoCore-renesas 1.6.0,
`variants/UNOWIFIR4/includes/ra/fsp/`. Renesas proprietary licence.

[3] Arduino (A. Entinger), *ArduinoCore-renesas*, 1.6.0: `libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp`
and `.h` (GPL-2.0 or LGPL-2.1, per file header), `cores/arduino/IRQManager.cpp` and `FspTimer.h`.
https://github.com/arduino/ArduinoCore-renesas

[4] Renesas Electronics, *R7FA4M1AB device header*, `R7FA4M1AB.h`, shipped in ArduinoCore-renesas
1.6.0, `variants/UNOWIFIR4/includes/ra/fsp/src/bsp/cmsis/Device/RENESAS/Include/`.

[5] Renesas Electronics, *RA4M1 Group Datasheet*, R01DS0355EJ0110, Rev. 1.10,
`docs/vendor/renesas/RA4M1_Group_Datasheet_R01DS0355EJ0110.pdf`.
