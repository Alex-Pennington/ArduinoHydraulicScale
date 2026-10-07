# `can_err` misses multi-flag CAN error events: proposal for issue #53

**2026-10-07. PROPOSAL ONLY. Nothing here is in the firmware, and nothing has been flashed.**
Read from source and from the shipped FSP binary; the CAN behaviour itself is not measured on
silicon. The test and the compile below ran on Saga.

## Recommendation

**Add a copy of Arduino_CAN to this repo, at `lib/Arduino_CAN`, with one `default:` arm added to
`onCanCallback()`, and have `tools/build-check.sh` build against that copy.** The added arm reports
any event word that is not "frame received" or "frame sent" as an error. Every existing case is
left as it is.

- **The issue is confirmed, and it is worse than it reads.** The driver's error interrupt hands the
  callback a raw register byte, so the word can carry several flags at once. Of the 248 words that
  interrupt can deliver, the shipped code reports 5 and misses 243 [1][2].
- **The fix costs 16 bytes of flash and no RAM** (104,080 to 104,096 bytes, measured by compile).
- **The fix can only live in the driver.** The library keeps the event word, the error flag and
  the controller handle private, so this repo's own code never sees the word it would need to test
  ([3] `R7FA4M1_CAN.h`; see Routes).
- **Whether a mailbox overwrite is ever reported cannot be settled from software.** It depends on
  RA4M1 silicon behaviour the shipped files do not describe. The pre-ring loss check under "What
  stays unproven" answers the question that matters without depending on it.

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

| Route | What it is | Verdict |
|---|---|---|
| **A. Patch the core in place** | Edit `~/.arduino15/.../Arduino_CAN/src/R7FA4M1_CAN.cpp` | **Reject.** Outside the repo, unversioned and invisible to the `build` stamp the board publishes. A core reinstall or upgrade silently undoes it. |
| **B. Repo copy of the library, patched** | `lib/Arduino_CAN` = the 1.6.0 library plus the diff below; `build-check.sh` passes `--library` | **Recommend.** Versioned, covered by `build`, and reviewable as a diff. Cost: a forked library to re-diff on any core upgrade. |
| C. Take over the callback from repo code | Find the controller handle through the vector table or ISR context, then call `R_CAN_CallbackSet` | Reject. It needs private state (`_can_ctrl`, [3] `R7FA4M1_CAN.h`) or IRQManager internals. Fragile, and harder to review than B. |
| D. Poll controller registers in `canDrainISR` | Read STR, RECR and TECR directly every 5 ms | Not a fix: it sees error *state*, not error *events*, and the FSP clears MSGLOST in its own interrupt. Worth adding later as a separate change, not instead of B. |

"A mask test in this repo's code", which the issue offers, is route B: the mask lives in the repo's
copy of the driver. No repo-only route exists that stays clear of private state.

### The exact change

**1. `lib/Arduino_CAN/`:** copy `~/.arduino15/packages/arduino/hardware/renesas_uno/1.6.0/libraries/Arduino_CAN/`
whole, then apply:

```diff
--- renesas_uno/1.6.0/libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp
+++ lib/Arduino_CAN/src/R7FA4M1_CAN.cpp
@@ -304,6 +304,20 @@
       _err_code = p_args->event;
     }
     break;
+    /* PLC #4 issue #53. can_error_isr() passes the error-flag register EIFR, OR'd
+     * with CAN_EVENT_MAILBOX_MESSAGE_LOST when STR.NMLST is set, as the event. One
+     * call can therefore carry several flags, and EIFR bits 0 and 6 name no
+     * can_event_t. Each case above matches one exact value, so such a word fell
+     * through and was never reported. RX and TX complete arrive alone, from their
+     * own ISRs; any other non-zero bit is an error. */
+    default:
+      if (static_cast<uint32_t>(p_args->event) &
+          ~static_cast<uint32_t>(CAN_EVENT_RX_COMPLETE | CAN_EVENT_TX_COMPLETE))
+      {
+        _is_error = true;
+        _err_code = p_args->event;
+      }
+    break;
   }
 }
```

**2. `tools/build-check.sh`, last line:**

```diff
-exec arduino-cli compile --fqbn "$FQBN" --warnings default "$SKETCH"
+exec arduino-cli compile --fqbn "$FQBN" --warnings default \
+  --library "$REPO/lib/Arduino_CAN" "$SKETCH"
```

PlatformIO picks up `lib/` on its own (inferred from PlatformIO's convention; not built here).

**What `can_last` means afterwards.** It now holds the whole word, so it reads as a bit set:
34 (0x22) means lost mailbox plus error warning. `can_err` still counts 5 ms drains that found the
error flag set, not individual events. That is the existing behaviour, left as it is.

## 3. Evidence

Host test, new, `test/test_can_event_mask.cpp`, marked PROPOSED in its header:

```
$ g++ -std=c++17 -Wall -Wextra -Iinclude test/test_can_event_mask.cpp -o /tmp/m && /tmp/m
  ok    reachable error-ISR words                            248 distinct words
  ok    proposed sets _is_error for every one                248 of 248
  ok    shipped catches only the single-flag words           5 of 248 (misses 243)
  ok    0x22 lost mailbox + error warning                    shipped MISSED, proposed error
  ok    no word changes behaviour the shipped code had       0
  ...
PASS: 0 failure(s)
```

- The test checks every word the error interrupt can build, the receive and transmit words, and all
  65,536 16-bit words. The proposed code changes nothing the shipped code did, and adds an error
  report only for a word carrying an error bit.
- **Mutation check:** with the `default:` arm's condition forced false, the same test reports
  `FAIL: 8 failure(s)` and exits 1.
- **Limit:** the test carries transcriptions of both callbacks, not the driver file itself. Its
  header says so.

Existing host suite, unchanged, both green:

```
$ g++ -std=c++17 -Wall -Wextra -Iinclude test/test_loop_scaling.cpp -o /tmp/t && /tmp/t
all checks passed                      (35 ok, 0 FAIL)
$ g++ -std=c++17 -Wall -Wextra -Iinclude test/test_j1939_replay.cpp -o /tmp/r && \
  /tmp/r ~/plc4-logs/census-2026-09-22/candump-2026-09-22_175858.log
all passed                             (21 ok, 0 FAIL)
```

Cross-compile of route B, Saga, arduino-cli 1.5.0, `arduino:renesas_uno:unor4wifi`, sketch
assembled as `build-check.sh` assembles it, in `/tmp`:

| Build | Arduino_CAN compiled from | Flash | RAM |
|---|---|---|---|
| unpatched | `~/.arduino15/.../libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp` | 104,080 | 17,784 |
| patched, `--library` | `/tmp/prop53-build/lib/Arduino_CAN/src/R7FA4M1_CAN.cpp` | 104,096 | 17,784 |

- The source paths come from each build's `compile_commands.json`, so `--library` does take the
  repo copy over the platform's.
- `--warnings all` raised nothing from the patched file.
- The patched object disassembles to the original comparisons, then `bic r3, #0xC00; cmp r3, #0`,
  setting the error flag when the result is not zero.

## 4. What stays unproven until silicon, and the smallest on-tractor check

**Unproven:**

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

1. **No false positives.** Key on, engine at idle for 15 minutes: `can_err` stays 0 and the frame
   rate matches the ~462 fps seen before. This confirms the new arm does not flag received frames
   on the real binary.
2. **Pre-ring loss, measured directly.** Run a `candump` capture on the tractor bus over the same
   window as the board, the way the 2026-09-22 census was taken, and compare the capture's frame
   count with the change in the board's `frames` over that window. With `ring_full` at 0, any
   shortfall is frames lost before the ring. This answers the issue's second half whether or not
   the silicon ever reports an overwrite.

Do not provoke a bus fault on the tractor to exercise the new error path: that disturbs the
tractor's own bus. If the path needs to be seen working, the two-node bench rig
(`findings/2026-09-20-two-node-bench.md`) is where to do it.

## References

[1] Renesas Electronics, *Flexible Software Package (FSP) v4.0.0*, `r_can` driver, object `r_can.o`
in `libfsp.a`, Arduino build, line table to `ra/fsp/src/r_can/r_can.c`. Shipped in Arduino,
ArduinoCore-renesas 1.6.0, `variants/UNOWIFIR4/libs/libfsp.a`. Renesas proprietary licence (file
header of [2]). Read by disassembly with `arm-none-eabi-objdump -d -l` (gcc 7-2017q4).

[2] Renesas Electronics, FSP v4.0.0, `r_can_api.h` and `instances/r_can.h`, shipped in
ArduinoCore-renesas 1.6.0, `variants/UNOWIFIR4/includes/ra/fsp/inc/`. Renesas proprietary licence.

[3] Arduino (A. Entinger), *ArduinoCore-renesas*, 1.6.0: `libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp`
and `.h` (GPL-2.0 or LGPL-2.1, per file header), `cores/arduino/IRQManager.cpp` and `FspTimer.h`.
https://github.com/arduino/ArduinoCore-renesas

[4] Renesas Electronics, *R7FA4M1AB device header*, `R7FA4M1AB.h`, shipped in ArduinoCore-renesas
1.6.0, `variants/UNOWIFIR4/includes/ra/fsp/src/bsp/cmsis/Device/RENESAS/Include/`.

[5] Renesas Electronics, *RA4M1 Group Datasheet*, R01DS0355EJ0110, Rev. 1.10,
`docs/vendor/renesas/RA4M1_Group_Datasheet_R01DS0355EJ0110.pdf`.
