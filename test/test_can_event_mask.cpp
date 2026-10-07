// PROPOSED -- issue #53. Host check of the CAN error-event classification.
//
// Built with g++ ON THE HOST, like test_loop_scaling.cpp. It runs the two pure
// functions the proposed firmware would call, from include/can_event_mask.h:
//
//   canEventIsError()  -- what the callback hook calls an error
//   canMergeError()    -- what one drain reports, from the library's flag and the hook's
//
// against every event word the FSP driver can deliver, and compares the result with
// Arduino_CAN as shipped.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_can_event_mask.cpp -o /tmp/m
//   /tmp/m
//
// What the FSP passes as `event` was read from the disassembly of r_can.o in
// variants/UNOWIFIR4/libs/libfsp.a (FSP 4.0.0), with its r_can.c line table:
//
//   can_error_isr  event = EIFR                  (r_can.c:899)
//                  event |= MAILBOX_MESSAGE_LOST if STR.NMLST  (r_can.c:908-910)
//                  runs only when an EIER-enabled flag is set; EIER = 0x3E
//                  (r_can.c:418): EWIE EPIE BOEIE BORIE ORIE
//   can_rx_isr     event = RX_COMPLETE (1024), a constant  (r_can.c:980)
//   can_tx_isr     event = TX_COMPLETE (2048), a constant  (r_can.c:1071)
//
// LIMIT. libraryCallback() below is a TRANSCRIPTION of the library's switch
// (renesas_uno 1.6.0, libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp:276-307), because that
// file needs the FSP and cannot build on the host. If a core upgrade changes that
// switch, this test will not notice. The firmware side guards only the event
// numbering (static_asserts in can_event_mask.h).
//
// THIS IS NOT A COMMISSIONING TEST. It proves the classification arithmetic over
// the words the driver can produce. It proves nothing about which EIFR bits the
// RA4M1 actually sets on a given bus fault. That needs silicon.

#include "can_event_mask.h"
#include <cstdint>
#include <cstdio>

static int failures = 0;

static void check(const char *what, bool ok, const char *detail) {
  printf("  %-5s %-52s %s\n", ok ? "ok" : "FAIL", what, detail);
  if (!ok) failures++;
}

// can_event_t, copied from variants/UNOWIFIR4/includes/ra/fsp/inc/api/r_can_api.h:69-83.
enum : uint32_t {
  CAN_EVENT_ERR_WARNING          = 2,
  CAN_EVENT_ERR_PASSIVE          = 4,
  CAN_EVENT_ERR_BUS_OFF          = 8,
  CAN_EVENT_BUS_RECOVERY         = 16,
  CAN_EVENT_MAILBOX_MESSAGE_LOST = 32,
  CAN_EVENT_ERR_BUS_LOCK         = 128,
  CAN_EVENT_ERR_CHANNEL          = 256,
  CAN_EVENT_TX_ABORTED           = 512,
  CAN_EVENT_RX_COMPLETE          = 1024,
  CAN_EVENT_TX_COMPLETE          = 2048,
  CAN_EVENT_ERR_GLOBAL           = 4096,
  CAN_EVENT_TX_FIFO_EMPTY        = 8192,
};

static const uint32_t EIER_VALUE = 0x3E;   // what R_CAN_Open writes (r_can.c:418)

// What the library's callback did with one event word.
struct Lib {
  bool enqueued;   // RX path: frame put in the software ring
  bool isError;    // _is_error set
  int code;        // _err_code
};

// TRANSCRIPTION of R7FA4M1_CAN::onCanCallback, R7FA4M1_CAN.cpp:276-307.
static Lib libraryCallback(uint32_t event) {
  Lib o = {false, false, 0};
  switch (event) {
    case CAN_EVENT_TX_COMPLETE: break;
    case CAN_EVENT_RX_COMPLETE: o.enqueued = true; break;
    case CAN_EVENT_ERR_WARNING:
    case CAN_EVENT_ERR_PASSIVE:
    case CAN_EVENT_ERR_BUS_OFF:
    case CAN_EVENT_BUS_RECOVERY:
    case CAN_EVENT_MAILBOX_MESSAGE_LOST:
    case CAN_EVENT_ERR_BUS_LOCK:
    case CAN_EVENT_ERR_CHANNEL:
    case CAN_EVENT_TX_ABORTED:
    case CAN_EVENT_ERR_GLOBAL:
    case CAN_EVENT_TX_FIFO_EMPTY:
      o.isError = true; o.code = (int)event; break;
  }
  return o;
}

// What one drain reports after one event.
struct Drain {
  bool enqueued;
  bool counted;    // can_err incremented
  int last;        // can_last
};

// Today: the drain reads the library's flag only (j1939_can.h:59-60).
static Drain drainShipped(uint32_t event) {
  const Lib l = libraryCallback(event);
  return {l.enqueued, l.isError, l.isError ? l.code : 0};
}

// Proposed: the hook records the word, then forwards to the library unchanged;
// the drain merges both. Uses the REAL functions from can_event_mask.h.
static Drain drainProposed(uint32_t event, bool hooked) {
  bool hookErr = false; uint32_t hookWord = 0;
  if (hooked && canEventIsError(event)) { hookErr = true; hookWord = event; }
  const Lib l = libraryCallback(event);
  int last = 0;
  const bool counted = canMergeError(l.isError, l.code, hookErr, hookWord, last);
  return {l.enqueued, counted, counted ? last : 0};
}

// The word can_error_isr builds from EIFR and STR.NMLST.
static uint32_t errorIsrWord(uint32_t eifr, bool nmlst) {
  return eifr | (nmlst ? (uint32_t)CAN_EVENT_MAILBOX_MESSAGE_LOST : 0u);
}

int main() {
  char d[96];

  printf("Every word can_error_isr can deliver (EIFR & EIER != 0, NMLST either way)\n");
  {
    bool seen[256] = {};
    unsigned reachable = 0, shippedCaught = 0, proposedCaught = 0, codeExact = 0;
    for (uint32_t eifr = 0; eifr < 256; eifr++) {
      if ((eifr & EIER_VALUE) == 0) continue;     // no enabled flag: the ISR does not fire
      for (int n = 0; n < 2; n++) {
        const uint32_t w = errorIsrWord(eifr, n != 0);
        if (seen[w]) continue;
        seen[w] = true;
        reachable++;
        const Drain s = drainShipped(w), p = drainProposed(w, true);
        if (s.counted) shippedCaught++;
        if (p.counted) proposedCaught++;
        if (p.counted && p.last == (int)w) codeExact++;
      }
    }
    snprintf(d, sizeof(d), "%u distinct words", reachable);
    check("reachable error-ISR words", reachable == 248, d);
    snprintf(d, sizeof(d), "%u of %u", proposedCaught, reachable);
    check("proposed counts every one in can_err", proposedCaught == reachable, d);
    snprintf(d, sizeof(d), "%u of %u", codeExact, reachable);
    check("proposed keeps the whole word in can_last", codeExact == reachable, d);
    snprintf(d, sizeof(d), "%u of %u (misses %u)", shippedCaught, reachable,
             reachable - shippedCaught);
    check("shipped counts only the single-flag words", shippedCaught == 5, d);
  }

  printf("Named cases\n");
  {
    struct Case { const char *name; uint32_t w; bool shippedErr; };
    const Case cases[] = {
      {"0x20 lost mailbox alone",             0x20, true},
      {"0x22 lost mailbox + error warning",   0x22, false},   // the issue's example
      {"0x03 bus error + error warning",      0x03, false},
      {"0x07 bus error + warning + passive",  0x07, false},
      {"0x09 bus error + bus off",            0x09, false},
      {"0x42 overload + error warning",       0x42, false},
      {"0x02 error warning alone",            0x02, true},
      {"0x08 bus off alone",                  0x08, true},
    };
    for (const Case &c : cases) {
      const Drain s = drainShipped(c.w), p = drainProposed(c.w, true);
      snprintf(d, sizeof(d), "shipped %s, proposed %s",
               s.counted ? "counted" : "MISSED", p.counted ? "counted" : "MISSED");
      check(c.name, s.counted == c.shippedErr && p.counted && p.last == (int)c.w, d);
    }
  }

  printf("RX and TX paths unchanged\n");
  {
    const Drain r = drainProposed(CAN_EVENT_RX_COMPLETE, true);
    check("RX_COMPLETE enqueues, not counted", r.enqueued && !r.counted,
          r.enqueued ? "enqueued" : "DROPPED");
    const Drain t = drainProposed(CAN_EVENT_TX_COMPLETE, true);
    check("TX_COMPLETE neither enqueues nor counts", !t.enqueued && !t.counted, "");
    const Drain z = drainProposed(0, true);
    check("word 0 sets nothing", !z.enqueued && !z.counted, "");
  }

  printf("Exhaustive over every 16-bit word (can_event_t is 16 bits on this target)\n");
  {
    unsigned regressions = 0, added = 0, wrongAdd = 0, unhookedDiffers = 0;
    for (uint32_t w = 0; w <= 0xFFFF; w++) {
      const Drain s = drainShipped(w), p = drainProposed(w, true), u = drainProposed(w, false);
      // Anything the shipped code did, the proposal does identically.
      if (s.enqueued != p.enqueued) regressions++;
      if (s.counted && (!p.counted || p.last != s.last)) regressions++;
      // Anything new is a count, and only for a word with a non-RX/TX bit.
      if (!s.counted && p.counted) {
        added++;
        if ((w & ~(uint32_t)(CAN_EVENT_RX_COMPLETE | CAN_EVENT_TX_COMPLETE)) == 0) wrongAdd++;
      }
      // Hook not installed: exactly today's behaviour.
      if (u.enqueued != s.enqueued || u.counted != s.counted || u.last != s.last) unhookedDiffers++;
    }
    snprintf(d, sizeof(d), "%u", regressions);
    check("no word changes behaviour the shipped code had", regressions == 0, d);
    snprintf(d, sizeof(d), "%u added, %u without an error bit", added, wrongAdd);
    check("every added count carries an error bit", wrongAdd == 0, d);
    // Only RX, TX, 0 and RX|TX (which no ISR builds) are not errors; 10 were already caught.
    check("added + shipped-errors + 4 non-errors == 65536", added + 10 + 4 == 65536, "");
    snprintf(d, sizeof(d), "%u words differ", unhookedDiffers);
    check("hook not installed == shipped, every word", unhookedDiffers == 0, d);
  }

  printf("canMergeError on its own\n");
  {
    int c = -7;
    check("neither flag: not counted, code untouched",
          !canMergeError(false, 0, false, 0, c) && c == -7, "");
    check("library only: counted, library code", canMergeError(true, 8, false, 0, c) && c == 8, "");
    check("hook only: counted, hook word", canMergeError(false, 0, true, 0x22, c) && c == 0x22, "");
    check("both: the hook word wins", canMergeError(true, 2, true, 0x03, c) && c == 0x03, "");
  }

  printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
