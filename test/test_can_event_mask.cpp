// PROPOSED -- issue #53. Host check of the CAN error-event classification.
//
// Built with g++ ON THE HOST, like test_loop_scaling.cpp. It compares two copies
// of Arduino_CAN's R7FA4M1_CAN::onCanCallback() over every event word the FSP
// driver can hand it:
//
//   classifyShipped()  -- the switch as shipped in renesas_uno 1.6.0,
//                         libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp:274-308
//   classifyProposed() -- the same switch plus the `default:` arm proposed in
//                         findings/2026-10-07-can-err-multibit-proposal.md
//
// Both are TRANSCRIPTIONS. If the proposal changes, change classifyProposed() to
// match the diff in that finding, or this test proves nothing.
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
// THIS IS NOT A COMMISSIONING TEST. It proves the classification arithmetic over
// the words the driver can produce. It proves nothing about which EIFR bits the
// RA4M1 actually sets on a given bus fault. That needs silicon.

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

// EIFR and STR bits, from R7FA4M1AB.h:13579-13594 and :13512-13513.
static const uint32_t EIFR_ORIF  = 0x20;   // same value as MAILBOX_MESSAGE_LOST
static const uint32_t EIER_VALUE = 0x3E;   // what R_CAN_Open writes (r_can.c:418)

// What the callback did with one event word.
struct Outcome {
  bool enqueued;   // RX path: frame put in the software ring
  bool isError;    // _is_error set
  uint32_t code;   // _err_code
};

// The shipped switch, R7FA4M1_CAN.cpp:276-307.
static Outcome classifyShipped(uint32_t event) {
  Outcome o = {false, false, 0};
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
      o.isError = true; o.code = event; break;
  }
  return o;
}

// The proposed switch: identical, plus the default arm.
static Outcome classifyProposed(uint32_t event) {
  Outcome o = {false, false, 0};
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
      o.isError = true; o.code = event; break;
    default:
      if (event & ~(uint32_t)(CAN_EVENT_RX_COMPLETE | CAN_EVENT_TX_COMPLETE)) {
        o.isError = true; o.code = event;
      }
      break;
  }
  return o;
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
        const Outcome s = classifyShipped(w), p = classifyProposed(w);
        if (s.isError) shippedCaught++;
        if (p.isError) proposedCaught++;
        if (p.isError && p.code == w) codeExact++;
      }
    }
    snprintf(d, sizeof(d), "%u distinct words", reachable);
    check("reachable error-ISR words", reachable == 248, d);
    snprintf(d, sizeof(d), "%u of %u", proposedCaught, reachable);
    check("proposed sets _is_error for every one", proposedCaught == reachable, d);
    snprintf(d, sizeof(d), "%u of %u", codeExact, reachable);
    check("proposed keeps the whole word in _err_code", codeExact == reachable, d);
    snprintf(d, sizeof(d), "%u of %u (misses %u)", shippedCaught, reachable,
             reachable - shippedCaught);
    check("shipped catches only the single-flag words", shippedCaught == 5, d);
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
      const Outcome s = classifyShipped(c.w), p = classifyProposed(c.w);
      snprintf(d, sizeof(d), "shipped %s, proposed %s",
               s.isError ? "error" : "MISSED", p.isError ? "error" : "MISSED");
      check(c.name, s.isError == c.shippedErr && p.isError && p.code == c.w, d);
    }
  }

  printf("RX and TX paths unchanged\n");
  {
    const Outcome rs = classifyShipped(CAN_EVENT_RX_COMPLETE);
    const Outcome rp = classifyProposed(CAN_EVENT_RX_COMPLETE);
    check("RX_COMPLETE enqueues, no error (proposed)", rp.enqueued && !rp.isError,
          rp.enqueued ? "enqueued" : "DROPPED");
    check("RX_COMPLETE same as shipped", rs.enqueued == rp.enqueued && rs.isError == rp.isError, "");
    const Outcome tp = classifyProposed(CAN_EVENT_TX_COMPLETE);
    check("TX_COMPLETE no error, no enqueue (proposed)", !tp.isError && !tp.enqueued, "");
    const Outcome zp = classifyProposed(0);
    check("word 0 sets nothing", !zp.isError && !zp.enqueued, "");
  }

  printf("Exhaustive over every 16-bit word (can_event_t is 16 bits on this target)\n");
  {
    unsigned regressions = 0, added = 0, wrongAdd = 0;
    for (uint32_t w = 0; w <= 0xFFFF; w++) {
      const Outcome s = classifyShipped(w), p = classifyProposed(w);
      // Anything the shipped code did, the proposal does identically.
      if (s.enqueued != p.enqueued) regressions++;
      if (s.isError && (!p.isError || p.code != s.code)) regressions++;
      // Anything new is an error report, and only for a word with a non-RX/TX bit.
      if (!s.isError && p.isError) {
        added++;
        if ((w & ~(uint32_t)(CAN_EVENT_RX_COMPLETE | CAN_EVENT_TX_COMPLETE)) == 0) wrongAdd++;
      }
    }
    snprintf(d, sizeof(d), "%u", regressions);
    check("no word changes behaviour the shipped code had", regressions == 0, d);
    snprintf(d, sizeof(d), "%u added, %u without an error bit", added, wrongAdd);
    check("every added report carries an error bit", wrongAdd == 0, d);
    // Sanity on the counter: only RX, TX and 0 are not errors under the proposal,
    // plus RX|TX together, which no ISR builds.
    check("added + shipped-errors + 4 non-errors == 65536",
          added + 10 + 4 == 65536, "");
  }

  printf("Note: EIFR_ORIF and MAILBOX_MESSAGE_LOST share bit 5 (0x%02X)\n", (unsigned)EIFR_ORIF);
  printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
