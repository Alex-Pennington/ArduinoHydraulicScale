#pragma once
// Issue #53. Used by include/j1939_can.h; see
// findings/2026-10-07-can-err-multibit-proposal.md.
//
// Which CAN driver event words are errors. Pure, no Arduino dependency, so the host
// test (test/test_can_event_mask.cpp) runs THIS function rather than a copy of it.
//
// The FSP error ISR passes the error-flag register EIFR as the event, OR'd with
// MAILBOX_MESSAGE_LOST when STR.NMLST is set. One word can therefore carry several
// flags, and EIFR bits 0 (bus error) and 6 (overload) name no can_event_t. The RX
// and TX ISRs pass exactly RX_COMPLETE and TX_COMPLETE. So: any bit other than
// those two is an error.

#include <stdint.h>

static const uint32_t CAN_EV_RX_COMPLETE = 1024u;   // r_can_api.h:79
static const uint32_t CAN_EV_TX_COMPLETE = 2048u;   // r_can_api.h:80

static inline bool canEventIsError(uint32_t event) {
  return (event & ~(CAN_EV_RX_COMPLETE | CAN_EV_TX_COMPLETE)) != 0u;
}

// What one drain reports, given the library's own error flag and the hook's. The
// hook's word wins because it keeps every flag; the library's flag still counts
// when the hook is not installed, which is the behaviour before #53.
static inline bool canMergeError(bool libErr, int libCode, bool hookErr, uint32_t hookWord,
                                 int &code) {
  if (hookErr) { code = (int)hookWord; return true; }
  if (libErr)  { code = libCode;       return true; }
  return false;
}

#ifdef ARDUINO
// On the board, fail the build if a core upgrade renumbers the events this relies on.
#include <Arduino_CAN.h>
static_assert(CAN_EVENT_RX_COMPLETE == CAN_EV_RX_COMPLETE, "can_event_t renumbered: re-check #53");
static_assert(CAN_EVENT_TX_COMPLETE == CAN_EV_TX_COMPLETE, "can_event_t renumbered: re-check #53");
static_assert(CAN_EVENT_MAILBOX_MESSAGE_LOST == 32, "can_event_t renumbered: re-check #53");
#endif
