#pragma once
// The CAN side of PLC #4 -- issue #24, the reader folded into the scale firmware.
//
// WHY THE DRAIN IS IN A TIMER ISR, AND NOT IN loop().
// Arduino_CAN's receive interrupt puts each frame in a 32-slot ring that drops
// the NEWEST frame when full and counts nothing. The 2026-09-22 census off this
// tractor measured 465 fps at idle and 1,590 fps during engine start, so the ring
// fills in 69 ms at idle and in 20 ms at start. loop() cannot promise either: its
// WiFi.begin() blocks for up to 23 s and a QoS 1 publish spins for its PUBACK
// (see netPoll()). Drained from loop(), every reconnect would be a blackout.
// findings/2026-09-20-soak-and-drain-cadence.md measured 67% loss that way.
//
// A timer interrupt keeps running through all of those blocking calls. At
// CAN_DRAIN_HZ = 200 it empties the ring every 5 ms -- 8 frames at the start
// burst, a quarter of the ring -- and decodes in place, which is cheap: a switch
// on the PGN and a few multiplies (include/j1939_decode.h is allocation-free).
//
// SHARING WITH loop(). The ISR is the ONLY writer of canState. loop() reads it
// through the snapshot helpers below, which copy under noInterrupts() so a
// publish never sees a half-written value. The one field loop() writes back --
// a DM1 slot's "published" mark -- also goes through a helper under the same guard.
//
// THIS NODE NEVER TRANSMITS. Nothing here calls CAN.write(). In normal mode the
// controller still ACKs frames on the bus; the hardware guarantee of silence is
// the transceiver's D pin tied recessive (findings/2026-09-20-two-node-bench.md),
// which is wiring, not firmware.

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "FspTimer.h"
#include "j1939_decode.h"

static const uint32_t CAN_BITRATE  = 250000UL;   // measured: 2026-09-22 census, 250k, error counters 0
static const float    CAN_DRAIN_HZ = 200.0f;

static J1939State canState;

static volatile bool     canActive     = false;  // CAN.begin() succeeded and the drain may run
static volatile bool     canFiltered   = false;  // hardware acceptance filter in force
static bool              canWantFilter = false;  // what `canf` last asked for
static volatile uint32_t canRingFull   = 0;      // drains that found the ring FULL: frames were lost
static volatile uint32_t canRingHigh   = 0;      // deepest the ring has been, of 32
static volatile uint32_t canErrEvents  = 0;      // times the driver flagged an error event
static volatile int      canLastErr    = 0;      // the last driver error code (can_event_t)
// How many times the drain has RUN, frames or not. With `frames` at 0 this is what
// tells "the drain never runs" (drains 0) from "nothing is arriving at the pin"
// (drains climbing at CAN_DRAIN_HZ) -- the two look identical otherwise.
static volatile uint32_t canDrains     = 0;
static bool              canTimerOn    = false;
static FspTimer          canTimer;

static void canDrainISR(timer_callback_args_t *) {
  if (!canActive) return;
  canDrains++;
  const size_t n = CAN.available();
  if (n > canRingHigh) canRingHigh = n;
  if (n >= CanMsgRingbuffer::RING_BUFFER_SIZE) canRingFull++;

  int err = 0;
  if (CAN.isError(err)) { canErrEvents++; canLastErr = err; CAN.clearError(); }

  const uint32_t now = millis();
  // Bounded, so a pathological flood can never pin the CPU inside this ISR: the
  // ring holds at most 32, and anything arriving meanwhile waits 5 ms.
  for (size_t i = 0; i < CanMsgRingbuffer::RING_BUFFER_SIZE && CAN.available(); i++) {
    const CanMsg m = CAN.read();
    const bool ext = m.isExtendedId();
    const uint8_t len = m.data_length > 8 ? 8 : m.data_length;
    j1939OnFrame(canState, ext, ext ? m.getExtendedId() : m.getStandardId(), m.data, len, now);
  }
  j1939Tick(canState, now);
}

// Configure the acceptance filter, then open the controller. The FSP reads the
// mailbox table at open, so the filter must be set BEFORE CAN.begin(); changing
// it means closing and reopening, which is what canSetFilter() does.
static bool canOpen(bool filtered) {
  if (filtered) {
    CAN.setFilterMask_Extended(J1939_FILTER_MASK);
    for (size_t i = 0; i < 8; i++) CAN.setFilterId_Extended(i, J1939_FILTER_IDS[i]);
  } else {
    CAN.setFilterMask_Extended(0);   // mask 0 = compare nothing = accept every extended id
  }
  const bool ok = CAN.begin(CAN_BITRATE);
  canFiltered = ok && filtered;
  return ok;
}

// Called once from setup(). Returns whether the controller opened. The scale
// does not depend on this: a board with no transceiver fitted weighs exactly
// as before and publishes no engine data.
static bool canBegin(bool filtered) {
  j1939Init(canState);
  const bool ok = canOpen(filtered);

  uint8_t timerType;
  const int8_t ch = FspTimer::get_available_timer(timerType);
  if (ch >= 0) {
    canTimerOn = canTimer.begin(TIMER_MODE_PERIODIC, timerType, (uint8_t)ch,
                                CAN_DRAIN_HZ, 0.0f, canDrainISR)
                 && canTimer.setup_overflow_irq()
                 && canTimer.open()
                 && canTimer.start();
  }
  canActive = ok;
  return ok;
}

// Fallback for a board where no timer was free: drain from loop(), which loses
// frames during blocking network calls but is better than nothing. Never runs
// when the timer is on.
// Under noInterrupts(): the timer path runs at the CAN ISRs' own priority and
// cannot be preempted by them, but from loop() a CAN error interrupt could land
// between isError() and clearError() and be wiped out.
static void canPollFromLoop() {
  if (canTimerOn || !canActive) return;
  noInterrupts();
  canDrainISR(nullptr);
  interrupts();
}

// The `canf` verb: reopen with the filter on or off. Decode state is kept.
// If a FILTERED reopen fails, fall straight back to unfiltered -- the known-good
// configuration -- so a failed trial can never leave the board deaf, and the
// next `canf` still toggles from the state actually in force.
static bool canSetFilter(bool filtered) {
  canActive = false;               // the ISR stops touching CAN before it is closed
  CAN.end();
  canWantFilter = filtered;
  bool ok = canOpen(filtered);
  if (!ok && filtered) {
    CAN.end();
    canWantFilter = false;
    ok = canOpen(false);
  }
  canActive = ok;
  return ok;
}

// ---------------------------------------------------------------------------
// Snapshots for loop(). Each copies under noInterrupts(); the copies are small
// (one signal set, or one DM1 slot) so the ISR is held off for microseconds.

struct CanSigSnapshot {
  J1939Sig rpm, torquePct, loadPct, coolantC, oilTempC;
  J1939Sig oilKpa, hours, fuelLph, battV, keyswV;
  uint32_t framesSeen, framesUnknown;
};

static void canSnapshotSigs(CanSigSnapshot &o) {
  noInterrupts();
  o.rpm = canState.rpm;           o.torquePct = canState.torquePct;
  o.loadPct = canState.loadPct;   o.coolantC = canState.coolantC;
  o.oilTempC = canState.oilTempC; o.oilKpa = canState.oilKpa;
  o.hours = canState.hours;       o.fuelLph = canState.fuelLph;
  o.battV = canState.battV;       o.keyswV = canState.keyswV;
  o.framesSeen = canState.framesSeen;
  o.framesUnknown = canState.framesUnknown;
  interrupts();
}

// Copies slot i if it is in use AND has changed since it was last published.
static bool canDm1TakeDirty(size_t i, J1939Dm1Src &o) {
  bool take = false;
  noInterrupts();
  if (canState.dm1[i].used && canState.dm1[i].dirty) { o = canState.dm1[i]; take = true; }
  interrupts();
  return take;
}

// Mark slot i published -- but only if the ISR has not changed it since the
// snapshot was taken. If it has, it stays dirty and goes out on the next pass.
static void canDm1MarkPublished(size_t i, uint32_t fingerprint) {
  noInterrupts();
  if (canState.dm1[i].fingerprint == fingerprint) {
    canState.dm1[i].lastFingerprint = fingerprint;
    canState.dm1[i].dirty = false;
  }
  interrupts();
}

// After a (re)connect: republish every source's retained fault state. A retained
// message the broker dropped while we were away is one nobody will resend.
static void canDm1ForceDirty() {
  noInterrupts();
  for (size_t i = 0; i < J1939_MAX_DM1_SRC; i++)
    if (canState.dm1[i].used) canState.dm1[i].dirty = true;
  interrupts();
}
