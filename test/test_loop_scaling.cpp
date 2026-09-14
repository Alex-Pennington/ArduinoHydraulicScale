// Host-side check of the 4-20 mA scaling arithmetic in include/loop_scaling.h.
//
// Built with g++ ON THE HOST. The firmware itself cross-compiles fine via
// tools/build-check.sh, but a compile proves only that the arithmetic is legal,
// not that it is right. This file carries no Arduino dependency so the values
// can actually be RUN and inspected, which on a board nobody has tethered is
// otherwise impossible.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_loop_scaling.cpp -o /tmp/t
//   /tmp/t
//
// (No backslash continuation in that comment -- a trailing backslash on a //
// line trips -Wcomment, and this file must build warning-free.)
//
// THIS IS NOT A COMMISSIONING TEST. It proves arithmetic. It proves nothing
// about a transmitter, a burden resistor, or a tractor. Alex writes those.

#include "loop_scaling.h"
#include <cstdio>
#include <cstring>

static int failures = 0;

static void check_int(const char *what, long got, long want) {
  if (got == want) { printf("  ok    %-44s %ld\n", what, got); }
  else { printf("  FAIL  %-44s got %ld want %ld\n", what, got, want); failures++; }
}

static void check_near(const char *what, double got, double want, double tol) {
  double d = got - want; if (d < 0) d = -d;
  if (d <= tol) { printf("  ok    %-44s %.4f\n", what, got); }
  else { printf("  FAIL  %-44s got %.4f want %.4f\n", what, got, want); failures++; }
}

static void check_state(const char *what, LoopState got, LoopState want) {
  if (got == want) { printf("  ok    %-44s %s\n", what, loopStateName(got)); }
  else { printf("  FAIL  %-44s got %s want %s\n", what,
                loopStateName(got), loopStateName(want)); failures++; }
}

static void check_str(const char *what, const char *got, const char *want) {
  if (got == want || (got && want && strcmp(got, want) == 0)) {
    printf("  ok    %-44s %s\n", what, got);
  } else {
    printf("  FAIL  %-44s got %s want %s\n", what, got, want); failures++;
  }
}

// The derived counts, against the values in MA-LOOP-DESIGN.md §2. If the burden
// is ever changed these must be recomputed there and here together.
static_assert(COUNTS_DEAD       == 1720,  "3.5 mA against 150 ohm");
static_assert(COUNTS_ZERO       == 1966,  "4.0 mA against 150 ohm");
static_assert(COUNTS_FS         == 9830,  "20.0 mA against 150 ohm");
static_assert(COUNTS_FAULT_HIGH == 10321, "21.0 mA against 150 ohm");

int main() {
  printf("counts derived from BURDEN_OHMS\n");
  check_int("3.5 mA  live-zero floor", COUNTS_DEAD,        1720);
  check_int("4.0 mA  live zero",       COUNTS_ZERO,        1966);
  check_int("20.0 mA full scale",      COUNTS_FS,          9830);
  check_int("21.0 mA NE43 fault-high", COUNTS_FAULT_HIGH, 10321);
  check_int("25.0 mA stays on scale",  mAToCounts(25.0f) <= 16383, 1);

  printf("\nclassifier boundaries -- every band edge, both sides\n");
  // AN OPEN LOOP READS ZERO, AND THAT IS THE WHOLE POINT OF THE SWAP.
  // The old front end's open-circuit defect was a floating CMOS input sitting
  // near mid-rail at ~4401 counts and reporting ok. There is deliberately NO
  // test for 4401 here: with a 150 ohm burden across the input a broken loop
  // CANNOT produce it -- 0 mA develops 0 V. The hardware closed that path, not
  // the classifier, and a test asserting otherwise would misrepresent the fix.
  check_state("0 counts -- open loop",        classifyLoop(0),     LOOP_DEAD);
  check_state("1719 -- just under 3.5 mA",    classifyLoop(1719),  LOOP_DEAD);
  check_state("1720 -- exactly 3.5 mA",       classifyLoop(1720),  LOOP_UNDER);
  check_state("1965 -- just under live zero", classifyLoop(1965),  LOOP_UNDER);
  check_state("1966 -- exactly live zero",    classifyLoop(1966),  LOOP_OK);
  check_state("9829 -- just under full scale",classifyLoop(9829),  LOOP_OK);
  check_state("9830 -- exactly full scale",   classifyLoop(9830),  LOOP_SATURATED);
  check_state("10320 -- just under NE43",     classifyLoop(10320), LOOP_SATURATED);
  check_state("10321 -- exactly NE43 21 mA",  classifyLoop(10321), LOOP_FAULT_HIGH);
  check_state("16383 -- ADC rail",            classifyLoop(16383), LOOP_FAULT_HIGH);

  printf("\nwhich states are still measuring\n");
  // SATURATED IS MEASURING. NE43 puts 20.0-20.5 mA in the valid measuring range
  // and this is the defect the swap exists to fix: today a real pressure
  // excursion publishes as a wiring fault. If this line ever flips to false,
  // the fix has been undone.
  check_int("LOOP_OK measures",         loopIsMeasuring(LOOP_OK),         1);
  check_int("LOOP_UNDER measures",      loopIsMeasuring(LOOP_UNDER),      1);
  check_int("LOOP_SATURATED MEASURES",  loopIsMeasuring(LOOP_SATURATED),  1);
  check_int("LOOP_DEAD does not",       loopIsMeasuring(LOOP_DEAD),       0);
  check_int("LOOP_FAULT_HIGH does not", loopIsMeasuring(LOOP_FAULT_HIGH), 0);

  printf("\nloopStateName -- the wire/LCD string for every state, by name\n");
  // Regression cover for queueFlush(), which hands loopStateName() a raw byte
  // pulled out of a RAM record (WeightRecord::sensorAtCapture) rather than a
  // value classifyLoop() produced. Before this round LOOP_OK fell through to
  // the same "ok" default as anything out of range, so a corrupted or stale
  // record and a healthy reading were indistinguishable on the wire.
  check_str("LOOP_OK",         loopStateName(LOOP_OK),         "ok");
  check_str("LOOP_DEAD",       loopStateName(LOOP_DEAD),       "NO SENSOR");
  check_str("LOOP_UNDER",      loopStateName(LOOP_UNDER),      "UNDER ZERO");
  check_str("LOOP_SATURATED",  loopStateName(LOOP_SATURATED),  "SATURATED");
  check_str("LOOP_FAULT_HIGH", loopStateName(LOOP_FAULT_HIGH), "XMTR FAULT");
  // Out-of-range: a corrupted or stale sensorAtCapture byte must fail safe,
  // not masquerade as "ok". This is the case classifyLoop() itself can never
  // produce -- only a raw byte from a record can reach it.
  check_str("(LoopState)7 -- out of range", loopStateName((LoopState)7), "BAD STATE");

  printf("\ncounts back to milliamps\n");
  check_near("live zero is 4 mA",  countsToMilliamps(COUNTS_ZERO),  4.0,  0.01);
  check_near("full scale is 20 mA",countsToMilliamps(COUNTS_FS),   20.0,  0.01);
  check_near("zero counts is 0 mA",countsToMilliamps(0),            0.0,  0.01);

  printf("\ncounts to psi, span 3000\n");
  check_int("live zero is 0 psi",      countsToPSI(COUNTS_ZERO, 3000),    0);
  check_int("full scale is 3000 psi",  countsToPSI(COUNTS_FS,   3000), 3000);
  check_int("midspan is 1500 psi",     countsToPSI(5898,        3000), 1500);
  // The floor. map() does not clamp, and LOOP_UNDER sits below the live zero,
  // so without this a healthy-looking negative pressure reaches the wire.
  check_int("below live zero floors at 0", countsToPSI(COUNTS_DEAD, 3000), 0);
  check_int("zero counts floors at 0",     countsToPSI(0,           3000), 0);
  // Above full scale it EXTRAPOLATES, deliberately: a saturating transmitter is
  // proportional up to its clamp. Per playbook §10 this is a validity boundary,
  // not a scale marking -- read it as "at least this much".
  check_int("saturated extrapolates",  countsToPSI(10000, 3000), 3064);

  printf("\n%s\n", failures ? "FAILURES" : "all checks passed");
  return failures ? 1 : 0;
}
