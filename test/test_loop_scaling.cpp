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

static_assert(COUNTS_DEAD       ==  952, "3.5 mA against 68 ohm, 14-bit, 2.048 V");
static_assert(COUNTS_ZERO       == 1088, "4.0 mA");
static_assert(COUNTS_FS         == 5440, "20.0 mA");
static_assert(COUNTS_FAULT_HIGH == 5712, "21.0 mA NE43");

int main() {
  printf("counts derived from BURDEN_OHMS\n");
  check_int("3.5 mA  live-zero floor", COUNTS_DEAD,        952);
  check_int("4.0 mA  live zero",       COUNTS_ZERO,       1088);
  check_int("20.0 mA full scale",      COUNTS_FS,         5440);
  check_int("21.0 mA NE43 fault-high", COUNTS_FAULT_HIGH, 5712);
  // 29.7 mA is the hard-over PLC #3 actually METERED. It must stay on scale;
  // that is what sized the burden. 14-bit positive full scale is 8191.
  check_int("29.7 mA measured hard-over stays on scale", mAToCounts(29.7f) < 8191, 1);

  printf("\nclassifier boundaries -- every band edge, both sides\n");
  check_state("0 counts -- open loop",         classifyLoop(0),    LOOP_DEAD);
  check_state("951 -- just under 3.5 mA",      classifyLoop(951),  LOOP_DEAD);
  check_state("952 -- exactly 3.5 mA",         classifyLoop(952),  LOOP_UNDER);
  check_state("1087 -- just under live zero",  classifyLoop(1087), LOOP_UNDER);
  check_state("1088 -- exactly live zero",     classifyLoop(1088), LOOP_OK);
  check_state("5439 -- just under full scale", classifyLoop(5439), LOOP_OK);
  check_state("5440 -- exactly full scale",    classifyLoop(5440), LOOP_SATURATED);
  check_state("5711 -- just under NE43",       classifyLoop(5711), LOOP_SATURATED);
  check_state("5712 -- exactly NE43 21 mA",    classifyLoop(5712), LOOP_FAULT_HIGH);
  check_state("8191 -- ADC positive rail",     classifyLoop(8191), LOOP_FAULT_HIGH);

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
  check_int("live zero is 0 psi",          countsToPSI(COUNTS_ZERO, 3000),    0);
  check_int("full scale is 3000 psi",      countsToPSI(COUNTS_FS,   3000), 3000);
  check_int("midspan is 1500 psi",         countsToPSI(3264,        3000), 1500);
  check_int("below live zero floors at 0", countsToPSI(COUNTS_DEAD, 3000),    0);
  check_int("zero counts floors at 0",     countsToPSI(0,           3000),    0);
  check_int("saturated extrapolates",      countsToPSI(5600,        3000), 3110);

  printf("\n%s\n", failures ? "FAILURES" : "all checks passed");
  return failures ? 1 : 0;
}
