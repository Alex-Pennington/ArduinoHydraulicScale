#pragma once

// ---------------------------------------------------------------------------
// PLC #4 — 4-20 mA current loop front end.
//
// The transmitter is a Rosemount 1151, 0-3000 psi, 2-wire, fed from a DEDICATED
// 24 V supply whose negative is bonded to this board's GND. Loop current is read
// as the voltage across a burden resistor from A2 to GND.
//
// BURDEN_OHMS IS THE ONLY NUMBER HERE THAT DESCRIBES HARDWARE. Change it and
// every threshold below follows. NEVER HAND-WRITE A COUNT: this firmware already
// carries three cases of a hand-derived scaling constant left stale when the
// resolution changed, and a count that silently *means* 21.0 mA would be a
// fourth.
//
// WHY 150 OHM, so nobody re-argues it on resolution grounds: resolution binds on
// nothing -- every candidate from 100 to 250 ohm is 52-131x finer than the
// sensor's own +/-1.0% FS -- and compliance binds on nothing at 24 V. What binds
// is that PLC #3 metered a 29.7 mA hard-over; against 200 ohm that puts 5.94 V
// on this pin, 0.94 V above AVCC and into the ESD clamp. 150 ohm holds to
// 33.3 mA. Full trade table: notes/2026-09-13-4-20ma-loop-decisions.md §2.
//
// BURDEN TOLERANCE IS NOW A CALIBRATION TERM AND IT WAS NOT BEFORE. A 1% burden
// is +/-30 psi at 3000 -- the entire sensor accuracy budget. Specify 0.1%,
// <=25 ppm/degC; at 100 ppm/degC a 60 degC enclosure swing adds another 18 psi.
//
// AND SO IS AVCC. The old transducer was RATIOMETRIC: its output tracked AVCC,
// so AVCC error cancelled. A burden does not cancel -- the reading is I*R/AVCC,
// so ADC_VREF_V below is a direct gain term. 5.0 is an ASSUMPTION about a rail
// that on USB power is typically 4.75-5.25 V. UNRESOLVED; may want measuring.
//
// Deliberately free of Arduino headers so test/test_loop_scaling.cpp can check
// the arithmetic on the host, where nothing else in this firmware can be run.
// ---------------------------------------------------------------------------

constexpr float BURDEN_OHMS = 150.0f;   // 0.1%, <=25 ppm/degC, A2 to GND
constexpr float ADC_VREF_V  = 5.0f;     // see the AVCC warning above
constexpr int   ADC_FS      = 16383;    // 14-bit full scale is 16383, not 16384

constexpr int mAToCounts(float mA) {
  return (int)(mA * BURDEN_OHMS / ADC_VREF_V / 1000.0f * (float)ADC_FS + 0.5f);
}

// NAMUR NE43 and the estate's settled figures. Every value cites a source; none
// is invented here.
//
// WHY 21.0 AND NOT 20.5, because it will be questioned. NE43 puts the valid
// measuring range at 3.8-20.5 mA and the fault signal at >=21.0; 20.5-21.0 is
// over-range-but-still-measuring and is deliberately NOT a fault. 21.0 also sits
// in the gap for both Rosemount alarm configurations (standard saturates 20.8 /
// alarms >=21.75; NE43 option saturates 20.5 / alarms >=21.5), so it
// discriminates correctly without anyone knowing the option code.
//
// A 20.5 THRESHOLD WOULD BE WRONG RATHER THAN CONSERVATIVE: it would report a
// transmitter legitimately pinned at full scale, measuring correctly and saying
// so, as FAILED -- which is precisely the defect this front end exists to fix.
constexpr float LOOP_DEAD_MA       =  3.5f;  // playbook §5; PLC #1 CURRENT_DEAD_LOOP_MA
constexpr float LOOP_ZERO_MA       =  4.0f;  // live zero
constexpr float LOOP_FS_MA         = 20.0f;  // span end
constexpr float LOOP_FAULT_HIGH_MA = 21.0f;  // NAMUR NE43

constexpr int COUNTS_DEAD       = mAToCounts(LOOP_DEAD_MA);        // 1720
constexpr int COUNTS_ZERO       = mAToCounts(LOOP_ZERO_MA);        // 1966
constexpr int COUNTS_FS         = mAToCounts(LOOP_FS_MA);          // 9830
constexpr int COUNTS_FAULT_HIGH = mAToCounts(LOOP_FAULT_HIGH_MA);  // 10321

enum LoopState { LOOP_OK, LOOP_DEAD, LOOP_UNDER, LOOP_SATURATED, LOOP_FAULT_HIGH };

inline LoopState classifyLoop(int counts) {
  if (counts <  COUNTS_DEAD)       return LOOP_DEAD;
  if (counts <  COUNTS_ZERO)       return LOOP_UNDER;
  if (counts >= COUNTS_FAULT_HIGH) return LOOP_FAULT_HIGH;
  if (counts >= COUNTS_FS)         return LOOP_SATURATED;
  return LOOP_OK;
}

// WHETHER TO TRUST THE READING. Distinct from `state != LOOP_OK`, which is
// whether to let the OPERATOR ACT. They differ on LOOP_UNDER and LOOP_SATURATED:
// both still carry a real measurement, and neither is a safe basis for a new
// tare or calibration. Do not collapse the two predicates into one.
inline bool loopIsMeasuring(LoopState s) {
  return s == LOOP_OK || s == LOOP_UNDER || s == LOOP_SATURATED;
}

// Operator-facing, and on the MQTT wire. All fit the 16-column LCD.
// `ok` and `NO SENSOR` keep their exact old spellings so the two values
// downstream already branches on do not move. `SENSOR SHORT` is gone: a
// current loop is current-limited and has no rail to short to.
inline const char *loopStateName(LoopState s) {
  switch (s) {
    case LOOP_OK:         return "ok";
    case LOOP_DEAD:       return "NO SENSOR";
    case LOOP_UNDER:      return "UNDER ZERO";
    case LOOP_SATURATED:  return "SATURATED";
    case LOOP_FAULT_HIGH: return "XMTR FAULT";
    // Reachable only from a raw byte pulled out of a RAM record (queueFlush's
    // WeightRecord::sensorAtCapture) rather than from classifyLoop(), which never
    // returns anything outside the five cases above. A corrupted or stale record
    // must not be able to masquerade as a healthy reading -- "BAD STATE" is a
    // deliberate wire value, 10 characters so it fits the 16-column LCD, and any
    // consumer treating non-"ok" as invalid fails safe on it.
    default:              return "BAD STATE";
  }
}

// The unit a person with a meter at the machine is comparing against.
inline float countsToMilliamps(int counts) {
  return (float)counts * ADC_VREF_V / (float)ADC_FS / BURDEN_OHMS * 1000.0f;
}

// Long arithmetic: (16383-1966)*3000 is 43 million and overflows a 16-bit int.
// FLOORED AT 0 (playbook §4) because LOOP_UNDER sits below the live zero and an
// unclamped conversion would put a negative pressure on the wire.
// Above COUNTS_FS it EXTRAPOLATES deliberately -- a saturating transmitter is
// proportional up to its clamp -- but per playbook §10 a reading past the
// transmitter's rated maximum is a VALIDITY BOUNDARY, NOT A SCALE MARKING.
// Read anything above maxPSI as "at least this much".
inline int countsToPSI(int counts, int maxPSI) {
  long num = (long)(counts - COUNTS_ZERO) * (long)maxPSI;
  long psi = num / (long)(COUNTS_FS - COUNTS_ZERO);
  return psi < 0 ? 0 : (int)psi;
}
