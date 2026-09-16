#pragma once

// ---------------------------------------------------------------------------
// The loop is read by an MCP3421 on Wire1 (Qwiic), NOT by the R4's internal ADC.
//
// WHY, AND IT IS NOT THE RESOLUTION -- the resolution is the same 14 bits. The
// MCP3421 carries its OWN 2.048 V reference, so the board's supply rail leaves
// the measurement entirely. The retired ratiometric transducer tracked AVCC and
// its error cancelled; a burden resistor does not cancel, and the internal ADC
// references a rail that on USB sits anywhere in 4.75-5.25 V -- up to +/-5% gain
// error against a transmitter rated +/-1.0% FS. That was issue #4, and this
// closes it by construction rather than by someone metering the rail.
//
// 14 BITS IS A RATE CHOICE. The MCP3421 trades resolution against sample rate:
// 18-bit is 3.75 SPS, 16-bit 15 SPS, 14-bit 60 SPS. This firmware samples every
// SAMPLE_SPACING_MS = 50 ms, so 18-bit would take 2.7 s for one 10-sample
// reading and the scale would visibly lag the load. 14-bit is still 43.5x finer
// than the sensor's own +/-30 psi. DO NOT "UPGRADE" THIS.
//
// WHY 68 OHM: the constraint is I * R <= 2.048 V. The binding current is the
// 29.7 mA hard-over PLC #3 actually METERED (playbook s10), giving R <= 68.9,
// so 68 is the largest E24 value. Same rule that picked 150 ohm against a 5 V
// rail. THE OLD 150 OHM CANNOT BE USED HERE -- 2.048/150 clips at 13.65 mA,
// below the 20 mA span end.
//
// Burden tolerance is still a direct gain term: 0.1%, <=25 ppm/degC.
// ---------------------------------------------------------------------------
constexpr float BURDEN_OHMS      = 68.0f;
constexpr float MCP3421_VREF_V   = 2.048f;
constexpr int   MCP3421_BITS     = 14;
constexpr float MCP3421_LSB_V    = 2.0f * MCP3421_VREF_V / (float)(1 << MCP3421_BITS);
constexpr int   MCP3421_MAX_CODE = (1 << (MCP3421_BITS - 1)) - 1;   // 8191

constexpr int mAToCounts(float mA) {
  return (int)(mA * BURDEN_OHMS / 1000.0f / MCP3421_LSB_V + 0.5f);
}

// Backwards compatibility: Arduino sketch still uses these. Both need updating
// to account for the new ADC and reference; this shim allows compilation while
// those updates are staged in separate tasks.
constexpr float ADC_VREF_V = MCP3421_VREF_V;
constexpr int   ADC_FS     = (1 << MCP3421_BITS) - 1;  // 16383

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

constexpr int COUNTS_DEAD       = mAToCounts(LOOP_DEAD_MA);        // 952
constexpr int COUNTS_ZERO       = mAToCounts(LOOP_ZERO_MA);        // 1088
constexpr int COUNTS_FS         = mAToCounts(LOOP_FS_MA);          // 5440
constexpr int COUNTS_FAULT_HIGH = mAToCounts(LOOP_FAULT_HIGH_MA);  // 5712

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
  return (float)counts * MCP3421_LSB_V / BURDEN_OHMS * 1000.0f;
}

// Long arithmetic: (5440-1088)*3000 is 13 million and fits a 32-bit int.
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
