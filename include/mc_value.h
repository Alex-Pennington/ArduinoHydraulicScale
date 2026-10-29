#pragma once
// FW-8: moisture content on a RECORD, design §4.14, matched to the pendant's `mc`
// (mesh 3a3e9c21c, Plc4Operator.h kMcMinTenths..kMcMaxTenths, mcText(), cmdBody()).
//
// What the panel takes, greybeard-plc4-pwa df172d2:
//   * api/moisture.py:49-50  MC_MIN_TENTHS 50, MC_MAX_TENTHS 600 (5.0 to 60.0 %)
//   * api/moisture.py:63     the typed form \A([0-9]{1,2})(?:\.([0-9])?)?\Z, a JSON STRING
//   * api/moisture.py:70-79  parse_tenths(): a non-string or a value out of range is a 400
//   * api/app.py:257-269     `mc` is taken on `+` only; any other verb carrying it is a 400
//   * api/simple_tally.py:437-448  the mc is pinned to the id at its first send; a resend under
//                                  the same id naming a different mc is a 409
//
// So the board holds mc as integer tenths (no float anywhere), always sends ONE decimal
// ("5.0", "18.5", "60.0"), sends it only on a `+` that carries a tag (the pendant's rule: a
// reading belongs to a bag and the tag is the bag), and fixes it into the request when the id
// is minted, so a resend carries the first send's value.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static const uint16_t MC_MIN_TENTHS  = 50;
static const uint16_t MC_MAX_TENTHS  = 600;
// UP and DOWN step 0.5 on the MC screen (design §4.14).
static const uint16_t MC_STEP_TENTHS = 5;
// Where the chooser starts when no mc is set for the armed tag. 20.0 % is the panel's
// Dry/Seasoned boundary (api/moisture.py DRY_BELOW_TENTHS = 200). The design does not name a
// start value; this is this change's choice.
static const uint16_t MC_START_TENTHS = 200;
// 0 means "no mc": it is below the range, so it can never be mistaken for a reading.
static const uint16_t MC_NONE = 0;

inline bool mcValid(uint16_t tenths) { return tenths >= MC_MIN_TENTHS && tenths <= MC_MAX_TENTHS; }

// "18.5": the digits as the panel parses them back to the same tenths. Empty for MC_NONE or a
// value out of range, so an invalid mc can never be formatted onto the wire.
inline void mcFormat(uint16_t tenths, char *out, size_t cap) {
  if (cap) out[0] = '\0';
  if (!mcValid(tenths) || cap < 5) return;
  snprintf(out, cap, "%u.%u", (unsigned)(tenths / 10), (unsigned)(tenths % 10));
}

// The chooser's UP and DOWN. The value below MC_MIN_TENTHS is MC_NONE ("MC --"): DOWN from 5.0
// clears it, UP from none starts at 5.0. UP stops at 60.0.
inline uint16_t mcStepUp(uint16_t t) {
  if (t == MC_NONE) return MC_MIN_TENTHS;
  const uint16_t v = (uint16_t)(t + MC_STEP_TENTHS);
  return v > MC_MAX_TENTHS ? MC_MAX_TENTHS : v;
}
inline uint16_t mcStepDown(uint16_t t) {
  if (t == MC_NONE || t <= MC_MIN_TENTHS) return MC_NONE;
  const uint16_t v = (uint16_t)(t - MC_STEP_TENTHS);
  return v < MC_MIN_TENTHS ? MC_MIN_TENTHS : v;
}

// The indicator beside the tag, the pendant's "MC --" / "MC 18.5" (mesh PlcPanel.cpp).
inline void mcIndicator(uint16_t tenths, char *out, size_t cap) {
  char t[6];
  mcFormat(tenths, t, sizeof(t));
  if (t[0]) snprintf(out, cap, "MC %s", t);
  else      snprintf(out, cap, "MC --");
}
