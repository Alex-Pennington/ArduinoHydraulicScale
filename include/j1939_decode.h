#pragma once
// J1939 decode for the JD 5055E -- PURE, no Arduino dependency.
//
// Lifted from sketches/can_mqtt/can_mqtt.ino (issue #24) so the same code runs
// in the firmware AND on the host against real captures. test/test_j1939_replay.cpp
// feeds it the 2026-09-22 candump census off this tractor; that replay is the
// only evidence the scalings below produce this machine's numbers, and it is
// evidence from a capture, not from the board.
//
// The frame source is the caller's business. In the firmware it is the CAN
// drain in include/j1939_can.h, running in a timer ISR -- so everything here must
// stay short, allocation-free and non-blocking.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

static const uint32_t J1939_SIG_STALE_MS = 5000UL;   // a value older than this is not reported

static const uint32_t PGN_EEC1  = 61444;  // engine speed, actual percent torque
static const uint32_t PGN_EEC2  = 61443;  // engine percent load at current speed
static const uint32_t PGN_ET1   = 65262;  // coolant temp, oil temp
static const uint32_t PGN_EFLP1 = 65263;  // oil pressure
static const uint32_t PGN_HOURS = 65253;  // total engine hours -- broadcast unprompted on the 5055E
static const uint32_t PGN_LFE   = 65266;  // fuel rate
static const uint32_t PGN_VEP1  = 65271;  // battery potential
static const uint32_t PGN_DM1   = 65226;  // active DTCs
static const uint32_t PGN_TPCM  = 60416;  // transport protocol, connection mgmt (0xEC00)
static const uint32_t PGN_TPDT  = 60160;  // transport protocol, data transfer (0xEB00)
// TP.DT is 60160, NOT 60417. A public source said 60417 and it is wrong: in the Cal
// Poly 5055E corpus there are 1491 frames on 60160 carrying sequence-numbered payloads
// and exactly zero on 60417. See findings/2026-09-18-calpoly-can-corpus.md.

// A decoded signal. `ts` is when it was last seen, so staleness is a property
// of the value and not of the publish loop.
struct J1939Sig {
  float    v;
  uint32_t ts;
  bool     ok;
};

inline void j1939SigSet(J1939Sig &s, float v, uint32_t now) { s.v = v; s.ts = now; s.ok = true; }
inline bool j1939SigFresh(const J1939Sig &s, uint32_t now) {
  return s.ok && (uint32_t)(now - s.ts) <= J1939_SIG_STALE_MS;
}

// J1939 "not available" / "error" sentinels. One byte: FF = N/A, FE = error.
// Two bytes: FF00..FFFF = N/A, FE00..FEFF = error. Four bytes likewise on the
// top byte. Treating FF as a real reading is how a parked tractor reports
// 8031 rpm.
inline bool j1939U8Bad(uint8_t v)   { return v >= 0xFE; }
inline bool j1939U16Bad(uint16_t v) { return (v >> 8) >= 0xFE; }
inline bool j1939U32Bad(uint32_t v) { return (v >> 24) >= 0xFE; }

// J1939 multi-byte values are little-endian (LSB first) on the wire.
inline uint16_t j1939Le16(const uint8_t *d) {
  return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}
inline uint32_t j1939Le32(const uint8_t *d) {
  return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

inline uint32_t j1939Pgn(uint32_t id29) {
  const uint8_t  pf  = (uint8_t)((id29 >> 16) & 0xFF);
  const uint8_t  ps  = (uint8_t)((id29 >> 8) & 0xFF);
  const uint32_t edp = (id29 >> 25) & 0x1UL;
  const uint32_t dp  = (id29 >> 24) & 0x1UL;
  uint32_t pgn = (edp << 17) | (dp << 16) | ((uint32_t)pf << 8);
  if (pf >= 240) pgn |= ps;    // PDU2 broadcast; PDU1's PS is a destination address
  return pgn;
}
inline uint8_t j1939Sa(uint32_t id29) { return (uint8_t)(id29 & 0xFF); }

// ---------------------------------------------------------------------------
// DM1 -- active diagnostic trouble codes
//
// DM1 IS PER SOURCE ADDRESS, NOT PER MACHINE. In the Cal Poly 5055E corpus four
// distinct ECUs publish DM1 -- source addresses 3, 6, 23 and 71 -- and they disagree:
// SA 3 raises the protect lamp while the other three report no faults. A single global
// DM1 store would flap between them at 1 Hz and report whichever ECU spoke last.
static const size_t J1939_MAX_DTC     = 12;
static const size_t J1939_MAX_DM1_SRC = 6;

struct J1939Dtc {
  uint32_t spn;
  uint8_t  fmi;
  uint8_t  oc;
};

struct J1939Dm1Src {
  uint8_t  sa;
  bool     used;
  uint8_t  lampMil, lampRed, lampAmber, lampProtect;
  J1939Dtc dtc[J1939_MAX_DTC];
  uint8_t  count;
  uint32_t fingerprint;
  uint32_t lastFingerprint;   // the fingerprint last PUBLISHED; owned by the publisher
  bool     dirty;
};

// BAM reassembly. Single session only: a second BAM opening while one is in
// flight abandons the first. Real J1939 allows one BAM per source address at a
// time, so this is correct for one ECU and lossy for several.
static const size_t   J1939_BAM_MAX        = 128;
static const uint32_t J1939_BAM_TIMEOUT_MS = 1500;   // J1939-21 allows 750 ms between TP.DT

struct J1939State {
  J1939Sig rpm, torquePct, loadPct, coolantC, oilTempC;
  J1939Sig oilKpa, hours, fuelLph, battV, keyswV;

  J1939Dm1Src dm1[J1939_MAX_DM1_SRC];

  uint8_t  bamBuf[J1939_BAM_MAX];
  uint32_t bamPgn;
  uint16_t bamTotal;
  uint8_t  bamPackets;
  uint8_t  bamNextSeq;
  uint8_t  bamSa;
  bool     bamActive;
  uint32_t bamStartMs;

  uint32_t framesSeen;      // every frame taken off the ring, decoded or not
  uint32_t framesUnknown;   // extended frames on a PGN this decoder does not use
};

inline void j1939Init(J1939State &st) { memset(&st, 0, sizeof(st)); }

// Returns an index, not a pointer, so a caller in an .ino never needs the struct in a
// signature (the .ino preprocessor hoists prototypes above type declarations).
// -1 means more than J1939_MAX_DM1_SRC ECUs publish DM1 and this one is dropped rather
// than blended into somebody else's fault state.
inline int j1939Dm1Slot(J1939State &st, uint8_t sa) {
  for (size_t i = 0; i < J1939_MAX_DM1_SRC; i++)
    if (st.dm1[i].used && st.dm1[i].sa == sa) return (int)i;
  for (size_t i = 0; i < J1939_MAX_DM1_SRC; i++)
    if (!st.dm1[i].used) {
      st.dm1[i].used = true;
      st.dm1[i].sa = sa;
      st.dm1[i].lastFingerprint = 0xFFFFFFFFUL;
      return (int)i;
    }
  return -1;
}

// Parse a DM1 payload (lamp bytes + N 4-byte DTC records) from a buffer of any
// length -- single frame (8 bytes) or reassembled BAM -- into that source's slot.
inline void j1939Dm1Parse(J1939State &st, const uint8_t *d, size_t len, uint8_t sa) {
  if (len < 2) return;
  const int slot = j1939Dm1Slot(st, sa);
  if (slot < 0) return;
  J1939Dm1Src *s = &st.dm1[slot];

  s->lampMil     = (d[0] >> 6) & 0x3;
  s->lampRed     = (d[0] >> 4) & 0x3;
  s->lampAmber   = (d[0] >> 2) & 0x3;
  s->lampProtect =  d[0]       & 0x3;

  s->count = 0;
  uint32_t fp = 0;
  for (size_t off = 2; off + 4 <= len && s->count < J1939_MAX_DTC; off += 4) {
    const uint8_t b0 = d[off], b1 = d[off + 1], b2 = d[off + 2], b3 = d[off + 3];

    const uint32_t spn = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)(b2 >> 5) << 16);
    const uint8_t  fmi = b2 & 0x1F;
    const uint8_t  oc  = b3 & 0x7F;

    // A no-fault DM1 carries a zero record; unused slots are 0xFF padding.
    if (b0 == 0x00 && b1 == 0x00 && fmi == 0x00) continue;
    if (b0 == 0xFF && b1 == 0xFF && b2 == 0xFF) continue;

    // The not-available record, which is NOT 0xFF padding and bit this decoder
    // once. The 5055E raises the protect lamp and sends `F4 FE FF 00`, which
    // reassembles to SPN 524020 / FMI 31 / OC 0 -- an SPN in the reserved region
    // with the not-available FMI and no occurrences. That is the ECU saying "a
    // lamp is on and I am not telling you which code", not a fault numbered
    // 524020. Emitting it would put a permanent phantom DTC on the dashboard.
    if (spn >= 0x7F000UL) continue;
    if (fmi == 31 && oc == 0) continue;

    s->dtc[s->count].spn = spn;
    s->dtc[s->count].fmi = fmi;
    s->dtc[s->count].oc  = oc;
    s->count++;

    fp = fp * 31u + spn;          // order-sensitive, which is fine: the ECU's
    fp = fp * 31u + fmi;          // order is stable between repeats
  }

  fp = fp * 31u + ((uint32_t)s->lampMil << 6 | (uint32_t)s->lampRed << 4 |
                   (uint32_t)s->lampAmber << 2 | s->lampProtect);
  fp = fp * 31u + s->count;

  s->fingerprint = fp;
  if (fp != s->lastFingerprint) s->dirty = true;
}

inline void j1939BamCm(J1939State &st, const uint8_t *d, uint8_t sa, uint32_t now) {
  if (d[0] != 0x20) return;                  // 0x20 = BAM; RTS/CTS sessions are point-to-point
  st.bamTotal   = j1939Le16(&d[1]);
  st.bamPackets = d[3];
  st.bamPgn     = (uint32_t)d[5] | ((uint32_t)d[6] << 8) | ((uint32_t)d[7] << 16);
  st.bamSa      = sa;
  st.bamNextSeq = 1;
  st.bamActive  = (st.bamTotal > 0 && st.bamTotal <= J1939_BAM_MAX);
  st.bamStartMs = now;
  if (st.bamActive) memset(st.bamBuf, 0, sizeof(st.bamBuf));
}

inline void j1939BamDt(J1939State &st, const uint8_t *d, uint8_t sa, uint32_t now) {
  if (!st.bamActive || sa != st.bamSa) return;
  if (d[0] != st.bamNextSeq) { st.bamActive = false; return; }   // a gap loses the session

  const size_t off = (size_t)(d[0] - 1) * 7u;
  for (size_t i = 0; i < 7; i++) {
    const size_t dst = off + i;
    if (dst < st.bamTotal && dst < J1939_BAM_MAX) st.bamBuf[dst] = d[1 + i];
  }
  st.bamNextSeq++;
  st.bamStartMs = now;

  if (st.bamNextSeq > st.bamPackets) {
    st.bamActive = false;
    // In the Cal Poly 5055E corpus NO multipacket session ever announces DM1 --
    // all 358 transport sessions carry PGN 65251 (Engine Configuration). So this
    // path is unexercised on that machine. It stays because a DM1 with two or more
    // active faults is required by J1939-73 to arrive this way, and the day it
    // does is the day we most want it.
    if (st.bamPgn == PGN_DM1) j1939Dm1Parse(st, st.bamBuf, st.bamTotal, st.bamSa);
  }
}

// Abandon a BAM session whose next packet never came.
inline void j1939Tick(J1939State &st, uint32_t now) {
  if (st.bamActive && (uint32_t)(now - st.bamStartMs) > J1939_BAM_TIMEOUT_MS) st.bamActive = false;
}

// One received frame. `len` is the DLC, clamped by the caller to <= 8.
inline void j1939OnFrame(J1939State &st, bool extended, uint32_t id, const uint8_t *d,
                         uint8_t len, uint32_t now) {
  st.framesSeen++;
  if (!extended) return;                     // 11-bit frames are not J1939

  const uint32_t pgn = j1939Pgn(id);
  const uint8_t  sa  = j1939Sa(id);

  switch (pgn) {
    case PGN_EEC1:
      if (len >= 5) {
        const uint16_t raw = j1939Le16(&d[3]);                    // bytes 4-5
        if (!j1939U16Bad(raw)) j1939SigSet(st.rpm, raw * 0.125f, now); // 0.125 rpm/bit
      }
      if (len >= 3 && !j1939U8Bad(d[2])) {
        j1939SigSet(st.torquePct, (float)d[2] - 125.0f, now);    // SPN 513, 1 %/bit, -125
      }
      break;

    case PGN_EEC2:
      // SPN 92 Engine Percent Load At Current Speed lives HERE, in EEC2 byte 3 --
      // not in EEC1. EEC1 byte 3 is actual percent TORQUE, a different number
      // with a different offset.
      if (len >= 3 && !j1939U8Bad(d[2])) j1939SigSet(st.loadPct, (float)d[2], now);
      break;

    case PGN_ET1:
      if (len >= 1 && !j1939U8Bad(d[0])) j1939SigSet(st.coolantC, (float)d[0] - 40.0f, now);
      if (len >= 4) {
        const uint16_t raw = j1939Le16(&d[2]);                    // bytes 3-4
        if (!j1939U16Bad(raw)) j1939SigSet(st.oilTempC, raw * 0.03125f - 273.0f, now);
      }
      break;

    case PGN_EFLP1:
      if (len >= 4 && !j1939U8Bad(d[3])) j1939SigSet(st.oilKpa, (float)d[3] * 4.0f, now);
      break;

    case PGN_HOURS:
      // J1939-71 lists this PGN as on-request for many engines. The 5055E
      // broadcasts it unprompted (39 of 44 Cal Poly captures, advancing across
      // sessions), so nothing is ever transmitted to read it.
      if (len >= 4) {
        const uint32_t raw = j1939Le32(&d[0]);                    // bytes 1-4
        if (!j1939U32Bad(raw)) j1939SigSet(st.hours, raw * 0.05f, now); // SPN 247, 0.05 h/bit
      }
      break;

    case PGN_LFE:
      if (len >= 2) {
        const uint16_t raw = j1939Le16(&d[0]);                    // bytes 1-2
        if (!j1939U16Bad(raw)) j1939SigSet(st.fuelLph, raw * 0.05f, now); // SPN 183
      }
      break;

    case PGN_VEP1:
      // ON THIS TRACTOR THE USEFUL VOLTAGE IS SPN 158, NOT SPN 168. Across 1206
      // VEP1 frames in the Cal Poly 5055E corpus, SPN 168 (bytes 5-6) was
      // not-available in ALL 1206 and SPN 158 (bytes 7-8, keyswitch battery
      // potential) was available in all 1206. Decode both anyway.
      if (len >= 6) {
        const uint16_t raw = j1939Le16(&d[4]);                    // bytes 5-6, SPN 168
        if (!j1939U16Bad(raw)) j1939SigSet(st.battV, raw * 0.05f, now);
      }
      if (len >= 8) {
        const uint16_t raw = j1939Le16(&d[6]);                    // bytes 7-8, SPN 158
        if (!j1939U16Bad(raw)) j1939SigSet(st.keyswV, raw * 0.05f, now);
      }
      break;

    case PGN_DM1:
      j1939Dm1Parse(st, d, len, sa);
      break;

    // BAM is addressed to global (PS = 0xFF). A point-to-point RTS/CTS session is
    // not ours to read, and its TP.DT from the SAME source would otherwise be taken
    // for the next packet of an open BAM and corrupt it. The 2026-09-22 census has
    // exactly that: SA 0x00 running RTS/CTS to SA 0x06 (PGN 0xFEE3) during start-up
    // while also broadcasting BAM.
    case PGN_TPCM:
      if (((id >> 8) & 0xFF) != 0xFF) { st.framesUnknown++; break; }
      if (len >= 8) j1939BamCm(st, d, sa, now);
      break;

    case PGN_TPDT:
      if (((id >> 8) & 0xFF) != 0xFF) { st.framesUnknown++; break; }
      if (len >= 8) j1939BamDt(st, d, sa, now);
      break;

    default:
      st.framesUnknown++;
      break;
  }
}

// ---------------------------------------------------------------------------
// Hardware acceptance filter for the RA4M1's 8 extended receive mailboxes.
//
// All 8 share ONE mask (Arduino_CAN's setFilterMask_Extended writes both mailbox
// groups), and a mask bit of 1 means "compare". Ten PGNs are wanted and only 8
// slots exist, so the mask ignores the low 3 bits of PS as well as priority and
// source address: EEC1/EEC2, ET1/EFLP1 and LFE/VEP1 then share a slot each.
//   ignored: bits 0-7 (SA), bits 8-10 (PS low 3), bits 26-28 (priority)
//   compared: bits 11-25 (PS high 5, PF, DP, EDP)
// What leaks through is the other PGNs in each block of 8 -- ETC1, CCVS and the
// like -- which the decoder counts as `unknown` and drops. What is kept OUT is
// the proprietary traffic (PropA 0xEF00, PropB 0xFFxx), ~60% of this bus in the
// 2026-09-22 census. TP.CM/TP.DT are kept only for destination 0xF8-0xFF, which
// covers BAM (destination 0xFF).
//
// test/test_j1939_replay.cpp proves against the census that every frame the
// decoder uses passes this filter. That the SILICON applies it the same way is
// NOT proven: it has never run on a board. The firmware therefore boots with the
// filter OFF and the `canf` console verb turns it on, so it can be tried at the
// machine with the `unknown` and `frames` counts as the evidence.
static const uint32_t J1939_FILTER_MASK = 0x03FFF800UL;
static const uint32_t J1939_FILTER_IDS[8] = {
  0xF000UL << 8,   // EEC2 0xF003, EEC1 0xF004
  0xFEE0UL << 8,   // HOURS 0xFEE5
  0xFEE8UL << 8,   // ET1 0xFEEE, EFLP1 0xFEEF
  0xFEF0UL << 8,   // LFE 0xFEF2, VEP1 0xFEF7
  0xFEC8UL << 8,   // DM1 0xFECA
  0xECF8UL << 8,   // TP.CM to 0xF8-0xFF (BAM)
  0xEBF8UL << 8,   // TP.DT to 0xF8-0xFF (BAM)
  0xF000UL << 8,   // spare slot: a duplicate is harmless and never matches anything new
};

inline bool j1939FilterPasses(uint32_t id29) {
  for (size_t i = 0; i < 8; i++)
    if (((id29 ^ J1939_FILTER_IDS[i]) & J1939_FILTER_MASK) == 0) return true;
  return false;
}
