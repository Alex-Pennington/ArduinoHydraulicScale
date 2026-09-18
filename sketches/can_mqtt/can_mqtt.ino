// can_mqtt -- J1939 engine telemetry -> MQTT skeleton for PLC #4 (JD 5055E).
//
// STATUS: SKELETON. It compiles and the decode arithmetic is written out in
// full, but NOTHING here has seen a tractor. Every scaling constant is from
// SAE J1939-71 as reproduced by public sources, not from a capture of this
// machine. Run sketches/can_sniff first; the TODOs below name exactly which
// numbers that capture settles.
//
// This sketch stands alone so it can be flashed to a BENCH board without
// disturbing the scale firmware. The production shape is different: the CAN
// reader becomes a module inside src/ArduinoHydraulicScale.cpp and reuses that
// file's netPoll() state machine, its will, and its status topic. See REPORT.md
// "Integration shape". Do not flash this to the tractor board -- it would
// replace the scale.
//
// WIRING -- UNO R4 WiFi:
//   D10 = CANTX -> transceiver TXD
//   D13 = CANRX <- transceiver RXD
// (D4/D5 is the Minima mapping and is wrong for this board. On the tractor
// board D4/D5 are keypad buttons.)
//
// TOPICS -- matching the estate conventions already in the consumer:
//   plc4/jd5055e/engine    QoS 0, NOT retained, 15 s   -- the numbers
//   plc4/jd5055e/dtc/<sa>  QoS 1, RETAINED, on change  -- fault state, PER ECU
//
// The <sa> leaf is not decoration. Four ECUs on this tractor publish DM1 and they
// disagree; one retained topic would hold whichever spoke last.
// Telemetry is an event and must not replay stale; fault state is state and a
// late subscriber must see it. That split is how plc4/jd5055e/telemetry and
// plc4/jd5055e/status already behave.
//
// VALIDITY CONVENTION -- deliberately PLC1/PLC3's, not PLC4's. J1939 encodes
// "not available" (0xFF...) and "error" (0xFE...) in the wire value itself, so
// an absent reading is genuinely absent rather than suspect. Absent keys are
// OMITTED from the JSON. PLC4's existing telemetry does the opposite (always
// populate, validity in sibling strings) because an ADC always returns a
// number. Do not copy that here; the consumer handler must be written to the
// omission contract.

#include <Arduino_CAN.h>
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>

#include "arduino_secrets.h"   // SECRET_SSID, SECRET_PASS, SECRET_MQTT_BROKER,
                               // SECRET_MQTT_USER, SECRET_MQTT_PASS

// ---------------------------------------------------------------------------
// Configuration

static const uint32_t CAN_BITRATE      = 250000UL;  // TODO(bench): confirm with can_sniff
static const uint16_t MQTT_PORT        = 1883;
static const char    *DEVICE_ID        = "plc4-jd5055e";
static const char    *TOPIC_ENGINE     = "plc4/jd5055e/engine";
static const char    *TOPIC_DTC        = "plc4/jd5055e/dtc";

static const uint32_t PUBLISH_MS       = 15000UL;   // engine summary cadence
static const uint32_t NET_RETRY_MS     = 30000UL;   // same bound as the scale firmware
static const uint32_t SIG_STALE_MS     = 5000UL;    // a value older than this is not reported

// ---------------------------------------------------------------------------
// J1939 identifiers

static const uint32_t PGN_EEC1  = 61444;  // engine speed, actual percent torque
static const uint32_t PGN_EEC2  = 61443;  // engine percent load at current speed
static const uint32_t PGN_ET1   = 65262;  // coolant temp, oil temp
static const uint32_t PGN_EFLP1 = 65263;  // oil pressure
static const uint32_t PGN_HOURS = 65253;  // total engine hours  -- MAY BE REQUEST-ONLY
static const uint32_t PGN_LFE   = 65266;  // fuel rate
static const uint32_t PGN_VEP1  = 65271;  // battery potential
static const uint32_t PGN_DM1   = 65226;  // active DTCs
static const uint32_t PGN_TPCM  = 60416;  // transport protocol, connection mgmt (0xEC00)
static const uint32_t PGN_TPDT  = 60160;  // transport protocol, data transfer (0xEB00)
// TP.DT is 60160, NOT 60417. A public source said 60417 and it is wrong: in the Cal
// Poly 5055E corpus there are 1491 frames on 60160 carrying sequence-numbered payloads
// and exactly zero on 60417. See findings/2026-09-18-calpoly-can-corpus.md.

// ---------------------------------------------------------------------------
// A decoded signal. `ts` is when it was last seen, so staleness is a property
// of the value and not of the publish loop.

struct Sig {
  float    v;
  uint32_t ts;
  bool     ok;
};

static Sig sigRpm, sigTorquePct, sigLoadPct, sigCoolantC, sigOilTempC;
static Sig sigOilKpa, sigHours, sigFuelLph, sigBattV, sigKeyswV;

static void sigSet(Sig &s, float v, uint32_t now) { s.v = v; s.ts = now; s.ok = true; }
static bool sigFresh(const Sig &s, uint32_t now) {
  return s.ok && (now - s.ts) <= SIG_STALE_MS;
}

// J1939 "not available" / "error" sentinels. One byte: FF = N/A, FE = error.
// Two bytes: FF00..FFFF = N/A, FE00..FEFF = error. Four bytes likewise on the
// top byte. Treating FF as a real reading is how a parked tractor reports
// 8031 rpm.
static inline bool u8Bad(uint8_t v)   { return v >= 0xFE; }
static inline bool u16Bad(uint16_t v) { return (v >> 8) >= 0xFE; }
static inline bool u32Bad(uint32_t v) { return (v >> 24) >= 0xFE; }

// J1939 multi-byte values are little-endian (LSB first) on the wire.
static inline uint16_t le16(const uint8_t *d) {
  return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}
static inline uint32_t le32(const uint8_t *d) {
  return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

static inline uint32_t j1939Pgn(uint32_t id29) {
  const uint8_t  pf  = (uint8_t)((id29 >> 16) & 0xFF);
  const uint8_t  ps  = (uint8_t)((id29 >> 8) & 0xFF);
  const uint32_t edp = (id29 >> 25) & 0x1UL;
  const uint32_t dp  = (id29 >> 24) & 0x1UL;
  uint32_t pgn = (edp << 17) | (dp << 16) | ((uint32_t)pf << 8);
  if (pf >= 240) pgn |= ps;    // PDU2 broadcast; PDU1's PS is a destination address
  return pgn;
}
static inline uint8_t j1939Sa(uint32_t id29) { return (uint8_t)(id29 & 0xFF); }

// ---------------------------------------------------------------------------
// DM1 -- active diagnostic trouble codes

struct Dtc {
  uint32_t spn;
  uint8_t  fmi;
  uint8_t  oc;
};

static const size_t MAX_DTC = 12;

// DM1 IS PER SOURCE ADDRESS, NOT PER MACHINE. In the Cal Poly 5055E corpus four
// distinct ECUs publish DM1 -- source addresses 3, 6, 23 and 71 -- and they disagree:
// SA 3 raises the protect lamp while the other three report no faults. A single global
// DM1 store would flap between them at 1 Hz and report whichever ECU spoke last.
static const size_t MAX_DM1_SRC = 6;

struct Dm1Src {
  uint8_t  sa;
  bool     used;
  uint8_t  lampMil, lampRed, lampAmber, lampProtect;
  Dtc      dtc[MAX_DTC];
  uint8_t  count;
  uint32_t fingerprint;
  uint32_t lastFingerprint;
  bool     dirty;
};

static Dm1Src dm1[MAX_DM1_SRC];

// Returns an index, not a pointer, for the same .ino prototype-hoisting reason as
// publishDtc(): a Dm1Src in a function signature is hoisted above the struct.
// -1 means more than MAX_DM1_SRC ECUs publish DM1 and this one is dropped rather
// than blended into somebody else's fault state.
static int dm1Slot(uint8_t sa) {
  for (size_t i = 0; i < MAX_DM1_SRC; i++)
    if (dm1[i].used && dm1[i].sa == sa) return (int)i;
  for (size_t i = 0; i < MAX_DM1_SRC; i++)
    if (!dm1[i].used) {
      dm1[i].used = true;
      dm1[i].sa = sa;
      dm1[i].lastFingerprint = 0xFFFFFFFFUL;
      return (int)i;
    }
  return -1;
}

// BAM reassembly. Single session only: a second BAM opening while one is in
// flight abandons the first. Real J1939 allows one BAM per source address at a
// time, so this is correct for one ECU and lossy for several.
// TODO(bench): if the capture shows two ECUs sending BAM concurrently, key this
// by source address.
static const size_t BAM_MAX = 128;
static uint8_t  bamBuf[BAM_MAX];
static uint32_t bamPgn      = 0;
static uint16_t bamTotal    = 0;
static uint8_t  bamPackets  = 0;
static uint8_t  bamNextSeq  = 0;
static uint8_t  bamSa       = 0;
static bool     bamActive   = false;
static uint32_t bamStartMs  = 0;
static const uint32_t BAM_TIMEOUT_MS = 1500;   // J1939-21 allows 750 ms between TP.DT

// Parse a DM1 payload (lamp bytes + N 4-byte DTC records) from a buffer of any
// length -- single frame (8 bytes) or reassembled BAM -- into that source's slot.
static void dm1Parse(const uint8_t *d, size_t len, uint8_t sa, uint32_t now) {
  if (len < 2) return;
  const int slot = dm1Slot(sa);
  if (slot < 0) return;
  Dm1Src *s = &dm1[slot];

  s->lampMil     = (d[0] >> 6) & 0x3;
  s->lampRed     = (d[0] >> 4) & 0x3;
  s->lampAmber   = (d[0] >> 2) & 0x3;
  s->lampProtect =  d[0]       & 0x3;

  s->count = 0;
  uint32_t fp = 0;
  for (size_t off = 2; off + 4 <= len && s->count < MAX_DTC; off += 4) {
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
  (void)now;
}

static void bamHandleCm(const uint8_t *d, uint8_t sa, uint32_t now) {
  if (d[0] != 0x20) return;                  // 0x20 = BAM; RTS/CTS sessions are point-to-point
  bamTotal   = le16(&d[1]);
  bamPackets = d[3];
  bamPgn     = (uint32_t)d[5] | ((uint32_t)d[6] << 8) | ((uint32_t)d[7] << 16);
  bamSa      = sa;
  bamNextSeq = 1;
  bamActive  = (bamTotal > 0 && bamTotal <= BAM_MAX);
  bamStartMs = now;
  if (bamActive) memset(bamBuf, 0, sizeof(bamBuf));
}

static void bamHandleDt(const uint8_t *d, uint8_t sa, uint32_t now) {
  if (!bamActive || sa != bamSa) return;
  if (d[0] != bamNextSeq) { bamActive = false; return; }   // a gap loses the session

  const size_t off = (size_t)(d[0] - 1) * 7u;
  for (size_t i = 0; i < 7; i++) {
    const size_t dst = off + i;
    if (dst < bamTotal && dst < BAM_MAX) bamBuf[dst] = d[1 + i];
  }
  bamNextSeq++;
  bamStartMs = now;

  if (bamNextSeq > bamPackets) {
    bamActive = false;
    if (bamPgn == PGN_DM1) dm1Parse(bamBuf, bamTotal, bamSa, now);
    // In the Cal Poly 5055E corpus NO multipacket session ever announces DM1 --
    // all 358 transport sessions carry PGN 65251 (Engine Configuration). So this
    // path is unexercised on that machine. It stays because a DM1 with two or more
    // active faults is required by J1939-73 to arrive this way, and the day it
    // does is the day we most want it.
  }
}

// ---------------------------------------------------------------------------
// Frame dispatch

static uint32_t framesSeen = 0;
static uint32_t framesUnknown = 0;

static void onFrame(const CanMsg &msg, uint32_t now) {
  framesSeen++;
  if (!msg.isExtendedId()) return;           // 11-bit frames are not J1939

  const uint32_t id  = msg.getExtendedId();
  const uint32_t pgn = j1939Pgn(id);
  const uint8_t  sa  = j1939Sa(id);
  const uint8_t *d   = msg.data;

  switch (pgn) {
    case PGN_EEC1:
      if (msg.data_length >= 5) {
        const uint16_t raw = le16(&d[3]);                    // bytes 4-5
        if (!u16Bad(raw)) sigSet(sigRpm, raw * 0.125f, now); // 0.125 rpm/bit, offset 0
      }
      if (msg.data_length >= 3 && !u8Bad(d[2])) {
        sigSet(sigTorquePct, (float)d[2] - 125.0f, now);     // SPN 513, 1 %/bit, -125
      }
      break;

    case PGN_EEC2:
      // SPN 92 Engine Percent Load At Current Speed lives HERE, in EEC2 byte 3 --
      // not in EEC1. EEC1 byte 3 is actual percent TORQUE, a different number
      // with a different offset.
      if (msg.data_length >= 3 && !u8Bad(d[2])) {
        sigSet(sigLoadPct, (float)d[2], now);                // 1 %/bit, offset 0
      }
      break;

    case PGN_ET1:
      if (msg.data_length >= 1 && !u8Bad(d[0])) {
        sigSet(sigCoolantC, (float)d[0] - 40.0f, now);       // SPN 110, 1 degC/bit, -40
      }
      if (msg.data_length >= 4) {
        const uint16_t raw = le16(&d[2]);                    // bytes 3-4
        if (!u16Bad(raw)) sigSet(sigOilTempC, raw * 0.03125f - 273.0f, now);
      }
      break;

    case PGN_EFLP1:
      if (msg.data_length >= 4 && !u8Bad(d[3])) {
        sigSet(sigOilKpa, (float)d[3] * 4.0f, now);          // SPN 100, 4 kPa/bit, offset 0
      }
      break;

    case PGN_HOURS: {
      // J1939-71 lists this PGN as on-request for many engines, which would have
      // meant TRANSMITTING a request onto the tractor bus to read hours. It does
      // not apply here: the 5055E broadcasts it unprompted. It appears in 39 of
      // the 44 Cal Poly captures, and it advances across them (138.15 h, then
      // 147.55 h in the later session), which also confirms the 0.05 h/bit
      // scaling. Nothing needs to be transmitted.
      if (msg.data_length >= 4) {
        const uint32_t raw = le32(&d[0]);                    // bytes 1-4
        if (!u32Bad(raw)) sigSet(sigHours, raw * 0.05f, now); // SPN 247, 0.05 h/bit
      }
      break;
    }

    case PGN_LFE:
      if (msg.data_length >= 2) {
        const uint16_t raw = le16(&d[0]);                    // bytes 1-2
        if (!u16Bad(raw)) sigSet(sigFuelLph, raw * 0.05f, now); // SPN 183, 0.05 L/h per bit
      }
      break;

    case PGN_VEP1:
      // ON THIS TRACTOR THE USEFUL VOLTAGE IS SPN 158, NOT SPN 168. Across 1206
      // VEP1 frames in the Cal Poly 5055E corpus, SPN 168 (bytes 5-6, "battery
      // potential / power input 1") was not-available in ALL 1206, and SPN 158
      // (bytes 7-8, keyswitch battery potential) was available in all 1206 --
      // reading 12.0-13.5 V, and dipping to 9.35 V under a deliberate high-beam
      // load. Decode both anyway: 168 is the standard's primary and costs two
      // lines, and a different ECU may populate it.
      if (msg.data_length >= 6) {
        const uint16_t raw = le16(&d[4]);                    // bytes 5-6, SPN 168
        if (!u16Bad(raw)) sigSet(sigBattV, raw * 0.05f, now);
      }
      if (msg.data_length >= 8) {
        const uint16_t raw = le16(&d[6]);                    // bytes 7-8, SPN 158
        if (!u16Bad(raw)) sigSet(sigKeyswV, raw * 0.05f, now);
      }
      break;

    case PGN_DM1:
      dm1Parse(d, msg.data_length, sa, now);
      break;

    case PGN_TPCM:
      if (msg.data_length >= 8) bamHandleCm(d, sa, now);
      break;

    case PGN_TPDT:
      if (msg.data_length >= 8) bamHandleDt(d, sa, now);
      break;

    default:
      framesUnknown++;
      break;
  }
}

// ---------------------------------------------------------------------------
// Network -- the two-stage, never-blocking-the-main-job shape from the scale
// firmware. WiFi.begin() and MqttClient::connect() ARE blocking calls; what is
// guaranteed is that they are attempted at a bounded rate and never in the same
// loop pass.

enum NetState { NET_OFFLINE, NET_WIFI_UP, NET_ONLINE };
static NetState  netState = NET_OFFLINE;
static uint32_t  lastNetAttemptMs = 0;

static WiFiClient wifiClient;
static MqttClient mqttClient(wifiClient);

static void netPoll(uint32_t now) {
  if (netState == NET_ONLINE) {
    if (!mqttClient.connected()) { netState = NET_OFFLINE; }
    else { mqttClient.poll(); return; }
  }
  if (now - lastNetAttemptMs < NET_RETRY_MS) return;
  lastNetAttemptMs = now;

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    netState = (WiFi.status() == WL_CONNECTED) ? NET_WIFI_UP : NET_OFFLINE;
    return;                                  // never connect the broker in the same pass
  }

  mqttClient.setId(DEVICE_ID);
  mqttClient.setUsernamePassword(SECRET_MQTT_USER, SECRET_MQTT_PASS);
  // TODO: in the merged firmware the will belongs to the scale's status topic,
  // which already carries one. A second will on a second client id would make
  // two "up" flags that can disagree.
  if (mqttClient.connect(SECRET_MQTT_BROKER, MQTT_PORT)) {
    netState = NET_ONLINE;
    // Force a republish of every source's retained fault state. A retained
    // message the broker dropped while we were away is one nobody will resend.
    for (size_t i = 0; i < MAX_DM1_SRC; i++)
      if (dm1[i].used) dm1[i].dirty = true;
  } else {
    netState = NET_WIFI_UP;
  }
}

static bool publishTo(const char *topic, const char *payload, bool retained, int qos) {
  if (netState != NET_ONLINE) return false;
  if (!mqttClient.beginMessage(topic, retained, qos)) return false;
  mqttClient.print(payload);
  return mqttClient.endMessage() == 1;
}

// ---------------------------------------------------------------------------
// Publish

// Append "key":value only when the signal is fresh. An omitted key means "this
// engine did not tell us", which is the contract the consumer handler reads.
static size_t appendSig(char *buf, size_t cap, size_t n, const char *key,
                        const Sig &s, int decimals, uint32_t now) {
  if (!sigFresh(s, now) || n >= cap) return n;
  char val[16];
  dtostrf(s.v, 0, decimals, val);
  return n + snprintf(buf + n, cap - n, "\"%s\":%s,", key, val);
}

static void publishEngine(uint32_t now) {
  char p[448];                 // generous: the status payload was truncated at 200 once
  size_t n = 0;
  n += snprintf(p + n, sizeof(p) - n, "{");
  n = appendSig(p, sizeof(p), n, "rpm",        sigRpm,        0, now);
  n = appendSig(p, sizeof(p), n, "load_pct",   sigLoadPct,    0, now);
  n = appendSig(p, sizeof(p), n, "torque_pct", sigTorquePct,  0, now);
  n = appendSig(p, sizeof(p), n, "coolant_c",  sigCoolantC,   1, now);
  n = appendSig(p, sizeof(p), n, "oil_temp_c", sigOilTempC,   1, now);
  n = appendSig(p, sizeof(p), n, "oil_kpa",    sigOilKpa,     0, now);
  n = appendSig(p, sizeof(p), n, "fuel_lph",   sigFuelLph,    2, now);
  n = appendSig(p, sizeof(p), n, "batt_v",     sigBattV,      2, now);
  n = appendSig(p, sizeof(p), n, "keysw_v",    sigKeyswV,     2, now);
  n = appendSig(p, sizeof(p), n, "hours",      sigHours,      2, now);
  n += snprintf(p + n, sizeof(p) - n,
                "\"frames\":%lu,\"unknown\":%lu,\"id\":\"%s\"}",
                (unsigned long)framesSeen, (unsigned long)framesUnknown, DEVICE_ID);
  publishTo(TOPIC_ENGINE, p, false, 0);
}

// One message PER SOURCE ADDRESS, on its own retained topic. Retaining a single
// combined payload would be wrong: the retained message is the current state, and
// four ECUs have four current states. The topic carries the SA so each ECU's fault
// state is retained and cleared independently.
// Takes an index, not a Dm1Src*, on purpose: the .ino preprocessor hoists function
// prototypes above the file's own type declarations, so a struct pointer in the
// signature fails to compile with a bewildering "'s' was not declared in this scope".
static void publishDtc(size_t idx) {
  Dm1Src *s = &dm1[idx];
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%u", TOPIC_DTC, (unsigned)s->sa);

  char p[448];
  size_t n = 0;
  n += snprintf(p + n, sizeof(p) - n,
                "{\"sa\":%u,\"n\":%u,\"mil\":%u,\"rsl\":%u,\"awl\":%u,\"pl\":%u,\"dtc\":[",
                (unsigned)s->sa, (unsigned)s->count, (unsigned)s->lampMil,
                (unsigned)s->lampRed, (unsigned)s->lampAmber, (unsigned)s->lampProtect);
  for (uint8_t i = 0; i < s->count && n < sizeof(p) - 48; i++) {
    n += snprintf(p + n, sizeof(p) - n, "%s{\"spn\":%lu,\"fmi\":%u,\"oc\":%u}",
                  i ? "," : "", (unsigned long)s->dtc[i].spn,
                  (unsigned)s->dtc[i].fmi, (unsigned)s->dtc[i].oc);
  }
  n += snprintf(p + n, sizeof(p) - n, "],\"id\":\"%s\"}", DEVICE_ID);

  if (publishTo(topic, p, true, 1)) {      // retained, QoS 1 -- fault state, not an event
    s->lastFingerprint = s->fingerprint;
    s->dirty = false;
  }
}

// ---------------------------------------------------------------------------

static uint32_t lastPublishMs = 0;
static bool     canUp = false;

void setup() {
  Serial.begin(115200);
  delay(200);

  canUp = CAN.begin(CAN_BITRATE);
  Serial.println(canUp ? F("CAN up (D10 TX / D13 RX)") : F("CAN.begin FAILED"));

  lastPublishMs    = millis();
  lastNetAttemptMs = millis() - NET_RETRY_MS;   // attempt the network immediately
}

void loop() {
  const uint32_t now = millis();

  // Drain first and drain fully. The 32-slot RX ring drops the NEWEST frame
  // when full and the hardware mailboxes behind it overwrite the OLDEST, and
  // neither keeps a counter -- so `frames` below is what we received, never what
  // the bus carried. For periodic PGNs a dropped frame is replaced within
  // milliseconds; for a BAM session it destroys the reassembly, which is why
  // bamHandleDt() abandons a session on a sequence gap instead of guessing.
  if (canUp) {
    while (CAN.available()) onFrame(CAN.read(), now);

    if (bamActive && (now - bamStartMs) > BAM_TIMEOUT_MS) bamActive = false;
  }

  netPoll(now);

  // Faults go out immediately, not on the 15 s tick, and one ECU at a time.
  for (size_t i = 0; i < MAX_DM1_SRC; i++)
    if (dm1[i].used && dm1[i].dirty) publishDtc(i);

  if (now - lastPublishMs >= PUBLISH_MS) {
    lastPublishMs = now;
    publishEngine(now);
  }
}
