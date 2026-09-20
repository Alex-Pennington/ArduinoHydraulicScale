// can_sniff -- J1939 bus discovery for PLC #4 (JD 5055E).
//
// WHAT THIS IS: a listener. It prints every CAN frame it sees and keeps a census
// of which PGNs appear, how often, and from which source address. It exists to
// answer the three questions no datasheet can answer for this tractor:
//
//   1. Is the diagnostic connector's bus running at 250 kbit/s?
//   2. Does the engine ECU broadcast standard J1939, and which PGNs?
//   3. What is the real frame rate, so we know whether the R4 can keep up?
//
// WHAT THIS IS NOT: it never calls CAN.write(). It publishes nothing and touches
// no network. It adds no traffic of its own -- but see the ACK note below, which
// is a hardware fact, not a software one.
//
// BOARD WIRING -- UNO R4 WiFi, NOT the Minima:
//   D10 = CANTX -> transceiver TXD
//   D13 = CANRX <- transceiver RXD
// The widely-quoted "D4/D5" CAN pins are the UNO R4 *Minima* mapping. On the
// WiFi the peripheral is pinned to D10/D13; see variants/UNOWIFIR4/pins_arduino.h
// in the renesas_uno core (PIN_CAN0_TX 10, PIN_CAN0_RX 13). D4 and D5 on the
// tractor board are the Right and Up keypad buttons and must not be reused.
//
// ACK: this node acknowledges frames it receives. That is one dominant bit per
// frame on a bus that already has several ECUs acking, so it is normal CAN
// participation, not "writing to the tractor". True listen-only (no ACK) exists
// in the RA4M1 silicon as CAN_TEST_MODE_LISTEN but Arduino_CAN does not expose
// it -- see REPORT.md. Do not connect this to the machine until that decision
// is made deliberately.

#include <Arduino_CAN.h>

static const uint32_t CAN_BITRATE     = 250000UL;  // J1939 default; VERIFY on the bench
static const uint32_t REPORT_MS       = 5000UL;    // census print cadence
static const size_t   MAX_PGN_ENTRIES = 64;        // census width; overflow is counted, not hidden

struct PgnEntry {
  uint32_t pgn;
  uint32_t count;
  uint32_t lastMs;
  uint32_t minGapMs;
  uint32_t maxGapMs;
  uint8_t  sa;        // last source address seen for this PGN
  uint8_t  saFirst;
  uint8_t  saCount;   // 2 means "more than one ECU sends this PGN"
};

static PgnEntry census[MAX_PGN_ENTRIES];
static size_t   censusUsed     = 0;
static uint32_t censusOverflow = 0;   // frames whose PGN did not fit the table

static uint32_t frameCount   = 0;
static uint32_t stdFrames    = 0;     // 11-bit frames: NOT J1939, worth knowing about
static uint32_t lastReportMs = 0;
static uint32_t startMs      = 0;
static bool     canUp        = false;

// Decode a 29-bit J1939 identifier. PDU1 (PF < 240) is destination-specific and
// the PS byte is the destination address, so it is NOT part of the PGN. PDU2
// (PF >= 240) is broadcast and PS is the group extension. Getting this wrong is
// the classic J1939 decode bug: every DM1 and every transport-protocol frame
// lands in the wrong bucket.
static inline uint32_t j1939Pgn(uint32_t id29) {
  const uint8_t  pf  = (uint8_t)((id29 >> 16) & 0xFF);
  const uint8_t  ps  = (uint8_t)((id29 >> 8) & 0xFF);
  const uint32_t edp = (id29 >> 25) & 0x1UL;
  const uint32_t dp  = (id29 >> 24) & 0x1UL;
  uint32_t pgn = (edp << 17) | (dp << 16) | ((uint32_t)pf << 8);
  if (pf >= 240) pgn |= ps;
  return pgn;
}

static inline uint8_t j1939Sa(uint32_t id29)  { return (uint8_t)(id29 & 0xFF); }
static inline uint8_t j1939Pri(uint32_t id29) { return (uint8_t)((id29 >> 26) & 0x7); }

static void censusRecord(uint32_t pgn, uint8_t sa, uint32_t now) {
  for (size_t i = 0; i < censusUsed; i++) {
    if (census[i].pgn != pgn) continue;
    const uint32_t gap = now - census[i].lastMs;
    if (census[i].count == 1 || gap < census[i].minGapMs) census[i].minGapMs = gap;
    if (gap > census[i].maxGapMs) census[i].maxGapMs = gap;
    census[i].lastMs = now;
    census[i].count++;
    if (sa != census[i].saFirst && census[i].saCount == 1) census[i].saCount = 2;
    census[i].sa = sa;
    return;
  }
  if (censusUsed >= MAX_PGN_ENTRIES) { censusOverflow++; return; }
  census[censusUsed].pgn      = pgn;
  census[censusUsed].count    = 1;
  census[censusUsed].lastMs   = now;
  census[censusUsed].minGapMs = 0;
  census[censusUsed].maxGapMs = 0;
  census[censusUsed].sa       = sa;
  census[censusUsed].saFirst  = sa;
  census[censusUsed].saCount  = 1;
  censusUsed++;
}

static void printFrame(const CanMsg &msg, uint32_t now) {
  char line[96];
  if (msg.isExtendedId()) {
    const uint32_t id  = msg.getExtendedId();
    const uint32_t pgn = j1939Pgn(id);
    snprintf(line, sizeof(line), "%8lu  %08lX  p%u  PGN %6lu (%04lX)  SA %3u  dlc %u  ",
             (unsigned long)now, (unsigned long)id, (unsigned)j1939Pri(id),
             (unsigned long)pgn, (unsigned long)pgn, (unsigned)j1939Sa(id),
             (unsigned)msg.data_length);
  } else {
    snprintf(line, sizeof(line), "%8lu  %03lX       STD  ---------------------  dlc %u  ",
             (unsigned long)now, (unsigned long)msg.getStandardId(),
             (unsigned)msg.data_length);
  }
  Serial.print(line);
  for (uint8_t i = 0; i < msg.data_length; i++) {
    char b[4];
    snprintf(b, sizeof(b), "%02X", msg.data[i]);
    Serial.print(b);
  }
  Serial.println();
}

static void printCensus(uint32_t now) {
  const uint32_t elapsedS = (now - startMs) / 1000UL;
  Serial.println();
  Serial.println(F("---- PGN census ------------------------------------------------"));
  char hdr[128];
  snprintf(hdr, sizeof(hdr),
           "elapsed %lus  frames %lu  (%lu/s)  ext %lu  std %lu  distinct %u  overflow %lu",
           (unsigned long)elapsedS, (unsigned long)frameCount,
           (unsigned long)(elapsedS ? frameCount / elapsedS : 0),
           (unsigned long)(frameCount - stdFrames), (unsigned long)stdFrames,
           (unsigned)censusUsed, (unsigned long)censusOverflow);
  Serial.println(hdr);
  Serial.println(F("     PGN   hex    count   min_ms   max_ms   SA  multiSA"));
  for (size_t i = 0; i < censusUsed; i++) {
    char row[96];
    snprintf(row, sizeof(row), "%8lu  %04lX  %7lu  %7lu  %7lu  %3u  %s",
             (unsigned long)census[i].pgn, (unsigned long)census[i].pgn,
             (unsigned long)census[i].count, (unsigned long)census[i].minGapMs,
             (unsigned long)census[i].maxGapMs, (unsigned)census[i].sa,
             census[i].saCount > 1 ? "YES" : "no");
    Serial.println(row);
  }
  int err = 0;
  if (CAN.isError(err)) {
    char e[72];
    snprintf(e, sizeof(e), "CAN ERROR EVENT: %d  (see r_can_api.h can_event_t)", err);
    Serial.println(e);
    CAN.clearError();
  }
  Serial.println(F("----------------------------------------------------------------"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  // No `while (!Serial)` -- that hangs a board with no USB host attached, which
  // is how this one lives on the tractor.
  delay(200);

  Serial.println();
  Serial.println(F("can_sniff -- PLC #4 J1939 bus discovery"));
  Serial.println(F("UNO R4 WiFi: D10 = CANTX, D13 = CANRX"));
  Serial.print(F("bitrate: "));
  Serial.println(CAN_BITRATE);

  canUp = CAN.begin(CAN_BITRATE);
  if (!canUp) {
    Serial.println(F("CAN.begin FAILED."));
    Serial.println(F("  - wrong pins for this variant, or"));
    Serial.println(F("  - bitrate not reachable from the 24 MHz CAN clock, or"));
    Serial.println(F("  - peripheral already claimed."));
    Serial.println(F("This sketch will idle. Nothing was written to the bus."));
  } else {
    Serial.println(F("CAN up. Listening. No frame is ever transmitted by this sketch."));
    Serial.println(F("  ms       id      pri     PGN               SA   dlc  data"));
  }

  startMs      = millis();
  lastReportMs = startMs;
}

void loop() {
  if (!canUp) { delay(1000); return; }

  // Drain the whole RX buffer every pass, and understand that the frame count
  // below is a count of what we RECEIVED, not of what the bus carried. There are
  // two silent-loss stages between the wire and this loop, with opposite
  // policies and no counter on either:
  //
  //   1. the hardware mailboxes run CAN_MESSAGE_MODE_OVERWRITE, so a mailbox
  //      that is not serviced loses its OLDEST frame;
  //   2. CanMsgRingbuffer::enqueue() returns early when the 32-slot ring is
  //      full, so a full ring loses the NEWEST frame.
  //
  // Nothing increments on either path. Never gate this drain behind a timer.
  while (CAN.available()) {
    const CanMsg msg = CAN.read();
    const uint32_t now = millis();
    frameCount++;
    if (msg.isExtendedId()) {
      censusRecord(j1939Pgn(msg.getExtendedId()), j1939Sa(msg.getExtendedId()), now);
    } else {
      stdFrames++;
    }
    printFrame(msg, now);
  }

  const uint32_t now = millis();
  if (now - lastReportMs >= REPORT_MS) {
    lastReportMs = now;
    printCensus(now);
  }
}
