// can_soak -- sustained receive test with real loss detection, and a simulated
// slow main loop.
//
// WHY THIS EXISTS, beyond "run it for longer". Two questions the two-node bench
// could not answer:
//
// 1. HOW MUCH DOES A SLOW LOOP COST? REPORT.md section 4.3 is the live
//    integration risk: the scale firmware's main loop is 500 ms, the CAN ring is
//    32 frames deep, and a ring that absorbs a burst inside a tight loop says
//    nothing about one drained twice a second. This sketch can insert a
//    deliberate blocking delay to imitate that loop and measure the loss.
//
// 2. IS THE LOSS REAL? The earlier test counted printed lines, which conflates
//    "received" with "printed" and cannot see a frame that never arrived. This
//    counts SEQUENCE GAPS in the payload, so a dropped frame is detected by its
//    absence rather than inferred from a total.
//
// The far end must send frames whose first four payload bytes are a big-endian
// incrementing counter. cangen does NOT do that, so use tools/can_soak_send.py.
//
// SERIAL COMMANDS (115200, newline-terminated):
//   d<ms>   set simulated loop delay, e.g. d500 -- imitates the scale firmware
//   r       reset all counters
//   s       print stats immediately
//
// Pins come from the variant: D4/D5 on a Minima, D10/D13 on a WiFi.

#include <Arduino_CAN.h>

static const uint32_t CAN_BITRATE = 250000UL;
static const uint32_t STATS_MS    = 5000UL;
static const uint32_t SEQ_ID      = 0x18FF0200UL;   // proprietary range

static const uint8_t PIN_CANTX = PIN_CAN0_TX;
static const uint8_t PIN_CANRX = PIN_CAN0_RX;

#if defined(ARDUINO_UNOWIFIR4)
  static const char *BOARD_NAME = "UNO R4 WiFi";
#elif defined(ARDUINO_MINIMA)
  static const char *BOARD_NAME = "UNO R4 Minima";
#else
  static const char *BOARD_NAME = "RA4M1 board";
#endif

static uint32_t rxTotal = 0;        // frames of any id
static uint32_t seqFrames = 0;      // frames carrying our sequence id
static uint32_t seqGapEvents = 0;   // times the sequence jumped forward
static uint32_t seqLost = 0;        // total frames implied missing by those jumps
static uint32_t seqBackwards = 0;   // restarts or reordering
static uint32_t otherFrames = 0;
static uint32_t lastSeq = 0;
static bool     haveLast = false;

static uint32_t loopDelayMs = 0;    // simulated slow main loop
static uint32_t lastStatsMs = 0;
static uint32_t runStartMs = 0;
static uint32_t maxAvailSeen = 0;   // high-water mark of the 32-slot ring
static bool     canUp = false;

static char cmd[16];
static uint8_t cmdLen = 0;

static void resetCounters() {
  rxTotal = seqFrames = seqGapEvents = seqLost = seqBackwards = otherFrames = 0;
  lastSeq = 0; haveLast = false; maxAvailSeen = 0;
  runStartMs = millis();
  CAN.clearError();
}

static void printStats() {
  const uint32_t elapsed = (millis() - runStartMs) / 1000UL;
  char line[200];
  snprintf(line, sizeof(line),
    "STATS t=%lus delay=%lums rx=%lu seq=%lu lost=%lu gaps=%lu back=%lu other=%lu ringmax=%lu",
    (unsigned long)elapsed, (unsigned long)loopDelayMs, (unsigned long)rxTotal,
    (unsigned long)seqFrames, (unsigned long)seqLost, (unsigned long)seqGapEvents,
    (unsigned long)seqBackwards, (unsigned long)otherFrames,
    (unsigned long)maxAvailSeen);
  Serial.println(line);

  const uint32_t expected = seqFrames + seqLost;
  if (expected > 0) {
    // Integer parts-per-million, to avoid float formatting on a hot path.
    const uint32_t ppm = (uint32_t)((uint64_t)seqLost * 1000000ULL / expected);
    snprintf(line, sizeof(line), "      loss = %lu of %lu expected (%lu ppm)",
             (unsigned long)seqLost, (unsigned long)expected, (unsigned long)ppm);
    Serial.println(line);
  }

  int err = 0;
  if (CAN.isError(err)) {
    snprintf(line, sizeof(line), "      CAN ERROR EVENT %d", err);
    Serial.println(line);
    CAN.clearError();
  }
}

static void handleCmd() {
  cmd[cmdLen] = '\0';
  if (cmdLen == 0) return;
  if (cmd[0] == 'd') {
    loopDelayMs = (uint32_t)atol(cmd + 1);
    char line[80];
    snprintf(line, sizeof(line), "OK loop delay = %lu ms, counters reset",
             (unsigned long)loopDelayMs);
    Serial.println(line);
    resetCounters();
  } else if (cmd[0] == 'r') {
    resetCounters();
    Serial.println(F("OK counters reset"));
  } else if (cmd[0] == 's') {
    printStats();
  }
  cmdLen = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  char line[128];
  Serial.println();
  Serial.println(F("=== can_soak: sustained RX with sequence-gap loss detection ==="));
  snprintf(line, sizeof(line), "board %s   CANTX D%u  CANRX D%u   bitrate %lu",
           BOARD_NAME, (unsigned)PIN_CANTX, (unsigned)PIN_CANRX,
           (unsigned long)CAN_BITRATE);
  Serial.println(line);
  Serial.println(F("commands: d<ms> set loop delay | r reset | s stats"));

  canUp = CAN.begin(CAN_BITRATE);
  Serial.println(canUp ? F("CAN up. Receive only -- this sketch never transmits.")
                       : F("CAN.begin FAILED"));
  resetCounters();
  lastStatsMs = millis();
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') handleCmd();
    else if (cmdLen < sizeof(cmd) - 1) cmd[cmdLen++] = c;
  }

  if (canUp) {
    // Record how deep the ring got before we drained it. With a simulated slow
    // loop this is the number that shows the ring filling towards its 32 limit.
    const size_t avail = CAN.available();
    if (avail > maxAvailSeen) maxAvailSeen = avail;

    while (CAN.available()) {
      const CanMsg msg = CAN.read();
      rxTotal++;
      if (msg.isExtendedId() && msg.getExtendedId() == SEQ_ID && msg.data_length >= 4) {
        const uint32_t seq = ((uint32_t)msg.data[0] << 24) | ((uint32_t)msg.data[1] << 16)
                           | ((uint32_t)msg.data[2] << 8)  |  (uint32_t)msg.data[3];
        seqFrames++;
        if (haveLast) {
          if (seq > lastSeq + 1) {
            seqGapEvents++;
            seqLost += (seq - lastSeq - 1);   // frames that never arrived
          } else if (seq <= lastSeq) {
            seqBackwards++;                    // sender restarted, or reordering
          }
        }
        lastSeq = seq;
        haveLast = true;
      } else {
        otherFrames++;
      }
    }
  }

  const uint32_t now = millis();
  if (now - lastStatsMs >= STATS_MS) {
    lastStatsMs = now;
    printStats();
  }

  // THE POINT OF THE SKETCH. Block for the configured period, imitating a main
  // loop that has other work to do -- the scale firmware's is 500 ms. Frames
  // arriving during this window must fit in the 32-slot ring or they are lost,
  // and CanMsgRingbuffer::enqueue() drops the NEWEST silently when full.
  if (loopDelayMs) delay(loopDelayMs);
}
