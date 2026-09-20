// can_loopback -- does Arduino_CAN actually RECEIVE on renesas_uno core 1.6.0?
//
// THIS IS THE FIRST TEST AND IT BLOCKS EVERY OTHER PIECE OF CAN WORK. Issue #19.
//
// THE DOUBT. In the core's own driver, libraries/Arduino_CAN/src/R7FA4M1_CAN.cpp,
// the receive branch of the FSP callback reads:
//
//     case CAN_EVENT_RX_COMPLETE: // Currently driver don't support this. This
//                                 // is unreachable code for now.
//
// That branch is the ONLY thing that enqueues into the ring buffer which
// CAN.available() and CAN.read() serve. If the comment is accurate,
// CAN.available() never becomes non-zero and no amount of correct J1939 decoding
// matters, because no frame ever arrives. Evidence it is probably stale:
// IRQManager.cpp:886 does register can_rx_isr and enable it, and the Renesas FSP
// r_can driver emits CAN_EVENT_RX_COMPLETE from that ISR. "Probably" is not a
// plan when the cost of being wrong is the whole approach.
//
// ---------------------------------------------------------------------------
// *** YOU MUST FIT A JUMPER WIRE BETWEEN THE TWO CAN PINS BEFORE RUNNING. ***
//
//     UNO R4 Minima : D4 (CANTX) to D5 (CANRX)
//     UNO R4 WiFi   : D10 (CANTX) to D13 (CANRX)
//
// The sketch prints the pair for the board it was built for. Do not take the
// numbers from memory -- getting these two boards' mappings crossed is the
// original error this whole workstream had to correct.
// ---------------------------------------------------------------------------
//
// This is not optional and it is not obvious, so here is why.
//
// The library method is called enableInternalLoopback(). It does not engage
// internal loopback. Read it:
//
//     int R7FA4M1_CAN::enableInternalLoopback() {
//       R_CAN_ModeTransition(&_can_ctrl, CAN_OPERATION_MODE_NORMAL,
//                            CAN_TEST_MODE_LOOPBACK_EXTERNAL);   // <-- EXTERNAL
//     }
//
// r_can_api.h:103-104 distinguishes them: CAN_TEST_MODE_LOOPBACK_EXTERNAL = 5,
// CAN_TEST_MODE_LOOPBACK_INTERNAL = 7. The misnamed method engages 5.
//
// EXTERNAL loopback drives the CTX pin and samples the CRX pin -- the signal
// goes out of the chip and must come back in. On a bare board with nothing on
// D10/D13, CRX sits recessive, no frame is ever received, and this sketch would
// print a FAIL that condemns the R4 path for a reason that has nothing to do
// with the RX callback. That false negative is the exact mistake this sketch
// exists to prevent, so it refuses to run without proving the jumper first.
//
// True INTERNAL loopback (mode 7) needs no wire, but Arduino_CAN does not expose
// it -- same missing-accessor problem as listen-only (issue #20), which is on
// hold. So the jumper is the only way to run this today.
//
// A plain wire is correct. Both pins are MCU logic level; no transceiver is
// involved and none should be fitted for this test.
//
// NO TRACTOR AND NO TRANSCEIVER ARE NEEDED -- and note that the board this repo
// normally flashes IS THE PRODUCTION SCALE, in service on the tractor. Do not
// put this sketch on that board to save a trip. Run it on a spare R4 WiFi.
//
// Read the verdict on the USB serial console at 115200 baud.

#include <Arduino_CAN.h>

static const uint32_t CAN_BITRATE   = 250000UL;  // matches the tractor; arbitrary in loopback
static const uint32_t RX_TIMEOUT_MS = 1000UL;    // generous: loopback should be microseconds

// TAKEN FROM THE CORE'S OWN VARIANT, NEVER HARDCODED. This is the whole D4/D5
// versus D10/D13 error made structurally impossible: the WiFi variant defines
// these as 10 and 13, the Minima defines them as 4 and 5, and whichever board
// this is compiled for gets its own correct pair. The sketch prints them at
// startup so the person fitting the jumper reads the number rather than guessing
// it, and so the serial log records which board the result came from.
static const uint8_t PIN_CANTX = PIN_CAN0_TX;
static const uint8_t PIN_CANRX = PIN_CAN0_RX;

#if defined(ARDUINO_UNOWIFIR4)
  static const char *BOARD_NAME = "UNO R4 WiFi";
#elif defined(ARDUINO_MINIMA)
  static const char *BOARD_NAME = "UNO R4 Minima";
#elif defined(ARDUINO_NANO_R4)
  static const char *BOARD_NAME = "Nano R4";
#else
  static const char *BOARD_NAME = "unknown RA4M1 board";
#endif

static const uint32_t TEST_EXT_ID = 0x18FEE000UL;  // shaped like a real J1939 id
static const uint32_t TEST_STD_ID = 0x123UL;

static uint8_t passes = 0;
static uint8_t failures = 0;

static void verdictLine(const char *name, bool ok, const char *detail) {
  char line[136];
  snprintf(line, sizeof(line), "  [%s] %-26s %s", ok ? "PASS" : "FAIL", name, detail);
  Serial.println(line);
  if (ok) passes++; else failures++;
}

// Prove the jumper exists BEFORE the CAN peripheral claims the pins, by driving
// D10 as a plain GPIO and reading D13. Without this, a missing wire and a dead
// RX path produce an identical symptom and the test cannot tell you which it is.
static bool jumperPresent(char *detail, size_t cap) {
  pinMode(PIN_CANTX, OUTPUT);
  pinMode(PIN_CANRX, INPUT);

  digitalWrite(PIN_CANTX, LOW);
  delay(2);
  const int readLow = digitalRead(PIN_CANRX);

  digitalWrite(PIN_CANTX, HIGH);
  delay(2);
  const int readHigh = digitalRead(PIN_CANRX);

  // Release both pins so CAN.begin() can reconfigure them as peripheral pins.
  pinMode(PIN_CANTX, INPUT);
  pinMode(PIN_CANRX, INPUT);

  if (readLow == LOW && readHigh == HIGH) {
    snprintf(detail, cap, "D10 -> D13 follows (low reads %d, high reads %d)", readLow, readHigh);
    return true;
  }
  snprintf(detail, cap,
           "D13 did not follow D10 (low reads %d, high reads %d) -- NO JUMPER FITTED",
           readLow, readHigh);
  return false;
}

// Send one frame and wait for it to return. True only if a frame arrived AND its
// id and payload survived: "something arrived" is not "the driver works".
static bool roundTrip(uint32_t id, bool extended, const uint8_t *payload, uint8_t len,
                      char *detail, size_t detailCap) {
  while (CAN.available()) (void)CAN.read();   // never mistake a stale frame for this one

  const CanMsg out(extended ? CanExtendedId(id) : CanStandardId(id), len, payload);
  const int wrc = CAN.write(out);
  if (wrc != 1) {
    snprintf(detail, detailCap, "CAN.write() returned %d, so nothing was ever sent", wrc);
    return false;
  }

  const uint32_t deadline = millis() + RX_TIMEOUT_MS;
  while (millis() < deadline) {
    if (!CAN.available()) continue;

    const CanMsg in = CAN.read();
    const uint32_t gotId = in.isExtendedId() ? in.getExtendedId() : in.getStandardId();

    if (in.isExtendedId() != extended) {
      snprintf(detail, detailCap, "returned as %s, sent as %s",
               in.isExtendedId() ? "extended" : "standard", extended ? "extended" : "standard");
      return false;
    }
    if (gotId != id) {
      snprintf(detail, detailCap, "id returned %08lX, sent %08lX",
               (unsigned long)gotId, (unsigned long)id);
      return false;
    }
    if (in.data_length != len) {
      snprintf(detail, detailCap, "dlc returned %u, sent %u",
               (unsigned)in.data_length, (unsigned)len);
      return false;
    }
    for (uint8_t i = 0; i < len; i++) {
      if (in.data[i] != payload[i]) {
        snprintf(detail, detailCap, "byte %u returned %02X, sent %02X",
                 (unsigned)i, in.data[i], payload[i]);
        return false;
      }
    }
    snprintf(detail, detailCap, "id, dlc and all %u data bytes intact", (unsigned)len);
    return true;
  }

  snprintf(detail, detailCap,
           "CAN.available() never went non-zero in %lums", (unsigned long)RX_TIMEOUT_MS);
  return false;
}

void setup() {
  Serial.begin(115200);
  // No `while (!Serial)`: a board with no USB host must not hang here.
  delay(2000);          // let a human open the monitor before the verdict scrolls

  char detail[136];

  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("can_loopback -- does Arduino_CAN receive on renesas_uno 1.6.0?"));
  snprintf(detail, sizeof(detail), "board: %s   CANTX = D%u   CANRX = D%u",
           BOARD_NAME, (unsigned)PIN_CANTX, (unsigned)PIN_CANRX);
  Serial.println(detail);
  snprintf(detail, sizeof(detail), "REQUIRES a jumper wire from D%u to D%u. No transceiver.",
           (unsigned)PIN_CANTX, (unsigned)PIN_CANRX);
  Serial.println(detail);
  Serial.println(F("The RESULT that transfers between boards is whether the RX event"));
  Serial.println(F("fires. The PIN NUMBERS DO NOT TRANSFER. Record the board above."));
  Serial.println(F("=================================================================="));
  Serial.println();

  // --- 0. the jumper, so that a later FAIL means something ---
  const bool jumper = jumperPresent(detail, sizeof(detail));
  verdictLine("D10-D13 jumper", jumper, detail);
  if (!jumper) {
    Serial.println();
    Serial.println(F("  STOPPING. Without the jumper this test cannot distinguish"));
    Serial.println(F("  \"the driver never receives\" from \"the signal had nowhere to go\","));
    Serial.println(F("  and a FAIL here would wrongly condemn the whole R4 CAN path."));
    Serial.println(F("  Fit a plain wire between D10 and D13 and re-run. Issue #19."));
    return;
  }

  // --- 1. does the peripheral come up at all ---
  const bool began = CAN.begin(CAN_BITRATE);
  snprintf(detail, sizeof(detail), "CAN.begin(%lu) returned %s",
           (unsigned long)CAN_BITRATE, began ? "true" : "false");
  verdictLine("CAN.begin", began, detail);
  if (!began) {
    Serial.println();
    Serial.println(F("  Cannot continue: the peripheral did not initialise. Check the"));
    Serial.println(F("  bitrate is reachable from the 24 MHz CAN clock and that nothing"));
    Serial.println(F("  else has claimed D10/D13."));
    return;
  }

  // --- 2. loopback mode (EXTERNAL, despite the method name) ---
  const int lb = CAN.enableInternalLoopback();
  snprintf(detail, sizeof(detail),
           "returned %d (1 = success); engages EXTERNAL loopback, hence the jumper", lb);
  verdictLine("enableInternalLoopback", lb == 1, detail);

  // --- 3. THE TEST THAT MATTERS: extended id, the J1939 case ---
  const uint8_t ext_payload[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67 };
  const bool extOk = roundTrip(TEST_EXT_ID, true, ext_payload, 8, detail, sizeof(detail));
  verdictLine("extended-id round trip", extOk, detail);

  // --- 4. standard id, to separate "extended is broken" from "rx is broken" ---
  const uint8_t std_payload[3] = { 0x11, 0x22, 0x33 };
  const bool stdOk = roundTrip(TEST_STD_ID, false, std_payload, 3, detail, sizeof(detail));
  verdictLine("standard-id round trip", stdOk, detail);

  // --- 5. does the 32-slot ring hold more than one frame ---
  while (CAN.available()) (void)CAN.read();
  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t p[2] = { i, (uint8_t)(0xA0 + i) };
    CAN.write(CanMsg(CanExtendedId(TEST_EXT_ID + i), 2, p));
    delay(5);                       // one TX mailbox: let each frame clear
  }
  delay(50);
  const size_t queued = CAN.available();
  snprintf(detail, sizeof(detail), "sent 4, CAN.available() reports %u", (unsigned)queued);
  verdictLine("multiple frames buffered", queued == 4, detail);
  while (CAN.available()) (void)CAN.read();

  // --- 6. error flags ---
  int err = 0;
  const bool isErr = CAN.isError(err);
  snprintf(detail, sizeof(detail),
           isErr ? "error event %d raised (see r_can_api.h can_event_t)"
                 : "no error events raised", err);
  verdictLine("no CAN error events", !isErr, detail);
  CAN.clearError();

  CAN.disableInternalLoopback();

  // --- the verdict, stated so nobody has to interpret it ---
  Serial.println();
  Serial.println(F("------------------------------------------------------------------"));
  char summary[96];
  snprintf(summary, sizeof(summary), "  %u passed, %u failed",
           (unsigned)passes, (unsigned)failures);
  Serial.println(summary);
  Serial.println();
  if (extOk) {
    Serial.println(F("  VERDICT: Arduino_CAN RECEIVES on this core."));
    Serial.println(F("  The \"unreachable code\" comment in R7FA4M1_CAN.cpp is STALE."));
    Serial.println(F("  Issue #19 is answered and the R4 path is viable."));
  } else {
    Serial.println(F("  VERDICT: NO FRAME WAS RECEIVED, WITH THE JUMPER PROVEN PRESENT."));
    Serial.println(F("  Arduino_CAN on renesas_uno 1.6.0 cannot receive as shipped and"));
    Serial.println(F("  the comment in R7FA4M1_CAN.cpp is accurate. Do not fit anything"));
    Serial.println(F("  else on the R4 CAN path until this is resolved. Options: a newer"));
    Serial.println(F("  core, a driver patched to poll the mailboxes, or an ESP32 (TWAI)"));
    Serial.println(F("  node instead. See REPORT.md section 4.4."));
  }
  Serial.println(F("  Report the result to the PLC #4 coordinator. Do not proceed to"));
  Serial.println(F("  #20, #21 or #22 on the strength of this alone."));
  Serial.println(F("------------------------------------------------------------------"));
}

void loop() {
  // Everything happens once in setup(). Re-state the headline every 10 s so a
  // monitor opened late still shows the verdict rather than a blank screen.
  delay(10000);
  Serial.println(failures == 0 ? F("[can_loopback] all checks passed -- RX works")
                               : F("[can_loopback] FAILURES -- see the verdict above"));
}
