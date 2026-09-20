// can_twonode -- R4 against a real differential bus, with a foreign node.
//
// WHAT THIS PROVES THAT #19 DID NOT. The loopback proof showed the controller
// receives a frame it transmitted itself, through the pads, on a wire. It said
// nothing about a transceiver, a differential bus, bus arbitration, or a frame
// originated by somebody else. This sketch pairs the Arduino against an RH-02
// (a CANable running candleLight, seen by Linux as SocketCAN can0) and tests
// both directions independently.
//
// THE SECOND DIRECTION IS THE ONE THAT MATTERS. The tractor is a bus full of
// foreign talkers and we are a listener. "Can the R4 receive a frame it did not
// send, across a transceiver, from another node" is the question the whole
// workstream rests on. Transmitting is a bonus.
//
// ---------------------------------------------------------------------------
// BENCH WIRING -- AND THE TERMINATION IS THE OPPOSITE OF THE TRACTOR CASE
// ---------------------------------------------------------------------------
//
//   Arduino CANTX -> transceiver TXD/D        (pins printed at startup)
//   Arduino CANRX <- transceiver RXD/R
//   Arduino 5V    -> transceiver VCC          *** 5 V PART ONLY ***
//   Arduino GND   -> transceiver GND
//   transceiver CANH <-> RH-02 CAN_H
//   transceiver CANL <-> RH-02 CAN_L
//   Arduino GND      <-> RH-02 GND            common ground, not optional
//
// TERMINATION: 120 ohm at BOTH ENDS. R120 switch ON at the RH-02, and leave the
// 120 ohm fitted on the transceiver breakout. This bench IS the whole bus, so it
// must terminate itself.
//
// AT THE TRACTOR THE SETTING IS REVERSED -- R120 OFF, because that bus is
// already terminated at both ends and a third terminator loads it. Do not carry
// the bench habit onto the machine.
//
// An unterminated two-node bench bus often passes anyway on short leads, then
// fails as soon as anything changes. That is the worst outcome, because it
// teaches the wrong lesson. Terminate it properly even though it "works" without.
//
// *** USE A 5 V TRANSCEIVER: TJA1050 or MCP2551. ***
// The UNO R4 is fixed at 5 V logic. An SN65HVD230 is a 3.3 V part: its D input
// is not 5 V tolerant (absolute max ~3.6 V) and its R output swings only to
// 3.3 V, below the RA4M1's ~0.8 x VCC input-high threshold. It can pass on a
// bench and fail hot, and it can be damaged by the Arduino driving its input.
//
// ---------------------------------------------------------------------------
// THE OTHER END, and get this right or you will misread the result
// ---------------------------------------------------------------------------
//
//   sudo ip link set can0 down
//   sudo ip link set can0 up type can bitrate 250000 loopback off listen-only off
//   ip -details link show can0        # MUST show no flags at all
//
// NORMAL MODE. Not loopback, not listen-only. SocketCAN retains ctrlmode flags
// across bring-ups, so a previous bench session leaves them set unless cleared
// explicitly. A listen-only RH-02 cannot ACK this board's frames; the Arduino
// will retransmit, go error-passive, and you will read a configuration mistake
// as the R4 failing to transmit.
//
//   candump -td can0                        # watch this board transmit
//   cansend can0 0CF00400#FFFF6812FFFFFFFF  # make this board receive
//   cangen can0 -e -g 100 -n 20             # sustained traffic

#include <Arduino_CAN.h>

static const uint32_t CAN_BITRATE = 250000UL;
static const uint32_t TX_PERIOD_MS = 1000UL;
static const uint32_t STATS_MS     = 5000UL;

// Pins from the variant, never hardcoded: D4/D5 on a Minima, D10/D13 on a WiFi.
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

// A proprietary-range J1939 id, so nothing mistakes our bench traffic for a
// decoded engine parameter if these logs ever get read next to a real capture.
static const uint32_t TX_ID = 0x18FF0100UL;

static uint32_t txAttempts = 0, txOk = 0, txFail = 0;
static uint32_t rxFrames = 0, rxExtended = 0, rxStandard = 0;
static uint32_t lastTxMs = 0, lastStatsMs = 0;
static uint32_t seq = 0;
static bool canUp = false;

static inline uint32_t j1939Pgn(uint32_t id29) {
  const uint8_t  pf  = (uint8_t)((id29 >> 16) & 0xFF);
  const uint8_t  ps  = (uint8_t)((id29 >> 8) & 0xFF);
  const uint32_t edp = (id29 >> 25) & 0x1UL;
  const uint32_t dp  = (id29 >> 24) & 0x1UL;
  uint32_t pgn = (edp << 17) | (dp << 16) | ((uint32_t)pf << 8);
  if (pf >= 240) pgn |= ps;
  return pgn;
}

static void printRx(const CanMsg &msg) {
  char line[128];
  if (msg.isExtendedId()) {
    const uint32_t id = msg.getExtendedId();
    snprintf(line, sizeof(line), "RX  ext %08lX  PGN %6lu  SA %3u  dlc %u  ",
             (unsigned long)id, (unsigned long)j1939Pgn(id),
             (unsigned)(id & 0xFF), (unsigned)msg.data_length);
  } else {
    snprintf(line, sizeof(line), "RX  std %03lX                          dlc %u  ",
             (unsigned long)msg.getStandardId(), (unsigned)msg.data_length);
  }
  Serial.print(line);
  for (uint8_t i = 0; i < msg.data_length; i++) {
    char b[4];
    snprintf(b, sizeof(b), "%02X", msg.data[i]);
    Serial.print(b);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  char line[140];
  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("can_twonode -- R4 against a foreign node over a real bus"));
  snprintf(line, sizeof(line), "board: %s   CANTX = D%u   CANRX = D%u   bitrate %lu",
           BOARD_NAME, (unsigned)PIN_CANTX, (unsigned)PIN_CANRX,
           (unsigned long)CAN_BITRATE);
  Serial.println(line);
  Serial.println(F("5 V TRANSCEIVER REQUIRED. 120 ohm at BOTH ends on a bench bus"));
  Serial.println(F("(R120 ON at the RH-02) -- the OPPOSITE of the tractor setting."));
  Serial.println(F("Other end must be in NORMAL mode: loopback off, listen-only off."));
  Serial.println(F("=================================================================="));
  Serial.println();

  canUp = CAN.begin(CAN_BITRATE);
  if (!canUp) {
    Serial.println(F("CAN.begin FAILED -- nothing further will happen."));
    return;
  }
  Serial.println(F("CAN up, NORMAL mode (no loopback). Listening and transmitting."));
  Serial.println();

  lastTxMs = lastStatsMs = millis();
}

void loop() {
  if (!canUp) { delay(1000); return; }

  // Drain fully every pass: the ring is 32 deep and drops the newest when full.
  while (CAN.available()) {
    const CanMsg msg = CAN.read();
    rxFrames++;
    if (msg.isExtendedId()) rxExtended++; else rxStandard++;
    printRx(msg);
  }

  const uint32_t now = millis();

  if (now - lastTxMs >= TX_PERIOD_MS) {
    lastTxMs = now;
    // Payload carries a sequence number so a dropped frame is visible at the
    // far end as a gap rather than as silence.
    uint8_t payload[8] = {
      (uint8_t)(seq >> 24), (uint8_t)(seq >> 16), (uint8_t)(seq >> 8), (uint8_t)seq,
      0xC0, 0xFF, 0xEE, 0x00
    };
    const int rc = CAN.write(CanMsg(CanExtendedId(TX_ID), 8, payload));
    txAttempts++;
    char line[112];
    if (rc == 1) {
      txOk++;
      snprintf(line, sizeof(line), "TX  ext %08lX  seq %lu  ok",
               (unsigned long)TX_ID, (unsigned long)seq);
    } else {
      txFail++;
      // A negative return here with nothing else on the bus almost always means
      // the far end is not acking -- check it is in NORMAL mode, not listen-only.
      snprintf(line, sizeof(line),
               "TX  ext %08lX  seq %lu  FAILED rc=%d (no ACK? far end listen-only?)",
               (unsigned long)TX_ID, (unsigned long)seq, rc);
    }
    Serial.println(line);
    seq++;
  }

  if (now - lastStatsMs >= STATS_MS) {
    lastStatsMs = now;
    char line[144];
    snprintf(line, sizeof(line),
             "--- tx %lu ok / %lu fail of %lu | rx %lu (ext %lu, std %lu) ---",
             (unsigned long)txOk, (unsigned long)txFail, (unsigned long)txAttempts,
             (unsigned long)rxFrames, (unsigned long)rxExtended,
             (unsigned long)rxStandard);
    Serial.println(line);

    int err = 0;
    if (CAN.isError(err)) {
      snprintf(line, sizeof(line),
               "    CAN ERROR EVENT %d -- see can_event_t in r_can_api.h", err);
      Serial.println(line);
      CAN.clearError();
    }
  }
}
