// PLC #4 — tractor-mounted hydraulic loader scale (John Deere 5055E)
// Author: alex.pennington@organicengineer.com
// MQTT layer forward-ported onto the `main` line, 2026-09-01. See PORT-PLAN.md.
//
// DESIGN SPINE: the weighing path may never wait on, or be stopped by, the network.
// Nothing in this file blocks on WiFi or MQTT. The scale is fully usable offline;
// recorded weights queue in RAM and flush when the broker returns.

#define calWeight 530 // the weight of the calibration object
#include <Arduino.h>
#include <EEPROM.h>
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <WDT.h>
#include "FspTimer.h"
#include "arduino_secrets.h"
#include "build_provenance.h"
#include "wifi_networks.h"
#include "loop_scaling.h"
#include "mcp3421.h"
#include "lcd2004.h"
#include "ina219.h"
#include "j1939_can.h"   // issue #24: the tractor's J1939 bus, decoded in a timer ISR

// ---------------------------------------------------------------------------
// The on-board NFC reader, lid buttons, menus and panel client.
// docs/design/2026-10-29-onboard-rfid-buttons-menus.md, FW-1 to FW-8.
//
// The panel's URL, PLC4_PANEL_URL, is defined ONCE, in include/panel_client.h: the
// pendant's default PLC4_PWA_URL (mesh Plc4Operator.h:32-34), the same host as the broker.
// A build overrides it with -DPLC4_PANEL_URL=... .
// Build order, design §4.2: 0 flashes part (a) only -- HOME, TAG and the result screens,
// OK and BACK, one POST /api/cmd per RECORD. 1 adds part (b), the rest of the pendant set.
#ifndef PLC4_HMI_FULL
#define PLC4_HMI_FULL 1
#endif
// FW-8, moisture entry on TAG (part (b) only). 0 leaves it out.
#ifndef PLC4_HMI_MC
#define PLC4_HMI_MC 1
#endif
#include "pn532.h"         // FW-1
#include "hmi_buttons.h"   // FW-2
#include "cmd_id.h"        // FW-5
#include "mc_value.h"      // FW-8
#include "panel_client.h"  // FW-4
#include "hmi_menu.h"      // FW-3, FW-6

// ---------------------------------------------------------------------------
// Console: the HC-05 on Serial1 (D0/D1) is the field interface, but the board
// also has a USB port and a technician with a laptop will plug into it. Mirror
// output to both and accept commands from either.
//
// NOTE: there is deliberately NO `while (!Serial)` here. On this board `Serial`
// is USB CDC and that guard blocks forever with no host attached — it is defect
// C4 from the audit, the single worst line in the original firmware. Writes to
// an unopened CDC port are harmless; waiting for one is not.
// ---------------------------------------------------------------------------
class DualPrint : public Print {
  public:
    size_t write(uint8_t c) override {
      Serial.write(c);
      return Serial1.write(c);
    }
    size_t write(const uint8_t *b, size_t n) override {
      Serial.write(b, n);
      return Serial1.write(b, n);
    }
};
DualPrint Console;

bool debug_flag = false;

// ---------------------------------------------------------------------------
// Transducer scaling lives in include/loop_scaling.h, in MILLIAMPS, with the
// ADC counts derived from BURDEN_OHMS at compile time. Nothing here hard-codes
// a count. See include/loop_scaling.h for why the burden is 68 ohm and
// MA-LOOP-DESIGN.md for the design.
// ---------------------------------------------------------------------------
LoopState loopState = LOOP_OK;

float vADC;  // the voltage
float smoothedValue;  //the filtered value from the last reading
float alpha = 0.1; //the weight (between 0 and 1) given to the previous filter value
unsigned long previousMillis = 0;
int interval = 500; //choose the time between readings in milliseconds
bool display_rolling_weight = true;

int sensorValue = 0;
int sensorVoltage = 0; // the voltage
int pressureValue = 0; // pressure
int outputValue = 0;
int tareValue = 0;
float calFactor = 1.0; //calibration factor
int sensorMaxPSI = 3000; // Rosemount 1151 span, 0-3000 psi. See MA-LOOP-DESIGN.md §5.
int addr = 10;
#define rTsize 50
int rT[rTsize];
int rTindex = 0;
// The 16x2 layout warned before the tally went full; the 20x4 port dropped
// that warning. This restores it: the operator gets notice this many entries
// before `+` silently stops recording.
const int TALLY_WARN_REMAINING = 5;
// SIGNED, and it matters: outputValue goes negative below tare and on an
// uncalibrated board, and an unsigned total underflowed to 4294967281 for three
// entries of -5. A total that is visibly absurd is better than one that is quietly
// wrong, but neither belongs on the operator's display.
long rTsum = 0;

// The tally's running total, summed from the entries on demand rather than
// carried. `-` zeroes an entry and `b` clears the lot, so a carried total
// drifts from the tally it claims to describe the first time the operator
// removes a weight, and a published total that disagrees with what `sum`
// prints on the display is worse than publishing none. Returns long because
// rTsum is one: fifty ints cannot overflow it, and the unsigned version of
// this total underflowed on a negative weight once already.
static long tallyTotal() {
  long t = 0;
  for (int i = 0; i < rTindex; i++) t += rT[i];
  return t;
}

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// ---------------------------------------------------------------------------
// HMI. A 20x4 LCD behind a clear coverplate, four sealed buttons and an NFC
// reader on the lid.
//
// The old five-button keypad is gone; Alex's 2026-10-29 instruction brought back
// four IP65 buttons (BACK UP DOWN OK on D2-D5) and a PN532 on the header I2C bus,
// so an operator at the tractor can scan a tag and record (design §1, §3). The
// phone app's MQTT commands are unchanged and still run through the console's
// own command handler. The buttons do NOT publish to the board: like the
// pendant, they ask the panel (POST /api/cmd), and the panel publishes the
// command the board then runs through that same handler (design §5.1).
//
// The serial and Bluetooth console keeps every verb it had. It is the service
// and recovery path, and it is the only one that works with no broker in reach.
// ---------------------------------------------------------------------------
// BACK, UP, DOWN, OK, design §3.4. Each has 4.7 k to +5 V and 100 nF to GND at the
// board end. INPUT_PULLUP as well, deliberately: the design says not to RELY on the
// weak internal pull-up, and nothing here does -- the 4.7 k dominates it. It is set so
// that a board flashed before the buttons are wired reads four released buttons
// instead of four floating pins, which on part (b) could walk a phantom press into
// MENU -> TARE -> OK.
const uint8_t BTN_PINS[BTN_COUNT] = {2, 3, 4, 5};
ButtonPad buttons;
FspTimer btnTimer;
bool btnTimerOn = false;
unsigned long lastBtnPollMs = 0;

static uint8_t readButtons() {
  uint8_t m = 0;
  for (uint8_t i = 0; i < BTN_COUNT; i++)
    if (digitalRead(BTN_PINS[i]) == LOW) m |= (uint8_t)(1u << i);   // NO contact to GND: pressed is low
  return m;
}

// 200 Hz, design §3.5: sampled in a timer so a press made while loop() is inside a
// 23 s WiFi reconnect is still caught and queued.
static void btnSampleISR(timer_callback_args_t *) {
  buttons.sample(readButtons(), millis());
}

// The PN532 on Wire (A4/A5) at 0x24, design §2.2. One I2C transaction per driver step.
// Wire's own transaction timeout is 100 ms (Wire.h:36 WIRE_DEFAULT_TIMEOUT_US), and
// Wire.begin() enables the RA4M1 pull-ups on SDA/SCL (Wire.cpp:255-256), so with no
// reader fitted a probe NACKs at once rather than waiting.
struct NfcWireBus {
  bool write(const uint8_t *d, size_t n) {
    Wire.beginTransmission(PN532_I2C_ADDR);
    Wire.write(d, n);
    return Wire.endTransmission() == 0;
  }
  bool read(uint8_t *d, size_t n) {
    if (Wire.requestFrom(PN532_I2C_ADDR, n) != n) return false;
    for (size_t i = 0; i < n; i++) d[i] = (uint8_t)Wire.read();
    return true;
  }
};
NfcWireBus nfcBus;
Pn532Driver<NfcWireBus> nfc(nfcBus);
TagTracker tagTracker;
char nfcLastTag[TAG_HEX_LEN] = "";

// The panel client's own socket, beside the MQTT one (design §5.1: a second socket on the
// WiFi coprocessor; whether WiFiS3 holds both is for a bench run to show). NOTE its
// constructor heap-allocates a 1024-byte receive FIFO (WiFiClient.h:253 RX_BUFFER_DIM),
// which the build's global-RAM figure does not count.
WiFiClient panelWifi;
struct PanelTransport {
  int connect(const char *h, uint16_t p) { return panelWifi.connect(h, p); }
  size_t write(const uint8_t *b, size_t n) { return panelWifi.write(b, n); }
  int read(uint8_t *b, size_t n) { return panelWifi.read(b, n); }
  void stop() { panelWifi.stop(); }
};
PanelTransport panelTransport;
PanelHttp<PanelTransport> panel(panelTransport);
uint16_t panelSeq = 0;   // the menu's sequence number for the request in flight

// FW-5. The session counter's slot: absolute EEPROM address 1024, the first byte of the
// data flash's SECOND 1 KB erase block (pins_arduino.h:176-177, FLASH_TOTAL_SIZE 0x2000,
// FLASH_BLOCK_SIZE 0x400). The calibration and tally block is addr(10)..+233, all in the
// first erase block (the EEPROM map below). EEPROM.put() erases the whole block it writes
// (see the note above tallySaveIndex()), so a separate block keeps this once-per-boot
// write from ever erasing and rewriting the block that holds the calibration and the
// operator's tally. The design left the slot to this change (§6 FW-5).
const int CMD_ID_EEPROM_ADDR = 1024;
static bool cmdIdLoad(CmdIdStore &o) { EEPROM.get(CMD_ID_EEPROM_ADDR, o); return true; }
static void cmdIdSave(const CmdIdStore &i) { EEPROM.put(CMD_ID_EEPROM_ADDR, i); }
CmdIdMinter cmdIds;
HmiMenu hmi;

// A verb the menu asked to run locally (TARE with no panel, design §5.5), through the same
// handler as the console. The console's half-typed line is parked here meanwhile, as the
// MQTT path parks it in consoleSaved.
bool localCmdPending = false;
char localSaved[65] = "";

// What setup() could and could not allocate (review item 4). The RA4M1 has 32 interrupt
// slots and the build is near them, so a timer or the Wire controller can fail to get one.
// Each failure is printed at boot and published as `hw_fail` on status.
bool hbTimerOn = false;   // the heartbeat watchdog refresher
bool wireUp    = false;   // Wire.begin() has run (after the heartbeat and CAN timers)
bool wireOk    = false;   // ...and the controller looks initialised, see wireLooksInitialised()
// Set by a verb that REFUSED, cleared immediately before each command runs.
// The MQTT ack cannot read the outcome at the point the verb is queued -- the
// handler runs later in the same loop pass -- so this is how the verdict gets
// back to the phone. Without it an ack would report success for a refusal,
// which is worse than no ack at all.
bool verbRefused = false;
// Set alongside verbRefused when the reason is NOT the loop state -- kv's
// span<=0.0f failure, a full tally, an empty tally, a rejected argument. Empty
// means "use displayStateName(), as before"; non-empty overrides it. Reset with
// verbRefused so a stale reason from one command can never be reported against
// a different one.
char verbRefusedReason[21] = "";

// --- non-blocking sample accumulation (replaces the 10x delay(10) burst) ----
const int SAMPLE_COUNT = 10;
const unsigned long SAMPLE_SPACING_MS = 50;   // 10 x 50 ms = the old 500 ms period
unsigned long lastSampleMs = 0;
unsigned long lastReadingMs = 0;   // millis() of the last COMPLETED sample cycle; see READING_STALE_MS
int  sampleN = 0;
float sampleTotal = 0;

// A nominal cycle is SAMPLE_COUNT 10 x SAMPLE_SPACING_MS 50 = 500 ms, so
// 2000 ms is four cycles of headroom -- decisively a real fault, not a
// transient bus hiccup or one slow cycle. This bounds the age of the
// DISPLAYED READING (the last completed sample cycle), not the age of the
// last successful individual read -- an intermittent contact can keep
// mcpRead() succeeding often enough to never look stale by that measure
// while the reading itself goes many seconds without updating. See F1 in
// .superpowers/sdd/2026-09-16-mcp3421-lcd2004/review-9-10.md.
const unsigned long READING_STALE_MS = 2000;

// ---------------------------------------------------------------------------
// F1 watchdog period, and the WiFi timeout that MUST sit under it.
//
// These two numbers are coupled, and getting them wrong produces a reboot loop
// in exactly the situation the whole design exists to serve. Read before editing.
//
// Verified against ArduinoCore-renesas 1.6.0:
//   * The WDT period ceiling is RL_16384 * PR_8192 / (PCLKB_Hz/1000)
//     (WDT.cpp getMaxTimeout). UNOWIFIR4 runs ICLK /1 from 48 MHz and PCLKB /2
//     (bsp_clock_cfg.h), so PCLKB = 24 MHz and the ceiling is 5592 ms. There is
//     no way to ask for a longer watchdog on this part.
//   * CWifi::begin() busy-waits `while (millis() - start < _timeout)` polling
//     status(), with NO watchdog refresh inside the loop (WiFiS3/WiFi.cpp), and
//     CWifi's constructor defaults _timeout to 10000 ms (WiFi.cpp:8).
//
// So with the watchdog armed at ANY achievable period, a FAILED WiFi.begin()
// would always blow it — and a failed begin() is precisely what happens when the
// tractor drives out of range. Left alone, the board would reset-loop whenever it
// was away from the yard: the exact opposite of "keep weighing, buffer, flush".
//
// The fix is WiFi.setTimeout() (WiFi.cpp:599), called in setup(). Association now
// gives up well inside the watchdog window and loop() carries on weighing.
//
// If you raise WIFI_CONNECT_TIMEOUT_MS, raise the watchdog to match — and it
// cannot go above 5592.
// ---------------------------------------------------------------------------
// Bump this if the EEPROM layout ever changes, and old stored values are ignored
// rather than reinterpreted.
//
// BUMPED 2026-09-13 for the 4-20 mA front end, with Alex's specific approval.
// Counts-per-psi moved by ~1.67x (13107 counts of span became 7864), so a
// voltage-era tareValue and calFactor would restore CLEANLY, pass the bounds
// check below, and publish "cal":"ok" over a scale wrong by that ratio. That is
// the same failure as the 16256 leftover documented at the EEPROM restore --
// which this signature was added to catch, and which the OLD value would miss.
//
// BUMPED AGAIN 2026-09-16 for the MCP3421 front end. Counts-per-psi moved from a
// 7864-count span to 4352, so a PL4A calibration would restore cleanly, pass the
// bounds check, and publish "cal":"ok" over a scale wrong by that ratio -- the
// same trap PL4A itself was created to close.
//
// CONSEQUENCE, ACCEPTED: the board demands a full recalibration after flashing
// and will not weigh correctly until it gets one.
const uint32_t EEPROM_SIGNATURE = 0x504C3442;   // 'PL4B'  4-20 mA calibration block, MCP3421
// The tally gets its OWN signature so clearing or rejecting calibration cannot
// silently discard the operator's recorded work, and vice versa.
const uint32_t TALLY_SIGNATURE  = 0x544C5934;   // 'TLY4'
// EEPROM map, all relative to addr(10):
//   +0  calFactor   +4  tareValue   +8  alpha   +12 EEPROM_SIGNATURE
//   +16 TALLY_SIGNATURE   +20 rTindex   +24 rT[rTsize]
// and, ABSOLUTE and in the next erase block: 1024 the command-id session record
// (CMD_ID_EEPROM_ADDR, include/cmd_id.h).
bool calibrationDefaulted = false;

const uint32_t WDT_TIMEOUT_MS            = 5000;   // ceiling on this part is 5592
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 3000;   // CWifi::begin association wait
const unsigned long MQTT_CONNECT_TIMEOUT_MS = 2000;   // CONNACK wait only

// R1: mqttClient.connect() does the TCP connect FIRST and only then enters its own
// timeout loop. The TCP half is deliberately NOT bounded — see the note in setup()
// on wifiClient.setConnectionTimeout(), which breaks the connect outright. The
// heartbeat watchdog covers it instead.
//
// BUT BOUNDING ALONE CANNOT SOLVE THIS, and an earlier revision of this file claimed
// it could. Proven on hardware 2026-09-01: the board reset-looped every 30 s against a
// real SSID. CWifi::begin has THREE blocking stages, not one —
//
//   WiFi.cpp  modem.write(_MODE)      capped only by MODEM_TIMEOUT = 10000 ms
//   WiFi.cpp  modem.write(_BEGINSTA)  capped only by MODEM_TIMEOUT = 10000 ms
//   WiFi.cpp  while (millis() - start < _timeout)   <-- the ONLY one WiFi.setTimeout bounds
//
// Worst case 23 s against a 5592 ms ceiling that cannot be raised. Lowering the modem
// timeout far enough to fit would be shorter than association itself needs, so the
// board would never connect. There is no set of timeouts that works.
//
// Hence the heartbeat watchdog below. The timeouts are still set — they keep the normal
// case short — but they are no longer what keeps the board alive.
const uint32_t HEARTBEAT_TOLERANCE_MS = 30000;   // must exceed the 23 s worst case above
volatile uint32_t loopHeartbeatMs = 0;
FspTimer wdtTimer;

// Refresh the hardware watchdog from a timer ISR, but ONLY while loop() is still
// ticking. A timer that refreshes unconditionally is the classic watchdog anti-pattern —
// it keeps the dog fed while the program is dead. Gating on the heartbeat keeps hang
// detection; it just widens it from 5 s to ~30 s, which is the price of a network stack
// whose blocking calls cannot be bounded below the watchdog ceiling.
static void wdtHeartbeatISR(timer_callback_args_t *) {
  if ((uint32_t)(millis() - loopHeartbeatMs) < HEARTBEAT_TOLERANCE_MS) {
    WDT.refresh();
  }
}


// ---------------------------------------------------------------------------
// Networking
// ---------------------------------------------------------------------------
// Power monitor (INA219, mux ch2). Polled on its own slow cadence rather than
// with the loop samples: it is diagnostics, not the measurement, and it must
// never share a pass with anything that matters. Both values are LAST KNOWN and
// only meaningful while inaOk is true.
float    inaBusVolts   = 0.0f;
float    inaShuntMv    = 0.0f;
bool     inaOk         = false;
unsigned long lastInaMs = 0;
const unsigned long INA_POLL_MS = 2000;

// WiFi networks, in priority order, from include/wifi_networks.h (generated by
// tools/gen-wifi-networks.sh, gitignored). ONE association attempt per netPoll
// pass, cycling this index on failure -- see the timing note above netPoll().
// Two WiFi.begin() calls in a single pass would be ~46 s against a 30 s
// heartbeat tolerance and would reset the board.
int wifiIndex = 0;

const char broker[] = SECRET_MQTT_BROKER;   // LAN form: 192.168.10.99
// The panel the lid buttons ask, PLC4_PANEL_URL, is defined in include/panel_client.h.
// Same host as the broker, port 8081.
const int  port     = 1883;
const char mqttUser[] = SECRET_MQTT_USER;
const char mqttPass[] = SECRET_MQTT_PASS;

// Topic root: role prefix for estate consistency, equipment segment for the
// identity this firmware already announces. ONE constant — every topic is built
// from it with snprintf into a sized buffer, so renaming it can never truncate.
// (That is X1 step 1, satisfied by construction rather than by a later fix.)
const char TOPIC_ROOT[] = "plc4/jd5055e";
const char CLIENT_ID[]  = "plc4-jd5055e";   // stable across reboots, unlike the library default

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

enum NetState { NET_OFFLINE, NET_WIFI_UP, NET_ONLINE };
NetState netState = NET_OFFLINE;
unsigned long lastNetAttemptMs = 0;
int  lastMqttError = 0;          // MqttClient::connectError() from the last attempt
// Bumped on every successful (re)connect. A captured command carries the value
// that was current when it arrived, and is discarded if it no longer matches --
// see cmdRunCapture(). This is what makes "captured on the connection that is
// still up" checkable; netState cannot answer it, because an MQTT-only drop can
// be detected and repaired inside a single loop pass and netState would read
// NET_ONLINE at both ends of the outage.
unsigned long netGeneration = 0;
unsigned long netAttempts = 0;   // how many times netPoll has tried to connect
const unsigned long NET_RETRY_MS = 30000;   // bounded attempt rate, never a loop
const unsigned long STAGE_GAP_MS = 500;     // gap between association and broker stages
unsigned long lastTelemetryMs = 0;
unsigned long lastStatusMs = 0;
bool willArmed = false;
// Counts in the 20.0-21.0 mA NE43 band (COUNTS_FS..COUNTS_FAULT_HIGH, LOOP_SATURATED)
// mean the transmitter is at or past its assumed 3000 psi full scale. That is NOT a
// fault — it is either a genuine pressure excursion above the circuit's relief (spikes
// on valve closure do this) or evidence the assumed span is wrong. Either way it must be
// VISIBLE rather than inferred from psi arithmetic, and it must not gate weighing: an
// over-range sample is still a reading.
unsigned long overRangeCount = 0;
int overRangeMaxCounts = 0;
// The 3.5-4.0 mA dead band: below the live zero but above a dead loop. Reads as
// a healthy 0 psi and is STRUCTURALLY UNCHANGED by the swap (playbook §8). What
// the swap does remove is its dominant cause, excitation sensitivity -- a
// current loop has no excitation dependency, so a sagging supply can no longer
// move the reading. What can still land here is a transmitter drifting below
// its own zero, which is a calibration problem rather than a wiring one.
// Counting it is the mitigation, not a fix.
unsigned long underRangeCount = 0;
const unsigned long STATUS_MS = 60000;   // heartbeat; also published immediately on connect
const unsigned long TELEMETRY_MS = 5000;
// Engine data off the J1939 bus (issue #24). Its own cadence, not TELEMETRY_MS:
// the scale's 5 s is a weighing cadence and engine numbers do not need it.
const unsigned long ENGINE_MS = 15000;
unsigned long lastEngineMs = 0;
// DTC publishes are QoS 1 and each can spin up to MQTT_CONNECT_TIMEOUT_MS for its
// PUBACK. Rate-limited whether they succeed or not -- a failed one used to be
// retried on the very next pass, 2 s at a time, which is the stall this file
// already refuses for publishStatus(). A failure backs off for longer.
const unsigned long DTC_GAP_MS     = 2000;
const unsigned long DTC_BACKOFF_MS = 30000;
unsigned long nextDtcMs = 0;
bool canUp = false;

// --- RAM backlog queue -----------------------------------------------------
// Ruled by Alex 2026-09-01: RAM only. A queued reading need not survive a power
// cycle, so there is no flash wear budget and no EEPROM layout work here.
//
// Only OPERATOR-RECORDED weights (the `+` verb) are queued. Periodic telemetry
// is a live sample — a stale one has no value, and queuing it would fill any
// RAM queue in minutes. A recorded weight is a deliberate act with lasting
// value. Flagged in PORT-PLAN.md for Alex; changing it is a one-line decision.
struct WeightRecord {
  int32_t  weight;
  uint32_t capturedAtMs;
  uint8_t  sensorAtCapture;   // LoopState AS IT WAS when this weight was taken
  uint8_t  calDefaultAtCapture;
};
// THIS NUMBER IS STATED IN TWO PLACES AND ONLY ONE OF THEM IS IN THIS REPO.
// The estate consumer's PLC #4 handler documents the same 200-record bound in
// its own docstring, written independently and reconciled against this line for
// the first time on 2026-09-20. Neither mentioned the other until then.
//
// So if this depth changes, the consumer's documented expectation goes stale
// silently -- nothing builds both, nothing tests both, and the mismatch would
// only surface as a surprise during a deep flush, which is exactly when nobody
// wants a surprise. Tell whoever owns /opt/plc1-monitoring.
const int QUEUE_DEPTH = 200;              // ~4 working days at 50 buckets/day
WeightRecord queueBuf[QUEUE_DEPTH];
int queueHead = 0;                        // oldest
int queueCount = 0;
unsigned long queueDropped = 0;           // overflow policy: drop OLDEST, and count it

// ---------------------------------------------------------------------------
// Why did we last reset? The RA4M1 records it in hardware, so this does not have
// to be inferred. Distinguishes a watchdog bite from a brownout, which is exactly
// the open question about the reset seen at first association.
//   RSTSR0: PORF b0, LVD0RF b1, LVD1RF b2, LVD2RF b3   (power / voltage events)
//   RSTSR1: IWDTRF b0, WDTRF b1, SWRF b2               (watchdog / software)
// ---------------------------------------------------------------------------
// MEASURED behaviour of this decode on real hardware, 2026-09-01 — it is less
// discriminating than the register map suggests, and `reset` is consumed downstream:
//   watchdog bite      -> "WATCHDOG (r1=0002)"   detected, verified with the `hang` verb
//   upload / software  -> "software (r1=0004)"   detected
//   USB power cycle    -> "none-flagged"         PORF is NOT observed
//   shield RESET button-> "none-flagged"         indistinguishable from a power cycle
// PORF is presumably read and cleared by the core's BSP startup before setup() runs.
// So this separates a watchdog reset and an upload from everything else, and does NOT
// separate a power cycle from the RES# pin. Do not claim more than that.
char resetCause[40] = "unknown";

static void captureResetCause() {
  uint8_t  r0 = R_SYSTEM->RSTSR0;
  uint16_t r1 = R_SYSTEM->RSTSR1;
  snprintf(resetCause, sizeof(resetCause), "%s%s%s%s%s%s (r0=%02X r1=%04X)",
           (r1 & 0x0002) ? "WATCHDOG "   : "",
           (r1 & 0x0001) ? "IWDT "       : "",
           (r1 & 0x0004) ? "software "   : "",
           (r0 & 0x0001) ? "power-on "   : "",
           (r0 & 0x000E) ? "BROWNOUT "   : "",
           (!(r0 & 0x000F) && !(r1 & 0x0007)) ? "none-flagged " : "",
           r0, r1);
  // Clear the flags so the NEXT boot reports its own cause, not this one.
  // These live behind register write protection; PRC1 covers them.
  R_SYSTEM->PRCR = 0xA50B;
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
  R_SYSTEM->PRCR = 0xA500;
}

void  serialEvent();
void printDebug();

// ---------------------------------------------------------------------------
// Topic building — sized buffer, never a hard-coded length
// ---------------------------------------------------------------------------
static bool buildTopic(const char *leaf, char *out, size_t len) {
  int n = snprintf(out, len, "%s/%s", TOPIC_ROOT, leaf);
  return n > 0 && (size_t)n < len;      // refuse a truncated topic rather than publish to a stub
}

static bool publishTo(const char *leaf, const char *payload) {
  if (netState != NET_ONLINE) return false;
  char topic[64];
  if (!buildTopic(leaf, topic, sizeof(topic))) return false;
  // SIZED beginMessage, not the bare form. Without a size the library buffers
  // the whole message into a 256-byte _txBuffer and SILENTLY TRUNCATES past
  // it; with one it writes the header and streams, so the only limit is the
  // caller's own buffer. Measured on the wire: the status payload was being
  // cut at 200 bytes, mid-token, so the JSON never closed and every consumer
  // discarded the whole message.
  if (!mqttClient.beginMessage(topic, (unsigned long)strlen(payload))) return false;   // return IS checked — silent loss was C6
  mqttClient.print(payload);
  return mqttClient.endMessage() == 1;
}

static const char *sensorStateName() { return loopStateName(loopState); }

// Whether the DISPLAYED READING is fresh enough to mean anything. Distinct
// from loop state: the loop can be perfectly healthy while the ADC that
// reads it has stopped answering, and a held sample is not a measurement.
// Gated on lastReadingMs -- stamped once per COMPLETED sample cycle, not per
// successful individual mcpRead() -- because an intermittent contact can
// keep individual reads succeeding often enough to never look stale while
// the reading itself sits unchanged for many seconds. Unsigned subtraction
// is deliberate and handles millis() rollover correctly -- do not "fix" it
// by comparing absolute values.
static bool readingFresh()       { return millis() - lastReadingMs <= READING_STALE_MS; }
static bool readingTrustworthy() { return loopIsMeasuring(loopState) && readingFresh(); }
static bool operatorMayAct()     { return loopState == LOOP_OK && readingFresh(); }

// sensorStateName() names the LOOP state and stays on the wire unchanged.
// This is the DISPLAY-only wrapper: staleness takes precedence over loop
// state wherever a state name is shown to the operator, because if the ADC
// is not answering we do not know the loop state, and reporting the
// last-known one would be a stale claim of its own.
static const char *displayStateName() {
  return readingFresh() ? sensorStateName() : "NO ADC";
}

// A buffered reading MUST be distinguishable from a live one. Publishing
// backlog bare is worse than not buffering, because the consumer cannot tell
// stale from fresh. age_ms is the delta at flush; live readings carry 0.
static bool publishWeight(int32_t w, uint32_t ageMs, bool backlog) {
  char payload[160];
  // `total` is the tally AFTER this weight, so a consumer gets the running
  // total atomically with the record that moved it and never has to add the
  // weights up itself. The status topic carries the same figure, but only once
  // a minute; this one arrives the moment the operator records.
  //
  // The replayed-backlog path in queueFlush() deliberately does NOT carry it.
  // The total there would be today's, standing beside a weight captured before
  // the link dropped -- a true number in a position that makes it read as the
  // total at capture.
  snprintf(payload, sizeof(payload),
           "{\"w\":%ld,\"age_ms\":%lu,\"backlog\":%s,\"sensor\":\"%s\","
           "\"cal\":\"%s\",\"n\":%d,\"total\":%ld,\"id\":\"%s\"}",
           (long)w, (unsigned long)ageMs, backlog ? "true" : "false",
           sensorStateName(), calibrationDefaulted ? "DEFAULT" : "ok",
           rTindex, tallyTotal(), CLIENT_ID);
  return publishTo("weight", payload);
}

// ---------------------------------------------------------------------------
// Retained status: whether to BELIEVE the numbers, not what they are.
//
// Everything here was already known to the board and reachable only from the LCD or
// a serial console in the cab. A consumer could not tell a calibrated scale from one
// running on defaults, could not see that weights had been dropped, and could not
// tell "out of radio range" from "wedged".
//
// Retained, so a subscriber that connects later gets the current state rather than
// silence. Paired with the last will below, silence finally means something.
//
// Deliberately NOT a boot counter: persisting one would mean an EEPROM write on every
// boot, and a board in a reset loop would then wear the flash out while failing.
// Uptime plus the hardware reset cause tell the same story without that risk.
// ---------------------------------------------------------------------------
unsigned long statusTooLong = 0;   // status payloads that did not fit even without optional keys

static void publishStatus(unsigned long now) {
  lastStatusMs = now;
  // 448 -> 512 for `nfc`, `nfc_fw` and `hw_fail`. Sized, streamed beginMessage below, so the
  // only limit is this buffer.
  char payload[512];
  // `stale` is a bare JSON boolean, not a string, and sits right after
  // `sensor` -- same form and placement as the telemetry payload, so the
  // two agree. This message is RETAINED: a subscriber that connects after
  // the ADC has already died gets stale:true on its very first delivery,
  // instead of a last-known-good "sensor":"ok" that lingers forever.
  // Supply diagnostics publish JSON null when the monitor is absent or has
  // failed, never 0 -- a real zero volts and an unread sensor are different
  // facts, the same rule `psi` already follows.
  char vbusField[14], shuntField[14];
  // FW-7: the reader, "ok" with its firmware ("1.6") or "none" with null. A silent reader
  // is not a scale fault and never reaches the fault row (design §4.12).
  char nfcFwField[12];
  if (nfc.health() == NFC_OK)
    snprintf(nfcFwField, sizeof(nfcFwField), "\"%u.%u\"", (unsigned)nfc.fwVer(), (unsigned)nfc.fwRev());
  else
    snprintf(nfcFwField, sizeof(nfcFwField), "null");
  if (inaOk) {
    snprintf(vbusField,  sizeof(vbusField),  "%.3f", (double)inaBusVolts);
    snprintf(shuntField, sizeof(shuntField), "%.3f", (double)inaShuntMv);
  } else {
    snprintf(vbusField,  sizeof(vbusField),  "null");
    snprintf(shuntField, sizeof(shuntField), "null");
  }

  // `hw_fail` (review item 4): what setup() could not allocate, comma-separated, and the
  // key is present only when something failed.
  char hwFail[48] = "";
  {
    size_t k = 0;
    const char *parts[4] = {hbTimerOn ? nullptr : "hb_timer", canTimerOn ? nullptr : "can_timer",
                            btnTimerOn ? nullptr : "btn_timer", (wireUp && !wireOk) ? "wire" : nullptr};
    for (const char *p : parts) {
      if (!p) continue;
      const int w = snprintf(hwFail + k, sizeof(hwFail) - k, "%s%s", k ? "," : "", p);
      if (w > 0 && (size_t)w < sizeof(hwFail) - k) k += (size_t)w;
    }
  }
  char opt[96] = "";
  snprintf(opt, sizeof(opt), "\"nfc\":\"%s\",\"nfc_fw\":%s,%s%s%s",
           nfc.health() == NFC_OK ? "ok" : "none", nfcFwField,
           hwFail[0] ? "\"hw_fail\":\"" : "", hwFail, hwFail[0] ? "\"," : "");

  // A truncated status is invalid JSON and every consumer drops the whole retained message
  // -- the defect the sized beginMessage below was added for. So the return is CHECKED
  // (review item 5): if the payload does not fit, it is built again without the optional
  // keys (`nfc`, `nfc_fw`, `hw_fail`), and if even that does not fit nothing is published.
  int pn = -1;
  for (int pass = 0; pass < 2; pass++) {
    pn = snprintf(payload, sizeof(payload),
    "{\"up\":true,\"cal\":\"%s\",\"sensor\":\"%s\",\"stale\":%s,\"tare\":%d,\"cf\":%.5f,"
    "\"n\":%d,\"total\":%ld,\"q\":%d,\"dropped\":%lu,\"uptime_s\":%lu,\"rssi\":%d,"
    "\"overrange\":%lu,\"overrange_max\":%d,\"underrange\":%lu,\"i2c_err\":%lu,"
    "\"vbus\":%s,\"shunt_mv\":%s,\"ina_err\":%lu,"
    "%s"
    "\"build\":\"%s\",\"reset\":\"%s\",\"id\":\"%s\"}",
    calibrationDefaulted ? "DEFAULT" : "ok",
    sensorStateName(), readingFresh() ? "false" : "true", tareValue, calFactor,
    rTindex, tallyTotal(), queueCount, queueDropped,
    (unsigned long)(now / 1000UL), (int)WiFi.RSSI(),
    overRangeCount, overRangeMaxCounts, underRangeCount,
    (unsigned long)mcpFailCount(),
    vbusField, shuntField, (unsigned long)inaFailCount(),
    pass == 0 ? opt : "",
    BUILD_PROVENANCE, resetCause, CLIENT_ID);
    if (pn > 0 && (size_t)pn < sizeof(payload)) break;
  }
  if (pn <= 0 || (size_t)pn >= sizeof(payload)) { statusTooLong++; return; }
  char topic[64];
  if (!buildTopic("status", topic, sizeof(topic))) return;
  if (!mqttClient.beginMessage(topic, (unsigned long)strlen(payload), true, 1)) return;   // retained, QoS 1, STREAMED not buffered
  mqttClient.print(payload);
  mqttClient.endMessage();
}

static void queuePush(int32_t w) {
  if (queueCount == QUEUE_DEPTH) {
    queueHead = (queueHead + 1) % QUEUE_DEPTH;   // drop OLDEST
    queueCount--;
    queueDropped++;
  }
  int tail = (queueHead + queueCount) % QUEUE_DEPTH;
  queueBuf[tail].weight = w;
  queueBuf[tail].capturedAtMs = millis();
  // Validity is a HISTORICAL FACT about this reading, so it is recorded here rather
  // than read at flush time. Omitting it entirely (the previous behaviour) was right
  // in that stamping CURRENT state on a past record would have been an undetectable
  // lie — but it left every weight taken out of radio range unusable as data, on the
  // one machine where being out of range is normal operation.
  queueBuf[tail].sensorAtCapture = (uint8_t)loopState;
  queueBuf[tail].calDefaultAtCapture = calibrationDefaulted ? 1 : 0;
  queueCount++;
}

static void queueFlush() {
  while (queueCount > 0 && netState == NET_ONLINE) {
    WeightRecord &r = queueBuf[queueHead];
    uint32_t age = (uint32_t)(millis() - r.capturedAtMs);
    char payload[128];
    // sensor and cal are the values AT CAPTURE, carried in the record — not the
    // current ones. That keeps the record honest and still lets a consumer decide
    // whether the reading was valid, which the earlier omit-entirely form could not.
    snprintf(payload, sizeof(payload),
             "{\"w\":%ld,\"age_ms\":%lu,\"backlog\":true,\"sensor\":\"%s\","
             "\"cal\":\"%s\",\"id\":\"%s\"}",
             (long)r.weight, (unsigned long)age,
             loopStateName((LoopState)r.sensorAtCapture),
             r.calDefaultAtCapture ? "DEFAULT" : "ok", CLIENT_ID);
    if (!publishTo("weight", payload)) return;   // stop on first failure; keep the rest
    queueHead = (queueHead + 1) % QUEUE_DEPTH;
    queueCount--;
    WDT.refresh();
  }
}

// ---------------------------------------------------------------------------
// J1939 engine data (issue #24). REPORT.md §7 is the contract:
//
//   plc4/jd5055e/engine    QoS 0, NOT retained, every ENGINE_MS -- the numbers
//   plc4/jd5055e/dtc/<sa>  QoS 1, RETAINED, on change            -- fault state, PER ECU
//
// VALIDITY: absent keys are OMITTED. This is PLC1/PLC3's convention and
// deliberately NOT the one the scale's telemetry uses: J1939 says "not available"
// in the wire value itself, so an absent reading is genuinely absent rather than
// suspect. The consumer handler must be written to the omission contract.
//
// The CAN counters are always present, so a consumer can tell a quiet bus (key
// off: `frames` stops moving) from a board with no CAN at all (`can_up` false)
// from frames being lost (`ring_full` climbing).
// ---------------------------------------------------------------------------
static size_t appendSig(char *buf, size_t cap, size_t n, const char *key,
                        const J1939Sig &s, int decimals, uint32_t now,
                        uint32_t staleMs = J1939_SIG_STALE_MS) {
  if (!j1939SigFresh(s, now, staleMs) || n >= cap) return n;
  int w = snprintf(buf + n, cap - n, "\"%s\":%.*f,", key, decimals, (double)s.v);
  return (w > 0) ? n + (size_t)w : n;
}

// Append "<key>_min","<key>_max","<key>_avg" for one window, or nothing when the
// window saw no valid sample -- the same omission contract as a stale signal.
static size_t appendWin(char *buf, size_t cap, size_t n, const char *key,
                        const J1939Win &w, int decimals) {
  if (w.n == 0 || n >= cap) return n;
  int k = snprintf(buf + n, cap - n, "\"%s_min\":%.*f,\"%s_max\":%.*f,\"%s_avg\":%.*f,",
                   key, decimals, (double)w.mn, key, decimals, (double)w.mx,
                   key, decimals, (double)(w.sum / (float)w.n));
  return (k > 0) ? n + (size_t)k : n;
}

// Called every ENGINE_MS whether or not the broker is up: the min/max/mean
// windows are taken and reset on the same 15 s grid either way, so a published
// window never silently spans an outage. publishTo() refuses when offline.
static void publishEngine() {
  CanSigSnapshot c;
  canSnapshotSigs(c);
  CanWinSnapshot win;
  canTakeWindows(win);
  const uint32_t now = millis();   // read AFTER the snapshot, never the loop's older clock
  if (netState != NET_ONLINE) return;
  char p[704];
  size_t n = 0;
  n += snprintf(p + n, sizeof(p) - n, "{");
  n = appendSig(p, sizeof(p), n, "rpm",        c.rpm,       0, now);
  n = appendSig(p, sizeof(p), n, "load_pct",   c.loadPct,   0, now);
  n = appendSig(p, sizeof(p), n, "torque_pct", c.torquePct, 0, now);
  n = appendSig(p, sizeof(p), n, "coolant_c",  c.coolantC,  1, now);
  n = appendSig(p, sizeof(p), n, "oil_temp_c", c.oilTempC,  1, now);
  n = appendSig(p, sizeof(p), n, "oil_kpa",    c.oilKpa,    0, now);
  n = appendSig(p, sizeof(p), n, "fuel_lph",   c.fuelLph,   2, now);
  n = appendSig(p, sizeof(p), n, "batt_v",     c.battV,     2, now);
  n = appendSig(p, sizeof(p), n, "keysw_v",    c.keyswV,    2, now);
  n = appendSig(p, sizeof(p), n, "hours",      c.hours,     2, now, J1939_HOURS_STALE_MS);
  n = appendSig(p, sizeof(p), n, "fuel_level_pct", c.fuelLevelPct, 1, now);
  n = appendSig(p, sizeof(p), n, "hyd_oil_c",  c.hydOilC,   0, now);
  n = appendWin(p, sizeof(p), n, "rpm",        win.rpm,     0);
  n = appendWin(p, sizeof(p), n, "load_pct",   win.load,    0);
  n = appendWin(p, sizeof(p), n, "torque_pct", win.torque,  0);
  if (n >= sizeof(p)) return;
  snprintf(p + n, sizeof(p) - n,
           "\"frames\":%lu,\"unknown\":%lu,\"ring_full\":%lu,\"ring_high\":%lu,"
           "\"can_err\":%lu,\"can_last\":%d,\"drains\":%lu,\"can_up\":%s,\"filt\":%s,\"id\":\"%s\"}",
           (unsigned long)c.framesSeen, (unsigned long)c.framesUnknown,
           (unsigned long)canRingFull, (unsigned long)canRingHigh,
           (unsigned long)canErrEvents, (int)canLastErr, (unsigned long)canDrains,
           canUp ? "true" : "false",
           canFiltered ? "true" : "false", CLIENT_ID);
  publishTo("engine", p);
}

// One message PER SOURCE ADDRESS, on its own retained topic: the retained message
// is the current state, and four ECUs have four current states. At most ONE per
// loop pass, because each is a QoS 1 publish that spins for its PUBACK.
static void publishNextDtc() {
  if (netState != NET_ONLINE) return;
  if ((long)(millis() - nextDtcMs) < 0) return;
  for (size_t i = 0; i < J1939_MAX_DM1_SRC; i++) {
    J1939Dm1Src s;
    if (!canDm1TakeDirty(i, s)) continue;

    char leaf[16];
    snprintf(leaf, sizeof(leaf), "dtc/%u", (unsigned)s.sa);
    char topic[64];
    if (!buildTopic(leaf, topic, sizeof(topic))) return;

    char p[448];
    size_t n = 0;
    n += snprintf(p + n, sizeof(p) - n,
                  "{\"sa\":%u,\"n\":%u,\"mil\":%u,\"rsl\":%u,\"awl\":%u,\"pl\":%u,\"dtc\":[",
                  (unsigned)s.sa, (unsigned)s.count, (unsigned)s.lampMil,
                  (unsigned)s.lampRed, (unsigned)s.lampAmber, (unsigned)s.lampProtect);
    // `trunc` is true when the ECU sent more real codes than the board keeps
    // (J1939_MAX_DTC) or than fit this buffer, so `n` alone never hides a loss.
    bool trunc = s.trunc;
    for (uint8_t k = 0; k < s.count; k++) {
      if (n >= sizeof(p) - 64) { trunc = true; break; }
      n += snprintf(p + n, sizeof(p) - n, "%s{\"spn\":%lu,\"fmi\":%u,\"oc\":%u}",
                    k ? "," : "", (unsigned long)s.dtc[k].spn,
                    (unsigned)s.dtc[k].fmi, (unsigned)s.dtc[k].oc);
    }
    snprintf(p + n, sizeof(p) - n, "],\"trunc\":%s,\"id\":\"%s\"}",
             trunc ? "true" : "false", CLIENT_ID);

    // Retained, QoS 1, STREAMED with its size -- the same form as publishStatus().
    bool ok = false;
    if (mqttClient.beginMessage(topic, (unsigned long)strlen(p), true, 1)) {
      mqttClient.print(p);
      ok = (mqttClient.endMessage() == 1);
    }
    if (ok) canDm1MarkPublished(i, s.fingerprint);
    nextDtcMs = millis() + (ok ? DTC_GAP_MS : DTC_BACKOFF_MS);
    return;
  }
}

// ---------------------------------------------------------------------------
// The MQTT command path.
//
// The five buttons are gone, so this is the operator's interface. It has to be
// at least as trustworthy as they were, and the failure it must never produce
// is an ack that reports success for something the board refused, or never did.
//
// Contract: commands arrive on plc4/jd5055e/cmd at QoS 1, never retained, as
//   {"id":"7f3a9c21","verb":"+"}      {"id":"7f3a9c22","verb":"sm","arg":"3000"}
// and every one of them is answered on plc4/jd5055e/ack at QoS 0, never
// retained. QoS 0 on the ack is deliberate: a QoS 1 ack spins in endMessage()
// waiting for a PUBACK, which ages the reading past READING_STALE_MS and makes
// the board refuse the operator's NEXT command. A lost ack costs nothing
// because the dedup ring below makes a retry idempotent; a stalled weighing
// loop is not recoverable that cheaply. `verb` is the same string the console accepts and runs through the
// SAME handler, so the two paths cannot diverge -- a property this firmware
// already leans on.
//
// FOUR LIBRARY BEHAVIOURS SHAPE THIS CODE, all read out of ArduinoMqttClient's
// source. Do not simplify around them.
//
//  1. A command must be read in the SAME poll() that opened it. poll() is
//     `while (clientAvailable())`, and entered with _rxState already
//     READ_PUBLISH_PAYLOAD it falls into the READ_PUBLISH_PAYLOAD/DISCARD case
//     (MqttClient.cpp:616-626) and eats the bytes. That is why the payload is
//     taken in the onMessage callback rather than left for the application to
//     collect afterwards.
//  2. THE CALLBACK IS THE ONLY DOOR THAT IS ALWAYS OPEN. parseMessage() cannot
//     run inside the library's own endMessage() PUBACK spin, so a command
//     arriving while a QoS 1 publish waits would be destroyed by behaviour 1
//     before the application ever saw it. onMessage fires from inside poll()
//     itself (MqttClient.cpp:568, :595), ahead of the discard case, so it
//     catches those too. That is what makes delivery meet the contract, and it
//     is why parseMessage() appears nowhere in this file.
//  3. cmdPoll() still sits immediately after netPoll(), so a command captured
//     during netPoll()'s own poll() executes in the SAME pass. Ones captured
//     later -- inside publishStatus()'s 2 s spin, say -- queue in the capture
//     ring and run one per pass after it.
//  4. A zero-length PUBLISH used to wedge the receiver: _rxState stuck at
//     READ_PUBLISH_PAYLOAD with nothing readable and only parseMessage() to
//     escape it. WITH A CALLBACK REGISTERED the library resets _rxState itself
//     (MqttClient.cpp:603-610), so the wedge is closed outright rather than
//     worked around.
//
// WHAT THIS COSTS, stated rather than papered over: reading the payload inside
// the callback makes the library PUBACK a QoS 1 command TWICE -- see the note on
// cmdOnMessage(). And the capture ring is four deep, not unbounded: a fifth
// command arriving before cmdPoll() has drained any of the four is dropped and
// counted, never queued and never overwriting an older one.
// ---------------------------------------------------------------------------
const char CMD_LEAF[] = "cmd";
const char ACK_LEAF[] = "ack";

bool cmdSubscribed = false;
unsigned long cmdReceived  = 0;   // PUBLISHes taken off the command topic
unsigned long cmdExecuted  = 0;   // passed the allowlist and were run
unsigned long cmdDuplicate = 0;   // acked from the ring, deliberately NOT re-run
unsigned long cmdRetained  = 0;   // arrived retained: dropped, counted, never acked
unsigned long cmdRefusedN  = 0;   // a verb this path will not run
unsigned long cmdBad       = 0;   // unparseable, oversized, or zero length
unsigned long ackFailed    = 0;   // the ack publish itself did not complete
unsigned long subFailed    = 0;   // SUBSCRIBE did not come back with a usable SUBACK

// --- the dedup ring --------------------------------------------------------
// It stores the VERDICT, not just the id. A repeat replays what actually
// happened. Storing only the id would let a repeat of a REFUSED command come
// back ok:true -- a success that never happened, on the one interface the
// operator has left, for an action they are about to trust.
struct CmdVerdict {
  char id[41];       // wide enough for a UUID with dashes; longer is REFUSED, never truncated
  char verb[12];
  char detail[21];
  bool ok;
};
const int CMD_RING = 8;
CmdVerdict cmdRing[CMD_RING];
int cmdRingCount = 0;
int cmdRingNext  = 0;

// The command in flight: queued into inputString by cmdPoll(), its verdict read
// and published after the shared handler has run, later in the same loop pass.
bool ackPending  = false;
char ackId[41]   = "";
char ackVerb[12] = "";
// A technician at the enclosure may be halfway through typing when a command
// arrives. inputString is the console's own buffer and the handler clears it,
// so the half-typed line is parked here and put back once the verb has run.
char consoleSaved[65] = "";

// Exact match, never a prefix: `cat` must not become the `c` setter.
//
// `hang` is refused because it deliberately stops the board to prove the
// watchdog, and over the network that is a denial of service against a machine
// holding a load. `read`, `net` and `sum` are refused as redundant -- the phone
// already subscribes to telemetry and status for that data.
static bool cmdAllowed(const char *v) {
  return !strcmp(v, "tare") || !strcmp(v, "+")  || !strcmp(v, "-") ||
         !strcmp(v, "b")    || !strcmp(v, "kv") || !strcmp(v, "c") ||
         !strcmp(v, "w")    || !strcmp(v, "sm");
}

// The three verbs whose argument is concatenated onto the verb in the console
// form the shared handler parses: c1.234, w0.1, sm3000.
static bool cmdTakesArg(const char *v) {
  return !strcmp(v, "c") || !strcmp(v, "w") || !strcmp(v, "sm");
}

// SUCCESS details are this path's own vocabulary and are safe to choose here.
// REFUSAL details are NOT -- see where the ack is built.
static const char *cmdSuccessDetail(const char *v) {
  if (!strcmp(v, "+"))    return "recorded";
  if (!strcmp(v, "-"))    return "removed";
  if (!strcmp(v, "tare")) return "tared";
  if (!strcmp(v, "b"))    return "cleared";
  if (!strcmp(v, "kv"))   return "calibrated";
  return "set";                                  // c, w, sm
}

// EVERY caller-supplied token that gets echoed into an ack must pass this --
// the id AND the verb. They are written into the JSON payload literally rather
// than escaped on the way out, so a quote or a backslash would produce an ack
// that is not valid JSON, unparseable by the consumer waiting to hear whether
// the weight was recorded. Worse for the verb: the refusal verdict is kept in
// the dedup ring, so one malformed verb would have poisoned every replay for
// that id thereafter.
//
// A whitelist, not a blacklist, so a character nobody thought of is refused
// rather than admitted. '+' is in it because '+' is a verb; none of the set
// needs escaping in JSON.
static bool cmdTokenSafe(const char *s) {
  if (!*s) return false;
  for (const char *p = s; *p; p++) {
    if (*p >= 'a' && *p <= 'z') continue;
    if (*p >= 'A' && *p <= 'Z') continue;
    if (*p >= '0' && *p <= '9') continue;
    if (*p == '-' || *p == '_' || *p == '.' || *p == ':' || *p == '+') continue;
    return false;
  }
  return true;
}

// String::toFloat() and toInt() return 0 for an EMPTY or non-numeric string,
// which is indistinguishable from a real zero. That is only survivable while a
// verb's accepted range excludes zero -- and `w`'s does not. `w` with no
// argument at all parsed to 0.0, passed its 0 <= v < 1 test, wrote alpha = 0 to
// EEPROM and acked ok:true "set": a persisted success for a value nobody sent,
// on the filter that sits in the weighing path. `c` and `sm` escaped only
// because their ranges happen to exclude zero, which is luck, not a check.
//
// So the argument is checked for being a number BEFORE its value is judged, for
// all three, and no future range change can reintroduce the hole.
static bool numericArg(const char *s) {
  if (!s || !*s) return false;
  bool digit = false, dot = false;
  for (const char *p = s; *p; p++) {
    if ((*p == '-' || *p == '+') && p == s) continue;   // sign, leading only
    if (*p == '.') { if (dot) return false; dot = true; continue; }
    if (*p < '0' || *p > '9') return false;
    digit = true;
  }
  return digit;                                          // ".", "-" and "+" alone are not numbers
}

// Minimal extractor for a quoted string field in a flat JSON object. NOT a JSON
// parser: escape sequences are not interpreted. That is safe here only because
// the id is character-checked by cmdIdValid() and the verb is matched against a
// fixed allowlist, so a value carrying a backslash fails to match rather than
// being mis-read. It REFUSES rather than truncates on an oversized value: a
// truncated id would collide in the dedup ring and let one command be acked
// with another command's verdict.
static bool jsonField(const char *src, const char *key, char *out, size_t outLen) {
  char needle[10];
  if (snprintf(needle, sizeof(needle), "\"%s\"", key) >= (int)sizeof(needle)) return false;
  const char *p = strstr(src, needle);
  if (!p) return false;
  p += strlen(needle);
  while (*p == ' ' || *p == '\t') p++;
  if (*p != ':') return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return false;
  p++;
  size_t i = 0;
  while (*p && *p != '"') {
    if (i + 1 >= outLen) return false;
    out[i++] = *p++;
  }
  if (*p != '"') return false;          // unterminated
  out[i] = '\0';
  return true;
}

// Returns the ring INDEX, or -1. Deliberately not a CmdVerdict* :
// tools/build-check.sh compiles this file as an .ino, and arduino-cli hoists a
// prototype for every free function above the whole sketch -- including above
// `struct CmdVerdict`. A signature naming a type declared in this file does not
// survive that. Do not "tidy" this back into a pointer.
static int cmdFind(const char *id) {
  for (int i = 0; i < cmdRingCount; i++)
    if (!strcmp(cmdRing[i].id, id)) return i;
  return -1;
}

static void cmdRemember(const char *id, const char *verb, bool ok, const char *detail) {
  CmdVerdict &e = cmdRing[cmdRingNext];
  snprintf(e.id,     sizeof(e.id),     "%s", id);
  snprintf(e.verb,   sizeof(e.verb),   "%s", verb);
  snprintf(e.detail, sizeof(e.detail), "%s", detail);
  e.ok = ok;
  cmdRingNext = (cmdRingNext + 1) % CMD_RING;
  if (cmdRingCount < CMD_RING) cmdRingCount++;
}

// SIZED beginMessage -- the bare form buffers the whole message into a 256-byte
// _txBuffer and truncates in SILENCE, the defect that cost the entire status
// topic. Kept whatever the QoS. The (unsigned long) cast is required: size_t is
// ambiguous between the unsigned-long and the bool overload.
//
// QoS 0, NEVER RETAINED. Changed from QoS 1 by Alex, and the reason is the
// weighing loop: endMessage() at QoS 1 spins on poll() for up to
// setConnectionTimeout() (2 s), and a 2 s pass ages the displayed reading past
// READING_STALE_MS -- so the board shows NO ADC and refuses the operator's NEXT
// command. Tap record, next tap bounces, entirely self-inflicted. A lost ack is
// cheap beside that: the dedup ring makes a retry safe, so the operator taps
// again and the board replays the stored verdict rather than re-executing.
// publishStatus() stays at QoS 1 -- it is retained, late subscribers need it,
// and once a minute is tolerable.
//
// CONSEQUENCE: endMessage() returns 1 unconditionally at QoS 0, and
// WiFiClient::write() reports success as soon as bytes reach the coprocessor. So
// ack_fail counts a bad topic or an offline board, NOT wire-level loss. Do not
// read ack_fail as "acks delivered".
static bool cmdPublishAck(const char *id, const char *verb, bool ok,
                          const char *detail, bool dup, bool withTally) {
  if (netState != NET_ONLINE) return false;
  char topic[64];
  if (!buildTopic(ACK_LEAF, topic, sizeof(topic))) return false;

  char payload[192];
  int n = snprintf(payload, sizeof(payload),
                   "{\"id\":\"%s\",\"verb\":\"%s\",\"ok\":%s,\"detail\":\"%s\"%s",
                   id, verb, ok ? "true" : "false", detail,
                   dup ? ",\"dup\":true" : "");
  if (n < 0 || n >= (int)sizeof(payload)) return false;
  // `n` and `total` ride along on a successful `+`, so the phone gets the
  // running total atomically with the record that moved it.
  if (withTally) {
    int m = snprintf(payload + n, sizeof(payload) - (size_t)n,
                     ",\"n\":%d,\"total\":%ld", rTindex, tallyTotal());
    if (m < 0 || m >= (int)(sizeof(payload) - (size_t)n)) return false;
    n += m;
  }
  if ((size_t)n + 2 > sizeof(payload)) return false;
  payload[n++] = '}';
  payload[n]   = '\0';

  if (!mqttClient.beginMessage(topic, (unsigned long)strlen(payload), false, 0)) return false;
  mqttClient.print(payload);
  return mqttClient.endMessage() == 1;
}

static void cmdAck(const char *id, const char *verb, bool ok,
                   const char *detail, bool dup, bool withTally) {
  if (!cmdPublishAck(id, verb, ok, detail, dup, withTally)) ackFailed++;
}

// --- the capture ring ------------------------------------------------------
// FOUR deep, and the depth is the whole point. One slot was not enough: the gap
// between two cmdPoll() runs is normally a few milliseconds, but publishStatus()
// spins at QoS 1 for up to 2 s once a minute and poll() runs repeatedly inside
// that spin, so two commands 200 ms apart in that window land in different
// poll() calls and the second had nowhere to go. That is 2 s in every 60 -- and
// each loss is a bucket of material the operator has to notice and re-tap.
//
// Written ONLY by cmdOnMessage(), read ONLY by cmdRunCapture(). Deliberately NOT
// volatile: the callback is invoked synchronously from MqttClient::poll() on the
// main thread, not from an interrupt. Marking it volatile would claim a
// concurrency that does not exist and cost code size for nothing.
struct CmdCapture {
  char     body[192];
  size_t   len;
  uint32_t gen;        // netGeneration when it arrived; see cmdRunCapture()
  bool     retained;
  bool     oversize;
};
const int CMD_CAP_DEPTH = 4;
CmdCapture cmdCapRing[CMD_CAP_DEPTH];
int cmdCapHead  = 0;             // the oldest captured command, the next to run
int cmdCapCount = 0;
unsigned long cmdCapFull  = 0;   // arrived with all four slots taken: dropped
unsigned long cmdCapStale = 0;   // captured on a connection that has since gone
unsigned long cmdPartial  = 0;   // payload not fully buffered: dropped, never waited for

// Discard whatever of the open message is ALREADY BUFFERED, and no more. It
// NEVER WAITS: each read is bounded by what WiFiClient reports as already at the
// board, so MqttClient's clientTimedRead() always finds its byte and its 1000 ms
// per-byte timeout is never reached. Any tail that has not arrived is discarded
// by the library's own poll(), whose loop is `while (clientAvailable())` and
// cannot block either.
static void cmdDrainBuffered() {
  uint8_t scratch[32];
  for (;;) {
    const int ready = wifiClient.available();   // bytes in the local rx ring
    const int want  = mqttClient.available();   // bytes this message still owes
    if (ready <= 0 || want <= 0) return;
    size_t n = (size_t)((ready < want) ? ready : want);
    if (n > sizeof(scratch)) n = sizeof(scratch);
    if (mqttClient.read(scratch, n) <= 0) return;
  }
}

// CAPTURE ONLY. This is the whole of the callback's job: take the payload,
// record its length, retain flag and connection generation, advance the ring,
// return.
//
// It parses nothing, allowlists nothing, dedups nothing, executes nothing,
// publishes nothing and WAITS ON NOTHING. It cannot: it runs inside
// MqttClient::poll(), which is itself called from endMessage()'s PUBACK spin,
// so anything reentrant here would be publishing from inside a publish. All of
// that work stays in cmdRunCapture(), on the main line of loop().
//
// IT MUST NOT BLOCK, AND THAT IS THE HARD CONSTRAINT, NOT A PREFERENCE. This
// function runs on the weighing path, and the design spine at the top of this
// file is that the weighing path may never wait on the network. An earlier
// revision read the payload with a bare MqttClient::read(), which goes through
// clientTimedRead() and waits up to 1000 ms FOR EACH BYTE that has not arrived
// -- so a slow trickle was bounded not by one second but by 1000 ms times the
// payload length. That put a network stall in front of the scale through a
// mechanism that did not exist before the command path was added.
//
// The fix is the gate below: read ONLY what has ALREADY ARRIVED AT THE BOARD,
// and if the whole command is not there yet, do not wait for the rest.
//
// Precisely what the gate buys, because the weaker claim is the true one:
// WiFiClient::available() returns its local rx ring count, or -- when that ring
// is empty -- asks the modem how many bytes it is holding (WiFiClient.cpp,
// available/read_if_needed). So once it reports >= size, every byte is either in
// the local ring or sitting at the coprocessor with the modem saying so. A read
// therefore costs at most ONE local bus fetch plus ring copies, which is exactly
// what poll() already does on every pass. It is NOT free -- but no read can
// reach clientTimedRead()'s 1000 ms wait, because that wait only happens when
// clientAvailable() is zero, and it cannot be. The unbounded 1000 ms x payload
// stall is structurally gone rather than merely made unlikely.
//
// WHY IT MUST STILL READ THE PAYLOAD HERE rather than leave it for cmdPoll():
// the only alternative is to return with _rxState still at READ_PUBLISH_PAYLOAD,
// and the very next poll() -- in this same spin -- falls into the
// READ_PUBLISH_PAYLOAD/DISCARD case (MqttClient.cpp:616-626) and eats it.
// Reading it here is the entire reason the callback exists. It is also the
// library's own documented pattern: examples/WiFiAdvancedCallback subscribes at
// QoS 1 and reads inside the callback.
//
// WHAT THAT COSTS, and it is visible on the wire: reading a payload to
// completion makes MqttClient::read() call ackRxMessage() (MqttClient.cpp:735-739),
// and poll() then sees _rxLength == 0 on return and calls ackRxMessage() AGAIN
// (MqttClient.cpp:603-606). Every QoS 1 command therefore produces TWO PUBACKs
// carrying the same packet id. It is inherent to the library's callback pattern
// and cannot be avoided from this file. The connection is MQTT 3.1.1 (protocol
// level 0x04, MqttClient.cpp:906), which does not make an unexpected PUBACK a
// protocol error the way 5.0 does. NOT VERIFIED AGAINST A BROKER from here.
static void cmdOnMessage(int size) {
  // The retain flag must be read BEFORE the payload: consuming the last byte
  // returns _rxState to READ_TYPE and messageRetain() then answers -1.
  const bool retained = mqttClient.messageRetain() > 0;

  cmdReceived++;

  const size_t need  = (size > 0) ? (size_t)size : 0;
  const int    avail = wifiClient.available();
  const size_t have  = (avail > 0) ? (size_t)avail : 0;

  if (need > have) {
    // The whole command is not here yet and this function will not stand still
    // for the rest of it. DROPPING IS SAFE: the sender retries and the dedup
    // ring makes the retry idempotent. A stalled weighing loop is not safe.
    //
    // Take what did arrive so the receiver is not left owing a large _rxLength;
    // poll() discards the tail as it lands, without blocking.
    cmdPartial++;
    cmdDrainBuffered();
    return;
  }

  // All four slots are taken. Drop the NEWCOMER, never an older command: an
  // operator who taps four times deserves the first four, not the last four,
  // and a dropped command they can retry beats a queue that silently reorders
  // what they asked for.
  if (cmdCapCount >= CMD_CAP_DEPTH) {
    cmdCapFull++;
    cmdDrainBuffered();
    return;
  }

  CmdCapture &c = cmdCapRing[(cmdCapHead + cmdCapCount) % CMD_CAP_DEPTH];

  size_t want = need;
  c.oversize = false;
  if (want > sizeof(c.body) - 1) {
    want = sizeof(c.body) - 1;
    c.oversize = true;
  }

  int got = 0;
  if (want > 0) {
    got = mqttClient.read((uint8_t *)c.body, want);   // cannot wait: see the gate above
    if (got < 0) got = 0;
  }
  c.body[got] = '\0';
  c.len       = (size_t)got;
  c.retained  = retained;
  // Which connection this arrived on. A command captured before an outage and
  // run after the reconnect would act on a machine state the operator never saw
  // -- see cmdRunCapture(). netState alone cannot catch it: an MQTT-only drop
  // can be detected and repaired inside a single loop pass.
  c.gen       = netGeneration;

  // Drain the oversized remainder, if any. Bounded to what is buffered, which
  // the gate above has already established is the whole payload.
  cmdDrainBuffered();

  // Published LAST. Until this line the slot is not part of the ring, so a
  // half-written capture can never be seen as a command.
  cmdCapCount++;
}

// Run one captured command. Takes the ring INDEX, not a CmdCapture& --
// tools/build-check.sh compiles this file as an .ino and arduino-cli hoists a
// prototype for every free function above `struct CmdCapture`, so a signature
// naming the type does not survive. Same reason as cmdFind(); do not "tidy" it.
//
// The caller releases the slot AFTER this returns, on every path. That ordering
// is load-bearing: while the entry is in hand cmdCapCount still counts it, so
// the callback's write slot -- (cmdCapHead + cmdCapCount) % CMD_CAP_DEPTH --
// can never be this one, and a command cannot be overwritten mid-parse.
static void cmdRunCapture(int slot) {
  const CmdCapture &cap = cmdCapRing[slot];

  // A command captured before an outage and executed after the reconnect would
  // act on a machine state the operator never saw -- a plausible wrong answer,
  // which is this machine's characteristic failure. netState alone CANNOT catch
  // that: an MQTT-only drop is detected and repaired inside a single loop pass,
  // so netState reads NET_ONLINE at both ends of it and the capture sails
  // straight through. The generation stamp is what makes this gate mean what it
  // says -- "captured on the connection that is still up". Discard and count;
  // the phone gets no ack, and a retry with the same id is safe.
  if (netState != NET_ONLINE || !cmdSubscribed || cap.gen != netGeneration) {
    cmdCapStale++;
    return;
  }

  const char *body = cap.body;

  // A retained command would re-execute on every reconnect -- re-taring, or
  // clearing the day's tally, each time the board reassociated. It is dropped
  // and COUNTED, never acked: an ack would publish a response to something
  // nobody just sent. `net` reports the count so it is diagnosable.
  if (cap.retained) { cmdRetained++; return; }

  // Nothing to parse. A zero-length PUBLISH lands here; with the callback
  // registered it no longer wedges the receiver (MqttClient.cpp:603-610), it is
  // simply an empty command.
  if (cap.len == 0) { cmdBad++; return; }

  // No id we can trust, so there is nothing to ack against. Counted, silent.
  if (cap.oversize) { cmdBad++; return; }

  char id[41];
  if (!jsonField(body, "id", id, sizeof(id)) || !cmdTokenSafe(id)) { cmdBad++; return; }

  // Dedup BEFORE the verb is even looked at, and replay the STORED verb as well
  // as the stored verdict: if an id is ever reused for a different command, the
  // ack describes what actually ran under that id rather than what the repeat
  // claimed to be. A repeat is acked and deliberately NOT executed.
  const int seen = cmdFind(id);
  if (seen >= 0) {
    cmdDuplicate++;
    cmdAck(cmdRing[seen].id, cmdRing[seen].verb, cmdRing[seen].ok,
           cmdRing[seen].detail, true, false);
    // A resend of an id the board minted (NO ANSWER -> OK, design §4.5) is answered here
    // from the ring, not re-run. The screen takes the stored verdict. The weight shown is
    // the tally's last entry, which is the replayed record's unless more were taken since.
    if (cmdIdIsBoards(cmdRing[seen].id))
      hmi.onVerdict(cmdRing[seen].id, cmdRing[seen].ok, cmdRing[seen].detail,
                    rTindex > 0 ? (long)rT[rTindex - 1] : 0L, hmiContext());
    return;
  }

  char verb[12];
  verb[0] = '\0';
  const char *why = nullptr;
  if (!jsonField(body, "verb", verb, sizeof(verb))) {
    why = "BAD COMMAND";
  } else if (!cmdTokenSafe(verb)) {
    // NEVER echo caller-controlled bytes into the ack. A verb carrying a quote
    // or a backslash would break the JSON, and the dedup ring would then replay
    // that broken ack for this id for as long as it is remembered. Blank it and
    // refuse: an empty verb cannot be mistaken for a real one, and there is
    // nothing safe to quote back.
    verb[0] = '\0';
    why = "BAD COMMAND";
  } else if (!cmdAllowed(verb)) {
    why = "NOT ALLOWED";
  }
  if (why) {
    // These two strings are INVENTED, and they are ACK-ONLY: neither ever
    // reaches the LCD or the console, so they cannot make the phone and the
    // display disagree about why something was refused. There is no firmware
    // string to reuse here, because until now nothing could ask the board to
    // run a verb it declines to run.
    cmdRefusedN++;
    cmdRemember(id, verb, false, why);
    cmdAck(id, verb, false, why, false, false);
    return;
  }

  char line[80];
  if (cmdTakesArg(verb)) {
    char arg[24];
    if (!jsonField(body, "arg", arg, sizeof(arg))) arg[0] = '\0';
    // A missing or unusable argument is NOT judged here. It goes to the handler,
    // which already validates it and now records its own refusal -- so the phone
    // is told "sm REJECTED" in the firmware's own words rather than in this
    // path's guess at them.
    snprintf(line, sizeof(line), "%s%s", verb, arg);
  } else {
    snprintf(line, sizeof(line), "%s", verb);
  }

  // Queue it for the CONSOLE'S OWN handler, which runs later in this same loop
  // pass. One execution path for the console and the phone is what stops the
  // two diverging, and it is already load-bearing in this firmware.
  //
  // stringComplete is false here by construction: serialEvent() is the only
  // other thing that sets it and it runs AFTER this function, and the handler
  // that consumes it clears it unconditionally at the end of every pass.
  snprintf(consoleSaved, sizeof(consoleSaved), "%s", inputString.c_str());
  inputString = line;
  stringComplete = true;

  // This command owns the next verdict, not the last one.
  verbRefused = false;
  verbRefusedReason[0] = '\0';

  snprintf(ackId,   sizeof(ackId),   "%s", id);
  snprintf(ackVerb, sizeof(ackVerb), "%s", verb);
  ackPending = true;
  cmdExecuted++;
}

// Run at most ONE captured command per pass, oldest first. One per pass, not the
// whole ring, because a command's verdict is published by the ack block at the
// end of loop() and there is exactly one ackPending slot -- draining four in a
// pass would need four pending acks. At a few milliseconds a pass a full ring
// clears in well under a tenth of a second, so the queue is not a delay the
// operator can see.
//
// Called immediately after netPoll(), so a command captured during netPoll()'s
// own poll() runs in the SAME pass.
//
// The slot is released HERE, after cmdRunCapture() has returned on whatever path
// it took. Releasing it earlier would hand the callback the very slot being
// parsed -- see the note on cmdRunCapture().
static void cmdPoll() {
  if (cmdCapCount == 0) return;
  cmdRunCapture(cmdCapHead);
  cmdCapHead = (cmdCapHead + 1) % CMD_CAP_DEPTH;
  cmdCapCount--;
}

// ---------------------------------------------------------------------------
// Network state machine — called from loop(), never loops internally.
//
// HONEST LIMITATION: WiFi.begin() and MqttClient::connect() are blocking calls
// in WiFiS3/ArduinoMqttClient; there is no non-blocking association API. What
// this code guarantees is that they are ATTEMPTED AT A BOUNDED RATE AND NEVER
// LOOPED ON — one attempt per NET_RETRY_MS, after which loop() continues and the
// scale keeps weighing. Each attempt is bounded to WIFI_CONNECT_TIMEOUT_MS +
// MQTT_CONNECT_TIMEOUT_MS, both of which sit under the watchdog period.
//
// Because the board is on the broker's own subnet (FOXTROTBRAVO is the
// 192.168.10.0 LAN), there are exactly TWO failure states to distinguish: out of
// WiFi range, and broker down. No route-change, gateway or inter-subnet case
// exists. Do not add defensive handling for hops that are not there.
// ---------------------------------------------------------------------------
static void netPoll() {
  unsigned long now = millis();

  if (netState == NET_ONLINE) {
    // R2: check the association FIRST and unconditionally. This used to sit inside
    // the !connected() branch, so a silent association loss left netState ONLINE for
    // up to ~120 s while mqttClient.connected() still returned true. In that window
    // publishTo() returned true — WiFiClient::write() reports success once bytes reach
    // the coprocessor, and QoS 0 waits for nothing — so the `+` handler took the
    // "published live" path and never queued. Every weight recorded was lost silently:
    // no queue entry, no dropped count, nothing on the LCD, and `net` still saying up.
    if (WiFi.status() != WL_CONNECTED) {
      netState = NET_OFFLINE;
      cmdSubscribed = false;      // the session went with the link
      return;
    }
    mqttClient.poll();
    if (!mqttClient.connected()) {
      netState = NET_WIFI_UP;
      cmdSubscribed = false;
    }
    return;
  }

  if (now - lastNetAttemptMs < NET_RETRY_MS) return;
  lastNetAttemptMs = now;

  WDT.refresh();   // the calls below block; give them a full watchdog period

  // STAGE 1 — association. This RETURNS afterwards even on success, so the broker
  // connect below always runs in a LATER loop() pass with a fresh heartbeat.
  //
  // Why that matters: loopHeartbeatMs is stamped once per loop() pass, so everything
  // netPoll does accumulates against a single heartbeat. Running both halves in one
  // pass gives a worst case of 23000 (WiFi.begin) + 10000 (connect) = 33000 ms against
  // a 30000 ms tolerance — the ISR stops refreshing and the watchdog bites.
  //
  // That is the reset observed at FIRST ASSOCIATION and nowhere else: it needs both
  // halves to run slow in the same pass, which only happens when the board has to
  // associate AND then reach a broker. Split, each pass is bounded by the larger half
  // alone and the margin comes back.
  if (WiFi.status() != WL_CONNECTED) {
    // ONE network per pass, never the whole list. WiFi.begin() blocks for up to
    // 23 s against a 30 s heartbeat tolerance, so two attempts in a single pass
    // would be ~46 s and reset the board -- the same arithmetic that forced the
    // split between association and the broker connect below.
    const WifiNetwork &wn = WIFI_NETWORKS[wifiIndex];
    Console.print("wifi: trying ");
    Console.println(wn.ssid);      // SSIDs are broadcast. Passphrases are never printed.
    WiFi.begin(wn.ssid, wn.pass);  // single attempt, never a while loop (was C5)
    WDT.refresh();
    if (WiFi.status() == WL_CONNECTED) {
      netState = NET_WIFI_UP;
      lastNetAttemptMs = now - NET_RETRY_MS + STAGE_GAP_MS;  // try the broker shortly, not in 30 s
    } else {
      netState = NET_OFFLINE;
      // Next pass tries the next network down the priority list, wrapping at the
      // end. Each extra network costs one more NET_RETRY_MS before the list
      // wraps, never another blocking call inside this pass.
      if (WIFI_NETWORK_COUNT > 1) wifiIndex = (wifiIndex + 1) % WIFI_NETWORK_COUNT;
    }
    return;
  }

  // STAGE 2 — broker.
  netState = NET_WIFI_UP;

  mqttClient.setId(CLIENT_ID);                          // was unset: C7
  mqttClient.setUsernamePassword(mqttUser, mqttPass);

  // The last will MUST be armed before connect: the broker records it at CONNECT and
  // publishes it if this client ever drops without a clean DISCONNECT. That is what
  // makes an absent tractor distinguishable from a wedged one — without it, silence
  // has no meaning and a dead board looks exactly like one out of radio range.
  //
  // TWO bugs lived in these six lines, both worth naming:
  //
  //  1. beginWill() returns 0 on SUCCESS (MqttClient.cpp:294). Testing it as a
  //     boolean inverted the check, so on success the payload was never written and
  //     endWill() never ran — leaving the CONNECT packet declaring a will flag over a
  //     zero-length will buffer. The broker dropped it without a CONNACK and the board
  //     reported mqtt_err = -1, CONNECTION_TIMEOUT, on every attempt.
  //  2. It was armed on EVERY retry. The library keeps _willBuffer and _willFlags
  //     across connects (MqttClient.cpp:916 only reads them), so once is enough.
  //
  // The first is the same misreading as publishWeight()'s void-in-a-boolean-test
  // earlier today: assuming a return contract instead of reading it.
  if (!willArmed) {
    char willTopic[64];
    if (buildTopic("status", willTopic, sizeof(willTopic))
        && mqttClient.beginWill(willTopic, true, 0) == 0) {   // retained, QoS 0
      mqttClient.print("{\"up\":false,\"id\":\"");
      mqttClient.print(CLIENT_ID);
      mqttClient.print("\"}");
      mqttClient.endWill();
      willArmed = true;
    }
  }

  // R1: this refresh was missing when WiFi was ALREADY associated — the common case.
  // The last refresh was then at the top of this function, and the connect below can
  // occupy most of the watchdog period on its own. Without it, WiFi up plus an
  // unreachable broker produced a 30-second reset cycle, and every reset wiped the
  // backlog AND the operator's tally, in exactly the case the queue exists for.
  WDT.refresh();
  netAttempts++;
  if (!mqttClient.connect(broker, port)) {              // no while(1) on failure: C5
    lastMqttError = mqttClient.connectError();
    WDT.refresh();
    return;
  }
  lastMqttError = 0;
  WDT.refresh();
  netState = NET_ONLINE;
  // Before the subscribe below, because the callback can fire inside its SUBACK
  // spin and anything captured there belongs to THIS connection.
  netGeneration++;

  // Subscribe to the command topic. The library's session is CLEAN by default,
  // so the subscription does not survive a disconnect and must be re-made on
  // every connect -- not once at boot.
  //
  // subscribe() blocks waiting for a SUBACK, bounded by setConnectionTimeout()
  // (MQTT_CONNECT_TIMEOUT_MS, 2 s), and calls stop() if it never arrives. It
  // returns 0 WITHOUT stopping when the broker answers with a failure code,
  // which is what an ACL refusal looks like.
  //
  // DELIBERATELY NOT FATAL. A broker that refuses the subscription must not take
  // this machine off the air: telemetry, status and the backlog flush are the
  // field-proven path and none of them depend on the command topic. cmdPoll()
  // gates on cmdSubscribed instead, and `net` reports cmd_sub so a board that
  // cannot be commanded says so rather than going quietly deaf.
  cmdSubscribed = false;
  {
    char cmdTopic[64];
    if (buildTopic(CMD_LEAF, cmdTopic, sizeof(cmdTopic)))
      cmdSubscribed = (mqttClient.subscribe(cmdTopic, 1) != 0);
  }
  if (!cmdSubscribed) subFailed++;
  WDT.refresh();

  publishStatus(now);      // say we are here, and on what terms, before any data
  queueFlush();                                          // was C6: no reconnect path at all
  canDm1ForceDirty();      // re-assert every ECU's retained fault state on this connection
}

// Probe every 7-bit address on Wire1 and report what answers. Bring-up only —
// it runs once in setup() and on the `i2c` verb, never in loop().
//
// Worth its ~200 bytes because "NO ADC" alone cannot tell three very different
// faults apart: nothing on the bus at all (cable or power), a device present at
// an address the driver does not expect, or a device answering but misbehaving.
// The MCP3421 in particular ships as A0-A7 with addresses 0x68-0x6F, so a part
// from a different reel answers at an address this firmware never asks about.
// Expected here: 0x27 (LCD2004) and 0x68 (MCP3421A0) behind the mux on Wire1, and
// 0x24 (PN532) alone on Wire, the header bus (FW-7).
// The mux address, the channel map and the select helper live in
// include/i2c_mux.h, beside the drivers that depend on them.
static int i2cScanRange(uint8_t skip) {
  int found = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    if (a == skip) continue;
    Wire1.beginTransmission(a);
    if (Wire1.endTransmission() == 0) {
      Console.print(" 0x");
      if (a < 0x10) Console.print("0");
      Console.print(a, HEX);
      found++;
    }
  }
  return found;
}

static void i2cScan() {
  Wire1.begin();
  // Close every channel first. A mux latches its last selection, so a trunk
  // scan taken while ch7 is still open reports the LCD as if it sat on the
  // trunk -- true of the wire, misleading about the topology.
  Wire1.beginTransmission(I2C_MUX_ADDR);
  Wire1.write((uint8_t)0x00);
  Wire1.endTransmission();
  i2cMuxInvalidate();
  Console.print("i2c Wire1:");
  int trunk = i2cScanRange(0xFF);
  if (!trunk) Console.print(" NOTHING FOUND");
  Console.println();

  // If a mux is present, walk its eight channels. Without this the trunk scan
  // reports only the mux and every device behind it looks missing.
  Wire1.beginTransmission(I2C_MUX_ADDR);
  if (Wire1.endTransmission() != 0) return;

  Console.println("i2c mux 0x70 present, scanning channels:");
  for (uint8_t ch = 0; ch < 8; ch++) {
    if (!i2cMuxSelect(ch)) continue;
    Console.print("  ch");
    Console.print(ch);
    Console.print(":");
    if (!i2cScanRange(I2C_MUX_ADDR)) Console.print(" -");
    Console.println();
  }
  // This walked the mux behind the drivers' backs, so the cached channel no
  // longer describes the hardware. Clear it or the next select is skipped as a
  // no-op and that transfer lands on channel 7, whatever it was meant for.
  i2cMuxInvalidate();

  i2cScanWire();
}

// Whether Wire's controller initialised. TwoWire::init_ok is private, and when it is false
// write_to() returns END_TX_OK for ANY address without touching the bus (Wire.cpp:497-530,
// rv is initialised to END_TX_OK and only set inside `if (init_ok)`). Nothing on this bus
// answers at both 0x08 (reserved) and 0x77, so both "answering" means the controller is not
// running -- an IRQ slot it did not get, say. Inference from the core's source, not measured.
static bool wireLooksInitialised() {
  Wire.beginTransmission(0x08);
  const bool a = Wire.endTransmission() == 0;
  Wire.beginTransmission(0x77);
  const bool b = Wire.endTransmission() == 0;
  return !(a && b);
}

// The header bus, Wire (A4/A5): the PN532 at 0x24 and nothing else (design §2.2, FW-7).
// Only once setup() has started it, after the heartbeat and CAN timers. A probe can land
// between a command and its answer, so the reader driver starts over afterwards.
static void i2cScanWire() {
  if (!wireUp) return;
  if (!wireOk) { Console.println("i2c Wire: NOT INITIALISED (no IRQ slot?)"); return; }
  Console.print("i2c Wire:");
  int hdr = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Console.print(" 0x");
      if (a < 0x10) Console.print("0");
      Console.print(a, HEX);
      hdr++;
    }
  }
  if (!hdr) Console.print(" NOTHING FOUND");
  Console.println();
  nfc.restart(millis());
}

void setup() {
  delay(2000);
  Serial1.begin(9600);   // HC-05 field console
  Serial.begin(115200);  // USB console for bring-up and service. No wait-for-host.
  inputString.reserve(200);
  // Which source made this image. "UNKNOWN" means provenance was never
  // captured; a trailing +dirty means the image contains work that is in no
  // commit, so the SHA beside it does not fully describe it.
  Console.print("build = "); Console.println(BUILD_PROVENANCE);
  i2cScan();             // say what is actually on the bus before blaming the ADC
  if (!mcpBegin()) Console.println("MCP3421 NOT FOUND on Wire1");
  // Diagnostics only. Its absence is reported once and never again: a missing
  // power monitor must not look like a scale fault, and must not add a line to
  // every render.
  if (!inaBegin()) Console.println("INA219 not found on mux ch2 (power monitor, optional)");
  { int c = 0; if (mcpRead(c)) smoothedValue = c; else smoothedValue = COUNTS_ZERO; }

  // -------------------------------------------------------------------------
  // EEPROM restore, with a signature.
  //
  // Range checks alone are NOT enough, and this bit me on the first flash: the
  // board came back with tareValue = 16256, calFactor ~0 and alpha 0, all of
  // which passed a plain range test and none of which were this firmware's. They
  // were leftovers from an earlier build with a different layout — 16256 is
  // 0x3F80, the top half of a float 1.0f landing at the wrong offset.
  //
  // Garbage that happens to fall in range is indistinguishable from a real stored
  // calibration, so the signature is what makes the distinction possible at all.
  // Without it, every board flashed over older firmware silently inherits
  // nonsense and reports `sensor = ok` while doing it.
  // -------------------------------------------------------------------------
  uint32_t sig = 0;
  EEPROM.get(addr+12, sig);
  bool eepromValid = (sig == EEPROM_SIGNATURE);

  if (eepromValid) {
    EEPROM.get(addr, calFactor);
    EEPROM.get(addr+4, tareValue);
    EEPROM.get(addr+8, alpha);
  }

  // Second line of defence: reject anything physically impossible even if the
  // signature matched. A zero outside the transducer's own operating band cannot
  // be a zero, whatever the EEPROM says.
  // Only calFactor and tareValue decide whether the CALIBRATION is trustworthy.
  //
  // alpha used to be in this test, and it caused a real fault in service: on a board
  // where `w` had never been written, alpha read back NaN, `defaulted` went true, and the
  // board announced CAL NOT SET with a perfectly good factor and zero still loaded. The
  // operator was told to recalibrate a scale that did not need it — and did. A filter
  // smoothing constant must never be able to condemn a calibration.
  bool defaulted = !eepromValid;
  if (isnan(calFactor) || calFactor <= 0.0001f || calFactor > 1000.0f) { calFactor = 1.0f; defaulted = true; }
  if (tareValue < COUNTS_DEAD || tareValue > COUNTS_FS) { tareValue = COUNTS_ZERO; defaulted = true; }
  // alpha defaults silently: harmless on its own, and not evidence about the calibration.
  if (isnan(alpha) || alpha <= 0.0f || alpha >= 1.0f)                  { alpha = 0.1f; }

  if (defaulted) {
    // Say so loudly. A silently defaulted calibration is the same failure the
    // stale-zero problem describes: the machine works, and reads wrong.
    calibrationDefaulted = true;
  }

  lcdBegin();
  lcdLine(0, "Loader Scale");
  lcdLine(1, "starting");

  tallyRestore();
  captureResetCause();
  Console.print("tally restored = ");
  Console.println(rTindex);
  Console.print("JD5055e OK  reset: ");
  Console.println(resetCause);
  printDebug();

  // Both timeouts must be set before the watchdog is armed — see the comment on
  // WDT_TIMEOUT_MS. The library defaults (10 s association) exceed the watchdog
  // ceiling and would reset the board on every failed connect.
  WiFi.setTimeout(WIFI_CONNECT_TIMEOUT_MS);
  // DO NOT call wifiClient.setConnectionTimeout() here. It looks like the obvious
  // way to bound the TCP connect, and an earlier revision of this file did exactly
  // that — but it SELECTS A DIFFERENT AT COMMAND, and that command does not work
  // with a dotted-quad host on this core:
  //
  //   WiFiClient.cpp:53  _connectionTimeout == 0  ->  +CLIENTCONNECTNAME  (works)
  //                      _connectionTimeout != 0  ->  +CLIENTCONNECT      (fails here)
  //
  // Measured: with it set, mqttClient.connect() returned -2 (MQTT_CONNECTION_REFUSED)
  // on every attempt and tcpdump on the broker saw ZERO packets — the connect failed
  // at the coprocessor before any TCP was attempted. The board could not reach any
  // broker at all.
  //
  // Bounding it is no longer needed anyway: the heartbeat watchdog below is what
  // covers blocking network calls. That is the whole reason it exists.
  mqttClient.setConnectionTimeout(MQTT_CONNECT_TIMEOUT_MS);

  // Registered ONCE, here, and it survives every reconnect -- the library only
  // holds a function pointer. It is what makes a command arriving inside a QoS 1
  // publish's PUBACK spin survive instead of being eaten; see cmdOnMessage().
  mqttClient.onMessage(cmdOnMessage);

  // F1: the watchdog is armed BEFORE any networking exists, so that a hang in
  // the reconnect path has a floor under it. This is the whole reason the
  // watchdog was sequenced first in the work order.
  loopHeartbeatMs = millis();
  WDT.begin(WDT_TIMEOUT_MS);

  // Start the heartbeat refresher at 2 Hz. If no timer is free the board still runs,
  // but it is back to resetting during network calls — so say so rather than fail quiet.
  uint8_t timerType;
  int8_t timerCh = FspTimer::get_available_timer(timerType);
  bool hb = false;   // copied to hbTimerOn below
  if (timerCh >= 0) {
    hb = wdtTimer.begin(TIMER_MODE_PERIODIC, timerType, (uint8_t)timerCh,
                        2.0f, 0.0f, wdtHeartbeatISR)
         && wdtTimer.setup_overflow_irq()
         && wdtTimer.open()
         && wdtTimer.start();
  }
  Console.print("heartbeat wdt = ");
  Console.println(hb ? "on" : "UNAVAILABLE - resets during network calls");
  hbTimerOn = hb;

  // CAN AFTER the heartbeat timer, so the watchdog gets the first free timer and
  // CAN can never be the reason it has none. Filter OFF at boot: it has never run
  // on silicon (see J1939_FILTER_MASK); `canf` turns it on at the machine.
  canUp = canBegin(false);
  Console.print("can = ");
  Console.print(canUp ? "up 250k D10/D13" : "BEGIN FAILED");
  Console.print(", drain ");
  Console.println(canTimerOn ? "timer 200 Hz" : "loop (NO TIMER - frames lost during network calls)");

  // FW-1: Wire, the reader's bus. Begun HERE, after the heartbeat and CAN timers, never in
  // the boot i2cScan() above: Wire takes interrupt slots, and a shortage must never cost the
  // watchdog its timer (review item 4).
  Wire.begin();
  wireUp = true;
  wireOk = wireLooksInitialised();
  i2cScanWire();
  Console.print("nfc = PN532 on Wire 0x24");
  Console.println(wireOk ? "" : " -- Wire NOT INITIALISED, reader off");

  // FW-2: the lid buttons. Their timer comes AFTER the heartbeat's and CAN's, so neither
  // can lose its timer to them. Whether a third is free is for the boot banner to say
  // (design §3.5); the core has 8 GPT and 2 AGT channels (pins_arduino.h:146-149).
  for (uint8_t i = 0; i < BTN_COUNT; i++) pinMode(BTN_PINS[i], INPUT_PULLUP);
  {
    uint8_t bt;
    const int8_t bch = FspTimer::get_available_timer(bt);
    if (bch >= 0) {
      btnTimerOn = btnTimer.begin(TIMER_MODE_PERIODIC, bt, (uint8_t)bch, 200.0f, 0.0f, btnSampleISR)
                   && btnTimer.setup_overflow_irq()
                   && btnTimer.open()
                   && btnTimer.start();
    }
  }
  Console.print("buttons = D2-D5, ");
  Console.println(btnTimerOn ? "timer 200 Hz" : "loop (NO TIMER - presses lost during network calls)");

  // FW-4, FW-5, FW-3.
  const bool urlOk = panel.setUrl(PLC4_PANEL_URL);
  Console.print("panel = ");
  Console.println(urlOk ? PLC4_PANEL_URL : "BAD PLC4_PANEL_URL - buttons cannot record");
  cmdIds.load = cmdIdLoad;
  cmdIds.save = cmdIdSave;
  hmi.minter = &cmdIds;
  hmi.full = (PLC4_HMI_FULL != 0);
  hmi.mcFeature = (PLC4_HMI_MC != 0);
  Console.print("hmi = ");
  Console.println(hmi.full ? (hmi.mcFeature ? "full set + mc" : "full set") : "part (a): scan and record");

  // Seeds the freshness clock so the first READING_STALE_MS after boot is not
  // reported stale before any sample cycle has completed.
  lastReadingMs = millis();
}

// ---------------------------------------------------------------------------
// Tally persistence.
//
// Added because the LCD shield's RESET button sat in the same row as the five
// operator keys — confirmed 2026-09-01 when pressing along the row reset the board
// and wiped the tally (reset cause reported "none-flagged", i.e. the RES# pin, which
// is what an external button does). The keys have since gone; the RESET button, the
// watchdog and a power cycle have not, and any of them still empties RAM.
//
// EEPROM.put() on this core is a virtual EEPROM over data flash: it skips unchanged
// BYTES but still erases and rewrites the whole block per CALL. So the cost is
// counted in calls, not bytes, and these helpers write the least that is sufficient:
//   +  one entry and the index      -  the index alone
//   b  the index alone (set to 0; the stale entries above it are never read,
//      because `sum` loops i < rTindex)
// At ~50 buckets a day that is well inside the 100,000-cycle minimum.
// ---------------------------------------------------------------------------
// The signature is written ONCE, not on every action. EEPROM.put() erases its block
// whether or not the value changed, so rewriting a constant every bucket would have
// tripled the wear for nothing.
bool tallySigWritten = false;
// Write the WHOLE calibration block, then the signature. Previously tare and kv wrote
// their own field plus the signature, while alpha at addr+8 was written only by the `w`
// verb — so on a board where `w` had never been used, the signature vouched for a slot
// that had never been written. It read back as 0xFFFFFFFF, which is NaN as a float.
//
// Four puts per calibration change instead of two. Calibration happens a few times a day
// at most, so the wear is irrelevant next to the correctness.
static void saveCalibration() {
  EEPROM.put(addr,     calFactor);
  EEPROM.put(addr + 4, tareValue);
  EEPROM.put(addr + 8, alpha);
  EEPROM.put(addr + 12, EEPROM_SIGNATURE);
}

static void tallySaveIndex() {
  EEPROM.put(addr + 20, rTindex);
  if (!tallySigWritten) {
    EEPROM.put(addr + 16, TALLY_SIGNATURE);
    tallySigWritten = true;
  }
}
static void tallySaveEntry(int i) {
  if (i < 0 || i >= rTsize) return;
  EEPROM.put(addr + 24 + (i * (int)sizeof(int)), rT[i]);
}
static void tallyRestore() {
  uint32_t sig = 0;
  EEPROM.get(addr + 16, sig);
  if (sig != TALLY_SIGNATURE) return;          // nothing stored, or a different layout
  int idx = 0;
  EEPROM.get(addr + 20, idx);
  if (idx < 0 || idx > rTsize) return;         // refuse an impossible index rather than trust it
  for (int i = 0; i < idx; i++) EEPROM.get(addr + 24 + (i * (int)sizeof(int)), rT[i]);
  rTindex = idx;
  tallySigWritten = true;                      // already on flash; do not rewrite it
}

// Row 3 is the FAULT ROW and belongs to the DISPLAY LAYER, not to any screen.
// It is written on every render regardless of which screen is active, so a
// screen added later CANNOT silently opt out of fault indication the way
// SCR_CHECKCAL did. That was a Critical: a dead transmitter was invisible on
// the calibration-check screen, and the frozen filter made the stale number
// rock-steady rather than visibly decaying -- more convincing, not less.
static void hmiRenderFaultRow() {
  char l3[21];
  // mcpFailCount() is a LIFETIME counter (include/mcp3421.h:24) that never
  // resets, so it sits LAST: a single transient I2C error at any point since
  // boot would otherwise latch "I2C ERR" over row 3 for the rest of the power
  // cycle and hide everything above it. KEY STUCK is back with the buttons and
  // ranks where the old keypad's branch did, above I2C ERR (design §3.5): a
  // button held 10 s is ignored until released, and this says why it is.
  noInterrupts();
  const bool stuck = buttons.keyStuck();
  interrupts();
  if (!readingTrustworthy())         snprintf(l3, sizeof(l3), "%-20s", displayStateName());
  else if (calibrationDefaulted)     snprintf(l3, sizeof(l3), "%-20s", "CAL NOT SET!");
  else if (stuck)                    snprintf(l3, sizeof(l3), "%-20s", "KEY STUCK");
  else if (mcpFailCount() > 0)       snprintf(l3, sizeof(l3), "I2C ERR %-12lu", (unsigned long)mcpFailCount());
  else                               snprintf(l3, sizeof(l3), "%-20s", "");
  lcdLine(3, l3);
}

// Today's link words, shared by HOME's row 2 and the NO PANEL screen (design §4.5).
// netState, NOT WiFi.status(): see the note in hmiDrawHome().
static const char *linkText() {
  if (netState == NET_ONLINE)  return "link mqtt up";
  if (netState == NET_WIFI_UP) return "link wifi only";
  return "link none";
}

// Row 0 is the weight row on EVERY screen, unchanged (design §4.1): the operator taring
// or recording always sees the weight they act on.
static void hmiDrawWeightRow() {
  char l0[21];
  // Row 0: the weight, blanked when the loop is not measuring (DEAD,
  // FAULT_HIGH). Row 3 (hmiRenderFaultRow) names the fault on every screen,
  // but that does not make this guard unnecessary: without it, a frozen
  // filter value (see the freeze block below) sits rock-steady on this row
  // while row 3 names the fault underneath it -- a confident wrong number
  // next to the truth. SATURATED and UNDER ZERO still show the weight: they
  // are real measurements, over-range or below the live zero but measuring,
  // and withholding a weight because the transmitter is pinned was the old
  // defect, not a safeguard.
  if (readingTrustworthy())
    snprintf(l0, sizeof(l0), "W %8d lb  n%-4d", outputValue, rTindex);
  else
    snprintf(l0, sizeof(l0), "%-20s", displayStateName());
  lcdLine(0, l0);
}

static void hmiDrawHome() {
  char l1[21], l2[21];
  hmiDrawWeightRow();

  // Row 1: psi and mA, when the loop is actually measuring.
  if (readingTrustworthy())
    snprintf(l1, sizeof(l1), "psi %5d  mA %5.2f", pressureValue,
             (double)countsToMilliamps(sensorValue));
  else
    snprintf(l1, sizeof(l1), "%-20s", "psi --    mA --");

  // Row 2: whatever the operator needs in order to trust row 0/1. Worst first —
  // a dropped record is worse than a backlog, which is only late.
  if (queueDropped > 0)        snprintf(l2, sizeof(l2), "DROPPED %-12lu", queueDropped);
  else if (queueCount > 0)     snprintf(l2, sizeof(l2), "Q%-4d %-14s", queueCount,
                                        netState == NET_ONLINE ? "sending" : "no link");
  else if (rTindex >= rTsize)  snprintf(l2, sizeof(l2), "%-20s", "TALLY FULL");
  else if (rTindex >= rTsize - TALLY_WARN_REMAINING)
                                snprintf(l2, sizeof(l2), "TALLY %d LEFT%-7s", rTsize - rTindex, "");
  // netState, NOT WiFi.status(): the latter is a modem round trip on every
  // render, and netPoll() already derives netState from it once per pass. It
  // also says more -- "wifi only" is a broker problem, "no link" is a radio one.
  else                         snprintf(l2, sizeof(l2), "%-20s", linkText());

  lcdLine(1, l1); lcdLine(2, l2);
}

// Everything the menu reads from the rest of the board, at this instant.
static HmiContext hmiContext() {
  HmiContext c;
  c.now = millis();
  c.online = (netState == NET_ONLINE);
  c.linkText = linkText();
  c.lastEntryW = rTindex > 0 ? (long)rT[rTindex - 1] : 0L;
  c.entries = rTindex;
  c.nfc = nfc.health();
  c.nfcVer = nfc.fwVer();
  c.nfcRev = nfc.fwRev();
  c.build = BUILD_PROVENANCE;
  c.haveEpoch = panel.epochMs(c.now, c.epochMs);
  return c;
}

// FW-1: one reader step (at most one Wire transaction), then hand any NEW scan to the
// menu. TagTracker decides what is new (design §4.4: kGoneMs).
static void nfcPoll() {
  if (!wireOk) return;   // no controller: every "transaction" would be a false success
  const uint32_t now = millis();
  nfc.step(now);
  if (nfc.takeEmpty()) tagTracker.empty(now);
  char hex[TAG_HEX_LEN];
  if (nfc.takeUid(hex, sizeof(hex)) && tagTracker.seen(hex, now)) {
    snprintf(nfcLastTag, sizeof(nfcLastTag), "%s", hex);
    Console.print("nfc tag ");
    Console.println(hex);
    hmi.onTag(hex, hmiContext());
  }
}

// The screens, rendered every pass. Row 0 and row 3 are the display layer's: row 0 is
// drawn on every screen, and hmiRenderFaultRow() is called UNCONDITIONALLY and last --
// the fault row belongs to the display layer, not to any screen, and nothing may opt
// out of it. On HOME, rows 1 and 2 are today's home rows, unchanged (design §4.3).
static void hmiPoll() {
  // Buttons. With no free timer they are sampled here instead, every 5 ms at most, and a
  // press made during a network stall is lost (design §3.5).
  if (!btnTimerOn && millis() - lastBtnPollMs >= 5) {
    lastBtnPollMs = millis();
    buttons.sample(readButtons(), lastBtnPollMs);
  }
  HmiContext c = hmiContext();
  for (;;) {
    BtnEvent e;
    noInterrupts();
    const bool got = buttons.pop(e);
    interrupts();
    if (!got) break;
    hmi.onButton(e, c);
  }
  hmi.tick(c);

  // A local TARE the menu asked for runs through the console's handler in THIS pass, the
  // way cmdRunCapture() queues a phone command. Only when nothing else holds the handler:
  // otherwise the menu's request waits for a later pass.
  const char *lv = nullptr;
  if (!stringComplete && !ackPending && !localCmdPending && hmi.takeLocal(lv)) {
    snprintf(localSaved, sizeof(localSaved), "%s", inputString.c_str());
    inputString = lv;
    stringComplete = true;
    verbRefused = false;
    verbRefusedReason[0] = '\0';
    localCmdPending = true;
  }

  if (hmi.isHome()) {
    hmiDrawHome();
  } else {
    char r1[21], r2[21];
    hmiDrawWeightRow();
    hmi.render(r1, r2, c);
    lcdLine(1, r1);
    lcdLine(2, r2);
  }
  hmiRenderFaultRow();
}

// FW-4, design §5.2: the panel client. Settles a finished request, starts the menu's next
// one, then runs AT MOST ONE WiFiClient stage -- and only while online and under
// PANEL_PASS_BUDGET_MS (2000 ms) into this pass. Called LAST in loop(): nothing in the
// pass follows the stage, so its worst case (20 s for OPEN, 10 s for any other) is the
// whole of the pass's remaining time, against the 30 s heartbeat tolerance.
static void panelPoll() {
  PanelOutcome o;
  int status = 0;
  if (panel.takeResult(o, status)) {
    const PanelKind k = panel.kind();
    if (o != PO_HTTP) {
      Console.print("panel: "); Console.print(panelKindPath(k));
      Console.println(o == PO_FAIL_CONNECT ? " connect failed" : o == PO_FAIL_SEND ? " send failed"
                      : o == PO_FAIL_DEADLINE ? " deadline (8 s)" : " bad answer");
    } else if (status >= 400) {
      Console.print("panel: "); Console.print(panelKindPath(k)); Console.print(" ");
      Console.print(status); Console.print(" "); Console.println(panel.reply.error);
    }
    hmi.onPanelResult(panelSeq, k, o, status, panel.wrote(), panel.state, panel.deliveries, panel.reply,
                      hmiContext());
  }
  if (!panel.busy()) {
    PanelKind k;
    char body[PANEL_BODY_MAX];
    uint16_t seq = 0;
    if (hmi.takeRequest(k, body, sizeof(body), seq)) {
      panelSeq = seq;
      if (!panel.start(k, body, millis())) {
        // Never left in the air: a request that cannot be started fails at once.
        hmi.onPanelResult(seq, k, PO_FAIL_SEND, 0, false, panel.state, panel.deliveries, panel.reply,
                          hmiContext());
      }
    }
  }
  const uint32_t used = (uint32_t)(millis() - loopHeartbeatMs);
  panel.step(millis(), panelStageGate(netState == NET_ONLINE, used));
}

void loop() {
  unsigned long currentMillis = millis();
  loopHeartbeatMs = currentMillis;   // the ISR refreshes the watchdog only while this moves
  canPollFromLoop();                 // a no-op unless the CAN drain timer was unavailable

  // --- power monitor: slow, optional, and never in the weighing path --------
  // Two I2C reads every INA_POLL_MS. It flips the mux to ch2 and the sampler
  // flips it back on its next pass; the cached select makes that two extra
  // bytes, not two extra transactions. A failure here clears inaOk and is
  // never allowed to touch loopState -- this is diagnostics about the supply,
  // not a statement about the load.
  if (inaPresent() && currentMillis - lastInaMs >= INA_POLL_MS) {
    lastInaMs = currentMillis;
    float v = 0.0f, mv = 0.0f;
    if (inaRead(v, mv)) { inaBusVolts = v; inaShuntMv = mv; inaOk = true; }
    else                { inaOk = false; }
  }

  // --- sampling: one reading per pass, no delay() ---------------------------
  if (currentMillis - lastSampleMs >= SAMPLE_SPACING_MS) {
    lastSampleMs = currentMillis;
    int c = 0;
    if (mcpRead(c)) { sampleTotal += c; sampleN++; }
  }

  if (sampleN >= SAMPLE_COUNT) {
    previousMillis = currentMillis;
    lastReadingMs = currentMillis;   // stamp on a COMPLETED cycle, not a single successful read -- see F1
    sensorValue = sampleTotal / sampleN;
    sampleTotal = 0;
    sampleN = 0;

    // Classify before converting. The burden resistor makes an open loop read
    // ~0 counts rather than floating mid-rail, so LOOP_DEAD is now a real
    // measurement of loop integrity rather than a hopeful threshold.
    loopState = classifyLoop(sensorValue);

    // SATURATED is the 20.0-21.0 mA NE43 band: at or past full scale and STILL
    // MEASURING. It is counted, not faulted. Counting it is how the question of
    // how far past 3000 psi this circuit actually goes gets answered.
    if (loopState == LOOP_SATURATED) {
      overRangeCount++;
      if (sensorValue > overRangeMaxCounts) overRangeMaxCounts = sensorValue;
    }
    if (loopState == LOOP_UNDER) underRangeCount++;

    // Convert ADC counts to millivolts using the MCP3421's 14-bit resolution and
    // 2.048 V reference. MCP3421_LSB_V is defined in loop_scaling.h and captures
    // both the reference and bit depth, so a single edit moves both the loop
    // thresholds and the display when the ADC or reference changes.
    sensorVoltage = (int)(sensorValue * MCP3421_LSB_V * 1000.0f);
    // A faulted read must not touch the filter. Fed unconditionally, a dead
    // loop drags the weight SMOOTHLY DOWN to a plausible-looking number -- it
    // looks like a load being set down, which is this machine's characteristic
    // failure: a plausible wrong answer rather than a visible one. Holding the
    // last reading is playbook §6's rule and what PLC #3 does. hmiDrawHome()
    // blanks row 0 whenever !readingTrustworthy(), so a held value is never
    // shown as current -- that guard is what makes freezing safe.
    if (readingTrustworthy()) {
      smoothedValue = (smoothedValue * alpha) + (sensorValue * (1 - alpha));
    }

    // The psi conversion starts at the LIVE ZERO (4 mA), not at zero counts.
    // Mapping from 0 counts reported 299 psi at true 0 psi and 2699 at true
    // 3000 -- an offset AND a compressed span. DO NOT UNDO THAT.
    //
    // This value is PUBLISHED as `psi`, so it is not display-only cosmetics.
    //
    // What `sm` can rescale, and it is CLEANER than the voltage front end it
    // replaces: a different PRESSURE range with the same 4-20 mA output needs
    // `sm` ALONE. The count endpoints come from the burden resistor and do not
    // move when the psi span does. Under the old 0.5-4.5 V part a different
    // OUTPUT range would have moved both endpoints too.
    //
    // The live hazard is unchanged and it is not the arithmetic: `sm` is NOT
    // PERSISTED and reverts to the compile-time default on every reset. A
    // non-default range wants the default below changed, not `sm` typed after
    // each boot.
    pressureValue = countsToPSI((int)smoothedValue, sensorMaxPSI);
    outputValue = (float)calFactor * ((float)smoothedValue - (float)tareValue);

    // The weight is drawn by hmiDrawHome(); nothing else writes to the LCD here.
  }

  // The NFC reader: one Wire transaction at most. Wire is not the modem, so it is not
  // part of the §5.2 count, but it is inside the pass the panel gate measures.
  nfcPoll();

  netPoll();

  // Sits HERE so a command captured during netPoll()'s own poll() is executed in
  // the SAME pass rather than waiting for the next one. It no longer has to sit
  // here to avoid losing commands -- cmdOnMessage() captures them wherever poll()
  // is called from, including inside publishStatus()'s PUBACK spin below -- but
  // ahead of the publishes is still the lowest-latency place for it.
  cmdPoll();

  if (netState == NET_ONLINE && currentMillis - lastStatusMs >= STATUS_MS) {
    publishStatus(currentMillis);
  }

  if (netState == NET_ONLINE && currentMillis - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = currentMillis;
    // A flush that failed part-way through would otherwise sit until the next
    // reconnect, which may never come if the link never actually drops.
    if (queueCount > 0) queueFlush();
    char payload[192];
    // psi publishes JSON null, NEVER 0, when the loop is not measuring. A real
    // zero and a dead transmitter are different things and must be reported
    // differently at every layer (playbook §6). psi_fault carries the DIRECTION
    // because fail-low is a wiring question and fail-high is not -- they lead to
    // different first checks at the machine. Both field names match PLC #3.
    // mA is the unit a person with a meter at the machine is comparing against.
    char psiField[12];
    if (readingTrustworthy()) snprintf(psiField, sizeof(psiField), "%d", pressureValue);
    else                      snprintf(psiField, sizeof(psiField), "null");

    // `w` IS GUARDED THE SAME WAY `psi` IS, AND FOR THE SAME REASON. It used to
    // be published bare on every pass, so when the loop stopped measuring this
    // payload declined to state a pressure while still asserting a weight
    // derived from that same absent pressure. A frozen number and a live one
    // were indistinguishable to anything reading `w` alone -- and `w` is the
    // field consumers act on, so the unguarded one was the one that mattered.
    //
    // `stale` and `sensor` were always present and a careful consumer could
    // discriminate, but it had to opt in, and nothing signalled that `w` needed
    // it while `psi` did not. Two fields in one object using opposite
    // conventions for the identical fault is the defect, not the omission.
    //
    // NOTE WHAT THIS DOES NOT DISCARD. readingTrustworthy() is
    // loopIsMeasuring() && readingFresh(), and loopIsMeasuring() INCLUDES
    // LOOP_UNDER and LOOP_SATURATED (include/loop_scaling.h:75-77). Both carry
    // a real measurement and both keep publishing a number here. The
    // over-range excursions this front end was fitted to capture are not lost.
    // Only a loop that is not measuring at all, or an ADC that has gone quiet,
    // nulls the field.
    char wField[12];
    if (readingTrustworthy()) snprintf(wField, sizeof(wField), "%d", outputValue);
    else                      snprintf(wField, sizeof(wField), "null");

    const char *psiFault = "null";
    if (loopState == LOOP_DEAD)            psiFault = "\"low\"";
    else if (loopState == LOOP_FAULT_HIGH) psiFault = "\"high\"";

    // `stale` is a bare JSON boolean, not a string, and sits right after
    // `sensor` -- the loop-state field it qualifies. `sensor` itself stays
    // sensorStateName(): it is the LOOP state and must not be overridden by
    // displayStateName() here, or a downstream consumer would lose the
    // distinction between "loop faulted" and "ADC not answering".
    snprintf(payload, sizeof(payload),
             "{\"w\":%s,\"psi\":%s,\"mA\":%.2f,\"counts\":%d,\"sensor\":\"%s\","
             "\"stale\":%s,\"psi_fault\":%s,\"queued\":%d,\"id\":\"%s\"}",
             wField, psiField, (double)countsToMilliamps(sensorValue),
             sensorValue, sensorStateName(), readingFresh() ? "false" : "true",
             psiFault, queueCount, CLIENT_ID);
    // A5: the periodic stream is telemetry, not debug. The deliberate record
    // goes to /weight. The old firmware had these inverted.
    publishTo("telemetry", payload);
  }


  hmiPoll();

  serialEvent(); // check if data has been sent

  if (stringComplete) {
    if (inputString == "tare") {
      // Procedure: tare AT THE BOOM MARK, empty. Boom position changes the
      // unloaded pressure as well as the loaded pressure, so a tare taken
      // anywhere else puts a constant offset into every reading.
      if (!operatorMayAct()) {
        verbRefused = true;
        Console.print("tare REFUSED: ");
        Console.println(displayStateName());
      } else {
        tareValue = sensorValue;
        saveCalibration();
        Console.println("tare");
      }
    }
    else if (inputString == "debug") {
      debug_flag = !debug_flag;
    }
    else if (inputString == "+") {
      // T4 again: recording while the transducer reads as open or shorted would
      // put a fabricated number into the tally and publish it as a real weight.
      // A fault must refuse the operation, not quietly produce a plausible value.
      if (!operatorMayAct()) {
        verbRefused = true;
        Console.print("NOT RECORDED: ");
        Console.println(displayStateName());
      }
      else if (rTindex < rTsize) {
        rT[rTindex] = outputValue;
        Console.print(rTindex);
        Console.print(" = ");
        Console.println(rT[rTindex]);
        tallySaveEntry(rTindex);
        rTindex++;
        tallySaveIndex();
        // Publish live, or queue for flush on reconnect. Never blocks.
        if (netState == NET_ONLINE) {
          if (!publishWeight(outputValue, 0, false)) queuePush(outputValue);
        } else {
          queuePush(outputValue);
        }
      } else {
        // Full is a refusal too, from a different cause than a bad loop -- the
        // ack must not report "recorded" over it, and the reason it gives must
        // say the tally is full, not name a sensor state.
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "TALLY FULL");
        Console.println("Buffer Full");
      }
    }
    else if (inputString == "-") {
      // A3: the missing guard was the UPPER bound. `rTindex >= 0` was never the
      // defect (rTindex is never negative); the defect was that `-` never
      // checked rTindex < rTsize while `+` does, so at rTindex == rTsize this
      // wrote rT[50] on int rT[50]. Decrementing first bounds it by construction.
      //
      // NOTE: this also changes WHICH element is zeroed. `sum` loops i < rTindex,
      // so the removed entry is already excluded by the index alone — the old
      // line cleared the next free slot, outside that loop, and did nothing.
      // Do not "restore" it as a lost cleanup step.
      if (rTindex > 0) {
        rTindex--;
        rT[rTindex] = 0;
        tallySaveIndex();     // the entry above is stale but never read: sum loops i < rTindex
      } else {
        // This branch was a silent no-op with no string of its own, so this one
        // string is INVENTED. It is printed to the Console as well as carried in
        // the ack, so the console and the phone cannot drift apart about it.
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "TALLY EMPTY");
        Console.println("TALLY EMPTY");
      }
    }
    else if (inputString == "i2c") {
      // Rescan the bus without a reboot. Bring-up and fault-finding only: it
      // says whether the LCD and the ADC are actually answering, and at what
      // address, which "NO ADC" on its own cannot.
      i2cScan();
    }
    else if (inputString == "nfc") {
      // FW-7. The reader, the buttons and the panel client, on the interface that works
      // with no broker in reach. CONSOLE ONLY, like `i2c`: it is not on the MQTT
      // allowlist (cmdAllowed()), and no allowlisted verb plus argument spells it.
      Console.print("nfc = ");
      Console.println(nfc.health() == NFC_OK ? "ok" : nfc.health() == NFC_STARTING ? "starting" : "NOT ANSWERING");
      Console.print("nfc_fw = ");
      if (nfc.health() == NFC_OK) { Console.print(nfc.fwVer()); Console.print("."); Console.println(nfc.fwRev()); }
      else Console.println("--");
      Console.print("nfc_polls = "); Console.println(nfc.polls());
      Console.print("nfc_reads = "); Console.println(nfc.reads());
      Console.print("nfc_fails = "); Console.println(nfc.fails());
      Console.print("nfc_last = ");  Console.println(nfcLastTag[0] ? nfcLastTag : "--");
      noInterrupts();
      const bool stuck = buttons.keyStuck();
      const uint32_t lost = buttons.dropped;
      interrupts();
      Console.print("buttons = ");   Console.print(btnTimerOn ? "timer" : "loop");
      Console.println(stuck ? " KEY STUCK" : "");
      Console.print("btn_lost = ");  Console.println(lost);
      Console.print("hmi = ");       Console.print(hmi.full ? "full" : "part a");
      Console.print(" screen ");     Console.println((int)hmi.screen());
      Console.print("panel = ");     Console.println(PLC4_PANEL_URL);
      Console.print("hb_timer = ");  Console.println(hbTimerOn ? "on" : "FAILED");
      Console.print("can_timer = "); Console.println(canTimerOn ? "on" : "FAILED");
      Console.print("wire = ");      Console.println(!wireUp ? "not started" : wireOk ? "ok" : "NOT INITIALISED");
      Console.print("status_too_long = "); Console.println(statusTooLong);
      Console.print("panel_req = "); Console.println(panel.requests);
      Console.print("panel_fail = "); Console.println(panel.failures);
      Console.print("panel_calls = "); Console.println(panel.calls);
      Console.print("panel_last = ");
      Console.println(hmi.lastAnswer() == 1 ? "answered" : hmi.lastAnswer() == 2 ? "FAILED" : "--");
      if (hmi.lastError_[0]) { Console.print("panel_err = "); Console.println(hmi.lastError_); }
      Console.print("id_session = "); Console.println(cmdIds.started ? (unsigned long)cmdIds.session : 0UL);
    }
    else if (inputString == "pwr") {
      // Supply diagnostics, deliberately separate from `read`. Nothing here is
      // a weight and nothing here can be mistaken for one.
      if (!inaPresent())  Console.println("power monitor = not fitted");
      else if (!inaOk)    Console.println("power monitor = NO DATA (i2c fail)");
      else {
        Console.print("bus = ");   Console.print(inaBusVolts, 3); Console.println(" V");
        Console.print("shunt = "); Console.print(inaShuntMv, 3);  Console.println(" mV");
        // Amps carry the INA219_SHUNT_OHMS assumption; the volts above do not.
        Console.print("amps = ");  Console.print(inaShuntAmps(inaShuntMv), 4);
        Console.print("  (assumes ");
        Console.print(INA219_SHUNT_OHMS, 3);
        Console.println(" ohm shunt -- unverified)");
      }
      Console.print("ina_err = "); Console.println((unsigned long)inaFailCount());
    }
    else if (inputString == "read") {
      if (debug_flag)
      {
        printDebug();
      }
      // Same treatment as `net`: this is the field interface and a bare
      // outputValue over a dead ADC is a confident frozen weight (F2).
      Console.print("weight = ");
      if (readingTrustworthy()) Console.println(outputValue);
      else                      Console.println(displayStateName());
    }
    else if (inputString == "sum") {
      for (int i = 0; i < rTindex; i++) {
        Console.print(i);
        Console.print(" = ");
        Console.println(rT[i]);
      }
      // One implementation of the total, so what the operator reads here and
      // what the broker receives can never be different numbers.
      rTsum = tallyTotal();
      Console.print("sum = ");
      Console.println(rTsum);
    }
    else if (inputString == "b") {
      for (int i = 0; i < rTsize; i++) {
        rT[i] = 0;
      }
      rTindex = 0;
      rTsum = 0;
      tallySaveIndex();
      Console.println("Buffer Cleared");
    }
    else if (inputString == "kv") {
      // Calibrate AT THE BOOM MARK, with the known load, after taring at the
      // same mark. A7: guard the divide. At or below tare the denominator is
      // zero or negative; the old code produced inf or a sign flip and wrote
      // it straight to EEPROM.
      float span = (float)smoothedValue - (float)tareValue;
      if (!operatorMayAct()) {
        verbRefused = true;
        // A short reads high and would yield a plausible-looking span. Both the
        // zero and the span are persisted, so a bad one survives the reboot.
        Console.print("cal REFUSED: ");
        Console.println(displayStateName());
      } else if (span <= 0.0f) {
        // Failed, not refused by a loop state -- the ack must not report
        // "calibrated" over this, and the reason it gives must say WHY the
        // arithmetic was rejected, not name a sensor state.
        // calFactor is untouched here: the old factor stands, so the lie this
        // guards against is purely in the report, not in what gets used.
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "at or below tare");
        Console.println("cal FAILED: at or below tare");
      } else {
        calFactor = float(calWeight) / span;
        saveCalibration();
        calibrationDefaulted = false;
        Console.print("c = ");
        Console.println(calFactor);
      }
    }
    // `can` and `canf` MUST sit above the bare 'c' branch below, which would
    // otherwise take them as a calibration factor and reject them.
    //
    // CONSOLE ONLY, ENFORCED HERE. They are not on the MQTT allowlist, but the
    // allowlisted arg-taking verb `c` is concatenated with its arg before it
    // reaches this chain, so {"verb":"c","arg":"anf"} arrives as "canf". An
    // MQTT-originated command always has ackPending set by this point, so both
    // refuse it -- and the ack says so instead of reporting "set".
    else if ((inputString == "can" || inputString == "canf") && ackPending) {
      verbRefused = true;
      snprintf(verbRefusedReason, sizeof(verbRefusedReason), "CONSOLE ONLY");
      Console.println("can/canf: console only, refused over MQTT");
    }
    else if (inputString == "can") {
      CanSigSnapshot c;
      canSnapshotSigs(c);
      const uint32_t now = millis();
      Console.print("can = ");      Console.println(canUp ? "up" : "DOWN");
      Console.print("drain = ");    Console.println(canTimerOn ? "timer" : "loop");
      Console.print("drains = ");   Console.println((unsigned long)canDrains);
      Console.print("filter = ");   Console.print(canFiltered ? "on" : "off");
      if (canWantFilter != canFiltered) Console.print(" (asked on, REOPEN FAILED, fell back)");
      Console.println();
      Console.print("frames = ");   Console.println((unsigned long)c.framesSeen);
      Console.print("unknown = ");  Console.println((unsigned long)c.framesUnknown);
      Console.print("ring_full = "); Console.println((unsigned long)canRingFull);
      Console.print("ring_high = "); Console.println((unsigned long)canRingHigh);
      Console.print("can_err = ");  Console.print((unsigned long)canErrEvents);
      Console.print(" last ");      Console.print(canLastErr);
      Console.println(canHooked ? " hook on" : " hook OFF");
      Console.print("rpm = ");      if (j1939SigFresh(c.rpm, now)) Console.println(c.rpm.v, 1); else Console.println("--");
      Console.print("coolant_c = "); if (j1939SigFresh(c.coolantC, now)) Console.println(c.coolantC.v, 1); else Console.println("--");
      Console.print("oil_kpa = ");  if (j1939SigFresh(c.oilKpa, now)) Console.println(c.oilKpa.v, 0); else Console.println("--");
      Console.print("keysw_v = ");  if (j1939SigFresh(c.keyswV, now)) Console.println(c.keyswV.v, 2); else Console.println("--");
      Console.print("fuel_level_pct = "); if (j1939SigFresh(c.fuelLevelPct, now)) Console.println(c.fuelLevelPct.v, 1); else Console.println("--");
      Console.print("hyd_oil_c = "); if (j1939SigFresh(c.hydOilC, now)) Console.println(c.hydOilC.v, 0); else Console.println("--");
      Console.print("hours = ");    if (j1939SigFresh(c.hours, now, J1939_HOURS_STALE_MS)) Console.println(c.hours.v, 2); else Console.println("--");
    }
    else if (inputString == "canf") {
      // Toggle the hardware acceptance filter and reopen the controller. The
      // evidence it works is `unknown` growing far slower while `rpm` et al.
      // stay fresh; if engine values go to "--" with it on, turn it back off.
      const bool asked = !canFiltered;
      canUp = canSetFilter(asked);
      Console.print("can filter = ");
      Console.println(canFiltered ? "on" : "off");
      if (asked && !canFiltered && canUp) Console.println("filtered reopen FAILED; back on unfiltered");
      if (!canUp) Console.println("can REOPEN FAILED -- power-cycle the board");
    }
    else if (inputString.charAt(0) == 'c') {
      // A7: main added this direct setter with no validation at all.
      inputString.remove(0, 1);
      inputString.trim();               // `c 1.5` and `c1.5` are the same command
      float v = inputString.toFloat();
      if (!numericArg(inputString.c_str()) || isnan(v) || v <= 0.0f || v > 1000.0f) {
        // The refusal is RECORDED, not just printed. Without this the console
        // says "c REJECTED" while the ack says ok:true -- a success that never
        // happened, reported to the only interface the operator has left.
        Console.println("c REJECTED");
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "c REJECTED");
      } else {
        calFactor = v;
        saveCalibration();
      }
    }
    else if (inputString.charAt(0) == 'w') {
      inputString.remove(0, 1);
      inputString.trim();
      float v = inputString.toFloat();
      // numericArg() FIRST, and it is not belt-and-braces here: this range
      // admits zero, so without it `w` with no argument at all set alpha = 0 and
      // persisted it while reporting success.
      if (!numericArg(inputString.c_str()) || isnan(v) || v < 0.0f || v >= 1.0f) {
        Console.println("w REJECTED");
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "w REJECTED");
      } else {
        alpha = v;
        saveCalibration();
      }
    }
    else if (inputString.startsWith("sm")) {
      inputString.remove(0, 2);
      inputString.trim();
      int v = inputString.toInt();
      if (!numericArg(inputString.c_str()) || v <= 0 || v > 20000) {
        Console.println("sm REJECTED");
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "sm REJECTED");
      } else {
        sensorMaxPSI = v;
      }
    }
    else if (inputString == "hang") {
      // Deliberate watchdog test. The ONLY way to establish whether WDTRF is
      // observable at boot — which matters because `reset` is published in status
      // and consumed downstream as a liveness signal. CONSOLE ONLY: it is not on
      // the MQTT allowlist, because stopping a machine that is holding a load,
      // from a phone, is a denial of service rather than a diagnostic. At the
      // console the worst case IS the recovery: the watchdog reboots the board in
      // a few seconds, which is precisely what is being tested.
      Console.println("hang: stopping the loop. watchdog should reset in ~5 s.");
      Console.flush();
      for (;;) { }        // no heartbeat, no WDT.refresh
    }
    else if (inputString == "net") {
      Console.print("wifi = ");
      Console.println(WiFi.status() == WL_CONNECTED ? "up" : "down");
      Console.print("mqtt = ");
      Console.println(netState == NET_ONLINE ? "up" : "down");
      Console.print("ip = ");
      Console.println(WiFi.localIP());
      Console.print("queued = ");
      Console.println(queueCount);
      Console.print("dropped = ");
      Console.println(queueDropped);
      // displayStateName(), not sensorStateName(): this is the field interface
      // (see the HC-05/Bluetooth note above `net`'s handler) and an operator
      // reading a stale ADC as "sensor = ok" over the phone is exactly F2.
      Console.print("sensor = ");
      Console.println(displayStateName());
      Console.print("broker = ");
      Console.println(broker);
      Console.print("attempts = ");
      Console.println(netAttempts);
      Console.print("mqtt_err = ");
      Console.println(lastMqttError);
      // The boot banner carries this too, but USB re-enumerates after a reset
      // and eats it, so a technician with a laptop could never ask the board
      // what it was running. The status topic carries it as well, and that is
      // no use at a machine with no broker in reach.
      Console.print("build = "); Console.println(BUILD_PROVENANCE);
      Console.print("last_reset = ");
      Console.println(resetCause);
      // The command path, on the interface that can be asked about it with no
      // broker in reach. cmd_retained is here because a retained command is
      // dropped and never acked -- without a count it would be invisible.
      Console.print("cmd_sub = ");
      Console.println(cmdSubscribed ? "yes" : "no");
      Console.print("sub_fail = ");
      Console.println(subFailed);
      Console.print("cmd_rx = ");
      Console.println(cmdReceived);
      Console.print("cmd_run = ");
      Console.println(cmdExecuted);
      Console.print("cmd_dup = ");
      Console.println(cmdDuplicate);
      Console.print("cmd_retained = ");
      Console.println(cmdRetained);
      Console.print("cmd_refused = ");
      Console.println(cmdRefusedN);
      Console.print("cmd_bad = ");
      Console.println(cmdBad);
      Console.print("cmd_partial = ");
      Console.println(cmdPartial);
      // cmd_q is "waiting / ring depth", so the number has a scale attached and
      // cmd_full is readable as "arrived with all of them taken".
      Console.print("cmd_q = ");
      Console.print(cmdCapCount);
      Console.print("/");
      Console.println(CMD_CAP_DEPTH);
      Console.print("cmd_full = ");
      Console.println(cmdCapFull);
      Console.print("cmd_stale = ");
      Console.println(cmdCapStale);
      Console.print("ack_fail = ");
      Console.println(ackFailed);
    }
    inputString = "";
    stringComplete = false;
  }

  // The remote command's verdict, published now that the shared handler has run
  // and the counters are current. This is the ONLY place an ack is written for a
  // command that was actually executed: the verb's own if-block cannot know the
  // outcome any earlier, and an ack sent before the verb ran would report a
  // success that had not happened yet.
  if (ackPending) {
    ackPending = false;

    // Give the technician back whatever they were halfway through typing at the
    // enclosure. The handler above cleared inputString; consoleSaved holds what
    // was in it when the remote command arrived.
    inputString = consoleSaved;
    consoleSaved[0] = '\0';

    bool ok = !verbRefused;
    char detail[21];
    if (ok) {
      snprintf(detail, sizeof(detail), "%s", cmdSuccessDetail(ackVerb));
    } else {
      // A refusal detail is the FIRMWARE'S OWN string -- verbRefusedReason where
      // the verb set one, displayStateName() otherwise. Never a string invented
      // at this layer: the phone and the LCD must never disagree about why
      // something was refused.
      snprintf(detail, sizeof(detail), "%s",
               verbRefusedReason[0] ? verbRefusedReason : displayStateName());
    }

    // Remembered BEFORE the publish, and remembered whether or not the publish
    // succeeds. A repeat of this id must replay this verdict even if the ack
    // never reached the phone -- that is the case the repeat exists for.
    cmdRemember(ackId, ackVerb, ok, detail);
    cmdAck(ackId, ackVerb, ok, detail, false, ok && strcmp(ackVerb, "+") == 0);

    // FW-5's local verdict, design §5.3: a command the board minted for its own screen
    // came back from the panel and ran through this handler. The screen shows RECORDED or
    // NOT RECORDED from THIS verdict, never from the panel's 202.
    if (cmdIdIsBoards(ackId))
      hmi.onVerdict(ackId, ok, detail,
                    (ok && strcmp(ackVerb, "+") == 0 && rTindex > 0) ? (long)rT[rTindex - 1] : 0L,
                    hmiContext());
  }

  // The menu's local TARE (no panel), run through the handler above in this pass.
  if (localCmdPending) {
    localCmdPending = false;
    inputString = localSaved;
    localSaved[0] = '\0';
    const bool ok = !verbRefused;
    hmi.onLocalVerdict(ok, ok ? "" : (verbRefusedReason[0] ? verbRefusedReason : displayStateName()),
                       hmiContext());
  }

  // Engine data LAST in the pass, after any command has run and been acked. A
  // DTC publish can spin for its PUBACK; sitting between cmdPoll() and the
  // handler, that spin could age the reading and get a queued phone command
  // refused as stale -- the hazard publishStatus() is already rate-limited for.
  if (millis() - lastEngineMs >= ENGINE_MS) {
    lastEngineMs = millis();
    publishEngine();         // takes the windows on schedule even offline; publishes only online
  }
  publishNextDtc();          // faults go out on change, rate-limited

  // The panel client's one WiFiClient stage, LAST, after everything else in the pass
  // (design §5.2 point 2). Nothing may be added below this line: the stage's worst case
  // is budgeted as the remainder of the pass.
  panelPoll();
}

void printDebug()
{
    Console.print("a = ");
    Console.println(sensorValue);
    Console.print("v = ");
    Console.println(sensorVoltage);
    // mV stays: it's the direct check that the burden resistor is the value
    // it is supposed to be -- a calibration term now, which it was not before.
    Console.print("mA = ");
    Console.println(countsToMilliamps(sensorValue), 2);
    Console.print("p = ");
    Console.println(pressureValue);
    Console.print("t = ");
    Console.println(tareValue);
    Console.print("c = ");
    Console.println(calFactor, 5);   // 2dp printed both garbage values as "0.00"
    Console.print("sm = ");
    Console.println(sensorMaxPSI);
    // The known load the `kv` calibration is taken against. It used to be on the
    // check-cal screen ("Should be 530") beside the live reading; that screen
    // left with the buttons, so the value lives here rather than being lost.
    // Compare it against `weight` from `read`, or against `w` on telemetry.
    Console.print("calwt = ");
    Console.println(calWeight);
    Console.print("w = ");
    Console.println(alpha, 4);
    Console.print("i = ");
    Console.println(rTindex);
    // sensorStateName() here is deliberate: this is the deep diagnostic dump
    // and it must show the raw LOOP state even when the ADC has gone stale.
    // The explicit stale line below is what keeps that raw state from being
    // misread as current (F2) -- readingFresh() qualifies every field above,
    // "sensor" included.
    Console.print("sensor = ");
    Console.println(sensorStateName());
    Console.print("stale = ");
    Console.println(readingFresh() ? "no" : "yes");
    Console.print("cal = ");
    Console.println(calibrationDefaulted ? "DEFAULTED - tare and kv needed" : "stored");
  }
 /*
   SerialEvent occurs whenever a new data comes in the hardware serial RX.
 */
void serialEvent() {
  while (Serial1.available() || Serial.available()) {
    // A6: stop consuming the buffer once a line is complete, or a second
    // command arriving in the same burst concatenates onto the first
    // ("tare" + "read" -> "tareread") and is silently discarded.
    if (stringComplete) return;
    char inChar = Serial1.available() ? (char)Serial1.read() : (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) stringComplete = true;
    } else if (inputString.length() < 64) {   // was unbounded: heap growth on garbage
      inputString += inChar;
    }
  }
}
