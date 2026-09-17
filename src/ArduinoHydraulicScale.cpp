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
// HMI. Five buttons, a 20x4 LCD, and a gloved operator on a tractor seat.
//
// Principles, from the review of the old handler:
//  * EDGE triggered, not level. The old code sampled every 1000 ms and re-fired
//    while held, so holding Right recorded duplicate weights and holding Select
//    hammered EEPROM with re-tares.
//  * Everyday actions are one press from home. Rare or destructive ones live in
//    the menu, and the destructive one confirms.
//  * Status the operator needs to trust the reading is ON THE HOME SCREEN, not
//    behind a button and not only on the serial console.
// ---------------------------------------------------------------------------
const unsigned long KEY_DEBOUNCE_MS = 40;
const unsigned long MSG_HOLD_MS     = 1500;

// Five buttons to GND on pins freed by the I2C display, read with INPUT_PULLUP.
// Order matches the old ladder exactly: Right, Up, Down, Left, Select.
const uint8_t KEY_PINS[5] = { 4, 5, 6, 7, 8 };

// Per-pin "continuously LOW since" timestamp, 0 meaning released. Nothing in
// this HMI auto-repeats on hold, so a pin held LOW past KEY_STUCK_MS is a
// fault (mud, water, a failed switch), never intent.
unsigned long keyDownSince[5] = {0};
const unsigned long KEY_STUCK_MS = 10000;

enum Screen { SCR_HOME, SCR_MENU, SCR_CONFIRM, SCR_CHECKCAL };
Screen screen = SCR_HOME;

int  keyPrev = -1, keyStable = -1;
unsigned long keyChangeMs = 0;
int  menuIndex = 0;
unsigned long msgUntil = 0;
int pendingMsg = 0;   // 0 none, 1 added, 2 removed, 3 total, 4 link, 5 tare, 6 cal
// Set by a verb that REFUSED, cleared by the keypress that queues the next one.
// The keypad cannot read the outcome at press time -- the verb runs later in the
// same loop pass -- so without this the deferred message painted "Added 4200"
// over a refusal. Over-range is routine on this machine and `+` refuses on
// LOOP_SATURATED, so that false success was an everyday event, not an exotic one.
bool verbRefused = false;
// Set alongside verbRefused when the reason is NOT the loop state -- kv's
// span<=0.0f failure and a full tally are both refusals with nothing to do
// with displayStateName(). Empty means "use displayStateName(), as before";
// non-empty overrides it. Reset with verbRefused so a stale reason from one
// press can never be shown against a different one.
char verbRefusedReason[21] = "";
int  tarePrev = -1;            // previous zero, for Undo tare
bool lcdDirty = true;

const char *MENU_ITEMS[] = { "Tare at mark", "Calibrate", "Check cal",
                             "Undo tare", "Clear tally", "Exit" };
const int MENU_COUNT = 6;

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

// --- LCD message hold (replaces the blocking delay(1000) after a command) ---
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
  if (!mqttClient.beginMessage(topic)) return false;       // return IS checked — silent loss was C6
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

// True once any pin has been continuously LOW for KEY_STUCK_MS or more.
// keyIndex() already excludes that pin from the scan so the rest of the
// keypad keeps working; this is only for telling the operator why a button
// stopped responding.
static bool keyStuck() {
  unsigned long now = millis();
  for (int i = 0; i < 5; i++)
    if (keyDownSince[i] != 0 && now - keyDownSince[i] >= KEY_STUCK_MS) return true;
  return false;
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
static void publishStatus(unsigned long now) {
  lastStatusMs = now;
  char payload[448];
  // `stale` is a bare JSON boolean, not a string, and sits right after
  // `sensor` -- same form and placement as the telemetry payload, so the
  // two agree. This message is RETAINED: a subscriber that connects after
  // the ADC has already died gets stale:true on its very first delivery,
  // instead of a last-known-good "sensor":"ok" that lingers forever.
  // Supply diagnostics publish JSON null when the monitor is absent or has
  // failed, never 0 -- a real zero volts and an unread sensor are different
  // facts, the same rule `psi` already follows.
  char vbusField[14], shuntField[14];
  if (inaOk) {
    snprintf(vbusField,  sizeof(vbusField),  "%.3f", (double)inaBusVolts);
    snprintf(shuntField, sizeof(shuntField), "%.3f", (double)inaShuntMv);
  } else {
    snprintf(vbusField,  sizeof(vbusField),  "null");
    snprintf(shuntField, sizeof(shuntField), "null");
  }

  snprintf(payload, sizeof(payload),
    "{\"up\":true,\"cal\":\"%s\",\"sensor\":\"%s\",\"stale\":%s,\"tare\":%d,\"cf\":%.5f,"
    "\"n\":%d,\"total\":%ld,\"q\":%d,\"dropped\":%lu,\"uptime_s\":%lu,\"rssi\":%d,"
    "\"overrange\":%lu,\"overrange_max\":%d,\"underrange\":%lu,\"i2c_err\":%lu,"
    "\"vbus\":%s,\"shunt_mv\":%s,\"ina_err\":%lu,"
    "\"build\":\"%s\",\"reset\":\"%s\",\"id\":\"%s\"}",
    calibrationDefaulted ? "DEFAULT" : "ok",
    sensorStateName(), readingFresh() ? "false" : "true", tareValue, calFactor,
    rTindex, tallyTotal(), queueCount, queueDropped,
    (unsigned long)(now / 1000UL), (int)WiFi.RSSI(),
    overRangeCount, overRangeMaxCounts, underRangeCount,
    (unsigned long)mcpFailCount(),
    vbusField, shuntField, (unsigned long)inaFailCount(),
    BUILD_PROVENANCE, resetCause, CLIENT_ID);
  char topic[64];
  if (!buildTopic("status", topic, sizeof(topic))) return;
  if (!mqttClient.beginMessage(topic, true, 1)) return;   // retained, QoS 1
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
      return;
    }
    mqttClient.poll();
    if (!mqttClient.connected()) {
      netState = NET_WIFI_UP;
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
  publishStatus(now);      // say we are here, and on what terms, before any data
  queueFlush();                                          // was C6: no reconnect path at all
}

// Probe every 7-bit address on Wire1 and report what answers. Bring-up only —
// it runs once in setup() and on the `i2c` verb, never in loop().
//
// Worth its ~200 bytes because "NO ADC" alone cannot tell three very different
// faults apart: nothing on the bus at all (cable or power), a device present at
// an address the driver does not expect, or a device answering but misbehaving.
// The MCP3421 in particular ships as A0-A7 with addresses 0x68-0x6F, so a part
// from a different reel answers at an address this firmware never asks about.
// Expected here: 0x27 (LCD2004) and 0x68 (MCP3421A0).
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
  lcdLine(1, "Press Key:");

  for (int i = 0; i < 5; i++) pinMode(KEY_PINS[i], INPUT_PULLUP);

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

  // F1: the watchdog is armed BEFORE any networking exists, so that a hang in
  // the reconnect path has a floor under it. This is the whole reason the
  // watchdog was sequenced first in the work order.
  loopHeartbeatMs = millis();
  WDT.begin(WDT_TIMEOUT_MS);

  // Start the heartbeat refresher at 2 Hz. If no timer is free the board still runs,
  // but it is back to resetting during network calls — so say so rather than fail quiet.
  uint8_t timerType;
  int8_t timerCh = FspTimer::get_available_timer(timerType);
  bool hb = false;
  if (timerCh >= 0) {
    hb = wdtTimer.begin(TIMER_MODE_PERIODIC, timerType, (uint8_t)timerCh,
                        2.0f, 0.0f, wdtHeartbeatISR)
         && wdtTimer.setup_overflow_irq()
         && wdtTimer.open()
         && wdtTimer.start();
  }
  Console.print("heartbeat wdt = ");
  Console.println(hb ? "on" : "UNAVAILABLE - resets during network calls");

  // Seeds the freshness clock so the first READING_STALE_MS after boot is not
  // reported stale before any sample cycle has completed.
  lastReadingMs = millis();
}

// ---------------------------------------------------------------------------
// Tally persistence.
//
// Added because the LCD shield's RESET button sits in the same row as the five
// operator keys — confirmed 2026-09-01 when pressing along the row reset the board
// and wiped the tally (reset cause reported "none-flagged", i.e. the RES# pin, which
// is what an external button does). A gloved thumb will find it in the field.
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

// A LADDER COULD ONLY EVER REPORT ONE KEY -- it is a voltage divider. Discrete
// buttons can be pressed together, and everything downstream assumes "one key or
// none": two at once would present as chatter between two stable values and the
// debounce would fire repeatedly. So scan in fixed priority and return the FIRST
// pressed, preserving the old contract exactly. A pin stuck LOW past
// KEY_STUCK_MS is excluded from the scan so it cannot mask every
// lower-priority button forever -- see keyStuck() below.
static int keyIndex(unsigned long now) {
  int first = -1;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(KEY_PINS[i]) == LOW) {
      if (keyDownSince[i] == 0) keyDownSince[i] = now;
      // A held button does nothing after its initial press -- there is no
      // auto-repeat -- so ten seconds LOW is a fault, never intent. Skip it and
      // let the other four work.
      if (now - keyDownSince[i] < KEY_STUCK_MS && first < 0) first = i;
    } else {
      keyDownSince[i] = 0;          // released: it is healthy again immediately
    }
  }
  return first;
}

// Queue a verb for the existing command handler. The keypad and the serial console
// share one execution path so a button and a typed command can never diverge.
static void hmiRun(const char *verb) {
  inputString = verb;
  stringComplete = true;
}

static void hmiMessage(const char *a, const char *b, unsigned long now) {
  lcdLine(0, a);
  lcdLine(1, b);
  msgUntil = now + MSG_HOLD_MS;
}

// Row 3 is the FAULT ROW and belongs to the DISPLAY LAYER, not to any screen.
// It is written on every render regardless of which screen is active, so a
// screen added later CANNOT silently opt out of fault indication the way
// SCR_CHECKCAL did. That was a Critical: a dead transmitter was invisible on
// the calibration-check screen, and the frozen filter made the stale number
// rock-steady rather than visibly decaying -- more convincing, not less.
static void hmiRenderFaultRow() {
  char l3[21];
  // keyStuck() outranks mcpFailCount(): a stuck button is an actionable fault
  // the operator can clear on the spot (mud, water, a failed switch), while
  // mcpFailCount() is a LIFETIME counter (include/mcp3421.h:24) that never
  // resets -- so a single transient I2C error, at any point since boot,
  // would otherwise latch "I2C ERR" over row 3 for the rest of the power
  // cycle and KEY STUCK could never render again (F3).
  if (!readingTrustworthy())         snprintf(l3, sizeof(l3), "%-20s", displayStateName());
  else if (calibrationDefaulted)     snprintf(l3, sizeof(l3), "%-20s", "CAL NOT SET!");
  else if (keyStuck())               snprintf(l3, sizeof(l3), "%-20s", "KEY STUCK");
  else if (mcpFailCount() > 0)       snprintf(l3, sizeof(l3), "I2C ERR %-12lu", (unsigned long)mcpFailCount());
  else                               snprintf(l3, sizeof(l3), "%-20s", "");
  lcdLine(3, l3);
}

static void hmiDrawHome() {
  char l0[21], l1[21], l2[21];
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
  else                         snprintf(l2, sizeof(l2), "%-20s", "Sel=menu");

  lcdLine(0, l0); lcdLine(1, l1); lcdLine(2, l2);
}

static void hmiDrawMenu() {
  char l1[21];
  lcdLine(0, "Menu  Lt=back");
  snprintf(l1, sizeof(l1), ">%-19s", MENU_ITEMS[menuIndex]);
  lcdLine(1, l1);
}

static void hmiDrawCheckCal() {
  char l0[21], l1[21];
  // Kept from the 4-20 mA work: when the reading cannot be trusted, show the
  // state and no number. displayStateName() is deliberate -- it names loop
  // state exactly as the wire does, except when the ADC itself has gone
  // stale, when it says so instead of repeating a held loop state. Row 3
  // (hmiRenderFaultRow) also carries this fault now, so this guard is
  // belt-and-braces, not the only place the fault shows -- but it still
  // keeps a stale `Reads 528` off this screen specifically.
  if (!readingTrustworthy()) {
    snprintf(l0, sizeof(l0), "%-20s", displayStateName());
    snprintf(l1, sizeof(l1), "%-20s", "NO READING Lt=x");
  } else {
    snprintf(l0, sizeof(l0), "Should be %-6d", calWeight);
    snprintf(l1, sizeof(l1), "Reads %-6d Lt=x", outputValue);
  }
  lcdLine(0, l0); lcdLine(1, l1);
}

static void hmiOnPress(int key, unsigned long now) {
  lcdDirty = true;
  verbRefused = false;      // this press owns the next verdict, not the last one
  verbRefusedReason[0] = '\0';
  if (screen == SCR_HOME) {
    switch (key) {
      // Feedback is deferred to pendingMsg because the verb runs LATER in this same
      // loop pass - reading rTindex here would show the value BEFORE the action.
      case 0: hmiRun("+");   pendingMsg = 1; break;       // Right  - add to tally
      case 3: hmiRun("-");   pendingMsg = 2; break;       // Left   - delete last
      case 1: pendingMsg = 3; break;                      // Up     - total, on the LCD
      case 2: pendingMsg = 4; break;                      // Down   - link and queue, on the LCD
      case 4: screen = SCR_MENU; menuIndex = 0; break;    // Select - menu
    }
    return;
  }
  if (screen == SCR_MENU) {
    switch (key) {
      case 1: menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT; break;
      case 2: menuIndex = (menuIndex + 1) % MENU_COUNT; break;
      case 3: screen = SCR_HOME; break;                   // Left = back, always
      case 4:
        switch (menuIndex) {
          // Deferred like `+` and `-` above, and for the same reason: these two
          // verbs can REFUSE, and a message written here is written before the
          // verb has run. "Tare taken" over a refused tare reached the operator
          // while only the console saw the refusal.
          case 0: tarePrev = tareValue; hmiRun("tare");
                  pendingMsg = 5; screen = SCR_HOME; break;
          case 1: hmiRun("kv");
                  pendingMsg = 6; screen = SCR_HOME; break;
          case 2: screen = SCR_CHECKCAL; break;
          case 3: if (tarePrev >= 0) { int t = tareValue; tareValue = tarePrev; tarePrev = t;
                    saveCalibration();
                    hmiMessage("Zero restored", "", now); }
                  else hmiMessage("No previous", "zero to undo", now);
                  screen = SCR_HOME; break;
          case 4: screen = SCR_CONFIRM; break;            // clear tally - confirms
          case 5: screen = SCR_HOME; break;
        }
        break;
    }
    return;
  }
  if (screen == SCR_CONFIRM) {
    if (key == 4) { hmiRun("b"); hmiMessage("Tally cleared", "", now); }
    screen = SCR_HOME;                                    // any other key cancels
    return;
  }
  if (screen == SCR_CHECKCAL) {
    if (key == 3 || key == 4) screen = SCR_HOME;
    return;
  }
}

// Edge-triggered keypad + screen renderer. Non-blocking; no delay() anywhere.
static void hmiPoll(unsigned long now) {
  int k = keyIndex(now);
  if (k != keyPrev) { keyPrev = k; keyChangeMs = now; }
  else if (now - keyChangeMs >= KEY_DEBOUNCE_MS && k != keyStable) {
    int prevStable = keyStable;
    keyStable = k;
    // A press is intentional only if every button was released first. Going
    // straight from one held button to another -- or releasing the top one and
    // exposing a lower-priority one still held -- is rollover, not a new press.
    if (k >= 0 && prevStable < 0) hmiOnPress(k, now);
  }

  // Row 3 is the display layer's fault row and every path below reaches it --
  // including this one. A toast must not outrank an alarm.
  if (now < msgUntil) { hmiRenderFaultRow(); return; }   // let a message stand
  if (screen == SCR_CONFIRM) {
    lcdLine(0, "Clear tally?");
    lcdLine(1, "Sel=YES  any=no");
    lcdLine(2, "");
  }
  else if (screen == SCR_MENU)     hmiDrawMenu();
  else if (screen == SCR_CHECKCAL) hmiDrawCheckCal();
  else                             hmiDrawHome();
  hmiRenderFaultRow();
}

void loop() {
  unsigned long currentMillis = millis();
  loopHeartbeatMs = currentMillis;   // the ISR refreshes the watchdog only while this moves

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

  netPoll();

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

    const char *psiFault = "null";
    if (loopState == LOOP_DEAD)            psiFault = "\"low\"";
    else if (loopState == LOOP_FAULT_HIGH) psiFault = "\"high\"";

    // `stale` is a bare JSON boolean, not a string, and sits right after
    // `sensor` -- the loop-state field it qualifies. `sensor` itself stays
    // sensorStateName(): it is the LOOP state and must not be overridden by
    // displayStateName() here, or a downstream consumer would lose the
    // distinction between "loop faulted" and "ADC not answering".
    snprintf(payload, sizeof(payload),
             "{\"w\":%d,\"psi\":%s,\"mA\":%.2f,\"counts\":%d,\"sensor\":\"%s\","
             "\"stale\":%s,\"psi_fault\":%s,\"queued\":%d,\"id\":\"%s\"}",
             outputValue, psiField, (double)countsToMilliamps(sensorValue),
             sensorValue, sensorStateName(), readingFresh() ? "false" : "true",
             psiFault, queueCount, CLIENT_ID);
    // A5: the periodic stream is telemetry, not debug. The deliberate record
    // goes to /weight. The old firmware had these inverted.
    publishTo("telemetry", payload);
  }

  hmiPoll(currentMillis);

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
        lcdLine(1, displayStateName());
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
        // deferred render must not paint "Added <n>" over it, and the reason it
        // gives must say the tally is full, not name a sensor state.
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "TALLY FULL");
        Console.println("Buffer Full");
        lcdLine(1, "Buffer Full");
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
      }
    }
    else if (inputString == "i2c") {
      // Rescan the bus without a reboot. Bring-up and fault-finding only: it
      // says whether the LCD and the ADC are actually answering, and at what
      // address, which "NO ADC" on its own cannot.
      i2cScan();
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
      lcdLine(1, "Buffer Cleared");
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
        // Failed, not refused by a loop state -- the deferred render must not
        // paint "Calibrated / check it now" over this, and the reason it gives
        // must say WHY the arithmetic was rejected, not name a sensor state.
        // calFactor is untouched here: the old factor stands, so the lie this
        // guards against is purely in the report, not in what gets used.
        verbRefused = true;
        snprintf(verbRefusedReason, sizeof(verbRefusedReason), "at or below tare");
        Console.println("cal FAILED: at or below tare");
        lcdLine(1, "cal FAILED");
      } else {
        calFactor = float(calWeight) / span;
        saveCalibration();
        calibrationDefaulted = false;
        Console.print("c = ");
        Console.println(calFactor);
      }
    }
    else if (inputString.charAt(0) == 'c') {
      // A7: main added this direct setter with no validation at all.
      inputString.remove(0, 1);
      float v = inputString.toFloat();
      if (isnan(v) || v <= 0.0f || v > 1000.0f) {
        Console.println("c REJECTED");
      } else {
        calFactor = v;
        saveCalibration();
      }
    }
    else if (inputString.charAt(0) == 'w') {
      inputString.remove(0, 1);
      float v = inputString.toFloat();
      if (isnan(v) || v < 0.0f || v >= 1.0f) {
        Console.println("w REJECTED");
      } else {
        alpha = v;
        saveCalibration();
      }
    }
    else if (inputString.startsWith("sm")) {
      inputString.remove(0, 2);
      int v = inputString.toInt();
      if (v <= 0 || v > 20000) {
        Console.println("sm REJECTED");
      } else {
        sensorMaxPSI = v;
      }
    }
    else if (inputString == "hang") {
      // Deliberate watchdog test. The ONLY way to establish whether WDTRF is
      // observable at boot — which matters because `reset` is published in status
      // and consumed downstream as a liveness signal. Console-only, never on the
      // keypad, and the worst case IS the recovery: the watchdog reboots the board
      // in a few seconds, which is precisely what is being tested.
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
      Console.print("last_reset = ");
      Console.println(resetCause);
    }
    inputString = "";
    stringComplete = false;
  }

  // Deferred keypad feedback, rendered now that the verb has run and the counters
  // are current. Nothing else writes to the LCD: the HMI owns the display.
  if (pendingMsg) {
    char a[21], b[21];
    // A refused verb NEVER paints a success message. OPERATOR.md promises the
    // refusal names the state; the console kept that promise and the keypad did
    // not, and the keypad is the operator's actual interface. Row 1 is
    // displayStateName() so the LCD, the console and the wire all say one thing
    // -- and a stale ADC is named as NO ADC here too, not as a held-over "ok".
    // This is the ONLY path a keypad-triggered tare/+/kv refusal actually
    // reaches the LCD through: hmiOnPress() sets pendingMsg, so any lcdLine()
    // written earlier in the verb's own if-block (see "+") is overwritten by
    // hmiMessage() below before the operator ever sees it.
    if (verbRefused) {
      const char *head = "REFUSED";
      switch (pendingMsg) {
        case 1: head = "NOT RECORDED"; break;
        case 5: head = "TARE REFUSED"; break;
        case 6: head = "CAL REFUSED";  break;
      }
      snprintf(a, sizeof(a), "%-16s", head);
      // verbRefusedReason overrides displayStateName() when the refusal was NOT
      // about the loop state (kv's span<=0.0f, a full tally) -- see where it is
      // set. Empty means the old behaviour: name the sensor state.
      snprintf(b, sizeof(b), "%-16s",
               verbRefusedReason[0] ? verbRefusedReason : displayStateName());
    }
    else switch (pendingMsg) {
      case 1: snprintf(a, sizeof(a), "Added %-10d", outputValue);
              snprintf(b, sizeof(b), "%d in tally", rTindex); break;
      case 2: snprintf(a, sizeof(a), "%-16s", "Removed last");
              snprintf(b, sizeof(b), "%d in tally", rTindex); break;
      case 3: { long t = tallyTotal();
                rTsum = t;
                snprintf(a, sizeof(a), "Total %10ld", t);
                snprintf(b, sizeof(b), "%d entries", rTindex); } break;
      case 4: snprintf(a, sizeof(a), "WiFi %-3s MQTT %-3s",
                       WiFi.status() == WL_CONNECTED ? "up" : "no",
                       netState == NET_ONLINE ? "up" : "no");
              snprintf(b, sizeof(b), "Q%-3d dropped %lu", queueCount, queueDropped); break;
      case 5: snprintf(a, sizeof(a), "%-16s", "Tare taken");
              snprintf(b, sizeof(b), "%-16s", "at the mark?"); break;
      case 6: snprintf(a, sizeof(a), "%-16s", "Calibrated");
              snprintf(b, sizeof(b), "%-16s", "check it now"); break;
    }
    hmiMessage(a, b, currentMillis);
    pendingMsg = 0;
  }
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
