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
#include <LiquidCrystal.h>
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <WDT.h>
#include "FspTimer.h"
#include "arduino_secrets.h"

// LCD pin to Arduino
const int pin_RS = 8;
const int pin_EN = 9;
const int pin_d4 = 4;
const int pin_d5 = 5;
const int pin_d6 = 6;
const int pin_d7 = 7;
const int pin_BL = 10;
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

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

// These constants won't change. They're used to give names to the pins used:
// Pressure transducer: A2, AS WIRED. Confirmed by Alex 2026-09-01.
//
// The history, because this flip-flopped and the next reader deserves to know why:
// `main` had A2, the r4_wifi_mqtt branch had A1, this file was briefly set to A1 on
// an early answer, and Alex then confirmed the harness is actually landing on A2.
// `main` was right all along. Every bench reading taken before this change was on a
// floating A1 and means nothing.
const int analogInPin = A2;

// ---------------------------------------------------------------------------
// Transducer scaling — SPEC-DEPENDENT, NOT YET CONFIRMED AGAINST A DATASHEET
//
// 0-3000 psi, 0.5-4.5 V ratiometric. Relayed from Alex; no part number was
// supplied and no datasheet has been read. At AVCC = 5.0 V with 14-bit
// resolution (0..16383) that gives the counts below. If the part number comes
// back as anything else — a 4-20 mA variant especially — every constant in this
// block has to be re-derived. Nothing else in the file hard-codes these values.
// ---------------------------------------------------------------------------
const int SENSOR_COUNTS_ZERO  = 1638;   // 0.5 V, 0 psi (live zero)
const int SENSOR_COUNTS_FS    = 14745;  // 4.5 V, 3000 psi
const int SENSOR_COUNTS_OPEN  = 1311;   // < 0.4 V: open circuit, cut wire, dead sensor
const int SENSOR_COUNTS_SHORT = 15400;  // > 4.7 V: short to rail

// T4: the live zero is what makes "no sensor" distinguishable from "no load".
// Pattern ported from PLC #1's CURRENT_DEAD_LOOP_MA; the constant is ours.
enum SensorState { SENSOR_OK, SENSOR_OPEN, SENSOR_SHORT };
SensorState sensorState = SENSOR_OK;

float vADC;  // the voltage
float smoothedValue;  //the filtered value from the last reading
float alpha = 0.1; //the weight (between 0 and 1) given to the previous filter value
float vReference = 5.0; //using the supply voltage as the reference
unsigned long previousMillis = 0;
int interval = 500; //choose the time between readings in milliseconds
bool display_rolling_weight = true;

int sensorValue = 0;
int sensorVoltage = 0; // the voltage
int pressureValue = 0; // pressure
int outputValue = 0;
int tareValue = 0;
float calFactor = 1.0; //calibration factor
int sensorMaxPSI = 3000; // maximum pressure in PSI
int addr = 10;
#define rTsize 50
int rT[rTsize];
int rTindex = 0;
// SIGNED, and it matters: outputValue goes negative below tare and on an
// uncalibrated board, and an unsigned total underflowed to 4294967281 for three
// entries of -5. A total that is visibly absurd is better than one that is quietly
// wrong, but neither belongs on the operator's display.
long rTsum = 0;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// ---------------------------------------------------------------------------
// HMI. Five buttons, a 16x2 LCD, and a gloved operator on a tractor seat.
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

enum Screen { SCR_HOME, SCR_MENU, SCR_CONFIRM, SCR_CHECKCAL };
Screen screen = SCR_HOME;

int  keyPrev = -1, keyStable = -1;
unsigned long keyChangeMs = 0;
int  menuIndex = 0;
unsigned long msgUntil = 0;
int pendingMsg = 0;   // 0 none, 1 added, 2 removed, 3 total, 4 link
int  tarePrev = -1;            // previous zero, for Undo tare
bool lcdDirty = true;

const char *MENU_ITEMS[] = { "Tare at mark", "Calibrate", "Check cal",
                             "Undo tare", "Clear tally", "Exit" };
const int MENU_COUNT = 6;

// --- non-blocking sample accumulation (replaces the 10x delay(10) burst) ----
const int SAMPLE_COUNT = 10;
const unsigned long SAMPLE_SPACING_MS = 50;   // 10 x 50 ms = the old 500 ms period
unsigned long lastSampleMs = 0;
int  sampleN = 0;
float sampleTotal = 0;

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
const uint32_t EEPROM_SIGNATURE = 0x504C4334;   // 'PLC4'  calibration block
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
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
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
static bool publishTo(const char *leaf, const char *payload) {
  if (netState != NET_ONLINE) return false;
  char topic[64];
  int n = snprintf(topic, sizeof(topic), "%s/%s", TOPIC_ROOT, leaf);
  if (n < 0 || (size_t)n >= sizeof(topic)) return false;   // refuse to publish a truncated topic
  if (!mqttClient.beginMessage(topic)) return false;       // return IS checked — silent loss was C6
  mqttClient.print(payload);
  return mqttClient.endMessage() == 1;
}

// Ladder decode, shared by the keypad handler and the `keys` diagnostic so the two
// can never disagree about what a reading means.
static const char *keyName(int x) {
  if (x < 60)  return "Right";
  if (x < 200) return "Up";
  if (x < 400) return "Down";
  if (x < 600) return "Left";
  if (x < 800) return "Select";
  return "none";
}

static const char *sensorStateName() {
  switch (sensorState) {
    case SENSOR_OPEN:  return "NO SENSOR";
    case SENSOR_SHORT: return "SENSOR SHORT";
    default:           return "ok";
  }
}

// A buffered reading MUST be distinguishable from a live one. Publishing
// backlog bare is worse than not buffering, because the consumer cannot tell
// stale from fresh. age_ms is the delta at flush; live readings carry 0.
static bool publishWeight(int32_t w, uint32_t ageMs, bool backlog) {
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"w\":%ld,\"age_ms\":%lu,\"backlog\":%s,\"sensor\":\"%s\",\"id\":\"%s\"}",
           (long)w, (unsigned long)ageMs, backlog ? "true" : "false",
           sensorStateName(), CLIENT_ID);
  return publishTo("weight", payload);
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
  queueCount++;
}

static void queueFlush() {
  while (queueCount > 0 && netState == NET_ONLINE) {
    WeightRecord &r = queueBuf[queueHead];
    uint32_t age = (uint32_t)(millis() - r.capturedAtMs);
    char payload[128];
    // No `sensor` field here on purpose: sensorState is the state NOW, and this
    // is a reading captured minutes or hours ago. Stamping the current state on a
    // historical record would be a small lie a consumer cannot detect.
    snprintf(payload, sizeof(payload),
             "{\"w\":%ld,\"age_ms\":%lu,\"backlog\":true,\"id\":\"%s\"}",
             (long)r.weight, (unsigned long)age, CLIENT_ID);
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
    WiFi.begin(ssid, pass);        // single attempt, never a while loop (was C5)
    WDT.refresh();
    if (WiFi.status() == WL_CONNECTED) {
      netState = NET_WIFI_UP;
      lastNetAttemptMs = now - NET_RETRY_MS + STAGE_GAP_MS;  // try the broker shortly, not in 30 s
    } else {
      netState = NET_OFFLINE;
    }
    return;
  }

  // STAGE 2 — broker.
  netState = NET_WIFI_UP;

  mqttClient.setId(CLIENT_ID);                          // was unset: C7
  mqttClient.setUsernamePassword(mqttUser, mqttPass);

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
  queueFlush();                                          // was C6: no reconnect path at all
}

void setup() {
  delay(2000);
  Serial1.begin(9600);   // HC-05 field console
  Serial.begin(115200);  // USB console for bring-up and service. No wait-for-host.
  inputString.reserve(200);
  // Set the resolution BEFORE seeding the filter: main seeded smoothedValue from
  // a 10-bit read and then ran the loop at 14 bits, so the filter started ~16x
  // low and took several seconds to converge.
  analogReadResolution(14);
  smoothedValue = analogRead(analogInPin);

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
  bool defaulted = !eepromValid;
  if (isnan(calFactor) || calFactor <= 0.0001f || calFactor > 1000.0f) { calFactor = 1.0f; defaulted = true; }
  if (tareValue < SENSOR_COUNTS_OPEN || tareValue > SENSOR_COUNTS_FS)  { tareValue = SENSOR_COUNTS_ZERO; defaulted = true; }
  if (isnan(alpha) || alpha <= 0.0f || alpha >= 1.0f)                  { alpha = 0.1f; defaulted = true; }

  if (defaulted) {
    // Say so loudly. A silently defaulted calibration is the same failure the
    // stale-zero problem describes: the machine works, and reads wrong.
    calibrationDefaulted = true;
  }

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Loader Scale");
  lcd.setCursor(0, 1);
  lcd.print("Press Key:");

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

// Map a raw ladder reading to a key index, or -1 for "no button".
//
// MEASURED on the fitted shield 2026-09-01 with the `keys` verb, at 10-bit:
//   Right 0   Up 145   Down 343   Left 516   Select 731   none 1016
// Every reading sits well inside its band, so the thresholds inherited from the
// uno-r3-swserial-lcd branch are correct for this hardware and need no remap.
// They are only valid at 10 bits - see the analogReadResolution(10) in hmiPoll().
static int keyIndex(int x) {
  if (x < 60)  return 0;   // Right
  if (x < 200) return 1;   // Up
  if (x < 400) return 2;   // Down
  if (x < 600) return 3;   // Left
  if (x < 800) return 4;   // Select
  return -1;
}

// Queue a verb for the existing command handler. The keypad and the serial console
// share one execution path so a button and a typed command can never diverge.
static void hmiRun(const char *verb) {
  inputString = verb;
  stringComplete = true;
}

static void hmiMessage(const char *a, const char *b, unsigned long now) {
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print(a);
  lcd.setCursor(0, 1); lcd.print("                ");
  lcd.setCursor(0, 1); lcd.print(b);
  msgUntil = now + MSG_HOLD_MS;
}

static void hmiDrawHome() {
  char l0[17], l1[17];
  // Row 0: the weight, or the fault that makes the weight meaningless.
  if (sensorState != SENSOR_OK) snprintf(l0, sizeof(l0), "%-16s", sensorStateName());
  else                          snprintf(l0, sizeof(l0), "%-11d n%-3d", outputValue, rTindex);

  // Row 1: whatever the operator needs in order to trust row 0. Worst first —
  // an uncalibrated scale is wrong, a backlog is only late.
  if (calibrationDefaulted)      snprintf(l1, sizeof(l1), "%-16s", "CAL NOT SET!");
  else if (queueDropped > 0)     snprintf(l1, sizeof(l1), "DROPPED %-8lu", queueDropped);
  else if (queueCount > 0)       snprintf(l1, sizeof(l1), "Q%-3d %-11s", queueCount,
                                          netState == NET_ONLINE ? "sending" : "no link");
  else if (rTindex >= rTsize)    snprintf(l1, sizeof(l1), "%-16s", "TALLY FULL");
  else if (rTindex >= rTsize - 5)snprintf(l1, sizeof(l1), "%d left  Sel=menu", rTsize - rTindex);
  else                           snprintf(l1, sizeof(l1), "%-16s", "Sel=menu");

  lcd.setCursor(0, 0); lcd.print(l0);
  lcd.setCursor(0, 1); lcd.print(l1);
}

static void hmiDrawMenu() {
  char l1[17];
  lcd.setCursor(0, 0); lcd.print("Menu  Lt=back   ");
  snprintf(l1, sizeof(l1), ">%-15s", MENU_ITEMS[menuIndex]);
  lcd.setCursor(0, 1); lcd.print(l1);
}

static void hmiDrawCheckCal() {
  char l0[17], l1[17];
  snprintf(l0, sizeof(l0), "Should be %-6d", calWeight);
  snprintf(l1, sizeof(l1), "Reads %-6d Lt=x", outputValue);
  lcd.setCursor(0, 0); lcd.print(l0);
  lcd.setCursor(0, 1); lcd.print(l1);
}

static void hmiOnPress(int key, unsigned long now) {
  lcdDirty = true;
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
          case 0: tarePrev = tareValue; hmiRun("tare");
                  hmiMessage("Tare taken", "at the mark?", now); screen = SCR_HOME; break;
          case 1: hmiRun("kv");
                  hmiMessage("Calibrated", "check it now", now); screen = SCR_HOME; break;
          case 2: screen = SCR_CHECKCAL; break;
          case 3: if (tarePrev >= 0) { int t = tareValue; tareValue = tarePrev; tarePrev = t;
                    EEPROM.put(addr+4, tareValue); EEPROM.put(addr+12, EEPROM_SIGNATURE);
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
  analogReadResolution(10);                 // the ladder is a 10-bit divider
  int k = keyIndex(analogRead(0));
  if (k != keyPrev) { keyPrev = k; keyChangeMs = now; }
  else if (now - keyChangeMs >= KEY_DEBOUNCE_MS && k != keyStable) {
    keyStable = k;
    if (k >= 0) hmiOnPress(k, now);         // act on press only, never on release or hold
  }

  if (now < msgUntil) return;               // let a message stand
  if (screen == SCR_CONFIRM) {
    lcd.setCursor(0, 0); lcd.print("Clear tally?    ");
    lcd.setCursor(0, 1); lcd.print("Sel=YES  any=no ");
    return;
  }
  if (screen == SCR_MENU)     { hmiDrawMenu();     return; }
  if (screen == SCR_CHECKCAL) { hmiDrawCheckCal(); return; }
  hmiDrawHome();
}

void loop() {
  unsigned long currentMillis = millis();
  loopHeartbeatMs = currentMillis;   // the ISR refreshes the watchdog only while this moves

  // --- sampling: one reading per pass, no delay() ---------------------------
  if (currentMillis - lastSampleMs >= SAMPLE_SPACING_MS) {
    lastSampleMs = currentMillis;
    analogReadResolution(14);
    sampleTotal += analogRead(analogInPin);
    sampleN++;
  }

  if (sampleN >= SAMPLE_COUNT) {
    previousMillis = currentMillis;
    sensorValue = sampleTotal / sampleN;
    sampleTotal = 0;
    sampleN = 0;

    // T4: classify before converting. A cut wire must read as a fault, not as
    // a plausible weight. This is the likeliest hardware failure on a tractor.
    if (sensorValue < SENSOR_COUNTS_OPEN)       sensorState = SENSOR_OPEN;
    else if (sensorValue > SENSOR_COUNTS_SHORT) sensorState = SENSOR_SHORT;
    else                                        sensorState = SENSOR_OK;

    // 14-bit full scale is 16383, not 16384.
    sensorVoltage = map((int)sensorValue, 0, 16383, 0, (int)(vReference * 1000.0));
    smoothedValue = (smoothedValue * alpha) + (sensorValue * (1 - alpha));

    // The psi conversion must start at the sensor's LIVE ZERO, not at zero counts.
    // Mapping from 0 counts reported 299 psi at true 0 psi and 2699 psi at true
    // 3000 — wrong at both ends, an offset AND a compressed span. It reached this
    // firmware from the pre-14-bit line and nobody had followed it through; it is
    // the third instance of the same unfinished resolution change, after the map
    // in-max and the 10-bit filter seed.
    //
    // This value is PUBLISHED as `psi`, so it is not display-only cosmetics.
    //
    // NOTE the asymmetry: sensorMaxPSI is runtime-settable via `sm`, but the count
    // endpoints are compile-time and derived from the 0.5-4.5 V spec. Changing `sm`
    // alone does NOT rescale a physically different transducer — if the part number
    // comes back as something else, SENSOR_COUNTS_ZERO/FS have to change with it.
    pressureValue = map((int)smoothedValue, SENSOR_COUNTS_ZERO, SENSOR_COUNTS_FS,
                        0, sensorMaxPSI);
    outputValue = (float)calFactor * ((float)smoothedValue - (float)tareValue);

    // The weight is drawn by hmiDrawHome(); nothing else writes to the LCD here.
  }

  netPoll();

  if (netState == NET_ONLINE && currentMillis - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = currentMillis;
    // A flush that failed part-way through would otherwise sit until the next
    // reconnect, which may never come if the link never actually drops.
    if (queueCount > 0) queueFlush();
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"w\":%d,\"psi\":%d,\"counts\":%d,\"sensor\":\"%s\",\"queued\":%d,\"id\":\"%s\"}",
             outputValue, pressureValue, sensorValue, sensorStateName(),
             queueCount, CLIENT_ID);
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
      if (sensorState != SENSOR_OK) {
        Console.print("tare REFUSED: ");
        Console.println(sensorStateName());
      } else {
        tareValue = sensorValue;
        EEPROM.put(addr+4, tareValue);
        EEPROM.put(addr+12, EEPROM_SIGNATURE);
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
      if (sensorState != SENSOR_OK) {
        Console.print("NOT RECORDED: ");
        Console.println(sensorStateName());
        lcd.setCursor(0, 1);
        lcd.setCursor(0, 1);
        lcd.print(sensorStateName());
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
        Console.println("Buffer Full");
        lcd.setCursor(0, 1);
        lcd.print ("Buffer Full");
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
    else if (inputString == "read") {
      if (debug_flag)
      {
        printDebug();
      }
      Console.print("weight = ");
      Console.println(outputValue);
    }
    else if (inputString == "sum") {
      rTsum = 0;
      for (int i = 0; i < rTindex; i++) {
        rTsum += rT[i];
        Console.print(i);
        Console.print(" = ");
        Console.println(rT[i]);
      }
      Console.print("sum = ");
      Console.println(rTsum);
      lcd.setCursor(0, 1);
    }
    else if (inputString == "b") {
      for (int i = 0; i < rTsize; i++) {
        rT[i] = 0;
      }
      rTindex = 0;
      rTsum = 0;
      tallySaveIndex();
      Console.println("Buffer Cleared");
      lcd.setCursor(0, 1);
      lcd.setCursor(0, 1);
      lcd.print("Buffer Cleared");
    }
    else if (inputString == "kv") {
      // Calibrate AT THE BOOM MARK, with the known load, after taring at the
      // same mark. A7: guard the divide. At or below tare the denominator is
      // zero or negative; the old code produced inf or a sign flip and wrote
      // it straight to EEPROM.
      float span = (float)smoothedValue - (float)tareValue;
      if (sensorState != SENSOR_OK) {
        // A short reads high and would yield a plausible-looking span. Both the
        // zero and the span are persisted, so a bad one survives the reboot.
        Console.print("cal REFUSED: ");
        Console.println(sensorStateName());
      } else if (span <= 0.0f) {
        Console.println("cal FAILED: at or below tare");
        lcd.setCursor(0, 1);
        lcd.setCursor(0, 1);
        lcd.print("cal FAILED");
      } else {
        calFactor = float(calWeight) / span;
        EEPROM.put(addr,  calFactor);
        EEPROM.put(addr+12, EEPROM_SIGNATURE);
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
        EEPROM.put(addr,  calFactor);
        EEPROM.put(addr+12, EEPROM_SIGNATURE);
      }
    }
    else if (inputString.charAt(0) == 'w') {
      inputString.remove(0, 1);
      float v = inputString.toFloat();
      if (isnan(v) || v < 0.0f || v >= 1.0f) {
        Console.println("w REJECTED");
      } else {
        alpha = v;
        EEPROM.put(addr+8,  alpha);
        EEPROM.put(addr+12, EEPROM_SIGNATURE);
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
    else if (inputString == "keys") {
      // Live ladder sampler. Press each button in turn; every change in the decoded
      // key is reported with its raw A0 value, so the thresholds can be re-derived
      // against the shield actually fitted rather than inherited from the R3 branch.
      Console.println("keys: press each button. 15 s.");
      analogReadResolution(10);          // the ladder is a 10-bit divider
      int lastKey = -999;
      unsigned long until = millis() + 15000UL;
      while ((long)(millis() - until) < 0) {
        int x = analogRead(0);
        const char *k = keyName(x);
        if (x / 8 != lastKey / 8) {      // report on a real change, not on jitter
          Console.print("  raw = ");
          Console.print(x);
          Console.print("   -> ");
          Console.println(k);
          lastKey = x;
        }
        loopHeartbeatMs = millis();      // this loop is long; keep the heartbeat alive
        WDT.refresh();
        delay(60);
      }
      Console.println("keys: done");
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
      Console.print("sensor = ");
      Console.println(sensorStateName());
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
    char a[17], b[17];
    switch (pendingMsg) {
      case 1: snprintf(a, sizeof(a), "Added %-10d", outputValue);
              snprintf(b, sizeof(b), "%d in tally", rTindex); break;
      case 2: snprintf(a, sizeof(a), "%-16s", "Removed last");
              snprintf(b, sizeof(b), "%d in tally", rTindex); break;
      case 3: { long t = 0; for (int i = 0; i < rTindex; i++) t += rT[i];
                rTsum = t;
                snprintf(a, sizeof(a), "Total %10ld", t);
                snprintf(b, sizeof(b), "%d entries", rTindex); } break;
      case 4: snprintf(a, sizeof(a), "WiFi %-3s MQTT %-3s",
                       WiFi.status() == WL_CONNECTED ? "up" : "no",
                       netState == NET_ONLINE ? "up" : "no");
              snprintf(b, sizeof(b), "Q%-3d dropped %lu", queueCount, queueDropped); break;
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
    Console.print("sensor = ");
    Console.println(sensorStateName());
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
