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

bool debug_flag = false;

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A2;  // pressure transducer. NOTE: the r4_wifi_mqtt branch used A1.

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
unsigned int rTsum = 0;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

unsigned long button_LastChange = 0;
unsigned long button_DebounceTime = 1000;
int button_PressedIndex = 0;
int button_LastPressedIndex = 0;

// --- non-blocking sample accumulation (replaces the 10x delay(10) burst) ----
const int SAMPLE_COUNT = 10;
const unsigned long SAMPLE_SPACING_MS = 50;   // 10 x 50 ms = the old 500 ms period
unsigned long lastSampleMs = 0;
int  sampleN = 0;
float sampleTotal = 0;

// --- LCD message hold (replaces the blocking delay(1000) after a command) ---
// F1: watchdog period. It must exceed the longest blocking call in loop(), which
// is WiFi.begin() inside netPoll(). WDT.refresh() is called immediately before
// and after that call so a normal connect attempt cannot trip it, while a hang
// inside it still does. See PORT-PLAN.md — the achievable maximum period on this
// part needs confirming against the core before this goes near a board.
const uint32_t WDT_TIMEOUT_MS = 5000;

unsigned long lcdHoldUntil = 0;
const unsigned long LCD_HOLD_MS = 1000;

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
const unsigned long NET_RETRY_MS = 30000;   // bounded attempt rate, never a loop
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
static void publishWeight(int32_t w, uint32_t ageMs, bool backlog) {
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"w\":%ld,\"age_ms\":%lu,\"backlog\":%s,\"sensor\":\"%s\",\"id\":\"%s\"}",
           (long)w, (unsigned long)ageMs, backlog ? "true" : "false",
           sensorStateName(), CLIENT_ID);
  publishTo("weight", payload);
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
// LOOPED ON — one attempt per NET_RETRY_MS, after which loop() continues and
// the scale keeps weighing. That is the ruling's requirement. See PORT-PLAN.md
// for the watchdog interaction, which needs confirming on real hardware.
// ---------------------------------------------------------------------------
static void netPoll() {
  unsigned long now = millis();

  if (netState == NET_ONLINE) {
    mqttClient.poll();
    if (!mqttClient.connected()) {
      netState = (WiFi.status() == WL_CONNECTED) ? NET_WIFI_UP : NET_OFFLINE;
    }
    return;
  }

  if (now - lastNetAttemptMs < NET_RETRY_MS) return;
  lastNetAttemptMs = now;

  WDT.refresh();   // the calls below block; give them a full watchdog period

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);        // single attempt, never a while loop (was C5)
    WDT.refresh();
    if (WiFi.status() != WL_CONNECTED) { netState = NET_OFFLINE; return; }
  }
  netState = NET_WIFI_UP;

  mqttClient.setId(CLIENT_ID);                          // was unset: C7
  mqttClient.setUsernamePassword(mqttUser, mqttPass);
  if (!mqttClient.connect(broker, port)) {              // no while(1) on failure: C5
    WDT.refresh();
    return;
  }
  WDT.refresh();
  netState = NET_ONLINE;
  queueFlush();                                          // was C6: no reconnect path at all
}

void setup() {
  delay(2000);
  Serial1.begin(9600);
  inputString.reserve(200);
  // Set the resolution BEFORE seeding the filter: main seeded smoothedValue from
  // a 10-bit read and then ran the loop at 14 bits, so the filter started ~16x
  // low and took several seconds to converge.
  analogReadResolution(14);
  smoothedValue = analogRead(analogInPin);

  EEPROM.get(addr, calFactor);
  EEPROM.get(addr+4, tareValue);
  EEPROM.get(addr+8, alpha);

  // A virgin EEPROM reads 0xFF, which is NaN as a float and -1 as an int. Without
  // this, a fresh board silently runs on garbage calibration and every reading is
  // NaN. Same defect class as A7 — unvalidated calibration values.
  if (isnan(calFactor) || calFactor <= 0.0f || calFactor > 1000.0f) calFactor = 1.0f;
  if (tareValue < 0 || tareValue > 16383) tareValue = 0;
  if (isnan(alpha) || alpha < 0.0f || alpha >= 1.0f) alpha = 0.1f;

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Loader Scale");
  lcd.setCursor(0, 1);
  lcd.print("Press Key:");

  Serial1.println("JD5055e OK");
  printDebug();

  // F1: the watchdog is armed BEFORE any networking exists, so that a hang in
  // the reconnect path has a floor under it. This is the whole reason the
  // watchdog was sequenced first in the work order.
  WDT.begin(WDT_TIMEOUT_MS);
}

void loop() {
  WDT.refresh();
  unsigned long currentMillis = millis();

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

    sensorVoltage = map((int)sensorValue, 0, 16384, 0, (int)(vReference * 1000.0));
    smoothedValue = (smoothedValue * alpha) + (sensorValue * (1 - alpha));
    pressureValue = map((int)smoothedValue, 0, 16384, 0, sensorMaxPSI);
    outputValue = (float)calFactor * ((float)smoothedValue - (float)tareValue);

    if (display_rolling_weight && currentMillis > lcdHoldUntil) {
      lcd.setCursor(0, 1);
      lcd.print ("                 ");
      lcd.setCursor(0, 1);
      if (sensorState == SENSOR_OK) lcd.print (outputValue);
      else                          lcd.print (sensorStateName());
    }
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

  if ((currentMillis - button_LastChange) >  button_DebounceTime) {
    int x;
    analogReadResolution(10); // A2: the ladder thresholds are 10-bit values
    x = analogRead (0);
    lcd.setCursor(0, 1);
    if (x < 60) { //Right
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "+";
      stringComplete = true;
      lcd.print (inputString);
    }
    else if (x < 200) { //Up
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "sum";
      stringComplete = true;
      lcd.print (inputString);
    }
    else if (x < 400) { //Down
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "b";
      stringComplete = true;
    }
    else if (x < 600) { //Left
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "-";
      stringComplete = true;
      lcd.print (inputString);
    }
    else if (x < 800) { //Select
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      inputString = "tare";
      stringComplete = true;
      lcd.print (inputString);
    }
    button_LastChange = currentMillis;
    button_LastPressedIndex = button_PressedIndex;
  }

  serialEvent(); // check if data has been sent

  if (stringComplete) {
    if (inputString == "tare") {
      // Procedure: tare AT THE BOOM MARK, empty. Boom position changes the
      // unloaded pressure as well as the loaded pressure, so a tare taken
      // anywhere else puts a constant offset into every reading.
      if (sensorState != SENSOR_OK) {
        Serial1.print("tare REFUSED: ");
        Serial1.println(sensorStateName());
      } else {
        tareValue = sensorValue;
        EEPROM.put(addr+4, tareValue);
        Serial1.println("tare");
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
        Serial1.print("NOT RECORDED: ");
        Serial1.println(sensorStateName());
        lcd.setCursor(0, 1);
        lcd.print ("                           ");
        lcd.setCursor(0, 1);
        lcd.print(sensorStateName());
      }
      else if (rTindex < rTsize) {
        rT[rTindex] = outputValue;
        Serial1.print(rTindex);
        Serial1.print(" = ");
        Serial1.println(rT[rTindex]);
        lcd.print(outputValue);
        rTindex++;
        // Publish live, or queue for flush on reconnect. Never blocks.
        if (netState == NET_ONLINE) {
          if (!publishWeight(outputValue, 0, false)) queuePush(outputValue);
        } else {
          queuePush(outputValue);
        }
      } else {
        Serial1.println("Buffer Full");
        lcd.print ("                           ");
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
      }
    }
    else if (inputString == "read") {
      if (debug_flag)
      {
        printDebug();
      }
      Serial1.print("weight = ");
      Serial1.println(outputValue);
      lcd.setCursor(0, 1);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print(outputValue);
    }
    else if (inputString == "sum") {
      rTsum = 0;
      for (int i = 0; i < rTindex; i++) {
        rTsum += rT[i];
        Serial1.print(i);
        Serial1.print(" = ");
        Serial1.println(rT[i]);
      }
      Serial1.print("sum = ");
      Serial1.println(rTsum);
      lcd.setCursor(0, 1);
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("sum = ");
      lcd.print(rTsum);
    }
    else if (inputString == "b") {
      for (int i = 0; i < rTsize; i++) {
        rT[i] = 0;
      }
      rTindex = 0;
      rTsum = 0;
      Serial1.println("Buffer Cleared");
      lcd.setCursor(0, 1);
      lcd.print ("                           ");
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
        Serial1.print("cal REFUSED: ");
        Serial1.println(sensorStateName());
      } else if (span <= 0.0f) {
        Serial1.println("cal FAILED: at or below tare");
        lcd.setCursor(0, 1);
        lcd.print ("                           ");
        lcd.setCursor(0, 1);
        lcd.print("cal FAILED");
      } else {
        calFactor = float(calWeight) / span;
        EEPROM.put(addr,  calFactor);
        Serial1.print("c = ");
        Serial1.println(calFactor);
      }
    }
    else if (inputString.charAt(0) == 'c') {
      // A7: main added this direct setter with no validation at all.
      inputString.remove(0, 1);
      float v = inputString.toFloat();
      if (isnan(v) || v <= 0.0f || v > 1000.0f) {
        Serial1.println("c REJECTED");
      } else {
        calFactor = v;
        EEPROM.put(addr,  calFactor);
      }
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("c = ");
      lcd.print(calFactor);
    }
    else if (inputString.charAt(0) == 'w') {
      inputString.remove(0, 1);
      float v = inputString.toFloat();
      if (isnan(v) || v < 0.0f || v >= 1.0f) {
        Serial1.println("w REJECTED");
      } else {
        alpha = v;
        EEPROM.put(addr+8,  alpha);
      }
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("w = ");
      lcd.print(alpha);
    }
    else if (inputString.startsWith("sm")) {
      inputString.remove(0, 2);
      int v = inputString.toInt();
      if (v <= 0 || v > 20000) {
        Serial1.println("sm REJECTED");
      } else {
        sensorMaxPSI = v;
      }
      lcd.print ("                           ");
      lcd.setCursor(0, 1);
      lcd.print("sm = ");
      lcd.print(sensorMaxPSI);
    }
    else if (inputString == "net") {
      Serial1.print("wifi = ");
      Serial1.println(WiFi.status() == WL_CONNECTED ? "up" : "down");
      Serial1.print("mqtt = ");
      Serial1.println(netState == NET_ONLINE ? "up" : "down");
      Serial1.print("ip = ");
      Serial1.println(WiFi.localIP());
      Serial1.print("queued = ");
      Serial1.println(queueCount);
      Serial1.print("dropped = ");
      Serial1.println(queueDropped);
      Serial1.print("sensor = ");
      Serial1.println(sensorStateName());
    }
    // The blocking delay(1000) that used to sit here is gone: it stalled the
    // whole loop, poll() included, after every keypress. The LCD message now
    // simply holds until lcdHoldUntil passes.
    if (display_rolling_weight) { lcdHoldUntil = currentMillis + LCD_HOLD_MS; }
    inputString = "";
    stringComplete = false;
  }
}

void printDebug()
{
    Serial1.print("a = ");
    Serial1.println(sensorValue);
    Serial1.print("v = ");
    Serial1.println(sensorVoltage);
    Serial1.print("p = ");
    Serial1.println(pressureValue);
    Serial1.print("t = ");
    Serial1.println(tareValue);
    Serial1.print("c = ");
    Serial1.println(calFactor);
    Serial1.print("sm = ");
    Serial1.println(sensorMaxPSI);
    Serial1.print("w = ");
    Serial1.println(alpha);
    Serial1.print("i = ");
    Serial1.println(rTindex);
    Serial1.print("sensor = ");
    Serial1.println(sensorStateName());
  }
 /*
   SerialEvent occurs whenever a new data comes in the hardware serial RX.
 */
void serialEvent() {
  while (Serial1.available()) {
    // A6: stop consuming the buffer once a line is complete, or a second
    // command arriving in the same burst concatenates onto the first
    // ("tare" + "read" -> "tareread") and is silently discarded.
    if (stringComplete) return;
    char inChar = (char)Serial1.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) stringComplete = true;
    } else if (inputString.length() < 64) {   // was unbounded: heap growth on garbage
      inputString += inChar;
    }
  }
}
