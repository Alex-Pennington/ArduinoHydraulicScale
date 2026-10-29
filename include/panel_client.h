#pragma once
// FW-4: the board as a client of the PLC #4 panel's HTTP API, design §5.1-§5.2 and §6 FW-4.
//
// The board sends the same requests the T-Deck pendant sends (mesh 3a3e9c21c Plc4Operator.cpp
// cmdBody(), tallyBody(), deliveryStartBody()) to greybeard-plc4-pwa df172d2:
//   POST /api/cmd            {"id","verb"[,"into"][,"tag"][,"mc"]}     app.py:221-380
//   GET  /api/state          streamed key scan into fixed buffers         app.py:191-219
//   POST /api/tally/select   {"tally"}                                   app.py:475-478
//   POST /api/tally/end      {"tally"}                                   app.py:480-483
//   POST /api/tally/start    {"attr","t_ms"[,"picking_id"]}               app.py:414-472
//   GET  /api/deliveries                                                 app.py:496-503
//
// ##### ONE WiFiClient CALL PER loop() PASS, AT MOST. design §5.2 #####
// A WiFiS3 modem command can block for MODEM_TIMEOUT, 10000 ms (WiFiS3/src/Modem.h:12), and the
// TCP connect cannot be bounded on this core (src/ArduinoHydraulicScale.cpp, the note on
// setConnectionTimeout() in setup()). So every request runs as stages, ONE STAGE PER step(),
// and step() is called once per pass, last in loop(), and only when panelStageGate() is open:
//
//   OPEN   connect(host, port)   _BEGINCLIENT + _CLIENTCONNECTNAME   worst 20000 ms
//   SEND   one write()           _CLIENTSEND + passthrough           worst 10000 ms
//   READ   one read(buf, n)      at most one _CLIENTRECEIVE          worst 10000 ms
//   CLOSE  stop()                _CLIENTCLOSE                        worst 10000 ms
//
// With the gate at < 2000 ms into the pass, the worst pass is under 22000 ms against the
// 30000 ms heartbeat tolerance (src HEARTBEAT_TOLERANCE_MS), and nothing in the pass follows
// the stage. available() is never called: it is a modem command of its own.
//
// A request has an 8 s deadline, checked between stages; past it the request fails (the screen
// shows NO PANEL) and the next gated pass closes the socket. One request at a time.
//
// Pure C++, no Arduino. The transport is a template parameter so the host test can count the
// calls each step makes.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pn532.h"      // tagHexValid()
#include "mc_value.h"

#ifndef PLC4_PANEL_URL
// The PLC #4 panel on the farm LAN: the pendant's default PLC4_PWA_URL (mesh Plc4Operator.h:32-34),
// the same host as the board's broker (BRIEF.md:197). The one place the board names it.
#define PLC4_PANEL_URL "http://192.168.10.99:8081"
#endif

static const uint32_t PANEL_PASS_BUDGET_MS = 2000;   // a stage runs only under this much into the pass
static const uint32_t PANEL_DEADLINE_MS    = 8000;   // per request
static const size_t   PANEL_REQ_MAX        = 256;    // one write() of the whole request, design §5.2
static const size_t   PANEL_BODY_MAX       = 160;
static const size_t   PANEL_READ_CHUNK     = 128;    // bytes asked of one read()
static const uint8_t  PANEL_MAX_TALLIES    = 6;      // design §5.1: keep at most 6 tallies
static const uint8_t  PANEL_MAX_DELIVERIES = 6;      // and 6 deliveries
static const size_t   PANEL_TALLY_ID_MAX   = 48;     // a UUID is 36 (pendant kTallyIdMax)
static const size_t   PANEL_LABEL_MAX      = 21;

// The gate, design §5.2 point 3: online, and the pass has used less than 2000 ms.
inline bool panelStageGate(bool online, uint32_t passUsedMs) {
  return online && passUsedMs < PANEL_PASS_BUDGET_MS;
}

// ---------------------------------------------------------------------------
// URL
// ---------------------------------------------------------------------------
// "http://host[:port][/]". False for anything else.
inline bool panelParseUrl(const char *url, char *host, size_t hostCap, uint16_t &port) {
  static const char P[] = "http://";
  if (!url || strncmp(url, P, 7) != 0) return false;
  const char *h = url + 7;
  size_t hl = 0;
  while (h[hl] && h[hl] != ':' && h[hl] != '/') hl++;
  if (hl == 0 || hl >= hostCap) return false;
  memcpy(host, h, hl);
  host[hl] = '\0';
  const char *p = h + hl;
  port = 80;
  if (*p == ':') {
    p++;
    unsigned long v = 0;
    int digits = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (unsigned long)(*p - '0'); p++; digits++; if (v > 65535) return false; }
    if (digits == 0 || v == 0) return false;
    port = (uint16_t)v;
  }
  if (*p == '/') p++;
  return *p == '\0';
}

// ---------------------------------------------------------------------------
// Numbers without printf's 64-bit support (newlib-nano lacks %lld)
// ---------------------------------------------------------------------------
inline size_t panelFmtI64(char *out, size_t cap, int64_t v) {
  char t[21];
  size_t n = 0;
  const bool neg = v < 0;
  uint64_t u = neg ? (uint64_t)(-(v + 1)) + 1 : (uint64_t)v;
  do { t[n++] = (char)('0' + (u % 10)); u /= 10; } while (u && n < sizeof(t));
  if (neg) t[n++] = '-';
  if (n + 1 > cap) { if (cap) out[0] = '\0'; return 0; }
  for (size_t i = 0; i < n; i++) out[i] = t[n - 1 - i];
  out[n] = '\0';
  return n;
}

// ---------------------------------------------------------------------------
// Request bodies
// ---------------------------------------------------------------------------
// The tally ids the panel mints are UUIDs (app.py tally_start(): "THE TALLY ID IS MINTED HERE, a
// UUID"). Anything outside this alphabet is refused rather than escaped, so a body can never
// carry a quote or a backslash this end did not write.
inline bool panelTallyIdSafe(const char *s) {
  if (!s || !*s || strlen(s) >= PANEL_TALLY_ID_MAX) return false;
  for (; *s; s++) {
    const char c = *s;
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '-' || c == '_'))
      return false;
  }
  return true;
}

// POST /api/cmd. Returns the body length, or 0 when the request must not go:
//   * `into` ("daily_log") goes only with "+" (app.py:280-282 refuses it otherwise);
//   * `tag` goes only with "+", and only when tagHexValid() (app.py:298-309);
//   * `mc` goes only with "+" AND a valid tag, always as a string with one decimal
//     (app.py:257-269, moisture.py:63). An mc that is set but out of range refuses the whole
//     body: the operator chose it for this RECORD, so the RECORD does not go without it -- the
//     pendant's rule (mesh cmdBody()).
// The id is the caller's, minted by CmdIdMinter; only p4-<hex>-<digits> reaches here.
inline size_t panelCmdBody(char *out, size_t cap, const char *id, const char *verb,
                           bool into, const char *tag, uint16_t mcTenths) {
  if (cap) out[0] = '\0';
  if (!id || !verb || !*id || !*verb) return 0;
  const bool record = strcmp(verb, "+") == 0;
  const bool withTag = record && tag && tag[0] && tagHexValid(tag);
  if (record && tag && tag[0] && !withTag) return 0;    // a scanned tag that will not pass: refuse
  const bool withMc = withTag && mcTenths != MC_NONE;
  if (withMc && !mcValid(mcTenths)) return 0;
  char mc[6] = "";
  if (withMc) mcFormat(mcTenths, mc, sizeof(mc));
  int n = snprintf(out, cap, "{\"id\":\"%s\",\"verb\":\"%s\"%s%s%s%s%s%s%s}",
                   id, verb,
                   (record && into) ? ",\"into\":\"daily_log\"" : "",
                   withTag ? ",\"tag\":\"" : "", withTag ? tag : "", withTag ? "\"" : "",
                   withMc ? ",\"mc\":\"" : "", withMc ? mc : "", withMc ? "\"" : "");
  if (n <= 0 || (size_t)n >= cap) { if (cap) out[0] = '\0'; return 0; }
  return (size_t)n;
}

// POST /api/tally/select and /api/tally/end: {"tally":"<id>"} (mesh tallyBody()).
inline size_t panelTallyBody(char *out, size_t cap, const char *tally) {
  if (cap) out[0] = '\0';
  if (!panelTallyIdSafe(tally)) return 0;
  const int n = snprintf(out, cap, "{\"tally\":\"%s\"}", tally);
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

// POST /api/tally/start. The panel requires t_ms, a positive integer of at most 15 digits
// (commands.py valid_epoch_ms(), via rfid.build_tally_start()). pickingId < 0: a daily log.
inline size_t panelStartBody(char *out, size_t cap, int64_t tMs, int64_t pickingId) {
  if (cap) out[0] = '\0';
  if (tMs <= 0 || tMs > 999999999999999LL) return 0;
  char t[21], p[21];
  panelFmtI64(t, sizeof(t), tMs);
  int n;
  if (pickingId < 0) {
    n = snprintf(out, cap, "{\"attr\":\"daily_log\",\"t_ms\":%s}", t);
  } else {
    panelFmtI64(p, sizeof(p), pickingId);
    n = snprintf(out, cap, "{\"attr\":\"delivery\",\"t_ms\":%s,\"picking_id\":%s}", t, p);
  }
  return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}

// ---------------------------------------------------------------------------
// The HTTP Date header, so the board can stamp t_ms with no clock of its own
// ---------------------------------------------------------------------------
// The board has no wall clock. The panel's answers carry an RFC 7231 Date header (the pendant
// reads the same header for the same reason: mesh Plc4Operator.cpp, the "date" header parse), so
// the board takes the time from the panel's last answer and counts forward with millis().
// "Thu, 29 Oct 2026 10:14:06 GMT" -> seconds since 1970. 0 when it does not parse.
inline uint32_t panelParseHttpDate(const char *s) {
  static const char MON[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  if (!s) return 0;
  while (*s == ' ') s++;
  const char *c = strchr(s, ',');
  if (!c) return 0;
  s = c + 1;
  unsigned d = 0, y = 0, hh = 0, mm = 0, ss = 0;
  char mon[4] = "";
  if (sscanf(s, " %2u %3s %4u %2u:%2u:%2u", &d, mon, &y, &hh, &mm, &ss) != 6) return 0;
  int m = -1;
  for (int i = 0; i < 12; i++) if (strncmp(mon, MON + 3 * i, 3) == 0) { m = i + 1; break; }
  if (m < 0 || d < 1 || d > 31 || y < 1970 || y > 2105 || hh > 23 || mm > 59 || ss > 60) return 0;
  // Days from civil (H. Hinnant's algorithm, public domain).
  const int yy = (int)y - (m <= 2 ? 1 : 0);
  const int era = yy / 400;
  const unsigned yoe = (unsigned)(yy - era * 400);
  const unsigned doy = (153u * (unsigned)(m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  const long days = (long)era * 146097L + (long)doe - 719468L;
  return (uint32_t)(days * 86400L + hh * 3600L + mm * 60L + ss);
}

// ---------------------------------------------------------------------------
// Streaming JSON scan
// ---------------------------------------------------------------------------
// Not a JSON tree: a byte-at-a-time tokenizer that reports each value with its path, so a
// response of any length is read into fixed buffers (design §5.1: "parsed by a streaming key
// scan into fixed buffers, not into a JSON tree"). Escapes are honoured; \uXXXX becomes '?'.
static const uint8_t JSON_MAX_DEPTH = 6;
static const size_t  JSON_KEY_MAX   = 24;
static const uint8_t JSON_NEST_MAX  = 32;   // deeper than this is an error, not a crash
static const size_t  JSON_VAL_MAX   = 64;

enum JsonEv : uint8_t { JSON_OBJ, JSON_ARR, JSON_STR, JSON_NUM, JSON_TRUE, JSON_FALSE, JSON_NULL };

struct JsonLevel {
  bool arr;
  char key[JSON_KEY_MAX];   // "" when the key was too long to keep: it matches nothing
  uint16_t idx;
};

struct JsonPath {
  uint8_t depth;            // levels in use; deeper values are not reported
  JsonLevel lv[JSON_MAX_DEPTH];
  bool keyIs(uint8_t i, const char *k) const { return i < depth && !lv[i].arr && strcmp(lv[i].key, k) == 0; }
  int idx(uint8_t i) const { return (i < depth && lv[i].arr) ? (int)lv[i].idx : -1; }
};

typedef void (*JsonFn)(void *ctx, const JsonPath &p, JsonEv ev, const char *val);

class JsonScan {
 public:
  void reset(JsonFn fn, void *ctx) {
    fn_ = fn; ctx_ = ctx; st_ = S_VALUE; depth_ = 0; done_ = false; err_ = false; vn_ = 0;
  }
  bool done() const { return done_; }
  bool error() const { return err_; }

  void feed(char c) {
    if (done_ || err_) return;
    for (;;) {   // a number or literal ends on the next byte, which is then processed again
      switch (st_) {
        case S_VALUE:
        case S_VALUE_OR_END:
          if (ws(c)) return;
          if (st_ == S_VALUE_OR_END && c == ']') { pop(); return; }
          if (c == '{') { emit(JSON_OBJ, nullptr); push(false); st_ = S_KEY_OR_END; return; }
          if (c == '[') { emit(JSON_ARR, nullptr); push(true); st_ = S_VALUE_OR_END; return; }
          if (c == '"') { vn_ = 0; st_ = S_STR; return; }
          if (c == '-' || (c >= '0' && c <= '9')) { vn_ = 0; add(c); st_ = S_NUM; return; }
          if (c == 't' || c == 'f' || c == 'n') { vn_ = 0; add(c); st_ = S_LIT; return; }
          err_ = true; return;
        case S_KEY_OR_END:
        case S_KEY_NEXT:
          if (ws(c)) return;
          if (c == '"') { kn_ = 0; keyLong_ = false; st_ = S_KEY; return; }
          if (st_ == S_KEY_OR_END && c == '}') { pop(); return; }
          err_ = true; return;
        case S_KEY:
          if (c == '\\') { st_ = S_KEY_ESC; return; }
          if (c == '"') { setKey(); st_ = S_COLON; return; }
          addKey(c); return;
        case S_KEY_ESC: addKey(c); st_ = S_KEY; return;
        case S_COLON:
          if (ws(c)) return;
          if (c == ':') { st_ = S_VALUE; return; }
          err_ = true; return;
        case S_STR:
          if (c == '\\') { st_ = S_STR_ESC; return; }
          if (c == '"') { val_[vn_] = '\0'; emit(JSON_STR, val_); valueDone(); return; }
          add(c); return;
        case S_STR_ESC:
          if (c == 'u') { hex_ = 0; st_ = S_STR_U; return; }
          add(c == 'n' ? '\n' : c == 't' ? '\t' : c == 'r' ? '\r' : c == 'b' ? '\b' : c == 'f' ? '\f' : c);
          st_ = S_STR; return;
        case S_STR_U:
          if (++hex_ >= 4) { add('?'); st_ = S_STR; }
          return;
        case S_NUM:
          if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') { add(c); return; }
          val_[vn_] = '\0'; emit(JSON_NUM, val_); valueDone();
          continue;   // reprocess c in S_AFTER
        case S_LIT:
          if (c >= 'a' && c <= 'z') { add(c); return; }
          val_[vn_] = '\0';
          if (!strcmp(val_, "true")) emit(JSON_TRUE, val_);
          else if (!strcmp(val_, "false")) emit(JSON_FALSE, val_);
          else if (!strcmp(val_, "null")) emit(JSON_NULL, val_);
          else { err_ = true; return; }
          valueDone();
          continue;
        case S_AFTER:
          if (ws(c)) return;
          if (depth_ == 0) { err_ = true; return; }
          if (c == ',') {
            if (topArr()) {
              if (depth_ <= JSON_MAX_DEPTH) stack_[depth_ - 1].idx++;
              st_ = S_VALUE;
            } else st_ = S_KEY_NEXT;
            return;
          }
          if (c == '}' && !topArr()) { pop(); return; }
          if (c == ']' && topArr()) { pop(); return; }
          err_ = true; return;
      }
      return;
    }
  }

 private:
  enum St : uint8_t { S_VALUE, S_VALUE_OR_END, S_KEY_OR_END, S_KEY_NEXT, S_KEY, S_KEY_ESC, S_COLON,
                      S_STR, S_STR_ESC, S_STR_U, S_NUM, S_LIT, S_AFTER };
  static bool ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
  void add(char c) { if (vn_ + 1 < JSON_VAL_MAX) val_[vn_++] = c; }
  void addKey(char c) { if (kn_ + 1 < JSON_KEY_MAX) key_[kn_++] = c; else keyLong_ = true; }
  void setKey() {
    key_[kn_] = '\0';
    if (depth_ >= 1 && depth_ <= JSON_MAX_DEPTH) {
      JsonLevel &l = stack_[depth_ - 1];
      if (keyLong_) l.key[0] = '\0';
      else memcpy(l.key, key_, kn_ + 1);
    }
  }
  void emit(JsonEv ev, const char *v) {
    if (!fn_ || depth_ > JSON_MAX_DEPTH) return;
    JsonPath p;
    p.depth = depth_;
    for (uint8_t i = 0; i < depth_; i++) p.lv[i] = stack_[i];
    fn_(ctx_, p, ev, v);
  }
  bool topArr() const { return depth_ && ((arrBits_ >> (depth_ - 1)) & 1u); }
  void push(bool arr) {
    if (depth_ >= JSON_NEST_MAX) { err_ = true; return; }
    if (arr) arrBits_ |= (1u << depth_); else arrBits_ &= ~(1u << depth_);
    if (depth_ < JSON_MAX_DEPTH) {
      stack_[depth_].arr = arr;
      stack_[depth_].key[0] = '\0';
      stack_[depth_].idx = 0;
    }
    depth_++;
  }
  void pop() { if (depth_) depth_--; valueDone(); }
  void valueDone() {
    st_ = S_AFTER;
    if (depth_ == 0) done_ = true;
  }

  JsonFn fn_ = nullptr;
  void *ctx_ = nullptr;
  St st_ = S_VALUE;
  uint8_t depth_ = 0;
  uint32_t arrBits_ = 0;   // container kind for every level, stored or not
  JsonLevel stack_[JSON_MAX_DEPTH];
  char val_[JSON_VAL_MAX];
  size_t vn_ = 0;
  char key_[JSON_KEY_MAX];
  size_t kn_ = 0;
  bool keyLong_ = false;
  uint8_t hex_ = 0;
  bool done_ = false, err_ = false;
};

// ---------------------------------------------------------------------------
// What the board keeps from each answer
// ---------------------------------------------------------------------------
struct PanelTally {
  char id[PANEL_TALLY_ID_MAX];
  char label[PANEL_LABEL_MAX];
  bool delivery;
  bool selected;
  bool haveN;
  int32_t n;
};

// /api/state, read the way the pendant reads it (mesh Plc4Operator.cpp:742-815).
struct PanelState {
  bool rfid;                     // the `rfid` key is an object: the panel runs the RFID build
  bool simple;                   // `simple_tally` is an object: today's firmware with the picker
  uint8_t count;                 // open tallies kept, at most PANEL_MAX_TALLIES
  PanelTally t[PANEL_MAX_TALLIES];
  bool canStartDaily, canStartDelivery;
  bool mcEnabled;                // simple_tally.mc_enabled (app.py: st["mc_enabled"] = today_mc)
  bool lastHaveAge, lastHaveEmptied, lastEmptied, lastHaveW;
  int32_t lastAgeTenths;         // last_record.age_s in tenths of a second
  int32_t lastW;
};

struct PanelDelivery {
  int64_t pickingId;
  char label[PANEL_LABEL_MAX];
  bool tomorrow;
};

struct PanelDeliveries {
  bool enabled;
  uint8_t count;                 // kept, at most PANEL_MAX_DELIVERIES
  uint16_t total;                // rows the panel listed
  PanelDelivery d[PANEL_MAX_DELIVERIES];
  bool haveCustomer[PANEL_MAX_DELIVERIES];
};

// Any other answer: the panel's refusal words, for the console and the NOT RECORDED row.
struct PanelReply {
  char error[JSON_VAL_MAX];
  char stateName[PANEL_LABEL_MAX];
};

inline void panelCopyLabel(char *dst, const char *src, size_t cap) {
  size_t i = 0;
  for (; src[i] && i + 1 < cap; i++) {
    char c = src[i];
    if (c >= 'a' && c <= 'z') c = (char)(c - 32);   // the LCD rows are upper-case labels
    if (c < 0x20 || c > 0x7E) c = '?';              // plain ASCII only, design §4.1
    dst[i] = c;
  }
  dst[i] = '\0';
}

inline bool panelParseInt(const char *s, int64_t &out) {
  if (!s || !*s) return false;
  bool neg = false;
  if (*s == '-') { neg = true; s++; }
  int64_t v = 0;
  int digits = 0;
  for (; *s >= '0' && *s <= '9'; s++) { v = v * 10 + (*s - '0'); if (++digits > 18) return false; }
  if (digits == 0 || (*s && *s != '.')) return false;
  if (*s == '.') { for (s++; *s; s++) if (*s < '0' || *s > '9') return false; }
  out = neg ? -v : v;
  return true;
}

// "50.3" -> 503. Rounds toward zero past the first decimal.
inline bool panelParseTenths(const char *s, int32_t &out) {
  int64_t whole = 0;
  if (!panelParseInt(s, whole)) return false;
  const char *dot = strchr(s, '.');
  int frac = (dot && dot[1] >= '0' && dot[1] <= '9') ? dot[1] - '0' : 0;
  if (whole > 200000000LL || whole < -200000000LL) return false;
  out = (int32_t)(whole * 10 + (s[0] == '-' ? -frac : frac));
  return true;
}

inline void panelStateOn(void *ctx, const JsonPath &p, JsonEv ev, const char *v) {
  PanelState &s = *(PanelState *)ctx;
  if (p.depth == 1 && p.keyIs(0, "rfid")) { s.rfid = (ev == JSON_OBJ); return; }
  if (!p.keyIs(0, "simple_tally")) return;
  if (p.depth == 1) { s.simple = (ev == JSON_OBJ); return; }
  if (p.depth == 2 && p.keyIs(1, "mc_enabled")) { s.mcEnabled = (ev == JSON_TRUE); return; }
  if (p.depth == 3 && p.keyIs(1, "can_start")) {
    if (p.keyIs(2, "daily_log")) s.canStartDaily = (ev == JSON_TRUE);
    if (p.keyIs(2, "delivery"))  s.canStartDelivery = (ev == JSON_TRUE);
    return;
  }
  if (p.depth == 3 && p.keyIs(1, "last_record")) {
    if (p.keyIs(2, "age_s") && ev == JSON_NUM) s.lastHaveAge = panelParseTenths(v, s.lastAgeTenths);
    if (p.keyIs(2, "emptied") && (ev == JSON_TRUE || ev == JSON_FALSE)) {
      s.lastHaveEmptied = true; s.lastEmptied = (ev == JSON_TRUE);
    }
    if (p.keyIs(2, "w") && ev == JSON_NUM) {
      int64_t w = 0;
      s.lastHaveW = panelParseInt(v, w) && w > -100000000LL && w < 100000000LL;
      if (s.lastHaveW) s.lastW = (int32_t)w;
    }
    return;
  }
  if (p.depth >= 3 && p.keyIs(1, "open")) {
    const int i = p.idx(2);
    if (i < 0 || i >= PANEL_MAX_TALLIES) return;
    if (p.depth == 3 && ev == JSON_OBJ) {
      if ((uint8_t)(i + 1) > s.count) s.count = (uint8_t)(i + 1);
      memset(&s.t[i], 0, sizeof(s.t[i]));
      return;
    }
    if (p.depth != 4) return;
    PanelTally &t = s.t[i];
    if (p.keyIs(3, "tally") && ev == JSON_STR) { strncpy(t.id, v, sizeof(t.id) - 1); t.id[sizeof(t.id) - 1] = '\0'; }
    if (p.keyIs(3, "label") && ev == JSON_STR) panelCopyLabel(t.label, v, sizeof(t.label));
    if (p.keyIs(3, "attr") && ev == JSON_STR) t.delivery = strcmp(v, "delivery") == 0;
    if (p.keyIs(3, "selected")) t.selected = (ev == JSON_TRUE);
    if (p.keyIs(3, "n") && ev == JSON_NUM) {
      int64_t n = 0;
      t.haveN = panelParseInt(v, n) && n >= 0 && n < 100000;
      if (t.haveN) t.n = (int32_t)n;
    }
  }
}

inline void panelDeliveriesOn(void *ctx, const JsonPath &p, JsonEv ev, const char *v) {
  PanelDeliveries &s = *(PanelDeliveries *)ctx;
  if (p.depth == 1 && p.keyIs(0, "enabled")) { s.enabled = (ev == JSON_TRUE); return; }
  if (!p.keyIs(0, "rows")) return;
  const int i = p.idx(1);
  if (i < 0) return;
  if (p.depth == 2 && ev == JSON_OBJ) {
    if (s.total < 0xFFFF) s.total++;
    if (i < PANEL_MAX_DELIVERIES) {
      if ((uint8_t)(i + 1) > s.count) s.count = (uint8_t)(i + 1);
      memset(&s.d[i], 0, sizeof(s.d[i]));
      s.d[i].pickingId = -1;
      s.haveCustomer[i] = false;
    }
    return;
  }
  if (p.depth != 3 || i >= PANEL_MAX_DELIVERIES) return;
  PanelDelivery &d = s.d[i];
  if (p.keyIs(2, "picking_id") && ev == JSON_NUM) { int64_t x = 0; if (panelParseInt(v, x) && x >= 0) d.pickingId = x; }
  if (p.keyIs(2, "customer_name") && ev == JSON_STR && v[0]) {
    panelCopyLabel(d.label, v, sizeof(d.label)); s.haveCustomer[i] = true;
  }
  if (p.keyIs(2, "picking_name") && ev == JSON_STR && v[0] && !s.haveCustomer[i])
    panelCopyLabel(d.label, v, sizeof(d.label));
  if (p.keyIs(2, "tomorrow")) d.tomorrow = (ev == JSON_TRUE);
}

inline void panelReplyOn(void *ctx, const JsonPath &p, JsonEv ev, const char *v) {
  PanelReply &r = *(PanelReply *)ctx;
  if (p.depth != 1 || ev != JSON_STR) return;
  if (p.keyIs(0, "error")) { strncpy(r.error, v, sizeof(r.error) - 1); r.error[sizeof(r.error) - 1] = '\0'; }
  if (p.keyIs(0, "state_name")) panelCopyLabel(r.stateName, v, sizeof(r.stateName));
}

// ---------------------------------------------------------------------------
// The HTTP response, byte at a time
// ---------------------------------------------------------------------------
struct HttpResponse {
  int status = 0;
  long contentLength = -1;
  uint32_t dateEpoch = 0;
  bool headersDone = false;
  bool done = false;
  bool bad = false;
  uint32_t bodyCount = 0;
  JsonScan json;

  void reset(JsonFn fn, void *ctx) {
    status = 0; contentLength = -1; dateEpoch = 0; headersDone = false; done = false; bad = false;
    bodyCount = 0; ln_ = 0; first_ = true;
    json.reset(fn, ctx);
  }

  void feed(char c) {
    if (done) return;
    if (!headersDone) {
      if (c == '\n') { line_[ln_] = '\0'; onLine(); ln_ = 0; return; }
      if (c == '\r') return;
      if (ln_ + 1 < sizeof(line_)) line_[ln_++] = c;
      return;
    }
    bodyCount++;
    json.feed(c);
    if ((contentLength >= 0 && (long)bodyCount >= contentLength) || json.done() || json.error()) done = true;
  }

 private:
  static bool startsNoCase(const char *s, const char *p) {
    for (; *p; s++, p++) {
      char a = *s, b = *p;
      if (a >= 'A' && a <= 'Z') a = (char)(a + 32);
      if (a != b) return false;
    }
    return true;
  }
  void onLine() {
    if (first_) {
      first_ = false;
      // "HTTP/1.1 202 ACCEPTED"
      if (strncmp(line_, "HTTP/", 5) != 0) { bad = true; done = true; return; }
      const char *sp = strchr(line_, ' ');
      if (!sp) { bad = true; done = true; return; }
      status = atoiN(sp + 1);
      if (status < 100 || status > 599) { bad = true; done = true; }
      return;
    }
    if (ln_ == 0) {
      headersDone = true;
      if (contentLength == 0) done = true;
      return;
    }
    if (startsNoCase(line_, "content-length:")) contentLength = atoiN(line_ + 15);
    else if (startsNoCase(line_, "date:")) dateEpoch = panelParseHttpDate(line_ + 5);
  }
  static long atoiN(const char *s) {
    while (*s == ' ') s++;
    long v = 0;
    int digits = 0;
    while (*s >= '0' && *s <= '9' && digits < 9) { v = v * 10 + (*s - '0'); s++; digits++; }
    return digits ? v : -1;
  }
  char line_[100];
  size_t ln_ = 0;
  bool first_ = true;
};

// ---------------------------------------------------------------------------
// The request and its result
// ---------------------------------------------------------------------------
enum PanelKind : uint8_t { PK_NONE = 0, PK_STATE, PK_DELIVERIES, PK_CMD, PK_SELECT, PK_END, PK_START };

enum PanelOutcome : uint8_t {
  PO_PENDING = 0,
  PO_HTTP,            // an HTTP status was read; see `status`
  PO_FAIL_CONNECT,
  PO_FAIL_SEND,
  PO_FAIL_DEADLINE,   // the 8 s deadline passed between stages
  PO_FAIL_RESPONSE    // the answer did not parse as HTTP
};

inline const char *panelKindPath(PanelKind k) {
  switch (k) {
    case PK_STATE:      return "/api/state";
    case PK_DELIVERIES: return "/api/deliveries";
    case PK_CMD:        return "/api/cmd";
    case PK_SELECT:     return "/api/tally/select";
    case PK_END:        return "/api/tally/end";
    case PK_START:      return "/api/tally/start";
    default:            return "";
  }
}
inline bool panelKindPost(PanelKind k) { return k == PK_CMD || k == PK_SELECT || k == PK_END || k == PK_START; }

// Transport: any type with
//   int    connect(const char *host, uint16_t port);   // 1 on success
//   size_t write(const uint8_t *b, size_t n);
//   int    read(uint8_t *b, size_t n);                  // bytes read, <= 0 for none
//   void   stop();
template <class Transport>
class PanelHttp {
 public:
  explicit PanelHttp(Transport &t) : t_(t) {}

  bool setUrl(const char *url) { urlOk_ = panelParseUrl(url, host_, sizeof(host_), port_); return urlOk_; }
  const char *host() const { return host_; }
  uint16_t port() const { return port_; }

  bool busy() const { return st_ != ST_IDLE; }
  PanelKind kind() const { return kind_; }

  // Queue one request. False while another is in flight (one at a time, design §5.2 point 5)
  // or when the request does not fit. The deadline clock starts here, so a request held back by
  // the gate -- the board offline, say -- still fails in 8 s rather than waiting forever.
  bool start(PanelKind kind, const char *body, uint32_t now) {
    if (busy() || !urlOk_ || kind == PK_NONE) return false;
    const bool post = panelKindPost(kind);
    const size_t bl = (post && body) ? strlen(body) : 0;
    char hostHdr[48];
    if (port_ == 80) snprintf(hostHdr, sizeof(hostHdr), "%s", host_);
    else snprintf(hostHdr, sizeof(hostHdr), "%s:%u", host_, (unsigned)port_);
    int n;
    // HTTP/1.0 with Connection: close, so the panel never holds the socket for a second request.
    if (post)
      n = snprintf(req_, sizeof(req_),
                   "POST %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n"
                   "Content-Type: application/json\r\nContent-Length: %u\r\n\r\n%s",
                   panelKindPath(kind), hostHdr, (unsigned)bl, bl ? body : "");
    else
      n = snprintf(req_, sizeof(req_), "GET %s HTTP/1.0\r\nHost: %s\r\nConnection: close\r\n\r\n",
                   panelKindPath(kind), hostHdr);
    if (n <= 0 || (size_t)n >= sizeof(req_)) return false;
    reqLen_ = (size_t)n;
    kind_ = kind;
    outcome_ = PO_PENDING;
    taken_ = false;
    startedAt_ = now;
    memset(&state, 0, sizeof(state));
    memset(&deliveries, 0, sizeof(deliveries));
    memset(&reply, 0, sizeof(reply));
    if (kind == PK_STATE) resp_.reset(panelStateOn, &state);
    else if (kind == PK_DELIVERIES) resp_.reset(panelDeliveriesOn, &deliveries);
    else resp_.reset(panelReplyOn, &reply);
    st_ = ST_OPEN;
    requests++;
    return true;
  }

  // One stage, if the gate is open. Returns true when it made a transport call -- never more
  // than one. The deadline is checked first and costs no call.
  bool step(uint32_t now, bool gateOpen) {
    if (st_ == ST_IDLE) return false;
    if (st_ != ST_CLOSE && outcome_ == PO_PENDING &&
        (uint32_t)(now - startedAt_) >= PANEL_DEADLINE_MS) {
      finish(PO_FAIL_DEADLINE);
    }
    if (!gateOpen) return false;
    switch (st_) {
      case ST_OPEN:
        calls++;
        if (t_.connect(host_, port_) == 1) st_ = ST_SEND;
        else finish(PO_FAIL_CONNECT);
        return true;
      case ST_SEND: {
        calls++;
        const size_t w = t_.write((const uint8_t *)req_, reqLen_);
        if (w == reqLen_) st_ = ST_READ;
        else finish(PO_FAIL_SEND);
        return true;
      }
      case ST_READ: {
        calls++;
        uint8_t buf[PANEL_READ_CHUNK];
        const int k = t_.read(buf, sizeof(buf));
        for (int i = 0; i < k && !resp_.done; i++) resp_.feed((char)buf[i]);
        if (resp_.done) {
          if (resp_.dateEpoch) { dateEpoch_ = resp_.dateEpoch; dateAtMs_ = now; }
          finish(resp_.bad ? PO_FAIL_RESPONSE : PO_HTTP);
        }
        return true;
      }
      case ST_CLOSE:
        calls++;
        t_.stop();
        st_ = ST_IDLE;
        return true;
      default:
        return false;
    }
  }

  // The result, once, as soon as it is known -- before the socket is closed if need be.
  bool takeResult(PanelOutcome &o, int &status) {
    if (outcome_ == PO_PENDING || taken_) return false;
    taken_ = true;
    o = outcome_;
    status = resp_.status;
    return true;
  }

  // The panel's clock, counted forward from its last Date header. False before the first one.
  bool epochMs(uint32_t now, int64_t &out) const {
    if (!dateEpoch_) return false;
    out = (int64_t)dateEpoch_ * 1000 + (int64_t)(uint32_t)(now - dateAtMs_);
    return true;
  }

  // What the last answer carried, valid after takeResult() until the next start().
  PanelState state;
  PanelDeliveries deliveries;
  PanelReply reply;

  uint32_t requests = 0;   // requests started
  uint32_t calls = 0;      // transport calls made
  uint32_t failures = 0;   // requests that ended other than PO_HTTP

 private:
  enum St : uint8_t { ST_IDLE, ST_OPEN, ST_SEND, ST_READ, ST_CLOSE };
  void finish(PanelOutcome o) {
    if (outcome_ == PO_PENDING) {
      outcome_ = o;
      if (o != PO_HTTP) failures++;
    }
    st_ = ST_CLOSE;
  }

  Transport &t_;
  char host_[32] = "";
  uint16_t port_ = 0;
  bool urlOk_ = false;
  St st_ = ST_IDLE;
  PanelKind kind_ = PK_NONE;
  PanelOutcome outcome_ = PO_PENDING;
  bool taken_ = false;
  uint32_t startedAt_ = 0;
  char req_[PANEL_REQ_MAX];
  size_t reqLen_ = 0;
  HttpResponse resp_;
  uint32_t dateEpoch_ = 0;
  uint32_t dateAtMs_ = 0;
};
