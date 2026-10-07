// RFID record / tally contract logic -- spec approved for consumer hand-off;
// board firmware gated, NOT approved, NOT WIRED IN.
//
// The pure decision and serialisation logic behind
// notes/2026-10-07-rfid-record-contract.md. Nothing in src/ includes this file,
// and it must stay that way until Alex approves the firmware change: the board
// on the tractor keeps the firmware it runs today.
//
// tools/build-check.sh copies every include/*.h into the scratch sketch, but a
// header is compiled only when something includes it, so this file changes
// nothing about the image that build produces.
//
// Plain C++17. No Arduino dependency, no heap, no printf: integers are formatted
// by hand so the output does not depend on the C library's printf support. Every
// write into a caller's buffer is bounded,
// and a payload that does not fit is REFUSED (-1), never truncated -- a truncated
// JSON message is discarded whole by every consumer, the failure the sized
// beginMessage() in src/ArduinoHydraulicScale.cpp:401-409 was written against.
//
// Host test: test/test_rfid_record.cpp.

#ifndef RFID_RECORD_H
#define RFID_RECORD_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace rfid {

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------
// The id cap is the command id cap today: CmdVerdict.id is char[41]
// (src/ArduinoHydraulicScale.cpp:743). Record and tally IDs reuse it so one
// validator covers all three.
constexpr size_t ID_MAX       = 40;
// EPC length in hex characters. Gen2 EPC lengths are whole 16-bit words, so a
// valid EPC is a multiple of 4 hex characters. 32 covers a 128-bit EPC; the
// tags are not chosen, so confirm this against the chosen tag before wiring.
constexpr size_t EPC_HEX_MIN  = 4;
constexpr size_t EPC_HEX_MAX  = 32;
constexpr int    ACK_TAGS_MAX = 8;   // tags listed in a refusal ack
// Distinct tags one decision keeps, strongest first. Equal to ACK_TAGS_MAX: the
// ack lists no more than that, and a chosen tag is checked against the raw read,
// not this list, so keeping more would only cost stack.
constexpr int    SEEN_MAX     = ACK_TAGS_MAX;
constexpr int32_t DEG7        = 10000000;   // lat/lon fixed point: 1e-7 degree

// Proposed sizes for the UNO R4 WiFi (Alex, 2026-10-07: the platform is a new
// R4 WiFi, same MCU, same 32 KB RAM). test_rfid_record.cpp builds the worst case
// of each message and checks that it fits.
constexpr size_t CMD_BUF     = 256;  // command capture; today CmdCapture.body[192] (src/ArduinoHydraulicScale.cpp:953)
constexpr size_t ACK_BUF     = 576;  // ack payload; today 192 (src/ArduinoHydraulicScale.cpp:912)
constexpr size_t EVENT_BUF   = 352;  // weight-topic payload; today 160 (src/ArduinoHydraulicScale.cpp:440)
// Backlog depth. Records are taken only within WiFi range (Alex, 2026-10-07),
// so this is a short buffer for publishes that fail while the link drops, not an
// out-of-range store. See the note's "Backlog and RAM" section for the sums.
constexpr int    QUEUE_DEPTH = 16;

// Ack details. All fit the 20-character detail the ack path holds today
// (char detail[21], src/ArduinoHydraulicScale.cpp:2228).
constexpr const char *D_RECORDED    = "recorded";
constexpr const char *D_VOIDED      = "voided";
constexpr const char *D_ENDED       = "ended";
constexpr const char *D_NO_TALLY    = "NO TALLY";
constexpr const char *D_BAD_TALLY   = "BAD TALLY";
constexpr const char *D_NO_TIME     = "NO TIME";
constexpr const char *D_BAD_TIME    = "BAD TIME";
constexpr const char *D_BAD_GPS     = "BAD GPS";
constexpr const char *D_BAD_TAG     = "BAD TAG";
constexpr const char *D_NO_REC      = "NO RECORD ID";
constexpr const char *D_BAD_REC     = "BAD RECORD ID";
constexpr const char *D_NO_READER   = "NO READER";
constexpr const char *D_NO_TAG      = "NO TAG";
constexpr const char *D_MULTIPLE    = "MULTIPLE TAGS";
constexpr const char *D_NOT_IN_VIEW = "TAG NOT IN VIEW";
constexpr const char *D_QUEUE_FULL  = "QUEUE FULL";

// ---------------------------------------------------------------------------
// Character classes
// ---------------------------------------------------------------------------
// Same whitelist as cmdTokenSafe() (src/ArduinoHydraulicScale.cpp:803-813). None
// of it needs escaping in JSON, so a valid ID is written into a payload as-is.
inline bool idChar(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
         c == '-' || c == '_' || c == '.' || c == ':' || c == '+';
}

inline bool idValid(const char *s) {
  if (!s || !*s) return false;
  size_t n = 0;
  for (; *s; s++) {
    if (!idChar(*s)) return false;
    if (++n > ID_MAX) return false;
  }
  return true;
}

// Upper-case hex only. The board publishes EPCs upper-case and the app echoes
// one back as the chosen tag, so exact comparison is enough.
inline bool epcValid(const char *s) {
  if (!s) return false;
  size_t n = 0;
  for (; *s; s++) {
    const char c = *s;
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
    if (++n > EPC_HEX_MAX) return false;
  }
  return n >= EPC_HEX_MIN && n % 4 == 0;
}

// ---------------------------------------------------------------------------
// Flat-JSON field extraction
// ---------------------------------------------------------------------------
// NOT a JSON parser, in the same terms as jsonField()
// (src/ArduinoHydraulicScale.cpp:837-864): a flat object, no escape sequences,
// and an oversized value is refused rather than truncated. Unlike jsonField() it
// also reads bare numbers, which the GPS and time fields are, and it treats a
// JSON null as absent.
enum class Field : uint8_t { Absent, Ok, Bad };

inline const char *findValue(const char *src, const char *key) {
  if (!src || !key) return nullptr;
  const size_t kl = strlen(key);
  for (const char *p = strchr(src, '"'); p; p = strchr(p + 1, '"')) {
    if (strncmp(p + 1, key, kl) != 0 || p[1 + kl] != '"') continue;
    const char *q = p + 2 + kl;
    while (*q == ' ' || *q == '\t') q++;
    if (*q != ':') continue;
    q++;
    while (*q == ' ' || *q == '\t') q++;
    return q;
  }
  return nullptr;
}

inline bool isNull(const char *v) {
  return strncmp(v, "null", 4) == 0 &&
         (v[4] == ',' || v[4] == '}' || v[4] == ' ' || v[4] == '\t' || v[4] == '\0');
}

inline Field stringField(const char *src, const char *key, char *out, size_t cap) {
  if (cap == 0) return Field::Bad;
  out[0] = '\0';
  const char *v = findValue(src, key);
  if (!v || isNull(v)) return Field::Absent;
  if (*v != '"') return Field::Bad;
  v++;
  size_t i = 0;
  while (*v && *v != '"') {
    if (*v == '\\') return Field::Bad;
    if (i + 1 >= cap) { out[0] = '\0'; return Field::Bad; }
    out[i++] = *v++;
  }
  if (*v != '"') { out[0] = '\0'; return Field::Bad; }
  out[i] = '\0';
  return Field::Ok;
}

inline Field numberToken(const char *src, const char *key, char *out, size_t cap) {
  if (cap == 0) return Field::Bad;
  out[0] = '\0';
  const char *v = findValue(src, key);
  if (!v || isNull(v)) return Field::Absent;
  size_t i = 0;
  while ((*v >= '0' && *v <= '9') || *v == '-' || *v == '+' || *v == '.' ||
         *v == 'e' || *v == 'E') {
    if (i + 1 >= cap) { out[0] = '\0'; return Field::Bad; }
    out[i++] = *v++;
  }
  out[i] = '\0';
  if (i == 0) return Field::Bad;
  if (!(*v == ',' || *v == '}' || *v == ' ' || *v == '\t' || *v == '\0')) return Field::Bad;
  return Field::Ok;
}

// ---------------------------------------------------------------------------
// Number parsing -- integer arithmetic only, so the value published is exactly
// the value the phone sent, to the stated resolution.
// ---------------------------------------------------------------------------
// Decimal with an optional sign and fraction; no exponent. Fraction digits past
// `frac` are checked to be digits and then truncated: phones commonly report
// more decimals than any GPS can mean. *roundedAway reports whether a non-zero
// digit was dropped, for callers that must round up.
inline bool parseFixed(const char *s, int frac, int64_t *out, bool *roundedAway = nullptr) {
  if (!s || !*s) return false;
  bool neg = false;
  if (*s == '-') { neg = true; s++; }
  int64_t v = 0;
  int intDigits = 0;
  while (*s >= '0' && *s <= '9') {
    if (++intDigits > 10) return false;
    v = v * 10 + (*s++ - '0');
  }
  if (intDigits == 0) return false;
  int got = 0;
  bool dropped = false;
  if (*s == '.') {
    s++;
    int fracDigits = 0;
    while (*s >= '0' && *s <= '9') {
      if (++fracDigits > 20) return false;
      if (got < frac) { v = v * 10 + (*s - '0'); got++; }
      else if (*s != '0') dropped = true;
      s++;
    }
    if (fracDigits == 0) return false;
  }
  if (*s != '\0') return false;                 // exponent, junk, second sign
  for (; got < frac; got++) v *= 10;
  *out = neg ? -v : v;
  if (roundedAway) *roundedAway = dropped;
  return true;
}

inline bool parseDeg7(const char *s, int32_t limitDeg, int32_t *out) {
  int64_t v;
  if (!parseFixed(s, 7, &v)) return false;
  const int64_t lim = (int64_t)limitDeg * DEG7;
  if (v < -lim || v > lim) return false;
  *out = (int32_t)v;
  return true;
}

// Accuracy in whole metres, rounded UP: a published accuracy may be pessimistic,
// never optimistic.
inline bool parseAccM(const char *s, uint16_t *out) {
  if (!s || *s == '-') return false;
  int64_t v;
  bool dropped = false;
  if (!parseFixed(s, 0, &v, &dropped)) return false;
  if (dropped) v++;
  if (v > 65535) return false;
  *out = (uint16_t)v;
  return true;
}

// Phone time: UTC milliseconds since the Unix epoch, as an integer.
inline bool parseEpochMs(const char *s, int64_t *out) {
  if (!s || !*s) return false;
  int64_t v = 0;
  int n = 0;
  for (; *s; s++) {
    if (*s < '0' || *s > '9') return false;
    if (++n > 15) return false;
    v = v * 10 + (*s - '0');
  }
  if (v <= 0) return false;
  *out = v;
  return true;
}

// ---------------------------------------------------------------------------
// GPS fix from the phone. All three of lat, lon, acc, or none of them.
// ---------------------------------------------------------------------------
struct Fix {
  bool     present = false;
  int32_t  lat7    = 0;      // degrees * 1e7
  int32_t  lon7    = 0;
  uint16_t accM    = 0;      // metres, rounded up
};

inline Field readFix(const char *body, Fix *f) {
  *f = Fix{};
  char lat[24], lon[24], acc[24];
  const Field a = numberToken(body, "lat", lat, sizeof(lat));
  const Field b = numberToken(body, "lon", lon, sizeof(lon));
  const Field c = numberToken(body, "acc", acc, sizeof(acc));
  if (a == Field::Absent && b == Field::Absent && c == Field::Absent) return Field::Absent;
  if (a != Field::Ok || b != Field::Ok || c != Field::Ok) return Field::Bad;
  if (!parseDeg7(lat, 90, &f->lat7) || !parseDeg7(lon, 180, &f->lon7) ||
      !parseAccM(acc, &f->accM)) {
    *f = Fix{};
    return Field::Bad;
  }
  f->present = true;
  return Field::Ok;
}

// Time then GPS, shared by all three verbs. nullptr when both are usable.
inline const char *readTimeAndFix(const char *body, int64_t *tMs, Fix *fix) {
  char t[24];
  const Field ft = numberToken(body, "t_ms", t, sizeof(t));
  if (ft == Field::Absent) return D_NO_TIME;
  if (ft == Field::Bad || !parseEpochMs(t, tMs)) return D_BAD_TIME;
  if (readFix(body, fix) == Field::Bad) return D_BAD_GPS;
  return nullptr;
}

// ---------------------------------------------------------------------------
// Commands. `id` and `verb` are read and checked by the existing path
// (src/ArduinoHydraulicScale.cpp:1140-1170) before any of these run.
// Each returns nullptr when the command is usable, else the refusal detail.
// ---------------------------------------------------------------------------
struct RecordCmd {
  char    tally[ID_MAX + 1]    = "";
  char    tag[EPC_HEX_MAX + 1] = "";   // the chosen tag, on the retry
  bool    hasTag               = false;
  int64_t tMs                  = 0;
  Fix     fix;
};

struct VoidCmd {
  char    rec[ID_MAX + 1] = "";
  int64_t tMs             = 0;
  Fix     fix;
};

struct EndCmd {
  char    tally[ID_MAX + 1] = "";
  int64_t tMs               = 0;
  Fix     fix;
};

inline const char *readTally(const char *body, char *out, size_t cap) {
  const Field f = stringField(body, "tally", out, cap);
  if (f == Field::Absent) return D_NO_TALLY;
  if (f == Field::Bad || !idValid(out)) return D_BAD_TALLY;
  return nullptr;
}

inline const char *parseRecord(const char *body, RecordCmd *c) {
  *c = RecordCmd{};
  if (const char *why = readTally(body, c->tally, sizeof(c->tally))) return why;
  if (const char *why = readTimeAndFix(body, &c->tMs, &c->fix)) return why;
  const Field t = stringField(body, "tag", c->tag, sizeof(c->tag));
  if (t == Field::Bad || (t == Field::Ok && !epcValid(c->tag))) return D_BAD_TAG;
  c->hasTag = (t == Field::Ok);
  return nullptr;
}

inline const char *parseVoid(const char *body, VoidCmd *c) {
  *c = VoidCmd{};
  const Field r = stringField(body, "rec", c->rec, sizeof(c->rec));
  if (r == Field::Absent) return D_NO_REC;
  if (r == Field::Bad || !idValid(c->rec)) return D_BAD_REC;
  return readTimeAndFix(body, &c->tMs, &c->fix);
}

inline const char *parseEnd(const char *body, EndCmd *c) {
  *c = EndCmd{};
  if (const char *why = readTally(body, c->tally, sizeof(c->tally))) return why;
  return readTimeAndFix(body, &c->tMs, &c->fix);
}

// ---------------------------------------------------------------------------
// Record ID
// ---------------------------------------------------------------------------
// The record ID IS the id of the record command that was accepted. The app
// generates it; the board holds no ID state at all, so nothing has to survive a
// reboot and nothing is written to flash on boot -- the board's own reason for
// refusing a persisted boot counter (src/ArduinoHydraulicScale.cpp:470-472).
// The note's "Record ID" section gives the full argument.
inline bool recordIdFor(const char *cmdId, char *out, size_t cap) {
  if (!idValid(cmdId)) return false;
  const size_t n = strlen(cmdId);
  if (n + 1 > cap) return false;
  memcpy(out, cmdId, n + 1);
  return true;
}

// ---------------------------------------------------------------------------
// The tag decision
// ---------------------------------------------------------------------------
struct TagRead {
  char    epc[EPC_HEX_MAX + 1];
  int16_t rssi;                  // dBm as the reader reports it; higher is stronger
};

enum class Outcome : uint8_t { Accept, NoReader, NoTag, MultipleTags, TagNotInView };

struct Decision {
  Outcome outcome   = Outcome::NoTag;
  TagRead tag       = {"", 0};        // the tag recorded, on Accept
  TagRead list[SEEN_MAX];             // distinct tags in view, strongest first
  int     listCount = 0;
  int     seen      = 0;              // distinct valid tags in view, uncapped
};

inline const char *outcomeDetail(Outcome o) {
  switch (o) {
    case Outcome::Accept:       return D_RECORDED;
    case Outcome::NoReader:     return D_NO_READER;
    case Outcome::NoTag:        return D_NO_TAG;
    case Outcome::MultipleTags: return D_MULTIPLE;
    case Outcome::TagNotInView: return D_NOT_IN_VIEW;
  }
  return D_NO_TAG;
}

// Stronger first; equal signal in EPC order, so the list does not depend on the
// order the reader happened to report the tags in.
inline bool tagBefore(const TagRead &a, const TagRead &b) {
  if (a.rssi != b.rssi) return a.rssi > b.rssi;
  return strcmp(a.epc, b.epc) < 0;
}

// `seen` is one read's raw output: a reader reports the same tag more than once
// in an inventory round, so duplicates are merged at their strongest reading.
// Entries with an invalid EPC are ignored. `chosen` is nullptr, or the EPC the
// app sent on the retry after a MULTIPLE TAGS refusal.
//
// One tag in view: accepted. None: NO TAG. Several, none chosen: MULTIPLE TAGS.
// A chosen tag is accepted only if it is in THIS read -- whatever else is also
// in view -- and refused TAG NOT IN VIEW otherwise. Every refusal carries the
// list of what is in view, so the operator can pick again.
inline Decision decide(bool readerOk, const TagRead *seen, int n, const char *chosen) {
  Decision d;
  if (!readerOk) { d.outcome = Outcome::NoReader; return d; }

  for (int i = 0; i < n; i++) {
    if (!epcValid(seen[i].epc)) continue;
    bool earlier = false;
    for (int j = 0; j < i; j++)
      if (epcValid(seen[j].epc) && strcmp(seen[j].epc, seen[i].epc) == 0) { earlier = true; break; }
    if (!earlier) d.seen++;

    // Strongest reading of this EPC in the input.
    TagRead t = seen[i];
    for (int j = 0; j < n; j++)
      if (strcmp(seen[j].epc, t.epc) == 0 && seen[j].rssi > t.rssi) t.rssi = seen[j].rssi;

    bool have = false;
    for (int k = 0; k < d.listCount; k++)
      if (strcmp(d.list[k].epc, t.epc) == 0) { have = true; break; }
    if (have) continue;

    // Keep the SEEN_MAX strongest: insert in order, dropping the weakest if full.
    int pos = d.listCount;
    while (pos > 0 && tagBefore(t, d.list[pos - 1])) pos--;
    if (pos >= SEEN_MAX) continue;
    const int last = (d.listCount < SEEN_MAX) ? d.listCount : SEEN_MAX - 1;
    for (int k = last; k > pos; k--) d.list[k] = d.list[k - 1];
    d.list[pos] = t;
    if (d.listCount < SEEN_MAX) d.listCount++;
  }

  if (chosen) {
    // Against the raw read, not the capped list: a chosen tag past the
    // SEEN_MAX strongest is still in view.
    for (int i = 0; i < n; i++) {
      if (strcmp(seen[i].epc, chosen) == 0 && epcValid(seen[i].epc)) {
        d.outcome = Outcome::Accept;
        d.tag = seen[i];
        for (int j = 0; j < n; j++)
          if (strcmp(seen[j].epc, chosen) == 0 && seen[j].rssi > d.tag.rssi) d.tag.rssi = seen[j].rssi;
        return d;
      }
    }
    d.outcome = Outcome::TagNotInView;
    return d;
  }
  if (d.seen == 0)      d.outcome = Outcome::NoTag;
  else if (d.seen == 1) { d.outcome = Outcome::Accept; d.tag = d.list[0]; }
  else                  d.outcome = Outcome::MultipleTags;
  return d;
}

// ---------------------------------------------------------------------------
// Bounded writer
// ---------------------------------------------------------------------------
struct Writer {
  char  *buf;
  size_t cap;
  size_t len = 0;
  bool   ok  = true;

  Writer(char *b, size_t c) : buf(b), cap(c) {
    if (!buf || cap == 0) ok = false;
    else buf[0] = '\0';
  }
  void ch(char c) {
    if (!ok) return;
    if (len + 1 >= cap) { ok = false; return; }   // keep room for the terminator
    buf[len++] = c;
    buf[len] = '\0';
  }
  void raw(const char *s) { while (ok && *s) ch(*s++); }
  void u64(uint64_t v) {
    char t[21];
    int i = 0;
    do { t[i++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (i) ch(t[--i]);
  }
  void i64(int64_t v) {
    if (v < 0) { ch('-'); u64((uint64_t)(-(v + 1)) + 1); }
    else u64((uint64_t)v);
  }
  void deg7(int32_t v) {
    uint32_t m;
    if (v < 0) { ch('-'); m = (uint32_t)(-(int64_t)v); } else m = (uint32_t)v;
    u64(m / (uint32_t)DEG7);
    ch('.');
    uint32_t f = m % (uint32_t)DEG7;
    char t[7];
    for (int i = 6; i >= 0; i--) { t[i] = (char)('0' + f % 10); f /= 10; }
    for (int i = 0; i < 7; i++) ch(t[i]);
  }
  void boolean(bool b) { raw(b ? "true" : "false"); }
  // A quoted value. Refuses a quote, backslash or control character rather than
  // escaping it: every string this header writes is either a validated ID or
  // EPC, or a firmware constant, so one of those turning up is a defect.
  void str(const char *s) {
    if (!s) { ok = false; return; }
    for (const char *p = s; *p; p++)
      if (*p == '"' || *p == '\\' || (unsigned char)*p < 0x20) { ok = false; return; }
    ch('"'); raw(s); ch('"');
  }
  void key(const char *k) { if (len > 1) ch(','); str(k); ch(':'); }
  int finish() { return ok ? (int)len : -1; }
};

inline void writeFix(Writer &w, const Fix &f) {
  if (f.present) {
    w.key("lat"); w.deg7(f.lat7);
    w.key("lon"); w.deg7(f.lon7);
    w.key("acc"); w.u64(f.accM);
  } else {
    w.key("lat"); w.raw("null");
    w.key("lon"); w.raw("null");
    w.key("acc"); w.raw("null");
  }
}

// ---------------------------------------------------------------------------
// Events: what goes on plc4/jd5055e/weight, live or from the backlog
// ---------------------------------------------------------------------------
enum class Kind : uint8_t { Record, Void, TallyEnd };

inline const char *kindName(Kind k) {
  switch (k) {
    case Kind::Record:   return "record";
    case Kind::Void:     return "void";
    case Kind::TallyEnd: return "tally_end";
  }
  return "record";
}

// One queue entry. Replaces WeightRecord (src/ArduinoHydraulicScale.cpp:328-333)
// and keeps its rule: sensor and cal are facts AT CAPTURE, never read at flush.
struct Event {
  Kind     kind          = Kind::Record;
  uint8_t  sensor        = 0;       // the firmware's LoopState at capture; records only
  bool     calDefault    = false;   // records only
  int32_t  w             = 0;       // records only
  uint32_t capturedAtMs  = 0;
  char     rec[ID_MAX + 1]       = "";   // record, void
  char     tally[ID_MAX + 1]     = "";   // record, tally_end
  char     epc[EPC_HEX_MAX + 1]  = "";   // record
  int64_t  tMs           = 0;
  Fix      fix;
};

inline Event recordEvent(const char *recId, const RecordCmd &c, const TagRead &tag,
                         int32_t w, uint8_t sensor, bool calDefault, uint32_t nowMs) {
  Event e;
  e.kind = Kind::Record;
  e.sensor = sensor;
  e.calDefault = calDefault;
  e.w = w;
  e.capturedAtMs = nowMs;
  recordIdFor(recId, e.rec, sizeof(e.rec));
  memcpy(e.tally, c.tally, sizeof(e.tally));
  memcpy(e.epc, tag.epc, sizeof(e.epc));
  e.tMs = c.tMs;
  e.fix = c.fix;
  return e;
}

inline Event voidEvent(const VoidCmd &c, uint32_t nowMs) {
  Event e;
  e.kind = Kind::Void;
  e.capturedAtMs = nowMs;
  memcpy(e.rec, c.rec, sizeof(e.rec));
  e.tMs = c.tMs;
  e.fix = c.fix;
  return e;
}

inline Event endEvent(const EndCmd &c, uint32_t nowMs) {
  Event e;
  e.kind = Kind::TallyEnd;
  e.capturedAtMs = nowMs;
  memcpy(e.tally, c.tally, sizeof(e.tally));
  e.tMs = c.tMs;
  e.fix = c.fix;
  return e;
}

// `backlog` means what it means today: true when the event was held on the
// machine and published later by the flush, false when published live.
// sensorName is the caller's name for e.sensor (loopStateName() in the
// firmware); it is read only for records. Returns the length, or -1 if the
// payload does not fit `cap` or a field is unusable.
inline int writeEvent(char *buf, size_t cap, const Event &e, uint32_t ageMs, bool backlog,
                      const char *sensorName, const char *clientId) {
  Writer w(buf, cap);
  w.ch('{');
  w.key("kind"); w.str(kindName(e.kind));
  if (e.kind == Kind::Record || e.kind == Kind::Void) {
    if (!idValid(e.rec)) return -1;
    w.key("rec"); w.str(e.rec);
  }
  if (e.kind == Kind::Record || e.kind == Kind::TallyEnd) {
    if (!idValid(e.tally)) return -1;
    w.key("tally"); w.str(e.tally);
  }
  if (e.kind == Kind::Record) {
    if (!epcValid(e.epc)) return -1;
    w.key("w");   w.i64(e.w);
    w.key("tag"); w.str(e.epc);
  }
  w.key("t_ms"); w.i64(e.tMs);
  writeFix(w, e.fix);
  w.key("age_ms");  w.u64(ageMs);
  w.key("backlog"); w.boolean(backlog);
  if (e.kind == Kind::Record) {
    w.key("sensor"); w.str(sensorName);
    w.key("cal");    w.str(e.calDefault ? "DEFAULT" : "ok");
  }
  w.key("id"); w.str(clientId);
  w.ch('}');
  return w.finish();
}

// ---------------------------------------------------------------------------
// Ack
// ---------------------------------------------------------------------------
struct Ack {
  const char    *id     = nullptr;
  const char    *verb   = nullptr;
  bool           ok     = false;
  const char    *detail = nullptr;
  bool           dup    = false;
  const char    *rec    = nullptr;   // + accepted (and its dup), - voided
  const char    *tally  = nullptr;   // + accepted, end ended
  const char    *tag    = nullptr;   // + accepted
  bool           hasW   = false;     // + accepted
  int32_t        w      = 0;
  const TagRead *tags   = nullptr;   // MULTIPLE TAGS, TAG NOT IN VIEW
  int            tagCount = 0;       // entries in `tags`; at most ACK_TAGS_MAX are written
  int            tagsSeen = -1;      // >= 0 writes tags_seen
};

inline int writeAck(char *buf, size_t cap, const Ack &a) {
  Writer w(buf, cap);
  w.ch('{');
  w.key("id");     w.str(a.id);
  w.key("verb");   w.str(a.verb ? a.verb : "");
  w.key("ok");     w.boolean(a.ok);
  w.key("detail"); w.str(a.detail);
  if (a.dup)   { w.key("dup");   w.boolean(true); }
  if (a.rec)   { w.key("rec");   w.str(a.rec); }
  if (a.tally) { w.key("tally"); w.str(a.tally); }
  if (a.tag)   { w.key("tag");   w.str(a.tag); }
  if (a.hasW)  { w.key("w");     w.i64(a.w); }
  if (a.tags) {
    w.key("tags");
    w.ch('[');
    const int n = a.tagCount < ACK_TAGS_MAX ? a.tagCount : ACK_TAGS_MAX;
    for (int i = 0; i < n; i++) {
      if (i) w.ch(',');
      w.raw("{\"epc\":"); w.str(a.tags[i].epc);
      w.raw(",\"rssi\":"); w.i64(a.tags[i].rssi);
      w.ch('}');
    }
    w.ch(']');
  }
  if (a.tagsSeen >= 0) { w.key("tags_seen"); w.i64(a.tagsSeen); }
  w.ch('}');
  return w.finish();
}

// The refusal ack for a tag decision, wired the way the contract specifies.
inline Ack refusalAck(const char *id, const char *verb, const Decision &d) {
  Ack a;
  a.id = id;
  a.verb = verb;
  a.ok = false;
  a.detail = outcomeDetail(d.outcome);
  if (d.outcome == Outcome::MultipleTags || d.outcome == Outcome::TagNotInView) {
    a.tags = d.list;
    a.tagCount = d.listCount;
    a.tagsSeen = d.seen;
  }
  return a;
}

// ---------------------------------------------------------------------------
// Backlog: one FIFO for records, voids and tally ends
// ---------------------------------------------------------------------------
// The weight topic stays at QoS 0, as today (src/ArduinoHydraulicScale.cpp:407).
// Delivery to the consumer is confirmed end to end by the consumer and the app,
// not by the board (Alex, 2026-10-07: "Consumer confirms, app resends").
//
// The flush keeps today's rules (queueFlush(), src/ArduinoHydraulicScale.cpp:534-554):
// oldest first, removed once publishTo() returns true, stop at the first failure.
//
// FULL IS A REFUSAL, NOT A DROP. Today's queue drops the oldest weight when full
// (src/ArduinoHydraulicScale.cpp:515-520). Records are now taken only in range
// and every event is answered by an ack to an operator standing there, so a full
// queue refuses the new event with QUEUE FULL and nothing accepted is discarded.
template <int N>
struct EventQueue {
  Event         buf[N];
  int           head    = 0;
  int           count   = 0;
  unsigned long refused = 0;   // pushes refused because the queue was full

  bool full() const { return count == N; }
  bool push(const Event &e) {
    if (count == N) { refused++; return false; }
    buf[(head + count) % N] = e;
    count++;
    return true;
  }
  const Event *front() const { return count ? &buf[head] : nullptr; }
  void pop() { if (count) { head = (head + 1) % N; count--; } }
};

// THE ORDERING RULE. An event may be published live only when nothing is
// waiting; otherwise it joins the back of the queue and the flush sends it in
// turn. Today's `+` publishes live whenever the board is online, even with a
// backlog (src/ArduinoHydraulicScale.cpp:1919-1924) -- harmless for weights,
// wrong once a void can overtake the record it cancels.
inline bool mustQueue(bool online, int queued) { return !online || queued > 0; }

}  // namespace rfid

#endif  // RFID_RECORD_H
