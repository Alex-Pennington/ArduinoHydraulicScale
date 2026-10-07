// Host check of the RFID record / tally contract logic in include/rfid_record.h.
//
// Built with g++ ON THE HOST, like test_loop_scaling.cpp. The header is a
// gated board proposal (notes/2026-10-07-rfid-record-contract.md); nothing in src/
// includes it; this proves its logic, not anything about a reader, a tag or a
// tractor.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_rfid_record.cpp -o /tmp/r
//   /tmp/r
//
// Exit status is the failure count.

#include "rfid_record.h"
#include <cstdio>
#include <cstring>

using namespace rfid;

static int failures = 0;

static void check(const char *what, bool cond) {
  if (cond) printf("  ok    %s\n", what);
  else { printf("  FAIL  %s\n", what); failures++; }
}

static void check_int(const char *what, long got, long want) {
  if (got == want) printf("  ok    %-52s %ld\n", what, got);
  else { printf("  FAIL  %-52s got %ld want %ld\n", what, got, want); failures++; }
}

static void check_str(const char *what, const char *got, const char *want) {
  if (got && want && strcmp(got, want) == 0) printf("  ok    %-52s %s\n", what, got);
  else { printf("  FAIL  %-52s\n        got  %s\n        want %s\n", what,
                got ? got : "(null)", want ? want : "(null)"); failures++; }
}

static TagRead tag(const char *epc, int rssi) {
  TagRead t;
  snprintf(t.epc, sizeof(t.epc), "%s", epc);
  t.rssi = (int16_t)rssi;
  return t;
}

// 96-bit EPCs, the length stock tags commonly ship with.
static const char *A = "E28011700000020F1A2B3C01";
static const char *B = "E28011700000020F1A2B3C02";
static const char *C = "E28011700000020F1A2B3C03";

// A cut-down model of the board's command path, enough to show what the record
// ID scheme does across a reboot. The dedup ring is RAM, eight deep, as today
// (src/ArduinoHydraulicScale.cpp:742-751).
struct Board {
  char ring[8][ID_MAX + 1];
  int  ringCount = 0, ringNext = 0;
  unsigned ramSeq = 0;   // the REJECTED scheme: a sequence counter with no persistence
  void reboot() { ringCount = 0; ringNext = 0; ramSeq = 0; }
  bool seen(const char *id) const {
    for (int i = 0; i < ringCount; i++) if (!strcmp(ring[i], id)) return true;
    return false;
  }
  // Returns true if the command executed (and so published a record).
  bool record(const char *cmdId, char *recOut, size_t cap, char *ramIdOut) {
    if (seen(cmdId)) return false;
    snprintf(ring[ringNext], sizeof(ring[ringNext]), "%s", cmdId);
    ringNext = (ringNext + 1) % 8;
    if (ringCount < 8) ringCount++;
    recordIdFor(cmdId, recOut, cap);
    snprintf(ramIdOut, 16, "r%u", ++ramSeq);
    return true;
  }
};

int main() {
  // -------------------------------------------------------------------------
  printf("validators\n");
  check("id: uuid accepted", idValid("3f2a9c21-7b4e-4c1d-9a0e-5d6f7a8b9c0d"));
  check("id: empty refused", !idValid(""));
  check("id: 40 chars accepted", idValid("0123456789012345678901234567890123456789"));
  check("id: 41 chars refused", !idValid("01234567890123456789012345678901234567890"));
  check("id: quote refused", !idValid("ab\"c"));
  check("id: space refused", !idValid("ab c"));
  check("epc: 96-bit accepted", epcValid(A));
  check("epc: lower-case refused", !epcValid("e28011700000020f1a2b3c01"));
  check("epc: not a whole word refused", !epcValid("E2801"));
  check("epc: 128-bit accepted", epcValid("E28011700000020F1A2B3C01DEADBEEF"));
  check("epc: 144-bit refused", !epcValid("E28011700000020F1A2B3C01DEADBEEF0000"));

  // -------------------------------------------------------------------------
  printf("decision: every outcome\n");
  {
    Decision d = decide(false, nullptr, 0, nullptr);
    check_str("reader down -> NO READER", outcomeDetail(d.outcome), D_NO_READER);
  }
  {
    Decision d = decide(true, nullptr, 0, nullptr);
    check_str("nothing in view -> NO TAG", outcomeDetail(d.outcome), D_NO_TAG);
    check_int("  seen", d.seen, 0);
  }
  {
    TagRead s[] = {tag(A, -55)};
    Decision d = decide(true, s, 1, nullptr);
    check_str("one tag -> recorded", outcomeDetail(d.outcome), D_RECORDED);
    check_str("  tag recorded", d.tag.epc, A);
  }
  {
    // The same tag reported three times in one inventory round is one tag.
    TagRead s[] = {tag(A, -60), tag(A, -52), tag(A, -58)};
    Decision d = decide(true, s, 3, nullptr);
    check_str("one tag read three times -> recorded", outcomeDetail(d.outcome), D_RECORDED);
    check_int("  strongest reading kept", d.tag.rssi, -52);
    check_int("  seen", d.seen, 1);
  }
  {
    TagRead s[] = {tag("garbage", -40), tag(A, -60)};
    Decision d = decide(true, s, 2, nullptr);
    check_str("invalid EPC ignored, one valid -> recorded", outcomeDetail(d.outcome), D_RECORDED);
  }
  {
    TagRead s[] = {tag(A, -70), tag(B, -45), tag(C, -60)};
    Decision d = decide(true, s, 3, nullptr);
    check_str("three tags -> MULTIPLE TAGS", outcomeDetail(d.outcome), D_MULTIPLE);
    check_int("  list count", d.listCount, 3);
    check_str("  strongest first", d.list[0].epc, B);
    check_str("  then", d.list[1].epc, C);
    check_str("  weakest last", d.list[2].epc, A);
  }

  // -------------------------------------------------------------------------
  printf("decision: RSSI ordering and ties\n");
  {
    // Equal signal sorts by EPC, whatever order the reader reported them in.
    TagRead s1[] = {tag(C, -50), tag(A, -50), tag(B, -50)};
    TagRead s2[] = {tag(B, -50), tag(C, -50), tag(A, -50)};
    Decision d1 = decide(true, s1, 3, nullptr);
    Decision d2 = decide(true, s2, 3, nullptr);
    check_str("tie: first by EPC", d1.list[0].epc, A);
    check_str("tie: second by EPC", d1.list[1].epc, B);
    check_str("tie: third by EPC", d1.list[2].epc, C);
    bool same = true;
    for (int i = 0; i < 3; i++) same = same && !strcmp(d1.list[i].epc, d2.list[i].epc);
    check("tie: order independent of report order", same);
  }
  {
    // A duplicate's strongest reading decides its place.
    TagRead s[] = {tag(A, -70), tag(B, -55), tag(A, -40)};
    Decision d = decide(true, s, 3, nullptr);
    check_str("duplicate ranked at its strongest", d.list[0].epc, A);
    check_int("  at", d.list[0].rssi, -40);
    check_int("  list has no duplicate", d.listCount, 2);
  }
  {
    // More distinct tags than SEEN_MAX: the strongest SEEN_MAX are kept, and
    // tags_seen still counts them all.
    TagRead s[SEEN_MAX + 4];
    for (int i = 0; i < SEEN_MAX + 4; i++) {
      char e[EPC_HEX_MAX + 1];
      snprintf(e, sizeof(e), "E2801170000000000000%04X", i);
      s[i] = tag(e, -80 + i);          // later entries are stronger
    }
    Decision d = decide(true, s, SEEN_MAX + 4, nullptr);
    check_int("overflow: seen counts every distinct tag", d.seen, SEEN_MAX + 4);
    check_int("overflow: list capped", d.listCount, SEEN_MAX);
    check_int("overflow: strongest kept first", d.list[0].rssi, -80 + SEEN_MAX + 3);
    check_int("overflow: weakest kept", d.list[SEEN_MAX - 1].rssi, -80 + 4);
    bool sorted = true;
    for (int i = 1; i < d.listCount; i++) sorted = sorted && !tagBefore(d.list[i], d.list[i - 1]);
    check("overflow: list sorted", sorted);
  }

  // -------------------------------------------------------------------------
  printf("decision: chosen-tag retry\n");
  {
    TagRead s[] = {tag(A, -70), tag(B, -45)};
    Decision d = decide(true, s, 2, A);
    check_str("chosen weaker tag in view -> recorded", outcomeDetail(d.outcome), D_RECORDED);
    check_str("  the chosen one, not the strongest", d.tag.epc, A);
  }
  {
    TagRead s[] = {tag(B, -45)};
    Decision d = decide(true, s, 1, A);
    check_str("chosen tag gone, another in view -> TAG NOT IN VIEW",
              outcomeDetail(d.outcome), D_NOT_IN_VIEW);
    check_int("  list says what is in view", d.listCount, 1);
    check_str("  which is", d.list[0].epc, B);
  }
  {
    Decision d = decide(true, nullptr, 0, A);
    check_str("chosen tag, nothing in view -> TAG NOT IN VIEW",
              outcomeDetail(d.outcome), D_NOT_IN_VIEW);
  }
  {
    // The chosen tag is weaker than SEEN_MAX others: still in view, still accepted.
    TagRead s[SEEN_MAX + 1];
    for (int i = 0; i < SEEN_MAX; i++) {
      char e[EPC_HEX_MAX + 1];
      snprintf(e, sizeof(e), "E2801170000000000000%04X", i);
      s[i] = tag(e, -40);
    }
    s[SEEN_MAX] = tag(A, -90);
    Decision d = decide(true, s, SEEN_MAX + 1, A);
    check_str("chosen tag below the capped list -> recorded", outcomeDetail(d.outcome), D_RECORDED);
  }
  {
    TagRead s[] = {tag(A, -50)};
    Decision d = decide(false, s, 1, A);
    check_str("chosen tag, reader down -> NO READER", outcomeDetail(d.outcome), D_NO_READER);
  }

  // -------------------------------------------------------------------------
  printf("record command\n");
  {
    RecordCmd c;
    const char *why = parseRecord(
        "{\"id\":\"r1\",\"verb\":\"+\",\"tally\":\"T-day-1007\",\"t_ms\":1791398400123,"
        "\"lat\":44.1234567,\"lon\":-72.7654321,\"acc\":4.2}", &c);
    check("full command accepted", why == nullptr);
    check_str("  tally", c.tally, "T-day-1007");
    check_int("  t_ms low part", (long)(c.tMs % 1000000), 400123);
    check_int("  lat7", c.fix.lat7, 441234567);
    check_int("  lon7", c.fix.lon7, -727654321);
    check_int("  acc rounded up", c.fix.accM, 5);
    check("  no chosen tag", !c.hasTag);
  }
  {
    RecordCmd c;
    const char *why = parseRecord(
        "{\"id\":\"r2\",\"verb\":\"+\",\"tally\":\"T1\",\"t_ms\":1791398400123,\"tag\":\"E28011700000020F1A2B3C01\"}", &c);
    check("no GPS, chosen tag: accepted", why == nullptr);
    check("  fix absent", !c.fix.present);
    check("  chosen tag carried", c.hasTag && !strcmp(c.tag, A));
  }
  {
    RecordCmd c;
    check_str("lat/lon/acc null -> no fix, accepted",
              parseRecord("{\"tally\":\"T1\",\"t_ms\":5,\"lat\":null,\"lon\":null,\"acc\":null}", &c)
                  ? "refused" : "accepted", "accepted");
    check("  fix absent", !c.fix.present);
  }
  {
    RecordCmd c;
    check_str("+ no tally", parseRecord("{\"t_ms\":5}", &c), D_NO_TALLY);
    check_str("+ tally not a token", parseRecord("{\"tally\":\"a b\",\"t_ms\":5}", &c), D_BAD_TALLY);
    check_str("+ tally is a number", parseRecord("{\"tally\":7,\"t_ms\":5}", &c), D_BAD_TALLY);
    check_str("+ no time", parseRecord("{\"tally\":\"T1\"}", &c), D_NO_TIME);
    check_str("+ time as string", parseRecord("{\"tally\":\"T1\",\"t_ms\":\"5\"}", &c), D_BAD_TIME);
    check_str("+ time fractional", parseRecord("{\"tally\":\"T1\",\"t_ms\":5.5}", &c), D_BAD_TIME);
    check_str("+ time zero", parseRecord("{\"tally\":\"T1\",\"t_ms\":0}", &c), D_BAD_TIME);
    check_str("+ lat without lon/acc",
              parseRecord("{\"tally\":\"T1\",\"t_ms\":5,\"lat\":44.1}", &c), D_BAD_GPS);
    check_str("+ lat out of range",
              parseRecord("{\"tally\":\"T1\",\"t_ms\":5,\"lat\":91,\"lon\":0,\"acc\":3}", &c), D_BAD_GPS);
    check_str("+ lon in exponent form",
              parseRecord("{\"tally\":\"T1\",\"t_ms\":5,\"lat\":1,\"lon\":1e-7,\"acc\":3}", &c), D_BAD_GPS);
    check_str("+ negative accuracy",
              parseRecord("{\"tally\":\"T1\",\"t_ms\":5,\"lat\":1,\"lon\":1,\"acc\":-3}", &c), D_BAD_GPS);
    check_str("+ chosen tag lower-case",
              parseRecord("{\"tally\":\"T1\",\"t_ms\":5,\"tag\":\"e2801170\"}", &c), D_BAD_TAG);
    check_str("+ tally checked before time",
              parseRecord("{\"lat\":1}", &c), D_NO_TALLY);
  }
  {
    // Fraction digits past 1e-7 are truncated, not refused: phones report them.
    int32_t v;
    check("lat with 15 decimals parses", parseDeg7("44.123456789012345", 90, &v));
    check_int("  truncated to 1e-7", v, 441234567);
    check("lat -0.5", parseDeg7("-0.5", 90, &v) && v == -5000000);
    check("lat exactly 90 accepted", parseDeg7("90", 90, &v));
    check("lat 90.0000001 refused", !parseDeg7("90.0000001", 90, &v));
    check("bare '.' refused", !parseDeg7(".", 90, &v));
    check("'1.' refused", !parseDeg7("1.", 90, &v));
    uint16_t a;
    check("acc 3 -> 3", parseAccM("3", &a) && a == 3);
    check("acc 3.0 -> 3", parseAccM("3.0", &a) && a == 3);
    check("acc 3.01 -> 4", parseAccM("3.01", &a) && a == 4);
    check("acc 70000 refused", !parseAccM("70000", &a));
  }

  printf("void and tally-end commands\n");
  {
    VoidCmd v;
    check("void with rec and time", parseVoid("{\"rec\":\"r1\",\"t_ms\":7}", &v) == nullptr);
    check_str("  rec", v.rec, "r1");
    check_str("- no record id", parseVoid("{\"t_ms\":7}", &v), D_NO_REC);
    check_str("- bad record id", parseVoid("{\"rec\":\"r 1\",\"t_ms\":7}", &v), D_BAD_REC);
    check_str("- no time", parseVoid("{\"rec\":\"r1\"}", &v), D_NO_TIME);
    EndCmd e;
    check("end with tally and time", parseEnd("{\"tally\":\"T1\",\"t_ms\":7}", &e) == nullptr);
    check_str("end no tally", parseEnd("{\"t_ms\":7}", &e), D_NO_TALLY);
    check_str("end bad GPS",
              parseEnd("{\"tally\":\"T1\",\"t_ms\":7,\"lat\":1,\"lon\":1}", &e), D_BAD_GPS);
  }

  printf("field extraction edges\n");
  {
    char out[8];
    check("string oversize refused, not truncated",
          stringField("{\"tally\":\"123456789\"}", "tally", out, sizeof(out)) == Field::Bad &&
          out[0] == '\0');
    check("string with escape refused",
          stringField("{\"tally\":\"a\\\"b\"}", "tally", out, sizeof(out)) == Field::Bad);
    check("key matched exactly, not as a prefix",
          stringField("{\"tallyx\":\"A\"}", "tally", out, sizeof(out)) == Field::Absent);
    check("value equal to a key name is not a key",
          stringField("{\"rec\":\"tally\",\"x\":1}", "tally", out, sizeof(out)) == Field::Absent);
    check("number followed by junk refused",
          numberToken("{\"t_ms\":5x}", "t_ms", out, sizeof(out)) == Field::Bad);
  }

  // -------------------------------------------------------------------------
  printf("record ID across a simulated reboot\n");
  {
    Board b;
    char rec1[ID_MAX + 1], rec2[ID_MAX + 1], rec3[ID_MAX + 1];
    char ram1[16], ram2[16], ram3[16];
    check("first record executes", b.record("app-0001", rec1, sizeof(rec1), ram1));
    check("resend before reboot is deduplicated", !b.record("app-0001", rec2, sizeof(rec2), ram2));
    b.reboot();
    check("next record after reboot executes", b.record("app-0002", rec2, sizeof(rec2), ram2));
    check("record IDs differ across the reboot", strcmp(rec1, rec2) != 0);
    // The rejected scheme: a RAM sequence restarts at 1 and reuses an ID.
    check("rejected RAM-sequence scheme collides across the reboot", strcmp(ram1, ram2) == 0);
    // A resend after the reboot re-executes (the dedup ring is RAM), but under
    // this scheme it carries the SAME record ID, so the consumer can drop it.
    check("resend after reboot re-executes", b.record("app-0001", rec3, sizeof(rec3), ram3));
    check_str("  with the original record ID", rec3, rec1);
    char small[4];
    check("record ID longer than the buffer refused", !recordIdFor("app-0001", small, sizeof(small)));
    check("record ID from an invalid command id refused", !recordIdFor("a\"b", rec3, sizeof(rec3)));
  }

  // -------------------------------------------------------------------------
  printf("payload shape\n");
  RecordCmd rc;
  parseRecord("{\"tally\":\"T-day-1007\",\"t_ms\":1791398400123,"
              "\"lat\":44.1234567,\"lon\":-72.7654321,\"acc\":5}", &rc);
  const Event rec = recordEvent("3f2a9c21", rc, tag(A, -52), 1834, 0, false, 1000);
  char buf[512];
  {
    int n = writeEvent(buf, sizeof(buf), rec, 0, false, "ok", "plc4-jd5055e");
    check_str("live record",
              buf,
              "{\"kind\":\"record\",\"rec\":\"3f2a9c21\",\"tally\":\"T-day-1007\",\"w\":1834,"
              "\"tag\":\"E28011700000020F1A2B3C01\",\"t_ms\":1791398400123,\"lat\":44.1234567,"
              "\"lon\":-72.7654321,\"acc\":5,\"age_ms\":0,\"backlog\":false,\"sensor\":\"ok\","
              "\"cal\":\"ok\",\"id\":\"plc4-jd5055e\"}");
    check_int("  length returned", n, (long)strlen(buf));
  }
  {
    RecordCmd nofix = rc;
    nofix.fix = Fix{};
    Event e = recordEvent("3f2a9c22", nofix, tag(A, -52), -5, 0, true, 1000);
    writeEvent(buf, sizeof(buf), e, 42235, true, "ok", "plc4-jd5055e");
    check_str("backlog record, no fix, negative weight, DEFAULT cal",
              buf,
              "{\"kind\":\"record\",\"rec\":\"3f2a9c22\",\"tally\":\"T-day-1007\",\"w\":-5,"
              "\"tag\":\"E28011700000020F1A2B3C01\",\"t_ms\":1791398400123,\"lat\":null,"
              "\"lon\":null,\"acc\":null,\"age_ms\":42235,\"backlog\":true,\"sensor\":\"ok\","
              "\"cal\":\"DEFAULT\",\"id\":\"plc4-jd5055e\"}");
  }
  {
    VoidCmd vc;
    parseVoid("{\"rec\":\"3f2a9c21\",\"t_ms\":1791398460000,\"lat\":-0.0000001,\"lon\":0,\"acc\":3}", &vc);
    writeEvent(buf, sizeof(buf), voidEvent(vc, 2000), 0, false, "ok", "plc4-jd5055e");
    check_str("void",
              buf,
              "{\"kind\":\"void\",\"rec\":\"3f2a9c21\",\"t_ms\":1791398460000,\"lat\":-0.0000001,"
              "\"lon\":0.0000000,\"acc\":3,\"age_ms\":0,\"backlog\":false,\"id\":\"plc4-jd5055e\"}");
  }
  {
    EndCmd ec;
    parseEnd("{\"tally\":\"T-day-1007\",\"t_ms\":1791420000000}", &ec);
    writeEvent(buf, sizeof(buf), endEvent(ec, 3000), 0, false, "ok", "plc4-jd5055e");
    check_str("tally end",
              buf,
              "{\"kind\":\"tally_end\",\"tally\":\"T-day-1007\",\"t_ms\":1791420000000,"
              "\"lat\":null,\"lon\":null,\"acc\":null,\"age_ms\":0,\"backlog\":false,"
              "\"id\":\"plc4-jd5055e\"}");
  }
  {
    Ack a;
    a.id = "3f2a9c21"; a.verb = "+"; a.ok = true; a.detail = D_RECORDED;
    a.rec = "3f2a9c21"; a.tally = "T-day-1007"; a.tag = A; a.hasW = true; a.w = 1834;
    writeAck(buf, sizeof(buf), a);
    check_str("ack: recorded",
              buf,
              "{\"id\":\"3f2a9c21\",\"verb\":\"+\",\"ok\":true,\"detail\":\"recorded\","
              "\"rec\":\"3f2a9c21\",\"tally\":\"T-day-1007\",\"tag\":\"E28011700000020F1A2B3C01\","
              "\"w\":1834}");
  }
  {
    TagRead s[] = {tag(A, -70), tag(B, -45)};
    Decision d = decide(true, s, 2, nullptr);
    writeAck(buf, sizeof(buf), refusalAck("7f3a0001", "+", d));
    check_str("ack: MULTIPLE TAGS, strongest first",
              buf,
              "{\"id\":\"7f3a0001\",\"verb\":\"+\",\"ok\":false,\"detail\":\"MULTIPLE TAGS\","
              "\"tags\":[{\"epc\":\"E28011700000020F1A2B3C02\",\"rssi\":-45},"
              "{\"epc\":\"E28011700000020F1A2B3C01\",\"rssi\":-70}],\"tags_seen\":2}");
  }
  {
    Decision d = decide(true, nullptr, 0, A);
    writeAck(buf, sizeof(buf), refusalAck("7f3a0002", "+", d));
    check_str("ack: TAG NOT IN VIEW, empty list",
              buf,
              "{\"id\":\"7f3a0002\",\"verb\":\"+\",\"ok\":false,\"detail\":\"TAG NOT IN VIEW\","
              "\"tags\":[],\"tags_seen\":0}");
  }
  {
    Decision d = decide(true, nullptr, 0, nullptr);
    writeAck(buf, sizeof(buf), refusalAck("7f3a0003", "+", d));
    check_str("ack: NO TAG carries no list",
              buf, "{\"id\":\"7f3a0003\",\"verb\":\"+\",\"ok\":false,\"detail\":\"NO TAG\"}");
  }
  {
    Ack a;
    a.id = "7f3a0004"; a.verb = "+"; a.ok = false; a.detail = D_NO_TALLY;
    writeAck(buf, sizeof(buf), a);
    check_str("ack: NO TALLY",
              buf, "{\"id\":\"7f3a0004\",\"verb\":\"+\",\"ok\":false,\"detail\":\"NO TALLY\"}");
  }
  {
    Ack a;
    a.id = "3f2a9c21"; a.verb = "+"; a.ok = true; a.detail = D_RECORDED; a.dup = true;
    a.rec = "3f2a9c21";
    writeAck(buf, sizeof(buf), a);
    check_str("ack: duplicate of a record still names it",
              buf,
              "{\"id\":\"3f2a9c21\",\"verb\":\"+\",\"ok\":true,\"detail\":\"recorded\","
              "\"dup\":true,\"rec\":\"3f2a9c21\"}");
  }
  {
    Ack a;
    a.id = "v-0001"; a.verb = "-"; a.ok = true; a.detail = D_VOIDED; a.rec = "3f2a9c21";
    writeAck(buf, sizeof(buf), a);
    check_str("ack: voided",
              buf, "{\"id\":\"v-0001\",\"verb\":\"-\",\"ok\":true,\"detail\":\"voided\",\"rec\":\"3f2a9c21\"}");
    Ack e;
    e.id = "e-0001"; e.verb = "end"; e.ok = true; e.detail = D_ENDED; e.tally = "T-day-1007";
    writeAck(buf, sizeof(buf), e);
    check_str("ack: ended",
              buf, "{\"id\":\"e-0001\",\"verb\":\"end\",\"ok\":true,\"detail\":\"ended\",\"tally\":\"T-day-1007\"}");
  }
  {
    // More tags than the ack lists: capped at ACK_TAGS_MAX, tags_seen tells the truth.
    TagRead s[12];
    for (int i = 0; i < 12; i++) {
      char e[EPC_HEX_MAX + 1];
      snprintf(e, sizeof(e), "E2801170000000000000%04X", i);
      s[i] = tag(e, -60);
    }
    Decision d = decide(true, s, 12, nullptr);
    writeAck(buf, sizeof(buf), refusalAck("7f3a0005", "+", d));
    int objs = 0;
    for (const char *p = buf; (p = strstr(p, "\"epc\"")); p++) objs++;
    check_int("ack: tag list capped", objs, ACK_TAGS_MAX);
    check("ack: tags_seen is the full count", strstr(buf, "\"tags_seen\":12}") != nullptr);
  }

  // -------------------------------------------------------------------------
  printf("buffer too small\n");
  {
    int full = writeEvent(buf, sizeof(buf), rec, 0, false, "ok", "plc4-jd5055e");
    char tight[512];
    check_int("exact fit (len + 1) succeeds",
              writeEvent(tight, (size_t)full + 1, rec, 0, false, "ok", "plc4-jd5055e"), full);
    check_int("one byte short refused",
              writeEvent(tight, (size_t)full, rec, 0, false, "ok", "plc4-jd5055e"), -1);
    check("  and the buffer stays terminated", strlen(tight) < (size_t)full);
    check_int("zero-size buffer refused",
              writeEvent(tight, 0, rec, 0, false, "ok", "plc4-jd5055e"), -1);
    TagRead s[] = {tag(A, -70), tag(B, -45)};
    Decision d = decide(true, s, 2, nullptr);
    char small[64];
    check_int("multi-tag ack into 64 bytes refused",
              writeAck(small, sizeof(small), refusalAck("7f3a0001", "+", d)), -1);
    check_int("sensor name with a quote refused",
              writeEvent(buf, sizeof(buf), rec, 0, false, "o\"k", "plc4-jd5055e"), -1);
    Event bad = rec;
    bad.epc[0] = '\0';
    check_int("record with no tag refused", writeEvent(buf, sizeof(buf), bad, 0, false, "ok", "x"), -1);
  }

  // -------------------------------------------------------------------------
  printf("worst-case sizes against the proposed UNO R4 WiFi buffers\n");
  {
    const char *id40 = "0123456789012345678901234567890123456789";
    RecordCmd w;
    snprintf(w.tally, sizeof(w.tally), "%s", id40);
    w.tMs = 999999999999999LL;
    w.fix.present = true; w.fix.lat7 = -900000000; w.fix.lon7 = -1800000000; w.fix.accM = 65535;
    TagRead t = tag("E28011700000020F1A2B3C01DEADBEEF", -128);
    Event e = recordEvent(id40, w, t, -2147483647 - 1, 0, true, 0);
    int n = writeEvent(buf, sizeof(buf), e, 4294967295u, true, "XMTR FAULT", "plc4-jd5055e");
    printf("  info  worst-case record payload                         %d bytes\n", n);
    check("worst-case record fits EVENT_BUF", n > 0 && (size_t)n < EVENT_BUF);

    char cmd[512];
    int cl = snprintf(cmd, sizeof(cmd),
        "{\"id\":\"%s\",\"verb\":\"+\",\"tally\":\"%s\",\"t_ms\":999999999999999,"
        "\"lat\":-90.0000000,\"lon\":-180.0000000,\"acc\":65535,\"tag\":\"%s\"}",
        id40, id40, "E28011700000020F1A2B3C01DEADBEEF");
    printf("  info  worst-case record command                         %d bytes\n", cl);
    check("worst-case record command exceeds today's 191-byte capture",
          cl > 191);   // CmdCapture.body[192], src/ArduinoHydraulicScale.cpp:953
    RecordCmd parsed;
    check("  and parses", parseRecord(cmd, &parsed) == nullptr);
    check("worst-case record command fits CMD_BUF", (size_t)cl < CMD_BUF);

    TagRead s[ACK_TAGS_MAX];
    for (int i = 0; i < ACK_TAGS_MAX; i++) {
      char ep[EPC_HEX_MAX + 1];
      snprintf(ep, sizeof(ep), "E28011700000020F1A2B3C01DEAD%04X", i);
      s[i] = tag(ep, -128);
    }
    Decision d = decide(true, s, ACK_TAGS_MAX, nullptr);
    char big[1024];
    int an = writeAck(big, sizeof(big), refusalAck(id40, "+", d));
    printf("  info  worst-case multi-tag ack                          %d bytes\n", an);
    check("worst-case ack fits ACK_BUF", an > 0 && (size_t)an < ACK_BUF);
    // Today's ack buffer is 192 (src/ArduinoHydraulicScale.cpp:912): too small
    // for any multi-tag refusal, so the firmware change must grow it.
    check("worst-case ack exceeds today's 192-byte ack buffer", an > 191);

    printf("  info  sizeof(Event), one backlog entry                   %zu bytes\n", sizeof(Event));
    printf("  info  sizeof(EventQueue<QUEUE_DEPTH=%d>)                 %zu bytes\n",
           QUEUE_DEPTH, sizeof(EventQueue<QUEUE_DEPTH>));
    printf("  info  200 entries (today's depth), for comparison        %zu bytes\n", 200 * sizeof(Event));
    printf("  info  sizeof(Decision), on the stack in the record path  %zu bytes\n", sizeof(Decision));
    // Ack.tags points INTO the Decision, so on a refusal both are live while
    // the ack is written: the stack cost is the sum, not the larger of the two.
    printf("  info  Decision + ACK_BUF, live together on a refusal     %zu bytes\n",
           sizeof(Decision) + ACK_BUF);
    // Today's 200 x 12-byte WeightRecord (src:328-343) is 2400 bytes. The new
    // queue must not cost materially more than the one it replaces.
    check("queue at QUEUE_DEPTH costs no more than today's 2400 + 64",
          sizeof(EventQueue<QUEUE_DEPTH>) <= 2400 + 64);
  }

  // -------------------------------------------------------------------------
  printf("backlog: order, refuse when full\n");
  {
    check("online, empty queue: publish live", !mustQueue(true, 0));
    check("online, queue waiting: queue behind it", mustQueue(true, 1));
    check("offline: queue", mustQueue(false, 0));

    EventQueue<4> q;
    VoidCmd vc;  parseVoid("{\"rec\":\"3f2a9c21\",\"t_ms\":2}", &vc);
    EndCmd ec;   parseEnd("{\"tally\":\"T-day-1007\",\"t_ms\":3}", &ec);
    q.push(rec);
    q.push(voidEvent(vc, 1));
    q.push(endEvent(ec, 2));
    check("flush order 1: record", q.front() && q.front()->kind == Kind::Record);   q.pop();
    check("flush order 2: void", q.front() && q.front()->kind == Kind::Void);       q.pop();
    check("flush order 3: tally end", q.front() && q.front()->kind == Kind::TallyEnd); q.pop();
    check("empty after flush", q.front() == nullptr && q.count == 0);

    EventQueue<2> f;
    check("push 1 accepted", f.push(rec));
    check("push 2 accepted", f.push(voidEvent(vc, 1)));
    check("full: push 3 refused", !f.push(endEvent(ec, 2)));
    check("  full() reports it", f.full());
    check_int("  refusal counted", (long)f.refused, 1);
    check("  nothing accepted was dropped: record still first", f.front()->kind == Kind::Record);
    check_int("  count unchanged", f.count, 2);
    check("QUEUE FULL fits the 20-char detail", strlen(D_QUEUE_FULL) <= 20);

    EventQueue<3> w;
    w.push(rec); w.push(voidEvent(vc, 1));
    w.pop();
    w.push(endEvent(ec, 2)); w.push(rec);
    check("wrap: order kept 1", w.front()->kind == Kind::Void);     w.pop();
    check("wrap: order kept 2", w.front()->kind == Kind::TallyEnd); w.pop();
    check("wrap: order kept 3", w.front()->kind == Kind::Record);   w.pop();
    check("wrap: empty", w.count == 0);
  }

  // -------------------------------------------------------------------------
  // The app's resend of an unconfirmed void or tally end. A model of the
  // board's command path: the eight-deep dedup ring (src:742-751), which
  // replays a stored verdict for a repeated id WITHOUT re-running the verb
  // (src:1147-1152), then the parse, then live publish or queue.
  printf("app resend of an unconfirmed void or tally end\n");
  {
    struct ResendBoard {
      char ring[8][ID_MAX + 1];
      int  ringCount = 0, ringNext = 0;
      bool online = true;
      EventQueue<QUEUE_DEPTH> q;
      char wire[8][EVENT_BUF];
      int  nWire = 0;
      bool seen(const char *id) const {
        for (int i = 0; i < ringCount; i++) if (!strcmp(ring[i], id)) return true;
        return false;
      }
      void publish(const Event &e, bool backlog) {
        if (nWire < 8) writeEvent(wire[nWire++], EVENT_BUF, e, 0, backlog, "ok", "plc4-jd5055e");
      }
      // Returns the ack detail, or "dup" for a replayed verdict.
      const char *command(const char *id, const char *verb, const char *body) {
        if (seen(id)) return "dup";
        snprintf(ring[ringNext], sizeof(ring[ringNext]), "%s", id);
        ringNext = (ringNext + 1) % 8;
        if (ringCount < 8) ringCount++;
        Event e;
        if (!strcmp(verb, "-")) {
          VoidCmd c;
          if (const char *why = parseVoid(body, &c)) return why;
          e = voidEvent(c, 0);
        } else {
          EndCmd c;
          if (const char *why = parseEnd(body, &c)) return why;
          e = endEvent(c, 0);
        }
        if (mustQueue(online, q.count)) { if (!q.push(e)) return D_QUEUE_FULL; }
        else publish(e, false);
        return strcmp(verb, "-") ? D_ENDED : D_VOIDED;
      }
      void flush() {
        while (online && q.front()) { publish(*q.front(), true); q.pop(); }
      }
    } b;

    // The app resends with the SAME rec / tally, t_ms and GPS, and a NEW id.
    const char *vbody = "{\"rec\":\"3f2a9c21\",\"t_ms\":1791398460000}";
    check_str("void, first send", b.command("v-1", "-", vbody), D_VOIDED);
    check_int("  published once", b.nWire, 1);
    check_str("resend with the SAME id: dedup replay", b.command("v-1", "-", vbody), "dup");
    check_int("  and NOT published again (the trap)", b.nWire, 1);
    check_str("resend with a NEW id", b.command("v-2", "-", vbody), D_VOIDED);
    check_int("  published again", b.nWire, 2);
    check_str("  identical payload, so the consumer can match it", b.wire[1], b.wire[0]);

    const char *ebody = "{\"tally\":\"T-day-1007\",\"t_ms\":1791420000000}";
    check_str("tally end, first send", b.command("e-1", "end", ebody), D_ENDED);
    check_str("resend with the SAME id: dedup replay", b.command("e-1", "end", ebody), "dup");
    check_int("  not published again", b.nWire, 3);
    check_str("resend with a NEW id", b.command("e-2", "end", ebody), D_ENDED);
    check_int("  published again", b.nWire, 4);
    check_str("  identical payload", b.wire[3], b.wire[2]);

    // Offline: the original and the resend both queue, and flush in order.
    b.online = false;
    check_str("void while offline: queued", b.command("v-3", "-", vbody), D_VOIDED);
    check_str("resend while still offline: queued too", b.command("v-4", "-", vbody), D_VOIDED);
    check_int("  two in the queue", b.q.count, 2);
    check_int("  nothing on the wire yet", b.nWire, 4);
    b.online = true;
    b.flush();
    check_int("reconnect: both flushed", b.nWire, 6);
    check("  marked backlog:true, as today", strstr(b.wire[5], "\"backlog\":true") != nullptr);
  }

  // A model of the consumer's rule, to show the duplicates above do no harm:
  // records keyed by rec, voids idempotent on rec, tally ends on tally.
  printf("consumer drops duplicates\n");
  {
    struct Consumer {
      char recs[8][ID_MAX + 1]; long w[8]; bool voided[8]; int n = 0;
      char ended[4][ID_MAX + 1]; int nEnded = 0;
      int dupDropped = 0;
      int find(const char *r) const { for (int i = 0; i < n; i++) if (!strcmp(recs[i], r)) return i; return -1; }
      void record(const char *r, long wt) {
        if (find(r) >= 0) { dupDropped++; return; }
        snprintf(recs[n], sizeof(recs[n]), "%s", r); w[n] = wt; voided[n] = false; n++;
      }
      void voidRec(const char *r) {
        const int i = find(r);
        if (i < 0 || voided[i]) { dupDropped++; return; }
        voided[i] = true;
      }
      void end(const char *t) {
        for (int i = 0; i < nEnded; i++) if (!strcmp(ended[i], t)) { dupDropped++; return; }
        snprintf(ended[nEnded], sizeof(ended[nEnded]), "%s", t); nEnded++;
      }
      long total() const { long t = 0; for (int i = 0; i < n; i++) if (!voided[i]) t += w[i]; return t; }
    } c;
    c.record("r-1", 1834);
    c.record("r-2", 1790);
    c.record("r-1", 1834);           // resend after a board reboot
    c.voidRec("r-2");
    c.voidRec("r-2");                // the app's resend of an unconfirmed void
    c.end("T-day-1007");
    c.end("T-day-1007");             // the app's resend of an unconfirmed end
    check_int("total counts r-1 once and r-2 not at all", c.total(), 1834);
    check_int("three duplicates dropped", c.dupDropped, 3);
    check_int("tally ended once", c.nEnded, 1);
  }

  printf("\n%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
  return failures;
}
