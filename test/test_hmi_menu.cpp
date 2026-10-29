// Host check of the screens and flows in include/hmi_menu.h (FW-3, FW-5's local verdict, FW-6,
// FW-8), design §4. Drives the menu as the firmware does: button events, scans, panel results
// and the board handler's verdicts in, requests and rows 1-2 out.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_hmi_menu.cpp -o "$TMPDIR/t_menu"
//   "$TMPDIR/t_menu"

#include "hmi_menu.h"
#include <cstdio>
#include <cstring>
#include <string>

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  %s  %s\n", ok ? "ok   " : "FAIL ", what);
  if (!ok) failures++;
}
static void check_str(const char *what, const std::string &got, const std::string &want) {
  const bool ok = got == want;
  if (ok) printf("  ok     %-44s |%s|\n", what, got.c_str());
  else { printf("  FAIL   %-44s\n         got  |%s|\n         want |%s|\n", what, got.c_str(), want.c_str()); failures++; }
}

static CmdIdStore flash;
static bool flashSet = false;
static bool loadFn(CmdIdStore &o) { if (!flashSet) return false; o = flash; return true; }
static void saveFn(const CmdIdStore &i) { flash = i; flashSet = true; }

struct Rig {
  HmiMenu m;
  CmdIdMinter id;
  HmiContext c;
  PanelKind lastKind = PK_NONE;
  std::string lastBody;
  uint16_t lastSeq = 0;
  int requests = 0;

  explicit Rig(bool full) {
    flashSet = false;   // each scenario is a fresh board: session 1
    id.load = loadFn; id.save = saveFn;
    m.minter = &id;
    m.full = full;
    c.now = 1000;
    c.online = true;
    c.linkText = "link wifi only";
    c.entries = 12;
    c.lastEntryW = 1834;
    c.nfc = NFC_OK; c.nfcVer = 1; c.nfcRev = 6;
    c.build = "699e3c6c1234";
    c.haveEpoch = true; c.epochMs = 1793268846000LL;
  }
  void adv(uint32_t ms) { c.now += ms; c.epochMs += ms; m.tick(c); }
  void press(uint8_t b) { m.onButton(BtnEvent{b, false}, c); m.tick(c); }
  void repeat(uint8_t b) { m.onButton(BtnEvent{b, true}, c); m.tick(c); }
  void scan(const char *hex) { m.onTag(hex, c); m.tick(c); }
  bool req() {
    char body[PANEL_BODY_MAX];
    PanelKind k;
    uint16_t s;
    if (!m.takeRequest(k, body, sizeof(body), s)) return false;
    lastKind = k; lastBody = body; lastSeq = s; requests++;
    return true;
  }
  // `wrote` defaults to what the outcome implies: an HTTP answer, a failed write or a bad answer
  // all mean SEND ran; a refused connect means it did not. A deadline can fall either side, so
  // the tests that matter say which with answerW().
  void answer(PanelOutcome o, int status, const PanelState *st = nullptr, const PanelDeliveries *dl = nullptr,
              const PanelReply *rp = nullptr) {
    answerW(o, status, o == PO_HTTP || o == PO_FAIL_SEND || o == PO_FAIL_RESPONSE, st, dl, rp);
  }
  void answerW(PanelOutcome o, int status, bool wrote, const PanelState *st = nullptr,
               const PanelDeliveries *dl = nullptr, const PanelReply *rp = nullptr) {
    PanelState s0{}; PanelDeliveries d0{}; PanelReply r0{};
    m.onPanelResult(lastSeq, lastKind, o, status, wrote, st ? *st : s0, dl ? *dl : d0, rp ? *rp : r0, c);
    m.tick(c);
  }
  void verdict(const char *vid, bool ok, const char *detail, long w = 1834) { m.onVerdict(vid, ok, detail, w, c); m.tick(c); }
  std::string r1() { char a[21], b[21]; m.render(a, b, c); return a; }
  std::string r2() { char a[21], b[21]; m.render(a, b, c); return b; }
};

static PanelState todayState(bool emptied = true, bool mc = true) {
  PanelState s{};
  s.simple = true;
  s.count = 2;
  strcpy(s.t[0].id, "6f1c2b9e-0d3a-4c5b-9e8f-1a2b3c4d5e6f"); strcpy(s.t[0].label, "DAILY LOG");
  s.t[0].haveN = true; s.t[0].n = 12;
  strcpy(s.t[1].id, "aa11bb22-cc33-dd44-ee55-ff6677889900"); strcpy(s.t[1].label, "SMITH #4411");
  s.t[1].delivery = true; s.t[1].selected = true; s.t[1].haveN = true; s.t[1].n = 3;
  s.mcEnabled = mc;
  s.lastHaveAge = true; s.lastAgeTenths = 503; s.lastHaveEmptied = true; s.lastEmptied = emptied;
  s.lastHaveW = true; s.lastW = 1834;
  return s;
}

int main() {
  printf("(a) HOME, TAG, RECORD, RECORDED (design §4.3-§4.5)\n");
  {
    Rig r(false);
    r.press(BTN_OK);
    check("(a): OK on HOME does nothing", r.m.isHome() && !r.req());
    r.scan("647207CA");
    check("a scan opens TAG", r.m.screen() == HS_TAG);
    check_str("TAG row 1", r.r1(), "TAG 64:72:07:CA     ");
    check_str("TAG row 2, (a) fixed", r.r2(), "DAILY LOG     OK=REC");
    check("(a) never reads /api/state", !r.req());
    r.press(BTN_OK);
    check("OK sends", r.req() && r.lastKind == PK_CMD);
    check_str("the (a) body", r.lastBody, "{\"id\":\"p4-00000001-1\",\"verb\":\"+\",\"into\":\"daily_log\",\"tag\":\"647207CA\"}");
    check_str("SENDING row 1", r.r1(), "RECORDING...        ");
    check_str("SENDING row 2", r.r2(), "TAG 64:72:07:CA     ");
    r.press(BTN_BACK);
    check("BACK cannot interrupt a send", r.m.screen() == HS_BUSY);
    // The board's own command often arrives over MQTT before the 202 is read.
    r.verdict("p4-00000001-1", true, "recorded", 1834);
    check("verdict before the 202: still waiting", r.m.screen() == HS_BUSY);
    r.answer(PO_HTTP, 202);
    check("202 with the verdict in hand: RECORDED", r.m.screen() == HS_DONE);
    check_str("RECORDED row 1", r.r1(), "RECORDED     1834 LB");
    check_str("RECORDED row 2", r.r2(), "INTO DAILY LOG      ");
    check("the tag is used up", !r.m.tagArmed());
    r.adv(3000);
    check("3 s later: HOME", r.m.isHome());
  }

  printf("(a) NO ANSWER resends under the SAME id\n");
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    const std::string first = r.lastBody;
    r.answer(PO_HTTP, 202);
    r.verdict("p4-99999999-1", true, "recorded");   // someone else's id: ignored
    r.adv(9900);
    check("9.9 s after the 202: still waiting", r.m.screen() == HS_BUSY);
    r.adv(200);
    check("10 s: NO ANSWER", r.m.screen() == HS_NOANSWER);
    check_str("NO ANSWER row 1", r.r1(), "NO ANSWER  OK=RESEND");
    r.adv(5000);
    check("NO ANSWER stays up", r.m.screen() == HS_NOANSWER);
    r.press(BTN_OK);
    check("OK resends", r.req() && r.lastKind == PK_CMD);
    check("the same body, the same id", r.lastBody == first);
    r.answer(PO_HTTP, 202);
    r.verdict(r.m.jobId(), true, "recorded");
    check("then RECORDED", r.m.screen() == HS_DONE);
  }

  printf("(a) a late verdict settles NO ANSWER\n");
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 202);
    const std::string id = r.m.jobId();
    r.adv(10100);
    check("NO ANSWER", r.m.screen() == HS_NOANSWER);
    r.verdict(id.c_str(), true, "recorded");
    check("the late verdict shows RECORDED", r.m.screen() == HS_DONE);
  }

  printf("(a) NOT RECORDED retries under a NEW id\n");
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    r.answer(PO_HTTP, 202);
    r.verdict(r.m.jobId(), false, "UNDER ZERO");
    check("refused by the board", r.m.screen() == HS_FAILED);
    check_str("NOT RECORDED row 1", r.r1(), "NOT RECORDED  OK=TRY");
    check_str("the board's own reason", r.r2(), "UNDER ZERO          ");
    check("the tag stays armed", r.m.tagArmed());
    r.press(BTN_OK);
    check("OK sends again", r.req());
    check("under a new id", r.lastBody.find("\"p4-00000001-2\"") != std::string::npos);
    r.answer(PO_HTTP, 409);
    check_str("a panel 409 without a state name", r.r2(), "PANEL 409           ");
    r.press(BTN_BACK);
    check("BACK returns to TAG, tag armed", r.m.screen() == HS_TAG && r.m.tagArmed());
    r.press(BTN_OK); r.req();
    PanelReply rp{}; strcpy(rp.stateName, "NO ADC"); strcpy(rp.error, "the board will not act in this state");
    r.answer(PO_HTTP, 409, nullptr, nullptr, &rp);
    check_str("a gate 409: the machine's state name", r.r2(), "NO ADC              ");
    check("the full error kept for the console", strcmp(r.m.lastError_, "the board will not act in this state") == 0);
    r.press(BTN_BACK); r.press(BTN_OK); r.req();
    r.answer(PO_HTTP, 400);
    check_str("a 400 (OPEN_DAILY_FIRST, the RFID build's into refusal)", r.r2(), "PANEL 400           ");
  }

  printf("(a) NO PANEL retries the same id\n");
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    const std::string first = r.lastBody;
    r.answer(PO_FAIL_CONNECT, 0);
    check("NO PANEL", r.m.screen() == HS_NOPANEL);
    check_str("NO PANEL row 1", r.r1(), "NO PANEL   OK=RETRY ");
    check_str("NO PANEL row 2: the link text", r.r2(), "link wifi only      ");
    r.press(BTN_OK);
    check("retry is the same request", r.req() && r.lastBody == first);
    r.answer(PO_FAIL_DEADLINE, 0);
    r.adv(30001);
    check("30 s with no press: HOME, tag dropped", r.m.isHome() && !r.m.tagArmed());
  }

  printf("BLOCKING-1: no double record after a deadline that follows the send\n");
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    const std::string first = r.lastBody;
    r.answerW(PO_FAIL_DEADLINE, 0, true);   // SEND went out, the READ missed the 8 s deadline
    r.verdict("p4-00000001-1", true, "recorded", 1834);   // ...and the board ran the "+"
    check("the late verdict settles it: RECORDED", r.m.screen() == HS_DONE);
    check("the tag is used up", !r.m.tagArmed());
  }
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    const std::string first = r.lastBody;
    r.answerW(PO_FAIL_DEADLINE, 0, true);
    check("a deadline after the send is NO ANSWER, not NO PANEL", r.m.screen() == HS_NOANSWER);
    r.press(BTN_BACK);                      // back to TAG, tag still armed
    if (r.m.screen() != HS_TAG) r.scan("647207CA");
    check_str("TAG says the record is unsettled", r.r2(), "NO ANSWER  OK=RESEND");
    r.press(BTN_OK);
    const bool sent = r.req();
    check("OK again resends", sent);
    check("under the SAME id, never a fresh one", sent && r.lastBody == first);
    printf("        second send: %s\n", r.lastBody.c_str());
    r.answerW(PO_FAIL_DEADLINE, 0, true);
    r.press(BTN_BACK);
    r.adv(121000);                          // the tag expires: not an explicit discard
    check("tag expiry goes HOME", r.m.isHome());
    r.scan("647207CA");
    r.press(BTN_OK); r.req();
    check("the same tag rescanned after expiry: still the same id", r.lastBody == first);
    r.answer(PO_HTTP, 202);
    r.verdict("p4-00000001-1", true, "recorded");
    check("its verdict settles it", r.m.screen() == HS_DONE && !r.m.tagArmed());
    r.adv(3000);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    check("after the verdict a new RECORD of that tag takes a new id",
          r.lastBody.find("\"p4-00000001-2\"") != std::string::npos);
  }
  {
    // The verdict arrives while the operator is back on TAG.
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    r.answerW(PO_FAIL_DEADLINE, 0, true);
    r.press(BTN_BACK);
    r.verdict("p4-00000001-1", true, "recorded", 1777);
    check("a verdict on TAG shows RECORDED", r.m.screen() == HS_DONE);
    check_str("with its weight", r.r1(), "RECORDED     1777 LB");
    check("tag used up", !r.m.tagArmed());
  }
  {
    // Explicit discard: BACK on the TAG that says NO ANSWER.
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    r.answerW(PO_FAIL_DEADLINE, 0, true);
    r.press(BTN_BACK);                      // NO ANSWER -> TAG
    r.press(BTN_BACK);                      // TAG (pending) -> HOME: the operator discards it
    r.scan("647207CA");
    check_str("discarded: TAG offers a fresh RECORD", r.r2(), "DAILY LOG     OK=REC");
    r.press(BTN_OK); r.req();
    check("a fresh id after the explicit discard", r.lastBody.find("\"p4-00000001-2\"") != std::string::npos);
  }
  {
    // A deadline BEFORE the send (offline: the gate never opened) is NO PANEL, nothing pending.
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    r.answerW(PO_FAIL_DEADLINE, 0, false);
    check("not sent: NO PANEL", r.m.screen() == HS_NOPANEL);
    r.press(BTN_BACK);
    check_str("nothing pending on TAG", r.r2(), "DAILY LOG     OK=REC");
  }
  {
    // (b): the same rule through the state read and a delivery select.
    Rig r(true);
    r.scan("647207CA"); r.req();
    PanelState s = todayState(true, false);
    r.answer(PO_HTTP, 200, &s);
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &s);
    r.req(); r.answer(PO_HTTP, 200);        // select
    r.req();
    const std::string first = r.lastBody;
    r.answerW(PO_FAIL_DEADLINE, 0, true);
    r.press(BTN_BACK);
    r.req(); r.answer(PO_HTTP, 200, &s);    // nothing, TAG did not refetch; harmless if it did
    r.press(BTN_OK);
    bool got = false;
    for (int i = 0; i < 3 && !got; i++) { if (r.req() && r.lastKind == PK_CMD) got = true; else if (r.lastKind == PK_STATE) r.answer(PO_HTTP, 200, &s); }
    check("(b) resend is the same cmd, same id", got && r.lastBody == first);
  }

  printf("(a) tag expiry, BACK, and a scan held during a send\n");
  {
    Rig r(false);
    r.scan("647207CA");
    r.adv(119000);
    check("119 s: TAG still up", r.m.screen() == HS_TAG);
    r.adv(1000);
    check("120 s: HOME", r.m.isHome() && !r.m.tagArmed());
    r.scan("647207CA");
    r.press(BTN_BACK);
    check("BACK drops the tag", r.m.isHome() && !r.m.tagArmed());
    r.scan("647207CA"); r.press(BTN_OK); r.req();
    r.scan("04A1B2C3D4E5F6");
    check("a scan during the send is held", r.m.screen() == HS_BUSY && strcmp(r.m.tag(), "647207CA") == 0);
    r.answer(PO_HTTP, 202); r.verdict(r.m.jobId(), true, "recorded");
    check("RECORDED shows first", r.m.screen() == HS_DONE);
    r.adv(3000);
    check("then TAG opens with the held tag", r.m.screen() == HS_TAG && strcmp(r.m.tag(), "04A1B2C3D4E5F6") == 0);
    check_str("7-byte row 1", r.r1(), "04:A1:B2:C3:D4:E5:F6");
    r.press(BTN_OK); r.req();
    r.scan("647207CA");
    r.answer(PO_FAIL_CONNECT, 0);
    check("a held tag after a failure opens at once", r.m.screen() == HS_TAG && strcmp(r.m.tag(), "647207CA") == 0);
    r.scan("not-a-tag");
    check("an invalid tag is ignored", strcmp(r.m.tag(), "647207CA") == 0);
  }

  printf("(b) TAG reads the panel; destinations (design §4.6)\n");
  {
    Rig r(true);
    r.scan("647207CA");
    check_str("reading", r.r2(), "READING PANEL...    ");
    check("TAG asks for /api/state", r.req() && r.lastKind == PK_STATE);
    r.press(BTN_OK);
    check("OK while reading does nothing", !r.req() && r.m.screen() == HS_TAG);
    PanelState s = todayState(true, false);
    r.answer(PO_HTTP, 200, &s);
    check_str("defaults to the panel's selected tally", r.r2(), "SMITH #4411   OK=REC");
    r.press(BTN_UP);
    check_str("UP steps to the daily log", r.r2(), "DAILY LOG     OK=REC");
    r.repeat(BTN_DOWN);
    check("mc disabled on the panel: DOWN held does not open MC", r.m.screen() == HS_TAG);
    r.press(BTN_OK);
    check("(b) RECORD reads /api/state first", r.req() && r.lastKind == PK_STATE);
    check_str("SENDING", r.r1(), "RECORDING...        ");
    r.answer(PO_HTTP, 200, &s);
    check("then the cmd", r.req() && r.lastKind == PK_CMD);
    check("daily log destination: into, tag", r.lastBody.find("\"into\":\"daily_log\",\"tag\":\"647207CA\"") != std::string::npos);
    check("no mc", r.lastBody.find("\"mc\"") == std::string::npos);
    r.answer(PO_HTTP, 202); r.verdict(r.m.jobId(), true, "recorded");
    check("RECORDED", r.m.screen() == HS_DONE);
  }

  printf("(b) a delivery RECORD: select, then the cmd without into\n");
  {
    Rig r(true);
    r.scan("647207CA"); r.req();
    PanelState s = todayState(true, false);
    r.answer(PO_HTTP, 200, &s);
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &s);
    check("POST /api/tally/select", r.req() && r.lastKind == PK_SELECT);
    check_str("select body", r.lastBody, "{\"tally\":\"aa11bb22-cc33-dd44-ee55-ff6677889900\"}");
    r.answer(PO_HTTP, 200);
    check("then /api/cmd", r.req() && r.lastKind == PK_CMD);
    check("no into on a delivery RECORD", r.lastBody.find("into") == std::string::npos);
    check("the tag rides", r.lastBody.find("\"tag\":\"647207CA\"") != std::string::npos);
    r.answer(PO_HTTP, 202); r.verdict(r.m.jobId(), true, "recorded", 950);
    check_str("RECORDED into the delivery", r.r2(), "INTO SMITH #4411    ");
    check_str("the recorded weight", r.r1(), "RECORDED      950 LB");
  }

  printf("(b) no tally open\n");
  {
    Rig r(true);
    r.scan("647207CA"); r.req();
    PanelState s{}; s.simple = true; s.canStartDaily = true;
    r.answer(PO_HTTP, 200, &s);
    check_str("NO TALLY", r.r2(), "NO TALLY      OK=REC");
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &s); r.req();
    check("the board's tally only: no into, no select", r.lastKind == PK_CMD && r.lastBody.find("into") == std::string::npos);
    r.answer(PO_HTTP, 202); r.verdict(r.m.jobId(), true, "recorded");
    check_str("BOARD TALLY ONLY", r.r2(), "BOARD TALLY ONLY    ");
  }

  printf("(b) DUP WARN (design §4.6)\n");
  {
    Rig r(true);
    r.scan("647207CA"); r.req();
    PanelState s = todayState(false, false);
    r.answer(PO_HTTP, 200, &s);
    r.press(BTN_UP);
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &s);
    check("a record 50 s old, not emptied: DUP WARN", r.m.screen() == HS_DUPWARN && !r.req());
    check_str("DUP WARN row 1", r.r1(), "LAST 1834 LB 50S AGO");
    check_str("DUP WARN row 2", r.r2(), "OK=AGAIN  BACK=STOP ");
    r.press(BTN_BACK);
    check("BACK stops: TAG, nothing sent", r.m.screen() == HS_TAG && !r.req());
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &s);
    r.press(BTN_OK);
    check("OK=AGAIN sends", r.req() && r.lastKind == PK_CMD);
    PanelState old = todayState(false, false); old.lastAgeTenths = 9000;
    Rig q(true);
    q.scan("647207CA"); q.req(); q.answer(PO_HTTP, 200, &old);
    q.press(BTN_OK); q.req(); q.answer(PO_HTTP, 200, &old);
    check("15 min old: no warning", q.req() && q.lastKind != PK_STATE && q.m.screen() == HS_BUSY);
  }

  printf("(b) MC entry (FW-8, design §4.14)\n");
  {
    Rig r(true);
    r.scan("647207CA"); r.req();
    PanelState s = todayState(true, true);
    r.answer(PO_HTTP, 200, &s);
    check_str("the MC indicator beside the destination", r.r2(), "SMITH #4411  MC --  ");
    r.press(BTN_UP);
    r.press(BTN_DOWN);
    check("DOWN pressed steps the destination", r.r2().rfind("SMITH", 0) == 0);
    r.repeat(BTN_DOWN);
    check("DOWN held opens MC", r.m.screen() == HS_MC);
    check_str("MC row 1 starts at 20.0", r.r1(), "MC 20.0 %     OK=SET");
    check_str("MC row 2 is the tag", r.r2(), "TAG 64:72:07:CA     ");
    r.repeat(BTN_DOWN); r.repeat(BTN_DOWN);
    check_str("the opening hold's repeats do not step", r.r1(), "MC 20.0 %     OK=SET");
    r.press(BTN_DOWN);
    check_str("DOWN steps 0.5", r.r1(), "MC 19.5 %     OK=SET");
    r.press(BTN_UP); r.press(BTN_UP); r.press(BTN_UP); r.press(BTN_UP);
    check_str("UP steps 0.5", r.r1(), "MC 21.5 %     OK=SET");
    r.press(BTN_BACK);
    check("BACK cancels: no mc", r.m.screen() == HS_TAG && r.m.mc() == MC_NONE);
    check("and the press that opened MC did not move the destination", r.r2().rfind("DAILY LOG", 0) == 0);
    r.press(BTN_DOWN); r.repeat(BTN_DOWN);
    r.press(BTN_UP); r.press(BTN_UP); r.press(BTN_UP);
    r.press(BTN_OK);
    check("OK sets 21.5", r.m.screen() == HS_TAG && r.m.mc() == 215);
    check_str("indicator shows it", r.r2(), "DAILY LOG    MC 21.5");
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &s); r.req();
    check_str("mc in the body as a one-decimal string", r.lastBody,
              "{\"id\":\"p4-00000001-1\",\"verb\":\"+\",\"into\":\"daily_log\",\"tag\":\"647207CA\",\"mc\":\"21.5\"}");
    const std::string first = r.lastBody;
    r.answer(PO_HTTP, 202);
    r.adv(10100);
    r.press(BTN_OK);
    check("a resend carries the first send's mc and id", r.req() && r.lastBody == first);
    r.answer(PO_HTTP, 202); r.verdict(r.m.jobId(), true, "recorded");
    check("credited: the tag and its mc are cleared", !r.m.tagArmed() && r.m.mc() == MC_NONE);

    r.adv(3000);
    r.scan("647207CA"); r.req(); r.answer(PO_HTTP, 200, &s);
    r.press(BTN_DOWN); r.repeat(BTN_DOWN);
    for (int i = 0; i < 40; i++) r.press(BTN_DOWN);
    check_str("DOWN below 5.0 is none", r.r1(), "MC --         OK=SET");
    r.press(BTN_UP);
    check_str("UP from none is 5.0", r.r1(), "MC 5.0 %      OK=SET");
    r.press(BTN_OK);
    check("5.0 set", r.m.mc() == 50);
    r.scan("04A1B2C3D4E5F6");
    check("a different tag clears the mc", r.m.mc() == MC_NONE);
  }

  printf("(b) RECORD NO TAG never carries a tag or mc\n");
  {
    Rig r(true);
    r.press(BTN_OK);
    check("OK on HOME opens MENU", r.m.screen() == HS_MENU);
    check_str("MENU row 1", r.r1(), ">RECORD NO TAG      ");
    check_str("MENU row 2", r.r2(), " TARE              v");
    r.press(BTN_OK);
    PanelState s = todayState(true, true);
    r.req(); r.answer(PO_HTTP, 200, &s); r.req();
    check_str("RECORD NO TAG body", r.lastBody, "{\"id\":\"p4-00000001-1\",\"verb\":\"+\",\"into\":\"daily_log\"}");
  }

  printf("(b) MENU, CONFIRM, TARE with no panel runs locally (design §4.7, §4.8, §5.5)\n");
  {
    Rig r(true);
    r.press(BTN_OK);
    r.press(BTN_DOWN);
    check_str("cursor on TARE", r.r2(), ">TARE              v");
    r.press(BTN_OK);
    check_str("TARE confirm", r.r1(), "TARE: EMPTY AT MARK?");
    check_str("TARE confirm row 2", r.r2(), "OK=TARE   BACK=NO   ");
    r.press(BTN_BACK);
    check("BACK=NO returns to MENU, nothing sent", r.m.screen() == HS_MENU && !r.req());
    r.press(BTN_OK); r.press(BTN_OK);
    check("TARE reads /api/state first", r.req() && r.lastKind == PK_STATE);
    PanelState st = todayState();
    r.answer(PO_HTTP, 200, &st);
    check("then tare goes through /api/cmd", r.req() && r.lastBody == "{\"id\":\"p4-00000001-1\",\"verb\":\"tare\"}");
    r.answer(PO_FAIL_CONNECT, 0);
    const char *v = nullptr;
    check("never sent (connect refused): the board tares locally", r.m.takeLocal(v) && strcmp(v, "tare") == 0);
    check_str("TARING LOCALLY", r.r1(), "TARING LOCALLY...   ");
    r.m.onLocalVerdict(true, "", r.c); r.m.tick(r.c);
    check_str("TARED", r.r1(), "TARED               ");
    check_str("LOCAL - NO PANEL", r.r2(), "LOCAL - NO PANEL    ");

    r.adv(3000);
    r.press(BTN_OK); r.press(BTN_DOWN); r.press(BTN_DOWN);
    r.press(BTN_OK);
    check_str("UNDO confirm names the weight", r.r1(), "UNDO LAST? 1834 LB  ");
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &st); r.req();
    check("UNDO is -", r.lastBody.find("\"verb\":\"-\"") != std::string::npos);
    r.answer(PO_FAIL_CONNECT, 0);
    const char *v2 = nullptr;
    check("UNDO never runs locally: NO PANEL", r.m.screen() == HS_NOPANEL && !r.m.takeLocal(v2));
    r.press(BTN_BACK);
    r.press(BTN_OK); r.press(BTN_DOWN); r.press(BTN_DOWN); r.press(BTN_DOWN); r.press(BTN_DOWN);
    check_str("scrolled: ^ above", r.r1(), " TALLIES           ^");
    r.press(BTN_OK);
    check_str("CLEAR confirm counts the entries", r.r1(), "CLEAR 12 ENTRIES?   ");
    r.press(BTN_OK); r.req(); r.answer(PO_HTTP, 200, &st); r.req();
    r.answer(PO_HTTP, 202);
    r.verdict(r.m.jobId(), true, "cleared");
    check_str("CLEARED", r.r1(), "CLEARED             ");
    r.adv(3000);
    r.press(BTN_OK);
    for (int i = 0; i < 5; i++) r.press(BTN_DOWN);
    r.press(BTN_OK);
    check_str("INFO row 1", r.r1(), "NFC PN532 FW 1.6    ");
    check_str("INFO row 2", r.r2(), "PANEL OK  b 699e3c6c");
    r.adv(30001);
    check("30 s: HOME", r.m.isHome());
  }

  printf("item 2: TARE after the send never tares locally\n");
  {
    Rig r(true);
    r.press(BTN_OK); r.press(BTN_DOWN); r.press(BTN_OK); r.press(BTN_OK);
    PanelState st = todayState();
    r.req(); r.answer(PO_HTTP, 200, &st); r.req();
    const std::string first = r.lastBody;
    r.answerW(PO_FAIL_DEADLINE, 0, true);
    const char *v = nullptr;
    check("tare sent, answer lost: no local tare", !r.m.takeLocal(v));
    check("NO ANSWER instead", r.m.screen() == HS_NOANSWER);
    r.press(BTN_OK);
    check("OK resends the same tare id", r.req() && r.lastBody == first);
    r.answerW(PO_FAIL_SEND, 0, true);
    check("a failed write may have gone out: still no local tare", !r.m.takeLocal(v) && r.m.screen() == HS_NOANSWER);
    Rig q(true);
    q.press(BTN_OK); q.press(BTN_DOWN); q.press(BTN_OK); q.press(BTN_OK);
    q.req(); q.answer(PO_FAIL_CONNECT, 0);
    check("the state read fails, nothing sent: local tare", q.m.takeLocal(v));
  }

  printf("item 3: TARE, UNDO, CLEAR refuse on the RFID build, read fresh\n");
  {
    Rig r(true);
    // Never read before: the verb reads it now.
    r.press(BTN_OK); r.press(BTN_DOWN); r.press(BTN_DOWN); r.press(BTN_OK); r.press(BTN_OK);
    check("UNDO reads /api/state before anything", r.req() && r.lastKind == PK_STATE);
    PanelState rf{}; rf.rfid = true;
    r.answer(PO_HTTP, 200, &rf);
    check("RFID build: no '-' sent, RFID BUILD shown", !r.req() && r.m.screen() == HS_RFIDOFF);
    Rig q(true);
    q.press(BTN_OK); for (int i = 0; i < 4; i++) q.press(BTN_DOWN); q.press(BTN_OK); q.press(BTN_OK);
    q.req();
    q.answer(PO_HTTP, 200, &rf);
    check("CLEAR on the RFID build: nothing sent", !q.req() && q.m.screen() == HS_RFIDOFF);
    Rig t(true);
    t.press(BTN_OK); t.press(BTN_DOWN); t.press(BTN_OK); t.press(BTN_OK);
    t.req(); t.answer(PO_HTTP, 200, &rf);
    check("TARE on the RFID build: nothing sent, no local tare", !t.req() && t.m.screen() == HS_RFIDOFF);
    PanelState ok = todayState();
    Rig u(true);
    u.press(BTN_OK); u.press(BTN_DOWN); u.press(BTN_DOWN); u.press(BTN_OK); u.press(BTN_OK);
    u.req(); u.answer(PO_HTTP, 500, &ok);
    check("a state read refused: nothing sent", !u.req() && u.m.screen() == HS_FAILED);
  }

  printf("(b) TALLIES, TALLY, DELIVERIES (design §4.9-§4.11)\n");
  {
    Rig r(true);
    r.press(BTN_OK);
    for (int i = 0; i < 3; i++) r.press(BTN_DOWN);
    r.press(BTN_OK);
    check("TALLIES reads the panel", r.m.screen() == HS_LOADING && r.req() && r.lastKind == PK_STATE);
    check_str("loading", r.r1(), "READING PANEL...    ");
    PanelState s = todayState(true, false);
    s.canStartDelivery = true;
    r.answer(PO_HTTP, 200, &s);
    check_str("TALLIES row 1", r.r1(), ">DAILY LOG   n12    ");
    check_str("TALLIES row 2", r.r2(), " SMITH #4411 n3   *v");
    r.press(BTN_DOWN);
    r.press(BTN_OK);
    check_str("TALLY row 1", r.r1(), "SMITH #4411 n3   *  ");
    check_str("TALLY row 2", r.r2(), ">SELECT             ");
    r.press(BTN_OK);
    check("SELECT reads /api/state first", r.req() && r.lastKind == PK_STATE);
    r.answer(PO_HTTP, 200, &s);
    check("then posts select", r.req() && r.lastKind == PK_SELECT);
    r.answer(PO_HTTP, 200);
    check("then back to TALLIES, read fresh", r.req() && r.lastKind == PK_STATE);
    r.answer(PO_HTTP, 200, &s);
    check("TALLIES", r.m.screen() == HS_TALLIES);
    r.press(BTN_DOWN); r.press(BTN_DOWN);
    check_str("+ DELIVERY", r.r2(), ">+ DELIVERY         ");
    r.press(BTN_OK);
    check("DELIVERIES reads /api/deliveries", r.req() && r.lastKind == PK_DELIVERIES);
    PanelDeliveries d{};
    d.enabled = true; d.count = 2; d.total = 2;
    d.d[0].pickingId = 4411; strcpy(d.d[0].label, "SMITH");
    d.d[1].pickingId = 4415; strcpy(d.d[1].label, "JONES"); d.d[1].tomorrow = true;
    r.answer(PO_HTTP, 200, nullptr, &d);
    check_str("DELIVERIES row 1", r.r1(), ">SMITH              ");
    check_str("DELIVERIES row 2, T for tomorrow", r.r2(), " JONES            T ");
    r.press(BTN_DOWN);
    r.press(BTN_OK);
    check_str("START confirm", r.r1(), "START JONES?        ");
    r.press(BTN_OK);
    check("START reads /api/state first", r.req() && r.lastKind == PK_STATE);
    r.answer(PO_HTTP, 200, &s);
    check("POST /api/tally/start", r.req() && r.lastKind == PK_START);
    check("with t_ms and the picking id", r.lastBody.find("\"attr\":\"delivery\",\"t_ms\":1793268846") != std::string::npos &&
                                         r.lastBody.find("\"picking_id\":4415}") != std::string::npos);
    r.answer(PO_HTTP, 202);
    check_str("STARTED", r.r1(), "STARTED             ");
  }

  printf("(b) RFID BUILD: buttons off (FW-6, design §4.13)\n");
  {
    Rig r(true);
    r.scan("647207CA"); r.req();
    PanelState rf{}; rf.rfid = true;
    r.answer(PO_HTTP, 200, &rf);
    check("the RFID build: RFID BUILD screen", r.m.screen() == HS_RFIDOFF);
    check_str("row 1", r.r1(), "PANEL IS RFID BUILD ");
    check_str("row 2", r.r2(), "BUTTONS OFF         ");
    check("the tag is dropped, nothing sent", !r.m.tagArmed() && !r.req());
    r.press(BTN_BACK);
    r.press(BTN_OK);
    check("HOME OK: still locked", r.m.screen() == HS_RFIDOFF && !r.req());
    r.press(BTN_OK);
    check("OK re-checks", r.req() && r.lastKind == PK_STATE);
    r.answer(PO_HTTP, 200, &rf);
    check("still RFID", r.m.screen() == HS_RFIDOFF && r.m.rfidLocked());
    r.press(BTN_OK); r.req();
    PanelState ok = todayState();
    r.answer(PO_HTTP, 200, &ok);
    check("today's firmware again: unlocked, HOME", r.m.isHome() && !r.m.rfidLocked());
    r.press(BTN_OK);
    check("MENU opens", r.m.screen() == HS_MENU);

    // A RECORD whose own /api/state read finds the RFID build sends nothing.
    Rig q(true);
    q.scan("647207CA"); q.req();
    PanelState s = todayState(true, false);
    q.answer(PO_HTTP, 200, &s);
    q.press(BTN_OK); q.req();
    q.answer(PO_HTTP, 200, &rf);
    check("RECORD finds the RFID build: no cmd sent", q.m.screen() == HS_RFIDOFF && !q.req());
  }

  printf("an answer to an abandoned request is never applied\n");
  {
    Rig r(true);
    r.press(BTN_OK);
    for (int i = 0; i < 3; i++) r.press(BTN_DOWN);
    r.press(BTN_OK);
    r.req();
    const uint16_t stale = r.lastSeq;
    r.press(BTN_BACK);
    check("BACK from loading", r.m.screen() == HS_MENU);
    r.press(BTN_OK); r.req();   // TALLIES again: a new request
    check("a new request went out", r.lastSeq != stale);
    PanelState rf{}; rf.rfid = true;
    PanelDeliveries d0{}; PanelReply r0{};
    r.m.onPanelResult(stale, PK_STATE, PO_HTTP, 200, true, rf, d0, r0, r.c);
    check("the stale answer changes nothing", r.m.screen() == HS_LOADING && !r.m.rfidLocked());
    PanelState ok = todayState();
    r.m.onPanelResult(r.lastSeq, PK_STATE, PO_HTTP, 200, true, ok, d0, r0, r.c);
    check("the current answer is applied", r.m.screen() == HS_TALLIES);
  }

  printf("a request the client never takes fails as NO PANEL\n");
  {
    Rig r(false);
    r.scan("647207CA"); r.press(BTN_OK);
    check("waiting to be taken", r.m.wantsRequest());
    r.adv(8000);
    check("8 s: NO PANEL", r.m.screen() == HS_NOPANEL && !r.m.wantsRequest());
  }

  printf("%d failure(s)\n", failures);
  return failures;
}
