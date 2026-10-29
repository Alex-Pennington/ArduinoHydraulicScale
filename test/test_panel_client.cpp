// Host check of the panel client, include/panel_client.h (FW-4, FW-8): the request bodies and
// the mc rules, the streaming JSON scan of /api/state and /api/deliveries, the HTTP Date clock,
// and the §5.2 stage machine -- ONE transport call per loop pass at most, only through the gate
// (online and under 2000 ms into the pass), with the 8 s deadline.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_panel_client.cpp -o "$TMPDIR/t_panel"
//   "$TMPDIR/t_panel"
//
// The panel's rules are greybeard-plc4-pwa df172d2: api/app.py:221-380, api/commands.py:306-313,
// api/moisture.py:49-79, api/simple_tally.py:437-448.

#include "panel_client.h"
#include <cstdio>
#include <cstring>
#include <string>

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  %s  %s\n", ok ? "ok   " : "FAIL ", what);
  if (!ok) failures++;
}
static void check_str(const char *what, const char *got, const char *want) {
  const bool ok = strcmp(got, want) == 0;
  if (ok) printf("  ok     %-44s %s\n", what, got);
  else { printf("  FAIL   %-44s\n         got  %s\n         want %s\n", what, got, want); failures++; }
}

// --- a transport that counts every call --------------------------------------
struct FakeNet {
  int calls = 0, connects = 0, writes = 0, reads = 0, stops = 0;
  bool connectOk = true;
  std::string sent;
  std::string response;
  size_t pos = 0;
  size_t chunk = 1000;        // bytes the modem hands over per read
  int connect(const char *, uint16_t) { calls++; connects++; return connectOk ? 1 : 0; }
  size_t write(const uint8_t *b, size_t n) { calls++; writes++; sent.assign((const char *)b, n); return n; }
  int read(uint8_t *b, size_t n) {
    calls++; reads++;
    size_t k = response.size() - pos;
    if (k > n) k = n;
    if (k > chunk) k = chunk;
    memcpy(b, response.data() + pos, k);
    pos += k;
    return (int)k;
  }
  void stop() { calls++; stops++; }
};

static std::string httpResp(int status, const std::string &body, bool withLen = true) {
  std::string s = "HTTP/1.1 " + std::to_string(status) + " X\r\nServer: waitress\r\n"
                  "Date: Thu, 29 Oct 2026 10:14:06 GMT\r\nContent-Type: application/json\r\n";
  if (withLen) s += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  return s + "\r\n" + body;
}

static const char *STATE_JSON =
  "{\"reading\":{\"w\":1834,\"state_name\":\"OK\"},\"acks\":[{\"id\":\"x\",\"ok\":true}],"
  "\"rfid\":null,\"bucket\":{\"a\":[1,[2,[3,[4,[5,[6,[7]]]]]]]},"
  "\"simple_tally\":{\"open\":[{\"tally\":\"6f1c2b9e-0d3a-4c5b-9e8f-1a2b3c4d5e6f\",\"attr\":\"daily_log\","
  "\"label\":\"Daily log\",\"selected\":false,\"n\":12},"
  "{\"tally\":\"aa11bb22-cc33-dd44-ee55-ff6677889900\",\"attr\":\"delivery\","
  "\"label\":\"Smith #4411 \\u00e9 \\\"q\\\"\",\"selected\":true,\"n\":3}],"
  "\"selected\":\"aa11bb22-cc33-dd44-ee55-ff6677889900\",\"board_n\":12,\"board_warning\":null,"
  "\"can_start\":{\"daily_log\":false,\"delivery\":false},\"persist_error\":null,"
  "\"last_record\":{\"cmd_id\":\"p4-00000001-3\",\"n\":12,\"w\":1834,\"age_s\":50.3,\"emptied\":false},"
  "\"director_enabled\":true,\"mc_enabled\":true}}";

int main() {
  printf("URL\n");
  {
    char host[32]; uint16_t port = 0;
    check("default URL parses", panelParseUrl(PLC4_PANEL_URL, host, sizeof(host), port));
    check_str("host", host, "192.168.10.99");
    check("port 8081", port == 8081);
    check("no port is 80", panelParseUrl("http://farm1/", host, sizeof(host), port) && port == 80);
    check("https refused", !panelParseUrl("https://x:1", host, sizeof(host), port));
    check("a path refused", !panelParseUrl("http://x:1/api", host, sizeof(host), port));
    check("port 0 refused", !panelParseUrl("http://x:0", host, sizeof(host), port));
    check("port 70000 refused", !panelParseUrl("http://x:70000", host, sizeof(host), port));
  }

  printf("POST /api/cmd bodies (app.py:221-380)\n");
  {
    char b[PANEL_BODY_MAX];
    panelCmdBody(b, sizeof(b), "p4-0000002a-7", "+", true, "647207CA", MC_NONE);
    check_str("(a) RECORD: design §5.1's exact body", b,
              "{\"id\":\"p4-0000002a-7\",\"verb\":\"+\",\"into\":\"daily_log\",\"tag\":\"647207CA\"}");
    panelCmdBody(b, sizeof(b), "p4-1-1", "+", true, nullptr, MC_NONE);
    check_str("RECORD NO TAG", b, "{\"id\":\"p4-1-1\",\"verb\":\"+\",\"into\":\"daily_log\"}");
    panelCmdBody(b, sizeof(b), "p4-1-2", "+", false, "647207CA", MC_NONE);
    check_str("delivery RECORD: no into", b, "{\"id\":\"p4-1-2\",\"verb\":\"+\",\"tag\":\"647207CA\"}");
    panelCmdBody(b, sizeof(b), "p4-1-3", "tare", true, "647207CA", 185);
    check_str("tare: never into, tag or mc", b, "{\"id\":\"p4-1-3\",\"verb\":\"tare\"}");
    panelCmdBody(b, sizeof(b), "p4-1-4", "-", false, nullptr, 0);
    check_str("UNDO LAST", b, "{\"id\":\"p4-1-4\",\"verb\":\"-\"}");
    panelCmdBody(b, sizeof(b), "p4-1-5", "b", false, nullptr, 0);
    check_str("CLEAR BOARD", b, "{\"id\":\"p4-1-5\",\"verb\":\"b\"}");
    check("a scanned tag that fails the rule refuses the RECORD",
          panelCmdBody(b, sizeof(b), "p4-1-6", "+", true, "647207ca", 0) == 0);
  }

  printf("mc (FW-8: moisture.py:49-79, app.py:257-269)\n");
  {
    char t[8];
    mcFormat(185, t, sizeof(t)); check_str("18.5", t, "18.5");
    mcFormat(50, t, sizeof(t));  check_str("5.0 keeps its decimal", t, "5.0");
    mcFormat(600, t, sizeof(t)); check_str("60.0", t, "60.0");
    mcFormat(200, t, sizeof(t)); check_str("20.0 keeps its decimal", t, "20.0");
    mcFormat(49, t, sizeof(t));  check_str("4.9 is out of range: empty", t, "");
    mcFormat(601, t, sizeof(t)); check_str("60.1 is out of range: empty", t, "");
    mcFormat(MC_NONE, t, sizeof(t)); check_str("none: empty", t, "");
    check("4.9 invalid", !mcValid(49));
    check("5.0 valid", mcValid(50));
    check("60.0 valid", mcValid(600));
    check("60.1 invalid", !mcValid(601));
    // Every value in range formats to the panel's regex with exactly one decimal.
    bool allOne = true;
    for (uint16_t v = MC_MIN_TENTHS; v <= MC_MAX_TENTHS; v++) {
      mcFormat(v, t, sizeof(t));
      const char *dot = strchr(t, '.');
      const size_t whole = dot ? (size_t)(dot - t) : 0;
      if (!dot || whole < 1 || whole > 2 || strlen(dot) != 2) allOne = false;
    }
    check("every value 5.0-60.0: 1-2 digits, a point, exactly one digit", allOne);

    char b[PANEL_BODY_MAX];
    panelCmdBody(b, sizeof(b), "p4-1-1", "+", true, "647207CA", 185);
    check_str("mc rides beside the tag, a string", b,
              "{\"id\":\"p4-1-1\",\"verb\":\"+\",\"into\":\"daily_log\",\"tag\":\"647207CA\",\"mc\":\"18.5\"}");
    panelCmdBody(b, sizeof(b), "p4-1-2", "+", true, "647207CA", 50);
    check("5.0 sends as \"5.0\"", strstr(b, "\"mc\":\"5.0\"") != nullptr);
    panelCmdBody(b, sizeof(b), "p4-1-3", "+", true, "647207CA", 600);
    check("60.0 sends as \"60.0\"", strstr(b, "\"mc\":\"60.0\"") != nullptr);
    check("4.9 refuses the RECORD, never sent", panelCmdBody(b, sizeof(b), "p4-1-4", "+", true, "647207CA", 49) == 0);
    check("60.1 refuses the RECORD, never sent", panelCmdBody(b, sizeof(b), "p4-1-5", "+", true, "647207CA", 601) == 0);
    panelCmdBody(b, sizeof(b), "p4-1-6", "+", true, nullptr, 185);
    check("never without a tag", strstr(b, "\"mc\"") == nullptr && b[0]);
    panelCmdBody(b, sizeof(b), "p4-1-7", "+", true, "", 185);
    check("never with an empty tag", strstr(b, "\"mc\"") == nullptr && b[0]);
    const char *verbs[] = {"tare", "-", "b", "kv"};
    bool none = true;
    for (const char *v : verbs) { panelCmdBody(b, sizeof(b), "p4-1-8", v, false, "647207CA", 185); if (strstr(b, "\"mc\"")) none = false; }
    check("never on a verb other than +", none);

    check("UP from none starts at 5.0", mcStepUp(MC_NONE) == 50);
    check("UP steps 0.5", mcStepUp(185) == 190);
    check("UP stops at 60.0", mcStepUp(600) == 600 && mcStepUp(598) == 600);
    check("DOWN steps 0.5", mcStepDown(185) == 180);
    check("DOWN from 5.0 clears", mcStepDown(50) == MC_NONE);
    check("DOWN from none stays none", mcStepDown(MC_NONE) == MC_NONE);
    char ind[12];
    mcIndicator(MC_NONE, ind, sizeof(ind)); check_str("indicator, none", ind, "MC --");
    mcIndicator(185, ind, sizeof(ind)); check_str("indicator, set", ind, "MC 18.5");
  }

  printf("tally bodies\n");
  {
    char b[PANEL_BODY_MAX];
    panelTallyBody(b, sizeof(b), "aa11bb22-cc33-dd44-ee55-ff6677889900");
    check_str("select/end", b, "{\"tally\":\"aa11bb22-cc33-dd44-ee55-ff6677889900\"}");
    check("a quote in a tally id is refused", panelTallyBody(b, sizeof(b), "a\"b") == 0);
    panelStartBody(b, sizeof(b), 1793268846000LL, -1);
    check_str("start daily log", b, "{\"attr\":\"daily_log\",\"t_ms\":1793268846000}");
    panelStartBody(b, sizeof(b), 1793268846000LL, 4411);
    check_str("start delivery", b, "{\"attr\":\"delivery\",\"t_ms\":1793268846000,\"picking_id\":4411}");
    check("no clock: no start", panelStartBody(b, sizeof(b), 0, -1) == 0);
  }

  printf("HTTP Date (the board's clock for t_ms)\n");
  {
    check("Thu, 01 Jan 1970 00:00:01 GMT = 1", panelParseHttpDate("Thu, 01 Jan 1970 00:00:01 GMT") == 1);
    check("Thu, 29 Oct 2026 10:14:06 GMT", panelParseHttpDate(" Thu, 29 Oct 2026 10:14:06 GMT") == 1793268846UL);
    check("29 Feb 2028 (leap)", panelParseHttpDate("Tue, 29 Feb 2028 00:00:00 GMT") == 1835395200UL);
    check("garbage = 0", panelParseHttpDate("yesterday") == 0);
  }

  printf("streaming scan of /api/state\n");
  {
    PanelState s; memset(&s, 0, sizeof(s));
    JsonScan j; j.reset(panelStateOn, &s);
    for (const char *p = STATE_JSON; *p; p++) j.feed(*p);
    check("scan completes, no error", j.done() && !j.error());
    check("rfid null: not the RFID build", !s.rfid);
    check("simple_tally present", s.simple);
    check("two open tallies", s.count == 2);
    check_str("tally 0 id", s.t[0].id, "6f1c2b9e-0d3a-4c5b-9e8f-1a2b3c4d5e6f");
    check_str("tally 0 label, upper-cased", s.t[0].label, "DAILY LOG");
    check("tally 0 n 12, not selected", s.t[0].haveN && s.t[0].n == 12 && !s.t[0].selected && !s.t[0].delivery);
    check_str("escapes honoured, cut to 20", s.t[1].label, "SMITH #4411 ? \"Q\"");
    check("tally 1 is the selected delivery", s.t[1].delivery && s.t[1].selected && s.t[1].n == 3);
    check("can_start both false", !s.canStartDaily && !s.canStartDelivery);
    check("mc_enabled", s.mcEnabled);
    check("last_record age 50.3 s", s.lastHaveAge && s.lastAgeTenths == 503);
    check("last_record not emptied, 1834 lb", s.lastHaveEmptied && !s.lastEmptied && s.lastHaveW && s.lastW == 1834);

    PanelState dl; memset(&dl, 0, sizeof(dl));
    j.reset(panelStateOn, &dl);
    const char *lab = "{\"simple_tally\":{\"open\":[{\"attr\":\"delivery\",\"label\":\"DELIVERY WH/OUT/04411 Smith\","
                      "\"customer_name\":\"Smith\",\"picking_name\":\"WH/OUT/04411\"},"
                      "{\"customer_name\":\"Jones\",\"attr\":\"delivery\",\"label\":\"DELIVERY WH/OUT/04415 Jones\"},"
                      "{\"attr\":\"delivery\",\"label\":\"DELIVERY WH/OUT/04416\"}]}}";
    for (const char *p = lab; *p; p++) j.feed(*p);
    check_str("delivery: the customer, not the panel's long label", dl.t[0].label, "SMITH");
    check_str("customer_name before label: still the customer", dl.t[1].label, "JONES");
    check_str("no customer: the label without DELIVERY", dl.t[2].label, "WH/OUT/04416");

    PanelState r; memset(&r, 0, sizeof(r));
    j.reset(panelStateOn, &r);
    const char *rf = "{\"rfid\":{\"tallies\":[],\"enabled\":true},\"simple_tally\":null}";
    for (const char *p = rf; *p; p++) j.feed(*p);
    check("the RFID build is seen", r.rfid && !r.simple && j.done());
    PanelState bad; memset(&bad, 0, sizeof(bad));
    j.reset(panelStateOn, &bad);
    const char *nest = "[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[1]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]";
    for (const char *p = nest; *p; p++) j.feed(*p);
    check("nesting past 32 is an error, not a crash", j.error());
  }

  printf("streaming scan of /api/deliveries\n");
  {
    PanelDeliveries d; memset(&d, 0, sizeof(d));
    JsonScan j; j.reset(panelDeliveriesOn, &d);
    std::string js = "{\"enabled\":true,\"reachable\":true,\"rows\":[";
    for (int i = 0; i < 8; i++) {
      if (i) js += ",";
      js += "{\"picking_id\":" + std::to_string(4410 + i) + ",\"picking_name\":\"WH/OUT/0" + std::to_string(4410 + i) +
            "\"" + (i == 1 ? ",\"customer_name\":\"Jones\"" : "") + ",\"tomorrow\":" + (i == 1 ? "true" : "false") + "}";
    }
    js += "]}";
    for (char c : js) j.feed(c);
    check("enabled", d.enabled);
    check("six kept of eight listed", d.count == 6 && d.total == 8);
    check("picking ids", d.d[0].pickingId == 4410 && d.d[5].pickingId == 4415);
    check_str("customer name preferred", d.d[1].label, "JONES");
    check_str("else the picking name", d.d[0].label, "WH/OUT/04410");
    check("tomorrow flag", d.d[1].tomorrow && !d.d[0].tomorrow);
  }

  printf("stage machine: one call per pass, only through the gate (§5.2)\n");
  {
    check("gate: online, 0 ms used", panelStageGate(true, 0));
    check("gate: online, 1999 ms used", panelStageGate(true, 1999));
    check("gate: closed at 2000 ms used", !panelStageGate(true, 2000));
    check("gate: closed offline", !panelStageGate(false, 0));

    FakeNet net;
    PanelHttp<FakeNet> h(net);
    check("URL set", h.setUrl(PLC4_PANEL_URL));
    net.response = httpResp(202, "{\"ok\":true,\"id\":\"p4-1-1\",\"verb\":\"+\"}");
    net.chunk = 16;   // the answer arrives in pieces
    char body[PANEL_BODY_MAX];
    panelCmdBody(body, sizeof(body), "p4-1-1", "+", true, "647207CA", 185);
    check("request starts", h.start(PK_CMD, body, 0));
    check("a second request waits: one at a time", !h.start(PK_STATE, "", 0));
    int maxCalls = 0, passes = 0;
    PanelOutcome o = PO_PENDING; int status = 0;
    bool got = false;
    for (uint32_t t = 10; t < 3000 && h.busy(); t += 10) {
      const int before = net.calls;
      h.step(t, panelStageGate(true, 5));
      const int d = net.calls - before;
      if (d > maxCalls) maxCalls = d;
      passes++;
      if (!got) got = h.takeResult(o, status);
    }
    check("never more than one transport call per pass", maxCalls == 1);
    check("connect, write and stop once each", net.connects == 1 && net.writes == 1 && net.stops == 1);
    check("answer read over several passes", net.reads > 3);
    check("every pass made exactly one call", net.calls == passes);
    check("HTTP 202", got && o == PO_HTTP && status == 202);
    check("request is HTTP/1.0 POST to /api/cmd", net.sent.rfind("POST /api/cmd HTTP/1.0\r\n", 0) == 0);
    check("Host header names the port", net.sent.find("Host: 192.168.10.99:8081\r\n") != std::string::npos);
    check("body is the last thing written", net.sent.size() >= strlen(body) &&
          net.sent.compare(net.sent.size() - strlen(body), strlen(body), body) == 0);
    check("whole request inside one 256-byte write", net.sent.size() <= PANEL_REQ_MAX);
    int64_t ms = 0;
    check("the panel's clock is known", h.epochMs(1000, ms) && ms > 1793268846000LL - 10000);
  }
  {
    // The gate closed: no call at all, whatever state the request is in.
    FakeNet net;
    PanelHttp<FakeNet> h(net);
    h.setUrl(PLC4_PANEL_URL);
    net.response = httpResp(200, STATE_JSON);
    h.start(PK_STATE, "", 0);
    for (uint32_t t = 0; t < 1000; t += 10) h.step(t, panelStageGate(true, 2500));
    check("a pass already 2500 ms long: no call", net.calls == 0);
    for (uint32_t t = 1000; t < 2000; t += 10) h.step(t, panelStageGate(false, 0));
    check("offline: no call", net.calls == 0);
    PanelOutcome o; int st;
    check("no result yet", !h.takeResult(o, st));
    for (uint32_t t = 2000; t < 9000; t += 10) h.step(t, false);
    check("8 s deadline fails the request with no call", h.takeResult(o, st) && o == PO_FAIL_DEADLINE && net.calls == 0);
    check("the socket is still to be closed", h.busy());
    h.step(9010, true);
    check("the next open gate closes it: one call", net.calls == 1 && net.stops == 1 && !h.busy());
  }
  {
    // A stalled answer: the deadline stops the next stage.
    FakeNet net;
    PanelHttp<FakeNet> h(net);
    h.setUrl(PLC4_PANEL_URL);
    net.response = "HTTP/1.1 200 OK\r\nContent-Length: 500\r\n\r\n{\"a\":";   // never finishes
    h.start(PK_STATE, "", 0);
    PanelOutcome o = PO_PENDING; int st = 0;
    bool got = false;
    for (uint32_t t = 0; t < 10000 && !got; t += 100) { h.step(t, true); got = h.takeResult(o, st); }
    check("an answer that never completes: deadline", got && o == PO_FAIL_DEADLINE);
    h.step(10000, true);
    check("then closed", !h.busy() && net.stops == 1);
  }
  {
    FakeNet net;
    net.connectOk = false;
    PanelHttp<FakeNet> h(net);
    h.setUrl(PLC4_PANEL_URL);
    h.start(PK_STATE, "", 0);
    h.step(0, true);
    PanelOutcome o; int st;
    check("connect refused: failed at once", h.takeResult(o, st) && o == PO_FAIL_CONNECT);
    check("only one call that pass", net.calls == 1);
    h.step(10, true);
    check("then CLOSE in a later pass", net.stops == 1 && !h.busy());
  }
  {
    // No Content-Length: the scan's end of the top-level value ends the answer.
    FakeNet net;
    PanelHttp<FakeNet> h(net);
    h.setUrl(PLC4_PANEL_URL);
    net.response = httpResp(200, STATE_JSON, false);
    h.start(PK_STATE, "", 0);
    PanelOutcome o = PO_PENDING; int st = 0; bool got = false;
    for (uint32_t t = 0; t < 2000 && !got; t += 10) { h.step(t, true); got = h.takeResult(o, st); }
    check("ends on the JSON, not the socket", got && o == PO_HTTP && st == 200);
    check("state kept", h.state.count == 2 && h.state.mcEnabled);
  }
  {
    // A refusal: the panel's words are kept for the console and the screen.
    FakeNet net;
    PanelHttp<FakeNet> h(net);
    h.setUrl(PLC4_PANEL_URL);
    net.response = httpResp(409, "{\"ok\":false,\"error\":\"the board will not act in this state\","
                                 "\"verdict\":\"stale\",\"state_name\":\"under zero\"}");
    h.start(PK_CMD, "{}", 0);
    PanelOutcome o = PO_PENDING; int st = 0; bool got = false;
    for (uint32_t t = 0; t < 2000 && !got; t += 10) { h.step(t, true); got = h.takeResult(o, st); }
    check("409 read", got && o == PO_HTTP && st == 409);
    check_str("state_name for the row", h.reply.stateName, "UNDER ZERO");
    check_str("error for the console", h.reply.error, "the board will not act in this state");
  }

  printf("wrote(): whether SEND ran (review BLOCKING-1)\n");
  {
    FakeNet net; PanelHttp<FakeNet> h(net); h.setUrl(PLC4_PANEL_URL);
    net.connectOk = false;
    h.start(PK_CMD, "{}", 0); h.step(0, true);
    check("connect refused: not written", !h.wrote());
    h.step(10, true);
    FakeNet n2; PanelHttp<FakeNet> h2(n2); h2.setUrl(PLC4_PANEL_URL);
    n2.response = "HTTP/1.1 202 X\r\nContent-Length: 50\r\n\r\n{";   // never completes
    h2.start(PK_CMD, "{}", 0);
    PanelOutcome o = PO_PENDING; int st = 0; bool got = false;
    for (uint32_t t = 0; t < 10000 && !got; t += 100) { h2.step(t, true); got = h2.takeResult(o, st); }
    check("deadline after SEND: written", got && o == PO_FAIL_DEADLINE && h2.wrote());
    FakeNet n3; PanelHttp<FakeNet> h3(n3); h3.setUrl(PLC4_PANEL_URL);
    h3.start(PK_CMD, "{}", 0);
    for (uint32_t t = 0; t < 9000; t += 100) h3.step(t, false);
    check("deadline with the gate shut: not written", h3.takeResult(o, st) && o == PO_FAIL_DEADLINE && !h3.wrote());
    h3.step(9000, true);
    h3.start(PK_CMD, "{}", 9100);
    check("a new request starts unwritten", !h3.wrote());
  }

  printf("a simulated loop: the HTTP stage only in quiet passes\n");
  {
    FakeNet net;
    PanelHttp<FakeNet> h(net);
    h.setUrl(PLC4_PANEL_URL);
    net.response = httpResp(202, "{\"ok\":true}");
    h.start(PK_CMD, "{}", 0);
    // Pass lengths: some passes have already run long (a reconnect stage, a PUBACK spin).
    const uint32_t used[] = {5, 2400, 7, 3100, 1999, 2000, 4, 9, 2500, 3, 6, 8};
    int ranInLongPass = 0, ranTotal = 0;
    uint32_t t = 0;
    for (uint32_t u : used) {
      t += 50;
      const int before = net.calls;
      h.step(t, panelStageGate(true, u));
      const int d = net.calls - before;
      ranTotal += d;
      if (d && u >= PANEL_PASS_BUDGET_MS) ranInLongPass++;
    }
    check("no stage in a pass already >= 2000 ms in", ranInLongPass == 0);
    check("the stages ran in the quiet passes", ranTotal >= 4);
  }

  printf("%d failure(s)\n", failures);
  return failures;
}
