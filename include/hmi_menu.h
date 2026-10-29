#pragma once
// FW-3, FW-6, FW-8: the LCD screens and the flows behind them,
// docs/design/2026-10-29-onboard-rfid-buttons-menus.md §4.
//
// Rows 0 and 3 are not this file's. Row 0 is the weight row and row 3 the fault row, drawn by
// the firmware on every render whatever screen is up (design §4.1). This file owns rows 1 and 2.
// On HOME it owns nothing: the firmware draws today's home rows unchanged (design §4.3).
//
// Build order (design §4.2): part (a), scan -> record, is HOME, TAG and the result screens with
// OK and BACK only and one request per RECORD. Part (b) is the rest of the pendant set. Both
// are built; HmiMenu::full selects which one runs (PLC4_HMI_FULL in the firmware), so (a) can
// be flashed and proven first, as the design orders, without a second code base.
//
// Pure C++, no Arduino. The firmware feeds it button events, scans, panel results and the
// board handler's verdicts, and drains the requests it asks for. test/test_hmi_menu.cpp drives it.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hmi_buttons.h"
#include "pn532.h"
#include "cmd_id.h"
#include "mc_value.h"
#include "panel_client.h"

static const uint32_t HMI_MENU_TIMEOUT_MS   = 30000;   // a menu returns HOME after 30 s with no press
static const uint32_t HMI_RESULT_MS         = 3000;    // a success result returns after 3 s or on any press
static const uint32_t HMI_VERDICT_WAIT_MS   = 10000;   // NO ANSWER 10 s after the panel's 202 (§4.5)
static const uint32_t HMI_WANT_STALL_MS     = 8000;    // a request the client never took fails as NO PANEL
static const int32_t  HMI_DUP_WINDOW_TENTHS = 9000;    // 15 min, Alex's DUP WARN window (§4.6)

enum HmiScreen : uint8_t {
  HS_HOME = 0, HS_TAG, HS_MC, HS_BUSY, HS_DONE, HS_FAILED, HS_NOANSWER, HS_NOPANEL,
  HS_DUPWARN, HS_MENU, HS_CONFIRM, HS_TALLIES, HS_TALLY, HS_DELIVERIES, HS_INFO, HS_RFIDOFF,
  HS_LOADING
};

enum HmiOp : uint8_t {
  OP_NONE = 0, OP_FETCH_TAG, OP_FETCH_TALLIES, OP_FETCH_DELIVERIES, OP_RFID_CHECK,
  OP_RECORD, OP_TARE, OP_UNDO, OP_CLEAR, OP_SELECT, OP_END, OP_START_DAILY, OP_START_DELIVERY
};

enum HmiPhase : uint8_t { PH_STATE = 0, PH_SELECT, PH_CMD, PH_POST, PH_VERDICT, PH_LOCAL, PH_PAUSED };

enum HmiConfirm : uint8_t { CF_TARE = 0, CF_UNDO, CF_CLEAR, CF_START_DAILY, CF_START_DELIVERY, CF_END };

// Everything the screens read from the rest of the board, handed in on each call.
struct HmiContext {
  uint32_t now = 0;
  bool online = false;               // netState == NET_ONLINE
  const char *linkText = "";         // today's link row text, "link mqtt up" etc. (src hmiDrawHome())
  long lastEntryW = 0;               // rT[rTindex-1], for UNDO LAST's question
  int entries = 0;                   // rTindex
  Pn532Health nfc = NFC_STARTING;
  uint8_t nfcVer = 0, nfcRev = 0;
  const char *build = "";            // BUILD_PROVENANCE
  bool haveEpoch = false;            // the panel's clock, from its Date header
  int64_t epochMs = 0;
};

class HmiMenu {
 public:
  bool full = true;                  // part (b): the full pendant set. false: part (a) only
  bool mcFeature = true;             // FW-8, part (b) only
  CmdIdMinter *minter = nullptr;

  HmiScreen screen() const { return scr_; }
  bool isHome() const { return scr_ == HS_HOME; }
  bool tagArmed() const { return tag_[0] != '\0'; }
  const char *tag() const { return tag_; }
  uint16_t mc() const { return mc_; }
  HmiOp op() const { return job_.op; }
  const char *jobId() const { return job_.id; }
  bool rfidLocked() const { return rfid_; }
  // Whether the screen is waiting on the panel: a scan there is held, not opened (design §4.1).
  bool waiting() const { return scr_ == HS_BUSY; }

  // --- what the firmware drains ------------------------------------------------
  // A request for the panel client. The firmware starts it when the client is idle.
  // `seq` comes back with the result, so an answer to an abandoned request can never be
  // applied to a later one.
  bool takeRequest(PanelKind &kind, char *body, size_t cap, uint16_t &seq) {
    if (want_ == PK_NONE) return false;
    kind = want_;
    snprintf(body, cap, "%s", wantBody_);
    seq = wantSeq_;
    want_ = PK_NONE;
    return true;
  }
  bool wantsRequest() const { return want_ != PK_NONE; }
  // A verb to run through the board's own handler, locally (TARE with no panel, design §5.5).
  bool takeLocal(const char *&verb) {
    if (!localWant_) return false;
    localWant_ = false;
    verb = "tare";
    return true;
  }

  // --- inputs --------------------------------------------------------------------
  void onTag(const char *hex, const HmiContext &c) {
    if (!tagHexValid(hex)) return;
    if (waiting()) {   // held until the wait ends
      snprintf(held_, sizeof(held_), "%s", hex);
      heldAt_ = c.now;
      return;
    }
    openTag(hex, c);
  }

  void onButton(const BtnEvent &e, const HmiContext &c) {
    lastPress_ = c.now;
    switch (scr_) {
      case HS_HOME:
        if (full && !e.repeat && e.id == BTN_OK) {
          if (rfid_) go(HS_RFIDOFF, c); else { menuCur_ = 0; menuTop_ = 0; go(HS_MENU, c); }
        }
        return;
      case HS_TAG:      tagButton(e, c); return;
      case HS_MC:       mcButton(e, c); return;
      case HS_BUSY:     return;   // nothing interrupts a request that may record
      case HS_DONE:     if (!e.repeat) resultEnded(c); return;
      case HS_FAILED:   if (!e.repeat) failedButton(e, c); return;
      case HS_NOANSWER:
      case HS_NOPANEL:  if (!e.repeat) retryButton(e, c); return;
      case HS_DUPWARN:  if (!e.repeat) dupButton(e, c); return;
      case HS_MENU:     menuButton(e, c); return;
      case HS_CONFIRM:  if (!e.repeat) confirmButton(e, c); return;
      case HS_TALLIES:  talliesButton(e, c); return;
      case HS_TALLY:    tallyButton(e, c); return;
      case HS_DELIVERIES: deliveriesButton(e, c); return;
      case HS_INFO:     if (!e.repeat && (e.id == BTN_BACK || e.id == BTN_OK)) go(HS_MENU, c); return;
      case HS_RFIDOFF:
        if (e.repeat) return;
        if (e.id == BTN_BACK) goHome(c);
        else if (e.id == BTN_OK) beginFetch(OP_RFID_CHECK, HS_RFIDOFF, c);
        return;
      case HS_LOADING:
        if (!e.repeat && e.id == BTN_BACK) { abandonFetch(); backFromLoading(c); }
        return;
    }
  }

  // The board's handler ran a command under `id` (the MQTT ack block, or a dedup replay).
  // `w` is the weight it recorded, for RECORDED's row.
  void onVerdict(const char *id, bool ok, const char *detail, long w, const HmiContext &c) {
    if (job_.op == OP_NONE || !id || strcmp(id, job_.id) != 0 || !job_.id[0]) return;
    job_.verdictIn = true;
    job_.verdictOk = ok;
    job_.verdictW = w;
    snprintf(job_.detail, sizeof(job_.detail), "%s", detail ? detail : "");
    // A verdict that lands after NO ANSWER went up still settles it: the board did run it.
    if ((job_.phase == PH_VERDICT && scr_ == HS_BUSY) || scr_ == HS_NOANSWER) finishVerdict(c);
  }

  // The local TARE ran through the board's handler.
  void onLocalVerdict(bool ok, const char *detail, const HmiContext &c) {
    if (job_.op != OP_TARE || job_.phase != PH_LOCAL) return;
    job_.local = true;
    job_.verdictIn = true;
    job_.verdictOk = ok;
    snprintf(job_.detail, sizeof(job_.detail), "%s", detail ? detail : "");
    finishVerdict(c);
  }

  // The panel client finished the request this menu asked for.
  void onPanelResult(uint16_t seq, PanelKind kind, PanelOutcome o, int status, const PanelState &st,
                     const PanelDeliveries &dl, const PanelReply &rp, const HmiContext &c) {
    if (kind != PK_NONE) lastAnswer_ = (o == PO_HTTP) ? 1 : 2;
    if (seq == 0 || seq != expect_) return;   // an answer to a request this menu has let go of
    expect_ = 0;
    if (job_.op == OP_NONE) return;
    if (o == PO_HTTP && kind == PK_STATE && status == 200) {
      rfid_ = st.rfid;
      if (st.simple || st.rfid) { state_ = st; haveState_ = true; }
    }
    switch (job_.op) {
      case OP_FETCH_TAG:        fetchTagDone(o, status, c); return;
      case OP_FETCH_TALLIES:    fetchListDone(OP_FETCH_TALLIES, o, status, HS_TALLIES, c); return;
      case OP_FETCH_DELIVERIES:
        if (o == PO_HTTP && status == 200) dl_ = dl;
        fetchListDone(OP_FETCH_DELIVERIES, o, status, HS_DELIVERIES, c);
        return;
      case OP_RFID_CHECK:
        endJob();
        if (o != PO_HTTP || status != 200) { go(HS_RFIDOFF, c); return; }
        if (rfid_) go(HS_RFIDOFF, c); else goHome(c);
        return;
      default: actionResult(kind, o, status, rp, c); return;
    }
  }

  // Timeouts and stalls. Call every pass.
  void tick(const HmiContext &c) {
    if (want_ != PK_NONE && (uint32_t)(c.now - wantSince_) >= HMI_WANT_STALL_MS) {
      want_ = PK_NONE;   // the client never took it: treat as a failed request
      const PanelState s{}; const PanelDeliveries d{}; const PanelReply r{};
      onPanelResult(expect_, PK_NONE, PO_FAIL_DEADLINE, 0, s, d, r, c);
      return;
    }
    if (job_.op != OP_NONE && job_.phase == PH_VERDICT && scr_ == HS_BUSY && !job_.verdictIn &&
        (uint32_t)(c.now - job_.acceptedAt) >= HMI_VERDICT_WAIT_MS) {
      go(HS_NOANSWER, c);
      return;
    }
    // A tag scanned while the screen waited on the panel opens once the wait is over, unless a
    // success result is showing: that result keeps its 3 s and the tag opens after it.
    if (held_[0] && !waiting() && scr_ != HS_DONE) {
      char h[TAG_HEX_LEN];
      snprintf(h, sizeof(h), "%s", held_);
      held_[0] = '\0';
      if ((uint32_t)(c.now - heldAt_) < TAG_ARMED_MS) { openTag(h, c); return; }
    }
    switch (scr_) {
      case HS_TAG:
      case HS_MC:
        if ((uint32_t)(c.now - tagAt_) >= TAG_ARMED_MS) goHome(c);
        return;
      case HS_DONE:
        if ((uint32_t)(c.now - shownAt_) >= HMI_RESULT_MS) resultEnded(c);
        return;
      case HS_HOME:
      case HS_BUSY:
      case HS_LOADING:
        return;
      default:
        if ((uint32_t)(c.now - lastPress_) >= HMI_MENU_TIMEOUT_MS &&
            (uint32_t)(c.now - shownAt_) >= HMI_MENU_TIMEOUT_MS) goHome(c);
        return;
    }
  }

  // Rows 1 and 2, 20 characters each, padded. Not called on HOME.
  void render(char *r1, char *r2, const HmiContext &c) const {
    char a[48] = "", b[48] = "";
    switch (scr_) {
      case HS_HOME: break;
      case HS_TAG:  renderTag(a, b, sizeof(a)); break;
      case HS_MC: {
        char ind[12];
        mcIndicator(mcEdit_, ind, sizeof(ind));
        snprintf(a, sizeof(a), "%-14sOK=SET", mcEdit_ ? strcat(ind, " %") : ind);
        tagRowText(tag_, b, sizeof(b));
        break;
      }
      case HS_BUSY:
        snprintf(a, sizeof(a), "%s", busyWord());
        jobSubject(b, sizeof(b));
        break;
      case HS_LOADING: snprintf(a, sizeof(a), "READING PANEL..."); break;
      case HS_DONE:     renderDone(a, b, sizeof(a)); break;
      case HS_FAILED:
        snprintf(a, sizeof(a), "%-14sOK=TRY", failWord());
        snprintf(b, sizeof(b), "%s", failReason_);
        break;
      case HS_NOANSWER:
        snprintf(a, sizeof(a), "NO ANSWER  OK=RESEND");
        jobSubject(b, sizeof(b));
        break;
      case HS_NOPANEL:
        snprintf(a, sizeof(a), "NO PANEL   OK=RETRY");
        snprintf(b, sizeof(b), "%s", c.linkText ? c.linkText : "");
        break;
      case HS_DUPWARN: {
        char age[16];
        const int32_t s = state_.lastAgeTenths / 10;
        if (s < 100) snprintf(age, sizeof(age), "%ldS", (long)s);
        else snprintf(age, sizeof(age), "%ldM", (long)(s / 60));
        if (state_.lastHaveW) snprintf(a, sizeof(a), "LAST %ld LB %s AGO", (long)state_.lastW, age);
        else snprintf(a, sizeof(a), "LAST REC %s AGO", age);
        snprintf(b, sizeof(b), "OK=AGAIN  BACK=STOP");
        break;
      }
      case HS_MENU: renderList(a, b, sizeof(a), menuCur_, menuTop_, MENU_COUNT, &HmiMenu::menuItem, c); break;
      case HS_CONFIRM: renderConfirm(a, b, sizeof(a), c); break;
      case HS_TALLIES:
        if (talliesCount() == 0) snprintf(a, sizeof(a), " NO TALLY OPEN");
        else renderList(a, b, sizeof(a), listCur_, listTop_, talliesCount(), &HmiMenu::talliesItem, c);
        break;
      case HS_TALLY: {
        const PanelTally &t = state_.t[tallyIdx_];
        tallyLine(t, a, sizeof(a));
        snprintf(b, sizeof(b), ">%s", tallyChoice_ == 0 ? "SELECT" : "END TALLY");
        break;
      }
      case HS_DELIVERIES:
        if (!dl_.enabled) { snprintf(a, sizeof(a), "NO DIRECTOR LIST"); snprintf(b, sizeof(b), "TYPE IT ON THE PHONE"); }
        else if (dl_.count == 0) snprintf(a, sizeof(a), "NO BOOKED DELIVERY");
        else renderList(a, b, sizeof(a), listCur_, listTop_, dl_.count, &HmiMenu::deliveryItem, c);
        break;
      case HS_INFO:
        if (c.nfc == NFC_OK) snprintf(a, sizeof(a), "NFC PN532 FW %u.%u", (unsigned)c.nfcVer, (unsigned)c.nfcRev);
        else if (c.nfc == NFC_STARTING) snprintf(a, sizeof(a), "NFC STARTING");
        else snprintf(a, sizeof(a), "NFC NOT ANSWERING");
        snprintf(b, sizeof(b), "PANEL %-4sb %.8s",
                 lastAnswer_ == 1 ? "OK" : lastAnswer_ == 2 ? "ERR" : "--", c.build ? c.build : "");
        break;
      case HS_RFIDOFF:
        snprintf(a, sizeof(a), "PANEL IS RFID BUILD");
        snprintf(b, sizeof(b), "BUTTONS OFF");
        break;
    }
    pad20(r1, a);
    pad20(r2, b);
  }

  // The status the INFO row and `nfc` report for the panel: 0 none yet, 1 answered, 2 failed.
  uint8_t lastAnswer() const { return lastAnswer_; }

 private:
  // --- the job: what the current flow is doing --------------------------------
  struct Job {
    HmiOp op;
    HmiPhase phase;
    HmiPhase failedAt;          // the phase a NO PANEL retry restarts from
    char id[24];                // the minted id, fixed for the job's resends
    char tag[TAG_HEX_LEN];      // fixed at mint: a resend carries the first send's tag (§4.5)
    uint16_t mc;                // fixed at mint, likewise (simple_tally.py:437-448)
    bool into;                  // `into: daily_log`
    bool select;                // POST /api/tally/select before the cmd (a delivery RECORD)
    char tally[PANEL_TALLY_ID_MAX];
    char label[PANEL_LABEL_MAX];
    int64_t picking;
    uint32_t acceptedAt;
    bool verdictIn, verdictOk, local;
    long verdictW;
    char detail[21];
  };

  // --- screens ---------------------------------------------------------------
  void go(HmiScreen s, const HmiContext &c) { scr_ = s; shownAt_ = c.now; lastPress_ = c.now; }
  void goHome(const HmiContext &c) {
    if (!jobMayRecord()) { abandonFetch(); endJob(); }
    retryFetch_ = OP_NONE;
    clearTag();
    go(HS_HOME, c);
  }
  void clearTag() { tag_[0] = '\0'; mc_ = MC_NONE; }

  // A job that has sent, or may have sent, a command must not be dropped while it waits.
  bool jobMayRecord() const { return job_.op != OP_NONE && scr_ == HS_BUSY; }

  void openTag(const char *hex, const HmiContext &c) {
    if (job_.op != OP_NONE) { abandonFetch(); endJob(); }
    if (strcmp(hex, tag_) != 0) mc_ = MC_NONE;   // a different tag clears the mc (the pendant's McLatch)
    snprintf(tag_, sizeof(tag_), "%s", hex);
    tagAt_ = c.now;
    dest_ = 0;
    tagLoad_ = full ? 1 : 0;
    go(HS_TAG, c);
    if (full) beginFetch(OP_FETCH_TAG, HS_TAG, c);
  }

  void tagButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_BACK && !e.repeat) { goHome(c); return; }
    if (!full) {
      if (e.id == BTN_OK && !e.repeat) startRecord(true, c);
      return;
    }
    if (tagLoad_ == 1) return;                              // still reading the panel
    if (tagLoad_ == 2) {                                    // the read failed: OK reads again
      if (e.id == BTN_OK && !e.repeat) { tagLoad_ = 1; beginFetch(OP_FETCH_TAG, HS_TAG, c); }
      return;
    }
    const uint8_t n = state_.count;
    if (e.id == BTN_UP && n > 1) { if (dest_ > 0) dest_--; return; }
    if (e.id == BTN_DOWN) {
      if (!e.repeat) { destBeforeDown_ = dest_; if (n > 1 && dest_ + 1 < n) dest_++; return; }
      // DOWN held to its first repeat opens MC (design §4.14), and undoes the step its press made.
      if (mcAllowed() && !mcOpenedThisHold_) {
        dest_ = destBeforeDown_;
        mcEdit_ = mc_ ? mc_ : MC_START_TENTHS;
        mcOpenedThisHold_ = true;
        go(HS_MC, c);
      }
      return;
    }
    if (e.id == BTN_OK && !e.repeat) startRecord(true, c);
  }

  void mcButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_UP)   { mcEdit_ = mcStepUp(mcEdit_); return; }
    if (e.id == BTN_DOWN) {
      // The repeats of the very hold that opened this screen do not step it.
      if (e.repeat && mcOpenedThisHold_) return;
      mcOpenedThisHold_ = false;
      mcEdit_ = mcStepDown(mcEdit_);
      return;
    }
    mcOpenedThisHold_ = false;
    if (e.repeat) return;
    if (e.id == BTN_OK) { mc_ = mcEdit_; go(HS_TAG, c); return; }
    if (e.id == BTN_BACK) { go(HS_TAG, c); return; }
  }

  bool mcAllowed() const { return full && mcFeature && haveState_ && state_.mcEnabled && tag_[0]; }

  void menuButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_UP)   { if (menuCur_ > 0) menuCur_--; scroll(menuCur_, menuTop_); return; }
    if (e.id == BTN_DOWN) { if (menuCur_ + 1 < MENU_COUNT) menuCur_++; scroll(menuCur_, menuTop_); return; }
    if (e.repeat) return;
    if (e.id == BTN_BACK) { goHome(c); return; }
    if (e.id != BTN_OK) return;
    if (rfid_) { go(HS_RFIDOFF, c); return; }
    switch (menuCur_) {
      case 0: startRecord(false, c); return;                      // RECORD NO TAG
      case 1: openConfirm(CF_TARE, HS_MENU, c); return;
      case 2: openConfirm(CF_UNDO, HS_MENU, c); return;
      case 3: beginFetch(OP_FETCH_TALLIES, HS_MENU, c); return;
      case 4: openConfirm(CF_CLEAR, HS_MENU, c); return;
      case 5: go(HS_INFO, c); return;
    }
  }

  void openConfirm(HmiConfirm k, HmiScreen back, const HmiContext &c) {
    confirm_ = k;
    confirmBack_ = back;
    go(HS_CONFIRM, c);
  }

  void confirmButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_BACK) { go(confirmBack_, c); return; }
    if (e.id != BTN_OK) return;
    if (rfid_) { go(HS_RFIDOFF, c); return; }
    switch (confirm_) {
      case CF_TARE:  startVerb(OP_TARE, c); return;
      case CF_UNDO:  startVerb(OP_UNDO, c); return;
      case CF_CLEAR: startVerb(OP_CLEAR, c); return;
      case CF_START_DAILY: startTally(OP_START_DAILY, nullptr, "DAILY LOG", -1, c); return;
      case CF_START_DELIVERY: {
        const PanelDelivery &d = dl_.d[listCur_];
        startTally(OP_START_DELIVERY, nullptr, d.label, d.pickingId, c);
        return;
      }
      case CF_END: {
        const PanelTally &t = state_.t[tallyIdx_];
        startTally(OP_END, t.id, t.label, -1, c);
        return;
      }
    }
  }

  uint8_t talliesCount() const {
    return (uint8_t)(state_.count + (state_.canStartDaily ? 1 : 0) + (state_.canStartDelivery ? 1 : 0));
  }
  // Which TALLIES row is which: the open tallies, then "+ DAILY LOG", then "+ DELIVERY".
  int talliesKind(uint8_t i) const {   // >= 0 a tally index; -1 start daily; -2 start delivery
    if (i < state_.count) return i;
    uint8_t k = (uint8_t)(i - state_.count);
    if (state_.canStartDaily) { if (k == 0) return -1; k--; }
    return -2;
  }

  void talliesButton(const BtnEvent &e, const HmiContext &c) {
    const uint8_t n = talliesCount();
    if (e.id == BTN_UP)   { if (listCur_ > 0) listCur_--; scroll(listCur_, listTop_); return; }
    if (e.id == BTN_DOWN) { if (listCur_ + 1 < n) listCur_++; scroll(listCur_, listTop_); return; }
    if (e.repeat) return;
    if (e.id == BTN_BACK) { go(HS_MENU, c); return; }
    if (e.id != BTN_OK || n == 0) return;
    if (rfid_) { go(HS_RFIDOFF, c); return; }
    const int k = talliesKind(listCur_);
    if (k >= 0) { tallyIdx_ = (uint8_t)k; tallyChoice_ = 0; go(HS_TALLY, c); }
    else if (k == -1) openConfirm(CF_START_DAILY, HS_TALLIES, c);
    else beginFetch(OP_FETCH_DELIVERIES, HS_TALLIES, c);
  }

  void tallyButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_UP || e.id == BTN_DOWN) { tallyChoice_ = (uint8_t)(1 - tallyChoice_); return; }
    if (e.repeat) return;
    if (e.id == BTN_BACK) { go(HS_TALLIES, c); return; }
    if (e.id != BTN_OK) return;
    if (rfid_) { go(HS_RFIDOFF, c); return; }
    const PanelTally &t = state_.t[tallyIdx_];
    if (tallyChoice_ == 0) startTally(OP_SELECT, t.id, t.label, -1, c);
    else openConfirm(CF_END, HS_TALLY, c);
  }

  void deliveriesButton(const BtnEvent &e, const HmiContext &c) {
    const uint8_t n = dl_.enabled ? dl_.count : 0;
    if (e.id == BTN_UP)   { if (listCur_ > 0) listCur_--; scroll(listCur_, listTop_); return; }
    if (e.id == BTN_DOWN) { if (listCur_ + 1 < n) listCur_++; scroll(listCur_, listTop_); return; }
    if (e.repeat) return;
    if (e.id == BTN_BACK) { listCur_ = 0; listTop_ = 0; go(HS_TALLIES, c); return; }
    if (e.id != BTN_OK || n == 0) return;
    if (rfid_) { go(HS_RFIDOFF, c); return; }
    if (dl_.d[listCur_].pickingId < 0) return;
    openConfirm(CF_START_DELIVERY, HS_DELIVERIES, c);
  }

  void dupButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_OK) { job_.phase = job_.select ? PH_SELECT : PH_CMD; advance(c); return; }
    if (e.id == BTN_BACK) { endJob(); if (tag_[0]) go(HS_TAG, c); else goHome(c); }
  }

  void failedButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_BACK) { endJob(); if (tag_[0]) go(HS_TAG, c); else goHome(c); return; }
    if (e.id != BTN_OK) return;
    // A refusal is retried under a NEW id: a repeat of the old one would replay the refusal
    // (references/03_mqtt_contract.md:380-381).
    const HmiOp op = job_.op;
    if (op == OP_RECORD) { const bool t = job_.tag[0] != 0; endJob(); startRecord(t, c); return; }
    if (op == OP_TARE || op == OP_UNDO || op == OP_CLEAR) { endJob(); startVerb(op, c); return; }
    Job j = job_;
    endJob();
    startTally(op, j.tally[0] ? j.tally : nullptr, j.label, j.picking, c);
  }

  // NO ANSWER resends under the SAME id: the panel pins the tag and mc to it and the board's
  // dedup ring replays rather than weighing twice (design §4.5). NO PANEL retries the same way,
  // from the phase that failed: a request that may have reached the panel is never re-minted.
  void retryButton(const BtnEvent &e, const HmiContext &c) {
    if (e.id == BTN_BACK) {
      endJob();
      if (retryFetch_ != OP_NONE) { const HmiScreen b = retryFrom_; retryFetch_ = OP_NONE; go(b, c); return; }
      if (tag_[0]) go(HS_TAG, c); else goHome(c);
      return;
    }
    if (e.id != BTN_OK) return;
    if (retryFetch_ != OP_NONE) { const HmiOp op = retryFetch_; retryFetch_ = OP_NONE; beginFetch(op, retryFrom_, c); return; }
    if (job_.op == OP_NONE) return;
    if (scr_ == HS_NOANSWER) job_.phase = PH_CMD;
    else job_.phase = job_.failedAt;
    job_.verdictIn = false;
    advance(c);
  }

  // --- starting flows ----------------------------------------------------------
  void beginJob(HmiOp op) {
    memset(&job_, 0, sizeof(job_));
    job_.op = op;
    job_.picking = -1;
  }
  void endJob() { memset(&job_, 0, sizeof(job_)); job_.op = OP_NONE; }

  void beginFetch(HmiOp op, HmiScreen from, const HmiContext &c) {
    beginJob(op);
    fetchFrom_ = from;
    if (op != OP_FETCH_TAG && op != OP_RFID_CHECK) go(HS_LOADING, c);
    ask(op == OP_FETCH_DELIVERIES ? PK_DELIVERIES : PK_STATE, "", c);
  }
  void abandonFetch() {
    want_ = PK_NONE;
    expect_ = 0;
  }
  void backFromLoading(const HmiContext &c) {
    const HmiScreen back = fetchFrom_;
    endJob();
    go(back == HS_LOADING ? HS_MENU : back, c);
  }

  bool mint(const HmiContext &) {
    if (!minter) return false;
    return minter->mint(job_.id, sizeof(job_.id));
  }

  void startRecord(bool withTag, const HmiContext &c) {
    beginJob(OP_RECORD);
    if (withTag && tag_[0]) { snprintf(job_.tag, sizeof(job_.tag), "%s", tag_); job_.mc = mc_; }
    job_.into = true;
    snprintf(job_.label, sizeof(job_.label), "DAILY LOG");
    if (full && withTag && haveState_ && tagLoad_ == 0) {
      if (state_.count == 0) { job_.into = false; job_.label[0] = '\0'; }   // NO TALLY: the board's tally only
      else {
        const PanelTally &t = state_.t[dest_ < state_.count ? dest_ : 0];
        snprintf(job_.label, sizeof(job_.label), "%s", t.label);
        if (t.delivery) {
          if (!panelTallyIdSafe(t.id)) { fail("BAD TALLY ID", c); return; }
          job_.into = false; job_.select = true;
          snprintf(job_.tally, sizeof(job_.tally), "%s", t.id);
        }
      }
    }
    if (!mint(c)) { fail("NO ID", c); return; }
    // Part (b) reads /api/state first: the RFID check, and DUP WARN (design §4.6, §4.13).
    job_.phase = full ? PH_STATE : PH_CMD;
    advance(c);
  }

  void startVerb(HmiOp op, const HmiContext &c) {
    beginJob(op);
    if (!mint(c)) { fail("NO ID", c); return; }
    job_.phase = PH_CMD;
    advance(c);
  }

  void startTally(HmiOp op, const char *tally, const char *label, int64_t picking, const HmiContext &c) {
    beginJob(op);
    if (tally) snprintf(job_.tally, sizeof(job_.tally), "%s", tally);
    snprintf(job_.label, sizeof(job_.label), "%s", label ? label : "");
    job_.picking = picking;
    // A start needs the panel's clock for t_ms; read /api/state first if it is not known yet.
    job_.phase = ((op == OP_START_DAILY || op == OP_START_DELIVERY) && !c.haveEpoch) ? PH_STATE : PH_POST;
    advance(c);
  }

  // Issue the request for the job's current phase.
  void advance(const HmiContext &c) {
    go(HS_BUSY, c);
    char body[PANEL_BODY_MAX];
    switch (job_.phase) {
      case PH_STATE: ask(PK_STATE, "", c); return;
      case PH_SELECT:
        if (!panelTallyBody(body, sizeof(body), job_.tally)) { fail("BAD TALLY ID", c); return; }
        ask(PK_SELECT, body, c);
        return;
      case PH_CMD: {
        const char *verb = job_.op == OP_RECORD ? "+" : job_.op == OP_TARE ? "tare" : job_.op == OP_UNDO ? "-" : "b";
        if (!panelCmdBody(body, sizeof(body), job_.id, verb, job_.into, job_.tag, job_.mc)) { fail("BAD REQUEST", c); return; }
        ask(PK_CMD, body, c);
        return;
      }
      case PH_POST: {
        size_t n = 0;
        PanelKind k = PK_START;
        if (job_.op == OP_SELECT) { k = PK_SELECT; n = panelTallyBody(body, sizeof(body), job_.tally); }
        else if (job_.op == OP_END) { k = PK_END; n = panelTallyBody(body, sizeof(body), job_.tally); }
        else if (c.haveEpoch) n = panelStartBody(body, sizeof(body), c.epochMs, job_.op == OP_START_DELIVERY ? job_.picking : -1);
        if (!n) { fail(c.haveEpoch ? "BAD REQUEST" : "NO PANEL CLOCK", c); return; }
        ask(k, body, c);
        return;
      }
      case PH_LOCAL: localWant_ = true; return;
      default: return;
    }
  }

  void ask(PanelKind k, const char *body, const HmiContext &c) {
    want_ = k;
    if (++seq_ == 0) seq_ = 1;
    wantSeq_ = seq_;
    expect_ = seq_;
    snprintf(wantBody_, sizeof(wantBody_), "%s", body ? body : "");
    wantSince_ = c.now;
  }

  // --- results -------------------------------------------------------------------
  void fetchTagDone(PanelOutcome o, int status, const HmiContext &c) {
    endJob();
    if (scr_ != HS_TAG) return;
    if (o != PO_HTTP || status != 200 || !haveState_) { tagLoad_ = 2; return; }
    if (rfid_) { clearTag(); go(HS_RFIDOFF, c); return; }
    tagLoad_ = 0;
    dest_ = 0;
    for (uint8_t i = 0; i < state_.count; i++) if (state_.t[i].selected) dest_ = i;
  }

  void fetchListDone(HmiOp op, PanelOutcome o, int status, HmiScreen to, const HmiContext &c) {
    const HmiScreen from = fetchFrom_;
    endJob();
    if (scr_ != HS_LOADING) return;
    if (o != PO_HTTP || status != 200) {
      retryFetch_ = op; retryFrom_ = from;
      go(HS_NOPANEL, c);
      return;
    }
    if (rfid_) { go(HS_RFIDOFF, c); return; }
    listCur_ = 0; listTop_ = 0;
    go(to, c);
  }

  void actionResult(PanelKind kind, PanelOutcome o, int status, const PanelReply &rp, const HmiContext &c) {
    (void)kind;
    if (o != PO_HTTP) {
      // TARE with no panel runs locally through the same handler (design §5.5). UNDO and CLEAR
      // never do: the panel tracks the board's records.
      if (job_.op == OP_TARE && job_.phase == PH_CMD) { job_.phase = PH_LOCAL; advance(c); return; }
      job_.failedAt = job_.phase;
      go(HS_NOPANEL, c);
      return;
    }
    switch (job_.phase) {
      case PH_STATE:
        if (status != 200) { panelRefused(status, rp, c); return; }
        if (rfid_) { endJob(); clearTag(); go(HS_RFIDOFF, c); return; }
        if (job_.op == OP_RECORD) {
          if (dupWarns()) { job_.phase = PH_PAUSED; go(HS_DUPWARN, c); return; }
          job_.phase = job_.select ? PH_SELECT : PH_CMD;
        } else {
          job_.phase = PH_POST;
        }
        advance(c);
        return;
      case PH_SELECT:
        if (status != 200) { panelRefused(status, rp, c); return; }
        job_.phase = PH_CMD;
        advance(c);
        return;
      case PH_CMD:
        if (status != 202) { panelRefused(status, rp, c); return; }
        job_.phase = PH_VERDICT;
        job_.acceptedAt = c.now;
        if (job_.verdictIn) finishVerdict(c);
        return;
      case PH_POST:
        if (status != 200 && status != 202) { panelRefused(status, rp, c); return; }
        if (job_.op == OP_SELECT) { beginFetch(OP_FETCH_TALLIES, HS_TALLIES, c); return; }
        job_.verdictOk = true;
        go(HS_DONE, c);
        return;
      default: return;
    }
  }

  bool dupWarns() const {
    return state_.lastHaveAge && state_.lastHaveEmptied && !state_.lastEmptied &&
           state_.lastAgeTenths < HMI_DUP_WINDOW_TENTHS;
  }

  // NOT RECORDED's row 2 from a panel refusal: the machine's own state name on a 409 from the
  // action gate (app.py:328-339), else PANEL <status>. The full error goes to the console.
  void panelRefused(int status, const PanelReply &rp, const HmiContext &c) {
    failReasonFrom(PO_HTTP, status, &rp);
    snprintf(lastError_, sizeof(lastError_), "%s", rp.error);
    go(HS_FAILED, c);
  }
  void failReasonFrom(PanelOutcome o, int status, const PanelReply *rp) {
    if (o == PO_HTTP && status == 409 && rp && rp->stateName[0]) snprintf(failReason_, sizeof(failReason_), "%s", rp->stateName);
    else if (o == PO_HTTP) snprintf(failReason_, sizeof(failReason_), "PANEL %d", status);
    else snprintf(failReason_, sizeof(failReason_), "NO PANEL");
  }
  void fail(const char *why, const HmiContext &c) {
    snprintf(failReason_, sizeof(failReason_), "%s", why);
    go(HS_FAILED, c);
  }

  void finishVerdict(const HmiContext &c) {
    if (job_.verdictOk) {
      // A tag is used up when the board records with it, and its mc with it.
      if (job_.op == OP_RECORD && job_.tag[0] && strcmp(job_.tag, tag_) == 0) clearTag();
      go(HS_DONE, c);
    } else {
      snprintf(failReason_, sizeof(failReason_), "%s", job_.detail);
      go(HS_FAILED, c);
    }
  }

  // A result screen has ended: a tag held during the wait opens now, else HOME.
  void resultEnded(const HmiContext &c) {
    const bool select = job_.op == OP_SELECT;
    endJob();
    if (held_[0] && (uint32_t)(c.now - heldAt_) < TAG_ARMED_MS) {
      char h[TAG_HEX_LEN];
      snprintf(h, sizeof(h), "%s", held_);
      held_[0] = '\0';
      openTag(h, c);
      return;
    }
    held_[0] = '\0';
    if (select) { go(HS_TALLIES, c); return; }
    if (tag_[0]) go(HS_TAG, c); else goHome(c);
  }

  // --- rendering -----------------------------------------------------------------
  static void pad20(char *out, const char *in) {
    size_t n = strlen(in);
    if (n > 20) n = 20;
    memcpy(out, in, n);
    for (size_t i = n; i < 20; i++) out[i] = ' ';
    out[20] = '\0';
  }

  void renderTag(char *a, char *b, size_t cap) const {
    tagRowText(tag_, a, cap);
    if (!full) { snprintf(b, cap, "DAILY LOG     OK=REC"); return; }
    if (tagLoad_ == 1) { snprintf(b, cap, "READING PANEL..."); return; }
    if (tagLoad_ == 2) { snprintf(b, cap, "NO PANEL   OK=RETRY"); return; }
    char label[PANEL_LABEL_MAX];
    if (state_.count == 0) snprintf(label, sizeof(label), "NO TALLY");
    else snprintf(label, sizeof(label), "%s", state_.t[dest_].label[0] ? state_.t[dest_].label : "TALLY");
    if (mcAllowed()) {
      // The pendant's indicator beside the tag, "MC --" or "MC 18.5" (mesh PlcPanel.cpp). It takes
      // the place of OK=REC: 12 columns of label, a space, 7 of MC.
      char ind[12];
      mcIndicator(mc_, ind, sizeof(ind));
      snprintf(b, cap, "%-12.12s %-7s", label, ind);
    } else {
      snprintf(b, cap, "%-14.13sOK=REC", label);
    }
  }

  const char *busyWord() const {
    switch (job_.op) {
      case OP_RECORD: return "RECORDING...";
      case OP_TARE:   return job_.phase == PH_LOCAL ? "TARING LOCALLY..." : "TARING...";
      case OP_UNDO:   return "REMOVING...";
      case OP_CLEAR:  return "CLEARING...";
      case OP_SELECT: return "SELECTING...";
      case OP_END:    return "ENDING...";
      default:        return "STARTING...";
    }
  }
  const char *failWord() const {
    switch (job_.op) {
      case OP_TARE:   return "NOT TARED";
      case OP_UNDO:   return "NOT REMOVED";
      case OP_CLEAR:  return "NOT CLEARED";
      case OP_SELECT: return "NOT SELECTED";
      case OP_END:    return "NOT ENDED";
      case OP_START_DAILY:
      case OP_START_DELIVERY: return "NOT STARTED";
      default:        return "NOT RECORDED";
    }
  }
  void jobSubject(char *b, size_t cap) const {
    if (job_.op == OP_RECORD && job_.tag[0]) tagRowText(job_.tag, b, cap);
    else if (job_.op == OP_RECORD) snprintf(b, cap, "RECORD NO TAG");
    else if (job_.op == OP_TARE) snprintf(b, cap, "TARE");
    else if (job_.op == OP_UNDO) snprintf(b, cap, "UNDO LAST");
    else if (job_.op == OP_CLEAR) snprintf(b, cap, "CLEAR BOARD");
    else snprintf(b, cap, "%s", job_.label);
  }
  void renderDone(char *a, char *b, size_t cap) const {
    switch (job_.op) {
      case OP_RECORD:
        snprintf(a, cap, "RECORDED %8ld LB", job_.verdictW);
        if (job_.label[0]) snprintf(b, cap, "INTO %s", job_.label);
        else snprintf(b, cap, "BOARD TALLY ONLY");
        return;
      case OP_TARE:  snprintf(a, cap, "TARED"); if (job_.local) snprintf(b, cap, "LOCAL - NO PANEL"); return;
      case OP_UNDO:  snprintf(a, cap, "REMOVED"); return;
      case OP_CLEAR: snprintf(a, cap, "CLEARED"); return;
      case OP_END:   snprintf(a, cap, "TALLY ENDED"); snprintf(b, cap, "%s", job_.label); return;
      default:       snprintf(a, cap, "STARTED"); snprintf(b, cap, "%s", job_.label); return;
    }
  }

  void renderConfirm(char *a, char *b, size_t cap, const HmiContext &c) const {
    switch (confirm_) {
      case CF_TARE:  snprintf(a, cap, "TARE: EMPTY AT MARK?"); snprintf(b, cap, "OK=TARE   BACK=NO"); return;
      case CF_UNDO:
        if (c.entries > 0) snprintf(a, cap, "UNDO LAST? %ld LB", c.lastEntryW);
        else snprintf(a, cap, "UNDO LAST? NONE");
        snprintf(b, cap, "OK=UNDO   BACK=NO");
        return;
      case CF_CLEAR: snprintf(a, cap, "CLEAR %d ENTRIES?", c.entries); snprintf(b, cap, "OK=CLEAR  BACK=NO"); return;
      case CF_START_DAILY: snprintf(a, cap, "START DAILY LOG?"); snprintf(b, cap, "OK=START  BACK=NO"); return;
      case CF_START_DELIVERY: snprintf(a, cap, "START %.13s?", dl_.d[listCur_].label); snprintf(b, cap, "OK=START  BACK=NO"); return;
      case CF_END: snprintf(a, cap, "END %.15s?", state_.t[tallyIdx_].label); snprintf(b, cap, "OK=END    BACK=NO"); return;
    }
  }

  static const uint8_t MENU_COUNT = 6;
  void menuItem(uint8_t i, char *out, size_t cap, const HmiContext &) const {
    static const char *const ITEMS[MENU_COUNT] = {"RECORD NO TAG", "TARE", "UNDO LAST", "TALLIES", "CLEAR BOARD", "INFO"};
    snprintf(out, cap, "%s", ITEMS[i]);
  }
  void tallyLine(const PanelTally &t, char *out, size_t cap) const {
    char n[16] = "";
    if (t.haveN) snprintf(n, sizeof(n), "n%ld", (long)t.n);
    snprintf(out, cap, "%-12.11s%-5s%c", t.label[0] ? t.label : "TALLY", n, t.selected ? '*' : ' ');
  }
  void talliesItem(uint8_t i, char *out, size_t cap, const HmiContext &) const {
    const int k = talliesKind(i);
    if (k >= 0) tallyLine(state_.t[k], out, cap);
    else snprintf(out, cap, "%s", k == -1 ? "+ DAILY LOG" : "+ DELIVERY");
  }
  void deliveryItem(uint8_t i, char *out, size_t cap, const HmiContext &) const {
    // `T` marks tomorrow, in column 19: column 20 carries the list's ^ and v.
    snprintf(out, cap, "%-17.17s%c", dl_.d[i].label[0] ? dl_.d[i].label : "DELIVERY", dl_.d[i].tomorrow ? 'T' : ' ');
  }

  static void scroll(uint8_t cur, uint8_t &top) {
    if (cur < top) top = cur;
    if (cur > top + 1) top = (uint8_t)(cur - 1);
  }

  typedef void (HmiMenu::*ItemFn)(uint8_t, char *, size_t, const HmiContext &) const;
  void renderList(char *a, char *b, size_t cap, uint8_t cur, uint8_t top, uint8_t n, ItemFn fn,
                  const HmiContext &c) const {
    char *rows[2] = {a, b};
    for (uint8_t r = 0; r < 2; r++) {
      const uint8_t i = (uint8_t)(top + r);
      char item[24] = "";
      if (i < n) (this->*fn)(i, item, sizeof(item), c);
      char line[24];
      snprintf(line, sizeof(line), "%c%-18.18s", (i < n && i == cur) ? '>' : ' ', item);
      // Column 20: ^ on the top row when items lie above, v on the bottom row when items lie below.
      const char mark = (r == 0 && top > 0) ? '^' : (r == 1 && top + 2 < n) ? 'v' : ' ';
      snprintf(rows[r], cap, "%-19.19s%c", i < n ? line : "", mark);
    }
  }

  // --- state ---------------------------------------------------------------------
  HmiScreen scr_ = HS_HOME;
  uint32_t shownAt_ = 0, lastPress_ = 0;
  Job job_ = {};
  PanelKind want_ = PK_NONE;
  char wantBody_[PANEL_BODY_MAX] = "";
  uint32_t wantSince_ = 0;
  uint16_t seq_ = 0, wantSeq_ = 0, expect_ = 0;
  HmiOp retryFetch_ = OP_NONE;       // NO PANEL after a list fetch: OK fetches it again
  HmiScreen retryFrom_ = HS_MENU;
  bool localWant_ = false;
  HmiScreen fetchFrom_ = HS_MENU;

  char tag_[TAG_HEX_LEN] = "";
  uint32_t tagAt_ = 0;
  uint16_t mc_ = MC_NONE, mcEdit_ = MC_NONE;
  bool mcOpenedThisHold_ = false;
  uint8_t tagLoad_ = 0;                // 0 loaded, 1 reading, 2 the read failed
  uint8_t dest_ = 0, destBeforeDown_ = 0;
  char held_[TAG_HEX_LEN] = "";
  uint32_t heldAt_ = 0;

  bool rfid_ = false;
  bool haveState_ = false;
  PanelState state_ = {};
  PanelDeliveries dl_ = {};
  uint8_t lastAnswer_ = 0;

  uint8_t menuCur_ = 0, menuTop_ = 0, listCur_ = 0, listTop_ = 0;
  uint8_t tallyIdx_ = 0, tallyChoice_ = 0;
  HmiConfirm confirm_ = CF_TARE;
  HmiScreen confirmBack_ = HS_MENU;
  char failReason_[21] = "";

 public:
  char lastError_[JSON_VAL_MAX] = "";   // the panel's last refusal text, for the console
};
