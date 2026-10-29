#pragma once
// FW-1: the on-board NFC reader, an NXP PN532 (Elechouse PN532 NFC RFID Module V3) on the
// header I2C bus `Wire` (A4/A5) at 7-bit 0x24. docs/design/2026-10-29-onboard-rfid-buttons-menus.md
// §2, §5.7, §6 FW-1.
//
// WRITTEN FRESH FROM THE NXP PN532 USER MANUAL, UM0701-02 Rev. 02. Not copied from the mesh
// pendant driver, which is GPL-3.0 (design §6 FW-1). The two drivers must agree on the rules in
// design §5.7; test/test_pn532.cpp pins this side of them.
//
// What the manual fixes, and where:
//   * Normal information frame, §6.2.1.1:
//       00 | 00 FF | LEN | LCS | TFI | PD0..PDn | DCS | 00
//     LEN counts TFI and the data bytes; LEN + LCS == 0 (mod 256); TFI + PD.. + DCS == 0.
//     TFI is D4 host -> PN532 and D5 PN532 -> host. A response's first data byte is the
//     command code plus one.
//   * ACK frame 00 00 FF 00 FF 00, §6.2.1.3.
//   * Error frame 00 00 FF 01 FF 7F 81 00 (application level error), §6.2.1.5.
//   * I2C, §6.2.4: write address 0x48, read 0x49 (7-bit 0x24). Every host read begins with a
//     status byte whose bit 0 is RDY; with RDY 0 the rest of the read is meaningless and the
//     host tries again later (§6.2.4.1). No IRQ line is needed.
//   * Commands, in the order this driver sends them:
//       SAMConfiguration    14 -> 15, mode 01 normal          §7.2.10 (leaves LowVbat)
//       GetFirmwareVersion  02 -> 03 IC Ver Rev Support        §7.2.2
//       RFConfiguration     32 -> 33, CfgItem 05 MaxRetries    §7.3.1
//       InListPassiveTarget 4A -> 4B, MaxTg 1, BrTy 00 (106 kbps type A)   §7.3.5
//     MxRtyPassiveActivation = 1, design §5.7: with no tag in the field the PN532 then answers
//     NbTg = 0 instead of retrying forever, so every poll ends in an answer and a silent reader
//     is a fault rather than an empty field.
//
// Pure C++, no Arduino: the bus is a template parameter, so the host test drives the same
// driver through a scripted fake PN532. The firmware gives it a Wire adapter.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------
static const uint8_t PN532_I2C_ADDR       = 0x24;   // UM0701-02 §6.2.4: 0x48 in 8-bit form
static const uint8_t PN532_TFI_HOST       = 0xD4;
static const uint8_t PN532_TFI_PN532      = 0xD5;
static const uint8_t PN532_CMD_GETFW      = 0x02;
static const uint8_t PN532_CMD_SAMCONFIG  = 0x14;
static const uint8_t PN532_CMD_RFCONFIG   = 0x32;
static const uint8_t PN532_CMD_INLIST     = 0x4A;
static const uint8_t PN532_RDY            = 0x01;   // status byte bit 0

// ISO/IEC 14443-3 type A UIDs are 4, 7 or 10 bytes (the three cascade levels). The parser
// accepts 1..10 and leaves the length rule to the wire-form check below.
static const size_t  PN532_UID_MAX        = 10;
// The wire form, upper-case hex with no separator: 20 characters and the terminator.
static const size_t  TAG_HEX_LEN          = 2 * PN532_UID_MAX + 1;

// The frame this driver builds is at most 8 header/trailer bytes plus the parameters.
static const size_t  PN532_FRAME_MAX      = 24;
// One read of a response: the status byte plus a frame. The longest answer this driver asks
// for is InListPassiveTarget with a 10-byte UID: 00 00 FF LEN LCS D5 4B NbTg Tg SENS_RES(2)
// SEL_RES NFCIDLength NFCID1(10) DCS 00 = 24 bytes. 32 leaves room for extra leading 00s.
static const size_t  PN532_READ_LEN       = 1 + 32;
// The ACK read: status byte plus the six ACK bytes.
static const size_t  PN532_ACK_READ_LEN   = 1 + 6;

// ---------------------------------------------------------------------------
// Frame build and parse
// ---------------------------------------------------------------------------
// Build a host -> PN532 information frame for `cmd` with `n` parameter bytes. Returns its
// length, or 0 when it would not fit in `cap`.
inline size_t pn532BuildFrame(uint8_t cmd, const uint8_t *params, size_t n,
                              uint8_t *out, size_t cap) {
  const size_t len = 2 + n;                 // TFI + command code + parameters
  const size_t total = 3 + 2 + len + 2;     // preamble+start, LEN+LCS, body, DCS+postamble
  if (len > 0xFF || total > cap) return 0;
  size_t i = 0;
  out[i++] = 0x00;
  out[i++] = 0x00;
  out[i++] = 0xFF;
  out[i++] = (uint8_t)len;
  out[i++] = (uint8_t)(0x100 - len);        // LEN + LCS == 0
  uint8_t sum = PN532_TFI_HOST;
  out[i++] = PN532_TFI_HOST;
  out[i++] = cmd;
  sum = (uint8_t)(sum + cmd);
  for (size_t k = 0; k < n; k++) {
    out[i++] = params[k];
    sum = (uint8_t)(sum + params[k]);
  }
  out[i++] = (uint8_t)(0x100 - sum);        // TFI + PD.. + DCS == 0
  out[i++] = 0x00;
  return i;
}

// The six ACK bytes, after the I2C status byte has been stripped.
inline bool pn532IsAck(const uint8_t *b, size_t n) {
  static const uint8_t ACK[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
  return n >= 6 && memcmp(b, ACK, 6) == 0;
}

enum Pn532Parse : uint8_t {
  PN532_OK = 0,
  PN532_NO_START,     // no 00 FF start code
  PN532_SHORT,        // the frame runs past the bytes read
  PN532_BAD_LCS,      // LEN + LCS != 0
  PN532_BAD_TFI,      // not D5
  PN532_WRONG_CMD,    // not the answer to the command asked
  PN532_BAD_DCS,      // TFI + data + DCS != 0
  PN532_ERROR_FRAME,  // the application-level error frame, §6.2.1.5
  PN532_ACK_FRAME     // an ACK where a response was expected
};

// Parse a PN532 -> host response, status byte already stripped. On PN532_OK, `*data` points at
// the bytes after the response code and `*dlen` counts them.
inline Pn532Parse pn532ParseResponse(const uint8_t *b, size_t n, uint8_t cmd,
                                     const uint8_t **data, size_t *dlen) {
  size_t i = 0;
  // Preamble: any number of 00s, then FF. The start code is the 00 FF pair.
  while (i < n && b[i] == 0x00) i++;
  if (i == 0 || i >= n || b[i] != 0xFF) return PN532_NO_START;
  i++;
  if (i + 2 > n) return PN532_SHORT;
  const uint8_t len = b[i], lcs = b[i + 1];
  if (len == 0x00 && lcs == 0xFF) return PN532_ACK_FRAME;
  if ((uint8_t)(len + lcs) != 0) return PN532_BAD_LCS;
  i += 2;
  if (i + len + 1 > n) return PN532_SHORT;  // body plus DCS
  if (len == 1 && b[i] == 0x7F) return PN532_ERROR_FRAME;
  if (len < 2) return PN532_SHORT;
  if (b[i] != PN532_TFI_PN532) return PN532_BAD_TFI;
  uint8_t sum = 0;
  for (size_t k = 0; k < len; k++) sum = (uint8_t)(sum + b[i + k]);
  if ((uint8_t)(sum + b[i + len]) != 0) return PN532_BAD_DCS;
  if (b[i + 1] != (uint8_t)(cmd + 1)) return PN532_WRONG_CMD;
  *data = b + i + 2;
  *dlen = (size_t)len - 2;
  return PN532_OK;
}

// GetFirmwareVersion's answer, §7.2.2: IC, Ver, Rev, Support. IC is 0x32 on a PN532.
inline bool pn532ParseFirmware(const uint8_t *d, size_t n, uint8_t &ic, uint8_t &ver, uint8_t &rev) {
  if (n < 4) return false;
  ic = d[0]; ver = d[1]; rev = d[2];
  return true;
}

// InListPassiveTarget's answer for 106 kbps type A, §7.3.5:
//   NbTg, then per target Tg SENS_RES(2) SEL_RES NFCIDLength NFCID1[...] [ATS...]
// Returns the UID length (1..10) and copies the UID; 0 for "no target"; -1 for a malformed answer.
inline int pn532ParseInList(const uint8_t *d, size_t n, uint8_t *uid) {
  if (n < 1) return -1;
  if (d[0] == 0) return 0;
  if (n < 6) return -1;
  const uint8_t idLen = d[5];
  if (idLen == 0 || idLen > PN532_UID_MAX || n < (size_t)6 + idLen) return -1;
  memcpy(uid, d + 6, idLen);
  return idLen;
}

// ---------------------------------------------------------------------------
// The tag's two text forms, design §5.7
// ---------------------------------------------------------------------------
// Wire form: upper-case hex, no separator. `out` needs 2*len+1. Returns false for a length the
// panel would refuse (it takes 2..20 digits: greybeard-plc4-pwa api/commands.py:306-313).
inline bool uidToHex(const uint8_t *uid, size_t len, char *out, size_t cap) {
  static const char kHexDigits[] = "0123456789ABCDEF";
  if (cap) out[0] = '\0';
  if (len < 1 || len > PN532_UID_MAX || cap < 2 * len + 1) return false;
  for (size_t i = 0; i < len; i++) {
    out[2 * i]     = kHexDigits[uid[i] >> 4];
    out[2 * i + 1] = kHexDigits[uid[i] & 0x0F];
  }
  out[2 * len] = '\0';
  return true;
}

// The panel's valid_pendant_tag(), api/commands.py:306-313: a string of upper-case hex digits,
// an even count, 2 to 20 of them. The board never sends a tag that fails this.
inline bool tagHexValid(const char *s) {
  if (!s) return false;
  size_t n = 0;
  for (; s[n]; n++) {
    const char c = s[n];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
  }
  return n >= 2 && n <= 20 && (n % 2) == 0;
}

// The colon form, "64:72:07:CA", as the pendant's hexShown() and the T-Deck spike print it.
// `out` needs 3*bytes. Empty for an invalid tag.
inline void tagColonForm(const char *hex, char *out, size_t cap) {
  if (cap) out[0] = '\0';
  if (!tagHexValid(hex)) return;
  const size_t n = strlen(hex);
  if (cap < n + n / 2) return;
  size_t o = 0;
  for (size_t i = 0; i < n; i += 2) {
    if (i) out[o++] = ':';
    out[o++] = hex[i];
    out[o++] = hex[i + 1];
  }
  out[o] = '\0';
}

// The TAG screen's row 1, design §4.4: "TAG " plus the colon form where both fit in 20
// columns (a 4-byte UID); the colon form alone where only it fits (a 7-byte UID is exactly
// 20); else the bare wire form (a 10-byte UID is 20 hex characters). `out` needs 21.
inline void tagRowText(const char *hex, char *out, size_t cap) {
  if (cap) out[0] = '\0';
  if (cap < 21 || !tagHexValid(hex)) return;
  char colon[3 * PN532_UID_MAX];
  tagColonForm(hex, colon, sizeof(colon));
  const size_t cl = strlen(colon);
  if (cl + 4 <= 20) { memcpy(out, "TAG ", 4); memcpy(out + 4, colon, cl + 1); }
  else if (cl <= 20) memcpy(out, colon, cl + 1);
  else { size_t h = strlen(hex); memcpy(out, hex, h + 1); }
}

// ---------------------------------------------------------------------------
// The scan rules, design §4.4 and §5.7 (the pendant's kGoneMs and kArmedMs)
// ---------------------------------------------------------------------------
static const uint32_t TAG_GONE_MS  = 2000;     // kGoneMs
static const uint32_t TAG_ARMED_MS = 120000;   // kArmedMs

// Decides which reads are new scans. A UID is a new scan when it differs from the last one
// seen, or when the reader has CONFIRMED the last one gone: polls it answered, finding no tag,
// spanning TAG_GONE_MS. A reader that stops answering confirms nothing, so a tag still on the
// reader across a dropout does not come back as a second scan.
struct TagTracker {
  char     last[TAG_HEX_LEN] = "";
  bool     absent = false;
  uint32_t absentFromMs = 0;
  uint32_t absentToMs = 0;

  bool goneConfirmed() const {
    return absent && (uint32_t)(absentToMs - absentFromMs) >= TAG_GONE_MS;
  }
  // The reader read `hex` at `now`. True when that is a new scan.
  bool seen(const char *hex, uint32_t now) {
    (void)now;
    const bool isNew = strcmp(hex, last) != 0 || last[0] == '\0' || goneConfirmed();
    strncpy(last, hex, sizeof(last) - 1);
    last[sizeof(last) - 1] = '\0';
    absent = false;
    return isNew;
  }
  // The reader answered a poll at `now` with no tag in its field.
  void empty(uint32_t now) {
    if (!absent) { absent = true; absentFromMs = now; }
    absentToMs = now;
  }
};

// ---------------------------------------------------------------------------
// The driver: step-based, never waits
// ---------------------------------------------------------------------------
// Each step() makes at most ONE bus transaction and returns. Between steps it holds its state,
// so the reader costs loop() one short I2C exchange at a time and never a wait for the PN532.
//
// Bus is any type with
//   bool write(const uint8_t *d, size_t n);   // one I2C write transaction to 0x24
//   bool read(uint8_t *d, size_t n);          // one I2C read transaction of exactly n bytes
enum Pn532Health : uint8_t { NFC_STARTING = 0, NFC_OK, NFC_NOT_ANSWERING };

static const uint32_t PN532_STATUS_GAP_MS   = 5;      // between RDY polls; keeps Wire quiet
static const uint32_t PN532_ACK_TIMEOUT_MS  = 50;     // §6.2.3 gives 15 ms for HSU; I2C is not stated
static const uint32_t PN532_RESP_TIMEOUT_MS = 500;    // InList with MxRtyPassiveActivation 1 answers in tens of ms
static const uint32_t PN532_POLL_GAP_MS     = 100;    // about ten polls a second, design §5.7
static const uint32_t PN532_RETRY_MS        = 1000;   // after a failure, restart from SAMConfiguration
static const uint32_t PN532_BACKOFF_MS      = 5000;   // after PN532_FAILS_DOWN failures in a row
static const uint8_t  PN532_FAILS_DOWN      = 3;

template <class Bus>
class Pn532Driver {
 public:
  explicit Pn532Driver(Bus &bus) : bus_(bus) {}

  Pn532Health health() const { return health_; }
  uint8_t fwVer() const { return fwVer_; }
  uint8_t fwRev() const { return fwRev_; }
  uint32_t polls() const { return polls_; }
  uint32_t reads() const { return reads_; }
  uint32_t fails() const { return fails_; }

  // Start over from SAMConfiguration at the next step (after `i2c` has walked the bus, say).
  void restart(uint32_t now) { st_ = ST_SEND; cmd_ = PN532_CMD_SAMCONFIG; nextAt_ = now; }

  // A UID read since the last call: true once per read, repeats included. TagTracker decides
  // which reads are new scans.
  bool takeUid(char *hex, size_t cap) {
    if (!haveUid_) return false;
    haveUid_ = false;
    strncpy(hex, uid_, cap - 1);
    hex[cap - 1] = '\0';
    return true;
  }
  // A poll the reader ANSWERED with no tag in its field, since the last call.
  bool takeEmpty() { const bool e = haveEmpty_; haveEmpty_ = false; return e; }

  // One step. Returns true when it made a bus transaction.
  bool step(uint32_t now) {
    if ((int32_t)(now - nextAt_) < 0) return false;
    switch (st_) {
      case ST_SEND: {
        uint8_t frame[PN532_FRAME_MAX];
        size_t n = 0;
        if (cmd_ == PN532_CMD_SAMCONFIG) {
          static const uint8_t p[] = {0x01, 0x14, 0x01};   // normal mode, timeout 1 s, use IRQ
          n = pn532BuildFrame(cmd_, p, sizeof(p), frame, sizeof(frame));
        } else if (cmd_ == PN532_CMD_GETFW) {
          n = pn532BuildFrame(cmd_, nullptr, 0, frame, sizeof(frame));
        } else if (cmd_ == PN532_CMD_RFCONFIG) {
          static const uint8_t p[] = {0x05, 0xFF, 0x01, 0x01};   // MaxRetries: ATR FF, PSL 01, PassiveActivation 01
          n = pn532BuildFrame(cmd_, p, sizeof(p), frame, sizeof(frame));
        } else {
          static const uint8_t p[] = {0x01, 0x00};   // MaxTg 1, 106 kbps type A
          n = pn532BuildFrame(cmd_, p, sizeof(p), frame, sizeof(frame));
        }
        if (!bus_.write(frame, n)) { fail(now); return true; }
        st_ = ST_ACK;
        sentAt_ = now;
        nextAt_ = now + PN532_STATUS_GAP_MS;
        return true;
      }
      case ST_ACK:
      case ST_RESP: {
        const bool ack = (st_ == ST_ACK);
        const uint32_t limit = ack ? PN532_ACK_TIMEOUT_MS : PN532_RESP_TIMEOUT_MS;
        if (!ready_) {
          uint8_t s = 0;
          const bool ok = bus_.read(&s, 1);
          if (ok && (s & PN532_RDY)) {
            ready_ = true;
            nextAt_ = now;            // the frame is read on the next step
          } else if ((uint32_t)(now - sentAt_) >= limit) {
            fail(now);
          } else {
            nextAt_ = now + PN532_STATUS_GAP_MS;
          }
          return true;
        }
        ready_ = false;
        uint8_t buf[PN532_READ_LEN];
        const size_t want = ack ? PN532_ACK_READ_LEN : PN532_READ_LEN;
        if (!bus_.read(buf, want) || !(buf[0] & PN532_RDY)) { fail(now); return true; }
        if (ack) {
          if (!pn532IsAck(buf + 1, want - 1)) { fail(now); return true; }
          st_ = ST_RESP;
          sentAt_ = now;
          nextAt_ = now + PN532_STATUS_GAP_MS;
          return true;
        }
        const uint8_t *d = nullptr;
        size_t dn = 0;
        if (pn532ParseResponse(buf + 1, want - 1, cmd_, &d, &dn) != PN532_OK) { fail(now); return true; }
        onResponse(d, dn, now);
        return true;
      }
    }
    return false;
  }

 private:
  enum St : uint8_t { ST_SEND, ST_ACK, ST_RESP };

  void onResponse(const uint8_t *d, size_t n, uint32_t now) {
    failRun_ = 0;
    st_ = ST_SEND;
    nextAt_ = now;
    if (cmd_ == PN532_CMD_SAMCONFIG) { cmd_ = PN532_CMD_GETFW; return; }
    if (cmd_ == PN532_CMD_GETFW) {
      uint8_t ic = 0, v = 0, r = 0;
      if (!pn532ParseFirmware(d, n, ic, v, r)) { fail(now); return; }
      fwVer_ = v; fwRev_ = r;
      cmd_ = PN532_CMD_RFCONFIG;
      return;
    }
    if (cmd_ == PN532_CMD_RFCONFIG) { health_ = NFC_OK; cmd_ = PN532_CMD_INLIST; return; }
    // InListPassiveTarget
    polls_++;
    health_ = NFC_OK;
    uint8_t uid[PN532_UID_MAX];
    const int len = pn532ParseInList(d, n, uid);
    if (len < 0) { fail(now); return; }
    if (len == 0) haveEmpty_ = true;
    else if (uidToHex(uid, (size_t)len, uid_, sizeof(uid_))) { haveUid_ = true; reads_++; }
    nextAt_ = now + PN532_POLL_GAP_MS;
  }

  void fail(uint32_t now) {
    fails_++;
    ready_ = false;
    if (failRun_ < 0xFF) failRun_++;
    // A failure in the middle of the poll loop is retried as a poll; one during start-up, or a
    // run of them, goes back to SAMConfiguration so a reader that power-cycled is set up again.
    const bool down = failRun_ >= PN532_FAILS_DOWN || cmd_ != PN532_CMD_INLIST;
    if (down) {
      if (failRun_ >= PN532_FAILS_DOWN || health_ == NFC_STARTING) health_ = NFC_NOT_ANSWERING;
      cmd_ = PN532_CMD_SAMCONFIG;
    }
    st_ = ST_SEND;
    nextAt_ = now + (failRun_ >= PN532_FAILS_DOWN ? PN532_BACKOFF_MS : PN532_RETRY_MS);
  }

  Bus &bus_;
  St st_ = ST_SEND;
  uint8_t cmd_ = PN532_CMD_SAMCONFIG;
  bool ready_ = false;
  uint32_t nextAt_ = 0;
  uint32_t sentAt_ = 0;
  uint8_t failRun_ = 0;
  Pn532Health health_ = NFC_STARTING;
  uint8_t fwVer_ = 0, fwRev_ = 0;
  uint32_t polls_ = 0, reads_ = 0, fails_ = 0;
  char uid_[TAG_HEX_LEN] = "";
  bool haveUid_ = false, haveEmpty_ = false;
};
