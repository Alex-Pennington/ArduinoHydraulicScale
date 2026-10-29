// Host check of the PN532 frame build and parse, the tag's text forms and scan rules, and the
// step-based driver, all in include/pn532.h (FW-1). Frames are worked by hand from NXP UM0701-02
// §6.2.1 and §7; the 4-byte UID is the one the T-Deck spike read (spikes/tdeck-nfc/README.md:59-63).
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_pn532.cpp -o "$TMPDIR/t_pn532"
//   "$TMPDIR/t_pn532"
//
// THIS IS NOT A COMMISSIONING TEST. It proves the protocol arithmetic and the driver's sequencing
// against a scripted PN532. It proves nothing about a reader on a tractor.

#include "pn532.h"
#include <cstdio>
#include <cstring>
#include <vector>

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  %s  %s\n", ok ? "ok   " : "FAIL ", what);
  if (!ok) failures++;
}
static void check_str(const char *what, const char *got, const char *want) {
  const bool ok = strcmp(got, want) == 0;
  if (ok) printf("  ok     %-46s %s\n", what, got);
  else { printf("  FAIL   %-46s got \"%s\" want \"%s\"\n", what, got, want); failures++; }
}
static bool bytes_eq(const uint8_t *a, size_t an, const std::vector<uint8_t> &b) {
  return an == b.size() && memcmp(a, b.data(), an) == 0;
}

// --- a scripted PN532 on the I2C bus ------------------------------------------
// It answers each command frame with an ACK and then the response the script holds, raising RDY
// only after `readyAfter` status reads, so the driver's polling is exercised.
struct FakePn532 {
  bool present = true;
  int readyAfter = 1;
  int statusReads = 0;
  int transactions = 0;
  std::vector<uint8_t> pending;   // what the next ready read returns, after the status byte
  bool ackNext = false;
  uint8_t lastCmd = 0;
  std::vector<uint8_t> uid;       // empty: no tag in the field
  bool corruptDcs = false;

  std::vector<uint8_t> respond(uint8_t cmd) {
    std::vector<uint8_t> data;
    data.push_back(cmd + 1);
    if (cmd == PN532_CMD_GETFW) { data.insert(data.end(), {0x32, 0x01, 0x06, 0x07}); }
    if (cmd == PN532_CMD_INLIST) {
      if (uid.empty()) data.push_back(0x00);
      else {
        data.insert(data.end(), {0x01, 0x01, 0x00, 0x04, 0x08, (uint8_t)uid.size()});
        data.insert(data.end(), uid.begin(), uid.end());
      }
    }
    std::vector<uint8_t> f = {0x00, 0x00, 0xFF};
    const uint8_t len = (uint8_t)(1 + data.size());
    f.push_back(len);
    f.push_back((uint8_t)(0x100 - len));
    uint8_t sum = PN532_TFI_PN532;
    f.push_back(PN532_TFI_PN532);
    for (uint8_t b : data) { f.push_back(b); sum = (uint8_t)(sum + b); }
    f.push_back((uint8_t)(0x100 - sum + (corruptDcs ? 1 : 0)));
    f.push_back(0x00);
    return f;
  }

  bool write(const uint8_t *d, size_t n) {
    transactions++;
    if (!present || n < 8) return false;
    lastCmd = d[6];
    ackNext = true;
    statusReads = 0;
    pending.clear();
    return true;
  }
  bool read(uint8_t *d, size_t n) {
    transactions++;
    if (!present) return false;
    memset(d, 0, n);
    const bool ready = statusReads >= readyAfter;
    if (n == 1) {
      statusReads++;
      d[0] = ready ? PN532_RDY : 0x00;
      return true;
    }
    if (!ready) { d[0] = 0; return true; }
    d[0] = PN532_RDY;
    std::vector<uint8_t> f;
    if (ackNext) { f = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00}; ackNext = false; statusReads = 0; }
    else f = respond(lastCmd);
    for (size_t i = 0; i < f.size() && i + 1 < n; i++) d[i + 1] = f[i];
    return true;
  }
};

int main() {
  printf("frame build (UM0701-02 §6.2.1.1)\n");
  {
    uint8_t f[PN532_FRAME_MAX];
    size_t n = pn532BuildFrame(PN532_CMD_GETFW, nullptr, 0, f, sizeof(f));
    check("GetFirmwareVersion = 00 00 FF 02 FE D4 02 2A 00", bytes_eq(f, n, {0x00,0x00,0xFF,0x02,0xFE,0xD4,0x02,0x2A,0x00}));
    const uint8_t sam[] = {0x01, 0x14, 0x01};
    n = pn532BuildFrame(PN532_CMD_SAMCONFIG, sam, 3, f, sizeof(f));
    check("SAMConfiguration 01 14 01, DCS 02", bytes_eq(f, n, {0x00,0x00,0xFF,0x05,0xFB,0xD4,0x14,0x01,0x14,0x01,0x02,0x00}));
    const uint8_t il[] = {0x01, 0x00};
    n = pn532BuildFrame(PN532_CMD_INLIST, il, 2, f, sizeof(f));
    check("InListPassiveTarget 01 00, DCS E1", bytes_eq(f, n, {0x00,0x00,0xFF,0x04,0xFC,0xD4,0x4A,0x01,0x00,0xE1,0x00}));
    const uint8_t rf[] = {0x05, 0xFF, 0x01, 0x01};
    n = pn532BuildFrame(PN532_CMD_RFCONFIG, rf, 4, f, sizeof(f));
    // D4+32+05+FF+01+01 = 0x20C -> 0x0C -> DCS 0xF4
    check("RFConfiguration MaxRetries FF 01 01, DCS F4", bytes_eq(f, n, {0x00,0x00,0xFF,0x06,0xFA,0xD4,0x32,0x05,0xFF,0x01,0x01,0xF4,0x00}));
    check("refuses a frame that does not fit", pn532BuildFrame(PN532_CMD_INLIST, il, 2, f, 8) == 0);
    // Every built frame satisfies the two checksum rules.
    n = pn532BuildFrame(PN532_CMD_RFCONFIG, rf, 4, f, sizeof(f));
    uint8_t s = 0; for (size_t i = 5; i < n - 1; i++) s = (uint8_t)(s + f[i]);
    check("LEN + LCS == 0", (uint8_t)(f[3] + f[4]) == 0);
    check("TFI + data + DCS == 0", s == 0);
  }

  printf("ACK and response parse (§6.2.1.3, §6.2.1.5, §7.2.2, §7.3.5)\n");
  {
    const uint8_t ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    check("ACK recognised", pn532IsAck(ack, 6));
    const uint8_t nack[] = {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
    check("NACK is not an ACK", !pn532IsAck(nack, 6));
    check("short ACK refused", !pn532IsAck(ack, 5));

    const uint8_t fw[] = {0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03, 0x32, 0x01, 0x06, 0x07, 0xE8, 0x00};
    const uint8_t *d = nullptr; size_t dn = 0;
    check("firmware answer parses", pn532ParseResponse(fw, sizeof(fw), PN532_CMD_GETFW, &d, &dn) == PN532_OK);
    uint8_t ic = 0, v = 0, r = 0;
    check("IC 0x32, firmware 1.6", pn532ParseFirmware(d, dn, ic, v, r) && ic == 0x32 && v == 1 && r == 6);

    uint8_t bad[sizeof(fw)]; memcpy(bad, fw, sizeof(fw)); bad[4] = 0xFB;
    check("bad LCS refused", pn532ParseResponse(bad, sizeof(bad), PN532_CMD_GETFW, &d, &dn) == PN532_BAD_LCS);
    memcpy(bad, fw, sizeof(fw)); bad[11] = 0xE9;
    check("bad DCS refused", pn532ParseResponse(bad, sizeof(bad), PN532_CMD_GETFW, &d, &dn) == PN532_BAD_DCS);
    check("answer to another command refused", pn532ParseResponse(fw, sizeof(fw), PN532_CMD_INLIST, &d, &dn) == PN532_WRONG_CMD);
    memcpy(bad, fw, sizeof(fw)); bad[5] = 0xD4;
    // D4 instead of D5 also breaks DCS; fix DCS so TFI is the only fault.
    { uint8_t s = 0; for (int i = 5; i < 11; i++) s = (uint8_t)(s + bad[i]); bad[11] = (uint8_t)(0x100 - s); }
    check("TFI D4 (host's) refused", pn532ParseResponse(bad, sizeof(bad), PN532_CMD_GETFW, &d, &dn) == PN532_BAD_TFI);
    const uint8_t err[] = {0x00, 0x00, 0xFF, 0x01, 0xFF, 0x7F, 0x81, 0x00};
    check("error frame recognised", pn532ParseResponse(err, sizeof(err), PN532_CMD_GETFW, &d, &dn) == PN532_ERROR_FRAME);
    check("ACK where a response was due", pn532ParseResponse(ack, 6, PN532_CMD_GETFW, &d, &dn) == PN532_ACK_FRAME);
    check("truncated frame refused", pn532ParseResponse(fw, 8, PN532_CMD_GETFW, &d, &dn) == PN532_SHORT);
    const uint8_t extra[] = {0x00, 0x00, 0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03, 0x32, 0x01, 0x06, 0x07, 0xE8, 0x00};
    check("extra leading preamble 00s accepted", pn532ParseResponse(extra, sizeof(extra), PN532_CMD_GETFW, &d, &dn) == PN532_OK);

    const uint8_t il[] = {0x01, 0x01, 0x00, 0x04, 0x08, 0x04, 0x64, 0x72, 0x07, 0xCA};
    uint8_t uid[PN532_UID_MAX] = {0};
    check("InList: 4-byte UID", pn532ParseInList(il, sizeof(il), uid) == 4 && uid[0] == 0x64 && uid[3] == 0xCA);
    const uint8_t none[] = {0x00};
    check("InList: NbTg 0 is no target", pn532ParseInList(none, 1, uid) == 0);
    const uint8_t longid[] = {0x01, 0x01, 0x00, 0x44, 0x00, 0x0B, 1,2,3,4,5,6,7,8,9,10,11};
    check("InList: 11-byte UID refused", pn532ParseInList(longid, sizeof(longid), uid) == -1);
    const uint8_t cut[] = {0x01, 0x01, 0x00, 0x04, 0x08, 0x07, 0x04, 0x11};
    check("InList: UID cut short refused", pn532ParseInList(cut, sizeof(cut), uid) == -1);
  }

  printf("UID to the wire form (greybeard-plc4-pwa api/commands.py:306-313)\n");
  {
    char hex[TAG_HEX_LEN];
    const uint8_t u4[] = {0x64, 0x72, 0x07, 0xCA};
    check("4 bytes convert", uidToHex(u4, 4, hex, sizeof(hex)));
    check_str("spike UID 64:72:07:CA", hex, "647207CA");
    const uint8_t lo[] = {0x0a, 0xbc, 0xde, 0xf0};
    uidToHex(lo, 4, hex, sizeof(hex));
    check_str("upper-case, leading zero kept", hex, "0ABCDEF0");
    const uint8_t u7[] = {0x04, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};
    uidToHex(u7, 7, hex, sizeof(hex));
    check_str("7 bytes, no separator", hex, "04A1B2C3D4E5F6");
    const uint8_t u10[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xAB};
    uidToHex(u10, 10, hex, sizeof(hex));
    check_str("10 bytes = 20 digits", hex, "010203040506070809AB");
    check("10-byte form is 20 characters", strlen(hex) == 20);
    check("0 bytes refused", !uidToHex(u4, 0, hex, sizeof(hex)) && hex[0] == '\0');
    uint8_t u11[11] = {0};
    check("11 bytes refused", !uidToHex(u11, 11, hex, sizeof(hex)));
    check("too small a buffer refused", !uidToHex(u4, 4, hex, 8));

    check("valid: 647207CA", tagHexValid("647207CA"));
    check("valid: 2 digits", tagHexValid("AB"));
    check("valid: 20 digits", tagHexValid("0102030405060708090A"));
    check("refused: lower case", !tagHexValid("647207ca"));
    check("refused: odd count", !tagHexValid("647207C"));
    check("refused: 22 digits", !tagHexValid("0102030405060708090A0B"));
    check("refused: empty", !tagHexValid(""));
    check("refused: separator", !tagHexValid("64:72:07:CA"));
    check("refused: null", !tagHexValid(nullptr));
  }

  printf("screen forms (design §4.4)\n");
  {
    char row[21];
    tagRowText("647207CA", row, sizeof(row));
    check_str("4-byte: TAG and colon form", row, "TAG 64:72:07:CA");
    tagRowText("04A1B2C3D4E5F6", row, sizeof(row));
    check_str("7-byte: colon form alone, 20 columns", row, "04:A1:B2:C3:D4:E5:F6");
    tagRowText("0102030405060708090A", row, sizeof(row));
    check_str("10-byte: bare wire form", row, "0102030405060708090A");
    tagRowText("bad", row, sizeof(row));
    check_str("invalid tag: empty row", row, "");
  }

  printf("scan rules (kGoneMs 2000, design §4.4)\n");
  {
    TagTracker t;
    check("first read is a new scan", t.seen("647207CA", 0));
    check("tag left on the reader: not new", !t.seen("647207CA", 100));
    t.empty(200);
    t.empty(1500);
    check("lifted 1.3 s: not confirmed gone, not new", !t.seen("647207CA", 1600));
    t.empty(1700); t.empty(2500); t.empty(3800);
    check("answered empty polls for 2.1 s: new scan", t.seen("647207CA", 3900));
    check("a different tag is new at once", t.seen("04A1B2C3D4E5F6", 4000));
    TagTracker d;
    d.seen("647207CA", 0);
    // The reader stops answering for 5 s: no empty() calls. The tag is still on it.
    check("a reader dropout confirms nothing", !d.seen("647207CA", 5000));
  }

  printf("driver against a scripted PN532\n");
  {
    FakePn532 bus;
    bus.uid = {0x64, 0x72, 0x07, 0xCA};
    Pn532Driver<FakePn532> drv(bus);
    uint32_t now = 0;
    char hex[TAG_HEX_LEN] = "";
    bool got = false;
    int maxPerStep = 0;
    for (int i = 0; i < 400 && !got; i++, now += 5) {
      const int before = bus.transactions;
      drv.step(now);
      const int made = bus.transactions - before;
      if (made > maxPerStep) maxPerStep = made;
      got = drv.takeUid(hex, sizeof(hex));
    }
    check("reads the tag", got);
    check_str("UID via the driver", hex, "647207CA");
    check("health OK", drv.health() == NFC_OK);
    check("firmware 1.6", drv.fwVer() == 1 && drv.fwRev() == 6);
    check("never more than one bus transaction per step", maxPerStep == 1);

    bus.uid.clear();
    bool empty = false;
    for (int i = 0; i < 100 && !empty; i++, now += 5) { drv.step(now); empty = drv.takeEmpty(); }
    check("an answered poll with no tag reports empty", empty);

    bus.corruptDcs = true;
    for (int i = 0; i < 2000; i++, now += 5) drv.step(now);   // 10 s: retries are 1 s apart
    check("corrupt answers: NOT ANSWERING", drv.health() == NFC_NOT_ANSWERING);
    check("failures counted", drv.fails() >= PN532_FAILS_DOWN);
    bus.corruptDcs = false;
    bus.uid = {0x64, 0x72, 0x07, 0xCA};
    got = false;
    for (int i = 0; i < 4000 && !got; i++, now += 5) { drv.step(now); got = drv.takeUid(hex, sizeof(hex)); }
    check("recovers once answers are good again", got && drv.health() == NFC_OK);
  }
  {
    FakePn532 bus;
    bus.present = false;
    Pn532Driver<FakePn532> drv(bus);
    uint32_t now = 0;
    for (int i = 0; i < 2000; i++, now += 5) drv.step(now);   // 10 s
    check("no reader fitted: NOT ANSWERING", drv.health() == NFC_NOT_ANSWERING);
    // Backed off: with no reader, the bus is touched rarely, not every pass.
    check("no reader fitted: at most a few attempts in 10 s", bus.transactions <= 6);
    printf("        (%d bus transactions in 10 s with no reader)\n", bus.transactions);
  }

  printf("%d failure(s)\n", failures);
  return failures;
}
