// Host check of the board's command-id minting across simulated reboots, include/cmd_id.h
// (FW-5, design §5.3). One "EEPROM" image outlives several CmdIdMinter lifetimes, as data flash
// outlives a reset.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_cmd_id.cpp -o "$TMPDIR/t_id"
//   "$TMPDIR/t_id"

#include "cmd_id.h"
#include <cstdio>
#include <cstring>
#include <set>
#include <string>

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  %s  %s\n", ok ? "ok   " : "FAIL ", what);
  if (!ok) failures++;
}

// The data flash: erased (0xFF) until the first write.
static CmdIdStore flash;
static bool flashErased = true;
static int flashWrites = 0;
static bool loadFn(CmdIdStore &out) {
  if (flashErased) { memset(&out, 0xFF, sizeof(out)); return true; }
  out = flash;
  return true;
}
static void saveFn(const CmdIdStore &in) { flash = in; flashErased = false; flashWrites++; }

static CmdIdMinter boot() {
  CmdIdMinter m;
  m.load = loadFn;
  m.save = saveFn;
  return m;
}

int main() {
  char id[32];
  std::set<std::string> seen;
  bool unique = true;

  printf("first boot on erased flash\n");
  {
    CmdIdMinter m = boot();
    check("no write at boot, before any button", flashWrites == 0);
    m.mint(id, sizeof(id));
    printf("        %s\n", id);
    check("first id is p4-00000001-1", strcmp(id, "p4-00000001-1") == 0);
    check("one write, at the first command", flashWrites == 1);
    unique &= seen.insert(id).second;
    m.mint(id, sizeof(id));
    check("second id is p4-00000001-2", strcmp(id, "p4-00000001-2") == 0);
    check("still one write", flashWrites == 1);
    unique &= seen.insert(id).second;
  }

  printf("reboot loop with no button pressed\n");
  {
    for (int i = 0; i < 50; i++) { CmdIdMinter m = boot(); (void)m; }
    check("fifty boots, no writes", flashWrites == 1);
  }

  printf("later boots\n");
  {
    for (int b = 0; b < 5; b++) {
      CmdIdMinter m = boot();
      for (int k = 0; k < 20; k++) { m.mint(id, sizeof(id)); unique &= seen.insert(id).second; }
    }
    check("one write per boot that minted", flashWrites == 6);
    check("no id repeats across six boots", unique && seen.size() == 102);
    check("session after six boots is 6", flash.session == 6);
    CmdIdMinter m = boot();
    m.mint(id, sizeof(id));
    check("the next boot's id", strcmp(id, "p4-00000007-1") == 0);
  }

  printf("form and limits\n");
  {
    CmdIdMinter m = boot();
    m.session = 0xFFFFFFFF; m.started = true; m.n = 4294967294UL;
    m.mint(id, sizeof(id));
    printf("        %s (%zu chars)\n", id, strlen(id));
    check("longest id fits the panel's 40 (api/commands.py:138)", strlen(id) <= 22);
    check("refuses a buffer too small", !m.mint(id, 8));
    check("p4- prefix recognised", cmdIdIsBoards("p4-0000002a-7"));
    check("pendant id is not the board's", !cmdIdIsBoards("hh-1-2-3"));
    check("phone id is not the board's", !cmdIdIsBoards("7f3a9c21"));
  }

  printf("a slot holding someone else's bytes\n");
  {
    flash.sig = 0x12345678; flash.session = 999;
    CmdIdMinter m = boot();
    m.mint(id, sizeof(id));
    check("bad signature restarts at session 1", strcmp(id, "p4-00000001-1") == 0);
  }

  printf("%d failure(s)\n", failures);
  return failures;
}
