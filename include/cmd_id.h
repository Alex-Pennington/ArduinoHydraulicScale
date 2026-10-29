#pragma once
// FW-5: command ids the board mints for its own panel requests, design §5.3.
//
//   p4-<session, 8 hex>-<n>        e.g. p4-0000002a-7
//
// * `p4-` marks the origin in plc4_cmd. The pendant uses hh-<node>-<boot>-<n>.
// * <session> is a counter in EEPROM, incremented ONCE PER BOOT, AT THE FIRST BOARD-ORIGINATED
//   COMMAND after that boot, never at boot itself. A board in a reset loop therefore writes
//   nothing -- the objection that kept a boot counter out of this firmware
//   (src/ArduinoHydraulicScale.cpp, the note above publishStatus()). Wear is one write per boot
//   in which a button was used.
// * <n> counts from 1 within the session, in RAM.
//
// Why it must not repeat across reboots (design §4.2): the panel pins the tag to the id and
// answers 409 to the same id with a different tag (simple_tally.py _check_tag()); the board's
// dedup ring replays a repeated id instead of running it; the consumer joins on cmd_id.
//
// At most 3 + 8 + 1 + 10 = 22 characters, inside the panel's 40-character id rule
// (api/commands.py:138) and the board's own char id[41].
//
// Pure logic. The storage is two callbacks so the host test can simulate reboots over one
// "EEPROM" image.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 'PID1'. A slot that does not carry it is read as "no session yet", so a board flashed over
// whatever this EEPROM range held before starts at session 1 rather than at garbage.
static const uint32_t CMD_ID_SIGNATURE = 0x50494431;

struct CmdIdStore {
  uint32_t sig;
  uint32_t session;
};

typedef bool (*CmdIdLoadFn)(CmdIdStore &out);
typedef void (*CmdIdSaveFn)(const CmdIdStore &in);

struct CmdIdMinter {
  CmdIdLoadFn load = nullptr;
  CmdIdSaveFn save = nullptr;
  bool     started = false;   // this boot has taken its session
  uint32_t session = 0;
  uint32_t n = 0;
  uint32_t writes = 0;        // EEPROM writes this boot: 0 or 1

  // Mint the next id into `out` (at least 23 bytes). The first call after a boot takes the
  // session: stored + 1, or 1 if none is stored, written back in ONE put of the whole record.
  bool mint(char *out, size_t cap) {
    if (!started) {
      CmdIdStore s = {0, 0};
      const bool have = load && load(s) && s.sig == CMD_ID_SIGNATURE;
      session = have ? s.session + 1 : 1;
      if (session == 0) session = 1;   // a wrap past 0xFFFFFFFF never yields session 0
      CmdIdStore w = {CMD_ID_SIGNATURE, session};
      if (save) { save(w); writes++; }
      started = true;
      n = 0;
    }
    n++;
    const int k = snprintf(out, cap, "p4-%08lx-%lu", (unsigned long)session, (unsigned long)n);
    return k > 0 && (size_t)k < cap;
  }
};

// Whether an id is one this board mints. The ack block uses it to hand a verdict to the screen.
inline bool cmdIdIsBoards(const char *id) { return id && strncmp(id, "p4-", 3) == 0; }
