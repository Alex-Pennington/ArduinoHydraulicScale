// Host replay of a candump log through include/j1939_decode.h.
//
// Built with g++ ON THE HOST, like test_loop_scaling.cpp. It feeds a real capture
// off the JD 5055E through the SAME decoder the firmware runs, and checks two
// things a compile cannot:
//
//   1. the scalings produce physically plausible numbers for THIS tractor, and
//   2. every frame the decoder uses passes the hardware acceptance filter
//      (J1939_FILTER_MASK / J1939_FILTER_IDS) -- a filter that silently drops
//      EEC1 would compile, flash and publish nothing.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_j1939_replay.cpp -o /tmp/r
//   /tmp/r ~/plc4-logs/census-2026-09-22/candump-2026-09-22_175858.log
//
// Input is `candump -L` form: "(1790099938.470204) can0 0CF00400#F0FF7D0000FFFFFF".
// An 8-hex-digit id is extended; 3 digits is standard.
//
// THIS IS NOT A COMMISSIONING TEST. It proves the decoder against a recording.
// It proves nothing about the board's transceiver, its pins, or its silicon filter.

#include "j1939_decode.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static int failures = 0;

static void check(const char *what, bool ok, const char *detail) {
  printf("  %-5s %-44s %s\n", ok ? "ok" : "FAIL", what, detail);
  if (!ok) failures++;
}

static void checkRange(const char *what, const J1939Sig &s, double lo, double hi) {
  char d[64];
  if (!s.ok) { check(what, false, "never decoded"); return; }
  snprintf(d, sizeof(d), "%.2f  (want %.1f..%.1f)", (double)s.v, lo, hi);
  check(what, s.v >= lo && s.v <= hi, d);
}

static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s candump.log\n", argv[0]); return 2; }
  FILE *f = fopen(argv[1], "r");
  if (!f) { perror(argv[1]); return 2; }

  static J1939State st;
  j1939Init(st);

  unsigned long lines = 0, passed = 0, usedDropped = 0, usedTotal = 0;
  double t0 = -1, tLast = 0;
  double rpmMax = 0;
  char line[256];

  while (fgets(line, sizeof(line), f)) {
    double ts; char iface[16], frame[64];
    if (sscanf(line, "(%lf) %15s %63s", &ts, iface, frame) != 3) continue;
    char *hash = strchr(frame, '#');
    if (!hash) continue;
    *hash = '\0';
    const size_t idLen = strlen(frame);
    const bool ext = idLen == 8;
    const uint32_t id = (uint32_t)strtoul(frame, nullptr, 16);

    uint8_t d[8]; uint8_t len = 0;
    for (const char *p = hash + 1; p[0] && p[1] && len < 8; p += 2) {
      const int hi = hexv(p[0]), lo = hexv(p[1]);
      if (hi < 0 || lo < 0) break;
      d[len++] = (uint8_t)(hi << 4 | lo);
    }

    if (t0 < 0) t0 = ts;
    tLast = ts;
    const uint32_t now = (uint32_t)((ts - t0) * 1000.0);
    lines++;

    const bool pass = ext && j1939FilterPasses(id);
    if (pass) passed++;

    const uint32_t unknownBefore = st.framesUnknown;
    j1939OnFrame(st, ext, id, d, len, now);
    j1939Tick(st, now);
    const bool used = ext && st.framesUnknown == unknownBefore;
    if (used) { usedTotal++; if (!pass) usedDropped++; }

    if (st.rpm.ok && st.rpm.v > rpmMax) rpmMax = st.rpm.v;
  }
  fclose(f);

  const double secs = tLast - t0;
  printf("%s\n", argv[1]);
  printf("  frames %lu over %.1f s (%.0f fps); decoder-used %lu; unknown %lu\n",
         lines, secs, secs > 0 ? lines / secs : 0.0, usedTotal, (unsigned long)st.framesUnknown);
  printf("  hardware filter would pass %lu (%.0f fps, %.0f%% of bus)\n",
         passed, secs > 0 ? passed / secs : 0.0, lines ? 100.0 * passed / lines : 0.0);

  char d[64];
  // Freshness. Callers read `now` AFTER snapshotting, so ts <= now; the age is
  // unsigned, and j1939Tick() forgets a signal after J1939_SIG_FORGET_MS so no
  // compare ever runs near the wrap. (705bf3c's signed age made a signal dead
  // for 24.9+ days read fresh again -- the second check below.)
  {
    J1939Sig s{}; j1939SigSet(s, 900.0f, 1000);
    check("sig 4.9 s old is fresh", j1939SigFresh(s, 5900), "5 s window");
    check("sig 5.001 s old is stale", !j1939SigFresh(s, 6001), "5 s window");
    check("sig 24.9 days old is stale", !j1939SigFresh(s, 1000u + 2147483648u), "no signed wrap");
    check("hours 85 s old fresh at 90 s", j1939SigFresh(s, 86000, J1939_HOURS_STALE_MS), "");
    static J1939State t; j1939Init(t); t.rpm = s;
    j1939Tick(t, 1000 + J1939_SIG_FORGET_MS + 1);
    check("tick forgets a signal after 10 min", !t.rpm.ok, "");
  }

  // Two BAMs at once: SA 0x00 sends FEE3 (not DM1) while SA 0x03 sends a
  // two-fault DM1. The single-session decoder abandoned the DM1 here.
  {
    static J1939State t; j1939Init(t);
    const uint8_t cmDm1[8]  = {0x20, 10, 0, 2, 0xFF, 0xCA, 0xFE, 0x00};   // 10 bytes, 2 packets, PGN 65226
    const uint8_t cmFee3[8] = {0x20, 30, 0, 5, 0xFF, 0xE3, 0xFE, 0x00};
    // lamps 0x10 (red stop on) 0xFF, then SPN 2825 FMI 7 OC 1, SPN 100 FMI 1 OC 3
    const uint8_t dt1[8] = {1, 0x10, 0xFF, 0x09, 0x0B, 0x07, 0x01, 0x64};
    const uint8_t dt2[8] = {2, 0x00, 0x01, 0x03, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t dtX[8] = {1, 1, 2, 3, 4, 5, 6, 7};
    j1939OnFrame(t, true, 0x1CECFF03, cmDm1, 8, 10);
    j1939OnFrame(t, true, 0x1CECFF00, cmFee3, 8, 11);
    j1939OnFrame(t, true, 0x1CEBFF03, dt1, 8, 60);
    j1939OnFrame(t, true, 0x1CEBFF00, dtX, 8, 61);
    j1939OnFrame(t, true, 0x1CEBFF03, dt2, 8, 110);
    const int k = j1939Dm1Slot(t, 3);
    char b[64];
    snprintf(b, sizeof(b), "n=%u rsl=%u spn=%lu/%lu", t.dm1[k].count, t.dm1[k].lampRed,
             (unsigned long)t.dm1[k].dtc[0].spn, (unsigned long)t.dm1[k].dtc[1].spn);
    check("DM1 BAM survives a concurrent BAM", t.dm1[k].count == 2 && t.dm1[k].lampRed == 1
          && t.dm1[k].dtc[0].spn == 2825 && t.dm1[k].dtc[1].spn == 100, b);
  }

  // #57: the power-up EEC1 placeholder (torque raw 0 at rpm 0) is not data, but a
  // raw 0 while the engine turns is kept -- -125 % motoring is real.
  {
    static J1939State t; j1939Init(t);
    const uint8_t first[8]    = {0xF0, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF};   // census _175648 t+0
    const uint8_t motoring[8] = {0xF0, 0xFF, 0x00, 0x20, 0x1C, 0xFF, 0xFF, 0xFF};   // 900 rpm, torque raw 0
    j1939OnFrame(t, true, 0x0CF00400, first, 8, 10);
    check("power-up torque raw 0 at 0 rpm dropped", !t.torquePct.ok && t.torqueWin.n == 0 && t.rpm.ok, "");
    j1939OnFrame(t, true, 0x0CF00400, motoring, 8, 20);
    check("torque raw 0 while turning kept (-125)", t.torquePct.ok && t.torquePct.v == -125.0f
          && t.torqueWin.n == 1, "");
  }

  // More real codes than MAX_DTC: kept at 12, and trunc says so.
  {
    static J1939State t; j1939Init(t);
    uint8_t p[2 + 4 * 14]; p[0] = 0x04; p[1] = 0xFF;
    for (int i = 0; i < 14; i++) { p[2+4*i] = (uint8_t)(10 + i); p[3+4*i] = 0; p[4+4*i] = 2; p[5+4*i] = 1; }
    j1939Dm1Parse(t, p, sizeof(p), 0x17);
    const int k = j1939Dm1Slot(t, 0x17);
    check("14 codes: 12 kept, trunc set", t.dm1[k].count == 12 && t.dm1[k].trunc, "");
  }

  snprintf(d, sizeof(d), "%lu of %lu", usedDropped, usedTotal);
  check("filter drops no frame the decoder uses", usedDropped == 0, d);

  // Final values at the END of the capture. The census logs end at idle
  // (~900 rpm) or at key-on engine-off (0 rpm), so rpm is checked by its max.
  printf("  -- values at end of capture --\n");
  checkRange("keysw_v (SPN 158)", st.keyswV, 11.0, 15.0);
  checkRange("coolant_c (SPN 110)", st.coolantC, -30.0, 110.0);
  checkRange("hours (SPN 247)", st.hours, 1.0, 20000.0);
  if (st.oilKpa.ok)  checkRange("oil_kpa (SPN 100)", st.oilKpa, 0.0, 700.0);
  if (st.oilTempC.ok) checkRange("oil_temp_c (SPN 175)", st.oilTempC, -30.0, 150.0);
  if (st.rpm.ok) {
    snprintf(d, sizeof(d), "max %.1f, last %.1f", rpmMax, (double)st.rpm.v);
    check("rpm (SPN 190) within 0..3000", rpmMax >= 0 && rpmMax <= 3000, d);
  } else {
    check("rpm (SPN 190)", false, "never decoded");
  }
  if (st.fuelLph.ok) checkRange("fuel_lph (SPN 183)", st.fuelLph, 0.0, 60.0);
  if (st.loadPct.ok) checkRange("load_pct (SPN 92)", st.loadPct, 0.0, 125.0);
  checkRange("fuel_level_pct (SPN 96, DD)", st.fuelLevelPct, 30.0, 40.0);
  checkRange("hyd_oil_c (SPN 1638, VF)", st.hydOilC, 10.0, 30.0);
  {
    const J1939Win &w = st.rpmWin;   // never taken in the replay: the whole capture
    snprintf(d, sizeof(d), "n=%lu min %.0f avg %.0f max %.0f", (unsigned long)w.n,
             (double)w.mn, (double)(w.n ? w.sum / w.n : 0), (double)w.mx);
    check("rpm window min <= avg <= max", w.n > 0 && w.mn <= w.sum / w.n && w.sum / w.n <= w.mx, d);
  }
  {
    const J1939Win &w = st.torqueWin;   // #57: no capture should put the -125 placeholder in a window
    snprintf(d, sizeof(d), "n=%lu min %.0f", (unsigned long)w.n, (double)w.mn);
    check("torque window min is not the -125 placeholder", w.n > 0 && w.mn > -125.0f, d);
  }

  printf("  -- DM1 per source address --\n");
  for (size_t i = 0; i < J1939_MAX_DM1_SRC; i++) {
    const J1939Dm1Src &s = st.dm1[i];
    if (!s.used) continue;
    printf("  sa 0x%02X  mil %u rsl %u awl %u pl %u  dtc %u", s.sa, s.lampMil, s.lampRed,
           s.lampAmber, s.lampProtect, s.count);
    for (uint8_t k = 0; k < s.count; k++)
      printf("  [spn %lu fmi %u oc %u]", (unsigned long)s.dtc[k].spn, s.dtc[k].fmi, s.dtc[k].oc);
    printf("\n");
  }

  printf(failures ? "FAILED: %d\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
