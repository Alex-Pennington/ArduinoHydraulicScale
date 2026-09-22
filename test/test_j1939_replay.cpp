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
