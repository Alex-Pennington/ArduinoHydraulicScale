// Host check of the lid buttons' debounce, press queue, auto-repeat and KEY STUCK detection in
// include/hmi_buttons.h (FW-2, design §3.5). Samples are fed at 200 Hz, 5 ms apart, as the timer
// ISR feeds them.
//
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_hmi_buttons.cpp -o "$TMPDIR/t_btn"
//   "$TMPDIR/t_btn"

#include "hmi_buttons.h"
#include <cstdio>

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  %s  %s\n", ok ? "ok   " : "FAIL ", what);
  if (!ok) failures++;
}

static uint32_t now = 0;
static void feed(ButtonPad &p, uint8_t mask, int samples) {
  for (int i = 0; i < samples; i++) { p.sample(mask, now); now += 5; }
}
static int drain(ButtonPad &p, int &repeats, int id = -1) {
  BtnEvent e; int n = 0; repeats = 0;
  while (p.pop(e)) { if (id < 0 || e.id == id) { n++; if (e.repeat) repeats++; } }
  return n;
}

int main() {
  const uint8_t OK = 1 << BTN_OK, UP = 1 << BTN_UP, BACK = 1 << BTN_BACK, DOWN = 1 << BTN_DOWN;
  int rep = 0;

  printf("debounce: 4 equal samples (20 ms)\n");
  {
    ButtonPad p;
    // Contact bounce: pressed, released, pressed, released ... never 4 in a row.
    for (int i = 0; i < 10; i++) { feed(p, OK, 2); feed(p, 0, 1); }
    check("bounce alone makes no press", drain(p, rep) == 0);
    feed(p, OK, 3);
    check("3 stable samples: still no press", drain(p, rep) == 0);
    feed(p, OK, 1);
    check("4th stable sample: one press", drain(p, rep) == 1);
    feed(p, OK, 50);
    check("held OK makes no more events (OK never repeats)", drain(p, rep) == 0);
    feed(p, 0, 3);
    feed(p, OK, 4);
    check("a release under 20 ms is not a release", drain(p, rep) == 0);
    feed(p, 0, 4);
    feed(p, OK, 4);
    check("release then press: a second press", drain(p, rep) == 1);
    feed(p, 0, 4);
  }

  printf("queue: 4 deep, newest dropped\n");
  {
    ButtonPad p;
    for (int i = 0; i < 6; i++) { feed(p, BACK, 4); feed(p, 0, 4); }
    BtnEvent e; int n = 0;
    while (p.pop(e)) n++;
    check("six presses queue four", n == 4);
    check("two dropped and counted", p.dropped == 2);
  }

  printf("auto-repeat: UP and DOWN, 500 ms then every 150 ms\n");
  {
    ButtonPad p;
    feed(p, UP, 4);                 // pressed at 15 ms
    int n = drain(p, rep, BTN_UP);
    check("press", n == 1 && rep == 0);
    feed(p, UP, 96);                // to ~495 ms after the press: no repeat yet
    check("no repeat before 500 ms", drain(p, rep, BTN_UP) == 0);
    feed(p, UP, 4);                 // to 500 ms after the press, which registered at 15 ms
    n = drain(p, rep, BTN_UP);
    check("first repeat at 500 ms", n == 1 && rep == 1);
    feed(p, UP, 30);                // 150 ms more
    n = drain(p, rep, BTN_UP);
    check("then one every 150 ms", n == 1 && rep == 1);
    feed(p, UP, 60);                // 300 ms more
    n = drain(p, rep, BTN_UP);
    check("two in 300 ms", n == 2 && rep == 2);
    feed(p, 0, 4);
    drain(p, rep);
    feed(p, DOWN, 4);
    feed(p, DOWN, 100);
    n = drain(p, rep, BTN_DOWN);
    check("DOWN repeats too", n == 2 && rep == 1);
    feed(p, 0, 4);
    drain(p, rep);
    feed(p, BACK, 400);
    check("BACK never repeats", drain(p, rep, BTN_BACK) == 1 && rep == 0);
    feed(p, 0, 4);
  }

  printf("KEY STUCK: held 10 s or more\n");
  {
    ButtonPad p;
    feed(p, DOWN, 4);
    feed(p, DOWN, 1900);            // 9.5 s
    check("9.5 s held: not stuck", !p.keyStuck());
    drain(p, rep);
    feed(p, DOWN, 120);             // past 10 s
    check("10 s held: KEY STUCK", p.keyStuck());
    drain(p, rep);
    feed(p, DOWN, 400);
    check("a stuck key makes no events", drain(p, rep) == 0);
    check("a stuck key is not down", !p.isDown(BTN_DOWN));
    feed(p, OK | DOWN, 4);
    check("another button still works", drain(p, rep, BTN_OK) == 1);
    feed(p, 0, 4);
    check("release clears KEY STUCK", !p.keyStuck());
    feed(p, DOWN, 4);
    check("and the key works again", drain(p, rep, BTN_DOWN) == 1);
  }

  printf("%d failure(s)\n", failures);
  return failures;
}
