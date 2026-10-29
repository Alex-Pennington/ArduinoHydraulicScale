#pragma once
// FW-2: the four lid buttons, BACK UP DOWN OK on D2-D5, design §3.4-§3.5.
//
// Pure logic, no Arduino. The firmware samples the pins in a 200 Hz FspTimer ISR and calls
// sample(); loop() drains the event queue with interrupts off around pop(). On a board with no
// free timer it calls sample() from loop() instead, which loses presses made during a network
// stall (design §3.5) -- the console says which it got.
//
// Rules, design §3.5:
//   * Integrate: a button changes state after BTN_STABLE_SAMPLES equal samples in a row
//     (4 at 200 Hz is 20 ms; a common bounce allowance, not a datasheet figure).
//   * A release-to-press edge queues one press event, up to BTN_QUEUE_DEPTH deep. A full queue
//     drops the newcomer, never an older press.
//   * UP and DOWN auto-repeat while held: first repeat after 500 ms, then every 150 ms. BACK and
//     OK never repeat.
//   * KEY STUCK: a button held for 10 s or more is ignored until released, and keyStuck() is true
//     for as long as it stays down. The fault row shows it (hmiRenderFaultRow()).

#include <stddef.h>
#include <stdint.h>

enum BtnId : uint8_t { BTN_BACK = 0, BTN_UP = 1, BTN_DOWN = 2, BTN_OK = 3, BTN_COUNT = 4 };

static const uint8_t  BTN_STABLE_SAMPLES = 4;
static const uint8_t  BTN_QUEUE_DEPTH    = 4;
static const uint32_t BTN_REPEAT_FIRST_MS = 500;
static const uint32_t BTN_REPEAT_EVERY_MS = 150;
static const uint32_t BTN_STUCK_MS        = 10000;

struct BtnEvent {
  uint8_t id;       // BtnId
  bool    repeat;   // an auto-repeat of a held UP or DOWN, not a fresh press
};

struct ButtonPad {
  // `down` bit i is button i read as pressed (the pin low, through the NO contact to GND).
  void sample(uint8_t down, uint32_t nowMs) {
    for (uint8_t i = 0; i < BTN_COUNT; i++) {
      const bool raw = (down >> i) & 1;
      Btn &b = btn[i];
      if (raw == b.stable) {
        b.run = 0;
      } else if (++b.run >= BTN_STABLE_SAMPLES) {
        b.run = 0;
        b.stable = raw;
        if (raw) {
          b.downAt = nowMs;
          b.nextRepeat = nowMs + BTN_REPEAT_FIRST_MS;
          b.stuck = false;
          push(i, false);
        } else {
          b.stuck = false;
        }
      }
      if (b.stable && !b.stuck) {
        if ((uint32_t)(nowMs - b.downAt) >= BTN_STUCK_MS) {
          b.stuck = true;
        } else if ((i == BTN_UP || i == BTN_DOWN) && (int32_t)(nowMs - b.nextRepeat) >= 0) {
          b.nextRepeat += BTN_REPEAT_EVERY_MS;
          push(i, true);
        }
      }
    }
  }

  bool pop(BtnEvent &e) {
    if (count == 0) return false;
    e = q[head];
    head = (uint8_t)((head + 1) % BTN_QUEUE_DEPTH);
    count--;
    return true;
  }

  bool keyStuck() const {
    for (uint8_t i = 0; i < BTN_COUNT; i++) if (btn[i].stuck) return true;
    return false;
  }
  bool isDown(uint8_t i) const { return i < BTN_COUNT && btn[i].stable && !btn[i].stuck; }

  uint32_t dropped = 0;   // presses lost to a full queue

 private:
  struct Btn {
    bool stable = false;   // debounced state, true = pressed
    bool stuck = false;
    uint8_t run = 0;       // consecutive samples disagreeing with `stable`
    uint32_t downAt = 0;
    uint32_t nextRepeat = 0;
  };
  void push(uint8_t id, bool repeat) {
    if (count >= BTN_QUEUE_DEPTH) { dropped++; return; }
    q[(head + count) % BTN_QUEUE_DEPTH] = BtnEvent{id, repeat};
    count++;
  }
  Btn btn[BTN_COUNT];
  BtnEvent q[BTN_QUEUE_DEPTH];
  uint8_t head = 0, count = 0;
};
