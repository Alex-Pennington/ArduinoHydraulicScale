#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "i2c_mux.h"

// LCD2004A, 20x4, PCF8574 I2C backpack at 0x27, on Wire1 (Qwiic).
// Direct I2C rather than LiquidCrystal_I2C, matching LogSplitter_Monitor, which
// runs this display on this board.
//
// BACKPACK LEVELS ARE NOT VERIFIED HERE. These modules are typically 5 V parts
// and Qwiic is a 3.3 V bus. I2C is open-drain so it generally works and the
// sibling runs exactly this combination -- but that is proven on another
// machine, not measured on this one.
constexpr uint8_t LCD_ADDR = 0x27;
constexpr uint8_t LCD_COLS = 20;

inline void lcdPulse(uint8_t v) {
  Wire1.beginTransmission(LCD_ADDR); Wire1.write(v | 0x04); Wire1.endTransmission();
  delayMicroseconds(1);
  Wire1.beginTransmission(LCD_ADDR); Wire1.write(v & ~0x04); Wire1.endTransmission();
  delayMicroseconds(50);
}

inline void lcdSend(uint8_t value, uint8_t mode) {
  uint8_t backlight = 0x08;
  lcdPulse((value & 0xF0) | mode | backlight);
  lcdPulse(((value << 4) & 0xF0) | mode | backlight);
}

inline void lcdCmd(uint8_t c)  { lcdSend(c, 0x00); }
inline void lcdData(uint8_t c) { lcdSend(c, 0x01); }

// What each of the four rows is currently displaying, padded exactly as
// lcdLine() writes it. lcdLine() compares against this and skips the I2C
// traffic when a row has not changed -- see the note there.
//
// lcdInvalidate() fills it with a sentinel no rendered row can contain:
// lcdLine() pads with spaces and copies printable text, so 0xFF never appears
// and the first write to each row always differs. That removes any need for a
// "have we painted yet" flag, and with it any assumption about the ORDER the
// rows are written in -- which is the kind of assumption a screen added later
// quietly breaks.
//
// Call lcdInvalidate() after anything that changes the panel behind
// lcdLine()'s back. The init sequence clears the display, and the cache would
// otherwise claim rows hold text that was just wiped.
inline char lcdShadow[4][LCD_COLS + 1];

inline void lcdInvalidate() { memset(lcdShadow, 0xFF, sizeof(lcdShadow)); }

inline void lcdBegin() {
  Wire1.begin();
  // The display sits behind the mux; without its channel selected this entire
  // init sequence goes nowhere and the panel stays blank. See include/i2c_mux.h.
  i2cMuxSelect(I2C_CH_LCD2004);
  delay(50);
  lcdPulse(0x30); delay(5);
  lcdPulse(0x30); delayMicroseconds(150);
  lcdPulse(0x30); delayMicroseconds(150);
  lcdPulse(0x20); delayMicroseconds(150);   // 4-bit mode
  lcdCmd(0x28);                             // 4-bit, 2-line mode, 5x8
  lcdCmd(0x0C);                             // display on, cursor off
  lcdCmd(0x01); delay(2);                   // clear
  lcdCmd(0x06);                             // entry mode, increment
  // The clear above wiped the panel, so nothing the cache remembers is on it.
  lcdInvalidate();
}

// Row start addresses for a 20x4: rows 2 and 3 continue rows 0 and 1.
inline void lcdLine(uint8_t row, const char *text) {
  static const uint8_t base[4] = { 0x00, 0x40, 0x14, 0x54 };
  if (row > 3) return;

  // SKIP THE WRITE WHEN THE ROW ALREADY SAYS THIS. Every caller renders
  // unconditionally on every loop pass, so without this the panel is rewritten
  // 80 characters at a time forever, and each character is two I2C pulses on
  // the same bus the ADC is read over. The sample rate paid for it.
  //
  // THE CHECK LIVES HERE, NOT AT THE CALL SITES, AND THAT IS THE POINT. The
  // defect this replaces was a `lcdDirty` flag that every caller was supposed
  // to set and none did, so it was written and never read. A cache inside the
  // one function that touches the panel cannot be forgotten by a screen added
  // later, and needs no discipline from anybody.
  //
  // Compared against the PADDED form, because that is what actually gets
  // written -- "ok" and "ok  " land on the panel identically and must compare
  // equal, or a row that alternates between them repaints forever.
  char padded[LCD_COLS + 1];
  uint8_t p = 0;
  for (; p < LCD_COLS && text[p]; p++) padded[p] = text[p];
  for (; p < LCD_COLS; p++)            padded[p] = ' ';
  padded[LCD_COLS] = '\0';

  if (!memcmp(lcdShadow[row], padded, LCD_COLS)) return;

  // Select once per LINE, not per character: a render is 80 characters and the
  // select is skipped anyway when the channel has not changed. The sampler
  // flips the mux to the ADC between renders, so this cannot be hoisted out.
  i2cMuxSelect(I2C_CH_LCD2004);
  lcdCmd(0x80 | base[row]);
  // `padded` is already stopped at the null terminator and space-filled to
  // LCD_COLS, so this writes exactly the 20 characters the panel holds.
  for (uint8_t i = 0; i < LCD_COLS; i++) lcdData(padded[i]);

  // Recorded only after the write succeeded in reaching this point, so an
  // early return above can never leave the cache claiming a row was painted.
  memcpy(lcdShadow[row], padded, LCD_COLS + 1);
}
