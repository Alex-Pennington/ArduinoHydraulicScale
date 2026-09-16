#pragma once
#include <Arduino.h>
#include <Wire.h>

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

inline void lcdBegin() {
  Wire1.begin();
  delay(50);
  lcdPulse(0x30); delay(5);
  lcdPulse(0x30); delayMicroseconds(150);
  lcdPulse(0x30); delayMicroseconds(150);
  lcdPulse(0x20); delayMicroseconds(150);   // 4-bit mode
  lcdCmd(0x28);                             // 4-bit, 2-line mode, 5x8
  lcdCmd(0x0C);                             // display on, cursor off
  lcdCmd(0x01); delay(2);                   // clear
  lcdCmd(0x06);                             // entry mode, increment
}

// Row start addresses for a 20x4: rows 2 and 3 continue rows 0 and 1.
inline void lcdLine(uint8_t row, const char *text) {
  static const uint8_t base[4] = { 0x00, 0x40, 0x14, 0x54 };
  if (row > 3) return;
  lcdCmd(0x80 | base[row]);
  for (uint8_t i = 0; i < LCD_COLS; i++) lcdData(text[i] ? text[i] : ' ');
}
