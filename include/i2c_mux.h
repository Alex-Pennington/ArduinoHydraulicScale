#pragma once
#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// TCA9548A/PCA9548A I2C multiplexer on the Qwiic trunk.
//
// EVERY I2C DEVICE ON THIS BOARD SITS BEHIND IT. Nothing answers at its own
// address until its channel is selected, so a plain scan of the trunk finds the
// mux and nothing else, and the LCD and the ADC both look absent when they are
// simply on the far side of it. That is exactly how this presented on the
// bench: `sensor = NO ADC` and a dead display, with both devices healthy.
//
// Topology, read off the board by the `i2c` verb on 2026-09-17:
//
//   trunk  0x70  this mux
//     ch0  0x67  MCP9600 thermocouple amp   -- present, unused by this firmware
//     ch1  0x2A  NAU7802 load-cell ADC      -- present, unused by this firmware
//     ch2  0x40  INA219 volts+amps          <-- this firmware, diagnostics only
//     ch3  0x68  MCP3421, the loop ADC      <-- this firmware, the measurement
//     ch7  0x27  LCD2004 display            <-- this firmware
//
// Channel assignments confirmed by Alex against the harness on 2026-09-17 and
// independently matched by the `i2c` scan, so this map is measured on both
// sides rather than assumed from either.
//
// THE MUX ITSELF IS THE SINGLE POINT OF FAILURE. It sits on the trunk, ahead of
// every channel: if 0x70 does not answer, nothing behind it can be reached and
// a scan reports an empty bus. "NOTHING FOUND" therefore means the trunk or the
// mux, never a device -- do not go looking at the ADC for it.
//
// The thermocouple and the load cell are wired but belong to no code here.
// Leave them alone rather than assuming they are ours.
// ---------------------------------------------------------------------------

constexpr uint8_t I2C_MUX_ADDR   = 0x70;
constexpr uint8_t I2C_CH_INA219  = 2;
constexpr uint8_t I2C_CH_MCP3421 = 3;
constexpr uint8_t I2C_CH_LCD2004 = 7;

// Last channel written, -1 when unknown. Selecting costs one byte on the wire —
// cheap, but not free when the LCD writes 80 characters a render, so callers
// select once per LOGICAL OPERATION (one lcdLine, one mcpRead) and this skips
// the write when the channel has not changed.
inline int &i2cMuxCurrent() { static int c = -1; return c; }

// Forget the cached channel. Anything that drives the mux directly — the bus
// scan — must call this, or the next select will be skipped as a no-op and the
// transfer will go to whatever channel the scan left behind.
inline void i2cMuxInvalidate() { i2cMuxCurrent() = -1; }

inline bool i2cMuxSelect(uint8_t ch) {
  if ((int)ch == i2cMuxCurrent()) return true;
  Wire1.beginTransmission(I2C_MUX_ADDR);
  Wire1.write((uint8_t)(1 << ch));
  if (Wire1.endTransmission() != 0) {
    i2cMuxInvalidate();     // a failed select must never look cached
    return false;
  }
  i2cMuxCurrent() = (int)ch;
  return true;
}
