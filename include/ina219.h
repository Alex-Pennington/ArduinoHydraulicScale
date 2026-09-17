#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "i2c_mux.h"

// ---------------------------------------------------------------------------
// INA219 bus-voltage / shunt-current monitor, 0x40 on mux channel 2.
//
// Present on the LogSplitter_Monitor hardware this board reuses. What it is
// WIRED ACROSS is a fact about the harness, not about this firmware, and is
// not knowable from software -- so nothing here names it. The readings say
// what they are: a bus voltage and a shunt voltage. Read them on the bench and
// the numbers identify the rail themselves.
//
// CALIBRATION IS DELIBERATELY NOT USED. The INA219's current and power
// registers require its calibration register to be programmed with a value
// derived from the shunt resistance, and a wrong calibration yields confident
// wrong amps -- the exact failure mode this machine's whole history is about.
// Instead this reads the SHUNT VOLTAGE, which is calibration-free and depends
// on nothing but the part, and divides by INA219_SHUNT_OHMS in one place where
// the assumption is visible.
//
// INA219_SHUNT_OHMS IS AN ASSUMPTION, NOT A MEASUREMENT. 0.1 ohm is what the
// common Adafruit and generic breakouts fit, but a board built for high
// current may use 0.01. If the amps look out by 10x, this is why. The VOLTAGES
// above are unaffected either way.
// ---------------------------------------------------------------------------

constexpr uint8_t INA219_ADDR      = 0x40;
constexpr uint8_t INA219_REG_CONF  = 0x00;
constexpr uint8_t INA219_REG_SHUNT = 0x01;
constexpr uint8_t INA219_REG_BUS   = 0x02;

// Power-on default: 32 V range, +/-320 mV shunt, 12-bit, continuous shunt+bus.
constexpr uint16_t INA219_CONFIG   = 0x399F;

constexpr float INA219_SHUNT_OHMS  = 0.1f;   // ASSUMED -- see the note above
constexpr float INA219_BUS_LSB_V   = 0.004f; // 4 mV per bit, fixed by the part
constexpr float INA219_SHUNT_LSB_MV = 0.01f; // 10 uV per bit, fixed by the part

inline uint32_t &inaFailCountRef() { static uint32_t n = 0; return n; }
inline uint32_t  inaFailCount()    { return inaFailCountRef(); }

// Set once at boot. False means the part did not acknowledge: absent, on a
// different channel, or strapped to another address (0x40-0x4F).
inline bool inaPresentRef(bool set = false, bool v = false) {
  static bool present = false;
  if (set) present = v;
  return present;
}
inline bool inaPresent() { return inaPresentRef(); }

inline bool inaWrite16(uint8_t reg, uint16_t v) {
  Wire1.beginTransmission(INA219_ADDR);
  Wire1.write(reg);
  Wire1.write((uint8_t)(v >> 8));
  Wire1.write((uint8_t)(v & 0xFF));
  return Wire1.endTransmission() == 0;
}

inline bool inaRead16(uint8_t reg, uint16_t &out) {
  Wire1.beginTransmission(INA219_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission() != 0) return false;
  if (Wire1.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire1.read();
  uint8_t lo = Wire1.read();
  out = (uint16_t)((hi << 8) | lo);
  return true;
}

inline bool inaBegin() {
  Wire1.begin();
  if (!i2cMuxSelect(I2C_CH_INA219)) { inaPresentRef(true, false); return false; }
  bool ok = inaWrite16(INA219_REG_CONF, INA219_CONFIG);
  inaPresentRef(true, ok);
  return ok;
}

// Bus volts and shunt millivolts. Returns false and touches neither output on
// any bus failure -- a stale reading must never be presented as a live one.
inline bool inaRead(float &busVolts, float &shuntMv) {
  if (!inaPresent()) return false;
  if (!i2cMuxSelect(I2C_CH_INA219)) { inaFailCountRef()++; return false; }

  uint16_t rawBus = 0, rawShunt = 0;
  if (!inaRead16(INA219_REG_BUS, rawBus))     { inaFailCountRef()++; return false; }
  if (!inaRead16(INA219_REG_SHUNT, rawShunt)) { inaFailCountRef()++; return false; }

  // Bus register: bits 15-3 carry the value, bit 1 is CNVR and bit 0 OVF.
  // OVF means the power/current math overflowed; the bus reading itself is
  // still valid, and this driver does not use the current register anyway.
  busVolts = (float)(rawBus >> 3) * INA219_BUS_LSB_V;

  // Shunt register is SIGNED: current can flow either way through the shunt,
  // and reading it unsigned turns a small reverse current into ~655 V.
  shuntMv = (float)((int16_t)rawShunt) * INA219_SHUNT_LSB_MV;
  return true;
}

// Amps across the shunt. Carries the INA219_SHUNT_OHMS assumption; the caller
// should present it as derived, not measured.
inline float inaShuntAmps(float shuntMv) {
  return (shuntMv / 1000.0f) / INA219_SHUNT_OHMS;
}
