#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "loop_scaling.h"

// ---------------------------------------------------------------------------
// MCP3421 18-bit delta-sigma ADC, used here at 14-bit, on Wire1 (Qwiic).
//
// TWO CONSTANTS HERE ARE INVERTED IN LogSplitter_Monitor's version of this
// driver. Verified against Microchip DS22003E on 2026-09-16:
//   RDY when READING:  0 = latest result IS ready; 1 = not updated since the
//                      last read. That sibling header says the opposite.
//   O/C bit 4:         1 = Continuous (default); 0 = One-Shot. That sibling
//                      enum says the opposite.
// A driver carrying those runs one-shot believing it is continuous, and accepts
// only stale samples. Do not "restore" them.
//
// Config byte 0x14 = O/C 1 (continuous) | S1-S0 01 (60 SPS, 14-bit) | G1-G0 00 (x1).
// ---------------------------------------------------------------------------
constexpr uint8_t MCP3421_ADDR    = 0x68;
constexpr uint8_t MCP3421_CONFIG  = 0x14;
constexpr uint8_t MCP3421_RDY_BIT = 0x80;

inline uint32_t &mcpFailCountRef() { static uint32_t n = 0; return n; }
inline uint32_t  mcpFailCount()    { return mcpFailCountRef(); }

inline bool mcpBegin() {
  Wire1.begin();
  Wire1.beginTransmission(MCP3421_ADDR);
  Wire1.write(MCP3421_CONFIG);
  return Wire1.endTransmission() == 0;
}

// Returns TRUE only for a FRESH conversion. A bus failure or a not-ready sample
// returns false and leaves countsOut untouched.
//
// A FAILED READ IS NOT ZERO COUNTS. Zero counts is LOOP_DEAD -- an affirmative
// statement that the loop is open. Conflating them would publish a confident
// "no sensor" from a loose Qwiic connector, which is the plausible-wrong-answer
// failure this machine's whole history is about.
inline bool mcpRead(int &countsOut) {
  if (Wire1.requestFrom((uint8_t)MCP3421_ADDR, (uint8_t)3) != 3) {
    mcpFailCountRef()++;
    return false;
  }
  uint8_t hi  = Wire1.read();
  uint8_t lo  = Wire1.read();
  uint8_t cfg = Wire1.read();

  if (cfg & MCP3421_RDY_BIT) return false;   // RDY=1 -> stale, not a failure

  int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);
  raw = (int16_t)(raw << 2) >> 2;            // sign-extend the 14-bit result
  countsOut = (int)raw;
  return true;
}
