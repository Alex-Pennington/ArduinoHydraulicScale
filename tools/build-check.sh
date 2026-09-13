#!/usr/bin/env bash
# Compile-check PLC #4 against the Saga arduino-cli toolchain.
#
# THIS DOES NOT FLASH. It never opens a port and never takes a --upload flag.
# Flashing is Alex's, at the machine.
#
# Why this script exists: the repo is PlatformIO-layout (src/*.cpp) but the
# toolchain actually used on this estate is arduino-cli, which needs a sketch
# DIRECTORY containing <name>.ino. Rather than restructure the repo around the
# build tool, this assembles a throwaway sketch in a scratch directory.
#
# Secrets: uses src/arduino_secrets.h if you have one, otherwise synthesises a
# placeholder from the tracked example. Nothing real is needed to compile.
#
#   ./tools/build-check.sh
#
# Baseline at 7e7f2f4, before the 4-20 mA work: 85008 bytes flash (32%),
# 11196 bytes RAM (34%). Compare against these.
#
# The "library WDT claims to run on renesas architecture(s)" warning is
# PRE-EXISTING and appears on the untouched baseline. It is not yours.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FQBN="arduino:renesas_uno:unor4wifi"
SCRATCH="${TMPDIR:-/tmp}/plc4-build-check"
SKETCH="$SCRATCH/ArduinoHydraulicScale"

command -v arduino-cli >/dev/null || { echo "arduino-cli not on PATH" >&2; exit 127; }

rm -rf "$SCRATCH"
mkdir -p "$SKETCH"
cp "$REPO/src/ArduinoHydraulicScale.cpp" "$SKETCH/ArduinoHydraulicScale.ino"
for h in "$REPO"/include/*.h "$REPO"/src/*.h; do
  [ -e "$h" ] && cp "$h" "$SKETCH/"
done
if [ ! -f "$SKETCH/arduino_secrets.h" ]; then
  sed -e 's/"your-[a-z-]*"/"x"/g' "$REPO/arduino_secrets.h.example" > "$SKETCH/arduino_secrets.h"
fi

exec arduino-cli compile --fqbn "$FQBN" --warnings default "$SKETCH"
