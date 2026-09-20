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

# Provenance: capture the commit this image is compiled from. A repo with no git
# (a tarball, a CI export) falls back to the committed sentinel rather than
# failing the build -- "UNKNOWN" is honest about not knowing.
#
# THIS MUST RUN BEFORE THE COPY LOOP BELOW, and the ordering is the whole point.
# gen-provenance.sh writes $REPO/include/build_provenance.h; the loop below is
# what carries that file into the sketch. Generated AFTER the loop -- which is
# where this call used to sit -- the fresh header never reached the build, so
# every image was stamped with whatever the PREVIOUS run had left on disk, or
# with the "UNKNOWN" sentinel on a fresh clone where the gitignored header did
# not exist yet.
#
# Measured before the fix: a clean tree at 608aebf5 compiled an image that
# identified itself as c601d792+dirty -- four commits adrift AND falsely dirty.
# The board publishes that string as `build` on the status topic and a
# technician asks it what is running, so a stale value here is a machine in a
# field confidently reporting the wrong source. Do not move this call back down.
"$REPO/tools/gen-provenance.sh" >/dev/null 2>&1 || true

for h in "$REPO"/include/*.h "$REPO"/src/*.h; do
  [ -e "$h" ] && cp "$h" "$SKETCH/"
done
if [ ! -f "$SKETCH/arduino_secrets.h" ]; then
  sed -e 's/"your-[a-z-]*"/"x"/g' "$REPO/arduino_secrets.h.example" > "$SKETCH/arduino_secrets.h"
fi

# wifi_networks.h is generated from real credentials by tools/gen-wifi-networks.sh
# and is gitignored, so a fresh checkout has only the example. Fall back to it:
# the example declares a single network from arduino_secrets.h, which is exactly
# the pre-multi-AP behaviour and compiles identically.
if [ ! -f "$SKETCH/wifi_networks.h" ]; then
  cp "$REPO/include/wifi_networks.h.example" "$SKETCH/wifi_networks.h"
fi

# The no-git fallback, kept deliberately: gen-provenance.sh exits non-zero
# outside a repository and copies nothing, so the sentinel is what a tarball
# build gets. Inside a repository the loop above has already placed the freshly
# generated header and this branch does not fire.
if [ ! -f "$SKETCH/build_provenance.h" ]; then
  cp "$REPO/include/build_provenance.h.example" "$SKETCH/build_provenance.h"
fi

exec arduino-cli compile --fqbn "$FQBN" --warnings default "$SKETCH"
