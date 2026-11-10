# PLC #4 - tractor hydraulic scale

Firmware for Phoenix Nest machine #4: a hydraulic pressure transducer read as a weight, shown on a local LCD and published to MQTT over WiFi, on an Arduino UNO R4 WiFi (`CLAUDE.md:3-7`, `platformio.ini:11-13`). It is a fork of `ArduinoHydraulicScale` (remote `upstream`, `CLAUDE.md:25`). MQTT topics are rooted at `plc4/jd5055e` (`src/ArduinoHydraulicScale.cpp:381`).

Read `BRIEF.md` first (hardware, settled decisions, open questions), then `CLAUDE.md` (boundaries and the firmware change gate). Audit and field results are in `findings/`; `PORT-PLAN.md` records the MQTT forward-port, which is on `main`.

## Layout

- `src/ArduinoHydraulicScale.cpp` - the firmware (one translation unit).
- `include/` - headers (for example `lcd2004.h`, `loop_scaling.h`).
- `test/` - host-side tests, `test_*.cpp`.
- `tools/` - `build-check.sh`, `gen-provenance.sh`, `gen-wifi-networks.sh`, `can_soak_send.py`, `j1939_corpus_check.py`.
- `sketches/` - CAN bench sketches (`can_loopback`, `can_sniff`, `can_twonode`, `can_soak`, `can_mqtt`).
- `ingest/` - consumer-side proposals handed to the consumer repo; nothing there is deployed (`ingest/README.md:1-7`).

## Build

- Copy `arduino_secrets.h.example` to `src/` or `include/` and fill it in; `arduino_secrets*.h` is gitignored (`platformio.ini:24-25`, `.gitignore`). `tools/build-check.sh` synthesises a placeholder if none exists (`tools/build-check.sh:12-13`).
- Compile check: `./tools/build-check.sh`. It uses `arduino-cli` with FQBN `arduino:renesas_uno:unor4wifi` and never flashes (`tools/build-check.sh:3-5,25`).
- PlatformIO environment: `uno_r4_wifi` (`platformio.ini:11`).

## Test

Host-side, with g++, no board needed. Each test file carries its own command, for example:

    g++ -std=c++17 -Wall -Wextra -Iinclude test/test_loop_scaling.cpp -o "$TMPDIR/t" && "$TMPDIR/t"

(`test/test_loop_scaling.cpp:8-9`; `test/test_hmi_menu.cpp:5` uses the same form.) Run each `test/test_*.cpp` the same way.

## Deployment

The board is on the tractor and publishes its running commit as `build` in every status frame (`BRIEF.md:3-12`). Flashing is done at the machine by Alex (`tools/build-check.sh:4-5`). Remotes: `saga` (`/opt/repos/greybeard-plc4-tractor-scale.git`, canonical) and `origin` (GitHub mirror and issue tracker) (`CLAUDE.md:18-23`).
