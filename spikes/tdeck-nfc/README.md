# T-Deck + PN532 NFC spike (throwaway)

The UART version on the Grove plug was replaced 2026-10-25 by I2C, after Alex read the plug's
labels as SDA/SCL. The C3-header plan beside it is set aside.

Alex chose "NFC now, UHF later" on 2026-10-25 (`notes/2026-10-25-rfid-pendant-rulings.md`). This is
the "now" half. It proves the T-Deck host, the reader wiring, the screen and a RECORD press. **It
cannot read the UHF tie tags**: the PN532 is a 13.56 MHz reader. **RECORD sends nothing**: no WiFi,
no MQTT, no HTTP. PLC #4 is in service and is not touched.

Not PLC #4 firmware, and not the pendant's firmware. Keeping any of it is a new decision.

## Wiring

Original T-Deck (not Plus). LilyGO's schematic (`schematic/schematic.pdf`) puts the Grove plug P8
(4-pin, 2.0 mm) on GND, VDD3V3 and the ESP32-S3's GPIO43/GPIO44, named there as UART pins. **Alex's
board labels that plug SDA/SCL** (2026-10-25). So the PN532 runs in **I2C mode**, and the
firmware looks for it on the Grove pins both ways round (SDA43/SCL44, then SDA44/SCL43) and then on
the board's own I2C (SDA 18 / SCL 8, which the keyboard at 0x55 also uses). The screen says which
one answered.

| T-Deck Grove (as labelled) | PN532 V3 (I2C pins, printed on the front) |
|---|---|
| GND | GND |
| 3V3 / VCC | VCC (3.3–5 V per the V3 guide) |
| SDA | SDA |
| SCL | SCL |

PN532 V3 interface switches for I2C: **channel 1 ON, channel 2 OFF** (Elechouse V3 user guide).

Inference, not measured: the PN532 module's draw is well inside what the T-Deck's 3.3 V buck can
give. Its current was not read from a datasheet.

## Use

Power on. The screen shows `Reader: PN532 OK` with the bus it found, or `NOT ANSWERING`. Hold a 13.56 MHz card (MIFARE,
NTAG) within about 5 cm: its UID appears. Press the trackball, or Enter on the keyboard: the screen
says `RECORD: not sent (spike)`. USB serial (115200) logs `SCAN` and `RECORD (not sent)` lines.

## Build and flash

```
cd spikes/tdeck-nfc
TMPDIR=$CLAUDE_JOB_DIR/tmp ~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/serial/by-id/<the T-Deck>
```

The T-Deck enumerates as Espressif USB JTAG/serial (303a:1001, LilyGO `boards/T-Deck.json`).
Confirm `/dev/serial/by-id` names it before uploading; never upload to anything else.
Built 2026-10-25 on Saga with espressif32 6.13.0 (arduino-esp32 2.0.17): RAM 6.4 %, flash 5.8 %.
Not run on hardware yet.
