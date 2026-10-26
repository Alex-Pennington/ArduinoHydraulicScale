# T-Deck + PN532 NFC spike (throwaway)

Back to UART (HSU) on 2026-10-25. An I2C version (bdfc52e) came from misreading Alex: the
SDA/SCL labels he meant are on the PN532 board's header, whose back reads TX/RX (the V3 guide:
I2C pins printed on the front, HSU on the back, same pins). The C3-header plan beside this is
set aside.

Alex chose "NFC now, UHF later" on 2026-10-25 (`notes/2026-10-25-rfid-pendant-rulings.md`). This is
the "now" half. It proves the T-Deck host, the reader wiring, the screen and a RECORD press. **It
cannot read the UHF tie tags**: the PN532 is a 13.56 MHz reader. **RECORD sends nothing**: no WiFi,
no MQTT, no HTTP. PLC #4 is in service and is not touched.

Not PLC #4 firmware, and not the pendant's firmware. Keeping any of it is a new decision.

## Wiring

Original T-Deck (not Plus). Its Grove plug, P8 (4-pin, 2.0 mm), carries GND, VDD3V3, UAR0_TX and
UAR0_RX (LilyGO `schematic/schematic.pdf`); UAR0_TX/RX are the ESP32-S3 module's TXD0/RXD0,
GPIO43/GPIO44. On the T-Deck Plus that plug is the GPS's (LilyGO `README.MD`), so this does not
apply there.

| T-Deck Grove | PN532 V3 (HSU pins: the TX/RX labels on the back of the header) |
|---|---|
| GND | GND |
| 3V3 | VCC (3.3–5 V per the V3 guide) |
| TX (GPIO43) | RXD |
| RX (GPIO44) | TXD |

PN532 V3 interface switches: **both OFF = HSU**, the default (Elechouse V3 user guide). Which wire
colour is which Grove pin is not stated here; check the plug against the schematic's pin order
(1 GND, 2 VDD3V3, 3 UAR0_TX, 4 UAR0_RX) before powering.

Inference, not measured: the PN532 module's draw is well inside what the T-Deck's 3.3 V buck can
give. Its current was not read from a datasheet.

## Use

Power on. The screen shows `Reader: PN532 OK` or `NOT ANSWERING`. Hold a 13.56 MHz card (MIFARE,
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

## Result on hardware (2026-10-26, Saga)

Flashed be62ec4's build to Alex's original T-Deck (MAC 30:30:F9:6E:DB:D4), PN532 V3 on the Grove
plug in HSU mode. Serial log after a normal power-on:

```
PN532 firmware 1.6
SCAN 64:72:07:CA
RECORD (not sent) tag=64:72:07:CA
```

So the T-Deck talks to the reader on GPIO43/44, a 13.56 MHz card reads, and RECORD fires on the
pendant side. The one boot error, `__pinMode(): Invalid pin selected`, is the library being given
no reset pin (255), as intended.

**Flashing this T-Deck:** its download mode took several tries (power off, trackball held, power
on). Most tries came up as Espressif's USB JTAG/serial unit but every write timed out, which
matches LilyGO T-Deck issue #62 (GPIO46 held high by the keyboard ESP32-C3). A reset from esptool
returns it to download mode; a power cycle with the switch starts the app.

**Backup of what was on it before** (Meshtastic, DeviceUI, its settings):
`backup/tdeck-3030F96EDBD4-full-20261026.bin`, 16,777,216 bytes, SHA-256
`ff08d8df0cf90ebbef642f148f4b809014b71d169896c11caf5a1df6fe0c59b1`, verified against the chip
(`verify_flash`, digest matched) before anything was written. Not in git (`.gitignore`). Restore
by writing it whole at `0x0` in download mode.
