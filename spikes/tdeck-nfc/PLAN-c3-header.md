# Plan: the PN532 on the keyboard ESP32-C3's header (TX21 / RX20)

> **Set aside by Alex, 2026-10-25.** Asked about the USB-TTL adapter it needs, he chose "Use the
> Grove plug" (*"Skip the keyboard chip for this test; flash the Grove build already made"*, this
> session's option text) and wrote *"just use the othe pins"*. The test uses `src/main.cpp` on the
> Grove plug. This plan is kept as the record.

**Status: plan only. Nothing here is built or flashed.** Alex, 2026-10-25: *"lets plan to use pin
21,22"*. Asked whether that means the 6-pin ESP32-C3 header labelled TX(21), RX(20), IO09(BOOT),
RST, GND, VCC, he chose **"Yes, TX21 / RX20"**. The spike as first built put the reader on the
Grove plug (S3 GPIO43/44); this plan moves it.

Sources: LilyGO T-Deck repo, `images/c3_boot.jpg` (the header labels), `README.MD` Tips 3 (header
order and how to flash the C3), `schematic/schematic.pdf` (C3 nets), and
`examples/Keyboard_ESP32C3/Keyboard_ESP32C3.ino` (the stock keyboard firmware, MIT,
© 2023 Shenzhen Xin Yuan Electronic Technology). **Inference** marks what no source states. Adversarially reviewed against those sources on
2026-10-25; its corrections are folded in. Build core: espressif32 6.13.0, arduino-esp32 2.0.17.

## What the header is

- The T-Deck's keyboard is a separate ESP32-C3, an I2C slave at `0x55` on its GPIO2 (SDA) and
  GPIO10 (SCL) (`Keyboard_ESP32C3.ino:18-21`). The S3 reads it on its own I2C, SDA 18 / SCL 8.
- The header is the C3's programming port. README Tips 3: *"located at the 6Pin header on the
  side of the RST button, and the order starts from the top of the RST button, which are
  TX,RX,BOOT,RST,GND,VCC"*. The schematic's C3 pins `U0TXD` and `U0RXD` carry nets `UART_TX` and
  `UART_RX`.
- GPIO20 and GPIO21 are not in the keyboard matrix (rows `{0,3,19,12,18,6,7}`, cols
  `{1,4,5,11,13}`, `Keyboard_ESP32C3.ino:33-36`), so UART0 is free for the reader once the
  firmware stops using it.
- **The header's VCC is the `V3V` net**, the same net as the C3's VDD3P3_CPU (schematic J1 pin 1,
  per the review). Inference: 3.3 V. The PN532 V3 takes 3.3–5 V (Elechouse V3 guide). That rail's
  current budget is NOT DETERMINABLE here.
- **The header is 2.0 mm pitch** (schematic `6PIN_2.0`). Standard 2.54 mm Dupont leads may not
  seat.

## Wiring

| C3 header | PN532 V3 (HSU, switches both OFF) |
|---|---|
| GND | GND |
| VCC | VCC |
| TX(21) | RXD |
| RX(20) | TXD |

IO09(BOOT) and RST stay unconnected.

## Firmware: two images instead of one

**C3 (keyboard) image.** Start from LilyGO's `Keyboard_ESP32C3.ino` and keep its matrix scan,
backlight and I2C commands `0x01`–`0x04` unchanged.

- Add the PN532 on UART0, 115200 baud, pins RX 20 / TX 21.
- **Take out every `Serial.print`.** The stock firmware prints debug text on `Serial` at 115200
  (`:101`, `:106`, `:206-226`, `:239-268`, `:292-293`, `:367`, `:380-384`), some of it inside the
  I2C callbacks. LilyGO builds it as "ESP32C3 Dev Module" with "USB CDC On Boot: Disabled"
  (`images/c3.jpg`), so `Serial` is UART0, the header, and that text would go straight into the
  reader. **Build the C3 image with no `ARDUINO_USB_CDC_ON_BOOT` flag**; the S3 spike's `=1` would
  move `Serial` to USB.
- **Poll the reader in `loop()` and cache the tag.** `readPassiveTargetID()` blocks for its timeout;
  run from an I2C callback it would stall the bus and the matrix scan.
- Add one I2C command, proposed `0x05` ("tag"). **One shot:** only the read straight after a
  `0x05` write gets the tag reply (a length byte and up to 7 UID bytes, or `0x00` for no new tag),
  and the flag clears on that read. Every other read stays the stock 1-byte key reply
  (`Keyboard_ESP32C3.ino` `onRequest()`), so a latched tag can never be read as a keystroke.

**S3 image (the spike).** Same screen and RECORD as now. It reads the tag from the C3 over I2C
(`0x05`) instead of from `Serial1`. RECORD still sends nothing.

## Flashing the C3

- Through that same header, with an external USB-to-TTL adapter (README Tips 3). Short BOOT
  (IO09) to GND before power-on, flash, then remove the short.
- **The PN532 has to come off the header while flashing**, because the adapter uses the same
  TX/RX.
- **Saga has no free adapter.** Its only USB serial device today is `usb-1a86_USB_Single_Serial_…`,
  which belongs to other work and is not touched. A 3.3 V USB-TTL adapter is needed, plugged into
  Saga.
- **Never connect the adapter's VCC while the T-Deck is powered**: it is the same `V3V` net. Use a
  3.3 V adapter.
- **Undo:** LilyGO ships the stock keyboard image, `firmware/T-Keyboard_Keyboard_ESP32C3_250620.bin`
  (4,128,768 bytes). It is a merged image (bootloader, partitions, app), so it is written at
  **offset 0x0**, chip ESP32-C3. Written anywhere else it would leave the keyboard dead.

## Risks (inference)

- At reset the C3's ROM prints its boot log on U0TXD, so the PN532 sees noise on its RX. It cannot
  be turned off without eFuse changes. `Adafruit_PN532::begin()` flushes only the C3's own receive
  buffer before waking the chip (library 1.3.4, `Adafruit_PN532.cpp:162-173`); whether the noise
  upsets the PN532's parser is UNVERIFIED until tested.
- A bad C3 image leaves the keyboard dead until it is reflashed. The trackball still gives RECORD.
- The UHF reader that comes later (the M5Stack U107 / JRD-4035 is the candidate) is also a UART
  part at 115200, so this wiring carries over, but its draw (about 180 mA, more at peak) and its
  3.5–5 V supply may not suit the header's VCC. Not checked.

## The Grove build already on the branch

`src/main.cpp` (reader on the S3's Grove plug) stays as committed until the C3 version replaces
it.
