# Plan: the PN532 on the keyboard ESP32-C3's header (TX21 / RX20)

**Status: plan only. Nothing here is built or flashed.** Alex, 2026-10-25: *"lets plan to use pin
21,22"*. Asked whether that means the 6-pin ESP32-C3 header labelled TX(21), RX(20), IO09(BOOT),
RST, GND, VCC, he chose **"Yes, TX21 / RX20"**. The spike as first built put the reader on the
Grove plug (S3 GPIO43/44); this plan moves it.

Sources: LilyGO T-Deck repo, `images/c3_boot.jpg` (the header labels), `README.MD` Tips 3 (header
order and how to flash the C3), `schematic/schematic.pdf` (C3 nets), and
`examples/Keyboard_ESP32C3/Keyboard_ESP32C3.ino` (the stock keyboard firmware, MIT,
© 2023 Shenzhen Xin Yuan Electronic Technology). **Inference** marks what no source states.

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
- **The header's VCC voltage is NOT DETERMINABLE** from what was read. Inference: 3.3 V, the C3's
  rail. The PN532 V3 takes 3.3–5 V (Elechouse V3 guide), so either works; measure before wiring.

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
  (`:206-213`, `:239-268`, `:292-293`). Inference: on this board `Serial` is UART0, the header, so
  that text would go straight into the reader.
- Add one I2C command, proposed `0x05` ("tag"). The C3 answers the next read with a length byte
  and up to 7 UID bytes, or `0x00` when there is no new tag. The key protocol does not change, so
  the keyboard works as before.

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
- **Undo:** LilyGO ships the stock keyboard image, `firmware/T-Keyboard_Keyboard_ESP32C3_250620.bin`.
  Flashing it back restores the keyboard.

## Risks (inference)

- At reset the C3's ROM prints its boot log on U0TXD, so the PN532 sees noise on its RX.
  `Adafruit_PN532::begin()` flushes the port and wakes the chip (library 1.3.4,
  `Adafruit_PN532.cpp:162-173`). Expected harmless; not tested.
- A bad C3 image leaves the keyboard dead until it is reflashed. The trackball still gives RECORD.
- The UHF reader that comes later (the M5Stack U107 / JRD-4035 is the candidate) is also a UART
  part at 115200, so this wiring carries over, but its draw (about 180 mA, more at peak) and its
  3.5–5 V supply may not suit the header's VCC. Not checked.

## The Grove build already on the branch

`src/main.cpp` (reader on the S3's Grove plug) stays as committed until the C3 version replaces
it.
