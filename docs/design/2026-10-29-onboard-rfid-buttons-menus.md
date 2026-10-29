# PLC #4 on-board NFC reader, buttons and LCD menus — design (2026-10-29)

**Status: design only.** Nothing here is in the firmware, and nothing changes on the tractor until
Alex approves each firmware change named in §6 one by one. A binder card follows a change only once
it is approved and flashed. Until then `references/` describes the running firmware.

**How to read the claims.** A bracketed number cites §11 *References*. `src:N` means
`src/ArduinoHydraulicScale.cpp` line N on this branch's base. **Inference** marks a conclusion that
no source states. **NOT DETERMINABLE** marks a fact no source reached here settles, and says what
would settle it. **Pending gleipnir** marks a claim relayed from the gleipnir coordinator that this
design has not checked against source. Nothing in this document was measured on hardware.

## 1. The instruction in force

Alex, 2026-10-29, to the PLC #4 coordinator:

> *"coordinate with @ymir-gleipnir-coordinator-1028-0d32 to add the same rfid chip that the t-deck
> pendant uses.  that way an operator at the tracktor can scan a rfid tag.  add the nessisary
> buttons and menues to the design"*

This instruction is in force. It replaces three earlier lines, which stay on record as superseded:

| Superseded line | Where it is recorded | Superseded by |
|---|---|---|
| Alex, 2026-10-25: *"Disconnect the reader from the plc4 hardware and create a pendant style reader"* | `notes/2026-10-25-rfid-read-location-ruling.md:13` | The 2026-10-29 instruction: the reader goes back on PLC #4 |
| Alex, 2026-10-26, as quoted in consumer migration 129: *"plc4 will never have rfid we move it to the pendant"* | greybeard-plc-consumer `db/migrations/129_plc4_tag_from_pendant.sql:4-5` [20] | The same 2026-10-29 instruction. Restating that comment is the consumer coordinator's work, not this repo's |
| The board's discrete buttons were removed so the outdoor enclosure needs no holes: *"five holes being five places for water to get in"* | `BRIEF.md:191-193`; `references/01D_display.md:13` (*Inputs: None*) | The same instruction asks for buttons. §3 designs them for that enclosure |

The pendant is not retired. Alex said *"the same rfid chip that the t-deck pendant uses"*, so both
readers exist. The board and the pendant are two clients of one record path (§5). His 2026-10-25
answer on pendant count, *"Design so either works"* (`notes/2026-10-25-rfid-pendant-rulings.md:78`),
holds for a fleet of one board reader and any number of pendants.

## 2. The reader

### 2.1 Part

**Elechouse PN532 NFC RFID Module V3**, built around the NXP PN532 [1], [2], [3]. It is the reader
the T-Deck spike read on 2026-10-26. That spike logged reader firmware 1.6 and UID `64:72:07:CA`
(`spikes/tdeck-nfc/README.md:59-63`), and the mesh pendant driver names the same module
(mesh `firmware/src/modules/plc/core/Pn532.h:4-6` [14]).

| Fact | Value | Source |
|---|---|---|
| Size | 42.7 × 40.4 × 4 mm | [1] p. 2 |
| Supply, VCC | 3.3–5 V | [1] *Interface* |
| I2C and HSU logic | 3.3–24 V TTL, through an on-board level shifter | [1] *Features* 5, *Interface* |
| Mode switch | I2C = channel 1 ON, channel 2 OFF. HSU, both OFF, is the factory default | [1] *Interface* table |
| Antenna | On-board PCB antenna, 5–7 cm read distance | [1] *Features* 4 |
| Wiring rule | Wires cross the antenna lines at 90° | [1] *Hardware Installation* |
| I2C address | 0x48 in 8-bit form, which is 7-bit **0x24**. Clock up to 400 kHz | [3] §6.1.1.3; [2] |
| Host handshake | The host reads a status byte. RDY = 1 means a frame is waiting. No IRQ line is needed | [3] §6.2.4, §6.2.4.1 |
| SCL hold | The PN532 holds SCL after its address when it wakes from power-down. tStrWuSpd is 1 ms | [3] *I2C wake up condition*; [2] tStrWuSpd |
| Supply current, from Table 1 | IDVDD 25 mA typ. IAVDD 6 mA typ. ITVDD 60 mA typ and 150 mA max while transmitting | [2] Table 1 |
| Ambient temperature | −30 to +85 °C | [2] Table 1 |

**Consequence:** the reader reads a tag **held within about 5 cm of the mark on the lid**. It does
not read a tag on a bag hanging on the loader. The operator brings the tag to the box. That matches
the instruction's *"an operator at the tractor can scan a rfid tag"*. It is not the 2026-10-07
"board reads the bag at the press" picture (`notes/rfid-inventory-discussion.md:21-27`).

**Consequence:** the PN532 floor of −30 °C [2] becomes the enclosure's cold limit. The UNO R4 WiFi
is rated to −40 °C [4] §2.

The PN532 is a 13.56 MHz reader. It cannot read UHF EPC Gen2 tie tags. That was already true of
the pendant (`notes/2026-10-26-rfid-pendant-handoff.md:47`), and Alex chose *"Not yet"* for UHF on
2026-10-26 (`notes/2026-10-25-rfid-pendant-rulings.md:110`).

### 2.2 Bus: the header I2C bus, `Wire`, on A4/A5

Choice: **I2C on the header bus `Wire` (A4/SDA, A5/SCL)**. The mode switch is set to I2C. The
reader is not on the Qwiic trunk, and not on a UART.

What the board has free, read from source and the variant:

| Resource | State today | Source |
|---|---|---|
| D0/D1, `Serial1`, UART2 | HC-05 console at 9600 | `src:26-44`, `src:1460`; `references/01A_radio_module.md:23-24` |
| Other hardware UARTs | UART1 is on pins 22/23 and UART3 on 24/25. Neither is on the header | [7] `pins_arduino.h:106-112` |
| SPI | D10-D13 (`pins_arduino.h:130-133`). D10 and D13 are CAN transmit and receive | [7]; `references/01_controller_card.md:21-22`; [4] §12.2 |
| `Wire1`, Qwiic | Mux 0x70, the only device on the trunk. Channels 0-3 and 7 are used, 4-6 are free | `include/i2c_mux.h:14-39`; `references/01C_qwiic_stack.md:12-22` |
| `Wire`, A4/A5 and SDA/SCL | **Unused.** Nothing in `src/` or `include/` calls `Wire.` | grep of `src/` and `include/`; [4] §10 note "standard I2C bus object is Wire" |
| D2-D9, D11, D12, A0-A3 | Unused. No `pinMode`, `digitalRead` or `analogRead` anywhere | grep of `src/` and `include/` |

Why the alternatives lose:

- **HSU on D0/D1** (the spike's mode) takes the HC-05's only header UART. The 2026-10-07 plan did
  this for the UHF reader and mothballed the Bluetooth console (`notes/rfid-inventory-discussion.md:58-60, 102`).
  Using I2C leaves the console exactly as it is.
- **SPI** collides with CAN on D10/D13.
- **A Qwiic mux channel (4, 5 or 6)** would work by address: 0x24 is free on every channel. But the
  PN532 holds SCL for up to 1 ms when it wakes [2], [3], and a reader fault that holds SDA low would
  stall the whole trunk. The weight converter (ch3) and the LCD (ch7) are on that trunk, and the
  mux is already its single point of failure (`include/i2c_mux.h:27-30`). On `Wire`, a separate
  RA4M1 I2C controller [4] §10, a reader fault cannot touch the weight path or the display.

**Address and conflict check.** `Wire` carries only the reader at 0x24. Nothing on `Wire1` changes.
D0/D1 and the HC-05 do not change. CAN on D10/D13 does not change.

**Logic level.** The R4's RA4M1 runs at 5 V [4] §11.2, so the header bus is 5 V. The module's
I2C side takes 3.3–24 V through its level shifter [1], so power it from 5 V and wire it directly.
Do not wire it to the Qwiic 3.3 V rail.

**Pull-ups.** Whether the Elechouse V3 or the R4 carries pull-ups on the `Wire` lines is NOT
DETERMINABLE: neither datasheet read here says. The module schematic linked from [1] would answer
it. Choice: fit 4.7 kΩ from SDA and from SCL to +5 V at the board end. **Consequence:** if the
module carries 4.7 kΩ of its own, the parallel value is about 2.35 kΩ, which sinks about 2.1 mA at
5 V. That is inside the 3 mA I2C standard-mode sink (inference from the I2C specification, not read
here).

### 2.3 Wiring

| Reader pin, I2C label on the front [1] | Board terminal | Note |
|---|---|---|
| GND | GND | |
| VCC | +5V | §2.4 |
| SDA | A4/SDA (also on the JDIGITAL SDA pin, the same bus [4] §10) | 4.7 kΩ to +5 V at the board end |
| SCL | A5/SCL (also on the JDIGITAL SCL pin) | 4.7 kΩ to +5 V at the board end |

Run four conductors, about 150 mm with a service loop, from the lid to the board. Put a 4-pin
connector at the board end so the lid can come off. IRQ and RSTPDN stay unconnected.

The Adafruit PN532 1.3.4 constructor calls `pinMode()` on its IRQ argument
(`Adafruit_PN532.cpp:110` [16]), even though its I2C ready check reads the status byte over the
bus (`:1555-1558`). §6 FW-1 avoids that library, so this design has no effect on it.

### 2.4 Power

The reader draws from the board's +5V pin. Budget: Table 1 gives up to 181 mA while transmitting
(IDVDD 25 + IAVDD 6 + ITVDD 150 max) [2]. Plan on **0.2 A**, plus whatever the module's own LED
and regulator take, which is NOT DETERMINABLE from [1].

Whether the R4's 5 V rail has 0.2 A to spare is NOT DETERMINABLE here. The R4 datasheet names the
ISL854102FRZ buck that makes 5 V from VIN, but it gives no current figure [4] §11. How the board is
powered on the tractor is also changing (`notes/case-v2-log.md:30-34`). The ISL854102 datasheet
and the supply Alex fits settle it. If the rail is short, feed the module's VCC from the off-board
5 V supply named in `notes/rfid-inventory-discussion.md:35-36`, with a common ground. The I2C
levels still work, because the module's level shifter takes 3.3–24 V [1].

### 2.5 Antenna placement in the enclosure

The enclosure is the 3D-printed ASA case in greybeard-plc4-pwa `design/case-print/` [17]. Its lid
is a 4 mm plate (`LID_T = 4.0`) on a 200 × 165 mm case. Coordinates below are that file's layout
coordinates: top view, x along the 200 mm side, y running toward the gland wall. In
`case_params.py`, the USB edge of the reader faces "the bottom (gland) wall" at higher y
(`case_params.py:107`).

- **The board goes on the inside face of the lid, antenna side to the lid, with no metal in front
  of it.** This is the rule V1 already applied to the M7E (`notes/2026-10-25-rfid-case-print.md:46-49`).
- **Position: the top-left of the lid, x 14–56.7, y 16–56.4** (42.7 × 40.4 mm). That clears the
  top-left corner screw block (`CORNER_K = 25`, `case_params.py:42`: 14 + 16 = 30 > 25). It keeps
  the PN532 away from the LCD module's metal frame at x 51–149, y 92–152 (`case_params.py:91`).
  V1 put the M7E at x 10–45.6, y 87–148 (`case_params.py:100-101`), which would leave the PN532
  about 4 mm from that frame.
- **Mark the outer face** with an "NFC" target over the antenna centre, at about (35, 36).
- **Keep stainless parts 25 mm or more from the antenna edge.** The button bezels in §3.3 are 27 mm
  from it. The figure is inference: a nearby metal ring loads a 13.56 MHz loop. Before fixing the
  holes, compare the read distance with and without the bezels fitted.
- The UNO sits on the floor under this corner (x 10–78.6, y 12–65, `case_params.py:45`), roughly
  40 mm behind the antenna (inference from a 70 mm inside height, `case_params.py:20`, less the
  board stack). How much its ground plane shortens the 5–7 cm range is NOT DETERMINABLE here. A
  read with the lid closed and the case assembled answers it.
- The Elechouse board's mounting-hole pattern is NOT DETERMINABLE from [1]. Measure the board and
  set the V2 lid bosses from it. The M7E's bosses (`READER_HOLES_BOARD`, `case_params.py:105`)
  are removed.

These are changes to the V2 case in greybeard-plc4-pwa. That repo is not edited from here. The
case log's dispatched layout worker carries them (`notes/case-v2-log.md:77-80`).

## 3. The buttons

### 3.1 Count: four

**BACK, UP, DOWN, OK.** Four is the smallest set where every screen in §4 has a one-press cancel
and needs no "back" entry in a list:

- UP and DOWN move through a list or change a value.
- OK acts.
- BACK cancels or leaves.

Three buttons would save a hole, but every confirmation would then need a "NO" item that the
operator scrolls to. The mesh pendant case has four push buttons plus an encoder
(`notes/2026-10-25-rfid-pendant-rulings.md:14-17`). The baseline keypad had five
(`BRIEF.md:191-192`).

### 3.2 Part

**SparkFun COM-11966, "Metal Pushbutton with Wires — Momentary (16mm, Red)"** [5], [6]. The family
COM-11966 through COM-11976 offers other colours [6].

| Fact | Value | Source |
|---|---|---|
| Body | Stainless steel | [5], [6] |
| Ingress rating | IP65 | [5], [6] |
| Contact | SPDT, momentary; rated 3 A at 250 VAC | [6] |
| Thread | 5/8 in. The drawing gives a 15.862 mm major diameter | [6] |
| Bezel | 22.606 mm on the drawing (the 0.8900 in dimension) | [6] |
| Length | 30.975 mm on the drawing; "overall length (including leads) is 1.5 in" | [6] |
| Seal | One rubber gasket ring is supplied | [5] *Includes* |
| LED ring | Rated 5 V. Current is not stated | [6] |

**Consequence:** IP65 means protected against water jets, not immersion. That is a reading of the
IEC 60529 code, which was not read here. Rain and a hose are covered. Standing under water is not.
The printed case's own ingress rating is NOT DETERMINABLE, so the enclosure as a whole has no
rating here.

NOT DETERMINABLE from [6]: the maximum panel thickness the thread accepts (the lid is 4 mm), the
operating temperature range, and the contact bounce time. Check the thread length on a sample
against 4 mm of lid plus nut plus gasket before cutting holes.

**Holes:** 16.2 mm. That is the clearance the case already uses for its M16 panel-mount USB-C
(`USB_HOLE = 16.2`, `case_params.py:85`), applied here to a 15.86 mm thread (inference). Seat the
supplied gasket under the bezel on the outside face. Print the lid solid (100 % infill) for a
radius of at least 15 mm around each hole, so the gasket seals against plastic rather than infill
(inference; a print setting for the case worker).

**The LED rings are left unwired** in this design. Their current is not stated in [6], and the
RA4M1's GPIO limit is 8 mA [4] §11.3. Lighting OK while a tag is armed would be a useful later
change through a transistor. It is not designed here.

### 3.3 Where

A 2 × 2 cluster on the lid, above the LCD and to the right of the reader. Centres, in the layout
coordinates of §2.5:

| Button | Centre (x, y) |
|---|---|
| BACK | (95, 25) |
| UP | (130, 25) |
| OK | (95, 58) |
| DOWN | (130, 58) |

The pitch is 35 mm across and 33 mm down, which leaves 12.4 mm and 10.4 mm between 22.6 mm bezels,
enough for a gloved finger (inference). The bezels clear:

- the reader by 27 mm (95 − 11.3 − 56.7);
- the LCD module (y 92) by 22.7 mm;
- the DIN-rail and DDR-30 column, which starts at x 156 (`case_params.py:62`), at x 141.3.

Behind the lid, the buttons reach about 27 mm below the lid's inner face (inference from the
31 mm drawing length less the 4 mm lid). Below them sits the I2C block, x 92–152, y 12–57, 15 mm
tall on 3 mm pads (`case_params.py:161`, `:53`). That leaves about 25 mm of clearance inside the
70 mm height (inference). Up and down are glands-down, as V1 mounts (inference).

### 3.4 Pins and wiring

| Button | Pin | Contact used |
|---|---|---|
| BACK | D2 | C to GND, NO to the pin |
| UP | D3 | C to GND, NO to the pin |
| DOWN | D4 | C to GND, NO to the pin |
| OK | D5 | C to GND, NO to the pin |

At the board end, each pin has 4.7 kΩ to +5 V and 100 nF to GND. Do not use the internal pull-up:
the RA4M1's RU is 10–50 kΩ [8], weak for a 30 cm run on a tractor (inference). The RC is 0.47 ms,
which filters spikes and costs nothing against 20 ms debounce. A pressed button draws about 1 mA.

Run five conductors (GND plus four signals) from the lid with a service loop. Put a 5-pin connector
at the board end. Leave NC and the LED leads insulated. D6-D9, D11, D12 and A0-A3 stay free.

### 3.5 Debounce

- **Sample in a 200 Hz timer interrupt, not in `loop()`.** `loop()` blocks for up to 23 s during a
  WiFi reconnect (`src:209-220`), and a press made then would be lost. The firmware already runs two
  `FspTimer`s: the 2 Hz watchdog heartbeat (`src:1566-1575`) and the 200 Hz CAN drain
  (`include/j1939_can.h:35, 135-141`). A third comes from `FspTimer::get_available_timer()`, the
  same way. Whether one is free is NOT DETERMINABLE without a build on the board. If none is free,
  poll from `loop()` and accept that presses during a network stall are lost. The console says
  which it got, as it already does for the heartbeat (`src:1576-1577`).
- **Integrate:** a button's state changes after 4 equal samples in a row (20 ms). The 20 ms figure
  is a common bounce allowance, not a datasheet figure ([6] states none).
- **Events:** a release-to-press edge queues one press event, up to 4 deep. `loop()` drains the
  queue. UP and DOWN auto-repeat while held: first repeat after 500 ms, then every 150 ms. BACK and
  OK never repeat.
- **KEY STUCK:** a button held for 10 s or more is ignored until released, and `KEY STUCK` goes on
  the fault row. It ranks where the old keypad's branch ranked, above `I2C ERR`
  (`src:1662-1664` records that the branch "left with the buttons").

## 4. The menus

### 4.1 Rules every screen keeps

- **Row 0 is the weight row of today's home screen, on every screen, unchanged.** It reads
  `W %8d lb  n%-4d`, or the fault name when the reading cannot be trusted (`src:1682-1685`). The
  operator taring or recording always sees the weight they act on.
- **Row 3 is the fault row and belongs to the display layer.** It is drawn on every render, and no
  screen can opt out (`src:1652-1669`).
- **So a screen owns rows 1 and 2.** A list shows two items at a time.
- **Text is plain ASCII.** The cursor is `>`, and more items above or below are `^` or `v` in
  column 20. No CGRAM characters.
- **Timeouts:** a menu returns to HOME after 30 s with no press. TAG returns after 120 s
  (§4.3). A result screen returns after 3 s or on any press.
- **One rule for a scan:** a new tag opens TAG from any screen except one that is waiting on the
  panel (§4.4). There the tag is held and TAG opens when the wait ends.

Every screen below is shown in full, 20 columns by 4 rows, between `|` marks. The weight, counts
and labels are examples.

### 4.2 HOME (today's screen, unchanged)

```
|W     1834 lb  n12  |
|psi  2850  mA 19.21 |
|link mqtt up        |
|                    |
```

OK opens MENU (§4.7). A scan opens TAG. BACK, UP and DOWN do nothing.

### 4.3 TAG — a tag is armed

A new scan arms the tag. These rules match the pendant's `RfidTag.h` [15]:

- A UID is a new scan when it differs from the last one seen, or when the reader has seen no tag
  for 2000 ms (`kGoneMs`).
- A tag left on the reader is one scan, not ten a second.
- An armed tag expires after 120000 ms (`kArmedMs`).
- A tag is used up when the board records with it.

On the board, **armed means TAG is showing**. BACK drops the tag and goes HOME. There is no hidden
armed state.

```
|W     1834 lb  n12  |
|TAG 64:72:07:CA     |
|DAILY LOG     OK=REC|
|                    |
```

- Row 1 is the screen form the pendant uses, `hexShown()` with colons [15]. A 4-byte UID fits
  after `TAG `. A 7-byte UID is 20 characters alone, so `TAG ` drops. A 10-byte UID, 29 characters
  with colons, shows as 20 bare hex characters, the wire form.
- Row 2 is the destination: the open tally the record goes into. It defaults to the panel's
  selected tally. UP and DOWN step through the open tallies the panel lists. A delivery shows its
  panel label cut to 13 characters:

```
|W     1834 lb  n12  |
|TAG 64:72:07:CA     |
|SMITH #4411   OK=REC|
|                    |
```

- With no tally open, row 2 reads `NO TALLY      OK=REC`. The record then goes to the board's
  tally only, as a panel RECORD does with none open (greybeard-plc4-pwa `api/app.py:353-354` [18]).
- OK records with the tag into the destination shown (§5). If the panel's last credited bag is
  under 15 min old and not emptied, DUP WARN comes first (§4.4).

### 4.4 Record flow screens

**DUP WARN.** Alex ruled this warning for the panel and the handhelds after one bag was recorded
twice from two handhelds 50 s apart. He chose *"Yes, PWA + handhelds"* and set the window: *"under
15 min old"* (mesh merge commit of `feat/dup-record-warning` [14]). Applying it to the board, a
third client, is this design's reading of that ruling.

```
|W     1834 lb  n12  |
|LAST 1834 LB 50S AGO|
|OK=AGAIN  BACK=STOP |
|                    |
```

**SENDING.** This is drawn before the HTTP call, because the call blocks `loop()`.

```
|W     1834 lb  n12  |
|RECORDING...        |
|TAG 64:72:07:CA     |
|                    |
```

**RECORDED.** The board's own handler ran the `+` under the id it minted (§5.2).

```
|W     1834 lb  n13  |
|RECORDED    1834 LB |
|INTO DAILY LOG      |
|                    |
```

**NOT RECORDED.** Row 2 is the refusal reason:

- the board's ack detail, at most 20 characters (`src:2228-2238`); or
- the panel's own state name on a 409 (`api/app.py:339`, the `state_name` in the gate's answer [18]); or
- `PANEL 400` or `PANEL 409` for any other panel refusal. The full error text goes to the console.

OK tries again under a **new** id: a repeat of the old id would replay the refusal
(`references/03_mqtt_contract.md:380-381`). BACK returns to TAG with the tag still armed.

```
|W     1834 lb  n12  |
|NOT RECORDED  OK=TRY|
|UNDER ZERO          |
|                    |
```

**NO ANSWER.** The panel took the command (HTTP 202), but the board's handler did not run it within
10 s. OK resends under the **same** id. The panel pins the tag to the id
(`simple_tally.py` `_check_tag()` [18]), and the board's dedup ring replays rather than weighing
twice (`references/03_mqtt_contract.md:380-381`). The 10 s matches the app's resend wait in
`notes/2026-10-07-rfid-record-contract.md:473-476`.

```
|W     1834 lb  n12  |
|NO ANSWER  OK=RESEND|
|TAG 64:72:07:CA     |
|                    |
```

**NO PANEL.** The HTTP call failed. Row 2 is today's link text (`src:1704-1709`). OK retries.
Nothing records without the panel; that is the 2026-10-07 range-only ruling
(`notes/2026-10-07-rfid-record-contract.md:24-25`).

```
|W     1834 lb  n12  |
|NO PANEL   OK=RETRY |
|link wifi only      |
|                    |
```

**RFID BUILD.** The panel answers with its RFID build. The board's tag on `+` is refused there
(`api/app.py:299`), and the pendant switches its PLC #4 buttons off in the same case
(`notes/2026-10-26-rfid-pendant-handoff.md:26-27`). So does the board.

```
|W     1834 lb  n12  |
|PANEL IS RFID BUILD |
|BUTTONS OFF         |
|                    |
```

### 4.5 MENU

```
|W     1834 lb  n12  |
|>RECORD NO TAG      |
| TARE              v|
|                    |
```

The items, in order: `RECORD NO TAG`, `TARE`, `UNDO LAST`, `TALLIES`, `CLEAR BOARD`, `INFO`.
These are the panel's operator verbs and the pendant's labels: `tare`, `+`, `-` and `b` are
greybeard-plc4-pwa `PANEL_VERBS` (`api/commands.py:110`) [18], and the pendant's labels are mesh
`Plc4Operator.cpp:23-24` [14]. `kv` (calibration) is a panel verb but not a pendant action, and is
left off the buttons. The phone and the console keep it (`references/06_operating_card.md:20-25`).

```
|W     1834 lb  n12  |
| UNDO LAST         ^|
|>TALLIES           v|
|                    |
```

- `RECORD NO TAG` opens TAG's flow with no tag: SENDING, then a result.
- `TARE`, `UNDO LAST` and `CLEAR BOARD` open a CONFIRM.
- `TALLIES` opens TALLIES.
- `INFO` opens INFO.

### 4.6 CONFIRM

The pendant confirms the same four actions (mesh `Plc4Operator.cpp`, `confirmText()` [14]).

```
|W     1834 lb  n12  |
|TARE: EMPTY AT MARK?|
|OK=TARE   BACK=NO   |
|                    |
```

The question is the operating card's tare condition, *"Boom at the mark, bucket empty"*
(`references/06_operating_card.md:22`).

```
|W     1834 lb  n12  |
|UNDO LAST? 1834 LB  |
|OK=UNDO   BACK=NO   |
|                    |
```

The weight is the board tally's last entry, `rT[rTindex-1]` (`src:1934-1955`).

```
|W     1834 lb  n12  |
|CLEAR 12 ENTRIES?   |
|OK=CLEAR  BACK=NO   |
|                    |
```

`b` clears immediately and without confirmation on the console
(`references/06_operating_card.md:40`). The button path confirms.

OK sends the verb through the panel (§5.4). The result shows as `TARED`, `REMOVED` or `CLEARED`,
or as NOT RECORDED's refusal form with the verb's own reason.

### 4.7 TALLIES

The panel's open tallies, then the starts it allows. `*` marks the selected tally. The count is
the panel's `n` for that tally, read from `/api/state` the same way the pendant reads it
(mesh `Plc4Operator.cpp:631-663` [14]).

```
|W     1834 lb  n12  |
|>DAILY LOG   n12 *  |
| SMITH #4411 n3    v|
|                    |
```

```
|W     1834 lb  n12  |
| SMITH #4411 n3    ^|
|>+ DELIVERY         |
|                    |
```

- `+ DAILY LOG` appears only when the panel says a daily log can start (`can_start.daily_log`,
  mesh `Plc4Operator.cpp:640` [14]). OK opens CONFIRM `START DAILY LOG?`.
- `+ DELIVERY` opens DELIVERIES.
- OK on a tally opens TALLY.

### 4.8 TALLY

Row 1 names the tally. Row 2 is a one-line chooser that UP and DOWN step through.

```
|W     1834 lb  n12  |
|SMITH #4411 n3 *    |
|>SELECT             |
|                    |
```

```
|W     1834 lb  n12  |
|SMITH #4411 n3 *    |
|>END TALLY          |
|                    |
```

`SELECT` posts `/api/tally/select` and returns to TALLIES. `END TALLY` opens a CONFIRM, then posts
`/api/tally/end`. Both are app-only on the panel, with no MQTT (`api/app.py` `tally_select()` and
`tally_end()` [18]).

### 4.9 DELIVERIES

Today's and tomorrow's booked deliveries, from the panel's `/api/deliveries`, which reads the
director's loader schedule (`api/app.py` `delivery_list()` [18]). `T` in column 20 marks tomorrow.

```
|W     1834 lb  n12  |
|>SMITH #4411        |
| JONES #4415       T|
|                    |
```

OK opens CONFIRM `START SMITH #4411?`, then posts `/api/tally/start` with that `picking_id`. A
typed delivery (order reference and customer, `api/app.py` `_delivery()` [18]) needs text entry. It
stays on the phone.

### 4.10 INFO

```
|W     1834 lb  n12  |
|NFC PN532 FW 1.6    |
|PANEL OK  b 699e3c6c|
|                    |
```

Row 1 is the reader and its firmware, or `NFC NOT ANSWERING`. Row 2 is the panel's last answer and
the first 8 characters of `build` (`references/03_mqtt_contract.md:250`). A silent reader stays off
the fault row: it does not make the weight untrustworthy, and `RECORD NO TAG` still works.

### 4.11 MC — moisture entry (designed, deferred)

The panel takes `mc` on a `+` as a string of tenths in 5.0–60.0
(`api/moisture.py:49-50, 70-78`; `api/app.py:258-263` [18]). The pendant's `mc` entry is on the
unmerged mesh branch `feat/tdeck-moisture` [14]. If that lands and Alex wants it on the board, the
screen opens from TAG with DOWN held. UP and DOWN step 0.5, and OK returns to TAG with `mc` set.
**It is not in the first build.**

```
|W     1834 lb  n12  |
|MC 18.5 %    OK=SET |
|TAG 64:72:07:CA     |
|                    |
```

## 5. The data path

### 5.1 The choice: the board is a client of the panel's `/api/cmd`, as the pendant is

The gleipnir coordinator's key point, relayed 2026-10-29 and checked here: a tag reaches the
record only in the `+` body sent to the PLC #4 panel's `/api/cmd`. That is where it is:

- validated: `valid_pendant_tag()`, `api/commands.py:306-313` [18];
- pinned to the command id: `simple_tally.py` `_check_tag()` [18];
- published beside the `+` on `plc4/jd5055e/cmd`: `api/app.py:369` [18].

The consumer stores it from the command topic into `plc4_cmd.tag` and joins it to the record
through `cmd_id = plc4_ack.cmd_id` (migration 129 [20]; `consumer/plc1_consumer.py:4716-4730`).
Today's board ignores the key: it reads only `id`, `verb` and `arg` (`src:1141, 1158, 1186`).

So **the board sends the same HTTP request the pendant sends**:

```
POST http://192.168.10.99:8081/api/cmd
{"id":"p4-0000002a-7","verb":"+","into":"daily_log","tag":"647207CA"}
```

This is the pendant's `cmdBody()` shape (mesh `Plc4Operator.cpp:355-372` [14]) and the panel
README's RECORD (`README.md:330-338` [18]). The URL is the pendant's default `PLC4_PWA_URL`
(mesh `Plc4Operator.h:32-34` [14]), the same host as the board's broker (`BRIEF.md:197`).

| Step | Who | What |
|---|---|---|
| 1 | Board | OK on TAG. GET `/api/state` for the destination list and the DUP WARN check |
| 2 | Board | For a delivery destination: POST `/api/tally/select`. Then POST `/api/cmd` without `into`. That is the panel README's "delivery RECORD after a tally select" (`README.md:333-337` [18]). For the daily log, POST `/api/cmd` with `"into":"daily_log"` |
| 3 | Panel | Validates, pins `tag` (and `mc`) to the id, picks the tally, publishes the `+` on `plc4/jd5055e/cmd`, and answers 202 (`api/app.py:221-380` [18]) |
| 4 | Board | Receives its own command over MQTT, runs it through the one handler (`src:1902-1932`), and acks it as it acks any command |
| 5 | Board | Matches the id it minted and shows RECORDED or NOT RECORDED from that verdict. It does not wait on the panel |
| 6 | Consumer | Stores the command with `tag`, and the ack. Both are unchanged |
| 7 | Panel | Credits the record from the ack and builds the bag reading: command id, tag, weight, mc, grade, `picking_id`. **Pending gleipnir** for the field list and the Director and Odoo leg (§7) |

**Why this route:**

- **No other repo has to change for scan → record.** The panel, the consumer and the bag reading
  already handle a pendant `+` with a tag. A board `+` with a tag is the same request from a
  different client.
- **The tag pin, the 409 on a changed tag, the dup warning and the bag reading all see the board's
  tag**, because it enters at the one door gleipnir names.
- **The board needs no MQTT contract change for it.** The `+` the board receives is today's `+`
  with keys it already ignores.

**Consequences:**

- **The board gains an HTTP client.** Each call blocks `loop()`: the sampler, the LCD and the CAN
  window all wait. The TCP connect cannot be bounded on this core: `setConnectionTimeout()` breaks
  the connect (`src:1537-1551`). The heartbeat watchdog covers a hung call (`src:221`), as it covers
  MQTT. On the LAN, to the broker's own host, a call should be tens of milliseconds (inference).
  That is NOT DETERMINABLE until timed on the board.
- **A second socket on the WiFi coprocessor** beside the MQTT one. Whether WiFiS3 holds both is
  NOT DETERMINABLE here (inference: the modem indexes sockets). A bench run answers it.
- **`/api/state` is parsed by a streaming key scan into fixed buffers**, not into a JSON tree:
  32 KB of RAM, about 15 KB free on 2026-10-07 (`notes/2026-10-07-rfid-record-contract.md:368-371`).
  Its size on the live panel is NOT DETERMINABLE here. Keep at most 6 tallies, 6 deliveries and
  `last_record`. The board fetches it only on entering TAG, TALLIES, DELIVERIES or a RECORD, never
  on a timer.
- **A lost ack still loses the credit.** The ack is QoS 0, and the panel credits from it
  (`api/app.py:59-61` [18]). The board shows RECORDED from its own handler, and the panel may never
  credit it. The pendant path has the same property.
- **Nothing records when the panel does not answer** (NO PANEL). The console still records as
  today.

### 5.2 Command ids the board mints

Form: `p4-<session, 8 hex>-<n>`, for example `p4-0000002a-7`. It is at most 22 characters, inside
the panel's 40-character safe set (`api/commands.py:138`; `references/03_mqtt_contract.md:365`).

- `p4-` marks the origin in `plc4_cmd`. The pendant uses `hh-<node>-<boot>-<n>` (mesh
  `Plc4Operator.cpp`, `mintId()` [14]). Nothing downstream reads either prefix: a search of the
  consumer and the panel for `hh-` found only fixtures.
- `<session>` is an EEPROM counter, **incremented once per boot, at the first board-originated
  command after that boot**, not at boot itself. A board in a reset loop writes nothing, which was
  the objection that kept a boot counter out of the firmware (`src:470-472`). Wear is one write per
  boot in which a button was used, against 100,000 data-flash program/erase cycles [8].
- `<n>` counts from 1 within the session, in RAM.
- **Why it must be unique:** the panel pins tag and mc per id and answers 409 to a different value
  under the same id (`simple_tally.py` [18]). The consumer joins on `cmd_id`. The bag reading's
  `source_key` is the id (**pending gleipnir**).

### 5.3 What changes in the record contract

**On the wire, today's firmware path: nothing.** Commands, acks, `weight` and `telemetry` keep
their shapes (`references/03_mqtt_contract.md`). Board-originated commands appear on
`plc4/jd5055e/cmd` as the panel publishes them, with `p4-` ids. One addition is proposed, FW-7:
`nfc` on `status`.

**The 2026-10-07 RFID-build contract** (`notes/2026-10-07-rfid-record-contract.md`, approved for
consumer hand-off, firmware gated) is untouched, and this design does not implement it. Note one
consequence for anyone who later points it at this reader. Its tag is an EPC in whole 16-bit words,
4–32 hex characters (`valid_epc()`, `api/commands.py:183-185` [18]). A 7-byte NFC UID is 14 hex
characters, which is not a multiple of 4. That contract's tag rule would have to widen to the
pendant rule before an NFC UID could ride it.

### 5.4 The other verbs

TARE, UNDO LAST and CLEAR BOARD go through `/api/cmd` as `tare`, `-` and `b`. So does the
pendant's (mesh `Plc4Operator.cpp:28` [14]). That keeps one audit trail in `plc4_cmd` and the
panel's state gate ahead of the board's.

**Exception: TARE with no panel.** When `/api/cmd` does not answer, TARE runs locally through the
same handler, as the console's does (`src:1885-1898`). It changes no record. UNDO and CLEAR never
run locally: the panel tracks the board's records, and a local undo would leave it counting a bag
the board had removed.

### 5.5 Which repos would have to agree

| Repo | Owner | What it would need |
|---|---|---|
| greybeard-plc4-pwa | PLC #4 PWA coordinator | **No API change.** It must accept the board as a third client of `/api/cmd`, `/api/state`, `/api/tally/*` and `/api/deliveries`, with `p4-` ids. Separately, the **V2 case**: PN532 bosses, four 16.2 mm holes and the M7E bosses removed (§2.5, §3.3). That is `design/case-print/`, carried by the case layout worker |
| greybeard-plc-consumer | Consumer coordinator | **No code change.** Migration 129's comment that PLC #4 will never carry RFID is superseded (§1). Restating it is theirs to do |
| mesh | Mesh coordinator | **No change.** It must agree that the tag rules in §5.6 are shared, so a change on either side is a change on both |
| greybeard-gleipnir, Director, Odoo | gleipnir coordinator | **No change**, if a board `+` with a tag produces a bag reading exactly as a pendant `+` does. **Pending gleipnir** |

### 5.6 Consistency with the pendant

| Rule | Pendant | Board | Source |
|---|---|---|---|
| Reader | PN532 (Elechouse V3) | Same part | mesh `Pn532.h:4-6` [14]; §2.1 |
| Bus | HSU on the T-Deck (RX44/TX43); I2C at 0x24 on the Heltec | I2C at 0x24 on `Wire` | mesh `mesh_pendant_tdeck/platformio.ini:17-26`, `Pn532.h:20-21` [14] |
| Poll | InListPassiveTarget, one target, 106 kbps type A, about 10/s. `MxRtyPassiveActivation` = 1 | Same | mesh `Pn532.h:23-29` [14] |
| Wire form | Upper-case hex, no separator, an even count of 2–20 digits (`647207CA`) | Same | mesh `validTag()` [14]; `api/commands.py:306-313` [18] |
| Screen form | `64:72:07:CA` | Same where it fits in 20 columns (§4.3) | mesh `RfidTag.h` `hexShown()` [15] |
| Field name | `tag`, on `+` only | Same | `api/app.py:298-309` [18] |
| New scan, expiry, use | `kGoneMs` 2000, `kArmedMs` 120000, used up on a recorded RECORD | Same | mesh `RfidTag.h` [15] |
| RECORD | `+` to `/api/cmd`, `into` for the daily log, a tally select first for a delivery | Same | mesh `cmdBody()` [14]; `README.md:330-338` [18] |
| Dup warning | `last_record` under 15 min and not emptied | Same | mesh `feat/dup-record-warning` merge [14] |
| `mc` | Unmerged branch | Deferred (§4.11) | [14] |

## 6. Firmware changes proposed (each needs Alex's approval)

None of these is approved. Each is a separate gate.

| # | Change | Touches |
|---|---|---|
| FW-1 | **Reader driver on `Wire`:** step-based, never waits, constants as §5.6. Write it fresh from NXP UM0701-02 [3] rather than copying the mesh driver, which is under mesh's GPL-3.0 (`firmware/LICENSE` [14]). This repo states no licence (`references/01_controller_card.md:40`), and copying would decide one. **Consequence:** two drivers that must stay in agreement on §5.6, which a host test pins | new header in `include/`, `setup()`, `loop()` |
| FW-2 | **Buttons:** D2-D5 inputs, the 200 Hz debounce timer, the press queue, `KEY STUCK` on the fault row (§3.5) | new header, `setup()`, `hmiRenderFaultRow()` (`src:1658-1670`) |
| FW-3 | **Screens:** the §4 state machine. Row 0 and row 3 keep today's code. Rows 1 and 2 belong to the screen | `hmiPoll()`, `hmiDrawHome()` (`src:1672-1721`) |
| FW-4 | **Panel client:** GET `/api/state` (streamed key scan), POST `/api/cmd`, `/api/tally/select`, `/api/tally/start` and `/api/tally/end`, GET `/api/deliveries`. The URL is a `#define` beside the network settings, default `http://192.168.10.99:8081` (mesh `Plc4Operator.h:32-34` [14]) | new header; WiFiS3 `WiFiClient` [9] |
| FW-5 | **Id minting:** `p4-<session>-<n>`, with the lazily incremented EEPROM session counter (§5.2) | EEPROM map, a free slot chosen in the change |
| FW-6 | **Local verdict:** when the handler runs a command whose id the board minted, the screen takes its verdict. Otherwise NO ANSWER after 10 s | the ack block (`src:2216-2245`) |
| FW-7 | **Diagnostics:** `nfc` on `status` (`"ok"` with firmware `"1.6"`, or `"none"`); an `nfc` console verb, refused over MQTT like `i2c` (`src:769-773`); `i2c` also scans `Wire` | `publishStatus()`, the verb chain, `i2cScan()` (`src:1424-1450`) |
| FW-8 | **MC entry** (§4.11). Deferred | screens, FW-4's POST body |

RAM and flash cost: NOT DETERMINABLE until FW-1 to FW-6 are built. A `tools/build-check.sh` run on
the change answers it (`notes/2026-10-07-rfid-record-contract.md:368-371`).

## 7. Pending gleipnir

Relayed 2026-10-29 from mesh `main` and branch `feat/tdeck-moisture`. **Checked here against source,
read-only:**

- the tag wire rule;
- `cmdBody()` with `into` and `tag`;
- `/api/cmd` answering 202 on publish;
- the per-id tag and mc pin;
- `mc` as a string of 5.0–60.0, taken on `+` only;
- the consumer storing `tag` from the command topic;
- the panel's bag-readings merge being on its saga `main`;
- the PN532 at I2C 0x24 and HSU RX44/TX43;
- `mc` in `cmdBody()` on the moisture branch.

One correction: the reader thread is mesh `firmware/src/modules/plc/RfidReader.cpp`, not
`.../core/RfidReader.cpp`.

**Not checked; pending:**

- the bag reading's fields;
- the Director `POST /scheduler/api/bag-readings` leg and the Odoo `phoenix_bag_trace` leg;
- the statement that tags are reused (Alex, 2026-10-29, relayed) and that nothing looks a reading up
  by tag;
- the Heltec pendant's GPIO4/3 pins;
- whether the panel build with the pendant tag and the bag readings is the one running on farm1.

## 8. NOT DETERMINABLE, and what answers each

| Item | What answers it |
|---|---|
| The 5 V rail's spare current for 0.2 A (§2.4) | The ISL854102 datasheet and the supply Alex fits |
| Pull-ups on the module or the board's `Wire` lines (§2.2) | The Elechouse V3 schematic |
| Read distance through the lid with the bezels fitted and the UNO behind (§2.5) | A read with the case assembled |
| The Elechouse V3 mounting holes (§2.5) | Measuring the board |
| Maximum panel thickness, temperature range and bounce of COM-11966 (§3.2) | A sample on the 4 mm lid; the maker's full datasheet, which [6] is not |
| The printed case's ingress rating (§3.2) | A spray test of the assembled case |
| A free `FspTimer` for the debounce (§3.5) | A build and boot on the board |
| HTTP call time, and two sockets on WiFiS3 (§5.1) | Timing the calls on the board |
| `/api/state` size on the live panel (§5.1) | One GET against farm1 |
| RAM and flash of FW-1 to FW-6 (§6) | `tools/build-check.sh` on the change |

## 9. What this design does not do

- It does not read a tag on a bag on the loader. The PN532 reads at 5–7 cm [1].
- It does not read UHF tags.
- It does not take the HC-05 off D0/D1, and it does not touch the Qwiic trunk.
- It does not put `kv`, `c`, `w`, `sm` or a typed delivery on the buttons.
- It does not light the LED rings.
- It does not record without the panel.
- It does not implement the 2026-10-07 RFID-build contract.

## 10. Change from today, in one line each

- **Hardware:** a PN532 V3 on `Wire` behind the lid's top-left, and four IP65 buttons in a 2 × 2
  cluster on D2-D5. HC-05, CAN and Qwiic are unchanged.
- **Screen:** row 0 and row 3 unchanged. Rows 1 and 2 become a menu driven by four buttons.
- **Record:** a board RECORD is a pendant RECORD from a different client. The tag reaches the
  record through `/api/cmd`, as the pendant's does.

## 11. References

[1] Elechouse, "PN532 NFC RFID Module User Guide," Version 3, Rev. B. [Online]. Available:
https://www.elechouse.com/elechouse/images/product/PN532_module_V3/PN532_%20Manual_V3.pdf

[2] NXP Semiconductors, "PN532/C1 Near Field Communication (NFC) controller," product data sheet,
Rev. 3.6, doc. 115436. [Online]. Available: https://www.nxp.com/docs/en/nxp/data-sheets/PN532_C1.pdf

[3] NXP Semiconductors, "PN532 User Manual," UM0701-02, Rev. 02. [Online]. Available:
https://www.nxp.com/docs/en/user-guide/141520.pdf

[4] Arduino, "Arduino UNO R4 WiFi," datasheet, SKU ABX00087, §2, §10, §11, §12. Filed in
`docs/vendor/arduino/ABX00087_UNO-R4-WiFi_datasheet.pdf`

[5] SparkFun Electronics, "Metal Pushbutton with Wires — Momentary (16mm, Red)," product page,
COM-11966. [Online]. Available: https://www.sparkfun.com/products/11966

[6] SparkFun Electronics, "COM-11966 through COM-11976," mechanical drawing and description.
[Online]. Available: http://cdn.sparkfun.com/datasheets/Components/Switches/com-11966.pdf

[7] Arduino, *ArduinoCore-renesas*, `variants/UNOWIFIR4/pins_arduino.h`, lines 104–140, as
installed with `arduino:renesas_uno` 1.6.0. License: LGPL-2.1. [Online]. Available:
https://github.com/arduino/ArduinoCore-renesas

[8] Renesas Electronics, "RA4M1 Group Datasheet," R01DS0355EJ0110: input pull-up resistance RU and
data-flash reprogramming/erasure cycles NDPEC. Filed in
`docs/vendor/renesas/RA4M1_Group_Datasheet_R01DS0355EJ0110.pdf`

[9] Arduino, *ArduinoCore-renesas*, `libraries/WiFiS3/src/WiFiClient.h`, line 229, and
`WiFi.h`, line 447, as installed with `arduino:renesas_uno` 1.6.0. License: LGPL-2.1. [Online].
Available: https://github.com/arduino/ArduinoCore-renesas

[10] A. Pennington, *greybeard-plc4-tractor-scale*, `src/ArduinoHydraulicScale.cpp`,
`include/i2c_mux.h`, `include/lcd2004.h`, `include/j1939_can.h`, at `main` 15ca6ee. License: none
stated. [Online]. Available: https://github.com/Alex-Pennington/greybeard-plc4-tractor-scale

[11] A. Pennington, *greybeard-plc4-tractor-scale*, `notes/2026-10-25-rfid-read-location-ruling.md`,
`notes/2026-10-25-rfid-pendant-rulings.md`, `notes/2026-10-26-rfid-pendant-handoff.md`,
`notes/2026-10-07-rfid-record-contract.md`, `notes/rfid-inventory-discussion.md`,
`notes/case-v2-log.md`, `notes/2026-10-25-rfid-case-print.md`, at `main` 15ca6ee. License: none
stated.

[12] A. Pennington, *greybeard-plc4-tractor-scale*, `references/01_controller_card.md`,
`01A_radio_module.md`, `01C_qwiic_stack.md`, `01D_display.md`, `03_mqtt_contract.md`,
`06_operating_card.md`, at `main` 15ca6ee. License: none stated.

[13] A. Pennington, *greybeard-plc4-tractor-scale*, `spikes/tdeck-nfc/README.md`, at `main`
15ca6ee. License: none stated.

[14] A. Pennington, *mesh*, `firmware/src/modules/plc/core/Plc4Operator.cpp`, `Plc4Operator.h`,
`Pn532.h`, `firmware/src/modules/plc/RfidReader.cpp`,
`firmware/variants/esp32s3/mesh_pendant_tdeck/platformio.ini`, `firmware/LICENSE`, at `main`
c8783cfc8 (the `feat/dup-record-warning` merge), and branch `feat/tdeck-moisture` at 6c98fdb84.
License: GPL-3.0 (`firmware/LICENSE`). Saga: `/opt/repos/mesh.git`.

[15] A. Pennington, *mesh*, `firmware/src/modules/plc/core/RfidTag.h`, at `main` c8783cfc8.
License: GPL-3.0. Saga: `/opt/repos/mesh.git`.

[16] Adafruit Industries, *Adafruit-PN532*, `Adafruit_PN532.cpp` and `Adafruit_PN532.h`, version
1.3.4. License: BSD. [Online]. Available: https://github.com/adafruit/Adafruit-PN532

[17] A. Pennington, *greybeard-plc4-pwa*, `design/case-print/case_params.py`, at `main` 70c1fa7.
License: none stated. Saga: `/opt/repos/greybeard-plc4-pwa.git`.

[18] A. Pennington, *greybeard-plc4-pwa*, `api/app.py`, `api/commands.py`, `api/simple_tally.py`,
`api/moisture.py`, `README.md`, at saga `main` (contains df172d2, the bag-readings merge). License:
none stated. Saga: `/opt/repos/greybeard-plc4-pwa.git`.

[19] A. Pennington, *greybeard-plc-consumer*, `consumer/plc1_consumer.py`, lines 3516 and
4716–4767, at `master` 48379d8. License: none stated.

[20] A. Pennington, *greybeard-plc-consumer*, `db/migrations/129_plc4_tag_from_pendant.sql`, lines
1–9, at `master` 48379d8. License: none stated.
