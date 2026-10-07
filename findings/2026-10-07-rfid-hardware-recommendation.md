# RFID tally system: hardware recommendation for PLC #4

**Date:** 2026-10-07. **Scope:** research and a recommendation only. Nothing here is approved for
purchase, wiring or firmware. Settled design: `notes/rfid-inventory-discussion.md`. This document
does not reopen anything recorded there as decided.

**Sizing rule:** this is a firewood business, not NORAD (Alex, 2026-10-07). The aim is robust and
simple: no signing, no key management, no security hardware, nothing mil-spec.

**How to read the claims.** A claim with a bracketed number is taken from the document listed
under *References*. A claim with `file:line` is read from this repository. **Inference** marks
a conclusion drawn from cited facts but not stated in any source. **NOT DETERMINABLE** marks
something no available source settles. Nothing here was measured on hardware. The only
measurement is the firmware build in §1.1.

---

## Summary

| Item | Recommendation | Approx. price |
|---|---|---|
| Platform | **Keep the Arduino UNO R4 WiFi (ABX00087), with a new board, and spend the robustness budget on power, enclosure and wiring** | $27.50 |
| Reader | **SparkFun Simultaneous RFID Reader – M7E Hecto (WRL-24738)** on D0/D1, through the level interface in §2.2 | $309.95 |
| Level interface | **TI SN74AHCT125N** (reader TX → board RX) plus a resistor divider (board TX → reader RX) | not established |
| Antenna | **Vulcan RFID C610 (VUL-C610-FCC)**: IP67, circular, 6 dBiC, waterproof SMA | $149.00 |
| Antenna feed | **Taoglas CAB.011** U.FL-compatible to SMA(F) bulkhead, then SMA–SMA 195-series coax | CAB.011 not established; cable $57 (12 ft) |
| Tags | **HID Sentry Duo, US (AST1-160-0300)**: IP68, zip-tie slot, reusable | $1.86 each, box of 150 |
| Board supply | **Mean Well DDR-30G-12** (9–36 V in, reverse-polarity protected) behind a fuse and a load-dump TVS | not established |
| Reader supply | The existing off-board 5 V supply, as Alex stated, **only if it meets the M7E's ripple limits**. Otherwise a **Pololu D36V28F5** | $23.95 if needed |
| Enclosure | **Hammond 1554 series**, polycarbonate, cUL Type 4, 4X, 6, 6P, sized to the layout | not established |

**Priced subtotal:** $567.40 for the board, reader, antenna, coax and the Pololu regulator.
Adding one box of 150 tags ($279.00) brings it to **$846.40**. The DC-DC converter, enclosure,
pigtail, glands, buffer, TVS and fuse are not priced here (§7). **Inference:** those are a small
fraction of the reader's price.

---

## 1. Platform

### 1.1 What the firmware depends on today

These are read from source. Every platform-specific item below has to be ported on any board
other than an RA4M1 Arduino.

| Function | How it is done now | Where |
|---|---|---|
| 4–20 mA loop | MCP3421 over I2C on `Wire1` (Qwiic), behind a TCA9548A/PCA9548A mux at `0x70`, channel 3 | `include/mcp3421.h:8`, `:21-22`; `include/i2c_mux.h:16-21` |
| LCD | LCD2004A behind a PCF8574 backpack at `0x27`, mux channel 7. The backpack is typically a 5 V part on a 3.3 V bus | `include/lcd2004.h:6`, `:10-13` |
| Power monitor | INA219 at `0x40`, mux channel 2 | `include/ina219.h:7`; `references/01_controller_card.md:39` |
| J1939 | RA4M1 on-chip CAN through `Arduino_CAN`, drained from an `FspTimer` ISR. Receive on D13; the transceiver's D pin is tied recessive so the node cannot transmit | `include/j1939_can.h:23-30`; `references/01_controller_card.md:26-27`, `:53-62` |
| MQTT over WiFi | `WiFiS3` (the R4's ESP32-S3 bridge) and `ArduinoMqttClient` | `src/ArduinoHydraulicScale.cpp:12-13`; `platformio.ini:643-650` |
| Watchdog | `WDT.h`, armed with `WDT.begin(WDT_TIMEOUT_MS)` | `src/ArduinoHydraulicScale.cpp:14`, `:1563` |
| Reset cause | Reads RA4M1 `RSTSR0`/`RSTSR1` directly | `src/ArduinoHydraulicScale.cpp:350-382` |
| Calibration store | `EEPROM.get`/`put` (virtual EEPROM on data flash) | `src/ArduinoHydraulicScale.cpp:1490-1496`, `:1623-1647` |
| UART | `Serial1` on D0/D1 at 9600 baud, today the HC-05 | `src/ArduinoHydraulicScale.cpp:1460`; `references/01_controller_card.md:24-25` |

**Headroom, measured.** `tools/build-check.sh` on this branch's base, run with `TMPDIR` pointed at
a private scratch directory, compiles to **104,328 bytes of flash (39%)** and **17,792 bytes of
RAM (54%), leaving 14,976 bytes**. The SparkFun reader library allocates a 255-byte message
buffer [11]. **Inference:** the R4 has room for the reader driver and the record fields the
notes describe.

### 1.2 The options compared

| | **(a) UNO R4 WiFi** ABX00087 | **(b) Opta WiFi** AFX00002 | **(c) Portenta Machine Control** AKX00032 | **(d) ESP32-S3 industrial**: Waveshare ESP32-S3-RS485-CAN |
|---|---|---|---|---|
| Spare hardware UART for the reader | 1, D0/D1 (`Serial1`), freed by retiring the HC-05 [1]; core 1.6.0 `variants/UNOWIFIR4/variant.cpp:27-28` | None exposed as TTL. One half-duplex RS-485 port [3] | One port, configurable RS-232/422/485, no TTL [4] | External terminal exposes only GPIO1, GPIO2 and 3V3 [5]. The internal 2.0 mm header carries further IO [6]; which pins are free is NOT DETERMINABLE without mapping it |
| CAN for J1939 | On-chip controller; external transceiver (fitted today) [1] | **None** [3] | On-board TJA1049T/3J, rated for a 12 V or 24 V bus [4] | On-board isolated CAN with TVS protection [5] |
| I2C (MCP3421, mux, LCD, INA219) | 2 buses, one on Qwiic [1] | **None exposed.** The expansion port is proprietary [3] | Grove connector, 3.4 V max, 10 kΩ pull-ups [4] | Shares the two exposed GPIOs [5] |
| WiFi for MQTT | ESP32-S3-MINI-1 [1] | 802.11 b/g/n [3] | Via the Portenta H7 (included); SMA antenna connector [4] | ESP32-S3 [5] |
| Supply | VIN 6–24 V [1] | 12–24 VDC [3] | **24 V DC ±20 %** (19.2–28.8 V) [4] | 7–36 V DC [5] |
| On 12 V tractor power | In range | In range | **Out of range. Needs a 12→24 V boost** (inference from the range) | In range |
| Operating temperature | −40 to +85 °C [1] | −20 to +50 °C [3] | −40 to +85 °C, "conservative thermal limits" [4] | NOT DETERMINABLE (not published [5]) |
| Ingress / vibration | None. Bare PCB (inference) | IP20 [3] | Not stated; DIN-rail module [4] | Rail-mount ABS case; IP and vibration not stated [5] |
| Logic level at the reader | 5 V. Needs translation both ways (§2.2) | n/a | n/a | 3.3 V. Matches the M7E directly (inference from [5], [8]) |
| Price | $27.50 [7] | $199.00 [7] | $300.00, Portenta H7 included [7] | $18.99–$19.99 listed [5] |

### 1.3 What carries over, and what must be ported

| Function | (a) R4 WiFi | (b) Opta WiFi | (c) Machine Control | (d) ESP32-S3 |
|---|---|---|---|---|
| 4–20 mA via MCP3421 | Carries over | **Lost.** No I2C. Analog inputs are 0–10 V [3] | Replaced by the on-board 4–20 mA input (120 Ω burden, internal 12–16-bit ADC) [4]. Burden, scaling, NE43 banding and calibration are all re-derived | Ports (I2C exists); driver re-tested |
| J1939 CAN | Carries over | **Lost.** No CAN | Port from `Arduino_CAN`/`FspTimer` to the H7 core's CAN | Port to the ESP32 TWAI driver. The hardware "cannot transmit" guarantee (`j1939_can.h:23-26`) relies on tying the transceiver's TX pin, which an on-board isolated transceiver may not allow. **Inference:** silence becomes a software listen-only setting |
| MQTT over WiFi | Carries over | Port (`WiFiS3` → the Opta's WiFi stack) | Port (`WiFiS3` → the Portenta WiFi stack) | Port (`WiFiS3` → ESP32 `WiFi`) |
| LCD | Carries over | **Lost.** No I2C | Ports. The backpack must run on a 3.4 V-max connector (`lcd2004.h:10-13`; [4]) | Ports |
| Watchdog / reset cause | Carries over | Rewrite for STM32H747 | Rewrite for STM32H747 | Rewrite for ESP32 |
| Calibration store | Carries over | Rewrite | Rewrite | Rewrite |
| Reader UART | `Serial1` re-used at the reader's baud rate | Needs an RS-485 reader or a TTL↔RS-485 converter | Needs a TTL↔RS-232/485 converter at the reader | Ports only if the internal header has a free UART |

### 1.4 Recommendation: stay on the UNO R4 WiFi

**Buy a new UNO R4 WiFi for the RFID build. Put the robustness money into what surrounds it:
the 12 V front end, a sealed enclosure, glands and a proper bulkhead.**

The reason is short. It is the only option on which every function in §1.1 carries over
unchanged. The reader needs a three-part interface, not a port. The Opta lacks both CAN and I2C,
so it loses the J1939 feed, the converter and the display. The Machine Control is the strongest
industrial unit, but it needs 24 V on a 12 V tractor and an RS-232 adapter at the reader. It
would also mean porting and re-calibrating firmware that is in service and proven. The Waveshare
ESP32 board publishes no temperature rating and shows too few pins to be judged.

**What would change this:** "a more robust platform" (Alex, 2026-10-07) is not tied to any
failure recorded in this repository. A search of `findings/`, `notes/` and `references/` finds no
failure of the board in service. The one unexplained reset on record happened on the bench at
first WiFi association. Its two candidate causes were a timing tolerance and bench USB power
(`findings/2026-09-01-post-flash-review.md:266-275`). Neither is a board fault. If Alex has a specific failure in mind, such as
connectors shaking loose, heat, or a board that died, the Machine Control is the step up, at about $300 plus a boost converter and a
firmware port. See §7.

---

## 2. UHF reader

### 2.1 SparkFun M7E Hecto (WRL-24738) against the requirement

| Requirement | M7E Hecto | Source |
|---|---|---|
| EPC Gen2 | EPCglobal Gen2V2 (ISO 18000-63); Impinj E310 core | [8] spec table |
| Under ~1 m, one bag per lift | RF power 0 to +27 dBm in 0.5 dB steps, read and write set separately. About 1–2 ft on the board's own antenna, up to 16 ft with an external one | [8] §5.2.1; [9] |
| US band | NA region: 902.750–927.250 MHz, 50 channels; "FCC (NA, SA) 902-928MHz" | [8] Table 9, spec table |
| UART | TTL UART, 9600–921600 baud, **default 115200**. No hardware flow control. The host must accept up to 255 bytes in one burst | [8] §5.1.4 |
| UART logic level | Module: **3.3 V CMOS.** RX VIH min 1.9 V, **max 3.7 V while powered**, and no more than V3R3 + 0.3 V while off. RX has an internal 49.9 kΩ pull-up. TX VOH **2.7 V min**, 3.0 V typ | [8] Table 2, §5.1.4.1 |
| Board header level | "0.1"-Spaced PTH Header (**3.3V logic**)". The schematic shows TX/RX going straight from the module to header J2 through a PRTR5V0U2F ESD clamp only, with **no level shifting** | [9]; [10] |
| Current | DC input 3.3–5.5 V. Internal limit at **1 A**. Full power 0.665 W at 5 V; up to 4 W into a poor antenna match at high temperature | [8] §5.2, §5.2.1, §5.2.4 |
| Supply quality | **5 V ±5 %, < 25 mV p-p ripple, < 11 mV p-p below 100 kHz, switching frequency ≥ 500 kHz** | [8] §5.2.2 |
| Temperature | −40 to +60 °C operating (case). Transmission stops at +85 °C on the internal sensor | [8] §5.4.1, §5.4.2.3 |
| Antenna port | U.FL on the SparkFun board. **External use requires moving a 0 Ω resistor from R4 to R5** ("Move 0 Ohm resistor to switch between internal and external antennas") | [9]; [10] |
| Control | RUN/ENABLE: high or open = run; low for 50 ms resets the module. [8] recommends driving it from a host GPIO | [8] §5.1.6; [10] header J1 |
| Arduino driver | SparkFun library v1.2.0, MIT. Takes any `Stream`, supports `ThingMagic_M7E_HECTO`, `REGION_NORTHAMERICA` and `getTagRSSI()` | [11] |
| RF exposure | Keep people at least 21 cm from the antenna face (FCC long-term limit) | [9] |

### 2.2 The UART level problem on the R4, and the fix

**The R4's D0/D1 are 5 V and the reader is 3.3 V. Neither direction works wired straight
through.**

- **Board TX → reader RX.** D1 is RA4M1 port P302 (`TXD2`), driving to 5 V
  (`~/.arduino15/packages/arduino/hardware/renesas_uno/1.6.0/variants/UNOWIFIR4/variant.cpp:28`;
  [2] pin function table). The reader's RX allows 3.7 V
  max while powered [8]. **Over the absolute limit.**
- **Reader TX → board RX.** D0 is P301 (`RXD2`) (`variant.cpp:27`). It is not on the 5 V-tolerant
  list and needs **VIH ≥ VCC × 0.8 = 4.0 V** [2] Table 2.4, the same reading as
  `notes/2026-09-20-can-level-interface-plan.md:66-72`. The reader guarantees only 2.7 V [8].
  **Not a valid logic high.**

**Fix, three parts:**

1. **Reader TX → D0: one gate of a TI SN74AHCT125N** (PDIP-14) on the board's 5 V. Its TTL input
   has VIH min **2.0 V** [24]. That accepts the reader's 2.7 V minimum. Its output VOH is
   **≥ 4.4 V at −50 µA** and ≥ 3.8 V at −8 mA (VCC 4.5 V) [24], which clears the R4's 4.0 V. Tie
   the unused gates' inputs to a rail [24].
2. **D1 → reader RX: a resistor divider** to about 3.0 V. **Inference:** 2.2 kΩ over 3.3 kΩ gives
   3.0 V from 5 V. That sits inside the 1.9–3.7 V window, and the module's 49.9 kΩ pull-up
   barely moves it.
3. **ENABLE ← a spare GPIO**, such as D2. The controller card says no header pin beyond D0, D1, D10
   and D13 is used (`references/01_controller_card.md:22-31`), through the same kind of divider. This lets the
   board reset a hung reader, as [8] §5.1.6 recommends.

**Power sequencing (inference from [8] Table 2).** With the reader unpowered, its RX must not sit
more than 0.3 V above its own rail. Power the reader whenever the board is up, or hold D1 low
until it is.

**Rejected:** TXB0104. Its datasheet requires any pull-up to be above 50 kΩ [25] §7.3.5, and the
M7E's internal RX pull-up is 49.9 kΩ. It also caps the load at 70 pF, which a harness run to the
reader would exceed. A BSS138 two-FET shifter is also rejected: it is open-drain with an RC
rising edge. `notes/2026-09-20-can-level-interface-plan.md:230-238` already sets that topology
aside for push-pull signals.

**Not needed on a 3.3 V platform** (option d). This is the one point in that option's favour.

### 2.3 Alternative reader

- **CAEN RFID Lepton3x1 (R3101C)** [13]. RAIN Gen2, FCC 902–928 MHz with 50 hopping channels,
  0–25 dBm. UART at 9.6–921.6 kbps (default 921.6 kbps), **3.3 V I/O**. Supply 3.2–5.25 V,
  **800 mA at 5 V at 25 dBm**, −20 to +70 °C, MMCX antenna jack. It needs the same level
  interface on the R4. There is no Arduino library for it; the driver would be written from
  CAEN's protocol documentation, which was not read here. Price: NOT DETERMINABLE (distributor
  quote).
- **SparkFun M6E Nano (SEN-14066) is not an alternative.** SparkFun lists it as retired, with the
  M7E Hecto as its replacement [12].

---

## 3. Antenna

### 3.1 Recommendation: Vulcan RFID C610 (VUL-C610-FCC), $149.00 [15], [16]

| Property | Value [15] |
|---|---|
| Band | FCC 902–928 MHz |
| Polarisation | Circular. Reads tags in any orientation, which suits a bag that arrives at any angle |
| Gain | 6 dBiC |
| Beamwidth | 70° elevation, 70° azimuth |
| Front-to-back | 20 dB |
| Axial ratio | 1.2 dB |
| VSWR | < 1.1 |
| Connector | **SMA female, waterproof** |
| Rating | **IP67**, UV-resistant ASA radome, DC-grounded |
| Temperature | −40 to +70 °C operating |
| Size / mount | 250 × 250 × 20 mm, 0.69 kg, 100 × 100 mm M4 VESA studs |

**Why this one for about 1 m.** The risk at 1 m is reading too much, not too little: a second bag
in the field forces a multi-tag refusal (`notes/rfid-inventory-discussion.md:25`). The C610's
narrow 70° beam and 20 dB front-to-back reject tags beside and behind it better than the compact
alternative below. **Inference:** set RF power well below +27 dBm and tune it at the machine. The
reader allows that in 0.5 dB steps [8].

**FCC modular grant (inference).** [8] §5.7 lists a circular patch (MTI-242043, maximum linear
gain 6.0 dBiL; sold as MT-242043/TRH/A/K, IP67, 7.5–9.5 dBic [14]) as an authorised antenna.
It states that circular gain runs 3 dB above the linear figure. On that relation the C610's 6 dBiC is about 3 dBiL: same type, lower gain. That is the
case [8] §5.8 says needs no further testing. Whether the grant carries over to SparkFun's carrier
board, as distinct from the bare module, is NOT DETERMINABLE.

### 3.2 Alternative: Vulcan RFID S9025PL (VUL-S9025PLNF), $199.00 [17], [16]

FCC 902–928 MHz, LHCP, 5.5 dBi, 100° beamwidth, front-to-back 8 dB. **N-type female.** 132 × 132
× 18 mm, IP67, MIL-STD-810 vibration and salt. It is half the size of the C610, but its wider
beam and weaker back rejection read more of what is around it. **Its datasheet is internally
inconsistent:** it gives operating temperature as "- 0° to +70°C (-22° to +158°F)", and −22 °F is
−30 °C. Confirm with Vulcan before relying on cold operation.

**Not recommended:** the SparkFun UHF antenna (WRL-14131). It is linear vertical, 6 dBi, with no
IP or temperature rating on its drawing [18].

### 3.3 Feed from the reader to the antenna

1. **Inside the box: Taoglas CAB.011.** IPEX MHF I (U.FL-compatible) to an SMA(F) straight
   bulkhead, 95 mm of Ø1.13 mm coax [19]. It plugs onto the SparkFun board's U.FL and passes
   through the enclosure wall. Its datasheet states no IP rating or sealing gasket. **Inference:**
   seal the bulkhead with an O-ring or a sealant bead and check it is watertight.
2. **Outside: SMA-male to SMA-male 195-series coax**, offered as a C610 accessory at 12 ft for
   $57 [16]. Use the shortest length that reaches. Cable loss figures were not read; at 1 m, loss
   is not the limit (inference).
3. **Inference:** a U.FL plug on a vibrating machine wants strain relief. Clip or tie the pigtail
   to the board so the antenna lead never pulls on it.

---

## 4. Tags

Bag material and tag placement are out of scope (`notes/rfid-inventory-discussion.md:64`). This
section names tag types and their properties only.

### 4.1 Recommendation: HID Sentry Duo, US variant (AST1-160-0300)

| Property | Value [20] |
|---|---|
| Band / chip | 902–928 MHz (FCC); Impinj M750; 96-bit EPC, 32-bit user memory |
| Read distance | 18 ft (5 m), dependent on reader and environment |
| Size | 36.3 × 10.9 × 2.8 mm, 2 g |
| Mounting | Adhesive sticker **or slot for a zip tie**; affixes to any surface |
| Water | IP68 (20 °C, 1 m, 24 h) |
| Chemicals | Dilute acids and alkalis, alcohols, petroleum, salt mist, vegetable oil, hydraulic fluid |
| Temperature | −50 to +85 °C operating |
| Impact | 18 in drop of a 1 in steel ball |
| Box | 150 pieces |
| Price | **$1.86 listed** [21]. Whether that is a unit or a volume price was not established |

**Reuse is plausible (inference).** The tag is a rigid PCB tag with a zip-tie slot, so it can be
cut free and re-tied to another bag. If tags move between bags, a tag ID identifies a bag only
until it moves. That is a consumer-side bookkeeping point, not a hardware one. UV resistance is
not stated for the Duo [20], so its life in sunlight is NOT DETERMINABLE.

### 4.2 Alternative: Xerafy TEX 55×12 Silicone (X5512-GL070-U9)

EPC Gen2 (ISO 18000-63), 860–960 MHz, NXP UCODE 9, 96-bit EPC. Up to 6 m off metal, linear
polarisation. Silicone, 55 × 12 × 2.5 mm, IP68, −40 to +85 °C. Survives 100+ commercial wash
cycles and bleach. MIL-STD-810G vibration. **Mounting: sewn** into a hem or pocket [22]. Listed at
$0.98 [23]. Choose it if the bag is fabric and a sewn tag suits. Reuse is less plausible than the
Duo's (inference: it has to be unpicked).

---

## 5. Power and enclosure

### 5.1 12 V to the board

The board is fed today from a "board supply" that the repository names only generically
(`references/02_pressure_loop_wiring.md:71-73`). Its source and protection are NOT DETERMINABLE
from here.

**Recommended front end, in order from the tractor:**

1. **Fuse** at the tap, sized to the converter's input current. Part not specified here.
2. **Load-dump TVS across the line after the fuse.** The Vishay SM8S family is built "especially
   for automotive load dump protection", meets ISO 7637-2 by test condition, and is AEC-Q101
   [26]. The datasheet read marks SM8S10A–SM8S43A **Not For New Designs**, alternative
   **SM8S10AHM3 thru SM8S85AHM3** [26]. On the SM8S22A figures it stands off 22 V and clamps at
   **35.5 V max** at rated current [26], under the converter's 36 V input ceiling. The AHM3
   datasheet was not read. Confirm the same figures before ordering. It is a DO-218AB
   surface-mount part (inference: it needs a small carrier board or a pre-built module).
3. **Mean Well DDR-30G-12** DIN-rail DC-DC [27]: input **9–36 VDC**; **reverse polarity
   protected by an internal MOSFET, no damage**, auto-recovering; 12 V at 2.5 A; −40 to +85 °C
   with derating; vibration to IEC 60068-2-6; 4 kVdc input–output isolation; 35 × 90 × 54.5 mm.
   Its regulated 12 V feeds the R4's VIN (6–24 V [1]).

**Inference:** the output is isolated, so bond its negative to the board ground. The CAN
transceiver and the 24 V loop supply already share board ground
(`references/00_architecture.md:80`).

### 5.2 5 V to the reader

The reader draws up to 1 A at 5 V [8]. **Alex states the existing off-board 5 V supply has the
capacity. That is his statement, not a measurement** (`notes/rfid-inventory-discussion.md:35-36`).

**Capacity is not the only test.** The M7E needs **< 25 mV p-p ripple and a switching frequency of
at least 500 kHz** [8] §5.2.2. The existing supply's part is not recorded, so whether it meets
that is NOT DETERMINABLE. For scale, the obvious off-the-shelf 5 V DIN module, the DDR-30G-5,
fails on paper: **60 mV p-p ripple and a 100 kHz oscillator** [27].

**If the existing supply cannot be shown to comply, fit a dedicated regulator off the protected
12 V: Pololu D36V28F5** [28]. 5 V at 3.2 A, 5.3–50 V input, reverse protection to −40 V, $23.95.
It switches at **about 500 kHz under heavy load** but drops lower at light load (power-save
mode), and its ripple is **not published** [28]. **Inference:** heavy load is when the reader
transmits, which is when the spec matters. Even so, this is a best available part, not one shown
compliant on paper.

### 5.3 Enclosure, glands and bulkhead

- **Enclosure: Hammond 1554 series, polycarbonate.** The 1554W drawing lists cUL **Type 4, 4X, 6,
  6P**, UL 508A for the polycarbonate version, a preformed silicone gasket and four cover screws
  [29]. An IP code is not on the drawing. **Inference:** Type 6P (prolonged submersion) is at
  least as demanding as IP67, but that is an equivalence, not a published rating. Choose the size
  from the panel layout. The DDR-30 stands 54.5 mm deep [27] and the 1554W's inside height is
  53.7 mm [29], so the 1554W is too shallow for it. A clear lid keeps the LCD readable; it sits behind a clear
  coverplate today (`BRIEF.md:189-191`). Whether "CL" in Hammond part numbers means a clear lid
  is inferred from the part name, not read.
- **Cable glands:** IP68 metric nylon glands sized to each cable's outside diameter, one cable
  per gland. No gland datasheet could be fetched (§7), so no part is named.
- **Antenna bulkhead:** CAB.011's SMA(F) bulkhead, sealed (§3.3).
- **Heat (inference):** the M7E is rated to +60 °C case [8] and the box is sealed outdoors. One
  read per lift is a low duty cycle, which [8] §5.4.2 says may need no heat-sinking. Keep the box
  out of direct sun where the mount allows (mounting is out of scope).

---

## 6. Bill of materials

| # | Part | Manufacturer part number | Purpose | Approx. price (USD) | Datasheet |
|---|---|---|---|---|---|
| 1 | Arduino UNO R4 WiFi | ABX00087 | Controller | 27.50 | [1] |
| 2 | SparkFun Simultaneous RFID Reader – M7E Hecto | WRL-24738 (module: JADAK ThingMagic M7E-HECTO) | UHF EPC Gen2 reader | 309.95 | [8], [9], [10] |
| 3 | TI quad buffer, TTL inputs | SN74AHCT125N | Reader TX → D0 at 5 V levels | not established | [24] |
| 4 | Resistors, 2.2 kΩ and 3.3 kΩ (two pairs) | generic | D1 → reader RX and D2 → ENABLE dividers | not established | inference, §2.2 |
| 5 | Vulcan RFID C610 antenna | VUL-C610-FCC | Outdoor circular antenna | 149.00 | [15] |
| 6 | Taoglas cable assembly | CAB.011 | U.FL to SMA(F) bulkhead | not established | [19] |
| 7 | 195-series coax, SMA-M to SMA-M, 12 ft | Vulcan accessory (part number not established) | Bulkhead to antenna | 57.00 | [16] |
| 8 | HID Sentry Duo, US | AST1-160-0300 | Bag tag | 1.86 each (box of 150: 279.00) | [20], [21] |
| 9 | Mean Well DIN-rail DC-DC | DDR-30G-12 | Protected 12 V to board VIN | not established | [27] |
| 10 | Vishay load-dump TVS | SM8S22AHM3 (successor named for SM8S22A) | Clamp 12 V input transients | not established | [26] |
| 11 | Pololu step-down regulator | D36V28F5 | Reader 5 V, only if the existing supply fails §5.2 | 23.95 | [28] |
| 12 | Hammond enclosure, polycarbonate | 1554 series, size not fixed | Sealed housing | not established | [29] |
| 13 | Cable glands, IP68 metric | not established | Cable entries | not established | — |
| 14 | Fuse and holder | not established | Supply protection | not established | — |

**Priced lines:** items 1, 2, 5, 7 and 11 total **$567.40**. With one box of tags, **$846.40**.

---

## 7. What could not be established

- **What "more robust" is meant to fix.** No failure of the board in service is recorded in the
  repository. The platform choice in §1.4 turns on it.
- **The existing 5 V supply:** part, ripple and switching frequency, so whether it meets the
  M7E's §5.2.2 limits. Alex's capacity statement is not a measurement.
- **The board supply in service today** and whether any 12 V transient protection exists ahead of
  it.
- **The tractor's 12 V transient profile.** Whether the 5055E's alternator has load-dump
  suppression is not recorded. ISO 16750-2 / ISO 7637-2 test levels were not read.
- **Whether the M7E's FCC modular grant covers the SparkFun carrier** with the C610 (§3.1).
- **Prices** for items 3, 4, 6, 9, 10, 12, 13 and 14. Hammond's and Industrial Shields' pages sat behind a JavaScript
  check and Lapp's catalogue URLs redirected to a 404. Other distributors were not tried.
- **Whether the HID Sentry Duo price is per unit or a volume tier,** and the tag's UV resistance.
- **Cable glands:** no manufacturer datasheet could be retrieved, so no part is named.
- **Vishay SM8S..AHM3 figures:** taken from the superseded SM8S datasheet, not the AHM3 one.
- **Vulcan S9025PL low-temperature limit:** its datasheet contradicts itself (§3.2).
- **Option (d)'s internal header pin map** and its temperature and ingress ratings.
- **CAEN Lepton3x1 price and host protocol;** its user manual was not reached.
- **Read reliability in the field:** RF power, antenna position and tag orientation can only be
  settled at the machine. Mounting is out of scope.

## 8. What this does not cover

- Firmware. The reader driver, the 115200-baud `Serial1`, fitting a read into the 500 ms loop
  (`notes/rfid-inventory-discussion.md:97`) and the record contract are a separate, gated change.
- Mounting of the reader, antenna and enclosure (`notes/rfid-inventory-discussion.md:65`). Bag
  material and tag placement (`:64`).
- Security hardware of any kind.

---

## References

[1] Arduino, "Arduino UNO R4 WiFi Product Reference Manual, SKU ABX00087," datasheet, modified
Jan. 10, 2026. Filed at `docs/vendor/arduino/ABX00087_UNO-R4-WiFi_datasheet.pdf`. [Online].
Available: https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf

[2] Renesas Electronics, "RA4M1 Group Datasheet," R01DS0355EJ0110, Rev. 1.10, Sep. 2023, Table
2.4 and pin function list. Filed at `docs/vendor/renesas/RA4M1_Group_Datasheet_R01DS0355EJ0110.pdf`.
[Online]. Available: https://www.renesas.com/en/document/dst/ra4m1-group-datasheet

[3] Arduino, "Arduino Opta Product Reference Manual, SKU AFX00001-AFX00002-AFX00003," datasheet.
[Online]. Available: https://docs.arduino.cc/resources/datasheets/AFX00001-AFX00002-AFX00003-datasheet.pdf

[4] Arduino, "Arduino Portenta Machine Control User Manual, SKU AKX00032," datasheet, modified
Oct. 6, 2026. [Online]. Available: https://docs.arduino.cc/resources/datasheets/AKX00032-datasheet.pdf

[5] Waveshare, "ESP32-S3-RS485-CAN," product wiki and product page. [Online]. Available:
https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN ; https://www.waveshare.com/esp32-s3-rs485-can.htm

[6] Waveshare, "ESP32-S3-RS485-CAN Schematic." [Online]. Available:
https://files.waveshare.com/wiki/ESP32-S3-RS485-CAN/ESP32-S3-RS485-CAN-Schematic.pdf

[7] Arduino Store USA, product pages: UNO R4 WiFi, Opta WiFi, Portenta Machine Control. [Online].
Available: https://store-usa.arduino.cc/products/uno-r4-wifi ;
https://store-usa.arduino.cc/products/opta-wifi ;
https://store-usa.arduino.cc/products/portenta-machine-control

[8] JADAK (Novanta), "ThingMagic M7E-HECTO User Guide," Rev. 1.2, Nov. 2023, §5.1.3–5.1.6,
§5.2–5.2.4, §5.4, §5.7–5.8, Table 9 and spec table. [Online]. Available:
https://cdn.sparkfun.com/assets/8/5/2/4/d/M7E_HECTO_User_Guide.pdf

[9] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M7E Hecto, WRL-24738," product
page. [Online]. Available: https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html

[10] SparkFun Electronics (D. Wade), "SparkFun_Simultaneous_RFID_Reader_M7E," schematic, Rev. v10,
Apr. 4, 2024, CC BY-SA 4.0. Filed at
`docs/vendor/sparkfun/SparkFun_Simultaneous_RFID_Reader_M7E_schematic.pdf`. [Online]. Available:
https://cdn.sparkfun.com/assets/9/4/b/c/6/SparkFun_Simultaneous_RFID_Reader_M7E.pdf

[11] SparkFun Electronics, *SparkFun Simultaneous RFID Tag Reader Library*, v1.2.0, MIT License,
`src/SparkFun_UHF_RFID_Reader.h`. [Online]. Available:
https://github.com/sparkfun/SparkFun_Simultaneous_RFID_Tag_Reader_Library

[12] SparkFun Electronics, "SparkFun Simultaneous RFID Reader – M6E Nano, SEN-14066," product page
(retired). [Online]. Available: https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m6e-nano.html

[13] CAEN RFID, "Lepton3x1 – R3101C," product page and specification. [Online]. Available:
https://www.caenrfid.com/en/products/lepton3x1-r3101c/

[14] atlasRFIDstore, "MTI MT-242043/TRH/A/K (RHCP) Outdoor RFID Antenna (Global)," product page.
[Online]. Available: https://www.atlasrfidstore.com/mti-mt-242043-trh-a-k-rhcp-outdoor-rfid-antenna-global/

[15] Vulcan RFID, "C610 Outdoor RFID Antenna, VUL-C610-FCC," product spec sheet v5. [Online].
Available: https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/Vulcan%20RFID%E2%84%A2/Vulcan%20RFIDTM%20C610%20Antenna%20Datasheet%20v5.pdf

[16] atlasRFIDstore, product pages and prices: Vulcan RFID C610 (with cable options) and Vulcan RFID
S9025PL. [Online]. Available: https://www.atlasrfidstore.com/vulcan-rfid-c610-outdoor-rfid-antenna-fcc/ ;
https://www.atlasrfidstore.com/vulcan-rfid-s9025pl-lhcp-outdoor-rfid-antenna-fcc/

[17] Vulcan RFID, "S9025PL (LHCP) Outdoor RFID Antenna, VUL-S9025PLNF," product spec sheet, version
7. [Online]. Available: https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/Vulcan%20RFID%E2%84%A2/ATLAS%20Vulcan%20RFID%E2%84%A2%20S9025PL%20Outdoor%20Antenna%20FCC%20Data%20Sheet%20Version%207.pdf

[18] SparkFun Electronics, "UHF RFID Antenna (RP-TNC), WRL-14131," product page and drawing
GSM-34-900-58-30CM-1814. [Online]. Available: https://www.sparkfun.com/uhf-rfid-antenna-rp-tnc.html ;
https://cdn.sparkfun.com/assets/e/4/d/9/c/GSM-34-900-58-30CM-1814.pdf

[19] Taoglas, "CAB.011 Cable Assembly, IPEX MHFI to SMA(F) Bulkhead," SPE-11-8-068. [Online].
Available: https://www.taoglas.com/datasheets/CAB.011.pdf

[20] HID Global, "Sentry PCB Tag," datasheet (Sentry Duo column). [Online]. Available:
https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/HID/ATLAS%20HID%20Sentry%20PCB%20Tag%20Family%20Datasheet.pdf

[21] atlasRFIDstore, "HID Sentry Duo UHF RFID Tag | M750," product page and price. [Online].
Available: https://www.atlasrfidstore.com/hid-sentry-duo-uhf-rfid-tag-m750/

[22] Xerafy, "TEX 55x12 Silicone," datasheet, version 24 02 06. [Online]. Available:
https://rfid.atlasrfidstore.com/hubfs/1_Tech_Spec_Sheets/Xerafy/ATLAS%20Xerafy%20Tex%20Trak%2055%20x%2012%20Silicone%20v2.pdf

[23] atlasRFIDstore, "Xerafy TEX TRAK UHF RFID Silicone Laundry Tag | 55 mm x 12 mm | NXP UCODE 9,"
product page and price. [Online]. Available:
https://www.atlasrfidstore.com/xerafy-tex-trak-uhf-rfid-silicone-laundry-tag-55-mm-x-12-mm-nxp-ucode-9/

[24] Texas Instruments, "SNx4AHCT125 Quadruple Bus Buffer Gates With 3-State Outputs," SCLS264R,
Dec. 1995, rev. Feb. 2024, §5.2, §5.4. [Online]. Available: https://www.ti.com/lit/ds/symlink/sn74ahct125.pdf

[25] Texas Instruments, "TXB0104 4-Bit Bidirectional Voltage-Level Translator," SCES650K, rev. Mar.
2025, §7.3.2–7.3.5. Filed at `docs/vendor/texas-instruments/TXB0104_SCES650K.pdf`. [Online].
Available: https://www.ti.com/lit/ds/symlink/txb0104.pdf

[26] Vishay General Semiconductor, "SM8S10A thru SM8S43A, Surface Mount PAR Transient Voltage
Suppressors," Document 88387, rev. Feb. 6, 2026. [Online]. Available: https://www.vishay.com/docs/88387/sm8s.pdf

[27] Mean Well, "DDR-30 series, 30W DIN Rail Type DC-DC Converter," specification, file
DDR-30-SPEC 2026-03-31. [Online]. Available: https://www.meanwell.com/Upload/PDF/DDR-30/DDR-30-SPEC.PDF

[28] Pololu, "5V, 3.2A Step-Down Voltage Regulator D36V28F5," product page and specifications.
[Online]. Available: https://www.pololu.com/product/3782

[29] Hammond Manufacturing, "1554W Regular Enclosure," drawing 1554W2GYCL. [Online]. Available:
https://www.hammfg.com/files/parts/pdf/1554W2GYCL.pdf
