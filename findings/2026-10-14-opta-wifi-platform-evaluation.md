# Moving PLC #4 to an Arduino Opta WiFi: platform evaluation

**Date:** 2026-10-14. **Scope:** research and a recommendation only. Nothing here is approved
for purchase, wiring or firmware. **Question (Alex, 2026-10-14, verbatim):** *"evaluate moving the
plc4 hardware to a arduino opta wifi, consider we could keep the uno for the can bus if the opta
dosent support it"*.

**Decided: keep the UNO only.** Alex ruled 2026-10-14, in the coordinator's dialog. He chose
**"Keep UNO only (Recommended)"** over "Opta + UNO for CAN" and "Portenta Machine Control".
**Those labels are the coordinator's option text, not his sentence.** His act was choosing it.
The 2026-10-07 build stands: one UNO R4 WiFi with a screw-terminal shield. It is closed. The
analysis below is unchanged.

**Sizing rule:** this is a firewood business, not NORAD (Alex, 2026-10-07). No signing, no key
management, nothing mil-spec.

**How to read the claims.** A bracketed number is taken from the document under *References*.
A `file:line` is read from this repository. **Inference** marks a conclusion drawn from cited
facts and stated in no source. **NOT DETERMINABLE** marks something no available source
settles. Nothing here was measured on hardware.

---

## Recommendation and the decision (one screen)

**Recommendation: do not move PLC #4 to the Opta. Keep the 2026-10-07 build, one UNO R4 WiFi
with a screw-terminal shield.**

Plain-language reasons:

1. **The Opta has no CAN, and nothing adds it.** Its datasheet lists Ethernet, RS-485, WiFi,
   Bluetooth and a proprietary expansion port [1] §3.4. The three Opta expansions sold are
   analog I/O and relays [2], [3], [10]. So the UNO stays for the tractor's J1939 bus, as you
   foresaw.
2. **With the UNO kept, the Opta has little left to do.** The Opta has no plain serial port for
   the RFID reader and no connection for the LCD (§2). Without extra parts both stay on the UNO,
   which already does WiFi and MQTT. The Opta would then hold only the pressure loop, and for
   that it needs the $229 Analog Expansion. The strongest form of the idea moves the reader as
   well; it is set out below.
3. **It costs about $428 more for two boxes instead of one.** Opta WiFi $199 plus Analog
   Expansion $229 [10], against $42.45 for the UNO and shield already ruled (§5).
4. **It is rated for a narrower temperature range.** The Opta and its expansions are rated
   −20 to +50 °C [1] §4.1, [2] §3.1. The UNO R4 WiFi is rated −40 to +85 °C [6] §2. This box sits
   outdoors, sealed, unattended. **Inference:** in a sealed box in summer sun, the +50 °C ceiling
   is likely the bigger risk than the cold limit. It would make the Opta the weakest part in the
   box, below the reader's +60 °C case rating
   (`findings/2026-10-07-rfid-hardware-recommendation.md:210`).
5. **Two boxes cost more to run.** You would maintain two firmwares, two WiFi clients and status
   topics, more wiring, and a larger DIN enclosure. Today's firmware is written for the UNO's
   chip (watchdog, reset cause, timers, CAN, EEPROM), so moving to the Opta means rewriting
   those parts, not just recompiling (§2).

**What the Opta would give, fairly stated:** 1 MB of RAM against the UNO's 32 KB [1] §3.1, [6].
It also gives a DIN-rail housing with screw terminals, a ±1 % 4–20 mA input on the expansion
[2] §2.2.2, and the same platform as PLCs #1–#3 (`BRIEF.md:35-36`). The RAM is the only one the
UNO build cannot match. The RFID contract has already sized its queue to fit the UNO
(`include/rfid_record.h:53-62`, depth 16), so the RAM gain does not fix anything that is broken
today.

**The strongest form of your idea: the Opta does the weighing, the UNO does CAN only.** The
UNO reads J1939 and publishes the engine and fault-code topics to MQTT itself. The Opta reads
the loop through the A0602 and the reader through an RS-485-to-TTL converter, and publishes the
weight, the tag and everything else. Engine data never affects the weight
(`references/03_mqtt_contract.md:7-8`), so the boxes need no wire between them. The
one-record design (weight plus tag on one topic, `notes/rfid-inventory-discussion.md:30-34`)
survives, and the 1 MB RAM sits where the queue is. Its costs: the reader link depends on an RS-485-to-TTL
converter that has not been chosen or tested (§2). The LCD has no connection on the Opta, so it
is lost or replaced by a separate panel (no part chosen). It is still about $428 more (§5),
still two firmwares and two MQTT clients with a contract change (§3.1), and still rated to
+50 °C. **The recommendation holds:** this form removes the wired link, but it buys RAM the
UNO build no longer needs and a housing the shield and enclosure already give. In exchange it
takes on an untested reader path, loses or re-buys the display, and lowers the temperature
ceiling.

**If you want one industrial box rather than the UNO,** the candidate is the **Portenta Machine
Control ($300)**, not the Opta. It has CAN, three 4–20 mA inputs, WiFi and an I2C connector on
one board [7]. Its 24 V supply can come from the 24 V model of the Mean Well converter already
in the BOM [8]. It has two blockers: CAN termination is built into the board, which would add
a third terminator to the tractor bus, and it has no 3.3 V serial port for the reader (§4.2).

**The decision you need to make:**

| Option | What it means |
|---|---|
| **Keep the UNO-only build (recommended)** | The 10-07 ruling stands. $42.45 for the controller. One box, one firmware |
| Opta does the weighing, UNO does CAN only | Opta + A0602 read the loop and the reader and publish the weight record; the UNO publishes engine data on its own. No wire between boxes. About $428 more, plus an untested RS-485-to-TTL converter for the reader. The LCD is lost or replaced by a separate panel. Two firmwares, two MQTT clients. Rated to +50 °C |
| Opta + Analog Expansion, with the UNO kept for CAN, LCD and reader | About $428 more. Two boxes, two firmwares, two WiFi clients, and a wired link so the weight and tag meet (§3.2). Gains 1 MB RAM and DIN housing. Rated to +50 °C |
| Portenta Machine Control, one box | $300. CAN, 4–20 mA and WiFi on one board. CAN termination and the reader's serial link must be solved first, so a bench check comes before any purchase |

---

## 0. The 10-07 rejection, tested against the datasheets

`findings/2026-10-07-rfid-hardware-recommendation.md:137` rejected the Opta WiFi as: *"No CAN
and no exposed I2C (proprietary expansion port), so no J1939, loop converter or LCD. IP20,
−20 to +50 °C [3]"*. Each part checked:

| 10-07 claim | Verdict | Evidence |
|---|---|---|
| No CAN | **Confirmed.** | Not among the Opta's interfaces [1] §3.4. Arduino's mbed core defines no CAN pins for the Opta. Its Portenta H7 variant defines `CAN_HOWMANY 1` [5] (`variants/PORTENTA_H7_M7/pins_arduino.h:167-170`; nothing CAN-related in `variants/OPTA/pins_arduino.h`). The Opta build does define `DEVICE_CAN=1` (`variants/OPTA/defines.txt:15`), so the mbed driver is compiled in, but the variant gives it no pins. PB_9 and PH_14 are already Serial3 TX and RX (`variants/OPTA/pins_arduino.h:150-151`; RTS/CTS are PA_15/PB_15, `:152-153`). No Opta expansion has CAN [2], [3], [10] |
| No exposed I2C; proprietary expansion port | **Confirmed in effect, and imprecise.** | The datasheet calls the port "Proprietary Port … Expansion module communication" [1] §3.4. **Electrically it is I2C:** Arduino's expansion library opens `Wire` as master at 400 kHz, uses a detect line, and assigns module addresses from `0x0A` up [4] (`src/OptaController.cpp:324-326`, `src/OptaControllerCfg.h:18`, `src/OptaBluePrintCfg.h:40`). No connector pinout or logic level is published, so hanging the MCP3421 or the LCD backpack on it is NOT DETERMINABLE and unsupported |
| So no loop converter | **Incomplete.** | The MCP3421 cannot attach, but **the loop can still be read**: the Analog Expansion (AFX00007) takes 0/4–20 mA directly, 0–25 mA range, 16-bit, ±1 % [2] §2.2, §2.2.2. The Opta's own inputs are 0–10 V only [1] §3.5.1 |
| So no J1939 or LCD | **Confirmed.** | As above |
| IP20, −20 to +50 °C | **Confirmed.** | [1] §3.1, §4.1. The expansions match: IP20, −20 to +50 °C [2] §3.1, [3] §4.1 |
| (Portenta Machine Control) needs 24 V on a 12 V tractor, `:138` | **True, and solvable.** | 24 V DC ±20 % [7] p. 2. A Mean Well DDR-30**G-24** takes 9–36 V in and gives 24 V [8]. Not a disqualifier on its own (§4.2) |

---

## 1. Opta WiFi (AFX00002): the facts

All from the Arduino Opta collective datasheet [1] unless marked.

| Property | Opta WiFi | Source |
|---|---|---|
| Supply | 12–24 V nominal; **permissible 10.2–27.6 V** | [1] §4.2 |
| Power | 0.6–2 W at 12 V; 0.6–2.2 W at 24 V | [1] §4.2 |
| MCU | STM32H747XI, Cortex-M7 at up to 480 MHz plus Cortex-M4 at up to 240 MHz | [1] §3.2 |
| Memory | **1 MB RAM**, 2 MB flash, 16 MB QSPI flash "shared between manufacturer's internal usage and data logging" | [1] §3.1 |
| WiFi / BLE | 802.11 b/g/n, 2.4 GHz; AP, STA or both. BLE 4.2 in firmware | [1] §1, §5.7 |
| RS-485 | Half-duplex, **no termination fitted**; Modbus RTU or custom serial | [1] §3.4, §5.6 |
| Ethernet | 10/100BASE-T; TCP/IP, Modbus TCP | [1] §3.4, §5.5 |
| USB | USB-C, host **or** device, not both at once. Powers the processor for programming, not the outputs or peripherals | [1] §5.8 |
| Inputs | 8 × analog/digital, **0–10 V analog**, 12–16 bit, **±5 %**, repeatability ±2 %, 8.9 kΩ input impedance. **No current-input mode** | [1] §3.5, §3.5.1 |
| Relay outputs | 4 × NO, 10 A, 250 VAC; DC1 breaking 10 A at 24 V. No short-circuit protection | [1] §3.6 |
| Temperature | **−20 to +50 °C** | [1] §4.1 |
| Protection | **IP20**, pollution degree 2 | [1] §4.1 |
| RTC | About 10 days' retention; NTP over WiFi or Ethernet | [1] §3.1 |
| Watchdog | **Not stated in the datasheet.** PLCs #1–#3 run the STM32 IWDG on Opta (`BRIEF.md:35-36`). The STM32H747 datasheet could not be fetched to confirm it from silicon (§6) | |
| CAN | **None exposed** (§0) | [1] §3.4; [5] |
| Expansion port | Proprietary port, up to 5 modules [1] §3.4, §5.10. I2C on the wire [4] (§0) | |
| Dimensions | 70 mm wide, 88.8 mm high, 56.8 mm deep plus 4.3 mm at the rail | [1] §8.1 |
| Vibration / shock | **Not stated** | [1] |

**Expansions sold.** The Arduino USA store lists three, and no others [10]:

| Expansion | Purpose | Supply | Price |
|---|---|---|---|
| **Opta Ext A0602, AFX00007** | 8 channels, each an input (0–10 V, **0–25 mA current**, PT100) or an output (0–11 V, 0–25 mA), plus 4 PWM outputs | 12–24 V, **permissible 9.6–28.8 V**, 1.5 W at 12 V (minimum figure; no maximum given). Reverse-polarity protected; transient protection to 40 V. Not isolated from its supply | $229.00 [2] §2.1, §3.2, §3.2.1; [10] |
| Opta Ext D1608E, AFX00005 | 16 inputs (0–24 V digital, 0–10/0–24 V analog, ±5 %), 8 electromechanical relays | 12–24 V, permissible 10.2–27.6 V, 3 W max | $141.00 [3] §3.1, §4.2; [10] |
| Opta Ext D1608S, AFX00006 | As D1608E, with 8 solid-state relays, 24 VDC, 3 A | as above | $151.00 [3]; [10] |

**A0602 current input, the part that matters here** [2] §2.2.2: 0–25 mA range, 381.5 nA LSB,
±1 % accuracy, ±1 % repeatability. Short-circuit current limit 25–35 mA when externally
powered; a programmable limit of 0.5–24.5 mA when it powers the loop itself. The datasheet does
not say what the ±1 % is a percentage of. It does not give a compliance voltage for powering
the loop, so whether it could drive the Rosemount without the separate 24 V loop supply is
NOT DETERMINABLE. **Inference:** keep the dedicated 24 V loop supply
(`references/02_pressure_loop_wiring.md:14`) and use the expansion in externally powered mode.

### 1.1 Supply against a 12 V tractor

- **Running.** **Inference:** a charging 12 V system sits above 12 V, inside the Opta's 10.2–27.6 V
  window [1] §4.2. This tractor's charging voltage is not recorded in the repo.
- **Cranking.** Below 10.2 V the Opta is outside its permissible range [1] §4.2. How far this
  tractor's supply falls during a crank is NOT DETERMINABLE from the repo. **Inference:** fed
  straight from the battery, the Opta would be expected to reset during a crank.
- **Load dump.** The Opta datasheet gives no transient rating. The A0602 gives "up to 40 V" [2]
  §2.1. **Inference:** the Opta needs the same front end the 10-07 finding gives the UNO:
  fuse, load-dump TVS and a Mean Well DDR-30 (`findings/2026-10-07-rfid-hardware-recommendation.md:381-404`).
- **Behind a DDR-30.** The G-type models take 9–36 V, switch on at ≥ 9 V and off at ≤ 8.5 V,
  and are reverse-polarity protected [8]. The DDR-30G-12 output (12 V ±2 %) is inside the
  Opta's window. The DDR-30**G-24** (24 V, 1.25 A, adjustable 21.6–28 V) suits the Opta better
  [8]. **Inference:** trimmed to about 22 V, one DDR-30G-24 stays inside the UNO's 6–24 V VIN
  [6] §2, the Opta's 10.2–27.6 V and the A0602's 9.6–28.8 V. At an untrimmed 24 V ±2 % it could
  exceed the UNO's 24 V maximum. Below the converter's 8.5 V cut-off every box resets, Opta or
  UNO alike.

---

## 2. Each function, on the Opta

Today's functions, from `findings/2026-10-07-rfid-hardware-recommendation.md:101-110`:

| Function | Today on the UNO | On the Opta | Route |
|---|---|---|---|
| **4–20 mA loop** | MCP3421 over I2C behind the mux, 68 Ω burden (`include/mcp3421.h:8`; `references/02_pressure_loop_wiring.md:15-16`). Agreed with a gauge to about 0.13 % of span (`BRIEF.md:121-122`) | **A0602 current input**, ±1 % [2] §2.2.2. Or the Opta's native 0–10 V input across a burden, ±5 % [1] §3.5.1 | **Expansion** ($229). Whether the A0602 holds today's 0.13 % agreement is NOT DETERMINABLE without a bench test. **Inference:** single-point calibration (`BRIEF.md:76-79`) absorbs a gain error but not an offset |
| **LCD2004 over PCF8574** | I2C, mux channel 7 (`include/lcd2004.h:6`, `:10-13`) | No usable I2C (§0) and no display interface [1] | **UNO**, or not at all. An RS-485 Modbus panel is possible in principle, but no part was chosen or checked |
| **INA219 power monitor** | I2C, mux channel 2; diagnostics (`include/ina219.h:7`) | No usable I2C | **UNO**, or drop it. Diagnostic only (`references/01_controller_card.md:39`) |
| **J1939 receive** | RA4M1 on-chip CAN, FspTimer ISR, transceiver D tied recessive (`include/j1939_can.h:23-30`; `references/01_controller_card.md:61`) | No CAN (§0) | **UNO** |
| **MQTT over WiFi** | `WiFiS3`, `ArduinoMqttClient` (`src/ArduinoHydraulicScale.cpp:12-13`) | Native WiFi [1] §5.7. PLC #1 already publishes to the estate broker (`BRIEF.md:66`) | **Native.** A port from `WiFiS3` to the mbed WiFi stack (inference) |
| **Watchdog and reset cause** | `WDT.h` with an FspTimer heartbeat; RA4M1 `RSTSR0`/`RSTSR1` (`src/ArduinoHydraulicScale.cpp:14-15`, `:223-232`, `:353-382`) | IWDG, as on PLCs #1–#3 (`BRIEF.md:35-36`) | **Native, rewritten.** Every line cited is RA4M1-specific. The STM32 reset-cause registers were not read (§6) |
| **Calibration store** | `EEPROM.get`/`put` (`src/ArduinoHydraulicScale.cpp:1490`) | 2 MB flash; 16 MB QSPI shared with the maker [1] §3.1 | **Native, rewritten** (inference: a flash key-value store in place of EEPROM) |
| **Offline queue** | 200 records today (`src/ArduinoHydraulicScale.cpp:343`), 12 bytes each by the RFID contract's host test (`findings/2026-10-07-rfid-hardware-recommendation.md:119`; struct at `src/ArduinoHydraulicScale.cpp:328-333`); RFID contract sizes 16 events for 32 KB RAM (`include/rfid_record.h:53-62`) | 1 MB RAM [1] §3.1 | **Native, and roomier.** The one thing the Opta does clearly better |
| **RFID reader UART** | `Serial1` on D0/D1 at 9600, going to the M7E (`src/ArduinoHydraulicScale.cpp:1460`; `notes/rfid-inventory-discussion.md:102-103`) | **No 3.3 V/TTL serial on the terminals.** The only wired serial port is RS-485 [1] §3.4, §5.1. The core maps `Serial1` to Opta pins 13/14 [5] (`variants/OPTA/pins_arduino.h:117-118`), which are MCU pins PG_9/PG_14 [5] (`variants/OPTA/variant.cpp:34-35`), but no published document says which connector, if any, carries them. The SparkFun board has a CH340C USB bridge [9], and the Opta's USB-C can act as host [1] §5.8. Whether the Opta's core drives a CH340 is NOT DETERMINABLE | **UNO**, or an RS-485-to-TTL converter at the reader (no part chosen). **Inference:** one deliberate read per lift is a command and reply, which half-duplex RS-485 can carry. Needs a bench test |

**Net:** on the Opta, natively, only WiFi/MQTT, the watchdog, calibration storage and the queue,
all rewritten. The loop needs the A0602. CAN, LCD and INA219 cannot move off the UNO. The
reader cannot move without a converter nobody has tested.

---

## 3. The split: Opta plus the UNO as a CAN node

### 3.1 How the two would talk

| Link | Opta side | UNO side | Verdict |
|---|---|---|---|
| **Each publishes to MQTT on its own** | Native WiFi | Already does it | **Simplest: no wire between boxes.** Engine and fault-code data already have their own topics and "never affect the weight" (`references/03_mqtt_contract.md:7-8`, `:19-20`). The cost is a contract change: the contract has one client `id` on every payload and one status topic with its will message (`references/03_mqtt_contract.md:17-18`, `:25-26`). A second node needs its own of both |
| RS-485 / Modbus RTU | Native, no termination [1] §5.6 | Needs an RS-485 transceiver on the shield (no part chosen), terminators at both ends, and Modbus slave firmware | Works on paper. Adds a part, a cable, and a new protocol on both sides |
| TTL UART | None on the terminals (§2) | D0/D1 already belong to the reader | No |
| Ethernet | Native [1] §5.5 | The UNO R4 WiFi has none [6] | No |

### 3.2 What each box would actually hold

Following §2, the realistic split is:

- **Opta + A0602:** the pressure loop, the weight, the offline queue, MQTT for the weight,
  telemetry, status, commands and acks.
- **UNO:** J1939, the LCD, the INA219, and the RFID reader unless a converter is proven.
  It needs its own WiFi/MQTT client for the engine and fault-code topics.

**Then the weight and the tag are on different boxes.** The RFID design has the board publish
one complete record, weight plus tag (`notes/rfid-inventory-discussion.md:30-34`). With the reader
on the UNO and the weight on the Opta, one box must get the other's reading over a wired link
before it can publish. That brings back the RS-485 link the MQTT-only split avoids.
**Inference:** the LCD shows the weight (`notes/rfid-inventory-discussion.md:57`), which lives
on the Opta, so it also needs that link.

**The alternative split** moves the reader to the Opta through an RS-485-to-TTL converter and
leaves the UNO on CAN only. The weight and tag then meet on the Opta, and the MQTT-only link
holds. The cost is the untested converter and the loss of the LCD. See the opening decision
section.

### 3.3 What the Opta adds, and what two boxes cost

| The Opta adds | Two boxes cost |
|---|---|
| 1 MB RAM against 32 KB [1] §3.1; [6]. The RFID queue is already sized for 32 KB (`include/rfid_record.h:53-62`) | Two firmwares, two builds, two flashing paths. The Opta firmware is a port of the RA4M1-specific code (§2) |
| A DIN-rail housing with 0.5–4 mm² screw terminals [1] §5.2.2, replacing the screw shield | Two WiFi clients, two status topics, a contract change (§3.1) |
| ±1 % 4–20 mA input on the A0602 [2] | A wired link between the boxes once the weight and tag must meet (§3.2) |
| The PLC #1–#3 platform and toolchain (`BRIEF.md:35-36`) | A −20 to +50 °C box beside a −40 to +85 °C one [1], [2], [6] |
| An RTC with NTP [1] §3.1 (the phone already supplies time: `notes/rfid-inventory-discussion.md:28-29`) | About $428 more (§5). A larger enclosure; the A0602 adds 1.5 W or more at 12 V [2] §3.2 |
| Four 10 A relays [1] §3.6. Nothing on PLC #4 switches a load, so they go unused | Two places for a fault to hide. **Inference:** a dead UNO looks like missing engine data, and a dead Opta like a dead scale |

**The question your instruction raises:** once the UNO stays and already does WiFi and MQTT,
the Opta adds RAM, a housing and a better-specified loop input. The UNO build already gets
round the RAM limit by sizing (`include/rfid_record.h:53-62`). The shield and enclosure in the
10-07 BOM supply the housing. The loop input is fine today at 0.13 % of span. **Inference:** none
of the three justifies a second box.

---

## 4. Alternatives the evaluation surfaces

### 4.1 Keep the UNO-only build (the 10-07 ruling)

Everything carries over unchanged (`findings/2026-10-07-rfid-hardware-recommendation.md:101-110`).
The UNO is rated −40 to +85 °C, VIN 6–24 V, and has on-chip CAN [6] §2. **The binding
constraint is RAM** (`findings/2026-10-07-rfid-hardware-recommendation.md:112-131`), and the RFID
contract has sized for it (`include/rfid_record.h:53-62`).

### 4.2 Arduino Portenta Machine Control (AKX00032), $300 [10]

All from its datasheet [7]:

- **Supply:** 24 V DC ±20 %, reverse-polarity protected, absolute maximum 35 V. **Inference:** a
  DDR-30G-24 [8] feeds it from the tractor.
- **CAN:** on-board TJA1049T/3J, ISO 11898-2, "compatible with a 12V or 24V bus" [7] §3.14.
- **4–20 mA:** 3 analog inputs, each configurable as 0–10 V, 4–20 mA or NTC. 4–20 mA is read
  across a 120 Ω resistor [7] p. 2, §3.6.
- **WiFi/BLE** with an SMA antenna connector; **I2C** on a Grove connector, 3.4 V max; RS-232/422/485;
  Ethernet; USB-A host [7] p. 2–3.
- **MCU and RAM:** STM32H747XI, 1 MB RAM plus 8 MB SDRAM [7] p. 2.
- **Temperature:** −40 to +85 °C [7] §2.1.

**Two blockers:**

1. **Built-in CAN termination.** "60 Ω termination resistors are on board, with 4.7 nF to GND"
   [7] §3.14. The datasheet does not say whether it can be switched off. PLC #4 must add no
   terminator: "a third 120 Ω resistor degrades it" (`references/01_controller_card.md:62`).
   **Inference:** today the node cannot transmit because the transceiver's D pin is tied
   recessive in hardware (`references/01_controller_card.md:61`). On the PMC, receive-only would
   be a software setting.
2. **No 3.3 V serial for the reader.** Its serial port is RS-232/422/485 [7] §3.15. The M7E
   needs 3.3 V CMOS (`findings/2026-10-07-rfid-hardware-recommendation.md:205-206`). A converter
   would be needed, as on the Opta.

It is one box, and the only option here that could replace the UNO outright. **Inference:**
weigh it only if the UNO build fails in service; it is not a better answer today.

### 4.3 Opta with no expansion

The Opta's 0–10 V input across a burden could read the loop for $199 [1] §3.5.1. **Inference:**
a 500 Ω burden gives 10 V at 20 mA, and the 8.9 kΩ input impedance [1] §3.5 is a fixed parallel
load that single-point calibration absorbs. The Rosemount (output code M) needs 12–45 V across
its terminals [11] "Power Supply". At 20 mA that leaves 2 V of margin on the 24 V supply.
At the ±5 % accuracy [1] §3.5.1, this is a step down from today, and it still leaves CAN, LCD
and reader on the UNO.

---

## 5. Cost and enclosure

### 5.1 Prices

Arduino Store USA, read 2026-10-14 [10]:

| Item | SKU | Price |
|---|---|---|
| Arduino Opta WiFi | AFX00002 | $199.00 |
| Arduino Pro Opta Ext A0602 (Analog Expansion) | AFX00007 | $229.00 |
| Arduino Opta RS485 (no WiFi, for comparison) | AFX00001 | $166.00 |
| Arduino UNO R4 WiFi | ABX00087 | $27.50 |
| Portenta Machine Control | AKX00032 | $300.00 |
| Adafruit Proto-ScrewShield R3 (from the 10-07 BOM) | 196 | $14.95 (`findings/2026-10-07-rfid-hardware-recommendation.md:41`) |

| Build | Controller hardware, priced | Not priced |
|---|---|---|
| **UNO-only (ruled)** | **$42.45** (UNO + shield) | Enclosure, DC-DC, TVS, fuse (as 10-07) |
| Opta + A0602 + UNO for CAN | **$470.45** ($199 + $229 + $27.50 + $14.95) | As above, plus a larger DIN enclosure, and an RS-485 transceiver and cable if the boxes are wired |
| Portenta Machine Control | **$300.00** | As above, plus a DDR-30G-24 and a reader serial converter |

The reader, tags and reader supply ($603.90 of the ruled $646.35:
`findings/2026-10-07-rfid-hardware-recommendation.md:42-46`) are the same in every option.

### 5.2 Enclosure and DIN rail

- **DIN length (inference, from the drawings).** Opta 70 mm [1] §8.1, A0602 70 mm (75.5 mm with
  its connector) [2] §6.1, DDR-30 35 mm (`findings/2026-10-07-rfid-hardware-recommendation.md:399`).
  That is about 180 mm of rail before the UNO, which needs its own mounting.
- **Depth.** Opta and A0602 stand 56.8 mm plus 4.3 mm at the rail [1] §8.1, [2] §6.1. The
  Hammond 1554W's inside height is 53.7 mm, already too shallow for the DDR-30
  (`findings/2026-10-07-rfid-hardware-recommendation.md:429-430`). A deeper box is needed either
  way.
- **Sealing.** IP20 [1] §4.1 means a sealed outer enclosure. The bare UNO needs one too, so this
  is not a difference between them. Temperature is the difference (§1, §3.3).
- **The LCD** sits behind a clear coverplate (`BRIEF.md:190`). In the split it stays on the UNO.

---

## 6. What could not be established

- **STM32H747 silicon (FDCAN, IWDG, reset-cause registers).** st.com refused every request
  from Saga (HTTP/2 stream error, then a 100 s timeout with no bytes). Whether the MCU has CAN
  peripherals the Opta does not bring out was not read. It does not change the answer: no
  connector carries CAN [1], [5].
- **The expansion port's pinout and logic level.** Not published [1]. The I2C finding comes
  from library source [4], not from a hardware document.
- **Where the Opta core's `Serial1` (PG9/PG14) goes physically** [5]. NOT DETERMINABLE.
- **CH340 support in the Opta's USB host stack.** NOT DETERMINABLE.
- **The A0602's ±1 % basis and its loop-power compliance voltage** [2]. Not stated.
- **Vibration and shock ratings** for the Opta, the A0602 and the UNO. Not stated in [1], [2] or
  [6].
- **This tractor's cranking dip and charging voltage.** Not in the repo.
- **The site's winter low.** Not in the repo, so whether −20 °C binds is NOT DETERMINABLE.
- **Whether the PMC's CAN termination can be disabled** [7]. Not stated.
- **ArduinoCore-mbed's licence.** GitHub's API reports none, and there is no `LICENSE` file at
  tag 4.6.0 [5].

## 7. What this does not cover

No firmware change, no wiring design for the Opta, no enclosure part, and no RS-485 transceiver
part. Mounting stays out of scope (`notes/rfid-inventory-discussion.md:65`). PLC #1–#3 trees
were not read; the only estate fact used is `BRIEF.md:35-36`.

---

## References

[1] Arduino, "Arduino Opta Collective Datasheet, SKU AFX00001-AFX00002-AFX00003," user manual,
rev. 9. Filed at `docs/vendor/arduino/AFX00001-AFX00002-AFX00003_Opta_datasheet.pdf`. [Online].
Available: https://docs.arduino.cc/resources/datasheets/AFX00001-AFX00002-AFX00003-datasheet.pdf

[2] Arduino, "Arduino Opta Analog Expansion, SKU AFX00007," user manual, rev. 4. Filed at
`docs/vendor/arduino/AFX00007_Opta-Analog-Expansion_datasheet.pdf`. [Online]. Available:
https://docs.arduino.cc/resources/datasheets/AFX00007-datasheet.pdf

[3] Arduino, "Arduino Opta Digital Expansion Collective Datasheet, SKU AFX00005-AFX00006," user
manual, rev. 5. Filed at `docs/vendor/arduino/AFX00005-AFX00006_Opta-Digital-Expansion_datasheet.pdf`.
[Online]. Available: https://docs.arduino.cc/resources/datasheets/AFX00005-AFX00006-datasheet.pdf

[4] Arduino, *Arduino_Opta_Blueprint*, release 0.2.9, Mozilla Public License 2.0,
`src/OptaController.cpp:324-326`, `src/OptaControllerCfg.h:18`, `src/OptaBluePrintCfg.h:40`.
[Online]. Available: https://github.com/arduino-libraries/Arduino_Opta_Blueprint

[5] Arduino, *ArduinoCore-mbed*, release 4.6.0, licence NOT DETERMINABLE (see §6),
`variants/OPTA/pins_arduino.h:117-118`, `:140`, `:143-153`, `:197`; `variants/OPTA/variant.cpp:34-35`; `variants/OPTA/defines.txt:15`;
`variants/PORTENTA_H7_M7/pins_arduino.h:167-170`. [Online]. Available:
https://github.com/arduino/ArduinoCore-mbed

[6] Arduino, "Arduino UNO R4 WiFi Product Reference Manual, SKU ABX00087," datasheet, §2
"Recommended Operating Conditions". Filed at `docs/vendor/arduino/ABX00087_UNO-R4-WiFi_datasheet.pdf`.
[Online]. Available: https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf

[7] Arduino, "Arduino Portenta Machine Control, SKU AKX00032," datasheet, rev. 9. Filed at
`docs/vendor/arduino/AKX00032_Portenta-Machine-Control_datasheet.pdf`. [Online]. Available:
https://docs.arduino.cc/resources/datasheets/AKX00032-datasheet.pdf

[8] Mean Well, "DDR-30 series, 30W DIN Rail Type DC-DC Converter," specification, file
DDR-30-SPEC. Filed at `docs/vendor/mean-well/DDR-30-SPEC.pdf`. [Online]. Available:
https://www.meanwell.com/Upload/PDF/DDR-30/DDR-30-SPEC.PDF

[9] SparkFun Electronics (D. Wade), "SparkFun_Simultaneous_RFID_Reader_M7E," schematic, v10,
CC BY-SA 4.0 (U2, CH340C). Filed at `docs/vendor/sparkfun/SparkFun_Simultaneous_RFID_Reader_M7E_schematic.pdf`.
[Online]. Available: https://cdn.sparkfun.com/assets/9/4/b/c/6/SparkFun_Simultaneous_RFID_Reader_M7E.pdf

[10] Arduino Store USA, product pages and store search: Opta WiFi, Opta RS485, Opta Ext A0602,
Opta Ext D1608E, Opta Ext D1608S, UNO R4 WiFi, Portenta Machine Control. [Online]. Available:
https://store-usa.arduino.cc/products/opta-wifi ; https://store-usa.arduino.cc/products/opta-rs485 ;
https://store-usa.arduino.cc/products/opta-ext-a0602 ; https://store-usa.arduino.cc/products/opta-ext-d1608e ;
https://store-usa.arduino.cc/products/opta-ext-d1608s ; https://store-usa.arduino.cc/products/uno-r4-wifi ;
https://store-usa.arduino.cc/products/portenta-machine-control

[11] Emerson, "Rosemount 1151 Pressure Transmitter," product data sheet 00813-0100-4360 Rev JB,
"Power Supply". Filed at `docs/vendor/emerson-rosemount/Rosemount-1151_PDS_00813-0100-4360_RevJB.pdf`.
[Online]. Available: https://www.emerson.com/documents/automation/product-data-sheet-rosemount-1151-pressure-transmitter-en-73232.pdf
