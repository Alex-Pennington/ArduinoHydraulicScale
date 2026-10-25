# VENDOR DOCUMENT MANIFEST

> **Version:** 1
> **Created:** 2026-10-03
> **Purpose:** provenance for every vendor datasheet and manual in `docs/vendor/<maker>/`.

This file is what makes a re-fetch checkable: fetch the URL, hash it, compare. A checksum
mismatch means the vendor revised the document. That is not automatically a defect, but any
figure a note or card quotes from it must be re-read against the new revision before the
mismatch is dismissed.

"Cited at" gives the first places the device is named. It is not an exhaustive list.
Every file below was checked with `pdftotext` on its first pages for the part number before it
was committed.

---

## Committed

| Device | Cited at | File | Revision | Source | SHA-256 | Retrieved |
|---|---|---|---|---|---|---|
| Arduino UNO R4 WiFi (ABX00087), the controller | PLC #4 Tractor Scale binder, *Controller Card* §*Board*, §*Header pins*; `notes/2026-09-20-can-level-interface-plan.md:21` | `arduino/ABX00087_UNO-R4-WiFi_datasheet.pdf` | footer "Modified: 01/10/2026" | `https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf` | `66642f1c187065ca8402cfb71cd99891e841599a5174b16a0a974bbc5e1eb992` | 2026-10-03 |
| Arduino UNO R4 Minima (ABX00080), bench board | `notes/2026-09-20-can-level-interface-plan.md:25`; `findings/2026-09-20-soak-and-drain-cadence.md:13` | `arduino/ABX00080_UNO-R4-Minima_datasheet.pdf` | as published | `https://docs.arduino.cc/resources/datasheets/ABX00080-datasheet.pdf` | `f56c47b14da7a402de49cce313e8e462bc37445db0e778605d229fcf24b31bcd` | 2026-10-03 |
| Renesas RA4M1, the R4's MCU | `notes/2026-09-20-can-level-interface-plan.md:65`; `findings/2026-09-20-two-node-bench.md:115` | `renesas/RA4M1_Group_Datasheet_R01DS0355EJ0110.pdf` | R01DS0355EJ0110 Rev.1.10 (the revision the note cites) | `https://www.renesas.com/en/document/dst/ra4m1-group-datasheet` | `b96fe3862d2d91d910b04a662631d41dfd3ce02d026fbb80c09aefb853933dbf` | 2026-10-03 |
| TI TCA9548A, I2C multiplexer | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Trunk* | `texas-instruments/TCA9548A_SCPS207H.pdf` | SCPS207H, rev. Sep 2024 | `https://www.ti.com/lit/ds/symlink/tca9548a.pdf` | `2137822fd7128945ea44e83f4e100a932b30964d763715eeffe217ee6080a2dc` | 2026-10-03 |
| PCA9548A, I2C multiplexer (alternative) | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Trunk* | `texas-instruments/PCA9548A_SCPS143G.pdf` | SCPS143G | `https://www.ti.com/lit/ds/symlink/pca9548a.pdf` | `b2238acf0fecd58cbbdf8a9f09e8c354a2ae0414a10cbc4dcc5530516bac6145` | 2026-10-03 |
| Microchip MCP9600, thermocouple amplifier | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Bus channels* | `microchip/MCP960X_DS20005426G.pdf` | DS20005426G | `https://ww1.microchip.com/downloads/en/DeviceDoc/MCP960X-Data-Sheet-20005426.pdf` | `39f1fcca8fb1afd6a02d7bfc8a05c6884012db378434e78df416e6c0688e8558` | 2026-10-03 |
| Nuvoton NAU7802, load-cell amplifier | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Bus channels* | `nuvoton/NAU7802_DataSheet_Rev2.6.pdf` | Rev 2.6 | `https://www.nuvoton.com/export/resource-files/en-us--DS_NAU7802_DataSheet_EN_Rev2.6.pdf` | `1d52edc0267ea1c529003af5b29b1088cee01c64a511079258359ede91633a7d` | 2026-10-03 |
| TI INA219, voltage and current monitor | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Bus channels* | `texas-instruments/INA219_SBOS448G.pdf` | SBOS448G, rev. Dec 2015 | `https://www.ti.com/lit/ds/symlink/ina219.pdf` | `58004eda854d07478e6fc6f4398c114f60a3bcf18d4877471c7c1a306d1fa1cb` | 2026-10-03 |
| Microchip MCP3421, pressure-loop converter | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Bus channels*; *Pressure Loop Wiring* §*Parts* | `microchip/MCP3421_DS22003E.pdf` | DS22003E (the revision the plan cites) | `https://ww1.microchip.com/downloads/en/DeviceDoc/22003e.pdf` | `5249b67ed71256bcd6588a665a4e854955d3f2b17e7e57d8beb0ae7436d668ff` | 2026-10-03 |
| PCF8574, LCD backpack I/O expander | PLC #4 Tractor Scale binder, *Qwiic Stack* §*Bus channels*; *Display* §*Display*, §*Connection* | `texas-instruments/PCF8574_SCPS068K.pdf` | SCPS068K, rev. Sep 2024 | `https://www.ti.com/lit/ds/symlink/pcf8574.pdf` | `7d690920a73c2f71dade2f777bb34173ae582066d86e4af9dd406ed945d2b72a` | 2026-10-03 |
| TI SN65HVD230, CAN transceiver chip | `notes/2026-09-20-can-level-interface-plan.md:44`; `findings/2026-09-20-two-node-bench.md:106` | `texas-instruments/SN65HVD230_SLOS346O.pdf` | SLOS346O, rev. Apr 2018 (the revision the notes cite) | `https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf` | `e98fc0c59168c035d7958111924dae7e21e46ab45bdfd6400aef048299d6d716` | 2026-10-03 |
| Waveshare SN65HVD230 CAN Board, schematic | `notes/2026-09-20-can-level-interface-plan.md:12`; `findings/2026-09-20-two-node-bench.md:18` | `waveshare/SN65HVD230-CAN-Board_schematic.pdf` | 1 page, no revision stated | `https://www.waveshare.com/w/upload/c/c5/SN65HVD230-CAN-Board-Schematic.pdf` | `7f1862fb51dc0252b89cca1e81af58cc27bb59250ff78bdcf52ddb7fc3f87338` | 2026-10-03 |
| NXP TJA1051, CAN transceiver (recommended) | `notes/2026-09-20-can-level-interface-plan.md:100`, `:108` | `nxp/TJA1051_Rev7_2015-01-15.pdf` | **Rev. 7, 15 Jan 2015** | `https://www.farnell.com/datasheets/1931504.pdf` | `0d780a04c4c71142c2454d73980dbdd5be214fa6beb65c51f42d1c9ddc8363bc` | 2026-10-03 |
| Microchip MCP2551, CAN transceiver (evaluated) | `notes/2026-09-20-can-level-interface-plan.md:103`, `:183` | `microchip/MCP2551_DS20001667G.pdf` | DS20001667G | `https://ww1.microchip.com/downloads/en/DeviceDoc/20001667G.pdf` | `d7c9cc459e1b3763929b96b0a1ee9f1f8d0cee408f9d9d31010825d50fdb0418` | 2026-10-03 |
| Microchip MCP2561, CAN transceiver (named successor) | `notes/2026-09-20-can-level-interface-plan.md:185`, `:194` | `microchip/MCP2561-2_DS20005167C.pdf` | DS20005167C | `https://ww1.microchip.com/downloads/en/DeviceDoc/20005167C.pdf` | `c36b24e45446d8a1b7d90c0618a924cac0f117994390e60169287e6ba488c53e` | 2026-10-03 |
| TI TXS0108E, level shifter (evaluated) | `notes/2026-09-20-can-level-interface-plan.md:200` | `texas-instruments/TXS0108E_SCES642L.pdf` | SCES642L, rev. Nov 2024 (the revision the note cites) | `https://www.ti.com/lit/ds/symlink/txs0108e.pdf` | `a6ddbe03088d291d1a1f8012189086b4b9962d1c878a462a8393c59fe225c883` | 2026-10-03 |
| TI TXB0104, level shifter (evaluated) | `notes/2026-09-20-can-level-interface-plan.md:222` | `texas-instruments/TXB0104_SCES650K.pdf` | SCES650K, rev. Mar 2025 | `https://www.ti.com/lit/ds/symlink/txb0104.pdf` | `9a852f7bcc4104df0676d7ee3e4918429c3db2cbd79a5338aca0a1d34471e400` | 2026-10-03 |
| TI TXB0108, level shifter on the R4 WiFi | `notes/2026-09-20-can-level-interface-plan.md:247` | `texas-instruments/TXB0108_SCES643L.pdf` | SCES643L, rev. Feb 2026 | `https://www.ti.com/lit/ds/symlink/txb0108.pdf` | `8f8720b78129d8b4f2ba45ec53cfa16cb511185019ae24eb25253422cf650556` | 2026-10-03 |
| Diodes Inc BSS138, MOSFET (evaluated) | `notes/2026-09-20-can-level-interface-plan.md:232` | `diodes/BSS138_DS30144.pdf` | DS30144 Rev. 25 | `https://www.diodes.com/assets/Datasheets/ds30144.pdf` | `c93dcb33f41424e6a446a34f4222177b3a94e7dc27d2e2efb5e39eb245de3e17` | 2026-10-03 |
| Rosemount 1151 pressure transmitter, product data sheet | PLC #4 Tractor Scale binder, *Pressure Loop Wiring* §*Parts*, §*What each current means*, §*Supply*; *Decisions* §*Scaling* | `emerson-rosemount/Rosemount-1151_PDS_00813-0100-4360_RevJB.pdf` | 00813-0100-4360 Rev JB | `https://www.emerson.com/documents/automation/product-data-sheet-rosemount-1151-pressure-transmitter-en-73232.pdf` | `d22761ac8d4723c30f9b079c1811c6a9261c04aa6e22e2775e7f240579d6a4eb` | 2026-10-03 |
| Rosemount 1151 pressure transmitter, reference manual | PLC #4 Tractor Scale binder, *Pressure Loop Wiring* §*Connections* | `emerson-rosemount/Rosemount-1151_RefManual_00809-0100-4360_RevBA.pdf` | 00809-0100-4360 Rev BA | `https://www.emerson.com/is/content/emerson/en/measurement-instrumentation/technical/products/pressure/documents/doc-rosemount-1151-00809-0100-4360.pdf` | `2c437f3fe5c6d57eed3a3e739680023be1c3b6fe1062772a41fc2efea9edf5c0` | 2026-10-03 |
| John Deere 5055E tractor, equipment and price page | PLC #4 Tractor Scale binder, *Architecture* §*The machine*; `notes/2026-09-01-transducer-failure-and-selection.md:7` | `john-deere/JD_5055E_equipment-price-page_2015-05-11.pdf` | dated 2015-05-11 in the URL; Final Tier 4 model | `https://www.deere.com/assets/pdfs/region-4/industries/government-and-military-sales/contracts/price-pages/commercial/C20_A2_GreaterThan_or_Equalto40HP_LessThan75HP_20150511.pdf` | `4a8912f273553f8f963bf9414ef513ea0f3eefbe99b6e9332197712d6bb2813a` | 2026-10-03 |
| SparkFun Simultaneous RFID Reader – M7E Hecto (WRL-24738), schematic. CC BY-SA 4.0 | `findings/2026-10-07-rfid-hardware-recommendation.md:158`, `:162`, `:476` | `sparkfun/SparkFun_Simultaneous_RFID_Reader_M7E_schematic.pdf` | v10, dated 4/4/2024 | `https://cdn.sparkfun.com/assets/9/4/b/c/6/SparkFun_Simultaneous_RFID_Reader_M7E.pdf` | `5cf7c235bdb30b5c1a16dd8adbafd01af0a77211edaf6dcb492fd5e0920603af` | 2026-10-07 |
| John Deere 520M loader, spec sheet | No binder card; `notes/2026-09-13-4-20ma-loop-decisions.md:144` | `john-deere/JD_520M_SpecSheet.pdf` | not stated | **NOT DETERMINABLE.** Alex supplied it during the 4-20 mA work (commit `34d0a25`); no URL was recorded | `6b0504ab2371a104bcb548902eec1945a9909c3f299fc581f5d3032f9d97ab34` | 2026-09-14 (committed) |
| Arduino Opta, collective datasheet (AFX00001 RS485, **AFX00002 WiFi**, AFX00003 Lite), evaluated platform | `findings/2026-10-14-opta-wifi-platform-evaluation.md:109`, `:360` | `arduino/AFX00001-AFX00002-AFX00003_Opta_datasheet.pdf` | rev. 9; footer "Modified: 08/10/2026" | `https://docs.arduino.cc/resources/datasheets/AFX00001-AFX00002-AFX00003-datasheet.pdf` | `e4a0432eaab33ce68fecc099299fc968cd04f51b6cf76957bae24fa77b69c254` | 2026-10-14 |
| Arduino Opta Analog Expansion, Ext A0602 (AFX00007) | `findings/2026-10-14-opta-wifi-platform-evaluation.md:134`, `:364` | `arduino/AFX00007_Opta-Analog-Expansion_datasheet.pdf` | rev. 4; footer "Modified: 08/10/2026" | `https://docs.arduino.cc/resources/datasheets/AFX00007-datasheet.pdf` | `03509601b0f5565acb39e040224596c6e819c0616ddb3a12cc83f084b93b2dba` | 2026-10-14 |
| Arduino Opta Digital Expansions, Ext D1608E (AFX00005) and D1608S (AFX00006) | `findings/2026-10-14-opta-wifi-platform-evaluation.md:134`, `:368` | `arduino/AFX00005-AFX00006_Opta-Digital-Expansion_datasheet.pdf` | rev. 5; footer "Modified: 08/10/2026" | `https://docs.arduino.cc/resources/datasheets/AFX00005-AFX00006-datasheet.pdf` | `780bbbdb6bab30563c7faa2620ca8e0f851d9e2bdea354f855f40dd43e423be3` | 2026-10-14 |
| Arduino Portenta Machine Control (AKX00032), alternative | `findings/2026-10-14-opta-wifi-platform-evaluation.md:252`, `:385`; `findings/2026-10-07-rfid-hardware-recommendation.md:138` | `arduino/AKX00032_Portenta-Machine-Control_datasheet.pdf` | rev. 9; footer "Modified: 08/10/2026" | `https://docs.arduino.cc/resources/datasheets/AKX00032-datasheet.pdf` | `5a575c1463c017d73f2a82e41517d81270b29d357d7ae562dbd0e52c812528b5` | 2026-10-14 |
| Mean Well DDR-30 series DC-DC (DDR-30G-12, DDR-30G-24), board supply | `findings/2026-10-14-opta-wifi-platform-evaluation.md:150`, `:389`; `findings/2026-10-07-rfid-hardware-recommendation.md:397` | `mean-well/DDR-30-SPEC.pdf` | file name "DDR-30-SPEC 2026-03-31" | `https://www.meanwell.com/Upload/PDF/DDR-30/DDR-30-SPEC.PDF` | `7f88cbe504a9688ead8c239941de58c8f1fde0b930c922461748804b9f615d72` | 2026-10-14 |

**On the sources.**

- **NXP's site refused every request from Saga** on 2026-10-03, returning a "Page not available"
  page with HTTP 404 for `nxp.com/docs/en/data-sheet/*.pdf` (TJA1051, TJA1050, PCA9548A,
  PCF8574). The TJA1051 copy is a Farnell mirror of an *older* revision. Its part number and
  NXP branding were confirmed.
- **PCA9548A and PCF8574.** The cards name the part and not its maker. Both parts are made by
  TI and by NXP. The TI datasheets are filed because NXP's site was unreachable. Read the maker
  off the fitted part before relying on a maker-specific figure.
- **John Deere 5055E.** This is a 2015 government-sales equipment and price page, not an
  operator's or technical manual. It covers the Final Tier 4 5055E. The model year of this
  machine is not recorded in the repo, so treat its figures (for example "Implement Pump
  11.4 GPM") as that model's, not as confirmed for this tractor.
- **The 520M sheet describes a 5085E/5090E/5100E chassis, not a 5055E.** See commit `34d0a25`.
- **RFID hardware finding.** Only the SparkFun M7E Hecto schematic is filed from that finding,
  because it states CC BY-SA 4.0 on its face. The other documents it cites are cited by URL in
  its References section and not filed. Their redistribution terms were not checked.

---

## Not found

| Device | Cited at | What was tried | What would settle it |
|---|---|---|---|
| Jhoinrch RH-02 USB-CAN adapter (bench node) | `findings/2026-09-20-two-node-bench.md:19`; `notes/2026-09-22-tatonka-field-session.md:16` | Web search. The maker publishes only an HTML product page (`jhoinrch.net`) and Amazon listings, and no PDF datasheet was found | Nothing further. The maker does not appear to publish one |
| Waveshare SN65HVD230 CAN Board, user manual or wiki | as the schematic, above | Only the schematic was filed. The Waveshare "Datasheets" PDF is a copy of TI's SLOS346 at an older revision (G) than the one filed above, so it adds nothing | Not needed |

## Not needed

The node uses the SN65HVD230 (`docs/vendor/texas-instruments/SN65HVD230_SLOS346O.pdf` and the Waveshare board schematic). The TJA parts appear only as options in a planning note. Ruled by Alex, 2026-10-03.

| Device | Cited at |
|---|---|
| NXP TJA1051, **Rev. 9** (28 Nov 2017) | `notes/2026-09-20-can-level-interface-plan.md:110` |
| NXP TJA1050, product specification 2003 Oct 22 | `notes/2026-09-20-can-level-interface-plan.md:159` |

## Named only generically (no maker or model in the repo)

Not fetched, because fetching would mean guessing which part is fitted. A maker and model read
off the part would let each one be filed.

| Item | Cited at |
|---|---|
| LCD2004A 20x4 display module (the module maker; the cards do not name the controller chip) | PLC #4 Tractor Scale binder, *Display* §*Display* |
| HC-05 Bluetooth radio module (sold by many makers) | PLC #4 Tractor Scale binder, *Radio Module* §*Module* |
| The Qwiic multiplexer breakout and the other Qwiic breakouts (the board makers, as distinct from the chips filed above) | PLC #4 Tractor Scale binder, *Controller Card* §*Board*; *Qwiic Stack* §*Trunk* |
| 24 V loop supply | PLC #4 Tractor Scale binder, *Pressure Loop Wiring* §*Parts* |
| Board supply | PLC #4 Tractor Scale binder, *Pressure Loop Wiring* §*Parts* (as the supply the loop is kept separate from) |
| 68 Ω burden resistor (a spec is given, not a part) | PLC #4 Tractor Scale binder, *Pressure Loop Wiring* §*Parts* |
| The tractor's J1939 connector or diagnostic port | PLC #4 Tractor Scale binder, *Architecture* §*The machine*; *CAN Transceiver* §*Terminals* |
| The previous, failed transducer and the 0.5–4.5 V replacement candidate | `notes/2026-09-01-transducer-failure-and-selection.md:15`, `:88` |
