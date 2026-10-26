# RFID pendant: what moving it into ~/mesh would need (for the coordinators)

**Date:** 2026-10-26. Session `rfid-pendant-1025`. Alex chose **"In ~/mesh"** for the pendant
firmware (`notes/2026-10-25-rfid-pendant-rulings.md`). `~/mesh` is the mesh coordinator's repo;
this session read it and edited nothing. This note is findings, not a design he approved, and not
a work order. What a feature carries is his to state.

## What Alex has stated

- The reader comes off PLC #4 and is a pendant-style reader (2026-10-25).
- The pendant is its own copy of the Heltec V4 kit pendant; the LilyGO T-Deck is also a host.
- Scan and RECORD both happen on the pendant; nobody needs the phone for it.
- "The rest" (tally, delivery, tare, undo) is what the mesh handheld code already does.
- How many pendants: not decided, design so either works.
- NFC now, UHF later; no UHF reader yet.

## Measured (T-Deck test, `spikes/tdeck-nfc/`)

An original T-Deck reads an Elechouse PN532 V3 in HSU mode on its Grove plug (GPIO43 TX / GPIO44
RX, 115200): reader firmware 1.6, a 13.56 MHz card UID read, RECORD handled on the device.

## Read from source (cite before relying on it; re-read, it moves)

- `~/mesh` env `mesh-handheld` (Heltec V4 TFT) and `mesh-handheld-tdeck`. PLC #4 commands go by
  HTTP to the PLC #4 PWA (`firmware/src/modules/plc/core/Plc4Operator.h:33`), never MQTT
  (`docs/plans/plc4-handheld.md`). With the PWA's RFID build, its PLC #4 buttons disable themselves
  (`Plc4Operator.cpp:492-493, 618, 843-844`). No RFID code, no moisture entry, no void.
- PLC #4's weight message today has no record ID (`src/ArduinoHydraulicScale.cpp:450-455`). The
  approved contract (`notes/2026-10-07-rfid-record-contract.md`) has the board read the tag; its
  `tag` field on `+` is the retry-only chosen tag (`:85`). `include/rfid_record.h` is deleted on
  main.

## What it would take (inference, for the owners to decide)

1. **mesh (mesh coordinator):** a reader driver on the handheld (UART on the T-Deck Grove plug is
   proven; on the Heltec kit only GPIO43 is free on P5, so a UART needs a pin moved, per the
   hardware research), and RECORD sending the scanned tag.
2. **PLC #4 PWA:** accept a tag on RECORD from the handheld and pass it to the board.
3. **PLC #4 board firmware (gated on Alex approving a specific change):** take the tag from the
   record command instead of reading it, and publish it with a record ID. A contract change to the
   approved contract.
4. **Consumer:** none for the tag itself (it needs a tag on every record and does not care who
   read it, `consumer/plc1_consumer.py:3707`), per the 10-25 finding.

## Open, not decided

UHF reader and antenna (the tie tags are UHF; the PN532 cannot read them). Power for a UHF reader
(neither host has a 5 V rail on battery). The M7E's FCC modular grant conditions (21 cm, no
co-located transmitter) do not fit a hand-held unit. Moisture content entry. One pendant or two.
