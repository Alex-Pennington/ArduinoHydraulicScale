# The AVR → R4 port, and why it explains most of the baseline

Working note, 2026-09-01. Context for anyone picking up PLC #4 cold. The evidence is in
`findings/2026-09-01-plc4-baseline-audit.md`; this is the story that ties it together.

## One commit did it

`c00bafe` (2024-10-27, "button1, fluctuations") moved the firmware from an ATmega UNO to the
UNO R4 WiFi. In that single commit it added WiFi, MQTT, `analogReadResolution(14)` and
`while (!Serial)`. `git log -S` confirms all four arrived together and none existed before.

`origin/uno-r3-swserial-lcd` is the pre-port line and is the most useful artefact in the repo
for this work: when a constant looks wrong, check whether it was *right* on that branch. If it
was, it is a stranded leftover rather than a deliberate choice, and it can be fixed with
confidence about intent.

## Three things the port changed underneath the code, silently

**1. The ADC got 16x finer and nothing was rescaled.** `analogReadResolution(14)` moved the
range from 0..1023 to 0..16383. Two constant sets were calibrated against the old scale and
both were left alone: the pressure map's in-max of `1023` (:258) and the LCD keypad ladder
`60/200/400/600/800` (:203-217). Both are byte-identical to the R3 branch. The transducer's
live zero also moved out from under the default `tareValue = 241` (:22). Three defect clusters,
one omission.

**2. `Serial` stopped being a UART.** On the AVR, `Serial` *was* D0/D1 — which is where an
HC-05 Bluetooth adapter lives, and the repo commits a phone terminal profile for exactly that
(`serial_bluetooth_terminal_cfg.txt`, macros for all seven verbs, 9600 baud matching
`Serial.begin(9600)`). On the R4, `Serial` is USB CDC and D0/D1 became `Serial1`. The firmware
still reads only `Serial`. So the operator console silently migrated from a Bluetooth adapter
on the machine to a USB socket that does not exist on a tractor.

The same change turned `while (!Serial)` from a harmless no-op into a boot-time hang. On the
AVR that guard is always immediately true; the author's own comment (":146 — Needed for native
USB port only") shows it was copied in as boilerplate without the platform difference being
noticed. It is the single most severe line in the file.

**3. The recovery floor disappeared.** The other estate PLCs run on Opta hardware whose IWDG
resets a hung controller. The R4 has a WDT and an IWDT and this firmware enables neither. So
every blocking construct that would have been merely bad on the other machines is terminal
here.

## Why to reach for the R3 branch before guessing

Twice now, checking `origin/uno-r3-swserial-lcd` changed a conclusion from "this looks wrong"
to "this is a stranded constant with a known correct predecessor" — which is a much stronger
claim and points at a much narrower fix. It also settled the HC-05 question's history and
ruled out an integer-overflow hazard that looked real in the abstract (that branch never
produces values large enough to overflow, because it never gained the 14-bit change).

## The shape of the whole thing

The baseline is not badly written. It is a working AVR project that was moved to a new board
and gained network code, and the move was not audited. Nearly every serious defect is either a
constant that outlived its platform or a piece of desktop-oriented boilerplate (`while (!Serial)`,
`while (1);`, blocking connect loops) that is fine on a bench and fatal on a tractor.

That framing matters for the fix gate: this does not want a rewrite. It wants a port review.
