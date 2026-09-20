# Which broker PLC #4 talks to

Recorded 2026-09-20, closing issue #28. `CLAUDE.md` names "which broker this
machine talks to" as genuinely unknown and not inferable from source, so this
note exists to stop the next session re-deriving it from a config file.

## The answer

**`192.168.10.99` is farm1's address, and it is the one to use in the firmware.**
— Alex, 2026-09-20, asked directly.

`arduino_secrets.h.example:16` already carries it, with the comment "estate
broker, LAN form". That line is correct and needs no change.

## What was measured, and from where

All of this was read from Saga on 2026-09-20. It is measurement, not inference,
except where marked.

| Name | Resolves to | From Saga | Carries `plc4/#` |
|---|---|---|---|
| `192.168.10.99` | — (literal) | 1883 **unreachable** | not testable from Saga |
| `farm1.internal` | `10.77.77.13` | 1883 **open** | **yes**, anonymously |
| `mqtt.firewood.ltd` | `129.212.148.141` | connects | **refused: not authorised** |

Subscribing to `plc4/#` on `farm1.internal` returned the live retained status
and telemetry for `plc4-jd5055e`, so that host is unambiguously the broker this
machine publishes to.

## The bit that is NOT established, so nobody records it as settled

**Whether `mqtt.firewood.ltd` is the same broker was NOT DETERMINED.** It refused
an anonymous connection (`Connection Refused: not authorised`), so its retained
topics could not be compared against farm1's. Alex was not asked for credentials
and none are held here.

What would answer it: authenticate to `mqtt.firewood.ltd` and compare the
retained `plc4/jd5055e/status` payload against farm1's. If the `uptime_s` and
`build` fields track together, it is one broker behind two listeners.

Do not read the table above as saying the public name is a different broker. It
says only that it was not checkable from here.

## Why two addresses for one host is not a contradiction

`192.168.10.99` is the farm LAN address, which is what the board uses — the
board sits on that LAN. `10.77.77.13` is how Saga reaches the same host. Saga is
not on the farm LAN, which is why the literal is unreachable from here and why a
session testing the firmware's configured address from Saga will wrongly
conclude the broker is down.
