# PLC #4 — CAN logic-level interface: options and recommendation

**Status: planning only. No firmware change, no order placed, no part fitted.**

Every electrical figure below was read from a datasheet downloaded while writing this
document. Each carries the part, the document number, the section and the quoted line.
Where a figure could not be found, it says so instead of estimating.

## 1. Problem

PLC #4 is an Arduino UNO R4 (Renesas RA4M1) reading J1939 CAN on a tractor. The RA4M1 is
fixed at 5 V logic. The transceiver on hand is a **Waveshare SN65HVD230 CAN Board**, a
3.3 V part. The pairing is out of specification in both directions.

### Which board this is for

**The deployed board is an UNO R4 WiFi. The bench board was an UNO R4 Minima.** Both were
checked; the conclusion transfers, but only because both are 5 V RA4M1 boards:

> "The operating voltage for the RA4M1 is fixed at 5 V as to be hardware compatible with
> shields, accessories & circuits" — Arduino **UNO R4 WiFi** datasheet (ABX00087), §"Power
> Supply"

> "On the UNO R4 Minima, the operating voltage is fixed at 5 V to be fully retro compatible
> with shields, accessories" — Arduino **UNO R4 Minima** datasheet (ABX00080)

Had either board been 3.3 V the bench result would not have carried across. It does here.

**The CAN pins differ between the two boards even though the logic level does not.** From
the `renesas_uno` core 1.6.0:

| Board | Header pins | RA4M1 ports |
|---|---|---|
| UNO R4 Minima | `PIN_CAN0_TX` = **D4**, `PIN_CAN0_RX` = **D5** (`variants/MINIMA/pins_arduino.h:151-152`) | D4 → **P103**, D5 → **P102** (`variants/MINIMA/variant.cpp`) |
| UNO R4 WiFi | `PIN_CAN0_TX` = **D10**, `PIN_CAN0_RX` = **D13** (`variants/UNOWIFIR4/pins_arduino.h:155-156`) | D10 → **P103**, D13 → **P102** (`variants/UNOWIFIR4/variant.cpp`) |

The peripheral is on the same silicon pins on both boards — **P103 (CTX0) and P102 (CRX0)**
— brought out to different header positions. **Any wiring instruction must name its board,
or take the pins from `PIN_CAN0_TX` / `PIN_CAN0_RX`.** Writing D4/D5 on a WiFi build, or
D10/D13 on a Minima build, is the trap the existing sketches already guard against.

### Direction 1 — the R4 overdrives the transceiver input

TI **SN65HVD230**, document **SLOS346O** (March 2001, revised April 2018), §8.1 *Absolute
Maximum Ratings*:

> Digital Input and Output voltage, VI (D or R) — MIN **–0.5**, MAX **VCC + 0.5**, V

§8.3 *Recommended Operating Conditions*: `Supply voltage, VCC — MIN 3, MAX 3.6 V`.

At VCC = 3.3 V the ceiling on `D` is **3.8 V**. The R4 drives `D` at 5 V — **about 1.2 V
above an absolute maximum**. §8.1 note (1) states what that means:

> Stresses beyond those listed under Absolute Maximum Ratings may cause permanent damage to
> the device. These are stress ratings only, and functional operation of the device at these
> or any other conditions beyond those indicated under Recommended Operating Conditions is
> not implied. Exposure to absolute-maximum-rated conditions for extended periods may affect
> device reliability.

This is a damage and reliability limit, not a performance limit, and the condition is
continuous — every recessive bit, for the life of the machine.

### Direction 2 — the transceiver underdrives the R4 input

Renesas **RA4M1 Group Datasheet**, **R01DS0355EJ0110 Rev.1.10** (Sep 29, 2023), Table 2.4
*I/O VIH, VIL (1)* (conditions: VCC = 2.7 to 5.5 V):

> Other peripheral input pins excluding IIC — VIH — Min **VCC × 0.8**

At VCC = 5 V that is **4.0 V** required for a guaranteed logic high. P102 and P103 are not
among the 5 V-tolerant ports listed in Table 2.4 Note 3 (`P205, P206, P400 to P404, P407,
P408`) — and the 5 V-tolerant row carries the same `VCC × 0.8` minimum in any case, so
**4.0 V is the threshold either way.**

TI SLOS346O §8.6 *Electrical Characteristics: Receiver*:

> VOH — High-level output voltage — `–6 V ≤ VID ≤ 500 mV, IO = –8 mA` — MIN **2.4 V**

Unloaded, `R` sits near the 3.3 V rail. **Both figures are below 4.0 V.**

### The bench result does not clear it

`findings/2026-09-20-two-node-bench.md` records this exact pairing passing in both
directions with zero frame loss to ~1000 frames/s on 2026-09-20. **That is not evidence the
interface is acceptable**, and this plan does not weaken that position. Guaranteed limits
are worst-case across process and temperature; typical silicon is better. That was one
sample, at room temperature, on short leads, on a quiet two-node bus. The deployed machine
is a tractor — vibration, wide temperature range, a 12 V automotive supply — and it drives
out of radio range unattended.

What the bench proved is the **CAN stack and the firmware**. That remains valid and is not
re-opened here.

## 2. Options

Bit time at 250 kbit/s is **4000 ns**; delay figures are given against that.

| # | Option | TXD (5 V in) | RXD (must reach 4.0 V) | Bus fault rating | Verdict |
|---|---|---|---|---|---|
| 1 | **TJA1051T** (5 V, no VIO pin) | VIH min 0.7·VIO = **3.5 V**, max VIO+0.3 = 5.3 V — **in spec** | IOH spec'd at **VIO − 0.4 V = 4.6 V** — **clears 4.0 V, guaranteed** | **±58 V** | **RECOMMENDED** |
| 2 | TJA1051T/3 (VIO pin) | same, if VIO = 5 V | same, if VIO = 5 V | ±58 V | Works but adds a build-error mode — see §3 |
| 3 | TJA1050 | VIH min **2.0 V**, max VCC+0.3 — in spec | **No VOH minimum published.** Only guaranteed point is IOH −2 mA at **0.7·VCC = 3.5 V** — below 4.0 V | −27 to +40 V | Cannot be shown correct on paper |
| 4 | MCP2551 | VIH min **2.0 V**, max VDD — in spec | VOH min **0.7·VDD = 3.5 V** at IOH = 8 mA — below 4.0 V | Not extracted | **Reject — NRND** |
| 5 | Level shifter + keep SN65HVD230 | fixed | fixed | **still −4 to +16 V** | Reject — fixes the wrong half |
| 6 | TX-only resistor divider | fixed | **not fixed** | still −4 to +16 V | Reject — half a fix |
| 7 | Keep SN65HVD230, accept risk | out of abs max | below threshold | −4 to +16 V | Reject — see §2.7 |

### 2.1 TJA1051T — the recommendation

NXP **TJA1051**, **Rev. 9 — 28 November 2017**, Table 7 *Static characteristics*
(`Tvj = −40 °C to +150 °C; VCC = 4.5 V to 5.5 V; VIO = 2.8 V to 5.5 V`):

> **CAN transmit data input; pin TXD** — VIH HIGH-level input voltage — Min **0.7·VIO**, Max **VIO + 0.3** V

> **CAN receive data output; pin RXD** — IOH HIGH-level output current — `VRXD = VIO − 0.4 V` — Min **−8 mA**, Typ −3, Max −1 mA

Footnote [1] to Table 7: *"Only TJA1051T/3 and TJA1051TK/3 have a VIO pin. In transceivers
without a VIO pin, the VIO input is internally connected to VCC."*

So on a plain **TJA1051T at VCC = 5 V**, VIO = 5 V internally, and:

- **TXD accepts 5 V.** Threshold 3.5 V, ceiling 5.3 V. The R4's 5 V drive sits inside the
  window with margin at both ends.
- **RXD is guaranteed to reach 4.6 V** while still sourcing at least 1 mA. The RA4M1 needs
  4.0 V. **0.6 V of guaranteed margin**, specified across −40 to +150 °C.

This is the only candidate examined that closes **both** directions on published worst-case
numbers rather than on typical behaviour.

Loop delay, Table 8 *Dynamic characteristics*:

> td(TXDL-RXDL) / td(TXDH-RXDH) — Normal mode: versions with VIO pin — Min 40, Max **250 ns**
> — Normal mode: other versions — Min 40, Max **220 ns**

220 ns is **5.5 %** of a 4000 ns bit. Not a constraint at 250 kbit/s.

Bus fault rating, Table 5 *Limiting values*: `VCANH` / `VCANL` **−58 to +58 V**.

### 2.2 Does a separate VIO pin help or hurt here?

**It does not help, and on balance it hurts.** The VIO pin exists to let a 3.3 V MCU talk to
a 5 V transceiver. Our MCU side is **5 V**, so the problem it solves does not exist here.

Tie VIO to 5 V and the /3 part is electrically equivalent to the plain part (VIH 3.5 V,
VOH ≥ 4.6 V) — except that its specified loop delay is **250 ns rather than 220 ns**
(Table 8, quoted above). A small cost for no benefit.

The real cost is the failure mode it introduces. **VIO is a pin someone can wire to 3V3 out
of habit** — it sits next to a 3.3 V rail on every Arduino, and "the logic pin goes to the
logic rail" is the obvious wrong guess. With VIO at 3.3 V, RXD is guaranteed only to
VIO − 0.4 = **2.9 V**, against the RA4M1's 4.0 V — which is **exactly the defect this plan
exists to remove**, silently reintroduced, and it would pass a bench test for the same
reason the present one did.

**Specify the plain TJA1051T.** A part with no VIO pin cannot have VIO miswired.

### 2.3 TJA1050 — in spec on TXD, not provable on RXD

NXP/Philips **TJA1050**, *Product specification* **2003 Oct 22** (supersedes 2002 May 16),
*CHARACTERISTICS* (`VCC = 4.75 V to 5.25 V; Tvj = −40 °C to +150 °C`):

> **Transmitter data input (pin TXD)** — VIH HIGH-level input voltage — output recessive — MIN **2.0**, MAX **VCC + 0.3** V

> **Receiver data output (pin RXD)** — IOH HIGH-level output current — `VRXD = 0.7VCC` — MIN **−2**, TYP −6, MAX −15 mA

TXD is fine: 5 V is inside −0.3 to VCC+0.3 (*LIMITING VALUES*: `VTXD −0.3 to VCC + 0.3`).

**RXD is the problem.** The datasheet gives no VOH minimum — only an output current at a
test point of **0.7·VCC = 3.5 V at VCC = 5 V**. That test point is **below the RA4M1's
guaranteed VIH of 4.0 V**. In practice a CMOS push-pull output driving a µA-level MCU input
will sit near its rail, but **the datasheet does not guarantee it**, so a strict worst-case
argument cannot be made for this part the way it can for the TJA1051.

> **This corrects the recommendation in `findings/2026-09-20-two-node-bench.md`**, which
> says "For the permanent node use a 5 V transceiver — TJA1050 or MCP2551 — which is in spec
> against a 5 V R4 and needs no argument." On the TXD side that is right. On the RXD side
> **both named parts publish their high-level output only at 0.7·VCC**, which is under the
> RA4M1 threshold. The conclusion (use a 5 V transceiver) stands; the two part numbers were
> the weakest available choices for it.

### 2.4 MCP2551 — not recommended for new designs

Microchip **MCP2551** datasheet, **page 1 header**:

> **Not Recommended for New Designs** — Please use MCP2561

That settles it on its own. For completeness, *DC CHARACTERISTICS*:

> D25 VIH High-level input voltage — **2.0** to VDD V — Output Recessive
> D31 VOH High-level output voltage — **0.7 VDD** — IOH = 8 mA

0.7·VDD = **3.5 V** at VDD = 5 V, again below the RA4M1's 4.0 V. **Reject on both counts.**

**MCP2561, the successor Microchip names, was not evaluated** — its datasheet was not
downloaded. It is a plausible alternative to the TJA1051 and would need the same RXD check
before use.

### 2.5 Bidirectional level shifter plus the existing 3.3 V board

**TXS0108E** — TI **SCES642L**, December 2007, revised November 2024, §5.13 *Switching
Characteristics: VCCA = 3.3 V ± 0.3 V*, column `VCCB = 5 V ± 0.5 V`:

| Parameter | Push-pull (max) | Open-drain (min–max) |
|---|---|---|
| tPHL A-to-B | **3.1 ns** | 1.9 – 4.8 ns |
| tPLH A-to-B | **3.5 ns** | **87 – 352 ns** |
| tPHL B-to-A | **3.8 ns** | 1.7 – 4.5 ns |
| tPLH B-to-A | **4.3 ns** | **86 – 339 ns** |
| Maximum data rate | 60 Mbps | 1.2 Mbps |

**On the numbers, propagation delay is not the blocker at 250 kbit/s.** In push-pull mode
the worst case is 4.3 ns — **0.1 % of a 4000 ns bit**. Even the worst-case open-drain
low-to-high figure of 352 ns is **8.8 %** of a bit, which is survivable at this rate though
it would matter at 1 Mbit/s. Anyone rejecting this part on timing grounds alone is rejecting
it for the wrong reason.

The real objections are architectural and structural:

1. **TI steers push-pull users elsewhere.** §8.1 *Application Information*:
   > The device is an excellent choice for use in applications where an open-drain driver is
   > connected to the data I/Os. The device is appropriate for applications where a push-pull
   > driver is connected to the data I/Os, but the TXB0104 device, 4-Bit Bidirectional
   > Voltage-Level Translator might be a better option for such push-pull applications.

   CAN `TXD` and `RXD` are **unidirectional push-pull** signals. An auto-direction-sensing
   translator with one-shot edge accelerators is solving a problem CAN does not have, and
   §8.3 warns that load capacitance must be managed "to avoid one-shot retriggering, bus
   contention, output signal oscillations, or other adverse effects."

2. **BSS138-type shifter is the wrong topology.** The classic two-MOSFET circuit is
   open-drain and bidirectional; its low-to-high edge is an RC through the pull-up.
   Diodes Inc **BSS138**: `VGS(TH)` **0.5 / 1.2 / 1.5 V** (`VDS = VGS, ID = 250 µA`),
   `Ciss` max **50 pF**, `tD(ON)` max **20 ns**. **COMPUTED, NOT SOURCED:** with a 10 kΩ
   pull-up and roughly 60 pF of MOSFET plus stray capacitance, the rise toward threshold
   takes on the order of 1.3 µs — about a third of a bit — and a 1 kΩ pull-up brings that to
   roughly 130 ns. These are my arithmetic from the quoted `Ciss`, not datasheet figures, and
   the stray capacitance is a guess. Treat them as indicative only.

3. **Decisive, and it applies to every shifter option: it fixes the wrong half of the
   problem.** A level shifter corrects the logic interface and leaves the **bus** side
   exactly as exposed as it is now. See §2.7.

It also adds an 8-channel IC, two rails and a set of passives to a vibration-exposed
harness, to use two channels.

*Noted in passing:* the UNO R4 WiFi itself carries a **TXB0108DQSR** for its internal
5 V ↔ 3.3 V ESP32-S3 interface (ABX00087 datasheet, board topology table: "U4 TXB0108DQSR
logic level translator (5 V - 3.3 V)"). That is Arduino using the push-pull part TI
recommends — evidence the technique is sound in general, not evidence it is right here.

### 2.6 Resistor divider on TX only — why it is insufficient on its own

A divider from the R4's `CANTX` down to ~3.3 V does fix **direction 1**. It is cheap, it is
two resistors, and it removes the absolute-maximum violation.

**It is insufficient because it does nothing about direction 2, and a divider cannot fix
direction 2 even in principle.** The transceiver's `R` output must reach the RA4M1's
**4.0 V** (`VCC × 0.8`, Table 2.4). Its guaranteed VOH is **2.4 V** (SLOS346O §8.6) and it
sits near 3.3 V unloaded. **A resistive divider can only attenuate; it cannot raise a
voltage above its source.** Fitting one converts a violation at both ends into a violation
at one end and leaves the machine out of specification.

Two further objections: it leaves the bus fault rating untouched (§2.7), and the divider's
source impedance forms an RC with the `D` pin's input capacitance, which is a new timing
term to characterise for no benefit.

**A TX divider is a reasonable thing to fit on a bench rig to stop cooking a part you are
borrowing. It is not a fix for a deployed machine.**

### 2.7 Keeping the SN65HVD230 and accepting the risk

This deserves an honest hearing rather than dismissal.

**The case for it.** It is on hand and costs nothing. The wiring is known-good — the
`CAN_H`/`CAN_L` swap has already been found and corrected. It passed a real two-node bench
against a foreign node with **zero frame loss over 700 frames to ~1000 frames/s**. Guaranteed
limits are worst-case across process and temperature, and typical silicon has real margin:
the RA4M1 switches nearer VCC/2 than its specified 4.0 V, and a lightly loaded `R` output
drives close to 3.3 V. The overdrive on `D` is into an ESD structure and is current-limited
by the R4's pin drive. Plenty of hobby CAN projects run this pairing indefinitely.

**The case against it, which wins.**

1. **Absolute maximum is a damage limit and the exposure is continuous.** SLOS346O §8.1
   note (1), quoted in §1: exceeding it "may cause permanent damage", and exposure "for
   extended periods may affect device reliability." This is not a transient — `D` is held
   1.2 V over the limit on every recessive bit for the life of the machine. Degradation of
   this kind is cumulative and does not announce itself.

2. **The bench sampled none of the conditions that matter.** n = 1, room temperature, short
   leads, two nodes, quiet bus. The guaranteed numbers are worst-case over **−40 to +85 °C**
   (SLOS346O §8.3, operating free-air temperature). A tractor spans that range; the workshop
   did not. Passing the easiest possible case is the expected outcome for a marginal
   interface, not a discriminating test.

3. **The underdrive direction fails intermittently, and fails silently.** There is no damage
   mechanism on `R` → RA4M1 — it simply stops meeting VIH as temperature and part variation
   move, and the symptom is **dropped frames**. On this machine that means the scale quietly
   stops seeing the tractor, on a vehicle that **drives out of radio range unattended**.

4. **The bus fault rating is wrong for a vehicle, and no logic-side fix changes it.**
   SLOS346O §8.1: `Voltage at any bus terminal (CANH or CANL)` **−4 to 16 V**, transient
   through 100 Ω **−25 to +25 V**. The TJA1051 is rated **−58 to +58 V** (Rev. 9 Table 5)
   and the TJA1050 **−27 to +40 V** DC with ±200 V ISO 7637 transients. A 12 V automotive
   bus with alternator load-dump and short-to-battery faults is precisely what those higher
   ratings exist for. **This is the argument that also kills every level-shifter option:
   they fix the logic interface and leave this untouched.**

**Reject.** A marginal interface that works in a warm workshop is the failure this plan
exists to prevent.

## 3. Recommendation

**Fit a plain `TJA1051T` (SO8, no VIO pin) and retire the Waveshare SN65HVD230 board from
the permanent node.**

Reasoning, in order of weight:

1. It is the only option checked that closes **both** directions on guaranteed worst-case
   numbers: TXD accepts 5 V (VIH 3.5 V, ceiling 5.3 V), and RXD is guaranteed to
   **4.6 V** against the RA4M1's **4.0 V** requirement — 0.6 V of margin specified over
   −40 to +150 °C, not inferred from typical behaviour.
2. Its bus pins are rated **±58 V**, appropriate to a 12 V automotive harness. No
   level-shifting approach can give the existing board this.
3. It needs no extra parts — no shifter, no divider, no second rail. Fewer components on a
   vibrating machine.
4. Loop delay 220 ns max is 5.5 % of a 250 kbit/s bit; not a constraint.
5. **Plain `T`, not `T/3`:** with a 5 V MCU the VIO pin buys nothing, costs 30 ns of
   specified loop delay, and creates a miswiring path that would silently restore the
   present defect (§2.2).

The bench result stands as proof of the **CAN stack and firmware**. Replacing the
transceiver does not disturb it: `PIN_CAN0_TX` / `PIN_CAN0_RX` and the driver are unchanged.

## 4. Bill of materials

For one permanent node. **No parts ordered; quantities and part numbers are a proposal.**

| Item | Part | Qty | Notes |
|---|---|---|---|
| CAN transceiver | **TJA1051T** (SO8, no VIO pin) | 1 + 1 spare | **Not** `T/3` or `TK/3` — see §2.2. `T/E` has an EN pin and was not evaluated. |
| VCC decoupling | 100 nF X7R, 50 V, 0805 or leaded | 1 | Pin 3 (VCC) to pin 2 (GND), as close to the package as the build allows |
| Bulk decoupling | 4.7 µF X7R or tantalum | 1 | Local to the transceiver |
| SO8 breakout | SOIC-8 to DIP adapter | 1 | Only if hand-wiring rather than a made-up board |
| Bus termination | 120 Ω, 1 %, 0.25 W | **0 — DO NOT FIT by default** | See §5. Fit only if this node is proven to be a physical bus end. |

**Not specified, deliberately:** common-mode choke, TVS/bus protection, and any connector.
Automotive EMC hardening is a real topic and is outside what this document can source. Raise
it separately if the node is to be permanently harnessed.

**The Waveshare SN65HVD230 board is not scrapped** — it remains useful for 3.3 V bench work.
It is simply not the part for the tractor.

## 5. Unknown, or to be checked before ordering

1. **Lifecycle and stock status of the TJA1050 and TJA1051 — COULD NOT SOURCE.** The whole
   of `www.nxp.com` returned "Page not available" from this host for every path tried,
   including the root; `cache.nxp.com` serves `/docs/` only and not product-status pages;
   Digi-Key returned HTTP 403 and Mouser returned a stub. **The TJA1051 recommendation rests
   on its datasheet (Rev. 9, 2017), not on a confirmed active lifecycle status.** Confirm at
   a distributor before ordering. The TJA1050's datasheet is a **Philips-branded document
   dated 2003** and carries no lifecycle marking; its age is a reason to check, not a finding.
2. **MCP2551 lifecycle IS sourced** — "Not Recommended for New Designs", datasheet page 1.
   Its named successor **MCP2561 was not evaluated**; its datasheet was not downloaded and
   its RXD VOH is unchecked.
3. **The tractor's actual CAN bit rate.** 250 kbit/s is assumed here from the bench rig and
   J1939 convention. `findings/2026-09-20-two-node-bench.md` lists the tractor's real bit
   rate as still open. The timing margins above are generous enough that 500 kbit/s would not
   change the recommendation, but the assumption should be retired by measurement.
4. **Whether this node sits at a physical end of the bus.** This decides the 120 Ω terminator.
   A J1939 harness is already terminated at both ends; **adding a third terminator is a
   fault**, and it is not answerable from this repository.
5. **Whether the Waveshare board carries its own 120 Ω terminator**, which matters for any
   continued bench use. Not verified here — read it off the board.
6. **The Waveshare board's `Rs` (slope-control) resistor value is unknown.** SLOS346O §8.9
   makes this significant: total loop delay is 115 ns max with `V(Rs) = 0 V`, 185 ns with
   10 kΩ, and **990 ns with 100 kΩ**. Only relevant if that board is kept in service, which
   is not recommended.
7. **Package and footprint versus what can actually be built.** TJA1051T is SO8. If
   hand-soldering to a breakout is not wanted, a made-up TJA1051 module should be sourced
   instead — none has been identified here.

## 6. What must NOT be done

- **Do not wire `VIO` to 3.3 V** if a `TJA1051T/3` is fitted in place of the plain part.
  RXD would then be guaranteed only to 2.9 V against the RA4M1's 4.0 V — the present defect,
  restored, and it would pass a bench test. Specify the plain `T`.

- **Do not raise the Waveshare SN65HVD230 board's VCC to 5 V as a way to fix the logic
  levels.** §8.3 gives the recommended supply as **3 to 3.6 V**; the device is unspecified
  above that, and a CAN transceiver run outside its specified supply drives the bus outside
  ISO 11898-2 levels.

  > **A correction, since this document exists to stop unsourced claims.** The common
  > statement — repeated in `findings/2026-09-20-two-node-bench.md` as "On 5V the part would
  > likely have been destroyed" — is **not what the datasheet says.** SLOS346O §8.1 gives
  > `Supply voltage, VCC` as **−0.3 to 6 V** absolute maximum. 5 V is *below* that absolute
  > maximum. The reason not to do it is that the part is **unspecified and unguaranteed**
  > outside 3.0–3.6 V, not that destruction is certain. "Not instantly destroyed" is not
  > "works", and the honest reason is the better one to record.

- **Do not read the 2026-09-20 bench pass as clearance to deploy the SN65HVD230.** It proved
  the CAN stack and the firmware. It did not test temperature, vibration, harness length, bus
  fault conditions, or part-to-part variation.

- **Do not fit a 120 Ω terminator without confirming this node is a physical bus end.** A
  third terminator on an already-terminated J1939 bus is a fault.

- **Do not treat a TX-only resistor divider as a fix.** It leaves the receive direction out
  of specification and cannot, in principle, correct it (§2.6).

- **Do not carry the Minima's D4/D5 onto an UNO R4 WiFi build.** WiFi CAN is D10/D13. Take
  the pins from `PIN_CAN0_TX` / `PIN_CAN0_RX` rather than writing numbers.

- **Do not change firmware on the strength of this document.** Firmware is gated; nothing in
  the `.ino` changes without Alex approving that specific change. This plan requires no
  firmware change in any case.
