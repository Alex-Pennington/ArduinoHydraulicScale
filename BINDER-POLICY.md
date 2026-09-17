# Binder policy

**Owner: Alex. Effective 2026-09-08. Supersedes every prior convention about binder prose,
including every instruction the coordinator gave during the 2026-09-07 sweep.**

---

## 1. What a binder is

> **A binder is a guide for a person building, installing, or working on these PLCs.**
> **It is not a legal document.**

That sentence decides every question below. When a rule here conflicts with any other estate
convention, **this wins inside `references/`.**

The reader is standing at a cabinet with tools in their hands, or at a terminal diagnosing a
running machine. They are not auditing us.

---

## 1a. A binder is the PLANNED BUILD, not the as-is state

**Alex, 2026-09-08:** *"binders represent a planned build not a as is during bringup"* and
*"this is a work in progress and we cant update the documentation and firmware everytime a change
is made."*

> **The binder is the design of record. It describes what will be built.** It is not a status
> report on what is currently wired, flashed, fitted or energised.

**So drift between the binder and today's cabinet is NOT a defect.** During bringup the two are
expected to differ, and chasing the cabinet with the design is how the cards became unreadable in
the first place.

### What this forbids

- **Do not rewrite a design statement into a status statement.** *"The operator switch and the
  ESR NC contact are in series; either can break the run path independently"* is the design and
  **stays**. *"…but only the operator switch actually does"* is bringup state and **does not
  belong in the card at all.**
- **Do not add "as of <date>" status to a design card.** That is the dated-correction habit
  returning under a new name.
- **Do not record what is not yet fitted, flashed or landed** as a qualification on the design.

### What is still a defect, and the distinction is the whole rule

| | |
|---|---|
| **not a defect** | the design says the ESRs carry the run path; they are not yet installed |
| **a defect** | the design **contradicts itself** — one card saying installed, another not yet, on the same question |
| **a defect** | a card **instructs an action** that is wrong against the design — a stop contact onto a diagnostic input |
| **a defect** | a **safety instruction** whose own card disarms it three rows later |

> **Correct what the binder says about the BUILD. Never correct the binder toward the CABINET.**

### The subject test — when status IS the card's job

Some cards have status **as their subject**. A purchase list is about what is bought and what is
not. A defect register is about what is open and what is closed. A decisions record is about what
has been decided, including decisions not yet built.

> **Ask what the card is FOR.**
> **Status that IS the card's subject stays. Status hanging off a design statement goes.**

| | |
|---|---|
| stays | `05` *"Bleed resistors — proposed, not fitted"* — a BOM line means nothing without it |
| stays | `13` *"stated as INTENT"* — a decisions record exists to record decisions not yet built |
| stays | `14` *"D1 CLOSED 2026-08-28"* — that is what a defect register is |
| stays | a **reason** an instruction exists — *"it has been on the wrong terminal here once already"* justifies a standing instruction; a reason is not a status |
| **goes** | `04` *"the physical landing is not verified at the cabinet"* — a qualification on a design statement |
| **goes** | *"Mechanical NC switches AS OF 2026-08-31"* — the switches are the design; the date-stamp is the habit returning |
| **goes** | `12` *"NOT YET FLASHED"* — but the **ordering rule** is design and stays, written as an instruction: *flash a key only after its consumer is deployed* |


## 1b. Alex is the tester

**Alex, 2026-09-08:** *"I am the tester not you, inventing test is the first way to fool
yourself and is bad practice, your testing is for debug not commisioning."*

> **No session writes, invents, rewrites or critiques a commissioning test.** Our testing is
> debug. A commissioning criterion that looks wrong is **reported to Alex**, never edited.

## 2. The one test

Before any sentence stays in a card, it must pass:

> **Can the person doing the job act on this sentence, right now, without knowing anything about
> how the document got this way?**

If it needs the document's history to make sense, **it is not binder text.**

---

## 3. Banned in card prose

These are the things that made the binders unreadable. **None of them may appear in
`references/*.md`.** **Inline strikethrough is banned wider than that** — the gate sweeps
`BINDER-POLICY.md`, `ROADMAP.md` and `design/*.md` for it as well, this document included. A
policy gets no exemption for carrying the construct it forbids; where it must name one, it names
it in words.

| banned | why | where it goes instead |
|---|---|---|
| **Inline strikethrough** — the double-tilde span | a sentence containing its own previous version is a diff, not an instruction | git history; the record |
| **Stacked `Revised:` headers** | four dated revision notes before the reader reaches any content | git history |
| **Commit SHAs in prose** | `2fb579e` means nothing at a cabinet | the record |

> **The SHA clause is about PROVENANCE IN PROSE, not about hex-shaped text.** A build id in a
> payload example, a run id, a filename, a foreign identifier namespace and quoted machine output
> are **data**, and §3 permits all of them. **The gate cannot tell them apart** — it has been
> tightened five times and still over-reports.
>
> **So the SHA clause follows the panel-legend discipline:** where the gate's residual is data
> rather than provenance, **list every remaining hit and name what each one is**, and the card is
> accepted over the clause. **A gate stricter than its policy forces a writer to delete what the
> policy permits**, which is how the caps rule nearly cost a set of panel legends.
| **`NOT RE-VERIFIED`, `not checked against`, and similar hedges** | a guide that says it does not know is not a guide | fix it, or state the standing control |
| **Cross-references as provenance** — *"per `13_decisions.md` Q6"* mid-sentence | breaks reading; the reader wants the fact | a **See also** line at the end of the section |
| **Dates** — every date but the update stamp | *"corrected `<date>`"* is about us, and *"sensor replaced `<date>`"* is a fact with a shelf life; neither helps at the cabinet | the record; **§9** carries the full rule, the update stamp, and the narrow case where the date is itself the fact |
| **First person, and "this session"** | the reader does not know who "I" is | delete |
| **Audit and verification records** | records **about** a binder are not documents **in** it | `docs/`, and the render overlay |

---

## 4. How a correction is made

**The card is edited to say what is true. The old text is deleted.**

**Git holds every previous version. The `docs/` record holds the investigation.** Those two are
sufficient on their own, and they exist today.

> **An audit overlay at render time does NOT exist yet.** `render_branded.py` overlays a
> header/footer stamp and nothing else. Alex's *"we render the binders as written and we can
> present audit data overlayed when rendered"* is the **intended** home for the audit trail, and
> this policy does not depend on it. **Stated as intent rather than as a feature, because a
> document asserting a capability it does not have is the defect this policy exists to stop.**

### The one exception, and it is not strikethrough

When a withdrawn claim could still be **acted on** — somebody may have wired it, bought it, or
set it that way — **convert it into a forward-facing instruction.** Do not strike it.

> ✗ **P2 = DIR** struck through, with **O2 = DIR** set beside it
> ✓ **DIR lands on O2.** If you find it on P2, that is wrong and there will be no direction
>   signal at all — earlier cards said P2. Re-land it on O2.

**A strikethrough asks the reader to reconstruct history. An instruction tells them what to do
about it.** The second is shorter, safer, and reads forward.

---

## 5. Shape

- **Present tense, imperative where there is an action.**
- **The first sentence of every section is the actionable one.**
- **Tables over prose** wherever the content is a set of values.
- **A card states a standing condition, never restates another card.** A card that restates
  another must chase it forever, and will lose.
- **Safety text is never compressed, never de-emphasised, and never traded against any budget in
  this document.**

---

## 6. Size and emphasis

Measured against the same-numbered card in the PLC #1 Aug-15 binder where one exists, and against
the warning-card band (17.7–22.3 bold/1k) where it does not.

- **bold ≤ counterpart** — *the counterpart, not the gate's ceiling.* The gate defaults to 22.3
  (band top) because it cannot know a card's counterpart; **most counterparts are well below it,
  so the gate is a FLOOR and the counterpart is the target.** Measured with wrapped spans joined:
  `tr '\n' ' ' < FILE | grep -oE '\*\*[^*]+\*\*' | wc -l`
- **CAPS runs ≤ 4 per 1,000 words — EXCLUDING panel legends.**

  > **A run of capitals that reproduces a physical marking is a QUOTATION OF THE PANEL, not
  > emphasis.** `MAIN DISCONNECT`, `MASTER E-STOP`, a station label — the card must read the way
  > the thing in front of the operator reads. **Re-casing a legend makes the card disagree with
  > the panel**, which is a factual change to a control-surface reference.
  >
  > Six of the seventeen Aug-15 reference cards exceed 4.0/1k, two of them by four times, and
  > they are exactly the cards that reproduce panel legends — `07_saw_station` 16.0,
  > `08_splitter_station` 14.9, `06_cabinet_master` 13.7. **The 4.0 figure was derived from a
  > binder-wide mean without checking what the spread tracked.** It tracks content type, the
  > same defect as the withdrawn 15.0 bold mean.
  >
  > **The gate cannot tell a legend from shouting, so it still fails the card.** Where the
  > residual is legends, **list every remaining run and name the physical thing each one
  > labels**, and the card is accepted over budget. Where it is not legends, fix it.
- **Rewrite rather than edit** where a card is small enough — a rewrite inherits the reference
  binder's zero-wrapped-span shape for free.

## 6a. Commit congruence — a card cites only what the firmware defines

**A card must not name a firmware identifier the firmware does not define at the commit the card
ships with.** The binder already states the rule in its own voice, in
`references/00_architecture.md`:

> Every claim these cards make about firmware behaviour was read from the source for the firmware
> version stamped at the head of this card.

**The corpus is the working tree, not a git ref.** The gate runs on what is about to be
committed, because that is the commit the cards ship with.

**Why.** A card written ahead of the code reads, at the cabinet, as a description of the machine
standing in front of the reader — and describes a machine that does not exist yet. The reader
cannot tell a constant that is planned from one that is compiled in, so a name the source does
not carry sends them hunting a symbol, a fault code or a topic that nothing will ever emit.

**This is not a completeness rule.** It says nothing about firmware the binder does not document,
and it does not reopen §1a: a symbol the firmware defines is stated as the planned build even
where the hardware it drives is not yet fitted.

### The two exemptions, and they are the only two

**1. Existence-negation.** A card may name a symbol in order to deny that it exists, and for that
citation **absence IS congruence** — flagging it would tell the reader to add the symbol, which
inverts the card. **Where every occurrence line of an identifier denies its existence, the gate
reports it as absent by design rather than as a violation**, and prints it on every run so the
exemption stays visible instead of silent.

| card | symbol | the denial |
|---|---|---|
| `16_esr_system.md` | `ESR_CONTACT_FORM` | *"must not be reintroduced anywhere in firmware or documentation"* |
| `17_esr_build_spec.md` | `SAW_SPEED_OUTPUT_AUTHORITY` | *"does not exist anywhere in the tree"* |

**The narrowness is deliberate: negating a VALUE is not an exemption.** *"must not be set true"*
asserts that the symbol exists and constrains what it holds, so it is still checked. Only a
denial of the symbol itself qualifies.

**2. The accepted-residual listing.** Where a name is not a firmware symbol at all,
`.binder-policy-accepted` carries it, one line per residual, naming what it actually is — the
same discipline §3 applies to SHA-shaped data and §6 to panel legends. Three entries sit there
today, all of them `13_cab1_edge_stack.md`: `MQTT_HOST`, `GF_SERVER_HTTP_ADDR` and
`GF_SERVER_ROOT_URL` are environment variables for the cab1 Raspberry Pi, never compiled into the
sketch. **A residual is accepted by being named, never by being hidden** — list it and say what
it is, or fix the card.

**Firmware roots are per-repo**, so the check travels. `BINDER_FIRMWARE_ROOTS` names them, and
where no root is found the gate says NOT CHECKED out loud rather than passing silently.

**See also** §7 for what the gate covers and what it leaves unchecked.

---

## 7. Enforcement

`./check-binder-policy.sh` **exits non-zero on a violation, and must be committed to the repo it
governs** — a check that lives only in one person's working copy cannot run in a fresh clone, a
worktree, or CI, and therefore gates nothing.

Run it on every card before any binder change is merged. **A rule that produces a number beats a
rule that asks for diligence.**

**What the gate covers.** The card checks — §3 constructs, the §6 budgets and the §9 date ban —
run on `references/*.md`. The §3 strikethrough check additionally runs on `BINDER-POLICY.md`,
`ROADMAP.md` and `design/*.md`; those are not cards, so the §6 budgets do not apply to them.
**Everything outside those paths is unchecked.** The gate's silence there means *not looked at*,
not *clean* — read a clean run as covering exactly the paths named here.

---

## 8. Why this policy exists

The 2026-09-07 sweep cut the saw binder 43% and fixed emphasis, and **Alex was still unable to
follow the cards.** The volume was a symptom. **The cause was that every correction had been
preserved inside the prose**, so each sentence carried its own history and the reader had to
reconstruct the argument to reach the fact.

Three cards were also found asserting control behaviour that had stopped being true — a jog that
was closed, a terminal that was wrong, a pressure gate that had been removed. **A guide nobody
can read is a guide nobody re-reads, and a guide nobody re-reads goes stale.**

---

## 9. Dates

> **A card carries no dates. Two exceptions: the update stamp — the day the card was last
> updated — and a date the card cannot be read without, carried as a named accepted residual.**

**Every other date is banned, in any form, anywhere in the card.** A date about the work — when a
claim was corrected, when a decision was made, when a reading was taken — is history, and §2 keeps
history out of a card. **A date about the machine is banned too.** *"Sensor replaced `<date>`"* reads
at the cabinet as a fact with a shelf life; the card states what the machine is, not when it came
to be that way. **The second exception does not reach that sentence** — striking the date takes
nothing away from what the card tells the reader to build, which is the test below.

### The update stamp, and where it lives

**The stamp records when the card was last touched. It is a field, not a sentence in the card.**
In this binder it lives in `binder_manifest.yaml` as `last_updated`, one per document, and reaches
the page when the card is rendered. `printed_date` is a stamp of the same kind — the day the copy
in the folder was printed.

**The manifest is not a card.** The gate reads `references/*.md`; it does not read the manifest,
and §9 does not touch either field.

**Where a card carries the stamp in its own header block instead, it is one line and exactly this
shape**, and that line alone is exempt:

```
**Updated:** YYYY-MM-DD
```

Anything further on that line is prose and is checked like any other prose.

### What is not a date

| | |
|---|---|
| firmware and document versions | `0.0.60`, `V2.1` |
| part numbers | `AFX00005`, `A0602` |
| parameter, register and terminal numbers | `P1.06`, `0x8000`, `IR5`, `Q4` |
| measurements, ranges and durations | `4-20 mA`, `1.8 kΩ`, `24 V`, `2000 ms` |

**None of these are dates, and §9 does not touch them.** A card that documents a timestamp
**field** gives its format — `YYYY-MM-DDThh:mm:ssZ`, or milliseconds since the epoch — and never a
specimen value carrying a real calendar date.

### What is not a stamp

- **A `Date adopted:` header line is not an update stamp.** It records when something was decided,
  which is the provenance §3 bans. It goes.
- **The `Firmware:` stamp is a version, not a date.** §6a requires it. It stays.
- **"As of `<date>`" is not a stamp.** §1a bans it as status on a design statement, and §9 leaves
  it no way back in under another name.

### When the date IS the fact

Some dates are not provenance. **A date may stay where a stated fact cannot be read, checked or
bounded without it.** Strike such a date and something else on the card stops making sense.

**The test, and it takes one pass at the keyboard: delete the date and read the sentence again.**
If another statement on the card is now unreadable, uncheckable or unbounded, the date was
load-bearing and it stays. **If the sentence still means exactly what it meant, the date was only
recording when something happened, and it goes.**

The line is not *about us* versus *about the machine*. Both sides of that line are full of dates
that record only when, and §9 bans those either way.

**The four shapes that pass the test.**

- **A measurement or sample window.** Where a card states an aggregate — *"N working days between
  `<date>` and `<date>` peak at a mean of P, sd Q, across M samples"* — the window is part of the
  figure. Strike it and the statistic has no population and cannot be re-derived, and a safety
  constant resting on it cannot be argued from the card. **This is not a licence to date a
  reading.** A single measurement's timestamp is provenance: the card states the value. What may
  stay is the window a *stated aggregate* was drawn over.
- **The scope of a waiver, exemption or decision that is bounded in time.** *"Waived once, on
  `<date>`; the waiver covered that occasion only."* The date is what bounds it. Strike it and
  *"that occasion"* names nothing, and a bounded waiver reads at the cabinet as a standing one —
  the failure being dangerous, not untidy.
- **A date forming part of an identifier** — a filename, a path, a document or session id, a
  revision label. `tests/field/<date>-<name>.md` is a name. Digits inside a name are no more a
  date than the digits in `AFX00005`, and removing them breaks the reference exactly the same way.
- **A statement about the currency of a printed copy.** *"Any printed copy of this binder dated
  before `<date>` is out of date — destroy it."* It is about the paper in the reader's hands, and
  the date is the only thing in the sentence they can act on.

**The counter-examples, because an exception with none gets stretched.** All of these remain
banned, and none of them survives the test: **when a claim was corrected**; **when a decision was
taken**; **when a part was fitted, replaced or commissioned**; **when a phase or a test passed**;
*"in production since `<date>`"*; *"as of `<date>`"*. Strike the date in any of them and the card
tells the reader to build precisely what it told them before.

### How a load-bearing date is carried

**No marker goes in the card, and the gate is not taught to tell the two apart.** Nothing that
reads a card by pattern can see whether a date is load-bearing, and a syntax that claimed to say
so would be asserted by the writer and checked by nobody. So this exception is operated the way
every other residual in this policy is operated — **by naming it.**

**A load-bearing date is carried as an accepted residual, and its acceptance line must say why the
date cannot be struck** — not merely that a date is present. *"Pre-existing date"* is not a
reason. *"The window the pressure aggregate was sampled over; strike it and the figure has no
population"* is. **That difference is the whole control**: an exemption states what breaks without
the date, an excuse states that the date exists. Without it, the exception becomes a way to keep
any date at all.

**This line is not struck when the card is rewritten.** A drift residual is temporary and goes as
soon as the card is fixed; a load-bearing date is part of the fact and stays as long as the fact
does. It goes only when the fact it carries goes.

### The accepted residual

**Where a card still carries a date, `.binder-policy-accepted` carries a line naming the card and
saying what the date is** — the discipline §3 applies to SHA-shaped data, §6 to panel legends and
§6a to names that are not firmware symbols. **A residual is accepted by being named, never by being
hidden.** The line is struck when the card is rewritten without the date, **unless the date is
load-bearing**, in which case it stays with the fact.

### Precedence

**§9 governs wherever an earlier section would admit a date.** A date that is a fact about the
machine — when a part was fitted, when a reading was taken — is still a date, and it does not go
in a card. §1a is otherwise unchanged — a card whose subject is status still says what is
open and what is closed, and now says it without dating it.

**See also** §7 for what the gate covers.
