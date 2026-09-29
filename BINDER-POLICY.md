# Binder policy

The rules for this repo's binder are the estate binder rules: **`RULES.md` in
Alex-Pennington/greybeard-binder-rules.** This file states no rule. It carries only the configuration specific to this repo: where the binder
lives, how the gate runs, and which firmware the gate reads.

## The binder

The cards are `references/*.md`, listed in `references/binder_manifest.yaml`. Examples:
`00_architecture.md`, `01_controller_card.md`, `02_pressure_loop_wiring.md`,
`03_mqtt_contract.md`, `04_decisions.md`, `05_known_defects.md`.

## The gate

```
bash check-binder-policy.sh references [bold-ceiling]
```

Exit 0 is a pass, exit 1 a violation. The gate reads `references/*.md` only; it does not read
the manifest. Its checks, and the `RULES.md` rule each one counts against:

| check | RULES.md |
|---|---|
| inline strikethrough (the double-tilde span) | §2 rule 35 |
| `**Revised:**` header lines | §2 rule 18, §3 rule 52 |
| commit SHA in prose; the `**Firmware:**` line is not prose | §2 rule 36, §5 rule 83 |
| hedge phrases | §2 rule 28 |
| session voice | §2 rule 34 |
| citations into `.claude/` agent scratch | §1 rule 8 |
| dates outside a whole-line `**Updated:** YYYY-MM-DD` stamp | §2 rule 17, §3 rule 51 |
| commit congruence of cited identifiers | §2 rule 33 |
| strikethrough in `BINDER-POLICY.md`, `ROADMAP.md`, `design/*.md` | §2 rule 35; see "Open: policy copies to converge" |

Gate thresholds that `RULES.md` does not set, kept as this repo's configuration:

- bold spans: at most 22.3 per 1,000 words; the second argument overrides it
- runs of capitals: at most 4.0 per 1,000 words
- `**` markers: an even count per card

## Accepted residuals

`.binder-policy-accepted` lists gate hits that are accepted, one per line, as
`<card>|<substring of the gate's message>`. A date hit and a congruence hit are never
accepted; the gate ignores the file for both.

## The congruence corpus

The gate reads the firmware from the working tree, under `src/` and `include/`, in files
ending `.ino`, `.h`, `.hpp`, `.c`, `.cpp` or `.cc`. It extracts identifiers of the shape
`ABC_DEF` from each card and fails any that no firmware symbol contains.
`BINDER_FIRMWARE_ROOTS` (space-separated, relative to the repo root or absolute) overrides the
roots. Where no root exists, the gate prints NOT CHECKED rather than passing silently.
