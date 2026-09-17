#!/usr/bin/env bash
# Enforces standards/BINDER-POLICY.md against a binder's references/*.md, and
# enforces the §3 strikethrough ban across the repo's other governed markdown:
# BINDER-POLICY.md, ROADMAP.md and design/*.md. Those are not cards, so the card
# budgets (§6 bold, caps, shape) do not apply to them -- but §3 strikethrough is
# a repo-wide ban, and a gate that is silent outside references/ reads as "clean"
# when it means "not looked at".
# Exit 1 = the binder violates policy. Exit 0 = it does not.
# Usage: check-binder-policy.sh <references-dir> [warning-band-ceiling]
#   bold/1k is ENFORCED against the ceiling (default 22.3, the warning-card band top).
#   BINDER_FIRMWARE_ROOTS overrides the congruence roots (see below).
set -uo pipefail
DIR="${1:?usage: check-binder-policy.sh <references-dir> [bold-ceiling]}"
# §6 legends and §3 data residuals are ACCEPTED when listed. The listing is this
# file, one "card|clause-substring|reason" per line, so an accepted residual is
# machine-checkable and the gate's steady state is exit 0 -- a gate whose normal
# state is failure cannot be used as a blocking hook.
ACCEPT="$(dirname "$DIR")/.binder-policy-accepted"
[ -f "$ACCEPT" ] || ACCEPT="$(cd "$(dirname "$DIR")" && pwd)/.binder-policy-accepted"
CEIL="${2:-22.3}"   # §6: warning-card band top. Pass a card-specific counterpart to tighten.
# The repo root is the parent of <references-dir>. Both the congruence check
# and the §3 non-card sweep below resolve their paths against it, so the script
# travels with the policy into any repo this governs.
ROOT="$(cd "$(dirname "$DIR")" && pwd)"
fail=0; cards=0

say() { printf '%s\n' "$*"; }
accepted=0
viol() {
  if [ -f "$ACCEPT" ] && awk -F'|' -v c="$1" -v m="$2" '
        $1==c && index(m,$2)>0 {found=1} END{exit !found}' "$ACCEPT"; then
    accepted=$((accepted+1)); printf '  ok   %-34s %s  [accepted]\n' "$1" "$2"; return
  fi
  fail=1; printf '  FAIL %-34s %s\n' "$1" "$2"
}

# joined-line bold count: wrapped spans are invisible to a per-line grep
bold()  { tr '\n' ' ' < "$1" | grep -oE '\*\*[^*]+\*\*' | wc -l; }
capsr() { grep -oE '\b[A-Z][A-Z][A-Z]+( [A-Z][A-Z]+)+\b' "$1" | wc -l; }
words() { wc -w < "$1"; }

# --- commit congruence ----------------------------------------------------
# A card must not describe firmware behaviour the firmware does not have AT THIS
# COMMIT. The binder states the rule itself, in 00_architecture.md: "Every claim
# these cards make about firmware behaviour was read from the source for the
# firmware version stamped at the head of this card." A card written ahead of
# the code reads, at the cabinet, as a description of the machine in front of
# you -- and describes one that does not exist yet.
#
# The corpus is the WORKING TREE, not a git ref. The gate runs on what is about
# to be committed, which is the commit the cards ship with.
#
# FIRMWARE ROOTS ARE PER-REPO; these are PLC #1's. The splitter keeps firmware
# under src/firmware/opta-cpp/splitter_plc/ and the saw under
# firmware/saw_actuator/ -- set BINDER_FIRMWARE_ROOTS (space-separated, relative
# to the repo root) to carry this check there. Roots that do not exist are
# skipped; if none exist the check says NOT CHECKED out loud rather than passing
# silently, because a gate that is silent reads as "clean" when it means "not
# looked at".
FWROOTS="${BINDER_FIRMWARE_ROOTS:-plc1 libraries sensor_bringup}"

# A card may legitimately name a symbol in order to say it is NOT there --
# 16_esr_system.md on ESR_CONTACT_FORM, 17_esr_build_spec.md on
# SAW_SPEED_OUTPUT_AUTHORITY. For those citations absence IS congruence, and
# flagging one would tell the reader to add the symbol, inverting the card.
# The phrases below are about EXISTENCE only. "must not be set true" is
# deliberately absent: that negates a value, not the symbol, and must still be
# checked.
NEGATION='must not be re-?introduced|does not exist|do(es)? not appear anywhere|no longer exists?|no longer present|not present in|absent from|has been removed|was removed|never existed|not defined anywhere'

fwsyms=""; fwroots_found=0; cited_total=0; bydesign=0
# A root is taken relative to the repo root unless it is absolute. Absolute roots
# let the firmware corpus be pointed at a tree extracted from another commit,
# which is how this check is tested against a past state of the source.
fwpath() { case "$1" in /*) printf '%s' "$1";; *) printf '%s/%s' "$ROOT" "$1";; esac; }
for r in $FWROOTS; do [ -d "$(fwpath "$r")" ] && fwroots_found=$((fwroots_found+1)); done
if [ "$fwroots_found" -gt 0 ]; then
  fwsyms="$(mktemp)"; trap 'rm -f "$fwsyms"' EXIT
  # Source files only. A name that appears in a vendored library's README is not
  # a symbol the firmware defines.
  roots=(); for r in $FWROOTS; do p="$(fwpath "$r")"; [ -d "$p" ] && roots+=("$p"); done
  find "${roots[@]}" -type f \( -name '*.ino' -o -name '*.h' -o -name '*.hpp' \
       -o -name '*.c' -o -name '*.cpp' -o -name '*.cc' \) -print0 \
    | xargs -0 grep -hoE '\b[A-Z][A-Z0-9_]{2,}\b' | sort -u > "$fwsyms"
fi

# --- §9 dates -------------------------------------------------------------
# A card carries no dates; the update stamp is the only exception, and in this
# binder that stamp is a manifest field (last_updated) rather than card prose,
# so nothing in references/*.md is exempt by shape except a header stamp line
# written exactly "**Updated:** YYYY-MM-DD".
#
# The pattern is deliberately narrow, because this estate writes a great deal
# of text that LOOKS like a date and is not. Each shape below was run against
# the real cards before it was kept:
#   0.0.60, V2.1, AFX00005, P1.06, IR5, 0x8000, 1.8 kOhm, 4-20 mA, 24 V   -- no match
#   "Deceleration time" (Dec), "What plc1 May and May Not Do" (May)       -- no match,
#     because a month name only counts when a number sits next to it
#   2000 ms, 1999 ms dwell constants                                      -- no match,
#     which is why a BARE four-digit year is NOT checked: this binder is full of
#     millisecond constants in the 1900-2099 range and a year check would flag them
#   "cards 06/07/08", "IR3/4/7/8/11/12"                                   -- no match,
#     which is why a slashed date must carry a full four-digit year
# The residual is stated rather than hidden: a two-digit-year slashed date
# (9/9/26) is not detected, because nothing tells it apart from a card-number or
# terminal list, and a gate that flags those teaches people to skip it.
D_ISO='\b(19|20)[0-9]{2}-(0[1-9]|1[0-2])(-(0[1-9]|[12][0-9]|3[01]))?\b'
D_DMY='\b(0[1-9]|[12][0-9]|3[01])-(0[1-9]|1[0-2])-(19|20)[0-9]{2}\b'
D_MONTH='(Jan|January|Feb|February|Mar|March|Apr|April|May|Jun|June|Jul|July|Aug|August|Sep|Sept|September|Oct|October|Nov|November|Dec|December)'
D_MON="\\b${D_MONTH}\\.?[ -][0-9]{1,4}\\b|\\b[0-9]{1,2}(st|nd|rd|th)?[ -]${D_MONTH}\\b"
D_SLASH='\b[0-9]{1,2}/[0-9]{1,2}/(19|20)[0-9]{2}\b|\b(19|20)[0-9]{2}/[0-9]{1,2}/[0-9]{1,2}\b'
DATES="$D_ISO|$D_DMY|$D_MON|$D_SLASH"
# The stamp exemption is the WHOLE line or it is not the stamp. Anything further
# on that line is prose and stays checked.
STAMP='^[0-9]+:\*\*Updated:\*\* [0-9]{4}-[0-9]{2}-[0-9]{2}$'

# Identifier shape. The leading segment is [A-Z][A-Z0-9]+ -- two characters
# minimum -- so a single-letter segment does not qualify. That excludes algebra
# from the electrical sections, where V_PSU is a term in
# "V_PSU = V_transmitter + I x R_cable", not a C++ identifier.
cardids() { grep -ohE '\b[A-Z][A-Z0-9]+(_[A-Z0-9]+)+\b' "$1" | sort -u; }

# The firmware side is matched as a SUBSTRING of the symbol set, not whole-word,
# and that is the load-bearing choice. Cards name a fault FAMILY where the
# source carries per-drive instances: PRESSURE_HIGH on the card against
# PRESSURE_HIGH_SAW and PRESSURE_HIGH_SPL in the source, DRIVE_FAULT against
# DRIVE_FAULT_SAW/_SPLITTER/_BLOWER. Whole-word matching flags every family name
# in the binder, and a gate whose normal state is failure is not a control --
# people pass it a skip flag and it becomes decoration. A family whose members
# all exist describes behaviour that exists.
congruence() {
  local f="$1" b="$2" id occ tot neg ln
  [ -s "$fwsyms" ] || return 0
  while read -r id; do
    [ -n "$id" ] || continue
    cited_total=$((cited_total+1))
    occ="$(grep -nE "\b${id}\b" "$f")"
    tot=$(printf '%s\n' "$occ" | grep -c .)
    neg=$(printf '%s\n' "$occ" | grep -icE "$NEGATION")
    ln=$(printf '%s\n' "$occ" | head -1 | cut -d: -f1)
    # PRESENCE FIRST, THEN THE EXEMPTION. A symbol the firmware defines is
    # congruent whatever the card says about it, so it must never reach the
    # negation branch. Tested the other way round, a card that mentions a real
    # symbol in a sentence carrying a negation phrase — "was removed", "not
    # present in" — is reported as absent by design about a symbol that plainly
    # exists. Measured on the splitter: CYCLE_TIEDOWN_MS is in config.h and
    # cycle.cpp, and its only card line reads "the CYCLE tie-down time was not
    # carried over". Pass/fail was right and the printed reason was false, which
    # is still a gate that lies.
    grep -qF -- "$id" "$fwsyms" && continue
    if [ "$tot" -gt 0 ] && [ "$neg" -eq "$tot" ]; then
      # Named in order to be denied, and genuinely absent. Absence confirms the
      # card; print it so the exemption is visible on every run rather than silent.
      bydesign=$((bydesign+1))
      printf '  ok   %-34s line %s: %s absent by design (congruence)\n' "$b" "$ln" "$id"
      continue
    fi
    viol "$b" "line $ln: $id — no such symbol in the firmware at this commit (congruence). Cite a symbol the firmware defines, or remove the claim from the card."
  done < <(cardids "$f")
}

say "=== binder policy check: $DIR"
for f in "$DIR"/*.md; do
  [ -e "$f" ] || { say "no cards found"; exit 1; }
  b=$(basename "$f"); cards=$((cards+1)); w=$(words "$f"); [ "$w" -gt 0 ] || continue

  # §3 banned constructs. Each is counted, not judged.
  n=$(grep -c '~~' "$f");                                        [ "$n" -gt 0 ] && viol "$b" "inline strikethrough on $n lines (§3)"
  n=$(grep -cE '^\*\*Revised:\*\*|^\*\*Revised:' "$f");          [ "$n" -gt 1 ] && viol "$b" "$n stacked Revised: headers (§3)"
  # a real short SHA essentially always contains an a-f; requiring one stops this
  # matching decimal strings such as a numeric password or a part number.
  # backticked AND bare. \b bounds stop it matching inside a longer hex string.
  # a real SHA has BOTH a hex letter and a digit; that excludes decimal strings
  # (a numeric password) AND hex-lookalike words such as "defaced" or "deadbeef".
  # §3 bans a SHA in PROSE. So define prose: not inside a fenced code block, not
  # inside a URL, and not a segment of a UUID. Each exclusion below cost a false
  # positive on a real card before it was added.
  n=$(awk '/^```/{fence=!fence; next} !fence' "$f" \
      | sed -E 's/`[^`]*`//g' \
      | sed -E 's#https?://[^ )`]*##g' \
      | sed -E 's/[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}//g' \
      | grep -oE '\b[0-9a-f]{7,40}\b' | grep -E '[a-f]' | grep -cE '[0-9]')
  [ "$n" -gt 0 ] && viol "$b" "commit SHA in prose, $n occurrence(s) (§3)"
  n=$(grep -ciE 'NOT RE-VERIFIED|not checked against|this cell was not checked|unverified by' "$f")
                                                                 [ "$n" -gt 0 ] && viol "$b" "$n unresolved hedge(s) (§3)"
  n=$(grep -ciE 'this session|\bI (checked|verified|ran|found|wrote|did not)\b' "$f")
                                                                 [ "$n" -gt 0 ] && viol "$b" "$n session-voice instance(s) (§3)"
  n=$(grep -coE '\.claude/' "$f");                               [ "$n" -gt 0 ] && viol "$b" "$n citation(s) into agent scratch (§3)"

  # §9 dates. Counted per line, with the lines named: a date is fixed by reading
  # the sentence it sits in, so the reader needs to be sent there.
  dh=$(grep -nE "$DATES" "$f" | grep -vE "$STAMP")
  n=$(printf '%s' "$dh" | grep -c .)
  [ "$n" -gt 0 ] && viol "$b" "$n date(s) in prose, only the update stamp is exempt (§9) — line(s) $(printf '%s\n' "$dh" | cut -d: -f1 | tr '\n' ',' | sed 's/,$//')"

  # §6 measurable budgets
  bd=$(bold "$f"); cr=$(capsr "$f")
  bk=$(awk -v b="$bd" -v w="$w" 'BEGIN{printf "%.1f", w?b*1000/w:0}')
  ck=$(awk -v c="$cr" -v w="$w" 'BEGIN{printf "%.1f", w?c*1000/w:0}')
  awk -v c="$ck" 'BEGIN{exit !(c>4.0)}' && viol "$b" "caps-runs ${ck}/1k exceeds 4.0 (§6)"
  awk -v x="$bk" -v c="$CEIL" 'BEGIN{exit !(x>c)}' && viol "$b" "bold ${bk}/1k exceeds ${CEIL} (§6)"

  # unbalanced ** prints as literal asterisks on a laminated card
  tot=$(tr '\n' ' ' < "$f" | grep -o '\*\*' | wc -l)
  [ $((tot % 2)) -ne 0 ] && viol "$b" "unbalanced ** ($tot markers) — prints literally (§6)"

  # commit congruence
  congruence "$f" "$b"

  printf '  %-34s %6s words  bold %5s/1k  caps %4s/1k\n' "$b" "$w" "$bk" "$ck"
done

say "=== $cards card(s) checked, $accepted accepted residual(s)"
if [ "$fwroots_found" -gt 0 ]; then
  say "=== congruence: $cited_total identifier(s) cited, checked against [$FWROOTS] in $ROOT; $bydesign absent by design"
else
  say "=== congruence: NOT CHECKED — no firmware root found under $ROOT (looked for: $FWROOTS). Set BINDER_FIRMWARE_ROOTS."
fi

# §3 strikethrough outside the cards. Paths are relative to $ROOT, set at the top.
#
# There is deliberately NO accept path here. BINDER-POLICY.md is the document that
# bans strikethrough and it carried two struck lines of its own; ruled 2026-09-12
# that it gets no exemption for quoting what it forbids. A file that needs to name
# the construct names it in words.
NONCARD=( "BINDER-POLICY.md" "ROADMAP.md" "design/*.md" )
swept=0
say "=== §3 strikethrough sweep outside the cards: $ROOT"
for g in "${NONCARD[@]}"; do
  for f in $ROOT/$g; do
    [ -e "$f" ] || continue
    swept=$((swept+1)); rel="${f#$ROOT/}"
    n=$(grep -c '~~' "$f")
    if [ "$n" -gt 0 ]; then
      fail=1; printf '  FAIL %-34s inline strikethrough on %s line(s) (§3)\n' "$rel" "$n"
    else
      printf '  %-34s clean\n' "$rel"
    fi
  done
done
say "=== $swept non-card file(s) swept for §3 strikethrough"
if [ "$fail" -ne 0 ]; then
  say "BINDER POLICY VIOLATED — see standards/BINDER-POLICY.md"
  exit 1
fi
say "binder policy: PASS"
