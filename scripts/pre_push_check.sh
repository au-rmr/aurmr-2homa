#!/usr/bin/env bash
# Repo rules until CI exists: file size, lab values outside sites/, layer imports, secrets.
set -u; cd "$(git rev-parse --show-toplevel)"; fail=0
files=$( { git ls-files; git ls-files -o --exclude-standard; } | sort -u)
while IFS= read -r f; do
  [ -f "$f" ] || continue
  if [ "$(stat -c %s "$f")" -gt 5242880 ]; then echo "too large (>5 MB): $f"; fail=1; fi
  case "$f" in sites/*|scripts/pre_push_check.sh) ;; *)
    if grep -IlE '192\.168\.(10|43|137)\.[0-9]+|10\.158\.54\.|\b(inter|emmons|winthrop)\.' "$f" >/dev/null 2>&1; then echo "lab value outside sites/: $f"; fail=1; fi;; esac
  if grep -IlE 'BEGIN [A-Z ]*PRIVATE KEY|ghp_[A-Za-z0-9]{20}|github_pat_' "$f" >/dev/null 2>&1; then [ "$f" = scripts/pre_push_check.sh ] || { echo "possible secret: $f"; fail=1; }; fi
done <<<"$files"
chk(){ local d=$1 pat=$2; [ -d "$d" ] || return 0; if grep -rIlE "^\s*(from|import)\s+($pat)" "$d" --include='*.py' >/dev/null; then echo "layer violation in $d: imports $pat"; fail=1; fi; }
chk robot 'aurmr_skills|demos|perception'; chk end_effectors 'aurmr_skills|demos'; chk skills 'demos'
[ $fail = 0 ] && echo "pre-push check OK"; exit $fail
