# Source this file. `iam <name>` sets the git author for this terminal from ~/.config/aurmr/people.tsv (name<TAB>full name<TAB>email).
iam() {
  local row; row=$(awk -F'\t' -v n="$1" '$1==n' "${HOME}/.config/aurmr/people.tsv" 2>/dev/null)
  [ -n "$row" ] || { echo "iam: '$1' not in ~/.config/aurmr/people.tsv" >&2; return 1; }
  local name email; name=$(cut -f2 <<<"$row"); email=$(cut -f3 <<<"$row")
  export GIT_AUTHOR_NAME="$name" GIT_AUTHOR_EMAIL="$email" GIT_COMMITTER_NAME="$name" GIT_COMMITTER_EMAIL="$email"
  echo "git identity: $name <$email>"
}
