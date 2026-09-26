#!/usr/bin/env bash
#
# Batch-publish the workspace crates to crates.io.
#
# The script is deliberately paranoid, because a publish cannot be undone:
#
#   * It is a DRY RUN unless you pass --execute.
#   * Even with --execute it prints the full plan and waits for you to type the
#     word `publish` before anything is uploaded.
#   * It refuses to start if the git tree is dirty (cargo enforces this too) or
#     if any crate's current version is already on crates.io, so you cannot get
#     halfway through a batch and then fail.
#
# Usage:
#   scripts/publish.sh                       # dry run, shows the plan
#   scripts/publish.sh --execute             # publish, after confirmation
#   scripts/publish.sh --only mhz19,pmsx003  # subset
#   scripts/publish.sh --skip hmc5883l       # everything except these
#   scripts/publish.sh --bump minor --execute
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------
if [ -t 1 ]; then
  BOLD=$'\033[1m'; DIM=$'\033[2m'; RED=$'\033[31m'; GREEN=$'\033[32m'
  YELLOW=$'\033[33m'; BLUE=$'\033[34m'; RESET=$'\033[0m'
else
  BOLD=''; DIM=''; RED=''; GREEN=''; YELLOW=''; BLUE=''; RESET=''
fi

info()  { printf '%s\n' "$*"; }
step()  { printf '\n%s==>%s %s%s%s\n' "$BLUE" "$RESET" "$BOLD" "$*" "$RESET"; }
ok()    { printf '  %s✓%s %s\n' "$GREEN" "$RESET" "$*"; }
warn()  { printf '  %s!%s %s\n' "$YELLOW" "$RESET" "$*"; }
err()   { printf '  %s✗%s %s\n' "$RED" "$RESET" "$*" >&2; }
die()   { err "$*"; exit 1; }

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
EXECUTE=0
ASSUME_YES=0
SKIP_CHECKS=0
ALLOW_DIRTY=0
BUMP=""
ONLY=""
SKIP=""

usage() {
  sed -n '3,20p' "$0" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

Options:
  --execute             Actually publish. Without this the script only dry-runs.
  --yes                 Skip the interactive confirmation (implies nothing else).
  --bump <spec>         Bump the workspace version before publishing. <spec> is
                        patch, minor, major, or an explicit X.Y.Z. The bump is
                        committed, since cargo publish refuses a dirty tree.
  --only <a,b,c>        Publish only these crates (directory names).
  --skip <a,b,c>        Publish everything except these.
  --skip-checks         Skip the build/test/clippy/fmt preflight.
  --allow-dirty         Pass --allow-dirty to cargo publish (not recommended).
  -h, --help            Show this help.
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --execute)      EXECUTE=1 ;;
    --yes|-y)       ASSUME_YES=1 ;;
    --skip-checks)  SKIP_CHECKS=1 ;;
    --allow-dirty)  ALLOW_DIRTY=1 ;;
    --bump)         BUMP="${2:-}"; shift ;;
    --only)         ONLY="${2:-}"; shift ;;
    --skip)         SKIP="${2:-}"; shift ;;
    -h|--help)      usage; exit 0 ;;
    *)              die "unknown option: $1 (try --help)" ;;
  esac
  shift
done

# ---------------------------------------------------------------------------
# Locate the workspace root
# ---------------------------------------------------------------------------
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
cd "$ROOT"

[ -f Cargo.toml ] || die "no Cargo.toml in $ROOT"
grep -q '^\[workspace\]' Cargo.toml || die "$ROOT is not a workspace root"

command -v cargo >/dev/null || die "cargo not found"
command -v curl  >/dev/null || die "curl not found"
command -v python3 >/dev/null || die "python3 not found"

USER_AGENT="edrv-publish-script (+https://github.com/embedded-drivers/embedded-drivers)"

# ---------------------------------------------------------------------------
# Enumerate workspace members: "name<TAB>version<TAB>directory"
# ---------------------------------------------------------------------------
list_members() {
  # The python source goes through a quoted heredoc so it can use both quote
  # characters freely; `python3 -c` gets it as one argument.
  cargo metadata --no-deps --format-version 1 2>/dev/null | python3 -c "$(cat <<'PY'
import json, os, sys
meta = json.load(sys.stdin)
root = os.path.realpath(meta["workspace_root"])
for pkg in meta["packages"]:
    d = os.path.relpath(os.path.realpath(os.path.dirname(pkg["manifest_path"])), root)
    print(f"{pkg['name']}\t{pkg['version']}\t{d}")
PY
)"
}

# crates.io lookup: is <name>@<version> already published?
# 0 = exists, 1 = does not, 2 = could not tell
cr_version_published() {
  local name=$1 want=$2 json
  if ! json=$(curl -sS --max-time 30 -H "User-Agent: $USER_AGENT" \
                "https://crates.io/api/v1/crates/$name" 2>/dev/null); then
    return 2
  fi
  printf '%s' "$json" | python3 -c '
import json, sys
want = sys.argv[1]
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(2)
if "crate" not in d:
    sys.exit(1)          # 404 body: the crate itself does not exist yet
for v in d.get("versions", []):
    if v.get("num") == want:
        sys.exit(0)
sys.exit(1)
' "$want"
}

# crate exists at all? 0 = yes, 1 = no, 2 = unknown
cr_name_exists() {
  local name=$1 json
  if ! json=$(curl -sS --max-time 30 -H "User-Agent: $USER_AGENT" \
                "https://crates.io/api/v1/crates/$name" 2>/dev/null); then
    return 2
  fi
  printf '%s' "$json" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(2)
sys.exit(0 if "crate" in d else 1)
'
}

# ---------------------------------------------------------------------------
# Select the crates to publish
# ---------------------------------------------------------------------------
contains() { case ",$1," in *",$2,"*) return 0 ;; *) return 1 ;; esac; }

SELECTED=()
while IFS=$'\t' read -r name version dir; do
  [ -n "$name" ] || continue
  if [ -n "$ONLY" ] && ! contains "$ONLY" "$dir"; then continue; fi
  if [ -n "$SKIP" ] && contains "$SKIP" "$dir"; then continue; fi
  SELECTED+=("$name|$version|$dir")
done < <(list_members)

[ ${#SELECTED[@]} -gt 0 ] || die "no crates selected (check --only/--skip)"

step "Preflight"

# git tree must be clean: cargo publish enforces it, and a batch that fails
# halfway is worse than one that refuses to start.
if [ "$ALLOW_DIRTY" -eq 0 ]; then
  if [ -n "$(git status --porcelain 2>/dev/null)" ]; then
    git status --short >&2 || true
    die "git tree is dirty - commit first, or pass --allow-dirty"
  fi
  ok "git tree clean"
else
  warn "skipping the dirty-tree check (--allow-dirty)"
fi

if [ ! -f "${CARGO_HOME:-$HOME/.cargo}/credentials.toml" ] && \
   [ ! -f "${CARGO_HOME:-$HOME/.cargo}/credentials" ] && \
   [ -z "${CARGO_REGISTRY_TOKEN:-}" ]; then
  warn "no cargo credentials found; run 'cargo login' before the real publish"
fi

if [ "$SKIP_CHECKS" -eq 0 ]; then
  info "  running build / test / clippy / fmt (use --skip-checks to skip)"
  RUSTFLAGS="${RUSTFLAGS:-} -Dwarnings" cargo build --all --quiet 2>&1 | tail -5
  cargo test --all --quiet 2>&1 | grep -E "test result: FAILED" && die "tests failed"
  cargo +nightly fmt --all --check || die "formatting check failed"
  cargo +nightly clippy --all --all-targets --quiet -- -D warnings || die "clippy failed"
  ok "build, tests, fmt and clippy are clean"
else
  warn "skipping the preflight checks (--skip-checks)"
fi

# ---------------------------------------------------------------------------
# Optional version bump
# ---------------------------------------------------------------------------
if [ -n "$BUMP" ]; then
  step "Bumping the workspace version ($BUMP)"
  CURRENT=$(python3 -c '
import re
s = open("Cargo.toml").read()
m = re.search(r"^version\s*=\s*\"([^\"]+)\"", s, re.M)
print(m.group(1))
')
  NEW=$(python3 - "$CURRENT" "$BUMP" <<'PY'
import sys
cur, spec = sys.argv[1], sys.argv[2]
parts = cur.split("+")[0].split("-")[0].split(".")
if spec in ("patch", "minor", "major"):
    major, minor, patch = (int(x) for x in (parts + ["0", "0"])[:3])
    if spec == "patch": patch += 1
    elif spec == "minor": minor, patch = minor + 1, 0
    else: major, minor, patch = major + 1, 0, 0
    print(f"{major}.{minor}.{patch}")
else:
    print(spec)
PY
)
  [ "$NEW" != "$CURRENT" ] || die "bump spec '$BUMP' did not change the version"
  info "  $CURRENT -> $NEW"
  python3 - "$CURRENT" "$NEW" <<'PY'
import re, sys
old, new = sys.argv[1], sys.argv[2]
s = open("Cargo.toml").read()
s2 = re.sub(r'^(version\s*=\s*")' + re.escape(old) + r'(")', r'\g<1>' + new + r'\2', s, count=1, flags=re.M)
assert s2 != s, "version line not found"
open("Cargo.toml", "w").write(s2)
PY
  cargo update --workspace --quiet 2>/dev/null || true
  git add Cargo.toml Cargo.lock 2>/dev/null || git add Cargo.toml
  git commit --quiet -m "chore: release $NEW"
  ok "bumped to $NEW and committed"

  # Re-read versions now that they changed.
  SELECTED=()
  while IFS=$'\t' read -r name version dir; do
    [ -n "$name" ] || continue
    if [ -n "$ONLY" ] && ! contains "$ONLY" "$dir"; then continue; fi
    if [ -n "$SKIP" ] && contains "$SKIP" "$dir"; then continue; fi
    SELECTED+=("$name|$version|$dir")
  done < <(list_members)
fi

# ---------------------------------------------------------------------------
# Build the plan
# ---------------------------------------------------------------------------
step "Planning"

PLAN_STATUS=()
BLOCKED=0
UNKNOWN=0

for entry in "${SELECTED[@]+"${SELECTED[@]}"}"; do
  IFS='|' read -r name version dir <<<"$entry"
  set +e
  cr_version_published "$name" "$version"
  rc=$?
  set -e
  case $rc in
    0) status="BLOCKED" ; BLOCKED=$((BLOCKED + 1)) ;;
    1)
      set +e; cr_name_exists "$name"; nrc=$?; set -e
      if [ $nrc -eq 0 ]; then status="update"; else status="new"; fi
      ;;
    *) status="unknown"; UNKNOWN=$((UNKNOWN + 1)) ;;
  esac
  PLAN_STATUS+=("$name|$version|$dir|$status")
  printf '  %-16s %-8s %-8s %s\n' "$name" "$version" "$dir" "$status"
done

if [ "$UNKNOWN" -gt 0 ]; then
  die "could not reach crates.io for $UNKNOWN crate(s); refusing to guess"
fi

if [ "$BLOCKED" -gt 0 ]; then
  err "$BLOCKED crate(s) are already published at their current version."
  info ""
  info "  Bump the workspace version and commit it, then re-run, e.g.:"
  info "      scripts/publish.sh --bump patch --execute"
  info ""
  info "  Every crate inherits version.workspace, so one bump covers all of them."
  exit 1
fi

# ---------------------------------------------------------------------------
# Dry-run every crate
# ---------------------------------------------------------------------------
step "Dry run"

for entry in "${PLAN_STATUS[@]+"${PLAN_STATUS[@]}"}"; do
  IFS='|' read -r name version dir status <<<"$entry"
  printf '  %-16s %s ... ' "$name" "$version"
  if cargo publish --dry-run --quiet -p "$name" >/dev/null 2>&1; then
    printf '%sok%s\n' "$GREEN" "$RESET"
  else
    printf '%sfailed%s\n' "$RED" "$RESET"
    cargo publish --dry-run -p "$name" 2>&1 | tail -20
    die "dry run failed for $name"
  fi
done

# ---------------------------------------------------------------------------
# Confirmation
# ---------------------------------------------------------------------------
step "Plan"

COUNT=${#PLAN_STATUS[@]}
info "  $COUNT crate(s) will be published to crates.io:"
for entry in "${PLAN_STATUS[@]+"${PLAN_STATUS[@]}"}"; do
  IFS='|' read -r name version dir status <<<"$entry"
  printf '    %s%s %s%s  (%s)\n' "$BOLD" "$name" "$version" "$RESET" "$status"
done
info ""
info "  Once published, a version cannot be removed - only yanked."
info ""

if [ "$EXECUTE" -eq 0 ]; then
  info "${YELLOW}Dry run only.${RESET} Nothing was uploaded."
  info "Re-run with ${BOLD}--execute${RESET} to publish for real."
  exit 0
fi

if [ "$ASSUME_YES" -eq 0 ]; then
  printf 'Type %spublish%s to upload these %d crate(s), anything else aborts: ' \
    "$BOLD" "$RESET" "$COUNT"
  answer=""
  if ! read -r answer; then
    # EOF: no confirmation was given.
    printf '\n'
    info "No input; aborted. Nothing was uploaded."
    exit 1
  fi
  [ "$answer" = "publish" ] || { info "Aborted; nothing was uploaded."; exit 1; }
fi

# ---------------------------------------------------------------------------
# Publish
# ---------------------------------------------------------------------------
step "Publishing"

PUBLISHED=()

# `"${arr[@]}"` on an *empty* array trips `set -u` on bash older than 4.4, which
# includes the bash 3.2 that ships with macOS. The `+` form expands to nothing
# instead of erroring, so every array expansion below uses it.
publish_one() {
  if [ "$ALLOW_DIRTY" -eq 1 ]; then
    cargo publish --allow-dirty -p "$1"
  else
    cargo publish -p "$1"
  fi
}

for entry in "${PLAN_STATUS[@]+"${PLAN_STATUS[@]}"}"; do
  IFS='|' read -r name version dir status <<<"$entry"
  printf '  %s%-16s %s%s ... ' "$BOLD" "$name" "$version" "$RESET"
  if publish_one "$name" >"/tmp/edrv-publish-$name.log" 2>&1; then
    printf '%sok%s\n' "$GREEN" "$RESET"
    PUBLISHED+=("$name $version")
  else
    printf '%sfailed%s\n' "$RED" "$RESET"
    tail -25 "/tmp/edrv-publish-$name.log"
    info ""
    err "stopped at $name."
    if [ ${#PUBLISHED[@]} -gt 0 ]; then
      info "  Already published:"
      for p in "${PUBLISHED[@]+"${PUBLISHED[@]}"}"; do info "    $p"; done
      resume=$(printf '%s\n' "${PUBLISHED[@]+"${PUBLISHED[@]}"}" | awk '{print $1}' | paste -sd, -)
      info ""
      info "  Resume the remainder with:"
      info "      scripts/publish.sh --execute --skip $resume"
    else
      info "  Nothing was published."
    fi
    exit 1
  fi
done

step "Done"
for p in "${PUBLISHED[@]+"${PUBLISHED[@]}"}"; do ok "$p"; done
info ""
info "  The crates.io index takes a minute to catch up; docs.rs builds follow."
