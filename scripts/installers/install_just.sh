#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

if command -v just &>/dev/null; then
    log SUCCESS "just bereits installiert"
    exit 0
fi

log INFO "Installiere just..."
JUST_TMP=$(mktemp)
if ! curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh -o "$JUST_TMP"; then
    rm -f "$JUST_TMP"
    log WARN "just download fehlgeschlagen (optional)"
    exit 1
fi
chmod +x "$JUST_TMP"
if run_action "Run just installer" bash "$JUST_TMP" -- --to /usr/local/bin; then
    log SUCCESS "just installiert"
else
    log WARN "just Installation fehlgeschlagen (optional)"
    rm -f "$JUST_TMP"
    exit 1
fi
rm -f "$JUST_TMP"
exit 0
