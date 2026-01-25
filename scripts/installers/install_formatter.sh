#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"

# Ensure REAL_USER is set by caller; try fallback
: ${REAL_USER:=${SUDO_USER:-}}

if command -v shfmt &>/dev/null; then
    log SUCCESS "shfmt bereits installiert"
    exit 0
fi

log INFO "Installiere shfmt..."
TMP=$(mktemp)
if ! curl -sLo "$TMP" https://github.com/mvdan/sh/releases/download/v3.10.0/shfmt_v3.10.0_linux_amd64; then
    rm -f "$TMP"
    log ERROR "shfmt Download fehlgeschlagen"
    record_failure "shfmt installation"
    exit 1
fi

if [ -x /usr/local/bin/shfmt ] && cmp -s "$TMP" /usr/local/bin/shfmt; then
    log_result skip "shfmt already up-to-date"
    rm -f "$TMP"
    exit 0
fi
backup_file /usr/local/bin/shfmt
chmod +x "$TMP"
run_action "Install shfmt" mv "$TMP" /usr/local/bin/shfmt
log SUCCESS "shfmt installiert"
exit 0
