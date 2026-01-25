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
# --- ARCHITEKTUR-ERKENNUNG ---
ARCH=$(uname -m)
case "$ARCH" in
    x86_64)  BINARY_ARCH="amd64" ;;
    aarch64) BINARY_ARCH="arm64" ;;
    arm64)   BINARY_ARCH="arm64" ;;
    *)       log ERROR "Nicht unterstützte Architektur: $ARCH"; exit 1 ;;
esac

log INFO "Installiere shfmt ($BINARY_ARCH)..."
TMP=$(mktemp)
URL="https://github.com/mvdan/sh/releases/download/v3.10.0/shfmt_v3.10.0_linux_${BINARY_ARCH}"
if ! curl -sLo "$TMP" "$URL"; then
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
