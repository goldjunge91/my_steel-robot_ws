#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

log INFO "Installiere Oh My Bash..."
if sudo -u "$REAL_USER" bash "$SCRIPT_DIR/install_omb.sh"; then
    log SUCCESS "Oh My Bash installiert"
    exit 0
else
    log WARN "Oh My Bash Installation fehlgeschlagen"
    record_failure "Oh My Bash Installation"
    exit 1
fi
