#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

if [ -d "${REAL_USER:+/home/$REAL_USER}/.nvm" ] || [ -d "$HOME/.nvm" ]; then
    log SUCCESS "nvm bereits installiert"
    exit 0
fi

log INFO "Installiere nvm..."
NVM_TMP=$(mktemp)
if ! curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh -o "$NVM_TMP"; then
    rm -f "$NVM_TMP"
    log_result fail "nvm download fehlgeschlagen"
    exit 1
fi
if run_action "Run nvm installer" sudo -u "$REAL_USER" bash "$NVM_TMP"; then
    log SUCCESS "nvm installiert"
    export NVM_DIR="/home/$REAL_USER/.nvm"
    [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" || true
else
    log_result fail "nvm Installation fehlgeschlagen"
    rm -f "$NVM_TMP"
    exit 1
fi
rm -f "$NVM_TMP"
exit 0
