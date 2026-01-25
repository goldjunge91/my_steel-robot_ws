#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-$USER}}

# Prüfung: Ist NVM bereits im Home-Verzeichnis von Marco?
if [ -d "/home/$REAL_USER/.nvm" ]; then
    log SUCCESS "nvm bereits unter /home/$REAL_USER/.nvm installiert"
    exit 0
fi

log INFO "Installiere nvm für $REAL_USER..."
NVM_TMP=$(mktemp)

# Download des Installers
if ! curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh -o "$NVM_TMP"; then
    rm -f "$NVM_TMP"
    log_result fail "nvm download fehlgeschlagen"
    exit 1
fi

# --- WICHTIG: Leserechte für den User marco setzen ---
chmod 644 "$NVM_TMP"

# Installer als REAL_USER ausführen
if run_action "Run nvm installer" sudo -u "$REAL_USER" bash "$NVM_TMP"; then
    log SUCCESS "nvm installiert"
    
    # NVM für die aktuelle Root-Session verfügbar machen (falls nachfolgende Skripte Node brauchen)
    export NVM_DIR="/home/$REAL_USER/.nvm"
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
else
    log_result fail "nvm Installation fehlgeschlagen"
    rm -f "$NVM_TMP"
    exit 1
fi

rm -f "$NVM_TMP"
exit 0
