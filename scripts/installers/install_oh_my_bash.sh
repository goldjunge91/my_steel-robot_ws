#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"

log INFO "Starte Oh My Bash Modul..."

# Wir führen das Installations-Skript direkt aus. 
# Die internen sudo-Befehle im install_omb.sh regeln die Rechte.
if bash "$SCRIPT_DIR/install_omb.sh"; then
    log SUCCESS "Oh My Bash erfolgreich eingerichtet"
    exit 0
else
    log ERROR "Oh My Bash Installation fehlgeschlagen"
    exit 1
fi
