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

# Download des offiziellen Install-Skripts
if ! curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh -o "$JUST_TMP"; then
    rm -f "$JUST_TMP"
    log WARN "just download fehlgeschlagen"
    exit 1
fi

chmod +x "$JUST_TMP"

# Korrektur: Das zusätzliche "--" wurde entfernt
# Wir nutzen sudo direkt hier, falls das Hauptskript die Rechte nicht vererbt hat
if run_action "Run just installer" sudo bash "$JUST_TMP" --to /usr/local/bin; then
    log SUCCESS "just installiert"
else
    log ERROR "just Installation fehlgeschlagen"
    rm -f "$JUST_TMP"
    exit 1
fi

rm -f "$JUST_TMP"
exit 0
