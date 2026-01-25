#!/usr/bin/env bash
set -euo pipefail
# Script directory bestimmen
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Helpers laden für log/log_result
. "$SCRIPT_DIR/helpers.sh"

REPO_URL="https://github.com/ohmybash/oh-my-bash.git"
OH_DIR="$HOME/.oh-my-bash"
# Pfad zu DEINER perfekten Example-Datei
MY_EXAMPLE="$SCRIPT_DIR/.bashrc_example"

log_task "Oh My Bash Installation"

# 1. Repository klonen oder aktualisieren
if [ -d "$OH_DIR" ]; then
    log_result info "Repository existiert bereits, aktualisiere..."
    git -C "$OH_DIR" pull --ff-only || true
else
    log_task "Klone Oh My Bash..."
    git clone --depth=1 "$REPO_URL" "$OH_DIR"
fi

# 2. Bestehende .bashrc sichern
if [ -f "$HOME/.bashrc" ] && ! grep -q "oh-my-bash.sh" "$HOME/.bashrc"; then
    TIMESTAMP=$(date +%s)
    cp "$HOME/.bashrc" "$HOME/.bashrc.pre-omb-$TIMESTAMP"
    log_result ok "Backup der .bashrc erstellt"
fi

# 3. Deine Golden Copy (.bashrc_example) installieren
if [ -f "$MY_EXAMPLE" ]; then
    log_task "Installiere benutzerdefinierte .bashrc aus Example..."
    cp "$MY_EXAMPLE" "$HOME/.bashrc"
    
    # Pfad zum OSH Verzeichnis in der Datei korrigieren
    sed -i "s@export OSH=.*@export OSH=\"$OH_DIR\"@" "$HOME/.bashrc"
    
    # Sicherstellen, dass der User Marco Besitzer der Datei ist
    chown "${REAL_USER:-$USER}:" "$HOME/.bashrc"
    log_result ok "Benutzerdefinierte .bashrc installiert"
else
    log WARN "Kein benutzerdefiniertes Example gefunden, nutze OMB-Standard."
    [ -f "$OH_DIR/templates/bashrc.osh-template" ] && cp "$OH_DIR/templates/bashrc.osh-template" "$HOME/.bashrc"
fi
