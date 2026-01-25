#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"

# Pfade basierend auf dem echten User berechnen (wie im Pico SDK)
OH_DIR="$USER_HOME/.oh-my-bash"
MY_EXAMPLE="$SCRIPT_DIR/.bashrc_example"

log_task "Oh My Bash Installation für $REAL_USER"

# 1. Repository klonen oder aktualisieren
if [ -d "$OH_DIR" ]; then
    log_result info "Repository existiert bereits, aktualisiere..."
    sudo -u "$REAL_USER" git -C "$OH_DIR" pull --ff-only || true
else
    log_task "Klone Oh My Bash..."
    sudo -u "$REAL_USER" git clone --depth=1 "https://github.com/ohmybash/oh-my-bash.git" "$OH_DIR"
fi

# 2. Bestehende .bashrc sichern
if [ -f "$USER_HOME/.bashrc" ] && ! grep -q "oh-my-bash.sh" "$USER_HOME/.bashrc"; then
    TIMESTAMP=$(date +%s)
    sudo -u "$REAL_USER" cp "$USER_HOME/.bashrc" "$USER_HOME/.bashrc.pre-omb-$TIMESTAMP"
    log_result ok "Backup der .bashrc erstellt"
fi

# 3. Golden Copy (.bashrc_example) installieren
if [ -f "$MY_EXAMPLE" ]; then
    log_task "Installiere benutzerdefinierte .bashrc aus Example..."
    sudo -u "$REAL_USER" cp "$MY_EXAMPLE" "$USER_HOME/.bashrc"
    
    # Pfad zum OSH Verzeichnis in der Datei korrigieren
    sudo -u "$REAL_USER" sed -i "s@export OSH=.*@export OSH=\"$OH_DIR\"@" "$USER_HOME/.bashrc"
    log_result ok "Benutzerdefinierte .bashrc für $REAL_USER installiert"
else
    log WARN "Kein benutzerdefiniertes Example gefunden, nutze OMB-Standard."
    if [ -f "$OH_DIR/templates/bashrc.osh-template" ]; then
        sudo -u "$REAL_USER" cp "$OH_DIR/templates/bashrc.osh-template" "$USER_HOME/.bashrc"
    fi
fi
