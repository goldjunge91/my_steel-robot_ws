#!/usr/bin/env bash
set -euo pipefail

# Variablen definieren
REPO_URL="https://github.com/ohmybash/oh-my-bash.git"
OH_DIR="$HOME/.oh-my-bash"
TEMPLATE_FILE="$OH_DIR/templates/bashrc.osh-template"
MY_EXAMPLE="$SCRIPT_DIR/.bashrc_example"
echo "==> Oh My Bash Installation gestartet..."

# 1. Repository klonen oder aktualisieren
if [ -d "$OH_DIR" ]; then
    echo "Repository existiert bereits, aktualisiere..."
    git -C "$OH_DIR" pull --ff-only || true
else
    echo "Klone Oh My Bash von $REPO_URL..."
    git clone --depth=1 "$REPO_URL" "$OH_DIR"
fi

# 2. Bestehende .bashrc sichern (nur wenn sie noch kein OMB enthält)
if [ -f "$HOME/.bashrc" ]; then
    if ! grep -q "source.*oh-my-bash.sh" "$HOME/.bashrc"; then
        TIMESTAMP=$(date +%s)
        echo "Sichere originale .bashrc nach .bashrc.pre-omb-$TIMESTAMP"
        cp "$HOME/.bashrc" "$HOME/.bashrc.pre-omb-$TIMESTAMP"
    fi
fi

if [ -f "$MY_EXAMPLE" ]; then
    log INFO "Installiere deine benutzerdefinierte .bashrc aus $MY_EXAMPLE"
    cp "$MY_EXAMPLE" "$HOME/.bashrc"
    
    # Pfad zum OSH Verzeichnis in der Datei korrigieren (falls nötig)
    sed -i "s@export OSH=.*@export OSH=\"$OH_DIR\"@" "$HOME/.bashrc"
    chown "$REAL_USER:$REAL_USER" "$HOME/.bashrc"
    log_result ok "Benutzerdefinierte .bashrc installiert"
else
    log WARN "Kein benutzerdefiniertes Example gefunden, nutze OMB-Standard."
    # Fallback auf Standard-Template
    [ -f "$OH_DIR/templates/bashrc.osh-template" ] && cp "$OH_DIR/templates/bashrc.osh-template" "$HOME/.bashrc"
fi
