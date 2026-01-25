#!/usr/bin/env bash
set -euo pipefail

# Variablen definieren
REPO_URL="https://github.com/ohmybash/oh-my-bash.git"
OH_DIR="$HOME/.oh-my-bash"
TEMPLATE_FILE="$OH_DIR/templates/bashrc.osh-template"

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

# 3. Neue .bashrc aus Template erstellen
if [ -f "$TEMPLATE_FILE" ]; then
    echo "Erstelle neue .bashrc aus Template..."
    cp "$TEMPLATE_FILE" "$HOME/.bashrc"
    
    # Pfad zum OSH Verzeichnis in der Datei korrigieren
    # Wir nutzen @ als Trenner für sed, falls Pfade Slashes enthalten
    sed -i "s@export OSH=.*@export OSH=\"$OH_DIR\"@" "$HOME/.bashrc"
    
    echo "✓ Oh My Bash Template wurde erfolgreich nach ~/.bashrc kopiert."
else
    echo "ERROR: Template Datei nicht gefunden: $TEMPLATE_FILE" >&2
    exit 1
fi
