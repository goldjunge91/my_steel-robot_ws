#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"

# Sicherstellen, dass REAL_USER bekannt ist
: ${REAL_USER:=${SUDO_USER:-$USER}}
USER_HOME="/home/$REAL_USER"

REPO_URL="https://github.com/ohmybash/oh-my-bash.git"
OH_DIR="$USER_HOME/.oh-my-bash"
MY_EXAMPLE="$SCRIPT_DIR/.bashrc_example"

log_task "Oh My Bash Installation für $REAL_USER"

# 1. Repository klonen oder aktualisieren (als User!)
if [ -d "$OH_DIR" ]; then
    log_result info "Repository existiert bereits, aktualisiere..."
    sudo -u "$REAL_USER" git -C "$OH_DIR" pull --ff-only || true
else
    log_task "Klone Oh My Bash..."
    sudo -u "$REAL_USER" git clone --depth=1 "$REPO_URL" "$OH_DIR"
fi

# 2. Bestehende .bashrc sichern (als User!)
if [ -f "$USER_HOME/.bashrc" ] && ! grep -q "oh-my-bash.sh" "$USER_HOME/.bashrc"; then
    TIMESTAMP=$(date +%s)
    sudo -u "$REAL_USER" cp "$USER_HOME/.bashrc" "$USER_HOME/.bashrc.pre-omb-$TIMESTAMP"
    log_result ok "Backup der .bashrc erstellt"
fi

# 3. Deine Golden Copy (.bashrc_example) installieren
if [ -f "$MY_EXAMPLE" ]; then
    log_task "Installiere benutzerdefinierte .bashrc aus Example..."
    # Kopieren und Pfad anpassen direkt als User
    sudo -u "$REAL_USER" cp "$MY_EXAMPLE" "$USER_HOME/.bashrc"
    sudo -u "$REAL_USER" sed -i "s@export OSH=.*@export OSH=\"$OH_DIR\"@" "$USER_HOME/.bashrc"
    log_result ok "Benutzerdefinierte .bashrc installiert"
else
    log WARN "Kein benutzerdefiniertes Example gefunden, nutze OMB-Standard."
fi
