#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

USER_HOME="/home/$REAL_USER"

if command -v picotool &>/dev/null; then
    log SUCCESS "picotool already installed"
    exit 0
fi
# --- Aktive Suche nach dem SDK-Pfad ---
# Falls das Hauptskript die Variable nicht exportiert hat, setzen wir sie hier manuell
# --- SDK Pfad Validierung ---
if [ -z "${PICO_SDK_PATH:-}" ]; then
    export PICO_SDK_PATH="$USER_HOME/pico-sdk"
    log INFO "PICO_SDK_PATH war leer, verwende Standard: $PICO_SDK_PATH"
fi

if [ ! -d "$PICO_SDK_PATH" ]; then
    log ERROR "Pico SDK nicht gefunden unter $PICO_SDK_PATH!"
    log INFO "Bitte stelle sicher, dass install_pico_sdk.sh vorher erfolgreich lief."
    exit 1
fi

log INFO "Baue Picotool..."
TMP_PICO=$(mktemp -d)
trap 'rm -rf "$TMP_PICO"' EXIT
chmod 755 "$TMP_PICO"

if ! run_action "Picotool klonen" git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_PICO/picotool"; then
    record_failure "Picotool clone failed"
    exit 1
fi

cd "$TMP_PICO/picotool" || { record_failure "cd to picotool failed"; exit 1; }
# Hier nutzen wir mkdir -p zur Sicherheit, falls der Ordner doch existiert
if ! mkdir -p build; then record_failure "mkdir build failed"; exit 1; fi
cd build || { record_failure "cd build failed"; exit 1; }

# --- CMake-Aufruf ---
if ! run_action "Picotool cmake" cmake .. -DPICO_SDK_PATH="$PICO_SDK_PATH"; then
    log ERROR "Cmake fehlgeschlagen. Ist das SDK unter $PICO_SDK_PATH vorhanden?"
    record_failure "Picotool cmake failed"
    exit 1
fi

if ! run_action "Picotool build" make -j"$(nproc)"; then 
    record_failure "Picotool make failed"
    exit 1 
fi

if ! run_action "Picotool install" make install; then 
    record_failure "Picotool install failed"
    exit 1 
fi

log SUCCESS "Picotool erfolgreich installiert."
exit 0
