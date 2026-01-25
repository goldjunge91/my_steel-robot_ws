#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}
# --- NEU: Sicherstellen, dass USER_HOME definiert ist ---
USER_HOME="/home/$REAL_USER"
PICO_DIR="$USER_HOME/pico-sdk"

log INFO "Installiere Pico SDK Abhängigkeiten..."
# --- ERGÄNZT: cmake und build-essential für den späteren Picotool-Bau ---
if ! install_and_check gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib libusb-1.0-0-dev pkg-config cmake build-essential; then
    exit 1
fi
# Diese Variable ist nur für dieses Skript aktiv
export PICO_SDK_PATH="$PICO_DIR"

if [ ! -d "$PICO_DIR" ]; then
    log INFO "Klone Pico SDK nach $PICO_DIR..."
    if ! run_action "clone pico-sdk" sudo -u "$REAL_USER" git clone --depth 1 --recursive https://github.com/raspberrypi/pico-sdk.git "$PICO_DIR"; then
        record_failure "clone pico-sdk failed"
        exit 1
    fi
    # chown ist hier wichtig, falls sudo-Rechte das Verzeichnis "root" gegeben haben
    chown -R "$REAL_USER:$REAL_USER" "$PICO_DIR"
else
    log SUCCESS "Pico SDK bereits vorhanden."
fi
exit 0
