#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}
USER_HOME="/home/$REAL_USER"
PICO_DIR="$USER_HOME/pico-sdk"

log INFO "Installiere Pico SDK Abhängigkeiten..."
if ! install_and_check gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib libusb-1.0-0-dev pkg-config; then
    exit 1
fi

export PICO_SDK_PATH="$PICO_DIR"

if [ ! -d "$PICO_DIR" ]; then
    log INFO "Klone Pico SDK nach $PICO_DIR..."
    if ! run_action "clone pico-sdk" sudo -u "$REAL_USER" git clone --depth 1 --recursive https://github.com/raspberrypi/pico-sdk.git "$PICO_DIR"; then
        record_failure "clone pico-sdk failed"
        exit 1
    fi
    chown -R "$REAL_USER:$REAL_USER" "$PICO_DIR"
else
    log INFO "Pico SDK bereits vorhanden."
fi
exit 0
