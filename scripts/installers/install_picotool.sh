#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

if command -v picotool &>/dev/null; then
    log SUCCESS "picotool already installed"
    exit 0
fi

log INFO "Baue Picotool..."
TMP_PICO=$(mktemp -d)
chmod 755 "$TMP_PICO"

if ! run_action "Picotool klonen" git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_PICO/picotool"; then
    record_failure "Picotool clone failed"
    exit 1
fi

cd "$TMP_PICO/picotool" || { record_failure "cd to picotool failed"; exit 1; }
if ! mkdir build; then record_failure "mkdir build failed"; exit 1; fi
cd build || { record_failure "cd build failed"; exit 1; }

if ! run_action "Picotool cmake" cmake .. -DPICO_SDK_PATH="${PICO_SDK_PATH:-$USER_HOME/pico-sdk}"; then exit 1; fi
if ! run_action "Picotool build" make -j"$(nproc)"; then exit 1; fi
if ! run_action "Picotool install" make install; then exit 1; fi

cd / || true
rm -rf "$TMP_PICO"
log SUCCESS "Picotool installiert."
exit 0
