#!/usr/bin/env bash
set -uo pipefail

echo "--- install_picotool_and_sdk.sh: start ---"

# Destination for Pico SDK (default)
SDK_DEST="${1:-/opt/pico-sdk}"

FAILURES=()
SKIP_PICOTOOL=0
PICOTOOL_INSTALLED=0
TMP_DIR=""

log_info() {
  printf '[INFO] %s\n' "$*"
}

log_warn() {
  printf '[WARN] %s\n' "$*" >&2
}

record_failure() {
  FAILURES+=("$1")
}

cleanup() {
  if [ -n "${TMP_DIR:-}" ] && [ -d "${TMP_DIR}" ]; then
    rm -rf "${TMP_DIR}"
  fi
}
trap cleanup EXIT

run_command() {
  local desc="$1"
  shift
  log_info "$desc"
  if "$@"; then
    return 0
  else
    local rc=$?
    log_warn "$desc failed (exit $rc); continuing."
    record_failure "$desc (exit $rc)"
    return $rc
  fi
}

log_info "Checking for existing Pico SDK..."
CANDIDATES=("${PICO_SDK_PATH:-}" "$SDK_DEST" "/usr/local/share/pico-sdk")

SDK=""
for p in "${CANDIDATES[@]}"; do
  if [ -n "$p" ] && [ -d "$p" ]; then
    SDK="$p"
    break
  fi
done

if [ -z "$SDK" ]; then
  log_info "Pico SDK not found. Attempting to clone into ${SDK_DEST}."
  SDK_PARENT="$(dirname "$SDK_DEST")"
  if run_command "Creating parent directory ${SDK_PARENT}" mkdir -p "$SDK_PARENT"; then
    if run_command "Cloning Pico SDK repository" git clone --recursive --depth 1 https://github.com/raspberrypi/pico-sdk.git "$SDK_DEST"; then
      SDK="$SDK_DEST"
    else
      log_warn "Continuing without newly cloned Pico SDK."
      SKIP_PICOTOOL=1
    fi
  else
    log_warn "Could not create parent directory for Pico SDK; skipping clone."
    SKIP_PICOTOOL=1
  fi
fi

if [ -n "$SDK" ] && [ -d "$SDK" ]; then
  if SDK_REAL=$(realpath -s "$SDK" 2>/dev/null); then
    log_info "Using Pico SDK at: $SDK_REAL"
    export PICO_SDK_PATH="$SDK_REAL"
    if ! printf "export PICO_SDK_PATH='%s'\n" "$SDK_REAL" > /etc/profile.d/pico_sdk.sh; then
      log_warn "Failed to write /etc/profile.d/pico_sdk.sh."
      record_failure "Persisting PICO_SDK_PATH in /etc/profile.d/pico_sdk.sh"
    else
      if ! chmod 644 /etc/profile.d/pico_sdk.sh; then
        log_warn "Failed to set permissions on /etc/profile.d/pico_sdk.sh."
        record_failure "Setting permissions on /etc/profile.d/pico_sdk.sh"
      fi
    fi
    if ! grep -q '^PICO_SDK_PATH=' /etc/environment 2>/dev/null; then
      if ! printf "PICO_SDK_PATH=%s\n" "$SDK_REAL" >> /etc/environment; then
        log_warn "Failed to append PICO_SDK_PATH to /etc/environment."
        record_failure "Persisting PICO_SDK_PATH in /etc/environment"
      fi
    fi
  else
    log_warn "Unable to resolve Pico SDK path using realpath; skipping environment export."
    record_failure "Resolving Pico SDK path"
    SKIP_PICOTOOL=1
  fi
else
  log_warn "Pico SDK directory not available; skipping SDK environment setup."
  record_failure "Pico SDK setup"
  SKIP_PICOTOOL=1
fi

if [ "$SKIP_PICOTOOL" -eq 0 ]; then
  MISSING_CMDS=()
  for cmd in git cmake ninja install; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
      MISSING_CMDS+=("$cmd")
    fi
  done
  if [ "${#MISSING_CMDS[@]}" -gt 0 ]; then
    log_warn "Missing required command(s) for picotool build: ${MISSING_CMDS[*]}"
    record_failure "Missing commands for picotool: ${MISSING_CMDS[*]}"
    SKIP_PICOTOOL=1
  fi
fi

if [ "$SKIP_PICOTOOL" -eq 0 ]; then
  TMP_DIR=$(mktemp -d) || {
    log_warn "Unable to create temporary directory for picotool build."
    record_failure "Creating temporary directory for picotool"
    SKIP_PICOTOOL=1
  }
fi

if [ "$SKIP_PICOTOOL" -eq 0 ]; then
  if run_command "Cloning picotool repository" git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_DIR/picotool"; then
    if run_command "Configuring picotool build" cmake -S "$TMP_DIR/picotool" -B "$TMP_DIR/build" -G Ninja -DCMAKE_BUILD_TYPE=Release; then
      JOBS=1
      if command -v nproc >/dev/null 2>&1; then
        JOBS=$(nproc)
      fi
      if run_command "Building picotool" cmake --build "$TMP_DIR/build" --target picotool -j"$JOBS"; then
        PICOTOOL_BIN="$TMP_DIR/build/tools/picotool/picotool"
        if [ -z "$PICOTOOL_BIN" ] || [ ! -f "$PICOTOOL_BIN" ]; then
          log_warn "Picotool binary not found after build."
          record_failure "Locating built picotool binary"
          SKIP_PICOTOOL=1
        else
          if run_command "Installing picotool binary" install -Dm755 "$PICOTOOL_BIN" /usr/local/bin/picotool; then
            PICOTOOL_INSTALLED=1
          else
            SKIP_PICOTOOL=1
          fi
        fi
      else
        SKIP_PICOTOOL=1
      fi
    else
      SKIP_PICOTOOL=1
    fi
  else
    SKIP_PICOTOOL=1
  fi
fi

if [ "$PICOTOOL_INSTALLED" -eq 1 ]; then
  log_info "Picotool installed successfully."
elif [ "$SKIP_PICOTOOL" -eq 1 ]; then
  log_warn "Picotool installation skipped or incomplete due to earlier issues."
else
  log_warn "Picotool installation did not complete."
fi

if [ "${#FAILURES[@]}" -gt 0 ]; then
  log_warn "Encountered ${#FAILURES[@]} non-fatal issue(s):"
  for failure in "${FAILURES[@]}"; do
    log_warn " - ${failure}"
  done
else
  log_info "Completed all steps without errors."
fi

echo "--- install_picotool_and_sdk.sh: done ---"
exit 0
