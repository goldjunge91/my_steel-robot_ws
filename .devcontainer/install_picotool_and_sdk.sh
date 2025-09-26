#!/usr/bin/env bash
set -euo pipefail

echo "--- install_picotool_and_sdk.sh: start ---"

# Destination for the Pico SDK
SDK_DEST="${1:-/opt/pico-sdk}"

# --- Pico SDK Installation ---
echo "Checking for existing Pico SDK..."
# A simplified list of candidate paths for an existing pico-sdk
CANDIDATES=(
  "${PICO_SDK_PATH:-}"
  "$SDK_DEST"
  "/usr/local/share/pico-sdk"
)

SDK=""
for p in "${CANDIDATES[@]}"; do
    if [ -n "$p" ] && [ -d "$p" ]; then
        SDK="$p"
        break
    fi
done

if [ -z "$SDK" ]; then
    echo "Pico SDK not found. Cloning into $SDK_DEST ..."
    # Create the parent directory if it doesn't exist
    mkdir -p "$(dirname "$SDK_DEST")"
    git clone --recursive --depth 1 https://github.com/raspberrypi/pico-sdk.git "$SDK_DEST"
    # git clone --depth 1 https://github.com/raspberrypi/pico-sdk.git "$SDK_DEST"
    SDK="$SDK_DEST"
fi

SDK_REAL="$(realpath "$SDK")"
echo "Using Pico SDK at: $SDK_REAL"

# Export the variable for the CURRENT script session (for the picotool build)
export PICO_SDK_PATH="$SDK_REAL"

# Persist PICO_SDK_PATH globally for FUTURE user sessions
echo "Setting PICO_SDK_PATH environment variable for all users..."
echo "export PICO_SDK_PATH='$SDK_REAL'" > /etc/profile.d/pico_sdk.sh

chmod 644 /etc/profile.d/pico_sdk.sh
# Also add to /etc/environment for system-wide access in non-interactive sessions
grep -q '^PICO_SDK_PATH=' /etc/environment 2>/dev/null || echo "PICO_SDK_PATH=$SDK_REAL" >> /etc/environment

# --- Picotool Installation ---
echo "Installing Picotool separately..."

# Ensure required build tools are available
for cmd in git cmake ninja install; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command '$cmd' not found in PATH." >&2
    exit 1
  fi
done

# Create a temporary directory for cloning and building picotool
TMP_DIR="$(mktemp -d)"
# Ensure the temporary directory is removed when the script exits
trap 'rm -rf "$TMP_DIR"' EXIT

echo "Cloning the standalone picotool repository..."
git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_DIR"

echo "Configuring picotool build..."
cmake -S "$TMP_DIR" -B "$TMP_DIR/build" -G Ninja -DCMAKE_BUILD_TYPE=Release

echo "Building picotool..."
cmake --build "$TMP_DIR/build" --target picotool -j"$(nproc)"

# Find the compiled binary
PICOTOOL_BIN="$(find "$TMP_DIR/build" -type f -name picotool -print -quit)"
if [ -z "$PICOTOOL_BIN" ] || [ ! -f "$PICOTOOL_BIN" ]; then
    echo "ERROR: picotool binary not found after build." >&2
    exit 1
fi

echo "Installing picotool to the standard user-accessible directory /usr/local/bin..."
# This directory is in the PATH for all users by default
install -Dm755 "$PICOTOOL_BIN" /usr/local/bin/picotool

echo "Picotool installed successfully."
echo "--- install_picotool_and_sdk.sh: done ---"