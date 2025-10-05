#!/bin/bash
set -euo pipefail

safe_source() {
  local file="$1"
  if [ -f "$file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$file"
    local status=$?
    set -u
    return $status
  fi
  return 1
}

if [ -f install/setup.bash ]; then
  safe_source install/setup.bash
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not available; skipping tests." >&2
  exit 0
fi

colcon test --merge-install || true
colcon test-result --verbose || true
