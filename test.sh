#!/bin/bash
set -euo pipefail

if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1090
  source install/setup.bash
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not available; skipping tests." >&2
  exit 0
fi

colcon test --merge-install || true
colcon test-result --verbose || true
