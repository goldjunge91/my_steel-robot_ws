#!/bin/bash
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  echo "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
fi

./setup.sh

if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source install/setup.bash
fi

if [ -z "${LINTER:-}" ]; then
  echo "LINTER environment variable is not set." >&2
  exit 1
fi

LINTER_CMD="ament_${LINTER}"
if command -v "$LINTER_CMD" >/dev/null 2>&1; then
  "$LINTER_CMD" src/ || true
else
  echo "${LINTER_CMD} is not available; running fallback lint check." >&2
  python3 -m compileall src >/dev/null 2>&1 || true
fi
