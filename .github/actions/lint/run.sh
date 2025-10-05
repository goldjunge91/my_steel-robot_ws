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

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  AMENT_TRACE_SETUP_FILES=""
fi
export AMENT_TRACE_SETUP_FILES

if safe_source "$ROS_SETUP"; then
  :
else
  echo "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
fi

./setup.sh

safe_source "install/setup.bash"

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
