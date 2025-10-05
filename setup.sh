# #!/bin/bash
# set -euo pipefail

# safe_source() {
#   local file="$1"
#   if [ -f "$file" ]; then
#     set +u
#     # shellcheck disable=SC1090
#     source "$file"
#     local status=$?
#     set -u
#     return $status
#   fi
#   return 1
# }

# ROS_DISTRO=${ROS_DISTRO:-humble}
# ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# # Some ROS setup scripts expect this variable to exist even when tracing is disabled.
# if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
#   AMENT_TRACE_SETUP_FILES=""
# fi
# export AMENT_TRACE_SETUP_FILES

# if safe_source "$ROS_SETUP"; then
#   :
# else
#   echo "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
# fi

# if [ -f src/ros2.repos ]; then
#   if ! command -v vcs >/dev/null 2>&1; then
#     python3 -m pip install --upgrade pip >/dev/null 2>&1 || true
#     python3 -m pip install vcstool >/dev/null 2>&1
#     VCS_BIN_DIR=$(python3 - <<'PY'
# import sys
# from pathlib import Path
# print(Path(sys.executable).resolve().parent)
# PY
# )
#     export PATH="$VCS_BIN_DIR:$PATH"
#   fi
#   vcs import src < src/ros2.repos || true
# fi

# if command -v rosdep >/dev/null 2>&1; then
#   if command -v apt-get >/dev/null 2>&1; then
#     if [ -z "${APT_UPDATED:-}" ]; then
#       sudo apt-get update || true
#       APT_UPDATED=1
#     fi
#   fi
#   rosdep update --rosdistro="$ROS_DISTRO" || true
#   rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO" || true
# else
#   echo "rosdep is not available; skipping dependency resolution." >&2
# fi
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

# Some ROS setup scripts expect this variable to exist even when tracing is disabled.
if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  AMENT_TRACE_SETUP_FILES=""
fi
export AMENT_TRACE_SETUP_FILES

if safe_source "$ROS_SETUP"; then
  :
else
  echo "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
fi

if [ -f src/ros2.repos ]; then
  if ! command -v vcs >/dev/null 2>&1; then
    python3 -m pip install --upgrade pip >/dev/null 2>&1 || true
    python3 -m pip install vcstool >/dev/null 2>&1
    VCS_BIN_DIR=$(python3 - <<'PY'
import sys
from pathlib import Path
print(Path(sys.executable).resolve().parent)
PY
)
    export PATH="$VCS_BIN_DIR:$PATH"
  fi
  vcs import src < src/ros2.repos || true
fi

if command -v rosdep >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    if [ -z "${APT_UPDATED:-}" ]; then
      # The '|| true' ensures that the script continues even if this command fails.
      # This prevents the entire build from stopping due to a transient network issue.
      # The -y and -qq flags make the output less verbose and non-interactive.
      sudo apt-get update -y -qq || true
      export APT_UPDATED=1
    fi
  fi
  # FIX: Running 'rosdep update' with sudo to initialize the cache for the root user.
  # This prevents an error when 'sudo rosdep install' is called later.
  sudo rosdep update --rosdistro="$ROS_DISTRO" || true
  # rosdep install requires sudo to install system packages.
  sudo rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO" || true
else
  echo "rosdep is not available; skipping dependency resolution." >&2
fi


## Return an finish status that run can move on
exit 0