#!/usr/bin/env bash
# run_rviz_clean.sh
# Wrapper um rviz2 in einer sauberen Library-Umgebung zu starten.
# Entfernt SNAP/Core20 Pfade aus LD_LIBRARY_PATH, setzt die ROS-Umgebung
# und startet rviz2. Nutzung:
#   ./scripts/run_rviz_clean.sh

set -eo pipefail
# Note: keep 'nounset' (set -u) disabled while sourcing ROS/colcon setup scripts
# because some setup files (install/setup.bash) reference variables that may be
# intentionally unset (e.g. COLCON_TRACE). We will avoid 'set -u' here and keep
# a defensive approach.

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Starting rviz2 with cleaned library paths..."

# Source ROS2 and workspace (if installed)
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # Prefer workspace local setup
  # shellcheck source=/dev/null
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  # Fallback to system ROS
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi

# Remove any SNAP/core20 paths from LD_LIBRARY_PATH to avoid symbol conflicts
clean_snap_env() {
  # Ensure basic system PATH so tools like 'grep' and 'dirname' exist
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

  # Unset only specific SNAP or VSCODE snap related variables by name
  for var in $(env | cut -d= -f1); do
    case "$var" in
      *VSCODE_SNAP*|*VSCODE*SNAP*|SNAP*|GIO_MODULE_DIR|GIO_LAUNCHED_DESKTOP_FILE|GTK_PATH|GTK_EXE_PREFIX|GTK_IM_MODULE_FILE|LOCPATH|XDG_DATA_HOME|XDG_DATA_DIRS)
        unset "$var" || true
        ;;
    esac
  done

  # Clean PATH and LD_LIBRARY_PATH entries that contain /snap/ but keep system paths
  if [ -n "${PATH-}" ]; then
    IFS=':' read -ra _p <<< "$PATH"
    newp=()
    for p in "${_p[@]}"; do
      case "$p" in
        */snap/*) ;;
        *) newp+=("$p") ;;
      esac
    done
    # ensure standard system paths are present
    candidate="$(IFS=:; echo "${newp[*]}")"
    case ":$candidate:" in
      *:/usr/bin:*) : ;;
      *) candidate="/usr/bin:$candidate" ;;
    esac
    export PATH="$candidate"
  fi

  if [ -n "${LD_LIBRARY_PATH-}" ]; then
    IFS=':' read -ra _lp <<< "$LD_LIBRARY_PATH"
    newlp=()
    for p in "${_lp[@]}"; do
      case "$p" in
        */snap/*) ;;
        *) newlp+=("$p") ;;
      esac
    done
    export LD_LIBRARY_PATH="$(IFS=:; echo "${newlp[*]}")"
  fi

  # unset LD_PRELOAD which may force loading wrong libs
  unset LD_PRELOAD || true

  echo "Cleaned PATH: $PATH"
  echo "Cleaned LD_LIBRARY_PATH: ${LD_LIBRARY_PATH-}"
}

clean_snap_env

# Source ROS2 and workspace (safe to source now)
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  . "${WORKSPACE_DIR}/install/setup.bash"
else
  # shellcheck source=/dev/null
  . /opt/ros/humble/setup.bash
fi

echo "Launching rviz2..."
exec ros2 run rviz2 rviz2
