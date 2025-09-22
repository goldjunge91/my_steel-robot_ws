#!/usr/bin/env bash
# Keep -u off because ROS setup scripts may reference unset vars
set -eo pipefail

# Simple command hub to clean, build, source, and run sim/tests
# Usage examples:
#   ./scripts/robot_ws_commands.sh clean
#   ./scripts/robot_ws_commands.sh build-core
#   ./scripts/robot_ws_commands.sh source
#   ./scripts/robot_ws_commands.sh sim-headless
#   ./scripts/robot_ws_commands.sh sim-gui
#   ./scripts/robot_ws_commands.sh controllers
#   ./scripts/robot_ws_commands.sh drive-test

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

ros_setup() {
  # Base ROS env
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
  fi
  # Workspace overlay
  if [ -f "$ROOT_DIR/install/setup.bash" ]; then
    source "$ROOT_DIR/install/setup.bash"
  fi

  # Preserve X11/Wayland display variables for GUI launches inside the container.
  # If the host provided DISPLAY/XAUTHORITY, keep them; otherwise provide a reasonable default.
  if [ -n "${DISPLAY:-}" ]; then
    export DISPLAY="$DISPLAY"
  else
    export DISPLAY=":0"
  fi

  if [ -n "${XAUTHORITY:-}" ]; then
    export XAUTHORITY="$XAUTHORITY"
  else
    # Use user .Xauthority if available in the container
    if [ -f "$HOME/.Xauthority" ]; then
      export XAUTHORITY="$HOME/.Xauthority"
    fi
  fi
}

cmd_clean() {
  echo "[CLEAN] Removing build/install/log"
  rm -rf build install log
}

cmd_build_core() {
  ros_setup
  # Ensure we don't have stale build/install symlink artifacts that previously
  # caused Errno 17 during the symlink_data step.
  if [ -x "$ROOT_DIR/scripts/cleanup_symlinks.sh" ]; then
    "$ROOT_DIR/scripts/cleanup_symlinks.sh" || true
  fi
  echo "[BUILD] Building core packages: robot, robot_description, robot_controller"
  colcon build \
    --packages-select robot robot_description robot_controller \
    --merge-install --symlink-install \
    --event-handlers console_direct+
}

cmd_source() {
  ros_setup
  echo "[ENV] Sourced /opt/ros/humble and install/setup.bash"
}

cmd_sim_headless() {
  ros_setup
  echo "[LAUNCH] Headless Gazebo + controllers"
  # Prevent leftover Gazebo instances from causing 'Address already in use' errors
  pkill -f gzclient || true
  pkill -f gzserver || true
  sleep 0.2
  ros2 launch robot launch_sim.launch.py use_sim_time:=true headless:=true
}

cmd_sim_gui() {
  ros_setup
  echo "[LAUNCH] Gazebo client + RViz (requires GUI passthrough)"
  # Use software GL by default inside containers to avoid GPU-related crashes
  export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
  # Kill leftover gazebo processes to avoid port/entity conflicts
  pkill -f gzclient || true
  pkill -f gzserver || true
  sleep 0.2
  ros2 launch robot launch_sim.launch.py use_sim_time:=true headless:=false
}

cmd_description_only() {
  ros_setup
  echo "[LAUNCH] Publish robot_description only (no Gazebo)"
  ros2 launch robot_description load_urdf.launch.py configuration:=manipulation robot_model:=robot_xl use_sim:=True
}

cmd_controllers() {
  ros_setup
  ros2 control list_controllers || true
}

cmd_topics() {
  ros_setup
  ros2 topic list | sort
}

cmd_drive_test() {
  ros_setup
  echo "[TEST] Publishing cmd_vel to drive_controller (Ctrl-C to stop)"
  ros2 topic pub -r 5 /drive_controller/cmd_vel_unstamped geometry_msgs/Twist '{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}'
}

cmd_help() {
  cat <<EOF
Usage: $0 <command>

Commands:
  clean             Remove build/install/log
  build-core        Build robot, robot_description, robot_controller
  source            Source ROS and workspace overlays
  sim-headless      Launch Gazebo + controllers without GUI
  sim-gui           Launch Gazebo client + RViz (needs GUI)
  description-only  Publish URDF via robot_state_publisher
  controllers       List active controllers
  topics            List ROS 2 topics
  drive-test        Send a forward Twist to drive_controller
  help              Show this help

Typical flow:
  $0 clean && $0 build-core && $0 source && $0 sim-headless
EOF
}

case "${1:-help}" in
  clean)             cmd_clean ;;
  build-core)        cmd_build_core ;;
  source)            cmd_source ;;
  sim-headless)      cmd_sim_headless ;;
  sim-gui)           cmd_sim_gui ;;
  description-only)  cmd_description_only ;;
  controllers)       cmd_controllers ;;
  topics)            cmd_topics ;;
  drive-test)        cmd_drive_test ;;
  help|*)            cmd_help ;;
esac
