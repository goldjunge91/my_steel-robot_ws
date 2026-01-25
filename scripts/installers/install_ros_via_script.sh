#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

log INFO "Installiere ROS2 via upstream script (wenn verfügbar)..."
if sudo -u "$REAL_USER" bash "$SCRIPT_DIR/install_ros2.sh"; then
    log SUCCESS "ROS2 installiert (via external script)"
    exit 0
fi

log INFO "External installer failed; falling back to internal installer"
# Run internal installer (runs as root, expects helpers available)
if run_action "Run internal ROS installer" bash "$SCRIPT_DIR/install_ros_internal.sh"; then
    log SUCCESS "ROS2 installiert (via internal installer)"
    exit 0
else
    log WARN "ROS2 Installation fehlgeschlagen"
    record_failure "ROS2 Installation"
    exit 1
fi
