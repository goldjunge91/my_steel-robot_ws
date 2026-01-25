#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

log INFO "Installiere ROS2..."
if sudo -u "$REAL_USER" bash "$SCRIPT_DIR/../bash/install_ros2.sh"; then
    log SUCCESS "ROS2 installiert (via script)"
    exit 0
fi

log INFO "Installationsskript fehlgeschlagen, versuche internen Installer..."
if run_action "Run internal ROS installer" bash -c 'source "$SCRIPT_DIR/../ubuntu_first_setup.sh" && install_ros' ; then
    log SUCCESS "ROS2 installiert (via internal installer)"
    exit 0
else
    log WARN "ROS2 Installation fehlgeschlagen"
    record_failure "ROS2 Installation"
    exit 1
fi
