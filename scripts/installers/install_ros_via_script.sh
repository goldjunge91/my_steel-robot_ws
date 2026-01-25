#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-$USER}}

# Grobe Prüfung vorab (spart den Aufruf des Sub-Skripts)
if dpkg -l | grep -q "ros-humble-desktop" || dpkg -l | grep -q "ros-jazzy-desktop"; then
    log SUCCESS "ROS 2 ist bereits auf dem System vorhanden."
    exit 0
fi

log INFO "Starte ROS 2 Installation via internes Skript..."
if bash "$SCRIPT_DIR/install_ros_internal.sh"; then
    log SUCCESS "ROS 2 Installation erfolgreich abgeschlossen."
    exit 0
else
    log ERROR "Fehler bei der internen ROS 2 Installation."
    record_failure "ROS2 Installation"
    exit 1
fi
