#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}
USER_HOME="/home/$REAL_USER"

log_step "ROS 2 SETUP (internal)"

# OS Check & Distro Selection
local_ubuntu_codename=$(source /etc/os-release && echo "$UBUNTU_CODENAME")
ROS_DISTRO=""
case "$local_ubuntu_codename" in
    jammy) ROS_DISTRO="humble" ;;
    noble) ROS_DISTRO="jazzy" ;;
    *)
        log ERROR "Nicht unterstützte Ubuntu-Version: $local_ubuntu_codename (erwarte jammy oder noble)"
        exit 1
        ;;
esac

log_result info "Ubuntu $local_ubuntu_codename -> ROS 2 $ROS_DISTRO"

# Locales
run_action "install locales" apt install -y locales || exit 1
run_action "locale-gen" locale-gen en_US en_US.UTF-8 || exit 1
run_action "update-locale" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || exit 1
export LANG=en_US.UTF-8

# Repository key: download to temp and move atomically
TMP_KEY=$(mktemp)
if ! curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$TMP_KEY"; then
    rm -f "$TMP_KEY"
    log_result fail "ROS key download"
    exit 1
fi
install -m 0644 "$TMP_KEY" /usr/share/keyrings/ros-archive-keyring.gpg || { rm -f "$TMP_KEY"; log_result fail "Install ROS key"; exit 1; }
rm -f "$TMP_KEY"
log_result ok "ROS key installed"

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $local_ubuntu_codename main" | tee /etc/apt/sources.list.d/ros2.list >/dev/null
log_result ok "Repository konfiguriert"

wait_for_apt || exit 1
run_action "apt update (ROS)" apt update -y || log WARN "apt update (ROS) had errors, proceeding..."

# Install ROS packages
run_action "Install ROS desktop" apt install -y "ros-$ROS_DISTRO-desktop" || { log_result fail "ros-$ROS_DISTRO-desktop"; exit 1; }
run_action "Install ros-dev-tools" apt install -y ros-dev-tools || { log_result fail "ros-dev-tools"; exit 1; }

# Rosdep
log_task "Rosdep initialisieren"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    if ! rosdep init >/dev/null 2>&1; then
        log WARN "rosdep init fehlgeschlagen"
    fi
fi
if ! sudo -u "$REAL_USER" rosdep update >/dev/null 2>&1; then
    log WARN "rosdep update fehlgeschlagen"
fi
log_result ok "Rosdep initialisiert"

exit 0
