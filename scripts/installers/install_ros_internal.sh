#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}
USER_HOME="/home/$REAL_USER"

# 1. OS Check & Distro Selection (Humble für Jammy, Jazzy für Noble)
local_ubuntu_codename=$(source /etc/os-release && echo "$UBUNTU_CODENAME")
ROS_DISTRO=""
case "$local_ubuntu_codename" in
    jammy) ROS_DISTRO="humble" ;;
    noble) ROS_DISTRO="jazzy" ;;
    *)
        log ERROR "Nicht unterstützte Ubuntu-Version: $local_ubuntu_codename"
        exit 1
        ;;
esac

# --- GUARD: Verhindert Mehrfach-Installation ---
if dpkg -l | grep -q "ros-$ROS_DISTRO-desktop"; then
    log SUCCESS "ROS 2 $ROS_DISTRO ist bereits installiert."
    exit 0
fi

log_step "ROS 2 SETUP ($ROS_DISTRO) - Start"

# 2. Locales (Nur wenn nötig)
if [[ "${LANG:-}" != "en_US.UTF-8" ]]; then
    run_action "install locales" apt install -y locales
    run_action "locale-gen" locale-gen en_US en_US.UTF-8
    run_action "update-locale" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
fi

# 3. Repository Key (Sichere TMP-Logik mit trap)
log_task "Lade ROS GPG-Schlüssel..."
TMP_KEY=$(mktemp)
# Cleanup-Trap: löscht die Datei garantiert beim Beenden (Erfolg oder Fehler)
trap 'rm -f "$TMP_KEY"' EXIT 

if ! curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$TMP_KEY"; then
    log_result fail "ROS key download fehlgeschlagen"
    exit 1
fi

# Schlüssel atomar an Zielort kopieren
if install -m 0644 "$TMP_KEY" /usr/share/keyrings/ros-archive-keyring.gpg; then
    log_result ok "ROS key installiert"
else
    log_result fail "Fehler beim Installieren des Schlüssels"
    exit 1
fi

# 4. Repository konfigurieren
log_task "Konfiguriere Repository"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $local_ubuntu_codename main" | tee /etc/apt/sources.list.d/ros2.list >/dev/null
log_result ok "Repository konfiguriert"

# 5. Update & Installation
wait_for_apt || exit 1
run_action "apt update (ROS)" apt update -y || log WARN "apt update (ROS) mit Warnungen"

install_and_check "ros-$ROS_DISTRO-desktop"
install_and_check "ros-dev-tools"
install_and_check "python3-argcomplete"

# 6. Rosdep initialisieren (Nutzer-Space)
log_task "Rosdep initialisieren"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init >/dev/null 2>&1 || true
fi

# Rosdep update nur wenn nötig
if [ ! -d "$USER_HOME/.ros/rosdep" ]; then
    log_task "Führe rosdep update für $REAL_USER aus..."
    sudo -u "$REAL_USER" rosdep update >/dev/null 2>&1 || log WARN "rosdep update fehlgeschlagen"
    log_result ok "Rosdep initialisiert"
else
    log_result skip "Rosdep bereits vorhanden"
fi

log SUCCESS "ROS 2 $ROS_DISTRO Installation abgeschlossen."
exit 0
