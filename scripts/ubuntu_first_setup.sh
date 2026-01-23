#!/usr/bin/env bash

# ==============================================================================
# Setup Script: Ubuntu VM + ROS 2 Humble + Pico SDK + ZSH + GH CLI
# ==============================================================================

set -u -o pipefail -E

# set -m || true

# 1. Root-Check & User-Variable bestimmen
if [ "$EUID" -ne 0 ]; then
    echo "Bitte mit sudo ausführen: sudo ./vm_setup_complete.sh"
    exit 1
fi

# Der User, der sudo ausgeführt hat (nicht root)
REAL_USER=$SUDO_USER

if [ -z "$REAL_USER" ]; then
    echo "Konnte den normalen Benutzer nicht ermitteln. Bitte via sudo ausführen."
    exit 1
fi
USER_HOME="/home/$REAL_USER"

# Log-Dateien
LOG_FILE="setup.log"
ERR_FILE="errors.log"

# Wir leiten ALLES (stdout und stderr) in eine Pipe zu 'tee'.
# 'tee' schreibt es in die Datei UND auf den Bildschirm.
exec > >(tee -a "$LOG_FILE") 2>&1

echo "=== Setup gestartet am $(date) ==="

# Farbdefinitionen
RED='\033[31m'
GREEN='\033[32m'
BLUE='\033[34m'
RESET='\033[0m'

# --- KONFIGURATION ---
PICO_DIR="$USER_HOME/pico-sdk"

# --- Hilfsfunktionen ---

log() {
    local level="$1"
    shift
    local color="${RESET}"
    [ "$level" = "INFO" ] && color="${BLUE}"
    [ "$level" = "SUCCESS" ] && color="${GREEN}"
    [ "$level" = "ERROR" ] && color="${RED}"
    # echo -e "${color}[$level]${RESET} $*"
    # echo "$(date): $level: $*" >>"$LOG_FILE" 2>/dev/null || true
    # Wir machen hier kein >> $LOG_FILE mehr, da das globale exec das übernimmt!
    echo -e "${color}[$level]${RESET} $*"
}

wait_for_apt() {
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 ||
        fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
        echo -e "${BLUE}Warte auf apt-Lock...${RESET}"
        sleep 1
    done
}

install_and_check() {
    local pkg="$1"
    if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
        echo -e "${GREEN}✓ $pkg ist bereits installiert${RESET}"
        return 0
    fi
    wait_for_apt
    echo -e "${BLUE}... Installiere $pkg${RESET}"
    if DEBIAN_FRONTEND=noninteractive apt install -y "$pkg" >>"$LOG_FILE" 2>&1; then
        echo -e "${GREEN}✓ $pkg installiert${RESET}"
    else
        echo -e "${RED}✗ $pkg fehlgeschlagen${RESET}"
        echo "$(date): $pkg failed" >>"$ERR_FILE"
        return 1
    fi
}
# Funktion um Dinge in .bashrc UND .zshrc zu schreiben
add_to_shells() {
    local content="$1"
    local specific_file="${2:-both}" # both, bash, or zsh

    # .bashrc
    if [[ "$specific_file" == "both" || "$specific_file" == "bash" ]]; then
        local b_rc="$USER_HOME/.bashrc"
        if ! grep -Fq "$content" "$b_rc"; then
            echo "$content" >>"$b_rc"
            chown "$REAL_USER:$REAL_USER" "$b_rc"
            log INFO "Added to .bashrc: $content"
        fi
    fi

    # .zshrc
    if [[ "$specific_file" == "both" || "$specific_file" == "zsh" ]]; then
        local z_rc="$USER_HOME/.zshrc"
        # Falls nicht existiert, anlegen
        if [ ! -f "$z_rc" ]; then
            touch "$z_rc"
            chown "$REAL_USER:$REAL_USER" "$z_rc"
        fi
        if ! grep -Fq "$content" "$z_rc"; then
            echo "$content" >>"$z_rc"
            chown "$REAL_USER:$REAL_USER" "$z_rc"
            log INFO "Added to .zshrc: $content"
        fi
    fi
}
cleanup() {
    # Exit Code speichern
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        echo -e "\n${RED}Skript wurde unterbrochen oder fehlerhaft beendet.${RESET}"
    fi
    # Prozesse aufräumen
    pkill -P $$ 2>/dev/null || true
}

trap cleanup EXIT

# ==============================================================================
# --- INSTALLATIONS FUNKTIONEN ---
# ==============================================================================

install_picotool() {
    # Picotool bauen
    if ! command -v picotool &>/dev/null; then
        log INFO "Baue Picotool..."
        TMP_PICO=$(mktemp -d)
        chmod 777 "$TMP_PICO"

        # Klonen & Bauen als Root (im Temp), Install nach /usr/local/bin
        git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_PICO/picotool"
        cd "$TMP_PICO/picotool" || exit
        mkdir build && cd build || exit
        cmake .. -DPICO_SDK_PATH="$PICO_DIR"
        make -j$(nproc)
        make install
        cd /
        rm -rf "$TMP_PICO"
        log SUCCESS "Picotool installiert."
    else
        log SUCCESS "Picotool ist bereits installiert."
    fi

}

install_pico_sdk() {
    log INFO "Installiere Pico SDK Abhängigkeiten..."
    PICO_DEPS=(
        gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
        libusb-1.0-0-dev pkg-config
    )
    for pkg in "${PICO_DEPS[@]}"; do install_and_check "$pkg"; done

    export PICO_SDK_PATH="$PICO_DIR"

    if [ ! -d "$PICO_DIR" ]; then
        log INFO "Klone Pico SDK nach $PICO_DIR..."
        # Als normaler User klonen, damit Rechte stimmen
        sudo -u "$REAL_USER" git clone --depth 1 --recursive https://github.com/raspberrypi/pico-sdk.git "$PICO_DIR"
    else
        log INFO "Pico SDK bereits vorhanden."
    fi
}

install_formatter() {
    if ! command -v shfmt &> /dev/null; then
        log INFO "Installiere shfmt..."        
        # Download Binary
        curl -sLo /tmp/shfmt https://github.com/mvdan/sh/releases/download/v3.10.0/shfmt_v3.10.0_linux_amd64
        chmod +x /tmp/shfmt
        mv /tmp/shfmt /usr/local/bin/shfmt
        log SUCCESS "shfmt installiert"
    else
        log SUCCESS "shfmt ist bereits installiert."
    fi
}

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
log INFO "Setup gestartet für User: $REAL_USER"

# 1. System Update & Basis Tools
wait_for_apt
apt update -y && apt upgrade -y
install_and_check "software-properties-common"
add-apt-repository universe -y

TOOLS=(
    curl git gnupg2 lsb-release build-essential cmake
    python3-pip htop net-tools terminator shellcheck nano wget
    zsh fontconfig python3-vcstool
)
for tool in "${TOOLS[@]}"; do install_and_check "$tool"; done

install_formatter

# 2. GitHub CLI (gh)
if ! command -v gh &>/dev/null; then
    log INFO "Installiere GitHub CLI..."
    mkdir -p -m 755 /etc/apt/keyrings
    wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | tee /etc/apt/keyrings/githubcli-archive-keyring.gpg >/dev/null
    chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list >/dev/null
    wait_for_apt
    apt update && install_and_check "gh"
fi


# ==============================================================================
# INSTALL Pico SDK und Tools
# ==============================================================================
install_pico_sdk
install_picotool

# ==============================================================================
# 4. ROS 2 Humble Installation
# ==============================================================================
log INFO "Installiere ROS 2 Humble..."
# Locales
apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Repo
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" | tee /etc/apt/sources.list.d/ros2.list >/dev/null
wait_for_apt
apt update
install_and_check "ros-humble-desktop"
install_and_check "ros-dev-tools"

# Rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
sudo -u "$REAL_USER" rosdep update

# 5. Konfiguration der Shells (.bashrc und .zshrc)
log INFO "Konfiguriere Shell Pfade..."

# Pico SDK Variable
add_to_shells "export PICO_SDK_PATH=\"$PICO_DIR\"" "both"

# ROS 2 Sourcing (Unterschiedlich für Bash/Zsh)
add_to_shells "# ROS 2 Setup" "both"
add_to_shells "source /opt/ros/humble/setup.bash" "bash"
add_to_shells "source /opt/ros/humble/setup.zsh" "zsh"

# Colcon Autocomplete
add_to_shells "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" "bash"
add_to_shells "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" "zsh"

# # 6. Swap File (2GB) für Compiling in VM
# if [ ! -f /swapfile ]; then
#     log INFO "Erstelle 2GB Swapfile..."
#     fallocate -l 2G /swapfile
#     chmod 600 /swapfile
#     mkswap /swapfile
#     swapon /swapfile
#     echo '/swapfile none swap sw 0 0' | tee -a /etc/fstab
# fi

log SUCCESS "=========================================="
log SUCCESS " Setup abgeschlossen!"
log SUCCESS " ZSH installiert. Um zu wechseln: chsh -s \$(which zsh)"
log SUCCESS " Pico SDK Pfad: $PICO_DIR"
log SUCCESS " Bitte VM neu starten oder ausloggen."
log SUCCESS "=========================================="
