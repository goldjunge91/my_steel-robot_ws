#!/usr/bin/env bash

# ==============================================================================
# Setup Script: Ubuntu + ROS 2 Humble + Pico SDK + ZSH + GH CLI
# ==============================================================================

set -u -o pipefail -E

# set -m || true

# --- KONFIGURATION ---
REAL_USER=$SUDO_USER
USER_HOME="/home/$REAL_USER"
PICO_DIR="$USER_HOME/pico-sdk"
FAILURES=()
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TOOLS=(
    curl git gnupg2 lsb-release build-essential cmake
    python3-pip htop net-tools terminator shellcheck nano wget
    zsh fontconfig python3-vcstool ca-certificates curl gnupg joystick jstest-gtk evtest
)

# Farbdefinitionen
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
RESET='\033[0m'  

# 1. Root-Check & User-Variable bestimmen
if [ "$EUID" -ne 0 ]; then
    echo "Bitte mit sudo ausführen: sudo ./vm_setup_complete.sh"
    exit 1
fi

# Der User, der sudo ausgeführt hat (nicht root)
if [ -z "$REAL_USER" ] || [ "$REAL_USER" = "root" ]; then
    echo "Konnte den normalen Benutzer nicht ermitteln. Bitte via sudo ausführen."
    exit 1
fi
# Log files
LOG_FILE="$SCRIPT_DIR/setup_$(date +%Y%m%d_%H%M%S).log"
ERR_FILE="$SCRIPT_DIR/errors_$(date +%Y%m%d_%H%M%S).log"

# Wir leiten ALLES (stdout und stderr) in eine Pipe zu 'tee'.
# exec > >(tee -a "$LOG_FILE") 2>&1
# 'tee' schreibt es in die Datei UND auf den Bildschirm.
# Redirect ALL output (exec runs ONCE, not inside functions)
exec > >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$ERR_FILE" >&2)

echo "=== Setup gestartet am $(date) ==="

# ==============================================================================
# --- Hilfsfunktionen ---
# ==============================================================================

log() {
    local level="$1"
    shift
    local color="${RESET}"
    # [ "$level" = "INFO" ] && color="${BLUE}"
    # [ "$level" = "SUCCESS" ] && color="${GREEN}"
    # [ "$level" = "ERROR" ] && color="${RED}"
    case "$level" in
        INFO) color="${BLUE}" ;;
        SUCCESS) color="${GREEN}" ;;
        ERROR) color="${RED}" ;;
        WARN) color="${YELLOW}" ;;
    esac
    echo -e "${color}[$level]${RESET} $*"
    # echo "$(date): $level: $*" >>"$LOG_FILE" 2>/dev/null || true
    # Wir machen hier kein >> $LOG_FILE mehr, da das globale exec das übernimmt!
    # echo -e "${color}[$level]${RESET} $*"
}

# Hierarchische Log-Level
log_step() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════╗${RESET}"
    echo -e "${BLUE}║ $*${RESET}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${RESET}"
}

log_task() {
    echo -e "${BLUE}→${RESET} $*"
}

log_result() {
    local status="$1"
    shift
    case "$status" in
        ok)   echo -e "  ${GREEN}✓${RESET} $*" ;;
        skip) echo -e "  ${YELLOW}○${RESET} $*" ;;
        fail) echo -e "  ${RED}✗${RESET} $*"; record_failure "$*" ;;
        info) echo -e "  ${BLUE}ℹ${RESET} $*" ;;
    esac
}

record_failure() {
    FAILURES+=("$*")
    echo "$(date): FAILURE: $*" >>"$ERR_FILE" 2>/dev/null || true
    log ERROR "$*"

}

wait_for_apt() {
    local timeout=300
    local elapsed=0
    while fuser /var/lib/dpkg/lock >/dev/null 2>&1 ||
            fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 ||
            fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
        if [ $elapsed -ge $timeout ]; then
            log ERROR "apt lock timeout after ${timeout}s"
            record_failure "apt lock timeout"
            return 1
        fi
        echo -e "${BLUE}Warte auf apt-Lock...${RESET}"
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 0
}

run_command() {
    local description="$1"
    shift
    echo -e "${BLUE}[CMD]${RESET} $description"
    log INFO "$description"
    if "$@" 2>&1 | tee -a command.log; then
        echo -e "${GREEN}✓ $description erfolgreich${RESET}"
        echo "$(date): $description succeeded" >>"$LOG_FILE" 2>/dev/null || true
        log SUCCESS "$description erfolgreich"
        return 0
    else
        echo -e "${RED}✗ $description fehlgeschlagen${RESET}"
        echo "$(date): $description failed" >>"$ERR_FILE" 2>/dev/null || true
        log ERROR "$description fehlgeschlagen"
        record_failure "$description"
        return 1
    fi
}

# run_command() {
#     local description="$1"
#     shift
#     log INFO "$description"
    
#     local tmplog
#     tmplog=$(mktemp)
    
#     if "$@" > "$tmplog" 2>&1; then
#         cat "$tmplog"
#         rm -f "$tmplog"
#         log SUCCESS "$description completed"
#         return 0
#     else
#         local ret=$?
#         cat "$tmplog"
#         rm -f "$tmplog"
#         log ERROR "$description failed (exit $ret)"
#         record_failure "$description"
#         return $ret
#     fi
# }

# Install funktion benötigt paketname als Argument
install_and_check() {
    local packages=("$@")
    local missing=()
    echo -e "${BLUE}[STEP] ${packages[*]} installieren${RESET}"
    # Laufzeit-Cache initialisieren (verhindert doppelte Prüfungen)
    if [ -z "${INSTALL_CACHE+x}" ]; then
        INSTALL_CACHE=""
    fi

    local pkg
    for pkg in "${packages[@]}"; do
        # Wenn bereits im Cache, überspringen
        if [[ " $INSTALL_CACHE " == *" $pkg "* ]]; then
            echo -e "${GREEN}✓ $pkg bereits geprüft (Cache)${RESET}"
            continue
        fi

        # Robustere Prüfung, ob Paket bereits installiert ist
        if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            echo -e "${GREEN}✓ $pkg bereits installiert${RESET}"
            echo "$(date): $pkg bereits installiert" >>"$LOG_FILE"
            INSTALL_CACHE="$INSTALL_CACHE $pkg"
        else
            missing+=("$pkg")
        fi
    done

    # Nichts zu tun
    if [ ${#missing[@]} -eq 0 ]; then
        echo -e "${GREEN}✓ Keine neuen Pakete zu installieren${RESET}"
        return 0
    fi

    # Batch-Installation der fehlenden Pakete
    wait_for_apt
    if DEBIAN_FRONTEND=noninteractive apt install -y "${missing[@]}" 2>&1 | tee -a apt_install.log; then
        local failed=()
        for pkg in "${missing[@]}"; do
            if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
                echo -e "${GREEN}✓ $pkg erfolgreich installiert${RESET}"
                echo "$(date): $pkg erfolgreich installiert" >>"$LOG_FILE"
                INSTALL_CACHE="$INSTALL_CACHE $pkg"
            else
                # Meta-Paket-Fallback prüfen
                if grep -qE "$pkg.*(is already the newest version|newly installed)" apt_install.log; then
                    echo -e "${GREEN}✓ $pkg scheint installiert (Meta/keine Aktion)${RESET}"
                    echo "$(date): $pkg installiert (Meta/keine Aktion)" >>"$LOG_FILE"
                    INSTALL_CACHE="$INSTALL_CACHE $pkg"
                else
                    echo -e "${RED}✗ $pkg Installation fehlgeschlagen${RESET}"
                    echo "$(date): $pkg Installation fehlgeschlagen" >>"$ERR_FILE"
                    failed+=("$pkg")
                fi
            fi
        done

        [ ${#failed[@]} -eq 0 ]
        return $?
    else
        # Wenn Batch fehlschlägt: Einzelinstallation für bessere Diagnose
        echo -e "${RED}✗ Batch-Installation fehlgeschlagen, versuche Einzelinstallation${RESET}"
        for pkg in "${missing[@]}"; do
            wait_for_apt || return 1
            if DEBIAN_FRONTEND=noninteractive apt install -y "$pkg" 2>&1 | tee -a apt_install.log; then
                if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
                    echo -e "${GREEN}✓ $pkg erfolgreich installiert${RESET}"
                    echo "$(date): $pkg erfolgreich installiert" >>"$LOG_FILE"
                    INSTALL_CACHE="$INSTALL_CACHE $pkg"
                else
                    echo -e "${RED}✗ $pkg Installation fehlgeschlagen${RESET}"
                    record_failure "$pkg Installation"
                    return 1
                fi
            else
                echo -e "${RED}✗ $pkg Installation fehlgeschlagen${RESET}"
                record_failure "$pkg Installation"
                return 1
            fi
        done
        return 0
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
    if [ "$exit_code" -ne 0 ]; then
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
    if command -v picotool &>/dev/null; then
        log SUCCESS "picotool already installed"
        return 0
    fi

    log INFO "Baue Picotool..."
    local TMP_PICO
    TMP_PICO=$(mktemp -d)
    chmod 755 "$TMP_PICO"

    run_command "Picotool klonen" \
        git clone --depth 1 https://github.com/raspberrypi/picotool.git "$TMP_PICO/picotool" || {
        record_failure "Picotool clone failed"
        return 1
    }

    cd "$TMP_PICO/picotool" || {
        record_failure "cd to picotool failed"
        return 1
    }

    mkdir build && cd build || {
        record_failure "mkdir/cd build failed"
        return 1
    }

    run_command "Picotool cmake" cmake .. -DPICO_SDK_PATH="$PICO_DIR" || return 1
    run_command "Picotool build" make -j"$(nproc)" || return 1
    run_command "Picotool install" make install || return 1

    cd / || true
    rm -rf "$TMP_PICO"
    log SUCCESS "Picotool installiert."
}

install_pico_sdk() {
    log INFO "Installiere Pico SDK Abhängigkeiten..."
    install_and_check \
        gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
        libusb-1.0-0-dev pkg-config || return 1

    export PICO_SDK_PATH="$PICO_DIR"

    if [ ! -d "$PICO_DIR" ]; then
        log INFO "Klone Pico SDK nach $PICO_DIR..."
        run_command "clone pico-sdk" \
            sudo -u "$REAL_USER" git clone --depth 1 --recursive \
            https://github.com/raspberrypi/pico-sdk.git "$PICO_DIR" || return 1
        chown -R "$REAL_USER:$REAL_USER" "$PICO_DIR"
    else
        log INFO "Pico SDK bereits vorhanden."
    fi
}

install_formatter() {
    if command -v shfmt &>/dev/null; then
        log SUCCESS "shfmt bereits installiert"
        return 0
    fi

    log INFO "Installiere shfmt..."
    if curl -sLo /tmp/shfmt https://github.com/mvdan/sh/releases/download/v3.10.0/shfmt_v3.10.0_linux_amd64; then
        chmod +x /tmp/shfmt
        mv /tmp/shfmt /usr/local/bin/shfmt
        log SUCCESS "shfmt installiert"
    else
        log ERROR "shfmt Download fehlgeschlagen"
        record_failure "shfmt installation"
        return 1
    fi
}

install_just() {
    if command -v just &>/dev/null; then
        log SUCCESS "just bereits installiert"
        return 0
    fi

    log INFO "Installiere just..."
    if snap install just --classic; then
        log SUCCESS "just installiert"
    else
        log WARN "just Installation fehlgeschlagen (optional)"
        return 0
    fi
}

install_gh() {
    if command -v gh &>/dev/null; then
        log SUCCESS "gh bereits installiert"
        return 0
    fi

    log INFO "Installiere GitHub CLI..."
    mkdir -p -m 755 /etc/apt/keyrings
    wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | \
        tee /etc/apt/keyrings/githubcli-archive-keyring.gpg >/dev/null || {
        record_failure "gh keyring download"
        return 1
    }
    chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | \
        tee /etc/apt/sources.list.d/github-cli.list >/dev/null
    wait_for_apt || return 1
    apt update -y || return 1
    install_and_check "gh" || return 1
}

install_docker() {
    # Add GPG Key
    echo -e "${BLUE}[STEP] GPG-Schlüssel hinzufügen${RESET}"
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg |
        gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    # Add Docker Repository for Ubuntu
    echo -e "${BLUE}[STEP] Docker-Repository hinzufügen${RESET}"
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" |
        tee /etc/apt/sources.list.d/docker.list >/dev/null
    # WICHTIG: Nach neuem Repo unbedingt update machen!
    echo -e "${BLUE}[STEP] Paketquellen aktualisieren (Docker)${RESET}"
    wait_for_apt
    run_command "apt update (docker)" apt update -y
    echo -e "${BLUE}[STEP] Docker installieren${RESET}"
    DOCKER_PKGS=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)
    wait_for_apt
    # for pkg in docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; do
    for pkg in "${DOCKER_PKGS[@]}"; do
        install_and_check "$pkg"
    done
    # Docker-Dienst aktivieren
    log INFO "Docker-Dienst aktivieren..."
    systemctl enable --now docker || log WARN "systemctl enable docker fehlgeschlagen"

    # Checking Docker installation
    echo -e "${BLUE}[STEP] Docker-Installation prüfen${RESET}"
    log INFO "Docker-Installation prüfen"
    if docker --version &>/dev/null; then
        log SUCCESS "Docker installiert: $(docker --version)"
        log INFO "Nutzer $REAL_USER zu Gruppen hinzufügen"
        usermod -aG docker,dialout,video,plugdev,gpio,i2c,spi "$REAL_USER"
    else
        log ERROR "Docker installation fehlgeschlagen"
        record_failure "Docker installation"
        return 1
    fi

    # Checking Docker Compose installation
    log INFO "Docker Compose-Installation prüfen"
    if docker compose version &>/dev/null; then
        log SUCCESS "Docker Compose: $(docker compose version)"
    else
        log ERROR "Docker Compose installation fehlgeschlagen"
        record_failure "Docker Compose"
        return 1
    fi
}

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
echo ""
log INFO "Setup gestartet für User: $REAL_USER"
echo ""

echo -e "${BLUE}[STEP]${RESET} System aktualisieren"
wait_for_apt || exit 1
run_command "apt update" apt update -y || exit 1

wait_for_apt || exit 1
run_command "apt upgrade" apt upgrade -y || exit 1

install_and_check "software-properties-common" || exit 1

echo -e "${BLUE}[STEP]${RESET} universe Repository hinzufügen"
wait_for_apt || exit 1
run_command "add-apt-repository universe" add-apt-repository universe -y || exit 1

echo -e "${BLUE}[STEP]${RESET} Basis-Tools installieren"
install_and_check "${TOOLS[@]}" || exit 1

echo ""
echo -e "${BLUE}[STEP]${RESET} Optionale Tools installieren"
install_formatter || log WARN "Formatter Installation optional fehlgeschlagen"
install_just || log WARN "just Installation optional fehlgeschlagen"
install_gh || log WARN "gh Installation optional fehlgeschlagen"

echo ""
echo -e "${BLUE}[STEP]${RESET} Docker installieren"
install_docker || { log ERROR "Docker Installation fehlgeschlagen"; exit 1; }

# ==============================================================================
# INSTALL Pico SDK und Tools
# ==============================================================================
echo ""
echo -e "${BLUE}[STEP]${RESET} Pico SDK und Tools installieren"
install_pico_sdk || { record_failure "Pico SDK"; exit 1; }
install_picotool || { record_failure "Picotool"; exit 1; }

# ==============================================================================
# 4. ROS 2 Humble Installation
# ==============================================================================
echo ""
echo -e "${BLUE}[STEP]${RESET} ROS 2 Humble installieren"
log INFO "Installiere ROS 2 Humble..."

# Locales
run_command "locales installieren" apt install -y locales || exit 1
run_command "locale-gen" locale-gen en_US en_US.UTF-8 || exit 1
run_command "update-locale" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || exit 1
export LANG=en_US.UTF-8

# ROS 2 Repository
log INFO "ROS 2 Repository hinzufügen..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg || {
    record_failure "ROS key download"
    exit 1
}

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" | \
    tee /etc/apt/sources.list.d/ros2.list >/dev/null

wait_for_apt || exit 1
run_command "apt update (ROS)" apt update -y || exit 1
install_and_check "ros-humble-desktop" || exit 1
install_and_check "ros-dev-tools" || exit 1

# Rosdep
log INFO "Rosdep initialisieren..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init || log WARN "rosdep init fehlgeschlagen (möglicherweise bereits initialisiert)"
fi
sudo -u "$REAL_USER" rosdep update || log WARN "rosdep update fehlgeschlagen"

# ==============================================================================
# 5. Shell Konfiguration
# ==============================================================================
echo ""
echo -e "${BLUE}[STEP]${RESET} Shell-Konfiguration (.bashrc & .zshrc)"
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
log SUCCESS "=========================================="
log INFO " ZSH installiert. Wechsel: chsh -s \$(which zsh)"
log INFO " Pico SDK Pfad: $PICO_DIR"
log INFO " Docker Gruppe aktiv nach: sudo su - $REAL_USER"
log INFO " ROS 2 sourcing: source /opt/ros/humble/setup.bash"
log INFO " Logs: $LOG_FILE"
if [ ${#FAILURES[@]} -gt 0 ]; then
    log INFO " Fehler-Log: $ERR_FILE"
fi
log SUCCESS "=========================================="

if [ ${#FAILURES[@]} -gt 0 ]; then
    echo ""
    log ERROR "=== FEHLER-ZUSAMMENFASSUNG (${#FAILURES[@]} Fehler) ==="
    for fail in "${FAILURES[@]}"; do
        log ERROR "  ✗ $fail"
    done
    log ERROR "Details in: $ERR_FILE"
    exit 1
fi

log SUCCESS "Setup erfolgreich abgeschlossen ohne Fehler!"
exit 0