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
export DEBIAN_FRONTEND=noninteractive

TOOLS=(
    curl git gnupg2 lsb-release build-essential cmake
    python3-pip htop net-tools terminator shellcheck nano wget
    zsh fontconfig ca-certificates gnupg joystick jstest-gtk evtest fzf
)
DOCKER_PKGS=(docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)
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
echo "--- OS Release Info ---"
cat /etc/os-release
echo "-----------------------"

# ==============================================================================
# --- Hilfsfunktionen ---
# ==============================================================================

log() {
    local level="$1"
    shift
    local color="${RESET}"
    case "$level" in
        INFO) color="${BLUE}" ;;
        SUCCESS) color="${GREEN}" ;;
        ERROR) color="${RED}" ;;
        WARN) color="${YELLOW}" ;;
    esac
    echo -e "${color}[$level]${RESET} $*"

}

# Druckt eine hervorgehobene Abschnittsüberschrift (mehrzeilige Box)
log_step() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════╗${RESET}"
    echo -e "${BLUE}║ $*${RESET}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${RESET}"
}

# Druckt eine einfache Ein-Zeilen-Aufgabe mit Pfeil-Präfix
log_task() {
    echo -e "${BLUE}→${RESET} $*"
}
# Formatiert Ergebnis-Status für eine Aufgabe.
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
# Protokolliert einen Fehler: hängt Nachricht an das Array `FAILURES`
record_failure() {
    FAILURES+=("$*")
    echo "$(date): FAILURE: $*" >>"$ERR_FILE" 2>/dev/null || true
    log ERROR "$*"

}
# Wartet, bis apt/dpkg-Lockfiles freigegeben sind
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
# Führt ein externes Kommando aus mit Beschreibung.
run_command() {
        #    Verhalten:
        #   - Loggt die Aufgabe (`log_task`).
        #   - Leitet stdout/stderr in eine temporäre Datei.
        #   - Bei Erfolg: zeigt Output (wenn vorhanden) und meldet `ok`.
        #   - Bei Fehler: zeigt Output, meldet `fail` mit Exit-Code.
        #   - Rückgabewert: Exit-Status des Kommandos (0/1)."
    local description="$1"
    shift
    log_task "$description"
    
    # Temporäres Log für besseres Error-Handling
    local tmplog
    tmplog=$(mktemp)
    
    if "$@" > "$tmplog" 2>&1; then
        # Zeige Output nur wenn nicht leer
        if [ -s "$tmplog" ]; then
            cat "$tmplog"
        fi
        
        if [ -s "$tmplog" ]; then
            # Output is already captured by global tee via stdout
             :
        fi
        
        rm -f "$tmplog"
        log_result ok "$description"
        return 0
    else
        local ret=$?
        # Zeige Error-Output
        cat "$tmplog"
        rm -f "$tmplog"
        log_result fail "$description (exit $ret)"
        return 1
    fi
}

fix_broken_sources() {
    log_task "Prüfe und bereinige defekte Paketquellen..."
    local yarn_list="/etc/apt/sources.list.d/yarn.list"
    
    # Bekanntes Problem: Yarn Key Expiration (Error 100)
    if [ -f "$yarn_list" ]; then
        log_task "Entferne bekannten Problem-Kandidaten: $yarn_list"
        rm -f "$yarn_list"
        log_result ok "yarn.list gelöscht"
    fi
    
    # Optional: Allgemeiner Check auf "EXPKEYSIG" in apt update output wäre hier möglich,
    # aber "Force Fix" impliziert oft aggressives Vorgehen gegen bekannte Übeltäter.
    # Repariere dpkg falls vom letzten Lauf unterbrochen
    if ! dpkg --audit &>/dev/null; then
        log_task "Repariere unterbrochene dpkg-Installation"
        if dpkg --configure -a >/dev/null; then
            log_result ok "dpkg --configure -a erfolgreich"
        else
            log_result fail "dpkg --configure -a fehlgeschlagen"
        fi
        
        if apt-get --fix-broken install -y >/dev/null; then
            log_result ok "apt --fix-broken erfolgreich"
        else
            log WARN "apt --fix-broken hatte Probleme"
        fi
        
        # Erneuter Audit-Check
        if dpkg --audit &>/dev/null; then
            log_result ok "dpkg ist jetzt konsistent"
        else
            log ERROR "dpkg ist IMMER NOCH inkonsistent - manuelle Reparatur erforderlich"
            log ERROR "Bitte ausführen: sudo dpkg --configure -a"
            exit 1
        fi
    fi

}

# Install funktion benötigt paketname als Argument
install_and_check() {
    local packages=("$@")
    local missing=()
    
    log_task "Prüfe ${#packages[@]} Paket(e)"
    
    # Cache initialisieren
    if [ -z "${INSTALL_CACHE+x}" ]; then
        INSTALL_CACHE=""
    fi

    local pkg
    for pkg in "${packages[@]}"; do
        # Cache-Check
        if [[ " $INSTALL_CACHE " == *" $pkg "* ]]; then
            log_result skip "$pkg (Cache)"
            continue
        fi

        # Installation-Check
        if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            log_result ok "$pkg (bereits installiert)"
            INSTALL_CACHE="$INSTALL_CACHE $pkg"
        else
            missing+=("$pkg")
        fi
    done

    # Installation
    if [ ${#missing[@]} -eq 0 ]; then
        return 0
    fi

    log_task "Installiere ${#missing[@]} fehlende Paket(e)"
    
    wait_for_apt || return 1
    
    # Batch-Installation
    local install_output
    install_output=$(mktemp)
    if DEBIAN_FRONTEND=noninteractive apt install -y -qq "${missing[@]}" 2>&1 | tee "$install_output" >/dev/null; then
        # Verification
        local failed=()
        for pkg in "${missing[@]}"; do
            if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
                log_result ok "$pkg"
                INSTALL_CACHE="$INSTALL_CACHE $pkg"
            elif grep -qE "$pkg.*(is already the newest version|newly installed)" "$install_output"; then
                log_result ok "$pkg (Meta-Paket)"
                INSTALL_CACHE="$INSTALL_CACHE $pkg"
            else
                log_result fail "$pkg"
                failed+=("$pkg")
            fi
        done
        rm -f "$install_output"

        [ ${#failed[@]} -eq 0 ]
        return $?
    else
        rm -f "$install_output"
        # Fallback: Einzelinstallation
        log_task "Batch fehlgeschlagen, versuche einzeln"
        for pkg in "${missing[@]}"; do
            wait_for_apt || return 1
            if DEBIAN_FRONTEND=noninteractive apt install -y -qq "$pkg" >/dev/null 2>&1; then
                if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
                    log_result ok "$pkg"
                    INSTALL_CACHE="$INSTALL_CACHE $pkg"
                else
                    log_result fail "$pkg"
                    return 1
                fi
            else
                log_result fail "$pkg"
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

# shellcheck disable=SC2329
cleanup() {
    # Exit Code speichern
    local exit_code=$?
    if [ "$exit_code" -ne 0 ]; then
        echo -e "\n${RED}Skript wurde unterbrochen oder fehlerhaft beendet.${RESET}"
        
        # Repariere dpkg falls unterbrochen
        if ! dpkg --audit &>/dev/null 2>&1; then
            log WARN "dpkg scheint unterbrochen - versuche Reparatur"
            dpkg --configure -a &>/dev/null || true
            apt-get --fix-broken install -y &>/dev/null || true
        fi
    fi
    # Prozesse aufräumen
    pkill -P $$ 2>/dev/null || true
}

trap cleanup EXIT SIGINT SIGTERM

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

    if ! mkdir build; then
        record_failure "mkdir build failed"
        return 1
    fi
    if ! cd build; then
        record_failure "cd build failed"
        return 1
    fi

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
    if curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/local/bin; then
        log SUCCESS "just installiert"
    else
        log WARN "just Installation fehlgeschlagen (optional)"
        return 1
    fi
}

install_gh() {
    if command -v gh &>/dev/null; then
        log SUCCESS "gh bereits installiert"
        return 0
    fi

    log INFO "Installiere GitHub CLI..."
    mkdir -p /etc/apt/keyrings
    chmod 755 /etc/apt/keyrings
    run_command "gh keyring download" \
        wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | \
        tee /etc/apt/keyrings/githubcli-archive-keyring.gpg >/dev/null || return 1
    
    chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | \
        tee /etc/apt/sources.list.d/github-cli.list >/dev/null
    
    wait_for_apt || return 1
    run_command "apt update (gh)" apt update -y -qq || log WARN "apt update (gh) had errors, proceeding..."
    install_and_check "gh" || return 1
}

install_nvm() {
    # Check if nvm directory exists
    if [ -d "$USER_HOME/.nvm" ]; then
        log SUCCESS "nvm bereits installiert"
        return 0
    fi

    log INFO "Installiere nvm..."
    # Installation als REAL_USER durchführen, damit es im Home-Dir des Users landet
    # und die korrekten Shell-Configs (.bashrc/.zshrc des Users) bearbeitet werden.
    if sudo -u "$REAL_USER" bash -c "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.3/install.sh | bash"; then
        log SUCCESS "nvm installiert"
        
        # NVM environment vars für das laufende Skript verfügbar machen (optional, falls wir nvm noch nutzen wollen)
        export NVM_DIR="$USER_HOME/.nvm"
        # shellcheck disable=SC1091
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    else
        log_result fail "nvm Installation"
        return 1
    fi
}

install_docker() {
    # Check if Docker is already installed (e.g. in Codespaces)
    if command -v docker &>/dev/null; then
        log SUCCESS "Docker bereits installiert: $(docker --version)"
        return 0
    fi

    # Entferne moby-tini falls vorhanden (verhindert Konflikt mit docker-ce)
    if dpkg -l | grep -q moby-tini; then
        log_task "Entferne moby-tini (Konflikt)"
        apt-get remove -y -qq moby-tini >/dev/null 2>&1 || true
        log_result ok "moby-tini entfernt"
    fi
    
    log_task "GPG-Schlüssel hinzufügen"
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
        gpg --dearmor -o /etc/apt/keyrings/docker.gpg || {
        log_result fail "GPG-Key Download"
        return 1
    }
    log_result ok "GPG-Schlüssel"
    
    log_task "Docker-Repository konfigurieren"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
        tee /etc/apt/sources.list.d/docker.list >/dev/null
    log_result ok "Repository"
    
    log_task "Paketquellen aktualisieren"
    wait_for_apt || return 1
    run_command "apt update (docker)" apt update -y -qq || log WARN "apt update (docker) had errors, proceeding..."
    
    log_task "Docker-Pakete installieren"
    # local DOCKER_PKGS=(docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)
    wait_for_apt || return 1
    for pkg in "${DOCKER_PKGS[@]}"; do
        install_and_check "$pkg" || return 1
    done
    
    log_task "Docker-Dienst aktivieren"
    systemctl enable --now docker || log WARN "systemctl enable fehlgeschlagen"
    log_result ok "Docker-Dienst"
    
    log_task "Docker-Installation verifizieren"
    if docker --version &>/dev/null; then
        log_result ok "Docker: $(docker --version)"
    else
        log_result fail "Docker nicht verfügbar"
        return 1
    fi
    
    if docker compose version &>/dev/null; then
        log_result ok "Docker Compose: $(docker compose version)"
    else
        log_result fail "Docker Compose nicht verfügbar"
        return 1
    fi
    
    log_task "Benutzer zu Gruppen hinzufügen"
    usermod -aG docker,dialout,video,plugdev,gpio,i2c,spi "$REAL_USER"
    log_result ok "User $REAL_USER → docker,dialout,video,plugdev,gpio,i2c,spi"
}

install_ros() {
    log_step "ROS 2 SETUP"
    # OS Check & Distro Selection
    # Note: ROS_DISTRO is intentionally global for later use in shell config
    local ubuntu_codename
    ubuntu_codename=$(source /etc/os-release && echo "$UBUNTU_CODENAME")
    ROS_DISTRO=""

    case "$ubuntu_codename" in
        jammy) ROS_DISTRO="humble" ;;
        noble) ROS_DISTRO="jazzy" ;;
        *) 
            log ERROR "Nicht unterstützte Ubuntu-Version: $ubuntu_codename (erwarte jammy oder noble)"
            return 1
            ;;
    esac

    log_result info "Ubuntu $ubuntu_codename -> ROS 2 $ROS_DISTRO"

    # Locales
    install_and_check "locales" || return 1
    run_command "locale-gen" locale-gen en_US en_US.UTF-8 || return 1
    run_command "update-locale" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || return 1
    export LANG=en_US.UTF-8

    # Repository
    if ! curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg; then
        log_result fail "ROS key download"
        return 1
    fi
    log_result ok "ROS key download"
    
    # shellcheck disable=SC1091
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $ubuntu_codename main" | \
        tee /etc/apt/sources.list.d/ros2.list >/dev/null
    log_result ok "Repository konfiguriert"

    # Installation
    wait_for_apt || return 1
    run_command "apt update (ROS)" apt update -y || log WARN "apt update (ROS) had errors, proceeding..."
    install_and_check "ros-$ROS_DISTRO-desktop" || return 1
    install_and_check "ros-dev-tools" || return 1

    # Rosdep
    log_task "Rosdep initialisieren"
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        rosdep init || log WARN "rosdep init fehlgeschlagen"
    fi
    sudo -u "$REAL_USER" rosdep update || log WARN "rosdep update fehlgeschlagen"
    log_result ok "Rosdep initialisiert"
}
install_ros_via_script() {
    log INFO "Installiere ROS2..."
    # Execute the installer as the real user to ensure files land in $USER_HOME
    # and permissions are correct.
    if sudo -u "$REAL_USER" bash "$SCRIPT_DIR/bash/install_ros2.sh"; then
        log SUCCESS "ROS2 installiert (via script)"
        return 0
    fi

    # Script failed — versuche internen Installer als Fallback
    log INFO "Installationsskript fehlgeschlagen, versuche internen Installer..."
    if install_ros; then
        log SUCCESS "ROS2 installiert (via internal installer)"
        return 0
    else
        log WARN "ROS2 Installation fehlgeschlagen"
        record_failure "ROS2 Installation"
        return 1
    fi
}

install_oh_my_bash() {
    log INFO "Installiere Oh My Bash..."
    # Execute the installer as the real user to ensure files land in $USER_HOME
    # and permissions are correct.
    if sudo -u "$REAL_USER" bash "$SCRIPT_DIR/bash/install_omb.sh"; then
        log SUCCESS "Oh My Bash installiert"
    else
        log WARN "Oh My Bash Installation fehlgeschlagen"
        record_failure "Oh My Bash Installation"
        return 1
    fi
}
# ==============================================================================
# MAIN EXECUTION
# ==============================================================================

log_step "INITIALISIERUNG"
log_result info "User: $REAL_USER"
log_result info "Logs: $LOG_FILE"
log_result info "Fehler: $ERR_FILE"

log_step "SYSTEM AKTUALISIEREN"

# Force Fix für defekte Quellen
fix_broken_sources

wait_for_apt || exit 1
log_task "apt update"
apt update -qq >/dev/null 2>&1 && log_result ok "apt update" || log WARN "apt update mit Warnungen"
wait_for_apt || exit 1
log_task "apt upgrade"
apt upgrade -y -qq >/dev/null 2>&1 && log_result ok "apt upgrade" || log WARN "apt upgrade übersprungen"
install_and_check "software-properties-common" || exit 1

log_step "REPOSITORIES KONFIGURIEREN"
wait_for_apt || exit 1
log_task "add-apt-repository universe"
add-apt-repository universe -y >/dev/null 2>&1 && log_result ok "universe repository" || exit 1

log_step "BASIS-TOOLS"
install_and_check "${TOOLS[@]}" || exit 1

log_step "OPTIONALE TOOLS"
install_formatter || log WARN "shfmt optional übersprungen"
install_just || log WARN "just optional übersprungen"
install_gh || log WARN "gh optional übersprungen"
install_nvm || log WARN "nvm optional übersprungen"
install_oh_my_bash || log WARN "Oh My Bash optional übersprungen"

log_step "DOCKER"
install_docker || exit 1

log_step "PICO SDK & TOOLS"
install_pico_sdk || exit 1
install_picotool || exit 1

install_ros_via_script || exit 1

log_step "SHELL-KONFIGURATION"
log_task "Pico SDK Pfad setzen"
add_to_shells "export PICO_SDK_PATH=\"$PICO_DIR\"" "both"
log_result ok "PICO_SDK_PATH"

log_task "ROS 2 Setup"
add_to_shells "# ROS 2 Setup" "both"
add_to_shells "source /opt/ros/$ROS_DISTRO/setup.bash" "bash"
add_to_shells "source /opt/ros/$ROS_DISTRO/setup.zsh" "zsh"
log_result ok "ROS 2 Sourcing"

log_task "Colcon Autocomplete"
add_to_shells "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" "bash"
add_to_shells "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" "zsh"
log_result ok "Colcon Autocomplete"

# ==============================================================================
# FINAL SUMMARY
# ==============================================================================
echo ""
log_step "SETUP ABGESCHLOSSEN"
log_result ok "ZSH installiert → chsh -s \$(which zsh)"
log_result ok "Pico SDK → $PICO_DIR"
log_result ok "Docker Gruppe → sudo su - $REAL_USER"
log_result ok "ROS 2 → source /opt/ros/$ROS_DISTRO/setup.bash"
log_result info "Logs → $LOG_FILE"

if [ ${#FAILURES[@]} -gt 0 ]; then
    echo ""
    log ERROR "╔══════════════════════════════════════════╗"
    log ERROR "║ FEHLER-ZUSAMMENFASSUNG: ${#FAILURES[@]} Fehler"
    log ERROR "╚══════════════════════════════════════════╝"
    for fail in "${FAILURES[@]}"; do
        log ERROR "  ✗ $fail"
    done
    log ERROR "Details in: $ERR_FILE"
    exit 1
fi

# Aufräumen & Rechte setzen
log_task "Aufräumen"
apt-get autoremove -y >/dev/null 2>&1
apt-get clean >/dev/null 2>&1
# Logs dem User übergeben
chown "$REAL_USER:$REAL_USER" "$LOG_FILE" "$ERR_FILE" 2>/dev/null || true

log SUCCESS "╔══════════════════════════════════════════╗"
log SUCCESS "║ Setup erfolgreich ohne Fehler!           ║"
log SUCCESS "╚══════════════════════════════════════════╝"
exit 0