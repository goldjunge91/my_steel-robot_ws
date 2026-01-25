#!/usr/bin/env bash

# ==============================================================================
# Setup Script: Ubuntu + ROS 2 Humble + Pico SDK + ZSH + GH CLI
# ==============================================================================

set -u -o pipefail -E

# set -m || true

# --- KONFIGURATION ---
# Dry-run and verbose flags
export DRY_RUN=${DRY_RUN:-false}
export VERBOSE=${VERBOSE:-false}
export REAL_USER=${SUDO_USER:-$USER}
export USER_HOME="/home/$REAL_USER"
export DEBIAN_FRONTEND=noninteractive
# Falls root der REAL_USER ist (direkter Login), ist das Home /root
[ "$REAL_USER" = "root" ] && export USER_HOME="/root"

PICO_DIR="$USER_HOME/pico-sdk"
FAILURES=()
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Automatische ROS 2 Distro Auswahl ---
if [ -f /etc/os-release ]; then
    # Lädt System-Variablen wie VERSION_CODENAME
    . /etc/os-release
    case "$VERSION_CODENAME" in
        "focal") ROS_DISTRO="foxy" ;;   # Ubuntu 20.04
        "jammy") ROS_DISTRO="humble" ;; # Ubuntu 22.04
        "noble") ROS_DISTRO="jazzy" ;;  # Ubuntu 24.04
        *)
            # Fallback auf Humble, falls Version unbekannt
            ROS_DISTRO="humble" 
            ;;
    esac
else
    ROS_DISTRO="humble"
fi

# Track applied changes for rollback
APPLIED_CHANGES=()
record_change() {
    APPLIED_CHANGES+=("$1")
}
# Backup directory for changed files
BACKUP_DIR="${SCRIPT_DIR}/backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
# 1. Root-Check & User-Variable bestimmen
if [ "$DRY_RUN" != "true" ] && [ "$EUID" -ne 0 ]; then
    echo "Bitte mit sudo ausführen: sudo ./vm_setup_complete.sh"
    exit 1
fi

TOOLS=(
    curl
    git
    gnupg2
    lsb-release
    build-essential
    cmake
    shfmt
    python3-pip
    htop
    net-tools
    terminator
    shellcheck
    nano
    wget
    zsh
    fontconfig
    ca-certificates
    gnupg
    joystick
    jstest-gtk
    evtest
    fzf
    rpi-imager
)

# Der User, der sudo ausgeführt hat (nicht root)
if [ -z "$REAL_USER" ] || [ "$REAL_USER" = "root" ]; then
    if [ "$DRY_RUN" = "true" ]; then
        # Allow dry-run without sudo: fall back to invoking user
        REAL_USER=${SUDO_USER:-$USER}
        USER_HOME="/home/$REAL_USER"
        log INFO "DRY_RUN: using REAL_USER=$REAL_USER"
    else
        echo "Konnte den normalen Benutzer nicht ermitteln. Bitte via sudo ausführen."
        exit 1
    fi
fi


# Helpers laden & Funktionen für Sub-Shells bereitstellen
source "$SCRIPT_DIR/installers/helpers.sh"
export -f log log_step log_task log_result record_failure run_action install_and_check run_action

echo "=== Setup gestartet am $(date) ==="
echo "--- OS Release Info ---"
cat /etc/os-release
echo "-----------------------"

# ==============================================================================
# --- Hilfsfunktionen ---
# ==============================================================================



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


# Funktion um Konfigurationen sicher in .bashrc und .zshrc zu schreiben
add_to_shells() {
    local content="$1"
    local shell_type="${2:-both}"
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] Würde in Shell-Configs schreiben: $content"
        return 0
    fi
    local targets=()
    
    [[ "$shell_type" == "both" || "$shell_type" == "bash" ]] && targets+=("$USER_HOME/.bashrc")
    [[ "$shell_type" == "both" || "$shell_type" == "zsh" ]] && targets+=("$USER_HOME/.zshrc")

    for target in "${targets[@]}"; do
        # Sicherstellen, dass die Datei existiert
        if [ ! -f "$target" ]; then
            sudo -u "$REAL_USER" touch "$target"
        fi

        # -F (Fixed strings) verhindert Fehler bei Pfaden/Sonderzeichen
        if grep -Fq "$content" "$target"; then
            log_result skip "$(basename "$target"): bereits vorhanden"
        else
            # Schreibt die Zeile sauber mit neuem Zeilenumbruch als REAL_USER
            echo -e "\n$content" | sudo -u "$REAL_USER" tee -a "$target" > /dev/null
            log_result ok "$(basename "$target"): Eintrag hinzugefügt"
            record_change "file:$target"
        fi
    done
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
# --- INITIALISIERUNG LOGS ---
LOG_FILE="$SCRIPT_DIR/setup_$(date +%Y%m%d_%H%M%S).log"
export ERR_FILE="$SCRIPT_DIR/errors_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$ERR_FILE" >&2)

# --- AUTOMATISCHE ROS DISTRO ---
source /etc/os-release
case "$VERSION_CODENAME" in
    focal) ROS_DISTRO="foxy" ;; jammy) ROS_DISTRO="humble" ;; noble) ROS_DISTRO="jazzy" ;;
    *) ROS_DISTRO="humble" ;;
esac
# ==============================================================================
# --- INSTALLATIONS WRAPPER ---
# ==============================================================================

install_picotool()   { run_action "install_picotool" bash "$SCRIPT_DIR/installers/install_picotool.sh"; }
install_pico_sdk()   { run_action "install_pico_sdk" bash "$SCRIPT_DIR/installers/install_pico_sdk.sh"; }
install_formatter()  { run_action "install_formatter" bash "$SCRIPT_DIR/installers/install_formatter.sh"; }
install_just()       { run_action "install_just" bash "$SCRIPT_DIR/installers/install_just.sh"; }
install_gh()         { run_action "install_gh" bash "$SCRIPT_DIR/installers/install_gh.sh"; }
install_nvm()        { run_action "install_nvm" bash "$SCRIPT_DIR/installers/install_nvm.sh"; }
install_docker()     { run_action "install_docker" bash "$SCRIPT_DIR/installers/install_docker.sh"; }
install_ros()        { run_action "install_ros" bash "$SCRIPT_DIR/installers/install_ros_via_script.sh"; }
install_oh_my_bash() { run_action "install_oh_my_bash" bash "$SCRIPT_DIR/installers/install_oh_my_bash.sh"; }

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
log_step "INITIALISIERUNG"
log_result info "User: $REAL_USER"
log_result info "Ubuntu: ${VERSION_ID:-unbekannt} ($VERSION_CODENAME)"
log_result info "Nutze ROS 2 Distro: $ROS_DISTRO"
log_result info "Logs: $LOG_FILE"
log_result info "Fehler: $ERR_FILE"

log_step "SYSTEM AKTUALISIEREN"
# Force Fix für defekte Quellen
fix_broken_sources

wait_for_apt || exit 1
log_task "apt update && upgrade"
run_action "apt update" apt update -qq
run_action "apt upgrade" apt upgrade -y -qq

install_and_check "software-properties-common" || exit 1

log_step "REPOSITORIES KONFIGURIEREN"
wait_for_apt || exit 1
log_task "add-apt-repository universe"
if add-apt-repository universe -y >/dev/null 2>&1; then
    log_result ok "universe repository"
else
    exit 1
fi

log_step "BASIS-TOOLS"
install_and_check "${TOOLS[@]}" || exit 1

log_step "ERWEITERTE TOOLS (erforderlich)"
# These tools are required. Fail fast if any installer returns non-zero.
if ! install_formatter; then
    log ERROR "shfmt installation failed"
    exit 1
fi
if ! install_just; then
    log ERROR "just installation failed"
    exit 1
fi
if ! install_gh; then
    log ERROR "gh installation failed"
    exit 1
fi
if ! install_nvm; then
    log ERROR "nvm installation failed"
    exit 1
fi
if ! install_oh_my_bash; then
    log ERROR "Oh My Bash installation failed"
    exit 1
fi

log_step "DOCKER"
if ! install_docker; then
    exit 1
fi

log_step "PICO SDK & TOOLS"
if ! install_pico_sdk; then
    exit 1
fi
if ! install_picotool; then
    exit 1
fi

if ! install_ros; then
    exit 1
fi

log_step "SHELL-KONFIGURATION"
log_task "PATH für lokale Tools (.local/bin)"
add_to_shells 'export PATH="$HOME/.local/bin:$PATH"' "both"

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

log_task "NVM Initialisierung"
NVM_INIT='export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"'
add_to_shells "$NVM_INIT" "both"
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
if [ "$DRY_RUN" != "true" ]; then
    apt-get autoremove -y >/dev/null 2>&1
    apt-get clean >/dev/null 2>&1
else
    log_result info "DRY-RUN: apt-get autoremove/clean skipped"
fi
# Logs dem User übergeben
chown "$REAL_USER:$REAL_USER" "$LOG_FILE" "$ERR_FILE" 2>/dev/null || true

log SUCCESS "╔══════════════════════════════════════════╗"
log SUCCESS "║ Setup erfolgreich ohne Fehler!           ║"
log SUCCESS "╚══════════════════════════════════════════╝"
exit 0
