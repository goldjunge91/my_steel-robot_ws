#!/usr/bin/env bash

# ==============================================================================
# Setup Script: Ubuntu + ROS 2 Humble + Pico SDK + ZSH + GH CLI
# ==============================================================================

set -u -o pipefail -E

# set -m || true

# --- KONFIGURATION ---
# Dry-run and verbose flags
DRY_RUN=${DRY_RUN:-false}
VERBOSE=${VERBOSE:-false}
REAL_USER=$SUDO_USER
USER_HOME="/home/$REAL_USER"
PICO_DIR="$USER_HOME/pico-sdk"
FAILURES=()
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Ensure ROS_DISTRO exists to avoid 'set -u' failures when dry-running

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

# Backup directory for changed files
BACKUP_DIR="${SCRIPT_DIR}/backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
# Track applied changes for rollback
APPLIED_CHANGES=()
record_change() {
    APPLIED_CHANGES+=("$1")
}

export DEBIAN_FRONTEND=noninteractive

TOOLS=(
    curl git gnupg2 lsb-release build-essential cmake shfmt
    python3-pip htop net-tools terminator shellcheck nano wget
    zsh fontconfig ca-certificates gnupg joystick jstest-gtk evtest fzf
)

# Farbdefinitionen
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
RESET='\033[0m'  

# 1. Root-Check & User-Variable bestimmen
if [ "$DRY_RUN" != "true" ] && [ "$EUID" -ne 0 ]; then
    echo "Bitte mit sudo ausführen: sudo ./vm_setup_complete.sh"
    exit 1
fi

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

# ---- Idempotence & Safety Helpers ----

run_action() {
    local desc="$1"; shift
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] $desc"
        if [ "$VERBOSE" = "true" ]; then
            # Show what would run but do not execute in dry-run
            log_result info "Would run: $*"
        fi
        return 0
    fi
    log_task "$desc"
    "$@"
}

backup_file() {
    local file="$1"
    if [ -e "$file" ]; then
        mkdir -p "$BACKUP_DIR$(dirname "$file")"
        cp -a "$file" "$BACKUP_DIR$file"
        log_result ok "Backup: $file -> $BACKUP_DIR$file"
    else
        log_result skip "Backup skipped (missing): $file"
    fi
}

restore_file() {
    local file="$1"
    local backup="$BACKUP_DIR$file"
    if [ -e "$backup" ]; then
        cp -a "$backup" "$file"
        log_result ok "Restore: $backup -> $file"
    else
        log WARN "Kein Backup zum Wiederherstellen: $file"
    fi
}

# Write content only if changed; reads content from stdin
write_if_changed() {
    local dest="$1"
    local tmp
    tmp=$(mktemp)
    cat >"$tmp"
    if [ -e "$dest" ]; then
        if cmp -s "$tmp" "$dest"; then
            log_result skip "No change for $dest"
            rm -f "$tmp"
            return 0
        else
            backup_file "$dest"
        fi
    else
        mkdir -p "$(dirname "$dest")"
    fi
    mv "$tmp" "$dest"
    chown "$REAL_USER:$REAL_USER" "$dest" 2>/dev/null || true
    log_result ok "Wrote $dest"
    # record file change for potential rollback
    record_change "file:$dest"
}

# Download, verify (sha256 optional) and run a script
download_and_verify_and_run() {
    local url="$1"; local dest="$2"; local sha256="$3"
    run_action "Download $url to $dest" curl -fsSL "$url" -o "$dest"
    if [ -n "$sha256" ]; then
        echo "$sha256  $dest" | sha256sum -c - || { log ERROR "Checksum failed for $dest"; return 1; }
    fi
    chmod +x "$dest"
    run_action "Execute $dest" bash "$dest"
}

# Druckt eine hervorgehobene Abschnittsüberschrift (mehrzeilige Box)
log_step() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════╗${RESET}"
    echo -e "${BLUE}║ $*${RESET}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${RESET}"
}

rollback_changes() {
    if [ ${#APPLIED_CHANGES[@]} -eq 0 ]; then
        log INFO "No changes recorded to rollback"
        return 0
    fi
    log_step "Rollback: starte Wiederherstellung von ${#APPLIED_CHANGES[@]} Änderungen"
    for ((i=${#APPLIED_CHANGES[@]}-1;i>=0;i--)); do
        local ch="${APPLIED_CHANGES[i]}"
        case "$ch" in
            file:*)
                local f="${ch#file:}"
                restore_file "$f"
                ;;
            pkg:*)
                local p="${ch#pkg:}"
                log_task "Remove package $p"
                apt remove -y "$p" >/dev/null 2>&1 || log WARN "Failed to remove $p"
                ;;
            *)
                log WARN "Unknown change: $ch"
                ;;
        esac
    done
    log SUCCESS "Rollback abgeschlossen"
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

    # If DRY_RUN requested, avoid executing commands; optionally show output in VERBOSE
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] $description"
        if [ "$VERBOSE" = "true" ]; then
            # Try to run in a subshell but don't fail the script
            ("$@") > "$tmplog" 2>&1 || true
            if [ -s "$tmplog" ]; then
                cat "$tmplog"
            fi
        fi
        rm -f "$tmplog"
        log_result ok "$description (dry-run)"
        return 0
    fi

    if "$@" > "$tmplog" 2>&1; then
        if [ -s "$tmplog" ]; then
            cat "$tmplog"
        fi
        rm -f "$tmplog"
        log_result ok "$description"
        return 0
    else
        local ret=$?
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

    # If DRY_RUN, do not perform installs — just show what would be installed
    if [ "$DRY_RUN" = "true" ]; then
        log_result info "DRY-RUN: would install: ${missing[*]}"
        return 0
    fi

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
                record_change "pkg:$pkg"
            elif grep -qE "$pkg.*(is already the newest version|newly installed)" "$install_output"; then
                log_result ok "$pkg (Meta-Paket)"
                INSTALL_CACHE="$INSTALL_CACHE $pkg"
                record_change "pkg:$pkg"
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
                    record_change "pkg:$pkg"
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

# Funktion um Konfigurationen sicher in .bashrc und .zshrc zu schreiben
add_to_shells() {
    local content="$1"
    local shell_type="${2:-both}" # both, bash, or zsh
    local targets=()

    # Ziel-Dateien bestimmen
    [[ "$shell_type" == "both" || "$shell_type" == "bash" ]] && targets+=("$USER_HOME/.bashrc")
    [[ "$shell_type" == "both" || "$shell_type" == "zsh" ]] && targets+=("$USER_HOME/.zshrc")

    for target in "${targets[@]}"; do
        # Sicherstellen, dass die Datei existiert und dem User gehört
        if [ ! -f "$target" ]; then
            touch "$target"
            chown "$REAL_USER:$REAL_USER" "$target" 2>/dev/null || true
        fi

        # -F (Fixed strings) interpretiert den Inhalt nicht als Regex (wichtig für Pfade)
        # -q (quiet) unterdrückt die Ausgabe
        # -z (line-regexp) erzwingt die Übereinstimmung der ganzen Zeile
        if grep -Fq "$content" "$target"; then
            log_result skip "$(basename "$target"): bereits vorhanden"
        else
            # Mit einer Leerzeile davor anhängen, um Dateien nicht zu "verkleben"
            echo -e "\n$content" >> "$target"
            chown "$REAL_USER:$REAL_USER" "$target" 2>/dev/null || true
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

# ==============================================================================
# --- INSTALLATIONS FUNKTIONEN ---
# ==============================================================================

install_picotool() {
    run_action "install_picotool" bash "$SCRIPT_DIR/installers/install_picotool.sh"
}

install_pico_sdk() {
    run_action "install_pico_sdk" bash "$SCRIPT_DIR/installers/install_pico_sdk.sh"
}

install_formatter() {
    run_action "install_formatter" bash "$SCRIPT_DIR/installers/install_formatter.sh"
}

install_just() {
    run_action "install_just" bash "$SCRIPT_DIR/installers/install_just.sh"
}

install_gh() {
    run_action "install_gh" bash "$SCRIPT_DIR/installers/install_gh.sh"
}

install_nvm() {
    run_action "install_nvm" bash "$SCRIPT_DIR/installers/install_nvm.sh"
}

install_docker() {
    run_action "install_docker" bash "$SCRIPT_DIR/installers/install_docker.sh"
}

install_ros() {
    run_action "install_ros" bash "$SCRIPT_DIR/installers/install_ros_via_script.sh"
}
install_ros_via_script() {
    run_action "install_ros_via_script" bash "$SCRIPT_DIR/installers/install_ros_via_script.sh"
}

install_oh_my_bash() {
    run_action "install_oh_my_bash" bash "$SCRIPT_DIR/installers/install_oh_my_bash.sh"
}
# ==============================================================================
# MAIN EXECUTION
# ==============================================================================
log_step "INITIALISIERUNG"
log_result info "User: $REAL_USER"
log_result info "Ubuntu: ${VERSION_ID:-unbekannt} ($VERSION_CODENAME)"
log_result info "Nutze ROS 2 Distro: $ROS_DISTRO" # Hier wird es jetzt korrekt angezeigt!
log_result info "Logs: $LOG_FILE"
log_result info "Fehler: $ERR_FILE"

log_step "SYSTEM AKTUALISIEREN"

# Force Fix für defekte Quellen
fix_broken_sources

wait_for_apt || exit 1
    log_task "apt update"
    if [ "$DRY_RUN" = "true" ]; then
        log_result info "DRY-RUN: apt update skipped"
    else
        if apt update -qq >/dev/null 2>&1; then
            log_result ok "apt update"
        else
            log WARN "apt update mit Warnungen"
        fi
    fi

wait_for_apt || exit 1
    log_task "apt upgrade"
    if [ "$DRY_RUN" = "true" ]; then
        log_result info "DRY-RUN: apt upgrade skipped"
    else
        if apt upgrade -y -qq >/dev/null 2>&1; then
            log_result ok "apt upgrade"
        else
            log WARN "apt upgrade übersprungen"
        fi
    fi

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
