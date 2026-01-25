#!/usr/bin/env bash
# ==============================================================================
# Robust Helper Functions for Installer Modules
# Centralized logic for logging, dry-run safety, and system checks.
# ==============================================================================

# Guard gegen mehrfaches Laden
: ${HELPERS_LOADED:=}
if [ -n "$HELPERS_LOADED" ]; then
    return 0
fi
HELPERS_LOADED=1

# --- BASIS VARIABLEN ---
export REAL_USER=${SUDO_USER:-$USER}
export USER_HOME="/home/$REAL_USER"
[ "$REAL_USER" = "root" ] && export USER_HOME="/root"
: ${BACKUP_DIR:="/tmp/setup_backups/$(date +%Y%m%d_%H%M%S)"}
: ${DRY_RUN:=false}

# --- FARBEN ---
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
BLUE='\033[34m'
RESET='\033[0m'

# ==============================================================================
# LOGGING FUNKTIONEN (Sicher vor Rekursion)
# ==============================================================================

if ! declare -f log >/dev/null 2>&1; then
    log() {
        local level="$1"; shift
        local color="${RESET}"
        case "$level" in
            INFO) color="${BLUE}" ;;
            SUCCESS) color="${GREEN}" ;;
            ERROR) color="${RED}" ;;
            WARN) color="${YELLOW}" ;;
        esac
        echo -e "${color}[$level]${RESET} $*"
    }
fi

if ! declare -f log_step >/dev/null 2>&1; then
    log_step() {
        echo -e "\n${BLUE}╔════════════════════════════════════════╗${RESET}"
        echo -e "${BLUE}║ $*${RESET}"
        echo -e "${BLUE}╚════════════════════════════════════════╝${RESET}"
    }
fi

if ! declare -f log_task >/dev/null 2>&1; then
    log_task() {
        echo -e "${BLUE}→${RESET} $*"
    }
fi

if ! declare -f log_result >/dev/null 2>&1; then
    log_result() {
        local status="$1"; shift
        case "$status" in
            ok)   echo -e "  ${GREEN}✓${RESET} $*" ;;
            fail) echo -e "  ${RED}✗ ERROR:${RESET} $*" >&2 ;;
            skip) echo -e "  ${YELLOW}○ SKIP:${RESET} $*" ;;
            info) echo -e "  ${BLUE}ℹ${RESET} $*" ;;
            *)    echo "  - $*" ;;
        esac
    }
fi

if ! declare -f record_failure >/dev/null 2>&1; then
    record_failure() {
        local msg="$*"
        echo "$(date): FAILURE: $msg" >> "/tmp/setup_errors.log"
        log ERROR "$msg"
    }
fi

# ==============================================================================
# ÄNDERUNGS-TRACKING
# ==============================================================================

if ! declare -f record_change >/dev/null 2>&1; then
    record_change() {
        if [ -z "${APPLIED_CHANGES+x}" ]; then
            APPLIED_CHANGES=()
        fi
        APPLIED_CHANGES+=("$1")
    }
fi

# ==============================================================================
# AKTIONEN & DATEI-OPERATIONEN
# ==============================================================================

run_action() {
    local desc="$1"; shift
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] $desc"
        return 0
    fi
    log_task "$desc"
    "$@"
}

backup_file() {
    local file="$1"
    if [ -e "$file" ]; then
        mkdir -p "$BACKUP_DIR$(dirname "$file")" 2>/dev/null || true
        cp -a "$file" "$BACKUP_DIR$file"
        log_result ok "Backup erstellt: $file"
    fi
}

restore_file() {
    local file="$1"
    local backup="$BACKUP_DIR$file"
    if [ -e "$backup" ]; then
        cp -a "$backup" "$file"
        log_result ok "Wiederhergestellt: $file"
    else
        log WARN "Kein Backup vorhanden für $file"
    fi
}

write_if_changed() {
    local dest="$1"
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] Würde Datei schreiben: $dest"
        cat > /dev/null # Stdin verbrauchen
        return 0
    fi

    local tmp
    tmp=$(mktemp)
    cat >"$tmp"
    if [ -e "$dest" ]; then
        if cmp -s "$tmp" "$dest"; then
            log_result skip "$dest (keine Änderung)"
            rm -f "$tmp"
            return 0
        fi
        backup_file "$dest"
    fi
    mkdir -p "$(dirname "$dest")" 2>/dev/null || true
    mv "$tmp" "$dest"
    chown "$REAL_USER:$REAL_USER" "$dest" 2>/dev/null || true
    log_result ok "Datei geschrieben: $dest"
    record_change "file:$dest"
}

# ==============================================================================
# SYSTEM-HELFER
# ==============================================================================

wait_for_apt() {
    local timeout=300 elapsed=0
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 || fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
        if [ $elapsed -ge $timeout ]; then
            log_result fail "apt lock timeout nach ${timeout}s"
            return 1
        fi
        echo -e "${BLUE}Warte auf apt-Lock...${RESET}"
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 0
}

install_and_check() {
    local pkg="$1"
    if dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
        log_result ok "$pkg (bereits installiert)"
        return 0
    fi

    if [ "$DRY_RUN" = "true" ]; then
        log_result info "[DRY-RUN] Würde installieren: $pkg"
        return 0
    fi

    wait_for_apt || return 1
    if apt-get install -y -qq "$pkg"; then
        log_result ok "$pkg erfolgreich installiert"
        record_change "pkg:$pkg"
        return 0
    else
        log_result fail "Installation von $pkg fehlgeschlagen"
        return 1
    fi
}
