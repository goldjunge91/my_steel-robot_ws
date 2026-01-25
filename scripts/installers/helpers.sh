#!/usr/bin/env bash
# ==============================================================================
# Vollständige Helper Functions (Single Source of Truth)
# ==============================================================================

# Guard gegen mehrfaches Laden
: ${HELPERS_LOADED:=}
if [ -n "$HELPERS_LOADED" ]; then return 0; fi
HELPERS_LOADED=1

# --- BASIS VARIABLEN ---
export REAL_USER=${SUDO_USER:-$USER}
export USER_HOME="/home/$REAL_USER"
[ "$REAL_USER" = "root" ] && export USER_HOME="/root"
: ${BACKUP_DIR:="/tmp/setup_backups/$(date +%Y%m%d_%H%M%S)"}
export DRY_RUN=${DRY_RUN:-false}

# Farben
RED='\033[31m'; GREEN='\033[32m'; YELLOW='\033[33m'; BLUE='\033[34m'; RESET='\033[0m'

# --- LOGGING ---
log() {
    local level="$1"; shift
    local color="${RESET}"
    case "$level" in
        INFO) color="${BLUE}" ;; SUCCESS) color="${GREEN}" ;;
        ERROR) color="${RED}" ;; WARN) color="${YELLOW}" ;;
    esac
    echo -e "${color}[$level]${RESET} $*"
}

log_step() {
    echo -e "\n${BLUE}╔════════════════════════════════════════╗${RESET}"
    echo -e "${BLUE}║ $*${RESET}"
    echo -e "${BLUE}╚════════════════════════════════════════╝${RESET}"
}

log_task() { echo -e "${BLUE}→${RESET} $*"; }

log_result() {
    local status="$1"; shift
    case "$status" in
        ok)   echo -e "  ${GREEN}✓${RESET} $*" ;;
        fail) echo -e "  ${RED}✗${RESET} $*"; record_failure "$*" ;;
        skip) echo -e "  ${YELLOW}○${RESET} $*" ;;
        info) echo -e "  ${BLUE}ℹ${RESET} $*" ;;
    esac
}

record_failure() {
    local msg="$*"
    # Schreibt in die globale FAILURES-Variable des Hauptskripts (wenn vorhanden)
    if declare -p FAILURES >/dev/null 2>&1; then FAILURES+=("$msg"); fi
    # Schreibt in das Error-Log
    local log_target="${ERR_FILE:-/tmp/setup_errors.log}"
    echo "$(date): FAILURE: $msg" >> "$log_target" 2>/dev/null || true
    log ERROR "$msg"
}

# --- AKTIONEN ---
run_action() {
    local desc="$1"; shift
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] $desc"; return 0
    fi
    log_task "$desc"
    "$@"
}

run_command() {
    local description="$1"; shift
    log_task "$description"
    local tmplog=$(mktemp)
    if [ "$DRY_RUN" = "true" ]; then
        log_result ok "$description (dry-run)"; rm -f "$tmplog"; return 0
    fi
    if "$@" > "$tmplog" 2>&1; then
        [ -s "$tmplog" ] && cat "$tmplog"
        log_result ok "$description"
        rm -f "$tmplog"
    else
        cat "$tmplog"; rm -f "$tmplog"
        log_result fail "$description"
        return 1
    fi
}

# --- DATEI-OPERATIONEN ---
backup_file() {
    local file="$1"
    if [ -e "$file" ]; then
        mkdir -p "$BACKUP_DIR$(dirname "$file")" 2>/dev/null || true
        cp -a "$file" "$BACKUP_DIR$file"
        log_result ok "Backup: $file"
    fi
}

write_if_changed() {
    local dest="$1"
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] Würde Datei schreiben: $dest"
        cat > /dev/null; return 0
    fi
    local tmp=$(mktemp)
    cat >"$tmp"
    if [ -e "$dest" ] && cmp -s "$tmp" "$dest"; then
        log_result skip "$dest (unverändert)"; rm -f "$tmp"; return 0
    fi
    [ -e "$dest" ] && backup_file "$dest"
    mkdir -p "$(dirname "$dest")" 2>/dev/null || true
    mv "$tmp" "$dest"
    chown "$REAL_USER:$REAL_USER" "$dest" 2>/dev/null || true
    log_result ok "Datei geschrieben: $dest"
}

download_and_verify_and_run() {
    local url="$1"; local dest="$2"; local sha256="${3:-}"
    run_action "Download $url" curl -fsSL "$url" -o "$dest"
    if [ "$DRY_RUN" = "true" ]; then return 0; fi
    if [ -n "$sha256" ]; then
        echo "$sha256  $dest" | sha256sum -c - || { log ERROR "Checksum failed"; return 1; }
    fi
    chmod +x "$dest"
    run_action "Ausführen: $(basename "$dest")" bash "$dest"
}

# --- SYSTEM-CHECKS ---
wait_for_apt() {
    local timeout=300 elapsed=0
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 || fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
        if [ $elapsed -ge $timeout ]; then return 1; fi
        echo -e "${BLUE}Warte auf apt-Lock...${RESET}"; sleep 2; elapsed=$((elapsed + 2))
    done
}

install_and_check() {
    local pkgs=("$@"); local missing=()
    for pkg in "${pkgs[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            missing+=("$pkg")
        else log_result ok "$pkg (bereits installiert)"; fi
    done
    if [ ${#missing[@]} -eq 0 ]; then return 0; fi
    if [ "$DRY_RUN" = "true" ]; then log_result info "[DRY-RUN] Würde installieren: ${missing[*]}"; return 0; fi
    wait_for_apt || return 1
    if DEBIAN_FRONTEND=noninteractive apt install -y -qq "${missing[@]}"; then
        for pkg in "${missing[@]}"; do log_result ok "$pkg installiert"; done
    else return 1; fi
}
