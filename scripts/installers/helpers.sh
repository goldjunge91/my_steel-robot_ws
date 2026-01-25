#!/usr/bin/env bash
# Shared helper functions for installer modules.
# Defines functions only if they are not already defined to avoid re-definition when sourced from main script.
# Guard gegen mehrfaches Laden
: ${HELPERS_LOADED:=}
if [ -n "$HELPERS_LOADED" ]; then
    return 0
fi
HELPERS_LOADED=1

# Require REAL_USER and USER_HOME to be exported by caller.
# --- BASIS VARIABLEN ---
# Falls vom Hauptskript nicht gesetzt, versuchen wir sie hier zu bestimmen
: ${REAL_USER:=${SUDO_USER:-$USER}}
: ${USER_HOME:="/home/$REAL_USER"}
: ${BACKUP_DIR:="/tmp/setup_backups/$(date +%Y%m%d_%H%M%S)"}

# Simple log functions (use main script colors if available)
# log() { if declare -f log >/dev/null 2>&1; then log "$@"; else echo "[$1] ${*:2}"; fi }
# log_step() { if declare -f log_step >/dev/null 2>&1; then log_step "$@"; else echo "== $* =="; fi }
# log_task() { if declare -f log_task >/dev/null 2>&1; then log_task "$@"; else echo "> $*"; fi }
# log_result() { if declare -f log_result >/dev/null 2>&1; then log_result "$@"; else echo "RESULT: $*"; fi }
# record_failure() { if declare -f record_failure >/dev/null 2>&1; then record_failure "$@"; else echo "FAIL: $*"; fi }

# --- SICHERE LOG-FUNKTIONEN (Verhindert Segmentation Fault) ---

# log: Standard-Ausgabe
if ! declare -f log >/dev/null 2>&1; then
    log() {
        local level="$1"; shift
        echo "[$level] $*"
    }
fi

if ! declare -f log_step >/dev/null 2>&1; then
    log_step() {
        echo -e "\n=== $* ==="
    }
fi

if ! declare -f log_task >/dev/null 2>&1; then
    log_task() {
        echo "→ $*"
    }
fi

if ! declare -f log_result >/dev/null 2>&1; then
    log_result() {
        local status="$1"; shift
        case "$status" in
            ok)   echo "  ✓ $*" ;;
            fail) echo "  ✗ ERROR: $*" >&2 ;;
            skip) echo "  ○ SKIP: $*" ;;
            *)    echo "  - $*" ;;
        esac
    }
fi

if ! declare -f record_failure >/dev/null 2>&1; then
    record_failure() {
        echo "FAILURE RECORDED: $*" >> "/tmp/setup_errors.log"
    }
fi
# ==============================================================================
# ÄNDERUNGS-TRACKING (Verhindert den Segmentation Fault)
# ==============================================================================

if ! declare -f record_change >/dev/null 2>&1; then
    record_change() {
        # Initialisiere Array falls nicht vorhanden (für Sub-Shells)
        if [ -z "${APPLIED_CHANGES+x}" ]; then
            APPLIED_CHANGES=()
        fi
        APPLIED_CHANGES+=("$1")
    }
fi

if ! declare -f rollback_changes >/dev/null 2>&1; then
    rollback_changes() {
        echo "[INFO] Rollback in dieser Sub-Shell nicht implementiert."
    }
fi
# ==============================================================================
# AKTIONEN & DATEI-OPERATIONEN
# ==============================================================================

run_action() {
    local desc="$1"; shift
    if [ "${DRY_RUN:-false}" = "true" ]; then
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
        log_result ok "Backup: $file"
    fi
}

write_if_changed() {
    local dest="$1"
    local tmp
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] Würde Datei schreiben: $dest"
        cat > /dev/null # Verbraucht den Input von stdin, damit das Skript nicht hängen bleibt
        return 0
    fi
    tmp=$(mktemp)
    cat >"$tmp"
    if [ -e "$dest" ]; then
        if cmp -s "$tmp" "$dest"; then
            log_result skip "Keine Änderung für $dest"
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
            log_result fail "apt lock timeout"
            return 1
        fi
        echo "Warte auf apt-Lock..."
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
    wait_for_apt
    if apt-get install -y -qq "$pkg"; then
        log_result ok "$pkg installiert"
        record_change "pkg:$pkg"
        return 0
    else
        log_result fail "$pkg Installation"
        return 1
    fi
}
