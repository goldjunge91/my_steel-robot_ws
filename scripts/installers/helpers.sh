#!/usr/bin/env bash
# Shared helper functions for installer modules.
# Defines functions only if they are not already defined to avoid re-definition when sourced from main script.

: ${HELPERS_LOADED:=}
if [ -n "$HELPERS_LOADED" ]; then
    return 0
fi
HELPERS_LOADED=1

# Require REAL_USER and USER_HOME to be exported by caller.

# Simple log functions (use main script colors if available)
# log() { if declare -f log >/dev/null 2>&1; then log "$@"; else echo "[$1] ${*:2}"; fi }
# log_step() { if declare -f log_step >/dev/null 2>&1; then log_step "$@"; else echo "== $* =="; fi }
# log_task() { if declare -f log_task >/dev/null 2>&1; then log_task "$@"; else echo "> $*"; fi }
# log_result() { if declare -f log_result >/dev/null 2>&1; then log_result "$@"; else echo "RESULT: $*"; fi }
# record_failure() { if declare -f record_failure >/dev/null 2>&1; then record_failure "$@"; else echo "FAIL: $*"; fi }

# --- SICHERE LOG-FUNKTIONEN (Verhindert Segmentation Fault) ---

# log: Standard-Ausgabe
if ! declare -f log >/dev/null 2>&1; then
    log() { local lvl="$1"; shift; echo "[$lvl] $*"; }
fi

# log_step: Große Überschriften
if ! declare -f log_step >/dev/null 2>&1; then
    log_step() { echo -e "\n=== $* ==="; }
fi

# log_task: Einzelschritte mit Pfeil
if ! declare -f log_task >/dev/null 2>&1; then
    log_task() { echo "→ $*"; }
fi

# log_result: Ergebnis eines Schritts
if ! declare -f log_result >/dev/null 2>&1; then
    log_result() { echo "  ✓ $*"; }
fi

# record_failure: Fehler protokollieren
if ! declare -f record_failure >/dev/null 2>&1; then
    record_failure() { echo "✗ ERROR: $*" >&2; }
fi

# run_action: respects DRY_RUN/VERBOSE if defined in caller
run_action() {
    local desc="$1"; shift
    if [ "${DRY_RUN:-false}" = "true" ]; then
        log_task "[DRY-RUN] $desc"
        if [ "${VERBOSE:-false}" = "true" ]; then
            "$@" || true
        fi
        return 0
    fi
    log_task "$desc"
    "$@"
}

# Backup helpers (use caller-provided BACKUP_DIR or default)
: ${BACKUP_DIR:="/tmp/setup_backups/$(date +%Y%m%d_%H%M%S)"}
mkdir -p "$BACKUP_DIR" 2>/dev/null || true
backup_file() {
    local file="$1"
    if [ -e "$file" ]; then
        mkdir -p "$BACKUP_DIR$(dirname "$file")" 2>/dev/null || true
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

# write_if_changed expects content on stdin
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
        mkdir -p "$(dirname "$dest")" 2>/dev/null || true
    fi
    mv "$tmp" "$dest"
    chown "${REAL_USER:-$USER}:" "$dest" 2>/dev/null || true
    log_result ok "Wrote $dest"
    # record change if function exists
    if declare -f record_change >/dev/null 2>&1; then
        record_change "file:$dest"
    fi
}

# download, optional verify, run
download_and_verify_and_run(){
    local url="$1"; local dest="$2"; local sha256="$3"
    run_action "Download $url to $dest" curl -fsSL "$url" -o "$dest"
    if [ -n "$sha256" ]; then
        echo "$sha256  $dest" | sha256sum -c - || { log ERROR "Checksum failed for $dest"; return 1; }
    fi
    chmod +x "$dest"
    run_action "Execute $dest" bash "$dest"
}

# wait_for_apt (copied minimal)
wait_for_apt(){
    local timeout=300 elapsed=0
    while fuser /var/lib/dpkg/lock >/dev/null 2>&1 || fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 || fuser /var/lib/apt/lists/lock >/dev/null 2>&1; do
        if [ $elapsed -ge $timeout ]; then
            log ERROR "apt lock timeout after ${timeout}s"
            return 1
        fi
        echo "Warte auf apt-Lock..."
        sleep 2
        elapsed=$((elapsed+2))
    done
    return 0
}

# record_change and rollback placeholders (if main defines them they'll be used)
record_change(){ if declare -f record_change >/dev/null 2>&1; then record_change "$@"; else APPLIED_CHANGES+=("$1"); fi }
rollback_changes(){ if declare -f rollback_changes >/dev/null 2>&1; then rollback_changes "$@"; else echo "No rollback implementation"; fi }

# End of helpers
