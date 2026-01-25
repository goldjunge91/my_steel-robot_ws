#!/usr/bin/env bash
set -euo pipefail
DOCKER_PKGS=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-}}

# Prüfung: Docker UND Engine müssen da sein
if command -v docker &>/dev/null && dpkg -l | grep -q docker-ce; then
    log SUCCESS "Docker Engine & CLI bereits installiert"
    exit 0
fi

# Remove moby-tini if present
if dpkg -l | grep -q moby-tini; then
    log_task "Entferne moby-tini (Konflikt)"
    apt-get remove -y -qq moby-tini >/dev/null 2>&1 || true
    log_result ok "moby-tini entfernt"
fi

log_task "GPG-Schlüssel hinzufügen"
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg || { log_result fail "GPG-Key Download"; exit 1; }
log_result ok "GPG-Schlüssel"

log_task "Docker-Repository konfigurieren"
ARCH=$(dpkg --print-architecture)
echo "deb [arch=$ARCH signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list >/dev/null
log_result ok "Repository"

wait_for_apt || exit 1
if ! run_action "apt update (docker)" apt update -y -qq; then
    log WARN "apt update (docker) had errors, proceeding..."
fi

for pkg in "${DOCKER_PKGS[@]}"; do
    if ! install_and_check "$pkg"; then
        log ERROR "Fehler bei Paket: $pkg"
        exit 1
    fi
done

if ! systemctl enable --now docker; then
    log WARN "systemctl enable fehlgeschlagen"
fi
log_result ok "Docker-Dienst"

if docker --version &>/dev/null; then
    log_result ok "Docker: $(docker --version)"
else
    log_result fail "Docker nicht verfügbar"
    exit 1
fi

if docker compose version &>/dev/null; then
    log_result ok "Docker Compose: $(docker compose version)"
else
    log_result fail "Docker Compose nicht verfügbar"
    exit 1
fi
# Liste der gewünschten Gruppen
WANTED_GROUPS=("docker" "dialout" "video" "plugdev" "gpio" "i2c" "spi")
EXISTING_GROUPS=()
log_task "Prüfe verfügbare Benutzergruppen..."
# Prüfe jede Gruppe, ob sie im System existiert
for grp in "${WANTED_GROUPS[@]}"; do
    if getent group "$grp" >/dev/null; then
        EXISTING_GROUPS+=("$grp")
    else
        log_result skip "Gruppe '$grp' existiert nicht auf diesem System (übersprungen)"
    fi
done

# Nur hinzufügen, wenn wir existierende Gruppen gefunden haben
if [ ${#EXISTING_GROUPS[@]} -gt 0 ]; then
    # Array zu kommagetrennter Liste umwandeln
    GROUP_LIST=$(IFS=,; echo "${EXISTING_GROUPS[*]}")
    
    if run_action "Benutzer $REAL_USER zu Gruppen hinzufügen ($GROUP_LIST)" usermod -aG "$GROUP_LIST" "$REAL_USER"; then
        log_result ok "Benutzer erfolgreich zugewiesen"
    else
        log_result fail "Fehler beim Zuweisen der Gruppen"
        exit 1
    fi
else
    log_result skip "Keine der Zielgruppen gefunden."
fi

exit 0
