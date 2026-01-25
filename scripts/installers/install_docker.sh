#!/usr/bin/env bash
set -euo pipefail
DOCKER_PKGS=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin)
DOCKER_GPG_PATH="/etc/apt/keyrings/docker.gpg"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$SCRIPT_DIR/helpers.sh"
: ${REAL_USER:=${SUDO_USER:-$USER}}

# 1. Prüfung: Ist Docker bereits vollständig installiert?
if command -v docker &>/dev/null && dpkg -l | grep -q docker-ce; then
    log SUCCESS "Docker Engine & CLI bereits installiert"
    exit 0
fi

# 2. Konflikte bereinigen
if dpkg -l | grep -q moby-tini; then
    log_task "Entferne moby-tini (Konflikt)"
    apt-get remove -y -qq moby-tini >/dev/null 2>&1 || true
    log_result ok "moby-tini entfernt"
fi

# 3. Docker GPG-Schlüssel (Überspringen, wenn vorhanden)
if [ -f "$DOCKER_GPG_PATH" ]; then
    log_result skip "Docker GPG-Schlüssel existiert bereits"
else
    if [ "$DRY_RUN" = "true" ]; then
        log_task "[DRY-RUN] Würde GPG-Key herunterladen und dearmoren"
    else
        log_task "GPG-Schlüssel hinzufügen"
        mkdir -p /etc/apt/keyrings
        # --batch verhindert interaktive Rückfragen von gpg
        if curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor --batch -o "$DOCKER_GPG_PATH"; then
            log_result ok "GPG-Schlüssel erfolgreich hinzugefügt"
        else
            log_result fail "Fehler beim Hinzufügen des GPG-Schlüssels"
            exit 1
        fi
    fi
fi

# 4. Repository konfigurieren
log_task "Docker-Repository konfigurieren"
ARCH=$(dpkg --print-architecture)
DISTRO=$(lsb_release -cs)
echo "deb [arch=$ARCH signed-by=$DOCKER_GPG_PATH] https://download.docker.com/linux/ubuntu $DISTRO stable" | tee /etc/apt/sources.list.d/docker.list >/dev/null
log_result ok "Repository ($DISTRO)"

# 5. Installation
wait_for_apt || exit 1
if ! run_action "apt update (docker)" apt update -y -qq; then
    log WARN "apt update (docker) hatte Fehler, fahre trotzdem fort..."
fi

for pkg in "${DOCKER_PKGS[@]}"; do
    if ! install_and_check "$pkg"; then
        log ERROR "Fehler bei Paket: $pkg"
        exit 1
    fi
done

# 6. Dienst starten
systemctl enable --now docker >/dev/null 2>&1 || log WARN "Docker-Dienst konnte nicht aktiviert werden"
log_result ok "Docker-Dienst"

# 7. Gruppen-Management (Hardware-agnostisch)
WANTED_GROUPS=("docker" "dialout" "video" "plugdev" "gpio" "i2c" "spi")
EXISTING_GROUPS=()
log_task "Prüfe verfügbare Benutzergruppen..."

for grp in "${WANTED_GROUPS[@]}"; do
    if getent group "$grp" >/dev/null; then
        EXISTING_GROUPS+=("$grp")
    else
        log_result skip "Gruppe '$grp' nicht auf diesem System"
    fi
done

if [ ${#EXISTING_GROUPS[@]} -gt 0 ]; then
    GROUP_LIST=$(IFS=,; echo "${EXISTING_GROUPS[*]}")
    if run_action "Benutzer $REAL_USER zu Gruppen hinzufügen" usermod -aG "$GROUP_LIST" "$REAL_USER"; then
        log_result ok "Zugefügt zu: $GROUP_LIST"
    else
        log ERROR "Gruppenzuweisung fehlgeschlagen"
        exit 1
    fi
fi

exit 0
