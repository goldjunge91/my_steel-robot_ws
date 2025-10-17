#!/usr/bin/env bash
set -euo pipefail

# Colored output keeps local runs readable
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

github_summary() {
  if [ "${GITHUB_ACTIONS:-false}" = "true" ] && [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    echo "$*" >> "$GITHUB_STEP_SUMMARY"
  fi
}

github_notice() {
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    echo "::notice title=${1}::${2}"
  fi
}

log_step() {
  echo -e "${BLUE}[SETUP]${NC} $1"
}

log_success() {
  echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
  echo -e "${YELLOW}[WARNING]${NC} $1" >&2
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $1" >&2
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log_error "Command '$1' not found in PATH"
    exit 1
  fi
}

ROS_DISTRO=${ROS_DISTRO:-humble}
log_step "ROS workspace setup gestartet (ROS_DISTRO=${ROS_DISTRO})"

require_cmd rosdep
require_cmd sudo

if [ -f "src/ros2.repos" ]; then
  log_step "Importiere Repositories aus src/ros2.repos"
  if command -v vcs >/dev/null 2>&1; then
    if command -v envsubst >/dev/null 2>&1; then
      envsubst < src/ros2.repos | vcs import src
    else
      log_warning "envsubst nicht gefunden – ros2.repos wird ohne Variablenersetzung importiert"
      vcs import src < src/ros2.repos
    fi
    log_success "Repositories importiert"
  else
    log_warning "vcs nicht gefunden – überspringe Repository-Import"
  fi
else
  log_warning "Keine src/ros2.repos gefunden – überspringe VCS-Import"
fi

log_step "Aktualisiere apt Paketquellen"
sudo apt-get update -y

log_step "Aktualisiere rosdep-Datenbank"
rosdep update --rosdistro="$ROS_DISTRO"

log_step "Installiere Abhängigkeiten via rosdep"
rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO"
log_success "Abhängigkeiten installiert"

log_success "Setup abgeschlossen"

if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
  github_summary "## ✅ Setup Completed"
  github_summary "**ROS Distribution:** $ROS_DISTRO"
  github_notice "Setup Complete" "ROS2 workspace setup completed successfully"
fi
