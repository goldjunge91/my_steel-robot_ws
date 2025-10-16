#!/usr/bin/env bash
# set -e beendet das Skript sofort, wenn ein Befehl fehlschlägt.
set -e

# Change to workspace directory in GitHub Actions
cd /github/workspace 2>/dev/null || cd "${GITHUB_WORKSPACE}" 2>/dev/null || true

# Fix git safe.directory issue in GitHub Actions
git config --global --add safe.directory '*' 2>/dev/null || true
git config --global --add safe.directory /github/workspace 2>/dev/null || true

# --- Farbdefinitionen für die Ausgabe ---
# Diese ANSI-Escape-Codes fügen den "echo"-Ausgaben Farben hinzu.
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color (Setzt die Farbe auf Standard zurück)

# --- Hilfsfunktionen ---

# Diese Funktion druckt einen gut sichtbaren Header für jeden Arbeitsschritt.
print_header() {
  echo -e "\n${BLUE}=======================================================================${NC}"
  echo -e "${BLUE}===== $1${NC}"
  echo -e "${BLUE}=======================================================================${NC}"
}

# Diese Funktion führt einen Befehl aus, misst die benötigte Zeit und gibt eine klare Erfolgs- oder Fehlermeldung aus.
run_command() {
  local description="$1"
  # 'shift' entfernt das erste Argument (die Beschreibung),
  # sodass $@ alle restlichen Argumente (den auszuführenden Befehl) enthält.
  shift
  local command_to_run="$*"

  print_header "$description"

  local start_time=$SECONDS

  # Führt den Befehl aus und prüft den Exit-Code
  if $command_to_run; then
    local duration=$((SECONDS - start_time))
    echo -e "\n${GREEN}SUCCESS:${NC} '$description' wurde in ${duration}s erfolgreich abgeschlossen."
  else
    local exit_code=$?
    local duration=$((SECONDS - start_time))
    echo -e "\n${RED}FEHLER:${NC} '$description' ist nach ${duration}s mit Exit-Code ${exit_code} fehlgeschlagen."
    # Beendet das gesamte Skript mit dem fehlerhaften Exit-Code
    exit $exit_code
  fi
}

# --- Haupt-Workflow ---

# Die 'main' Funktion kapselt die gesamte Logik.
main() {
  echo -e "${YELLOW}Starte den CI/CD Workflow...${NC}"

  # Führe die einzelnen Schritte über die 'run_command' Funktion aus.
  # Dies sorgt für eine konsistente und informative Ausgabe.
  run_command "Setup-Skript wird ausgeführt" ./setup.sh
  run_command "Build-Skript wird ausgeführt" ./build.sh
  run_command "Test-Skript wird ausgeführt"  ./test.sh

  echo -e "\n${GREEN}=======================================================================${NC}"
  echo -e "${GREEN}===== Workflow erfolgreich abgeschlossen! ====="
  echo -e "${GREEN}=======================================================================${NC}"
}

# Starte die Hauptfunktion des Skripts
main
