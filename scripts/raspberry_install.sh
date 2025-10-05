#!/usr/bin/env bash
# set -euo pipefail ist eine strikte Fehlereinstellung. Einzelne Befehle,
# die fehlschlagen dürfen, werden mit '|| true' behandelt.
set -euo pipefail

# --- Farbdefinitionen für die Ausgabe ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# --- Hilfsfunktionen ---
print_header() {
  echo -e "\n${BLUE}=======================================================================${NC}"
  echo -e "${BLUE}===== $1${NC}"
  echo -e "${BLUE}=======================================================================${NC}"
}

safe_source() {
  local file="$1"
  if [ -f "$file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$file"
    local status=$?
    set -u
    return $status
  fi
  return 1
}

# --- Haupt-Workflow ---
main() {
    ROS_DISTRO=${ROS_DISTRO:-humble}

    print_header "ROS-Robot wird eingerichtet (Distribution: $ROS_DISTRO)"
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
#     if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
#       AMENT_TRACE_SETUP_FILES=""
#     fi
#     export AMENT_TRACE_SETUP_FILES

    if safe_source "$ROS_SETUP"; then
      echo "ROS-Setup-Skript unter $ROS_SETUP erfolgreich geladen."
    else
      echo -e "${YELLOW}WARNUNG:${NC} ROS-Setup-Skript nicht unter $ROS_SETUP gefunden. Fahre ohne ROS-Umgebung fort." >&2
    fi

    if [ -f src/ros2.repos ]; then
      print_header "VCS Repositories werden importiert"
      if ! command -v vcs >/dev/null 2>&1; then
        echo "vcstool nicht gefunden, wird installiert..."
        python3 -m pip install --upgrade pip >/dev/null 2>&1 || true
        python3 -m pip install vcstool >/dev/null 2>&1
        VCS_BIN_DIR=$(python3 -c 'import sys, pathlib; print(pathlib.Path(sys.executable).resolve().parent)')
        export PATH="$VCS_BIN_DIR:$PATH"
      fi
      if vcs import src < src/ros2.repos; then
        echo -e "${GREEN}SUCCESS:${NC} Repositories erfolgreich importiert."
      else
        echo -e "${YELLOW}WARNUNG:${NC} Fehler beim Importieren der VCS-Repositories. Der Prozess wird fortgesetzt."
      fi
    else
      echo -e "${YELLOW}INFO:${NC} Keine 'src/ros2.repos'-Datei gefunden, überspringe den VCS-Import."
    fi

    if command -v rosdep >/dev/null 2>&1; then
      print_header "Abhängigkeiten mit rosdep werden installiert"
      if command -v apt-get >/dev/null 2>&1 && [ -z "${APT_UPDATED:-}" ]; then
          while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
            echo "Warte darauf, dass ein anderer apt/dpkg-Prozess beendet wird..."
            sleep 1
          done
          echo "Aktualisiere Paketlisten mit apt-get..."
          sudo apt-get update -y -qq || echo -e "${YELLOW}WARNUNG:${NC} 'apt-get update' ist fehlgeschlagen."
          export APT_UPDATED=1
      fi

      echo "Aktualisiere rosdep-Cache..."
      sudo rosdep update --rosdistro="$ROS_DISTRO" || echo -e "${YELLOW}WARNUNG:${NC} 'rosdep update' ist fehlgeschlagen."
      
      echo "Installiere Abhängigkeiten..."
      if sudo rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO"; then
        echo -e "${GREEN}SUCCESS:${NC} rosdep-Abhängigkeiten erfolgreich installiert."
      else
        echo -e "${YELLOW}WARNUNG:${NC} 'rosdep install' ist fehlgeschlagen. Einige Abhängigkeiten fehlen möglicherweise."
      fi
    else
      echo -e "${RED}ERROR:${NC} rosdep ist nicht verfügbar; die Auflösung der Abhängigkeiten wird übersprungen." >&2
    fi

    echo -e "\n${GREEN}Setup-Skript beendet. Der Workflow wird fortgesetzt.${NC}"
    # Gib immer den Exit-Code 0 zurück, um die CI/CD-Pipeline nicht zu blockieren.
    exit 0
}

# Starte die Hauptfunktion des Skripts
main