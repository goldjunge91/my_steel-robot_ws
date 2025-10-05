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
    # Prevent recursion by checking if already sourced
    if [[ ":$SOURCED_FILES:" != *":$file:"* ]]; then
      export SOURCED_FILES="${SOURCED_FILES:-}:$file:"
      set +u
      # shellcheck disable=SC1090
      source "$file"
      local status=$?
      set -u
      return $status
    else
      echo "Skipping already sourced file: $file"
      return 0
    fi
  fi
  return 1
}

# --- Haupt-Workflow ---
main() {
    ROS_DISTRO=${ROS_DISTRO:-humble}

    print_header "ROS-Umgebung wird eingerichtet (Distribution: $ROS_DISTRO)"
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
      AMENT_TRACE_SETUP_FILES=""
    fi
    export AMENT_TRACE_SETUP_FILES

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


# #!/bin/bash
# set -euo pipefail

# safe_source() {
#   local file="$1"
#   if [ -f "$file" ]; then
#     set +u
#     # shellcheck disable=SC1090
#     source "$file"
#     local status=$?
#     set -u
#     return $status
#   fi
#   return 1
# }

# ROS_DISTRO=${ROS_DISTRO:-humble}
# ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# # Some ROS setup scripts expect this variable to exist even when tracing is disabled.
# if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
#   AMENT_TRACE_SETUP_FILES=""
# fi
# export AMENT_TRACE_SETUP_FILES

# if safe_source "$ROS_SETUP"; then
#   :
# else
#   echo "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
# fi

# if [ -f src/ros2.repos ]; then
#   if ! command -v vcs >/dev/null 2>&1; then
#     python3 -m pip install --upgrade pip >/dev/null 2>&1 || true
#     python3 -m pip install vcstool >/dev/null 2>&1
#     VCS_BIN_DIR=$(python3 - <<'PY'
# import sys
# from pathlib import Path
# print(Path(sys.executable).resolve().parent)
# PY
# )
#     export PATH="$VCS_BIN_DIR:$PATH"
#   fi
#   vcs import src < src/ros2.repos || true
# fi

# if command -v rosdep >/dev/null 2>&1; then
#   if command -v apt-get >/dev/null 2>&1; then
#     if [ -z "${APT_UPDATED:-}" ]; then
#       # The '|| true' ensures that the script continues even if this command fails.
#       # This prevents the entire build from stopping due to a transient network issue.
#       # The -y and -qq flags make the output less verbose and non-interactive.
#       sudo apt-get update -y -qq || true
#       export APT_UPDATED=1
#     fi
#   fi
#   # FIX: Running 'rosdep update' with sudo to initialize the cache for the root user.
#   # This prevents an error when 'sudo rosdep install' is called later.
#   sudo rosdep update --rosdistro="$ROS_DISTRO" || true
#   # rosdep install requires sudo to install system packages.
#   sudo rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO" || true
# else
#   echo "rosdep is not available; skipping dependency resolution." >&2
# fi


# ## Return an finish status that run can move on
# exit 0