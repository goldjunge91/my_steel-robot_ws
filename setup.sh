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

print_step() {
  echo -e "${BLUE}[$(date +%H:%M:%S)]${NC} $1"
}

print_success() {
  echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}[WARNING]${NC} $1" >&2
}

print_error() {
  echo -e "${RED}[ERROR]${NC} $1" >&2
}

print_duration() {
  local start=$1
  local end=$2
  local duration=$((end - start))
  echo -e "${BLUE}[Duration: ${duration}s]${NC}"
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

wait_for_apt_lock() {
  local timeout=${1:-300}
  local waited=0
  local lock_files=(
    "/var/lib/dpkg/lock-frontend"
    "/var/lib/dpkg/lock"
    "/var/lib/apt/lists/lock"
  )
  
  while [ $waited -lt $timeout ]; do
    local locked=false
    for lock_file in "${lock_files[@]}"; do
      if sudo fuser "$lock_file" >/dev/null 2>&1; then
        locked=true
        break
      fi
    done
    
    if [ "$locked" = false ]; then
      return 0
    fi
    
    if [ $waited -eq 0 ]; then
      print_step "Waiting for apt/dpkg lock to be released..."
    fi
    
    sleep 2
    waited=$((waited + 2))
    
    if [ $((waited % 30)) -eq 0 ]; then
      print_step "Still waiting for apt lock... (${waited}s elapsed)"
    fi
  done
  
  print_error "Timeout waiting for apt lock after ${timeout}s"
  return 1
}

retry_command() {
  local max_attempts=${1:-3}
  local delay=${2:-5}
  shift 2
  local cmd=("$@")
  local attempt=1
  
  while [ $attempt -le $max_attempts ]; do
    if [ $attempt -gt 1 ]; then
      print_step "Retry attempt $attempt of $max_attempts..."
      sleep $delay
    fi
    
    if "${cmd[@]}"; then
      return 0
    fi
    
    attempt=$((attempt + 1))
  done
  
  return 1
}

verify_ros_environment() {
  if [ -z "${ROS_DISTRO:-}" ]; then
    print_error "ROS_DISTRO not set after sourcing ROS environment"
    return 1
  fi
  
  if ! command -v ros2 >/dev/null 2>&1; then
    print_warning "ros2 command not found in PATH"
    return 1
  fi
  
  print_success "ROS environment verified (ROS_DISTRO=$ROS_DISTRO)"
  return 0
}

verify_vcstool_in_path() {
  if command -v vcs >/dev/null 2>&1; then
    print_success "vcstool is available in PATH"
    return 0
  fi
  
  print_warning "vcstool not found in PATH after installation"
  
  # Try to find vcstool in common Python bin directories
  local python_bin_dir
  python_bin_dir=$(python3 -c 'import sys, pathlib; print(pathlib.Path(sys.executable).resolve().parent)' 2>/dev/null)
  
  if [ -n "$python_bin_dir" ] && [ -f "$python_bin_dir/vcs" ]; then
    export PATH="$python_bin_dir:$PATH"
    print_success "Added $python_bin_dir to PATH"
    return 0
  fi
  
  # Try user local bin
  if [ -f "$HOME/.local/bin/vcs" ]; then
    export PATH="$HOME/.local/bin:$PATH"
    print_success "Added $HOME/.local/bin to PATH"
    return 0
  fi
  
  return 1
}

# --- Haupt-Workflow ---
main() {
    local script_start=$(date +%s)
    ROS_DISTRO=${ROS_DISTRO:-humble}

    print_header "ROS-Umgebung wird eingerichtet (Distribution: $ROS_DISTRO)"
    local step_start=$(date +%s)
    
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
      AMENT_TRACE_SETUP_FILES=""
    fi
    export AMENT_TRACE_SETUP_FILES

    print_step "Sourcing ROS setup script from $ROS_SETUP..."
    if safe_source "$ROS_SETUP"; then
      print_success "ROS setup script loaded successfully"
      if verify_ros_environment; then
        local step_end=$(date +%s)
        print_duration $step_start $step_end
      else
        print_warning "ROS environment verification failed"
      fi
    else
      print_warning "ROS setup script not found at $ROS_SETUP. Continuing without ROS environment."
    fi

    if [ -f src/ros2.repos ]; then
      print_header "VCS Repositories werden importiert"
      step_start=$(date +%s)
      
      if ! command -v vcs >/dev/null 2>&1; then
        print_step "vcstool not found, installing..."
        python3 -m pip install --upgrade pip >/dev/null 2>&1 || true
        
        if python3 -m pip install vcstool >/dev/null 2>&1; then
          print_success "vcstool installed successfully"
        else
          print_error "Failed to install vcstool"
        fi
        
        if ! verify_vcstool_in_path; then
          print_error "vcstool installation failed or not in PATH"
        fi
      fi
      
      if command -v vcs >/dev/null 2>&1; then
        print_step "Importing repositories from src/ros2.repos..."
        if vcs import src < src/ros2.repos; then
          print_success "Repositories imported successfully"
          local step_end=$(date +%s)
          print_duration $step_start $step_end
        else
          print_warning "Failed to import VCS repositories. Continuing anyway."
        fi
      else
        print_error "Cannot import repositories: vcs command not available"
      fi
    else
      print_step "No 'src/ros2.repos' file found, skipping VCS import"
    fi

    if command -v rosdep >/dev/null 2>&1; then
      print_header "Abhängigkeiten mit rosdep werden installiert"
      step_start=$(date +%s)
      
      if command -v apt-get >/dev/null 2>&1 && [ -z "${APT_UPDATED:-}" ]; then
        print_step "Waiting for apt lock and updating package lists..."
        
        if wait_for_apt_lock 300; then
          if retry_command 3 5 sudo apt-get update -y -qq; then
            print_success "Package lists updated successfully"
            export APT_UPDATED=1
          else
            print_warning "'apt-get update' failed after retries. Continuing anyway."
          fi
        else
          print_error "Failed to acquire apt lock. Skipping apt-get update."
        fi
      fi

      print_step "Updating rosdep cache..."
      if retry_command 3 5 sudo rosdep update --rosdistro="$ROS_DISTRO"; then
        print_success "rosdep cache updated successfully"
      else
        print_warning "'rosdep update' failed after retries. Attempting to continue with existing cache."
      fi
      
      print_step "Installing dependencies with rosdep..."
      if sudo rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO"; then
        print_success "rosdep dependencies installed successfully"
        local step_end=$(date +%s)
        print_duration $step_start $step_end
      else
        print_warning "'rosdep install' failed. Some dependencies may be missing."
        print_warning "This could be due to:"
        print_warning "  - Missing package.xml files in src/ packages"
        print_warning "  - Unavailable packages in ROS repositories"
        print_warning "  - Network connectivity issues"
      fi
    else
      print_error "rosdep is not available; skipping dependency resolution."
    fi

    local script_end=$(date +%s)
    echo ""
    print_success "Setup script completed"
    print_duration $script_start $script_end
    
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