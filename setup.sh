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

# GitHub Actions functions
github_summary() {
  if [ "${GITHUB_ACTIONS:-false}" = "true" ] && [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    echo "$*" >> "$GITHUB_STEP_SUMMARY"
  fi
}

github_annotation() {
  local type="$1"  # error, warning, notice
  local title="$2"
  local message="$3"
  local file="${4:-}"
  local line="${5:-}"
  
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    local annotation="::${type}"
    if [ -n "$file" ]; then
      annotation="${annotation} file=${file}"
    fi
    if [ -n "$line" ]; then
      annotation="${annotation},line=${line}"
    fi
    annotation="${annotation} title=${title}::${message}"
    echo "$annotation"
  fi
}

github_error() {
  github_annotation "error" "$1" "$2" "$3" "$4"
}

github_warning() {
  github_annotation "warning" "$1" "$2" "$3" "$4"
}

github_notice() {
  github_annotation "notice" "$1" "$2" "$3" "$4"
}

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
  
  while [ $waited -lt "$timeout" ]; do
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
  
  while [ $attempt -le "$max_attempts" ]; do
    if [ $attempt -gt 1 ]; then
      print_step "Retry attempt $attempt of $max_attempts..."
      sleep "$delay"
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

# --- Main setup logic (simplified following athackst approach) ---
ROS_DISTRO=${ROS_DISTRO:-humble}

print_header "ROS workspace setup (Distribution: $ROS_DISTRO)"

# Import VCS repositories if ros2.repos exists
if [ -f "src/ros2.repos" ]; then
    print_step "Importing repositories from src/ros2.repos..."
    if vcs import src < src/ros2.repos; then
        print_success "VCS repositories imported successfully"
    else
        print_warning "VCS import failed, continuing anyway..."
    fi
else
    print_step "No src/ros2.repos found, skipping VCS import"
fi

# Update package lists and rosdep (following athackst approach)
print_step "Updating package lists..."
sudo apt-get update

print_step "Updating rosdep..."
rosdep update --rosdistro=$ROS_DISTRO

print_step "Installing dependencies with rosdep..."
if rosdep install --from-paths src --ignore-src -y --rosdistro=$ROS_DISTRO; then
    print_success "Dependencies installed successfully"
else
    print_warning "Some dependencies failed to install, continuing anyway..."
fi

print_success "Setup completed successfully"

# GitHub Actions summary
if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "## ✅ Setup Completed"
    github_summary "**ROS Distribution:** $ROS_DISTRO"
    github_notice "Setup Complete" "ROS2 workspace setup completed successfully"
fi


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
#     )
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
