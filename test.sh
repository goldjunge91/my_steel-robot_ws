#!/bin/bash
set -euo pipefail

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# GitHub Actions helpers (kept lightweight)
github_summary() {
  if [ "${GITHUB_ACTIONS:-false}" = "true" ] && [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    echo "$*" >> "$GITHUB_STEP_SUMMARY"
  fi
}

github_annotation() {
  local type="$1"
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

# Timing helpers for consistent colored output
start_time=$(date +%s)

log_with_color() {
  local color="$1"
  local label="$2"
  shift 2
  local message="$*"
  local now
  now=$(date +%s)
  local elapsed=$((now - start_time))
  printf "%b[%s - %ss]%b %s\n" "$color" "$label" "$elapsed" "$NC" "$message"
}

log_step() {
  log_with_color "$BLUE" "TEST" "$@"
}

log_success() {
  log_with_color "$GREEN" "SUCCESS" "$@"
}

log_warning() {
  log_with_color "$YELLOW" "WARNING" "$@"
}

log_error() {
  log_with_color "$RED" "ERROR" "$@"
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

log_step "Starting test execution"

if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  export AMENT_TRACE_SETUP_FILES=""
fi

log_step "Sourcing workspace environment"
if [ -f install/setup.bash ]; then
  if safe_source install/setup.bash; then
    log_success "Workspace environment sourced"
  else
    log_error "Failed to source install/setup.bash"
    github_error "Environment" "Failed to source install/setup.bash"
    exit 1
  fi
else
  log_error "install/setup.bash not found — run build before tests"
  github_error "Environment" "install/setup.bash not found; did you run build.sh?"
  exit 1
fi

log_step "Checking colcon availability"
if ! command -v colcon >/dev/null 2>&1; then
  log_warning "colcon is not available; skipping tests"
  github_warning "Test" "colcon is not available; tests skipped"
  exit 0
fi

log_step "Running colcon test --merge-install"
set +e
colcon test --merge-install
test_exit=$?
set -e
if [ $test_exit -ne 0 ]; then
  log_error "colcon test exited with code $test_exit"
  github_error "colcon test" "colcon test exited with code $test_exit"
  exit $test_exit
fi
log_success "colcon test completed"

log_step "Collecting test results (colcon test-result --all --verbose)"
set +e
colcon test-result --all --verbose
test_result_exit=$?
set -e

if [ $test_result_exit -ne 0 ]; then
  log_error "Tests reported failures (exit code $test_result_exit)"
  github_error "Test Results" "colcon test-result reported failures"
  exit $test_result_exit
fi

log_success "colcon test-result completed with no failures"

if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
  github_summary "## ✅ Tests Passed"
  github_summary ""
  github_summary "colcon test and colcon test-result completed successfully."
fi

log_success "Test script completed successfully"
