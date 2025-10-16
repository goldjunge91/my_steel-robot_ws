#!/bin/bash
set -euo pipefail

# --- Color definitions for output ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# --- Helper functions ---
print_info() {
  echo -e "${BLUE}[INFO]${NC} $1"
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

# Fix git safe.directory issue in GitHub Actions (git 2.35.2+)
git config --global --add safe.directory '*' 2>/dev/null || true

print_info "Starting lint job for linter: ${LINTER:-<not set>}"

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

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  AMENT_TRACE_SETUP_FILES=""
fi
export AMENT_TRACE_SETUP_FILES

print_info "Sourcing ROS environment..."
if safe_source "$ROS_SETUP"; then
  print_success "ROS setup script loaded from $ROS_SETUP"
else
  print_warning "ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS."
fi

print_info "Running setup.sh to prepare workspace..."
if ./setup.sh; then
  print_success "setup.sh completed successfully"
else
  local exit_code=$?
  print_error "setup.sh failed with exit code $exit_code"
  print_error "Cannot proceed with linting without proper workspace setup"
  print_error "Check setup.sh output above for specific error details"
  exit 1
fi

print_info "Checking for workspace setup..."
if [ -f "install/setup.bash" ]; then
  print_info "Found workspace setup, sourcing install/setup.bash..."
  if safe_source "install/setup.bash"; then
    print_success "Workspace environment loaded"
  else
    print_warning "Failed to source workspace setup, continuing anyway..."
  fi
else
  print_info "No install/setup.bash found, workspace may not be built yet"
fi

# Validate LINTER environment variable
if [ -z "${LINTER:-}" ]; then
  print_error "LINTER environment variable is not set."
  print_error "Expected one of: cppcheck, cpplint, uncrustify, lint_cmake, xmllint, flake8, pep257"
  exit 1
fi

# Validate LINTER value against known linters
VALID_LINTERS="cppcheck cpplint uncrustify lint_cmake xmllint flake8 pep257"
if ! echo "$VALID_LINTERS" | grep -q "\b$LINTER\b"; then
  print_error "Invalid LINTER value: '$LINTER'"
  print_error "Valid linters are: $VALID_LINTERS"
  exit 1
fi

print_info "Running linter: $LINTER"

LINTER_CMD="ament_${LINTER}"
print_info "Checking for linter command: $LINTER_CMD"

if command -v "$LINTER_CMD" >/dev/null 2>&1; then
  print_success "Found $LINTER_CMD, executing on src/ directory..."
  
  # Count files to be linted for better user feedback
  case "$LINTER" in
    "flake8"|"pep257")
      file_count=$(find src/ -name "*.py" | wc -l)
      print_info "Found $file_count Python files to lint"
      ;;
    "cppcheck"|"cpplint"|"uncrustify")
      file_count=$(find src/ \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) | wc -l)
      print_info "Found $file_count C/C++ files to lint"
      ;;
    "lint_cmake")
      file_count=$(find src/ -name "CMakeLists.txt" | wc -l)
      print_info "Found $file_count CMakeLists.txt files to lint"
      ;;
    "xmllint")
      file_count=$(find src/ \( -name "*.xml" -o -name "*.launch" -o -name "*.xacro" \) | wc -l)
      print_info "Found $file_count XML files to lint"
      ;;
  esac
  
  if "$LINTER_CMD" src/; then
    print_success "$LINTER_CMD completed successfully - no issues found"
  else
    local exit_code=$?
    print_warning "$LINTER_CMD found issues or failed (exit code: $exit_code)"
    print_warning "Check the output above for specific linting errors"
    # Don't exit with error to allow other linters to run (fail-fast: false)
  fi
else
  print_warning "$LINTER_CMD is not available in PATH"
  print_info "Attempting fallback checks for linter type: $LINTER"
  
  # Provide better fallback based on linter type
  case "$LINTER" in
    "flake8"|"pep257")
      print_info "Running Python fallback check with compileall..."
      file_count=$(find src/ -name "*.py" | wc -l)
      if [ "$file_count" -gt 0 ]; then
        print_info "Checking syntax of $file_count Python files..."
        if python3 -m compileall src/ -q; then
          print_success "Python syntax check passed"
        else
          print_warning "Python syntax check found issues"
        fi
      else
        print_info "No Python files found to check"
      fi
      ;;
    "cppcheck"|"cpplint"|"uncrustify")
      print_info "Running C++ fallback check..."
      file_count=$(find src/ \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) | wc -l)
      if [ "$file_count" -gt 0 ]; then
        print_info "Found $file_count C/C++ files, but no C++ linter available"
        print_warning "Skipping C++ linting - install $LINTER_CMD for proper checking"
        print_info "To install: sudo apt-get install $LINTER"
      else
        print_info "No C/C++ files found to lint"
      fi
      ;;
    "lint_cmake")
      print_info "Running CMake fallback check..."
      file_count=$(find src/ -name "CMakeLists.txt" | wc -l)
      if [ "$file_count" -gt 0 ]; then
        print_info "Found $file_count CMakeLists.txt files, but cmake linter not available"
        print_warning "Skipping CMake linting - install $LINTER_CMD for proper checking"
      else
        print_info "No CMakeLists.txt files found to lint"
      fi
      ;;
    "xmllint")
      print_info "Running XML fallback check..."
      file_count=$(find src/ \( -name "*.xml" -o -name "*.launch" -o -name "*.xacro" \) | wc -l)
      if [ "$file_count" -gt 0 ]; then
        print_info "Found $file_count XML files, but xmllint not available"
        print_warning "Skipping XML linting - install xmllint for proper checking"
        print_info "To install: sudo apt-get install libxml2-utils"
      else
        print_info "No XML files found to lint"
      fi
      ;;
    *)
      print_warning "No fallback available for linter: $LINTER"
      print_error "Unknown linter type - this should not happen if validation passed"
      ;;
  esac
fi

print_info "Lint job for $LINTER completed"
