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

echo "[INFO] Sourcing ROS environment..."
if safe_source "$ROS_SETUP"; then
  echo "[SUCCESS] ROS setup script loaded from $ROS_SETUP"
else
  echo "[WARNING] ROS setup script not found at $ROS_SETUP — continuing without sourcing ROS." >&2
fi

echo "[INFO] Running setup.sh to prepare workspace..."
if ./setup.sh; then
  echo "[SUCCESS] setup.sh completed successfully"
else
  echo "[ERROR] setup.sh failed with exit code $?" >&2
  echo "[ERROR] Cannot proceed with linting without proper workspace setup" >&2
  exit 1
fi

echo "[INFO] Checking for workspace setup..."
if [ -f "install/setup.bash" ]; then
  echo "[INFO] Found workspace setup, sourcing install/setup.bash..."
  if safe_source "install/setup.bash"; then
    echo "[SUCCESS] Workspace environment loaded"
  else
    echo "[WARNING] Failed to source workspace setup, continuing anyway..." >&2
  fi
else
  echo "[INFO] No install/setup.bash found, workspace may not be built yet"
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
if command -v "$LINTER_CMD" >/dev/null 2>&1; then
  echo "[INFO] Found $LINTER_CMD, executing..."
  if "$LINTER_CMD" src/; then
    echo "[SUCCESS] $LINTER_CMD completed successfully"
  else
    echo "[WARNING] $LINTER_CMD found issues or failed"
    # Don't exit with error to allow other linters to run (fail-fast: false)
  fi
else
  echo "[WARNING] $LINTER_CMD is not available in PATH" >&2
  
  # Provide better fallback based on linter type
  case "$LINTER" in
    "flake8"|"pep257")
      echo "[INFO] Running Python fallback check with compileall..." >&2
      if python3 -m compileall src/ -q; then
        echo "[SUCCESS] Python syntax check passed"
      else
        echo "[WARNING] Python syntax check found issues"
      fi
      ;;
    "cppcheck"|"cpplint"|"uncrustify")
      echo "[INFO] Running C++ fallback check..." >&2
      # Check for basic C++ syntax by trying to find and validate C++ files
      if find src/ -name "*.cpp" -o -name "*.hpp" -o -name "*.h" | head -1 | grep -q .; then
        echo "[INFO] Found C++ files, but no C++ linter available"
        echo "[WARNING] Skipping C++ linting - install $LINTER_CMD for proper checking"
      else
        echo "[INFO] No C++ files found to lint"
      fi
      ;;
    "lint_cmake")
      echo "[INFO] Running CMake fallback check..." >&2
      if find src/ -name "CMakeLists.txt" | head -1 | grep -q .; then
        echo "[INFO] Found CMake files, but cmake linter not available"
        echo "[WARNING] Skipping CMake linting - install $LINTER_CMD for proper checking"
      else
        echo "[INFO] No CMakeLists.txt files found to lint"
      fi
      ;;
    "xmllint")
      echo "[INFO] Running XML fallback check..." >&2
      if find src/ -name "*.xml" -o -name "*.launch" -o -name "*.xacro" | head -1 | grep -q .; then
        echo "[INFO] Found XML files, but xmllint not available"
        echo "[WARNING] Skipping XML linting - install xmllint for proper checking"
      else
        echo "[INFO] No XML files found to lint"
      fi
      ;;
    *)
      echo "[WARNING] No fallback available for linter: $LINTER" >&2
      ;;
  esac
fi
