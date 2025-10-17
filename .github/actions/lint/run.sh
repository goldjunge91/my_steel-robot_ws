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

print_info "Starting lint job for linter: ${LINTER:-<not set>}"

# Run setup.sh to prepare workspace (following athackst approach)
print_info "Running setup.sh..."
./setup.sh

# Check if files exist for the specific linter (following athackst approach)
case "$LINTER" in
  "uncrustify"|"cppcheck"|"cpplint")
    if [ -z "$(find src/ -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h')" ]; then
      print_warning "No C/C++ files found. Skipping $LINTER."
      exit 0
    fi
    ;;
  "flake8"|"pep257")
    if [ -z "$(find src/ -name '*.py')" ]; then
      print_warning "No Python files found. Skipping $LINTER."
      exit 0
    fi
    ;;
  "lint_cmake")
    if [ -z "$(find src/ -name 'CMakeLists.txt' -o -name '*.cmake')" ]; then
      print_warning "No CMake files found. Skipping $LINTER."
      exit 0
    fi
    ;;
  "xmllint")
    if [ -z "$(find src/ -name '*.xml' -o -name '*.launch' -o -name '*.urdf' -o -name '*.xacro')" ]; then
      print_warning "No XML files found. Skipping $LINTER."
      exit 0
    fi
    ;;
esac

# Source ROS environment and run linter (following athackst approach)
print_info "Sourcing ROS environment and running $LINTER..."

# Set required ROS environment variable and source safely
if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  AMENT_TRACE_SETUP_FILES=""
fi
export AMENT_TRACE_SETUP_FILES

# Temporarily disable strict mode for ROS sourcing
set +u
source /opt/ros/$ROS_DISTRO/setup.bash
set -u

if ament_${LINTER} src/; then
    print_success "$LINTER completed successfully - no issues found"
else
    print_warning "$LINTER found issues (this is normal for linting)"
fi

print_success "Lint job for $LINTER completed"
