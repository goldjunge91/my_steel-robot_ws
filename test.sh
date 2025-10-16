#!/bin/bash
set -euo pipefail

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Timing function
start_time=$(date +%s)

log_step() {
  local message="$1"
  local current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${BLUE}[TEST - ${elapsed}s]${NC} ${message}"
}

log_success() {
  local message="$1"
  local current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${GREEN}[SUCCESS - ${elapsed}s]${NC} ${message}"
}

log_warning() {
  local message="$1"
  local current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${YELLOW}[WARNING - ${elapsed}s]${NC} ${message}"
}

log_error() {
  local message="$1"
  local current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${RED}[ERROR - ${elapsed}s]${NC} ${message}"
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

# Set up AMENT_TRACE_SETUP_FILES
if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  AMENT_TRACE_SETUP_FILES=""
fi
export AMENT_TRACE_SETUP_FILES

# Ensure workspace is sourced before running tests (Requirement 2.4)
log_step "Sourcing workspace environment"
if [ -f install/setup.bash ]; then
  if safe_source install/setup.bash; then
    log_success "Workspace environment sourced successfully"
  else
    log_error "Failed to source workspace environment"
    exit 1
  fi
else
  log_error "install/setup.bash not found - workspace may not be built"
  exit 1
fi

# Check if colcon is available (Requirement 2.5)
if ! command -v colcon >/dev/null 2>&1; then
  log_warning "colcon is not available; skipping tests"
  exit 0
fi

# Check if any packages have tests (Requirement 2.1, 6.5)
log_step "Checking for test existence"
test_packages_found=false

if [ -d "src" ]; then
  # Look for test directories or test files in packages
  while IFS= read -r -d '' package_dir; do
    # Check for common test indicators
    if [ -d "$package_dir/test" ] || \
       [ -d "$package_dir/tests" ] || \
       [ -f "$package_dir/package.xml" ]; then
      # Check if package.xml contains test dependencies
      if [ -f "$package_dir/package.xml" ] && \
         grep -q "<test_depend>" "$package_dir/package.xml" 2>/dev/null; then
        test_packages_found=true
        break
      fi
      # Check for actual test files
      if [ -d "$package_dir/test" ] && [ -n "$(find "$package_dir/test" -type f -name '*.py' -o -name '*.cpp' 2>/dev/null)" ]; then
        test_packages_found=true
        break
      fi
      if [ -d "$package_dir/tests" ] && [ -n "$(find "$package_dir/tests" -type f -name '*.py' -o -name '*.cpp' 2>/dev/null)" ]; then
        test_packages_found=true
        break
      fi
    fi
  done < <(find src -mindepth 1 -maxdepth 2 -type d -print0 2>/dev/null)
fi

if [ "$test_packages_found" = false ]; then
  log_warning "No packages with tests found; skipping test execution"
  exit 0
fi

log_success "Found packages with tests"

# Run tests (Requirement 2.1)
log_step "Running colcon test"
test_exit_code=0
if colcon test --merge-install; then
  log_success "Test execution completed"
else
  test_exit_code=$?
  log_error "Test execution failed with exit code ${test_exit_code}"
fi

# Display test results (Requirement 2.2, 2.3, 5.1)
log_step "Displaying test results"
if colcon test-result --verbose; then
  log_success "All tests passed"
else
  result_exit_code=$?
  log_error "Some tests failed (exit code: ${result_exit_code})"
  
  # Try to provide more detailed failure information
  if [ -d "build" ]; then
    log_step "Searching for failed test details..."
    find build -name "*.xml" -path "*/test_results/*" -exec echo "Test result file: {}" \; 2>/dev/null || true
  fi
  
  # Propagate test failure (Requirement 2.3)
  if [ $test_exit_code -ne 0 ]; then
    exit $test_exit_code
  else
    exit $result_exit_code
  fi
fi

log_success "Test script completed successfully"
exit 0
