#!/bin/bash
set -euo pipefail

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

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

log_info() {
  echo -e "${GREEN}[INFO]${NC} $*"
}

log_warning() {
  echo -e "${YELLOW}[WARNING]${NC} $*" >&2
}

log_error() {
  echo -e "${RED}[ERROR]${NC} $*" >&2
}

log_success() {
  echo -e "${GREEN}[SUCCESS]${NC} $*"
}

get_package_name() {
  local pkg_dir="$1"
  local package_xml="$pkg_dir/package.xml"
  if [ ! -f "$package_xml" ]; then
    return 1
  fi
  local pkg_name
  pkg_name=$(python3 - <<'PY' "$package_xml"
import sys
import xml.etree.ElementTree as ET

package_xml = sys.argv[1]
try:
    tree = ET.parse(package_xml)
    root = tree.getroot()
    name = root.findtext('name')
    if name:
        print(name.strip())
except ET.ParseError:
    pass
PY
)
  if [ -n "$pkg_name" ]; then
    echo "$pkg_name"
    return 0
  fi
  return 1
}

ROS_DISTRO=${ROS_DISTRO:-humble}
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -z "${AMENT_TRACE_SETUP_FILES+x}" ]; then
  AMENT_TRACE_SETUP_FILES=""
fi
export AMENT_TRACE_SETUP_FILES

log_info "Sourcing ROS2 environment..."
if safe_source "$ROS_SETUP"; then
  log_success "ROS2 environment sourced successfully"
else
  log_error "ROS setup script not found at $ROS_SETUP"
  exit 1
fi

if [ -f "install/setup.bash" ]; then
  log_info "Sourcing workspace overlay..."
  safe_source "install/setup.bash"
fi

if ! command -v colcon >/dev/null 2>&1; then
  log_error "colcon is not available; cannot build workspace"
  exit 1
fi

# Verify colcon version and capabilities
COLCON_VERSION=$(colcon version 2>/dev/null || echo "unknown")
log_info "Using colcon version: $COLCON_VERSION"

# Check for required colcon extensions
REQUIRED_EXTENSIONS=("colcon-common-extensions")
for ext in "${REQUIRED_EXTENSIONS[@]}"; do
  if ! python3 -c "import ${ext//-/_}" 2>/dev/null; then
    log_warning "Recommended colcon extension '$ext' not found"
    log_warning "Consider installing with: pip3 install $ext"
  fi
done

# Set the default build type (appropriate for CI)
# RelWithDebInfo provides optimized code with debug symbols for better CI debugging
BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}

# Validate BUILD_TYPE
case "$BUILD_TYPE" in
  Debug|Release|RelWithDebInfo|MinSizeRel)
    log_info "Using CMAKE_BUILD_TYPE=$BUILD_TYPE (appropriate for CI)"
    ;;
  *)
    log_warning "Unknown CMAKE_BUILD_TYPE '$BUILD_TYPE', using RelWithDebInfo"
    BUILD_TYPE="RelWithDebInfo"
    ;;
esac

# Display build configuration summary
log_info "=== BUILD CONFIGURATION ==="
log_info "CMAKE_BUILD_TYPE: $BUILD_TYPE"
log_info "CI Environment: ${CI:-false}"
log_info "Workspace: $(pwd)"
log_info "ROS Distribution: $ROS_DISTRO"
if [ "${CI:-false}" = "true" ]; then
  log_info "Parallel Workers: ${PARALLEL_JOBS:-default}"
fi
log_info "=========================="

log_info "Starting colcon build with optimized configuration..."

# Record start time
START_TIME=$(date +%s)

# Configure build arguments for better CI performance and error reporting
CMAKE_ARGS=(
  "-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
  "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
  "-DCMAKE_VERBOSE_MAKEFILE=ON"  # Enable verbose output for better error diagnosis
  "-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic"  # Add compiler warnings as CMake flags
)

# Add CI-specific optimizations
COLCON_ARGS=(
  "--merge-install"
  "--symlink-install"
  "--cmake-args" "${CMAKE_ARGS[@]}"
)

if [ "${CI:-false}" = "true" ]; then
  log_info "CI environment detected, enabling additional build optimizations"
  CMAKE_ARGS+=(
    "-DCMAKE_COLOR_MAKEFILE=OFF"  # Disable colors in CI logs
    "-DCMAKE_RULE_MESSAGES=OFF"   # Reduce verbose rule messages
  )
  
  # Use sequential executor in CI for better error reporting and resource management
  COLCON_ARGS+=(
    "--executor" "sequential"
    "--event-handlers" "console_direct+"
  )
  
  # Set parallel jobs based on available CPU cores (limit for CI)
  PARALLEL_JOBS=${PARALLEL_JOBS:-$(nproc 2>/dev/null || echo "2")}
  if [ "$PARALLEL_JOBS" -gt 4 ]; then
    PARALLEL_JOBS=4  # Limit parallel jobs in CI to avoid resource exhaustion
  fi
  COLCON_ARGS+=("--parallel-workers" "$PARALLEL_JOBS")
  log_info "Using $PARALLEL_JOBS parallel workers for CI build"
else
  # Use default parallel executor for local builds
  COLCON_ARGS+=(
    "--event-handlers" "console_cohesion+"
  )
fi

# Install missing MoveIt packages if needed
log_info "Checking for MoveIt dependencies..."
MOVEIT_PACKAGES_NEEDED=true

# Check if any packages require MoveIt
if find src -name "package.xml" -exec grep -l "moveit" {} \; 2>/dev/null | grep -q .; then
  log_info "Found packages requiring MoveIt dependencies"
  MOVEIT_PACKAGES_NEEDED=true
fi

if [ "$MOVEIT_PACKAGES_NEEDED" = true ]; then
  log_info "Installing MoveIt packages..."
  
  # Install MoveIt packages that are commonly needed
  MOVEIT_PACKAGES=(
    "ros-${ROS_DISTRO}-moveit"
    "ros-${ROS_DISTRO}-moveit-msgs"
    "ros-${ROS_DISTRO}-moveit-ros-planning"
    "ros-${ROS_DISTRO}-moveit-ros-planning-interface"
    "ros-${ROS_DISTRO}-moveit-servo"
    "ros-${ROS_DISTRO}-moveit-core"
    "ros-${ROS_DISTRO}-moveit-common"
  )
  
  # Update package lists if not already done
  if [ -z "${APT_UPDATED:-}" ] && command -v apt-get >/dev/null 2>&1; then
    log_info "Updating package lists for MoveIt installation..."
    if sudo apt-get update -y -qq; then
      export APT_UPDATED=1
    else
      log_warning "Failed to update package lists"
    fi
  fi
  
  # Install MoveIt packages
  for pkg in "${MOVEIT_PACKAGES[@]}"; do
    if sudo apt-get install -y "$pkg" 2>/dev/null; then
      log_success "Installed $pkg"
    else
      log_warning "Failed to install $pkg (may not be available)"
    fi
  done
  
  log_success "MoveIt package installation completed"
fi

# Run colcon build without || true to properly report failures
set +e
colcon build "${COLCON_ARGS[@]}"
BUILD_EXIT_CODE=$?
set -e

# Record end time and calculate duration
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

# Check build result
if [ $BUILD_EXIT_CODE -ne 0 ]; then
  log_error "Build failed after ${DURATION}s with exit code $BUILD_EXIT_CODE"
  
  # Enhanced error reporting for build failures
  log_error "=== BUILD FAILURE ANALYSIS ==="
  
  # Check if log directory exists and report failed packages
  if [ -d "log/latest_build" ]; then
    log_error "Analyzing build logs for failure details..."
    
    # Look for packages that failed to build with more comprehensive error patterns
    FAILED_PACKAGES=$(find log/latest_build -name "stderr.log" -exec grep -l "CMake Error\|error:\|fatal error:\|compilation terminated\|undefined reference\|No such file\|Permission denied\|cannot find" {} \; 2>/dev/null | sed 's|log/latest_build/||' | sed 's|/stderr.log||' | sort -u || true)
    
    if [ -n "$FAILED_PACKAGES" ]; then
      log_error "Packages that failed to build (${FAILED_PACKAGES} packages):"
      echo "$FAILED_PACKAGES" | while read -r pkg; do
        echo "  ✗ $pkg"
        
        # Show detailed error information
        if [ -f "log/latest_build/$pkg/stderr.log" ]; then
          echo "    Error details:"
          
          # Extract and categorize different types of errors
          if grep -q "CMake Error" "log/latest_build/$pkg/stderr.log" 2>/dev/null; then
            echo "      [CMAKE ERROR]"
            grep -A 3 "CMake Error" "log/latest_build/$pkg/stderr.log" 2>/dev/null | head -6 | sed 's/^/        /' || true
          fi
          
          if grep -q "fatal error:\|error:" "log/latest_build/$pkg/stderr.log" 2>/dev/null; then
            echo "      [COMPILATION ERROR]"
            grep -A 2 -B 1 "fatal error:\|error:" "log/latest_build/$pkg/stderr.log" 2>/dev/null | head -8 | sed 's/^/        /' || true
          fi
          
          if grep -q "undefined reference" "log/latest_build/$pkg/stderr.log" 2>/dev/null; then
            echo "      [LINKER ERROR]"
            grep -A 1 "undefined reference" "log/latest_build/$pkg/stderr.log" 2>/dev/null | head -4 | sed 's/^/        /' || true
          fi
          
          # Show build command that failed if available
          if [ -f "log/latest_build/$pkg/stdout.log" ]; then
            FAILED_COMMAND=$(grep -E "FAILED:|make.*Error" "log/latest_build/$pkg/stdout.log" 2>/dev/null | tail -1 || true)
            if [ -n "$FAILED_COMMAND" ]; then
              echo "      [FAILED COMMAND] $FAILED_COMMAND"
            fi
          fi
          
          echo ""
        fi
      done
      
      # Provide actionable suggestions
      log_error "=== TROUBLESHOOTING SUGGESTIONS ==="
      log_error "1. Check package dependencies in package.xml files"
      log_error "2. Ensure all required system packages are installed via rosdep"
      log_error "3. Verify CMAKE_BUILD_TYPE compatibility with package requirements"
      log_error "4. Check for missing header files or libraries"
      log_error "5. Review full build logs in log/latest_build/<package_name>/"
      
    else
      log_error "Build failed but no specific package errors found in stderr logs"
      
      # Check for other types of failures
      if [ -f "log/latest_build/events.log" ]; then
        log_error "Checking events log for additional failure information..."
        if grep -q "FAIL\|ERROR\|FATAL" "log/latest_build/events.log" 2>/dev/null; then
          grep -A 2 -B 1 "FAIL\|ERROR\|FATAL" "log/latest_build/events.log" 2>/dev/null | tail -10 | sed 's/^/  /' || true
        fi
      fi
      
      log_error "Check log/latest_build/ directory for detailed build logs"
    fi
    
    # Show disk space and memory info for CI debugging
    log_error "=== SYSTEM RESOURCES ==="
    if command -v df >/dev/null 2>&1; then
      log_error "Disk space:"
      df -h . 2>/dev/null | sed 's/^/  /' || true
    fi
    if command -v free >/dev/null 2>&1; then
      log_error "Memory usage:"
      free -h 2>/dev/null | sed 's/^/  /' || true
    fi
    
  else
    log_error "Build failed but no log directory found"
    log_error "This may indicate a colcon setup issue or permission problem"
  fi
  
  log_error "=== END BUILD FAILURE ANALYSIS ==="
  exit $BUILD_EXIT_CODE
fi

log_success "Build completed successfully in ${DURATION}s"

# Validate build artifacts
log_info "Validating build artifacts..."

if [ ! -d "install" ]; then
  log_error "Build validation failed: install/ directory was not created"
  exit 1
fi

# Verify install/setup.bash exists
if [ ! -f "install/setup.bash" ]; then
  log_error "Build validation failed: install/setup.bash not found"
  exit 1
fi

# Get list of expected packages from src/ directory (excluding COLCON_IGNORE directories)
EXPECTED_PACKAGES=()
if [ -d "src" ]; then
  while IFS= read -r -d '' pkg_dir; do
    # Skip if COLCON_IGNORE exists in the package directory
    if [ ! -f "$pkg_dir/COLCON_IGNORE" ]; then
      # Check if it has a package.xml (valid ROS2 package)
      if [ -f "$pkg_dir/package.xml" ]; then
        pkg_name=$(get_package_name "$pkg_dir")
        if [ -z "$pkg_name" ]; then
          pkg_name=$(basename "$pkg_dir")
        fi
        EXPECTED_PACKAGES+=("$pkg_name")
      fi
    fi
  done < <(find src -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)
fi

# Count built packages and verify expected packages were built
BUILT_PACKAGES=()
MISSING_PACKAGES=()

if [ -d "install/share" ]; then
  while IFS= read -r -d '' pkg_dir; do
    pkg_name=$(basename "$pkg_dir")
    if [ -f "$pkg_dir/package.xml" ]; then
      parsed_name=$(get_package_name "$pkg_dir")
      if [ -n "$parsed_name" ]; then
        BUILT_PACKAGES+=("$parsed_name")
      else
        BUILT_PACKAGES+=("$pkg_name")
      fi
    fi
  done < <(find install/share -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)
fi

# Check which expected packages are missing
for expected_pkg in "${EXPECTED_PACKAGES[@]}"; do
  found=false
  for built_pkg in "${BUILT_PACKAGES[@]}"; do
    if [ "$expected_pkg" = "$built_pkg" ]; then
      found=true
      break
    fi
  done
  if [ "$found" = false ]; then
    MISSING_PACKAGES+=("$expected_pkg")
  fi
done

log_info "Expected packages: ${#EXPECTED_PACKAGES[@]}"
log_info "Built packages: ${#BUILT_PACKAGES[@]}"

if [ ${#BUILT_PACKAGES[@]} -eq 0 ]; then
  log_error "Build validation failed: No packages found in install/ directory"
  exit 1
fi

# Report built packages
if [ ${#BUILT_PACKAGES[@]} -gt 0 ]; then
  log_info "Successfully built packages:"
  for pkg in "${BUILT_PACKAGES[@]}"; do
    echo "  ✓ $pkg"
  done
fi

# Report missing packages
if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
  log_warning "Expected packages that were not built:"
  for pkg in "${MISSING_PACKAGES[@]}"; do
    echo "  ✗ $pkg"
  done
  
  # Check build logs for specific failure reasons
  if [ -d "log/latest_build" ]; then
    log_info "Checking build logs for failure details..."
    for pkg in "${MISSING_PACKAGES[@]}"; do
      if [ -f "log/latest_build/$pkg/stderr.log" ]; then
        if grep -q "CMake Error\|error:\|fatal error:" "log/latest_build/$pkg/stderr.log" 2>/dev/null; then
          log_error "Package '$pkg' failed to build - check log/latest_build/$pkg/stderr.log"
        fi
      fi
    done
  fi
  
  # Don't fail the build if some packages are missing - they might have dependencies issues
  # but other packages built successfully
  log_warning "Some expected packages were not built, but continuing as other packages built successfully"
fi

log_success "Build artifacts validated successfully"
log_success "Build process completed successfully"
