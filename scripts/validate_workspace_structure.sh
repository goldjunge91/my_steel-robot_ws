#!/usr/bin/env bash
# Workspace Structure Validation Script
# This script validates the workspace structure and checks for common issues
# before attempting to build the ROS2 workspace.

set -e

# Increase file descriptor limit to prevent "too many open files" errors
ulimit -n 4096 2>/dev/null || echo "Warning: Could not increase file descriptor limit" >&2

# --- Color definitions for output ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# --- Helper functions ---

print_header() {
  echo -e "\n${BLUE}=======================================================================${NC}"
  echo -e "${BLUE}===== $1${NC}"
  echo -e "${BLUE}=======================================================================${NC}"
}

print_success() {
  echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
  echo -e "${RED}✗${NC} $1"
}

# --- Validation functions ---

validate_colcon_ignore_files() {
  print_header "Validating COLCON_IGNORE files"
  
  local errors=0
  local directories=("firmware" "robot_firmware" "robot_control_gui" "lib")
  
  for dir in "${directories[@]}"; do
    if [ -d "$dir" ]; then
      if [ -f "$dir/COLCON_IGNORE" ]; then
        print_success "COLCON_IGNORE exists in $dir/"
      else
        print_error "Missing COLCON_IGNORE in $dir/ - this directory will be processed by colcon"
        errors=$((errors + 1))
      fi
    else
      print_warning "Directory $dir/ does not exist"
    fi
  done
  
  return $errors
}

validate_package_xml_files() {
  print_header "Validating package.xml files in src/"
  
  local errors=0
  local packages_found=0
  
  if [ ! -d "src" ]; then
    print_error "src/ directory does not exist"
    return 1
  fi
  
  # Find all directories in src/ that don't have COLCON_IGNORE
  # Use a more efficient approach to avoid opening too many file descriptors
  while IFS= read -r -d '' package_dir; do
    if [ -d "$package_dir" ]; then
      local package_name=$(basename "$package_dir")
      
      # Skip if COLCON_IGNORE exists
      if [ -f "$package_dir/COLCON_IGNORE" ]; then
        print_warning "Package $package_name has COLCON_IGNORE - will be skipped by colcon"
        continue
      fi
      
      packages_found=$((packages_found + 1))
      
      # Check for package.xml
      if [ -f "$package_dir/package.xml" ]; then
        # Validate package.xml is well-formed XML (skip xmllint to avoid file descriptor issues)
        if [ -r "$package_dir/package.xml" ] && grep -q "<package" "$package_dir/package.xml" 2>/dev/null; then
          print_success "Valid package.xml in $package_name"
        else
          print_error "Malformed package.xml in $package_name"
          errors=$((errors + 1))
        fi
      else
        print_error "Missing package.xml in $package_name"
        errors=$((errors + 1))
      fi
      
      # Check for CMakeLists.txt or setup.py (depending on package type)
      if [ -f "$package_dir/CMakeLists.txt" ] || [ -f "$package_dir/setup.py" ]; then
        print_success "Build file found in $package_name"
      else
        print_error "Missing CMakeLists.txt or setup.py in $package_name"
        errors=$((errors + 1))
      fi
    fi
  done < <(find src/ -maxdepth 1 -type d -print0 2>/dev/null)
  
  if [ $packages_found -eq 0 ]; then
    print_error "No ROS2 packages found in src/"
    return 1
  fi
  
  print_success "Found $packages_found ROS2 packages in src/"
  return $errors
}

validate_ros_environment() {
  print_header "Validating ROS environment"
  
  local errors=0
  
  # Check if ROS_DISTRO is set
  if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS_DISTRO environment variable is not set"
    errors=$((errors + 1))
  else
    print_success "ROS_DISTRO is set to: $ROS_DISTRO"
  fi
  
  # Check if ROS setup script exists
  local ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [ -f "$ros_setup" ]; then
    print_success "ROS setup script found: $ros_setup"
  else
    print_error "ROS setup script not found: $ros_setup"
    errors=$((errors + 1))
  fi
  
  # Check if colcon is available
  if command -v colcon >/dev/null 2>&1; then
    print_success "colcon build tool is available"
  else
    print_error "colcon build tool is not available"
    errors=$((errors + 1))
  fi
  
  return $errors
}

validate_build_tools() {
  print_header "Validating build tools"
  
  local errors=0
  local tools=("cmake" "make" "gcc" "g++" "python3")
  
  for tool in "${tools[@]}"; do
    if command -v "$tool" >/dev/null 2>&1; then
      print_success "$tool is available"
    else
      print_error "$tool is not available"
      errors=$((errors + 1))
    fi
  done
  
  return $errors
}

validate_workspace_structure() {
  print_header "Validating workspace structure"
  
  local errors=0
  local expected_dirs=("src")
  local optional_dirs=("build" "install" "log")
  
  # Check required directories
  for dir in "${expected_dirs[@]}"; do
    if [ -d "$dir" ]; then
      print_success "Required directory exists: $dir/"
    else
      print_error "Required directory missing: $dir/"
      errors=$((errors + 1))
    fi
  done
  
  # Check optional directories (info only)
  for dir in "${optional_dirs[@]}"; do
    if [ -d "$dir" ]; then
      print_success "Build directory exists: $dir/"
    else
      print_warning "Build directory will be created: $dir/"
    fi
  done
  
  return $errors
}

check_known_dependency_issues() {
  print_header "Checking for known dependency issues"
  
  local warnings=0
  
  # Check if rosdep is initialized
  if [ -d "/etc/ros/rosdep" ] || [ -d "$HOME/.ros/rosdep" ]; then
    print_success "rosdep appears to be initialized"
  else
    print_warning "rosdep may not be initialized - run 'rosdep update' if build fails"
    warnings=$((warnings + 1))
  fi
  
  # Check for common missing system dependencies (more efficiently)
  local sys_deps=("libeigen3-dev" "libboost-dev")
  for dep in "${sys_deps[@]}"; do
    if dpkg-query -W "$dep" >/dev/null 2>&1; then
      print_success "System dependency available: $dep"
    else
      print_warning "System dependency may be missing: $dep"
      warnings=$((warnings + 1))
    fi
  done
  
  return 0  # Warnings don't cause failure
}

# --- Main validation function ---

main() {
  echo -e "${YELLOW}Starting workspace structure validation...${NC}"
  
  local total_errors=0
  
  # Run all validation checks
  validate_workspace_structure || total_errors=$((total_errors + $?))
  validate_colcon_ignore_files || total_errors=$((total_errors + $?))
  validate_package_xml_files || total_errors=$((total_errors + $?))
  validate_ros_environment || total_errors=$((total_errors + $?))
  validate_build_tools || total_errors=$((total_errors + $?))
  check_known_dependency_issues  # This only produces warnings
  
  # Summary
  print_header "Validation Summary"
  
  if [ $total_errors -eq 0 ]; then
    echo -e "${GREEN}✓ Workspace validation passed successfully!${NC}"
    echo -e "${GREEN}  The workspace appears ready for building.${NC}"
    exit 0
  else
    echo -e "${RED}✗ Workspace validation failed with $total_errors error(s).${NC}"
    echo -e "${RED}  Please fix the issues above before attempting to build.${NC}"
    exit 1
  fi
}

# Run the main function
main "$@"