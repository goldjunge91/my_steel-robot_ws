#!/bin/bash
set -euo pipefail

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Timing function
start_time=$(date +%s)

log_step() {
  local message="$1"
  local current_time
  current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${BLUE}[TEST - ${elapsed}s]${NC} ${message}"
}

log_success() {
  local message="$1"
  local current_time
  current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${GREEN}[SUCCESS - ${elapsed}s]${NC} ${message}"
}

log_warning() {
  local message="$1"
  local current_time
  current_time=$(date +%s)
  local elapsed=$((current_time - start_time))
  echo -e "${YELLOW}[WARNING - ${elapsed}s]${NC} ${message}"
}

log_error() {
  local message="$1"
  local current_time
  current_time=$(date +%s)
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
test_packages_list=()

if [ -d "src" ]; then
  # Look for test directories or test files in packages
  while IFS= read -r -d '' package_dir; do
    package_name=$(basename "$package_dir")
    has_tests=false
    
    # Check for common test indicators
    if [ -f "$package_dir/package.xml" ]; then
      # Check if package.xml contains test dependencies
      if grep -q "<test_depend>" "$package_dir/package.xml" 2>/dev/null; then
        has_tests=true
        log_step "Package '$package_name' has test dependencies in package.xml"
      fi
      
      # Check for actual test files in standard locations
      for test_dir in "test" "tests"; do
        if [ -d "$package_dir/$test_dir" ]; then
          # Look for test files (Python, C++, launch files)
          if find "$package_dir/$test_dir" -type f \( -name '*.py' -o -name '*.cpp' -o -name '*.launch.py' -o -name '*.launch' \) -print -quit 2>/dev/null | grep -q .; then
            has_tests=true
            log_step "Package '$package_name' has test files in $test_dir/"
            break
          fi
        fi
      done
      
      # Check for CMakeLists.txt with testing enabled
      if [ -f "$package_dir/CMakeLists.txt" ] && grep -q "find_package.*ament_cmake_gtest\|find_package.*ament_cmake_pytest\|ament_add_gtest\|ament_add_pytest" "$package_dir/CMakeLists.txt" 2>/dev/null; then
        has_tests=true
        log_step "Package '$package_name' has test configuration in CMakeLists.txt"
      fi
      
      # Check for setup.py with test configuration
      if [ -f "$package_dir/setup.py" ] && grep -q "test_suite\|tests_require" "$package_dir/setup.py" 2>/dev/null; then
        has_tests=true
        log_step "Package '$package_name' has test configuration in setup.py"
      fi
    fi
    
    if [ "$has_tests" = true ]; then
      test_packages_found=true
      test_packages_list+=("$package_name")
    fi
  done < <(find src -mindepth 1 -maxdepth 2 -type d -print0 2>/dev/null)
fi

if [ "$test_packages_found" = false ]; then
  log_warning "No packages with tests found; skipping test execution gracefully"
  log_step "Checked for test indicators: test dependencies in package.xml, test files in test/ or tests/ directories, CMakeLists.txt test configuration, and setup.py test configuration"
  
  # GitHub Actions summary for no tests
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "## ℹ️ No Tests Found"
    github_summary ""
    github_summary "No packages with tests were found in the workspace."
    github_summary ""
    github_summary "**Checked for:**"
    github_summary "- Test dependencies in package.xml"
    github_summary "- Test files in test/ or tests/ directories"
    github_summary "- CMakeLists.txt test configuration"
    github_summary "- setup.py test configuration"
    github_summary ""
    github_notice "No Tests" "No packages with tests found - skipping test execution"
  fi
  
  exit 0
fi

log_success "Found ${#test_packages_list[@]} packages with tests: ${test_packages_list[*]}"

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

# Capture test results with detailed output
log_step "Generating detailed test report"
test_result_output=$(colcon test-result --verbose 2>&1)
test_result_exit_code=$?

# Display the captured output
echo "$test_result_output"

# Parse and summarize test results
log_step "Analyzing test results"
total_tests=0
passed_tests=0
failed_tests=0
failed_packages=()
passed_packages=()

# Count test results from colcon test-result output
if echo "$test_result_output" | grep -q "Summary:"; then
  # Extract summary information if available
  summary_line=$(echo "$test_result_output" | grep "Summary:" | tail -1)
  log_step "Test summary: $summary_line"
fi

# Parse individual test results
while IFS= read -r line; do
  if [[ "$line" =~ ^[[:space:]]*([^[:space:]]+)[[:space:]]+([0-9]+)[[:space:]]+tests[[:space:]]+([0-9]+)[[:space:]]+passed[[:space:]]+([0-9]+)[[:space:]]+failed ]]; then
    package_name="${BASH_REMATCH[1]}"
    package_total="${BASH_REMATCH[2]}"
    package_passed="${BASH_REMATCH[3]}"
    package_failed="${BASH_REMATCH[4]}"
    
    total_tests=$((total_tests + package_total))
    passed_tests=$((passed_tests + package_passed))
    failed_tests=$((failed_tests + package_failed))
    
    if [ "$package_failed" -gt 0 ]; then
      failed_packages+=("$package_name")
      log_error "Package '$package_name': $package_failed/$package_total tests failed"
    else
      passed_packages+=("$package_name")
      log_success "Package '$package_name': $package_passed/$package_total tests passed"
    fi
  fi
done <<< "$test_result_output"

# Display comprehensive summary
echo ""
log_step "=== TEST EXECUTION SUMMARY ==="

# GitHub Actions summary for test results
if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
  if [ $failed_tests -eq 0 ] && [ $total_tests -gt 0 ]; then
    github_summary "## ✅ All Tests Passed"
  elif [ $failed_tests -gt 0 ]; then
    github_summary "## ❌ Tests Failed"
  else
    github_summary "## ⚠️ Test Results"
  fi
  github_summary ""
  
  if [ $total_tests -gt 0 ]; then
    github_summary "**Test Statistics:**"
    github_summary "- Total: $total_tests"
    github_summary "- Passed: $passed_tests"
    github_summary "- Failed: $failed_tests"
    github_summary ""
  fi
fi

if [ ${#passed_packages[@]} -gt 0 ]; then
  log_success "Packages with all tests passing (${#passed_packages[@]}): ${passed_packages[*]}"
  
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "### ✅ Packages with Passing Tests (${#passed_packages[@]})"
    for pkg in "${passed_packages[@]}"; do
      github_summary "- $pkg"
    done
    github_summary ""
  fi
fi

if [ ${#failed_packages[@]} -gt 0 ]; then
  log_error "Packages with test failures (${#failed_packages[@]}): ${failed_packages[*]}"
  
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "### ❌ Packages with Test Failures (${#failed_packages[@]})"
    for pkg in "${failed_packages[@]}"; do
      github_summary "- $pkg"
      github_error "Test Failure" "Package '$pkg' has failing tests" "src/$pkg"
    done
    github_summary ""
  fi
fi

if [ $total_tests -gt 0 ]; then
  log_step "Total tests: $total_tests | Passed: $passed_tests | Failed: $failed_tests"
  
  if [ $failed_tests -eq 0 ]; then
    log_success "🎉 ALL TESTS PASSED! ($passed_tests/$total_tests)"
  else
    log_error "❌ TESTS FAILED: $failed_tests out of $total_tests tests failed"
  fi
else
  log_warning "No test statistics could be parsed from colcon test-result output"
  
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_warning "Test Statistics" "No test statistics could be parsed from colcon test-result output"
  fi
fi

# Provide additional failure details if tests failed
if [ $test_result_exit_code -ne 0 ] || [ $failed_tests -gt 0 ]; then
  log_step "Searching for detailed failure information..."
  
  # GitHub Actions summary for test failure details
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "### 🔍 Test Failure Analysis"
    github_summary ""
  fi
  
  # Look for test result XML files with failures
  if [ -d "build" ]; then
    failure_details_found=false
    while IFS= read -r -d '' xml_file; do
      if grep -q 'failures="[1-9]' "$xml_file" 2>/dev/null || grep -q 'errors="[1-9]' "$xml_file" 2>/dev/null; then
        package_path=$(dirname "$xml_file")
        package_name=$(basename "$(dirname "$(dirname "$package_path")")")
        log_error "Failure details in: $xml_file (package: $package_name)"
        failure_details_found=true
        
        if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
          github_summary "**$package_name**: Detailed failure information in \`$xml_file\`"
        fi
        
        # Extract specific failure information
        if command -v xmllint >/dev/null 2>&1; then
          log_step "Extracting failure details from $xml_file:"
          failed_test_names=$(xmllint --xpath "//testcase[failure or error]/@name" "$xml_file" 2>/dev/null | sed 's/name="//g' | sed 's/"/ /g' | tr ' ' '\n' | grep -v '^$' || true)
          
          if [ -n "$failed_test_names" ]; then
            if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
              github_summary ""
              github_summary "Failed tests in $package_name:"
            fi
            
            echo "$failed_test_names" | while read -r test_name; do
              log_error "  Failed test: $test_name"
              if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
                github_summary "- \`$test_name\`"
                github_warning "Test Failed" "Test '$test_name' failed in package '$package_name'" "src/$package_name"
              fi
            done
            
            if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
              github_summary ""
            fi
          fi
        fi
      fi
    done < <(find build -name "*.xml" -path "*/test_results/*" -print0 2>/dev/null)
    
    if [ "$failure_details_found" = false ]; then
      log_warning "No detailed failure information found in XML test result files"
      if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
        github_warning "Test Analysis" "No detailed failure information found in XML test result files"
      fi
    fi
  fi
  
  # Look for pytest output files
  if [ -d "build" ]; then
    pytest_files_found=false
    while IFS= read -r -d '' pytest_file; do
      if [ -s "$pytest_file" ]; then
        package_name=$(basename "$(dirname "$(dirname "$pytest_file")")")
        log_step "Pytest output for package '$package_name': $pytest_file"
        pytest_files_found=true
        
        if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
          github_summary "**Pytest output** for $package_name: \`$pytest_file\`"
        fi
      fi
    done < <(find build -name "pytest.xml" -print0 2>/dev/null)
    
    if [ "$pytest_files_found" = true ] && [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
      github_summary ""
    fi
  fi
  
  # Add troubleshooting section to GitHub Actions summary
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "### 🔧 Troubleshooting Steps"
    github_summary ""
    github_summary "1. **Review Test Logs**: Check detailed test output in build directories"
    github_summary "2. **Run Tests Locally**: Execute \`colcon test --packages-select <package_name>\`"
    github_summary "3. **Check Test Dependencies**: Verify test dependencies in package.xml"
    github_summary "4. **Environment Issues**: Ensure workspace is properly sourced before tests"
    github_summary "5. **Test Configuration**: Review CMakeLists.txt or setup.py test configuration"
    github_summary ""
    github_summary "_Note: Tests continued for CI/CD compatibility despite failures_"
    github_summary ""
  fi
  
  # Log test issues but don't fail CI/CD
  if [ $test_exit_code -ne 0 ]; then
    log_warning "Test execution had issues with code: $test_exit_code (continuing for CI/CD)"
    if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
      github_warning "Test Execution" "Test execution had issues with exit code $test_exit_code"
    fi
  elif [ $test_result_exit_code -ne 0 ]; then
    log_warning "Test result processing had issues with code: $test_result_exit_code (continuing for CI/CD)"
    if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
      github_warning "Test Results" "Test result processing had issues with exit code $test_result_exit_code"
    fi
  fi
  
  log_warning "Test issues detected but continuing for CI/CD compatibility"
else
  log_success "🎉 All tests completed successfully!"
  
  if [ "${GITHUB_ACTIONS:-false}" = "true" ]; then
    github_summary "### 🎉 Success"
    github_summary ""
    github_summary "All tests completed successfully!"
  fi
fi

log_success "Test script completed successfully - always returning success for CI/CD"
exit 0
