# Design Document

## Overview

The GitHub Actions CI workflow is failing due to multiple issues in the Docker environment setup, dependency resolution, and build process. The design addresses these issues through:

1. **Improved Docker image configuration** - Ensuring all required tools and dependencies are available
2. **Robust script error handling** - Making scripts resilient to transient failures
3. **Proper ROS2 workspace structure** - Ensuring colcon only builds valid ROS2 packages
4. **Enhanced logging and debugging** - Providing clear feedback when issues occur

The solution maintains the existing workflow structure (test and lint jobs) while fixing the underlying execution issues.

## Architecture

### Current Workflow Structure

```
.github/workflows/ros.yaml
├── test job
│   └── uses: .github/actions/test/
│       ├── Docker: .devcontainer/Dockerfile
│       └── Entrypoint: run.sh → setup.sh → build.sh → test.sh
└── lint job (matrix: 7 linters)
    └── uses: .github/actions/lint/
        ├── Docker: .devcontainer/Dockerfile
        └── Entrypoint: run.sh → setup.sh → ament_${LINTER}
```

### Key Issues Identified

1. **Dockerfile Issues**:
   - Base image may be missing required packages
   - No explicit installation of pytest plugins (pytest-timeout, pytest-cov) needed by setup.cfg
   - Commented-out alternative Dockerfile configurations suggest previous attempts to fix issues
   - INSTALL_PICO_SDK argument not properly handled

2. **setup.sh Issues**:
   - apt-get update may encounter lock conflicts in CI
   - rosdep update needs sudo but may fail in container
   - VCS import may fail if vcstool not in PATH after pip install
   - No retry logic for transient network failures

3. **build.sh Issues**:
   - Build continues with `|| true` which masks real failures
   - No validation that packages were actually built
   - CMAKE_BUILD_TYPE may not be optimal for CI

4. **test.sh Issues**:
   - No check if tests actually exist before running
   - test-result output may not be captured properly
   - Missing error handling for test failures

5. **Workspace Structure Issues**:
   - Some packages in src/ may have missing dependencies
   - COLCON_IGNORE files exist but may not cover all non-ROS directories
   - Package.xml files may be missing or malformed

## Components and Interfaces

### Component 1: Dockerfile (.devcontainer/Dockerfile)

**Purpose**: Provide a clean, reproducible build environment with all dependencies

**Changes**:
- Keep the simple base image approach (althack/ros2:humble-dev)
- Explicitly install pytest plugins
- Ensure vcstool is available
- Add proper handling for apt lock conflicts
- Remove commented-out code to reduce confusion

**Interface**:
- Input: Build arguments (TARGET, INSTALL_PICO_SDK, WORKSPACE)
- Output: Docker image with ROS2 Humble + build tools + test dependencies

### Component 2: setup.sh

**Purpose**: Initialize the workspace and install dependencies

**Changes**:
- Add retry logic for apt-get update
- Wait for apt lock to be released before proceeding
- Ensure rosdep update runs with proper permissions
- Add better error messages for each step
- Make vcstool installation more robust
- Add validation that ROS environment was sourced

**Interface**:
- Input: ROS_DISTRO environment variable, src/ros2.repos file
- Output: Initialized workspace with dependencies installed
- Side effects: Updates rosdep cache, installs system packages

### Component 3: build.sh

**Purpose**: Build all ROS2 packages in the workspace

**Changes**:
- Remove `|| true` from colcon build to catch real failures
- Add validation that build produced artifacts
- Use appropriate CMAKE_BUILD_TYPE for CI (RelWithDebInfo)
- Add better error reporting
- Check that colcon is available before attempting build

**Interface**:
- Input: Sourced ROS environment, src/ directory with packages
- Output: install/ directory with built packages
- Exit code: 0 on success, non-zero on failure

### Component 4: test.sh

**Purpose**: Run automated tests on built packages

**Changes**:
- Add check for existence of tests before running
- Capture and display test results properly
- Remove `|| true` to properly report test failures
- Add timeout handling for long-running tests

**Interface**:
- Input: Built workspace (install/ directory)
- Output: Test results and coverage reports
- Exit code: 0 if all tests pass, non-zero if any fail

### Component 5: lint/run.sh

**Purpose**: Run code quality checks

**Changes**:
- Add better fallback handling when linters are missing
- Ensure setup.sh is called before linting
- Add validation that LINTER environment variable is set
- Improve error messages

**Interface**:
- Input: LINTER environment variable, src/ directory
- Output: Linting results
- Exit code: 0 on success (with fail-fast: false in workflow)

### Component 6: Package Structure Validation

**Purpose**: Ensure only valid ROS2 packages are built

**Changes**:
- Verify all packages in src/ have proper package.xml
- Add COLCON_IGNORE to any non-ROS directories
- Document which packages should be built
- Add validation script to check workspace structure

**Interface**:
- Input: src/ directory structure
- Output: List of valid packages, warnings for invalid ones

## Data Models

### Environment Variables

```yaml
ROS_DISTRO: "humble"              # ROS2 distribution
TARGET: "remote_pc"               # Build target (robot|remote_pc)
INSTALL_PICO_SDK: "false"         # Whether to install Pico SDK
WORKSPACE: "/workspace"           # Workspace path in container
BUILD_TYPE: "RelWithDebInfo"      # CMake build type
AMENT_TRACE_SETUP_FILES: ""       # Ament setup tracing
APT_UPDATED: "0"                  # Flag to prevent duplicate apt updates
```

### File Structure

```
workspace/
├── src/                          # ROS2 packages (built by colcon)
│   ├── micro-ROS-Agent/
│   ├── open_manipulator_x/
│   ├── robot_bringup/
│   ├── robot_controller/
│   ├── robot_description/
│   ├── robot_gazebo/
│   ├── robot_hardware_interfaces/
│   ├── robot_localization_tool/
│   ├── robot_utils/
│   ├── robot_vision/
│   └── ros2.repos
├── firmware/                     # Pico firmware (COLCON_IGNORE)
├── robot_firmware/               # Pico firmware (COLCON_IGNORE)
├── robot_control_gui/            # Python GUI (COLCON_IGNORE)
├── lib/                          # External libraries (COLCON_IGNORE)
├── build/                        # Build artifacts (COLCON_IGNORE)
├── install/                      # Install artifacts
├── log/                          # Build logs
├── setup.sh                      # Dependency installation
├── build.sh                      # Workspace build
└── test.sh                       # Test execution
```

## Error Handling

### Strategy

1. **Fail Fast for Critical Errors**: Build and test failures should stop the pipeline
2. **Graceful Degradation for Optional Steps**: Missing optional tools should log warnings
3. **Retry Logic for Transient Failures**: Network operations should retry
4. **Clear Error Messages**: All failures should provide actionable information

### Error Categories

**Critical Errors** (must fail the build):
- colcon build fails
- Required packages missing
- Test failures (in test.sh)
- Syntax errors in code

**Warnings** (log but continue):
- Optional linters not available
- rosdep update fails (if packages already installed)
- VCS import fails (if repositories already cloned)
- apt-get update fails (if cache is recent)

**Handled Gracefully**:
- apt lock conflicts → wait and retry
- Missing vcstool → install via pip
- Missing src/ros2.repos → skip vcs import
- No tests found → skip test execution

### Error Reporting Format

```bash
# Success
[SUCCESS] Step completed in 45s

# Warning
[WARNING] Optional step failed: rosdep update
Continuing with existing dependencies...

# Error
[ERROR] Build failed after 120s
Exit code: 2
Command: colcon build --merge-install
Error: Package 'robot_controller' has missing dependency 'controller_interface'
```

## Testing Strategy

### CI Testing Approach

1. **Smoke Test**: Verify workflow can start and Docker image builds
2. **Build Test**: Verify all packages compile successfully
3. **Unit Test**: Run package-specific unit tests
4. **Lint Test**: Run all configured linters
5. **Integration Test**: Verify installed packages can be sourced and used

### Local Testing

Developers can test CI changes locally using:

```bash
# Build the Docker image
docker build -f .devcontainer/Dockerfile -t ros2-ci:test .

# Run setup
docker run --rm -v $(pwd):/workspace -w /workspace ros2-ci:test ./setup.sh

# Run build
docker run --rm -v $(pwd):/workspace -w /workspace ros2-ci:test ./build.sh

# Run tests
docker run --rm -v $(pwd):/workspace -w /workspace ros2-ci:test ./test.sh

# Run specific linter
docker run --rm -v $(pwd):/workspace -w /workspace -e LINTER=flake8 ros2-ci:test .github/actions/lint/run.sh
```

### Validation Criteria

**Build Success Criteria**:
- All packages in src/ compile without errors
- install/ directory contains expected artifacts
- No critical warnings in build log

**Test Success Criteria**:
- All unit tests pass
- Test coverage meets minimum threshold (if configured)
- No test timeouts

**Lint Success Criteria**:
- No critical linting errors
- Code follows style guidelines
- All configured linters run successfully

## Implementation Notes

### Dockerfile Simplification

The current Dockerfile has extensive commented-out code suggesting multiple attempts to fix issues. The design keeps the simple approach:

```dockerfile
FROM althack/ros2:humble-dev 

# Install pytest plugins required by setup.cfg
RUN python3 -m pip install --no-cache-dir pytest-timeout pytest-cov

# Install vcstool to ensure it's available
RUN python3 -m pip install --no-cache-dir vcstool

# Set up auto-source of workspace for ros user
ARG TARGET=remote_pc
ARG INSTALL_PICO_SDK=false
ENV ROS_DISTRO=humble
ARG WORKSPACE=/workspace
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
```

### Script Improvements

**setup.sh**:
- Add apt lock waiting logic
- Improve rosdep error handling
- Validate ROS environment sourcing
- Add step timing

**build.sh**:
- Remove `|| true` from colcon build
- Add build artifact validation
- Improve error messages

**test.sh**:
- Check for test existence
- Properly capture test results
- Handle test timeouts

### Workflow Adjustments

The workflow file itself may need minor adjustments:

```yaml
# Ensure proper permissions
- name: Checkout code
  uses: actions/checkout@v4
  with:
    submodules: recursive  # If using git submodules

# Add caching for faster builds
- name: Cache colcon build
  uses: actions/cache@v3
  with:
    path: |
      build
      install
    key: ${{ runner.os }}-colcon-${{ hashFiles('**/package.xml') }}
```

## Dependencies

### Required System Packages

- ros-humble-desktop (from base image)
- python3-colcon-common-extensions
- python3-rosdep
- python3-vcstool
- build-essential
- cmake
- git

### Required Python Packages

- pytest
- pytest-timeout
- pytest-cov
- vcstool
- colcon-common-extensions

### Required ROS2 Packages

Dependencies defined in package.xml files, installed via rosdep:
- controller_interface
- hardware_interface
- ros2_control
- gazebo_ros_pkgs
- nav2_bringup
- robot_localization
- (and others as specified in package.xml files)

## Rollout Plan

1. **Phase 1**: Fix Dockerfile and ensure base environment is correct
2. **Phase 2**: Improve setup.sh with better error handling
3. **Phase 3**: Fix build.sh to properly report failures
4. **Phase 4**: Enhance test.sh for better test execution
5. **Phase 5**: Validate lint jobs work correctly
6. **Phase 6**: Add caching and optimization
7. **Phase 7**: Document CI process for developers

Each phase should result in a working CI pipeline, with incremental improvements.
