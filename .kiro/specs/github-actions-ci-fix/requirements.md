# Requirements Document

## Introduction

The GitHub Actions CI workflow (`.github/workflows/ros.yaml`) is currently failing on every run. The workflow consists of two main jobs: `test` (which runs setup, build, and test scripts) and `lint` (which runs various ament linters). The goal is to identify and fix all issues preventing successful CI runs while maintaining code quality standards.

## Glossary

- **CI/CD**: Continuous Integration/Continuous Deployment - automated testing and deployment pipeline
- **GitHub Actions**: GitHub's built-in CI/CD platform
- **ROS2 Humble**: Robot Operating System 2, Humble Hawksbill distribution
- **colcon**: Build tool for ROS2 workspaces
- **rosdep**: ROS dependency management tool
- **ament**: ROS2 build system and testing framework
- **Docker Container**: Isolated environment for running CI jobs
- **Linter**: Static code analysis tool for checking code quality
- **BuildKit**: Docker's improved build system with caching capabilities

## Requirements

### Requirement 1

**User Story:** As a developer, I want the CI pipeline to successfully build the ROS2 workspace, so that I can verify my code changes don't break the build.

#### Acceptance Criteria

1. WHEN the test job runs, THE CI System SHALL successfully source the ROS2 Humble environment
2. WHEN setup.sh executes, THE CI System SHALL install all required dependencies via rosdep without blocking errors
3. WHEN build.sh executes, THE CI System SHALL compile all non-ignored ROS2 packages using colcon
4. WHEN the build completes, THE CI System SHALL produce an install/ directory with compiled artifacts
5. IF any build step fails, THEN THE CI System SHALL report the specific error and exit with non-zero status

### Requirement 2

**User Story:** As a developer, I want the CI pipeline to run automated tests, so that I can catch regressions before merging code.

#### Acceptance Criteria

1. WHEN test.sh executes, THE CI System SHALL run colcon test on all packages with tests
2. WHEN tests complete, THE CI System SHALL display test results using colcon test-result
3. IF any test fails, THEN THE CI System SHALL report the failure details
4. WHILE tests are running, THE CI System SHALL have access to the built workspace from the build step
5. THE CI System SHALL skip test execution if colcon is not available

### Requirement 3

**User Story:** As a developer, I want the CI pipeline to enforce code quality standards, so that the codebase remains maintainable.

#### Acceptance Criteria

1. WHEN the lint job runs, THE CI System SHALL execute each configured linter (cppcheck, cpplint, uncrustify, lint_cmake, xmllint, flake8, pep257)
2. WHEN a linter is not available, THE CI System SHALL run a fallback check or skip gracefully
3. THE CI System SHALL scan the src/ directory for code quality issues
4. IF critical linting errors are found, THEN THE CI System SHALL report them clearly
5. THE CI System SHALL allow the lint job to complete even if individual linters fail (fail-fast: false)

### Requirement 4

**User Story:** As a developer, I want the Docker build to be fast and reliable, so that CI runs don't waste time or fail due to infrastructure issues.

#### Acceptance Criteria

1. THE CI System SHALL use a stable base image (althack/ros2:humble-dev)
2. WHEN building the Docker image, THE CI System SHALL install all required build tools and dependencies
3. THE CI System SHALL handle apt lock conflicts gracefully during package installation
4. THE CI System SHALL cache Docker layers to speed up subsequent builds
5. WHERE the Pico SDK is needed, THE CI System SHALL optionally install it based on build arguments

### Requirement 5

**User Story:** As a developer, I want clear error messages when CI fails, so that I can quickly identify and fix issues.

#### Acceptance Criteria

1. WHEN a script fails, THE CI System SHALL output colored, formatted error messages indicating which step failed
2. THE CI System SHALL display the duration of each build step
3. THE CI System SHALL preserve and display the exit code of failed commands
4. WHEN rosdep or apt operations fail, THE CI System SHALL log the specific package or dependency causing the issue
5. THE CI System SHALL configure git safe.directory to avoid permission errors in the container

### Requirement 6

**User Story:** As a developer, I want the CI to handle missing or optional dependencies gracefully, so that the pipeline doesn't fail unnecessarily.

#### Acceptance Criteria

1. WHEN vcstool is not installed, THE CI System SHALL install it via pip before importing repositories
2. WHEN src/ros2.repos is missing, THE CI System SHALL skip the vcs import step
3. WHEN rosdep is unavailable, THE CI System SHALL log a warning and continue
4. WHEN apt-get update fails, THE CI System SHALL log a warning but not halt the pipeline
5. THE CI System SHALL use the `|| true` pattern for non-critical operations that may fail

### Requirement 7

**User Story:** As a developer, I want the CI to work with the project's specific structure, so that firmware and GUI packages are properly excluded from ROS builds.

#### Acceptance Criteria

1. THE CI System SHALL respect COLCON_IGNORE files in firmware/, robot_firmware/, robot_control_gui/, and lib/ directories
2. WHEN colcon build runs, THE CI System SHALL only build packages in src/ that are valid ROS2 packages
3. THE CI System SHALL not attempt to build Python GUI applications as ROS packages
4. THE CI System SHALL not attempt to build Pico firmware during the ROS build process
5. WHERE packages have missing dependencies, THE CI System SHALL report them clearly without failing the entire build
