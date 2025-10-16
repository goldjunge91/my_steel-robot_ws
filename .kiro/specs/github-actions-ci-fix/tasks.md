# Implementation Plan

- [x] 1. Simplify and fix Dockerfile
  - Remove all commented-out code to reduce confusion
  - Ensure pytest plugins (pytest-timeout, pytest-cov) are installed
  - Add vcstool installation to ensure it's available in PATH
  - Verify base image has all required ROS2 packages
  - _Requirements: 4.1, 4.2, 4.5_

- [ ] 2. Improve setup.sh error handling and robustness
  - [x] 2.1 Add apt lock waiting logic
    - Implement loop to wait for /var/lib/dpkg/lock-frontend to be released
    - Add timeout to prevent infinite waiting
    - _Requirements: 5.4, 6.4_
  
  - [x] 2.2 Enhance rosdep operations
    - Ensure rosdep update runs with proper permissions
    - Add retry logic for rosdep update failures
    - Improve error messages when dependencies are missing
    - _Requirements: 1.2, 6.3_
  
  - [x] 2.3 Make vcstool installation more robust
    - Verify vcstool is in PATH after pip install
    - Add explicit PATH update if needed
    - Handle case where src/ros2.repos doesn't exist
    - _Requirements: 6.1, 6.2_
  
  - [x] 2.4 Add validation and better logging
    - Verify ROS environment was sourced successfully
    - Add timing information for each step
    - Improve colored output for better readability
    - _Requirements: 5.1, 5.2, 1.1_

- [x] 3. Fix build.sh to properly report failures
  - [x] 3.1 Remove || true from colcon build command
    - Allow build failures to propagate properly
    - Ensure exit code reflects build status
    - _Requirements: 1.5, 3.1_
  
  - [x] 3.2 Add build artifact validation
    - Check that install/ directory was created
    - Verify expected packages were built
    - Report which packages failed to build
    - _Requirements: 1.4, 7.2_
  
  - [x] 3.3 Improve build configuration
    - Ensure CMAKE_BUILD_TYPE is appropriate for CI
    - Add better error reporting for build failures
    - _Requirements: 1.3, 5.2_

- [x] 4. Enhance test.sh for better test execution
  - [x] 4.1 Add test existence check
    - Verify packages have tests before running colcon test
    - Skip test execution gracefully if no tests found
    - _Requirements: 2.1, 6.5_
  
  - [x] 4.2 Improve test result reporting
    - Ensure test-result output is captured and displayed
    - Add clear indication of which tests passed/failed
    - _Requirements: 2.2, 2.3, 5.1_
  
  - [x] 4.3 Add proper error handling
    - Remove || true to allow test failures to propagate
    - Ensure workspace is sourced before running tests
    - _Requirements: 2.4, 2.5_

- [x] 5. Fix lint job execution
  - [x] 5.1 Improve linter availability checking
    - Add better fallback when specific linter is not available
    - Ensure LINTER environment variable is validated
    - _Requirements: 3.2, 5.4_
  
  - [x] 5.2 Ensure setup runs before linting
    - Verify setup.sh completes successfully
    - Source workspace if it exists
    - _Requirements: 3.3, 1.1_
  
  - [x] 5.3 Improve error messages
    - Add clear indication of which linter is running
    - Report linter-specific errors clearly
    - _Requirements: 3.4, 5.1_

- [-] 6. Validate and fix workspace structure
  - [x] 6.1 Verify COLCON_IGNORE files are in place
    - Check firmware/, robot_firmware/, robot_control_gui/, lib/ have COLCON_IGNORE
    - Add any missing COLCON_IGNORE files
    - _Requirements: 7.1, 7.3_
  
  - [-] 6.2 Validate package.xml files
    - Check all packages in src/ have valid package.xml
    - Identify packages with missing dependencies
    - _Requirements: 7.2, 7.5_
  
  - [ ] 6.3 Document expected package structure
    - List which packages should be built by CI
    - Document any known dependency issues
    - _Requirements: 7.4_

- [ ] 7. Add workflow improvements
  - [ ] 7.1 Add git submodule checkout if needed
    - Check if project uses git submodules
    - Add submodules: recursive to checkout action if needed
    - _Requirements: 5.5_
  
  - [ ] 7.2 Add build caching for faster CI runs
    - Implement GitHub Actions cache for build/ and install/
    - Use package.xml files as cache key
    - _Requirements: 4.4_
  
  - [ ] 7.3 Add validation step before build
    - Create script to validate workspace structure
    - Check for common issues before attempting build
    - _Requirements: 7.5_

- [ ] 8. Test and validate CI fixes
  - [ ] 8.1 Test Dockerfile builds successfully
    - Build Docker image locally
    - Verify all required tools are available
    - _Requirements: 4.1, 4.2_
  
  - [ ] 8.2 Test scripts run successfully in container
    - Run setup.sh in Docker container
    - Run build.sh and verify packages build
    - Run test.sh and verify tests execute
    - _Requirements: 1.1, 1.2, 1.3, 2.1_
  
  - [ ] 8.3 Test lint jobs work correctly
    - Run each linter individually
    - Verify linters report issues correctly
    - _Requirements: 3.1, 3.2_
  
  - [ ] 8.4 Verify full workflow runs successfully
    - Push changes and trigger GitHub Actions
    - Monitor workflow execution
    - Verify all jobs complete successfully
    - _Requirements: 1.5, 2.3, 3.4_

- [ ] 9. Document CI process for developers
  - Create documentation explaining CI workflow
  - Add troubleshooting guide for common CI failures
  - Document how to test CI changes locally
  - _Requirements: 5.1, 5.2, 5.3_
