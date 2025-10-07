cmake_minimum_required(VERSION 3.22)
# Test suite for my_firmware
project(my_firmware_tests)

# Build HAL library for tests
add_subdirectory(hal)

# Include directories for source and googletest
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${gtest_SOURCE_DIR}/include
    ${gtest_SOURCE_DIR}
)

# Create test executable - include source files being tested
add_executable(firmware_tests
    test_motor_pid.cpp
    test_odometry.cpp
    test_basic_math.cpp
    test_hal_mocks.cpp
)

# Link against googletest, gtest_main, and HAL mocks
target_link_libraries(firmware_tests
    gtest
    gtest_main
    hal_interfaces
    hal_mocks
)

# Enable CTest integration
enable_testing()
add_test(NAME firmware_tests COMMAND firmware_tests)