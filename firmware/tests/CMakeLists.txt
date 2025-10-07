# Test suite for my_firmware
project(my_firmware_tests)

# Include directories for source and googletest
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${gtest_SOURCE_DIR}/include
    ${gtest_SOURCE_DIR}
)

# Create test executable - include source files being tested
add_executable(firmware_tests
    test_motor_pid.cpp
    test_odometry.cpp
    test_basic_math.cpp
)

# Link against googletest and gtest_main
target_link_libraries(firmware_tests
    gtest
    gtest_main
)

# Enable CTest integration
enable_testing()
add_test(NAME firmware_tests COMMAND firmware_tests)