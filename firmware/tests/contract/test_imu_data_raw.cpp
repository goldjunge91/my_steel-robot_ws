// Contract test stub for /imu/data_raw publisher (sensor_msgs/Imu)
// This test intentionally fails until the publisher contract is implemented.
// Test procedure: Subscribe to /imu/data_raw and verify IMU data publishing
#include <cstdlib>
#include <iostream>

int main() {
    std::cerr << "T008: test_imu_data_raw - NOT IMPLEMENTED\n";
    std::cerr << "Test procedure: ros2 topic echo /imu/data_raw\n";
    std::cerr << "Expected: Continuous publishing of sensor_msgs/Imu with accelerometer, "
                 "gyroscope, and orientation data at ~100Hz\n";
    return EXIT_FAILURE;
}