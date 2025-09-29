// Contract test stub for /odom publisher (nav_msgs/Odometry)
// This test intentionally fails until the publisher contract is implemented.
// Test procedure: Subscribe to /odom and verify odometry data publishing
#include <cstdlib>
#include <iostream>

int main() {
    std::cerr << "T009: test_odom - NOT IMPLEMENTED\n";
    std::cerr << "Test procedure: ros2 topic echo /odom\n";
    std::cerr << "Expected: Continuous publishing of nav_msgs/Odometry with pose, twist, and "
                 "covariance data calculated from encoder readings\n";
    return EXIT_FAILURE;
}