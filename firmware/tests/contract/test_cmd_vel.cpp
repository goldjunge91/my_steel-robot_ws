// Contract test stub for /cmd_vel subscriber (geometry_msgs/Twist)
// This test intentionally fails until the subscriber contract is implemented.
// Test procedure: Publish geometry_msgs/Twist to /cmd_vel and verify motor movement
#include <cstdlib>
#include <iostream>

int main() {
    std::cerr << "T005: test_cmd_vel - NOT IMPLEMENTED\n";
    std::cerr << "Test procedure: ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, "
                 "y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'\n";
    std::cerr << "Expected: Motors should move according to twist command\n";
    return EXIT_FAILURE;
}