# Implementation Plan

- [x] 1. Update Pico Firmware Topic Names
  - Update all topic publishers and subscribers to use standard ROS2 names
  - add tests save it to firmware/tests
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7_

- [x] 1.1 Update DDD.cpp topic names
  - Change odometry publisher from `/ddd/odom` to `/odom`
  - Change cmd_vel subscriber from `/ddd/cmd_vel` to `/cmd_vel` (remove alternative)
  - Update frame_id assignments if needed
  - add tests save it to firmware/tests
  - _Requirements: 1.3, 1.6_

- [x] 1.2 Update ImuAgent.cpp topic name
  - Change IMU publisher from `/ddd/imu` to `/imu/data_raw`
  - Verify frame_id is set to `imu_link`
  - _Requirements: 1.3_

- [x] 1.3 Update vl6180xAgent.cpp topic names
  - Change range publisher from `/ddd/range_tof` to `/sensors/range_tof`
  - Change illuminance publisher from `/ddd/illuminance` to `/sensors/illuminance`
  - _Requirements: 1.5_

- [x] 1.4 Update HCSR04Agent.cpp topic name
  - Change range publisher from `/ddd/range` to `/sensors/range_ultrasonic`
  - Update frame_id for each sensor (range_front, range_back)
  - _Requirements: 1.5_

- [x] 1.5 Rebuild and flash firmware to Pico
  - Run `make build` in firmware directory
  - Flash firmware to Pico via USB
  - Verify firmware boots correctly via serial monitor
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 1.6_

- [x] 1.6 Test firmware topics with micro-ROS agent
  - Start micro-ROS agent
  - Run `ros2 topic list` and verify new topic names appear with `/rt/` prefix
  - Echo each topic to verify data is being published
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

- [x] 2. Implement Hardware Interface Velocity Command Publishing
  - Add Twist publisher to RobotSystem hardware interface
  - Implement inverse kinematics to convert wheel velocities to twist
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5, 2.6_

- [x] 2.1 Add Twist publisher member variables to robot_system.hpp
  - Add `cmd_vel_publisher_` (rclcpp::Publisher<geometry_msgs::msg::Twist>)
  - Add `realtime_cmd_vel_publisher_` (RealtimePublisher)
  - Add wheel parameters (wheel_radius, wheel_base) as member variables
  - Remove obsolete motor_command_publisher_ declarations
  - _Requirements: 2.1, 7.1, 7.2, 7.3_

- [x] 2.2 Initialize Twist publisher in on_activate()
  - Create publisher for `/cmd_vel` topic with SystemDefaultsQoS
  - Create realtime publisher wrapper
  - Read wheel parameters from hardware_info
  - _Requirements: 2.1, 2.6_

- [x] 2.3 Implement write() method with inverse kinematics
  - Calculate robot velocity (vx, vy, omega) from wheel velocities
  - Apply mecanum drive forward kinematics formula
  - Populate Twist message with calculated velocities
  - Publish Twist message using realtime publisher
  - _Requirements: 2.2, 2.3_

- [x] 2.4 Add safety timeout for velocity commands
  - Track last command timestamp
  - Publish zero velocity if no commands received for 500ms
  - Log warning when timeout occurs
  - _Requirements: 2.4_

- [x] 2.5 Update cleanup_node() to reset Twist publisher
  - Reset realtime_cmd_vel_publisher_
  - Reset cmd_vel_publisher_
  - Remove obsolete motor_command_publisher_ cleanup
  - _Requirements: 2.5, 7.3_

- [x] 2.6 Update on_deactivate() to stop publishing
  - Publish final zero velocity command
  - Call cleanup_node()
  - _Requirements: 2.5_

- [x] 3. Implement Hardware Interface State Reading
  - Update RobotSystem to properly read joint states from firmware
  - Remove mock mode fallback behavior
  - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5, 3.6_

- [x] 3.1 Update motor_state_subscriber_ topic in on_activate()
  - Subscribe to `/joint_states` directly (no remapping needed)
  - Use SensorDataQoS for real-time performance
  - _Requirements: 3.1_

- [x] 3.2 Update read() method to handle missing data correctly
  - Check if motor_state is null
  - Return ERROR status instead of using mock values
  - Log throttled error message when no data received
  - Remove mock mode integration code
  - _Requirements: 3.2, 3.4, 3.5_

- [x] 3.3 Verify joint name mapping in read() method
  - Ensure joint names match firmware: front_left_wheel_joint, front_right_wheel_joint, rear_left_wheel_joint, rear_right_wheel_joint
  - Log error if joint names don't match
  - Update position and velocity state interfaces
  - _Requirements: 3.3_

- [x] 3.4 Implement activation wait for first joint state
  - Wait up to 5 seconds for first joint state message in on_activate()
  - Return ERROR if timeout occurs
  - Log success message when real hardware feedback is received
  - Remove "mock mode enabled" warning
  - _Requirements: 3.6_

- [x] 3.5 Add connection timeout parameter to URDF
  - Verify connection_timeout_ms parameter exists in ros2_control.urdf.xacro
  - Set reasonable default (5000ms)
  - _Requirements: 3.4_

- [x] 4. Update IMU Hardware Interface
  - Anlyse first the workspace
  - Update RobotImuSensor to read from standardized IMU topic
  - Remove mock mode fallback behavior
  - add tests save it in a test folder close to the changes
  - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5_

- [x] 4.1 Update IMU subscriber topic in on_activate()
  - Anlyse first the workspace
  - Subscribe to `/imu/data_raw` directly
  - Use SensorDataQoS for real-time performance
  - add tests save it in a test folder close to the changes
  - _Requirements: 4.1_

- [x] 4.2 Update read() method error handling
  - Keep ERROR return when no IMU data received
  - Add throttled error logging
  - Ensure all 10 state interfaces are updated correctly
  - add tests save it in a test folder close to the changes
  - _Requirements: 4.2, 4.3_

- [x] 4.3 Implement activation wait for first IMU message
  - Wait up to 5 seconds for first IMU message in on_activate()
  - Return ERROR if timeout occurs
  - Log success message when real IMU feedback is received
  - Remove "mock mode enabled" warning
  - add tests save it in a test folder close to the changes
  - _Requirements: 4.5_

- [-] 5. Configure micro-ROS Agent Topic Remapping
  - Make a script or test that provides an log fill from the Pico firmware that we know the correct topics
  - Update micro-ROS agent launch file with correct topic remappings
  - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6_

- [-] 5.1 Update microros_agent.launch.py with topic remappings
  - Anlyse first the workspace
  - Map `/rt/joint_states` → `/joint_states`
  - Map `/rt/imu/data_raw` → `/imu/data_raw`
  - Map `/rt/odom` → `/odom`
  - Map `/rt/sensors/range_tof` → `/sensors/range_tof`
  - Map `/rt/sensors/range_ultrasonic` → `/sensors/range_ultrasonic`
  - Map `/rt/sensors/illuminance` → `/sensors/illuminance`
  - Map `/cmd_vel` → `/rt/cmd_vel` (bidirectional)
  - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5_

- [ ] 5.2 Add documentation comments to launch file
  - Document each topic remapping with inline comments
  - Explain the /rt/ prefix behavior
  - Reference firmware topic names
  - _Requirements: 5.6_

- [ ] 6. Update Controller Launch Configuration
  - Update controller.launch.py with correct topic remappings
  - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

- [ ] 6.1 Update ros2_control_node remappings in controller.launch.py
  - Remove `~/motors_cmd` remapping (no longer used)
  - Verify `~/motors_response` → `/joint_states` remapping
  - Verify `imu_sensor_node/imu` → `/imu/data_raw` remapping
  - _Requirements: 6.1, 6.2_

- [ ] 6.2 Verify mecanum_drive_controller remappings
  - Ensure `drive_controller/cmd_vel_unstamped` → `cmd_vel`
  - Ensure `drive_controller/odom` → `odometry/wheels`
  - _Requirements: 6.3, 6.4_

- [ ] 6.3 Add documentation comments to remappings
  - Document data flow for each remapping
  - Explain which component publishes/subscribes
  - _Requirements: 6.5_

- [ ] 7. Build and Test System Integration
  - Build updated packages and test complete system
  - _Requirements: All_

- [ ] 7.1 Build updated ROS2 packages
  - Run `colcon build --packages-select robot_hardware_interfaces robot_bringup robot_controller`
  - Verify no compilation errors
  - Source workspace setup
  - _Requirements: All_

- [ ] 7.2 Launch complete system
  - Run `ros2 launch robot_bringup hardware_bringup.launch.py`
  - Verify all nodes start successfully
  - Check for error messages in logs
  - _Requirements: All_

- [ ] 7.3 Verify topic names with ros2 topic list
  - Run `ros2 topic list`
  - Verify `/joint_states` exists (not `/rt/joint_states`)
  - Verify `/imu/data_raw` exists (not `/ddd/imu`)
  - Verify `/odom` exists (not `/ddd/odom`)
  - Verify `/sensors/*` topics exist
  - Verify `/cmd_vel` exists
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

- [ ] 7.4 Test joint states data
  - Run `ros2 topic echo /joint_states`
  - Verify 4 joints are published: front_left, front_right, rear_left, rear_right
  - Verify position and velocity data is non-zero and changing
  - Verify publishing rate is ~100Hz
  - _Requirements: 3.1, 3.2, 3.3_

- [ ] 7.5 Test IMU data
  - Run `ros2 topic echo /imu/data_raw`
  - Verify orientation, angular_velocity, and linear_acceleration are published
  - Verify data is non-zero and realistic
  - Verify publishing rate is ~50Hz
  - _Requirements: 4.1, 4.2_

- [ ] 7.6 Test velocity commands
  - Run `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once`
  - Verify robot moves forward
  - Verify joint states show wheel velocities changing
  - Stop command and verify robot stops
  - _Requirements: 2.1, 2.2, 2.3_

- [ ] 7.7 Check hardware interface logs
  - Verify no "mock mode" warnings appear
  - Verify "Successfully activated with real hardware feedback" messages
  - Verify no ERROR messages about missing data
  - _Requirements: 3.5, 3.6, 4.4, 4.5_

- [ ] 7.8 Test with RViz2 visualization
  - Launch RViz2 with robot model
  - Add displays for /joint_states, /imu/data_raw, /odom
  - Verify robot model moves correctly
  - Verify IMU orientation is displayed
  - _Requirements: All_

- [ ] 8. Update Documentation
  - Update project documentation with new topic names and architecture
  - _Requirements: 8.1, 8.2, 8.3, 8.4_

- [ ] 8.1 Create migration guide document
  - Document breaking changes in topic names
  - Provide before/after comparison table
  - Include troubleshooting section
  - _Requirements: 8.4_

- [ ] 8.2 Update README.md with new topic names
  - Update topic list in main README
  - Update architecture diagrams if present
  - Add link to migration guide
  - _Requirements: 8.3_

- [ ] 8.3 Update PINMAP.md or hardware documentation
  - Verify sensor topic names are documented
  - Update any references to old topic names
  - _Requirements: 8.3_

- [ ] 8.4 Add comments explaining removed code
  - Add comments in hardware interface explaining why motor_command_publisher was removed
  - Document the architectural decision to use Twist instead of Float32MultiArray
  - _Requirements: 7.5_
