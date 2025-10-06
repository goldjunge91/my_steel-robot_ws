# Requirements Document

## Introduction

This specification defines the standardization of ROS2 topics and hardware interface architecture for the robot_xl platform. The current implementation has inconsistent topic naming and an incomplete hardware interface that causes the system to run in mock mode despite having real hardware connected.

The goal is to align the system with ROS2 best practices (REP-105) and ros2_control standards, ensuring proper communication between the Pico firmware, hardware interface, and ROS2 controllers.

## Requirements

### Requirement 1: Standardize Firmware Topic Names

**User Story:** As a ROS2 developer, I want the robot to use standard ROS2 topic names, so that it's compatible with standard ROS2 tools and packages (Nav2, RViz, etc.).

#### Acceptance Criteria

1. WHEN the Pico firmware publishes sensor data THEN it SHALL use standard ROS2 topic names according to REP-105
2. WHEN the Pico firmware publishes joint states THEN it SHALL publish to `/joint_states` (not `/rt/joint_states`)  names according to REP-105
3. WHEN the Pico firmware publishes IMU data THEN it SHALL publish to `/imu/data_raw` (not `/ddd/imu`) names according to REP-105
4. WHEN the Pico firmware publishes odometry THEN it SHALL publish to `/odom` (not `/ddd/odom`) names according to REP-105
5. WHEN the Pico firmware publishes range sensors THEN it SHALL use descriptive names like `/sensors/range_tof` and `/sensors/range_ultrasonic`  names according to REP-105
6. WHEN the Pico firmware subscribes to velocity commands THEN it SHALL subscribe to `/cmd_vel` only (remove `/ddd/cmd_vel` alternative) names according to REP-105
7. WHEN topics are published THEN they SHALL NOT use robot-specific prefixes like `ddd` in the topic name

### Requirement 2: Implement Hardware Interface Velocity Command Publishing

**User Story:** As a ros2_control user, I want the hardware interface to properly convert controller commands to firmware messages, so that the robot responds to velocity commands from the mecanum_drive_controller.

#### Acceptance Criteria

1. WHEN the hardware interface `write()` method is called THEN it SHALL convert velocity commands from the controller to a `geometry_msgs::Twist` message
2. WHEN velocity commands are converted THEN the hardware interface SHALL calculate the twist message from the four wheel velocity commands using inverse kinematics
3. WHEN the twist message is ready THEN it SHALL be published to `/cmd_vel` topic
4. WHEN no velocity commands are received for 500ms THEN the hardware interface SHALL publish a zero-velocity twist message (safety stop)
5. WHEN the hardware interface is deactivated THEN it SHALL stop publishing velocity commands
6. WHEN velocity commands are published THEN they SHALL use `rclcpp::SensorDataQoS()` for real-time performance

### Requirement 3: Implement Hardware Interface State Reading

**User Story:** As a ros2_control user, I want the hardware interface to read real sensor data from the firmware, so that the system operates with actual hardware feedback instead of mock mode it is importent that we never implement MOCK hardware.

#### Acceptance Criteria

1. WHEN the hardware interface `read()` method is called THEN it SHALL read the latest `/joint_states` message from the firmware 
2. WHEN joint states are received THEN the hardware interface SHALL update position and velocity state interfaces for all four wheels
3. WHEN joint states match the expected wheel names THEN the hardware interface SHALL map them correctly (front_left_wheel_joint, front_right_wheel_joint, rear_left_wheel_joint, rear_right_wheel_joint)
4. WHEN no joint states are received within the connection timeout THEN the hardware interface SHALL log an error and return ERROR status
5. WHEN joint states are successfully read THEN the hardware interface SHALL NOT use mock values
6. WHEN the hardware interface activates THEN it SHALL wait for at least one valid joint state message before confirming activation

### Requirement 4: Update IMU Hardware Interface

**User Story:** As a ros2_control user, I want the IMU hardware interface to read real IMU data from the firmware, so that sensor fusion and localization work correctly.

#### Acceptance Criteria

1. WHEN the IMU hardware interface `read()` method is called THEN it SHALL read the latest `/imu/data_raw` message from the firmware
2. WHEN IMU data is received THEN the hardware interface SHALL update all IMU state interfaces (orientation, angular_velocity, linear_acceleration)
3. WHEN no IMU data is received within the connection timeout THEN the hardware interface SHALL log an error and return ERROR status
4. WHEN IMU data is successfully read THEN the hardware interface SHALL NOT use mock values
5. WHEN the IMU hardware interface activates THEN it SHALL wait for at least one valid IMU message before confirming activation

### Requirement 5: Configure micro-ROS Agent Topic Remapping

**User Story:** As a system integrator, I want the micro-ROS agent to properly map firmware topics to ROS2 topics, so that the `/rt/` prefix is handled correctly.

#### Acceptance Criteria

1. WHEN the micro-ROS agent starts THEN it SHALL map `/rt/joint_states` to `/joint_states`
2. WHEN the micro-ROS agent starts THEN it SHALL map `/rt/imu/data_raw` to `/imu/data_raw`
3. WHEN the micro-ROS agent starts THEN it SHALL map `/rt/odom` to `/odom`
4. WHEN the micro-ROS agent starts THEN it SHALL map `/rt/sensors/*` topics to `/sensors/*`
5. WHEN ROS2 publishes to `/cmd_vel` THEN the micro-ROS agent SHALL map it to `/rt/cmd_vel` for the firmware
6. WHEN topic remapping is configured THEN it SHALL be documented in the launch file comments

### Requirement 6: Update Controller Launch Configuration

**User Story:** As a system operator, I want the controller launch files to have correct topic remappings, so that all components communicate properly.

#### Acceptance Criteria

1. WHEN the ros2_control_node starts THEN it SHALL remap `~/motors_response` to `/joint_states`
2. WHEN the ros2_control_node starts THEN it SHALL remap `imu_sensor_node/imu` to `/imu/data_raw`
3. WHEN the mecanum_drive_controller starts THEN it SHALL publish `cmd_vel_unstamped` to `/cmd_vel`
4. WHEN the mecanum_drive_controller starts THEN it SHALL publish odometry to `/odometry/wheels`
5. WHEN topic remappings are configured THEN they SHALL be clearly documented with comments explaining the data flow

### Requirement 7: Remove Obsolete Code

**User Story:** As a maintainer, I want obsolete code removed from the hardware interface, so that the codebase is clean and maintainable.

#### Acceptance Criteria

1. WHEN the hardware interface is refactored THEN it SHALL remove the unused `motor_command_publisher_` (Float32MultiArray)
2. WHEN the hardware interface is refactored THEN it SHALL remove the unused `realtime_motor_command_publisher_`
3. WHEN obsolete code is removed THEN the header file SHALL be updated to remove unused member variables
4. WHEN obsolete code is removed THEN all related includes SHALL be cleaned up
5. WHEN code is removed THEN comments SHALL explain why the old approach was replaced

## Non-Functional Requirements

### Performance
- Topic publishing SHALL maintain at least 100Hz for joint states
- Hardware interface read/write cycle SHALL complete within 10ms
- Topic remapping SHALL add less than 1ms latency

### Reliability
- Hardware interface SHALL handle missing messages gracefully
- System SHALL recover automatically when firmware reconnects
- All error conditions SHALL be logged with appropriate severity levels

### Maintainability
- Code SHALL follow ROS2 coding standards
- All topic names SHALL be documented in a central location
- Launch files SHALL include clear comments explaining topic flow
