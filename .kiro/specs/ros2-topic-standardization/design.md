# Design Document

## Overview

This design document describes the implementation approach for standardizing ROS2 topics and completing the hardware interface implementation for the robot_xl platform. The solution involves changes to three main components: Pico firmware, ROS2 hardware interfaces, and launch configurations. We DONT need backward compatibbily!

## Architecture

### Current Architecture (Problematic)

```
┌─────────────────┐
│ mecanum_drive   │
│   controller    │
└────────┬────────┘
         │ velocity commands (hardware_interface)
         ▼
┌─────────────────┐
│ RobotSystem     │──X──> motors_cmd (unused)
│ HW Interface    │
└────────┬────────┘
         │ expects ~/motors_response
         ▼
    (no data received)
         │
         ▼
    MOCK MODE ❌

┌─────────────────┐
│ Pico Firmware   │
└────────┬────────┘
         │ publishes /rt/joint_states
         │ publishes /rt/ddd/imu
         │ subscribes /rt/cmd_vel
         ▼
┌─────────────────┐
│ micro-ROS Agent │──> /rt/* topics (unmapped)
└─────────────────┘
```

### Target Architecture (Standardized)

```
┌─────────────────┐
│ mecanum_drive   │
│   controller    │
└────────┬────────┘
         │ velocity commands (hardware_interface)
         ▼
┌─────────────────┐
│ RobotSystem     │──> /cmd_vel (Twist)
│ HW Interface    │<── /joint_states
└─────────────────┘

┌─────────────────┐
│ RobotImuSensor  │<── /imu/data_raw
│ HW Interface    │
└─────────────────┘

┌─────────────────┐
│ Pico Firmware   │
└────────┬────────┘
         │ publishes /joint_states
         │ publishes /imu/data_raw
         │ publishes /odom
         │ subscribes /cmd_vel
         ▼
┌─────────────────┐
│ micro-ROS Agent │──> Standard ROS2 topics
│ (with remapping)│
└─────────────────┘
```

## Components and Interfaces

### 1. Firmware Topic Standardization

#### Files to Modify:
- `firmware/src/DDD.cpp`
- `firmware/src/MotorsAgent.cpp`
- `firmware/src/application/ImuAgent.cpp`
- `firmware/src/application/vl6180xAgent.cpp`
- `firmware/src/HCSR04Agent.cpp`

#### Topic Mapping:

| Old Topic Name | New Topic Name | Message Type |
|----------------|----------------|--------------|
| `/ddd/imu` | `/imu/data_raw` | sensor_msgs/Imu |
| `/ddd/odom` | `/odom` | nav_msgs/Odometry |
| `/ddd/range_tof` | `/sensors/range_tof` | sensor_msgs/Range |
| `/ddd/range` | `/sensors/range_ultrasonic` | sensor_msgs/Range |
| `/ddd/illuminance` | `/sensors/illuminance` | sensor_msgs/Illuminance |
| `/ddd/cmd_vel` | (remove, use `/cmd_vel` only) | geometry_msgs/Twist |
| `/joint_states` | `/joint_states` | sensor_msgs/JointState (no change) |

#### Implementation Details:

**DDD.cpp:**
```cpp
// Change publisher initialization
rclc_publisher_init_default(
    &xPubOdom, node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), 
    "/odom");  // Changed from "/ddd/odom"

// Change subscriber initialization
rclc_subscription_init_default(
    &xSubTwist, node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
    "/cmd_vel");  // Changed from "/ddd/cmd_vel"
```

**ImuAgent.cpp:**
```cpp
rclc_publisher_init_default(
    &imu_publisher_, node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), 
    "/imu/data_raw");  // Changed from "/ddd/imu"
```

**vl6180xAgent.cpp:**
```cpp
rclc_publisher_init_default(
    &range_publisher_, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/sensors/range_tof");  // Changed from "/ddd/range_tof"

rclc_publisher_init_default(
    &illuminance_publisher_, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Illuminance),
    "/sensors/illuminance");  // Changed from "/ddd/illuminance"
```

**HCSR04Agent.cpp:**
```cpp
rclc_publisher_init_default(
    &xPubRange, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "/sensors/range_ultrasonic");  // Changed from "/ddd/range"
```

### 2. Hardware Interface Implementation

#### RobotSystem (Motor Control)

**File:** `src/robot_hardware_interfaces/src/robot_system.cpp`

**Key Changes:**

1. **Add Twist Publisher:**
```cpp
// In on_activate()
cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::SystemDefaultsQoS());
realtime_cmd_vel_publisher_ = 
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
        cmd_vel_publisher_);
```

2. **Implement write() Method:**
```cpp
return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (realtime_cmd_vel_publisher_->trylock()) {
        auto & cmd_vel_msg = realtime_cmd_vel_publisher_->msg_;
        
        // Convert wheel velocities to twist using inverse kinematics
        // For mecanum drive: 4 wheel velocities → linear_x, linear_y, angular_z
        
        double vx = (vel_commands_["front_left_wheel_joint"] + 
                     vel_commands_["front_right_wheel_joint"] +
                     vel_commands_["rear_left_wheel_joint"] + 
                     vel_commands_["rear_right_wheel_joint"]) / 4.0;
        
        double vy = (-vel_commands_["front_left_wheel_joint"] + 
                      vel_commands_["front_right_wheel_joint"] +
                      vel_commands_["rear_left_wheel_joint"] - 
                      vel_commands_["rear_right_wheel_joint"]) / 4.0;
        
        double omega = (-vel_commands_["front_left_wheel_joint"] + 
                         vel_commands_["front_right_wheel_joint"] -
                         vel_commands_["rear_left_wheel_joint"] + 
                         vel_commands_["rear_right_wheel_joint"]) / 4.0;
        
        cmd_vel_msg.linear.x = vx * wheel_radius_;
        cmd_vel_msg.linear.y = vy * wheel_radius_;
        cmd_vel_msg.angular.z = omega * wheel_radius_ / wheel_base_;
        
        realtime_cmd_vel_publisher_->unlockAndPublish();
    }
    
    return return_type::OK;
}
```

3. **Update read() Method:**
```cpp
return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
    std::shared_ptr<JointState> motor_state;
    received_motor_state_msg_ptr_.get(motor_state);
    
    if (!motor_state) {
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("RobotSystem"),
            *node_->get_clock(), 1000,
            "No joint states received from firmware");
        return return_type::ERROR;  // Changed from OK (mock mode)
    }
    
    // Update state interfaces from received joint states
    for (auto i = 0u; i < motor_state->name.size(); i++) {
        if (pos_state_.find(motor_state->name[i]) != pos_state_.end()) {
            pos_state_[motor_state->name[i]] = motor_state->position[i];
            vel_state_[motor_state->name[i]] = motor_state->velocity[i];
        }
    }
    
    return return_type::OK;
}
```

4. **Update on_activate():**
```cpp
CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Activating");
    
    // Wait for first joint state message
    auto start_time = node_->get_clock()->now();
    while (!received_motor_state_msg_ptr_.get()) {
        if ((node_->get_clock()->now() - start_time).seconds() > 5.0) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotSystem"), 
                        "Timeout waiting for joint states from firmware");
            return CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), 
                "Successfully activated with real hardware feedback");
    return CallbackReturn::SUCCESS;
}
```

#### RobotImuSensor

**File:** `src/robot_hardware_interfaces/src/robot_imu_sensor.cpp`

**Key Changes:**

1. **Update Subscriber Topic:**
```cpp
imu_subscriber_ = node_->create_subscription<Imu>(
    "/imu/data_raw",  // Changed from "~/imu"
    rclcpp::SensorDataQoS(),
    std::bind(&RobotImuSensor::imu_cb, this, std::placeholders::_1));
```

2. **Update read() Method:**
```cpp
return_type RobotImuSensor::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::shared_ptr<Imu> imu_msg;
    received_imu_msg_ptr_.get(imu_msg);
    
    if (!imu_msg) {
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("RobotImuSensor"),
            *node_->get_clock(), 1000,
            "No IMU data received from firmware");
        return return_type::ERROR;  // Changed from ERROR to be consistent
    }
    
    // Update state interfaces...
    return return_type::OK;
}
```

3. **Update on_activate():**
```cpp
CallbackReturn RobotImuSensor::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotImuSensor"), "Activating");
    
    // Wait for first IMU message
    auto start_time = node_->get_clock()->now();
    while (!received_imu_msg_ptr_.get()) {
        if ((node_->get_clock()->now() - start_time).seconds() > 5.0) {
            RCLCPP_ERROR(rclcpp::get_logger("RobotImuSensor"), 
                        "Timeout waiting for IMU data from firmware");
            return CallbackReturn::ERROR;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(rclcpp::get_logger("RobotImuSensor"), 
                "Successfully activated with real IMU feedback");
    return CallbackReturn::SUCCESS;
}
```

### 3. micro-ROS Agent Configuration

**File:** `src/robot_bringup/launch/microros_agent.launch.py`

**Topic Remapping Configuration:**
```python
topic_remappings = [
    # Standard ROS2 topics
    ('/rt/joint_states', '/joint_states'),
    ('/rt/imu/data_raw', '/imu/data_raw'),
    ('/rt/odom', '/odom'),
    
    # Sensor topics
    ('/rt/sensors/range_tof', '/sensors/range_tof'),
    ('/rt/sensors/range_ultrasonic', '/sensors/range_ultrasonic'),
    ('/rt/sensors/illuminance', '/sensors/illuminance'),
    
    # Command topics (bidirectional)
    ('/cmd_vel', '/rt/cmd_vel'),
]
```

### 4. Launch File Updates

**File:** `src/robot_controller/launch/controller.launch.py`

**Updated Remappings:**
```python
remappings=[
    # Controller outputs
    ("drive_controller/cmd_vel_unstamped", "cmd_vel"),
    ("drive_controller/odom", "odometry/wheels"),
    
    # Hardware interface connections (no remapping needed - direct topics)
    # /joint_states is already standard
    # /imu/data_raw is already standard
    
    # Event topics
    ("drive_controller/transition_event", "_drive_controller/transition_event"),
    ("imu_broadcaster/transition_event", "_imu_broadcaster/transition_event"),
    ("joint_state_broadcaster/transition_event", "_joint_state_broadcaster/transition_event"),
]
```

## Data Models

### Mecanum Drive Kinematics

**Forward Kinematics (Wheel velocities → Robot velocity):**
```
vx = (vfl + vfr + vrl + vrr) / 4 * r
vy = (-vfl + vfr + vrl - vrr) / 4 * r
ω = (-vfl + vfr - vrl + vrr) / 4 * r / L
```

Where:
- `vfl, vfr, vrl, vrr` = wheel angular velocities (rad/s)
- `r` = wheel radius
- `L` = wheel base (distance from center to wheel)

**Inverse Kinematics (Robot velocity → Wheel velocities):**
```
vfl = (vx - vy - ω*L) / r
vfr = (vx + vy + ω*L) / r
vrl = (vx + vy - ω*L) / r
vrr = (vx - vy + ω*L) / r
```

### Joint State Message Structure
```
sensor_msgs/JointState:
  header:
    stamp: current_time
  name: ["front_left_wheel_joint", "front_right_wheel_joint", 
         "rear_left_wheel_joint", "rear_right_wheel_joint"]
  position: [rad, rad, rad, rad]
  velocity: [rad/s, rad/s, rad/s, rad/s]
```

## Error Handling

### Connection Timeout
- **Timeout:** 5 seconds for initial connection
- **Action:** Return ERROR from on_activate()
- **Recovery:** Automatic retry on next activation attempt

### Missing Messages During Operation
- **Detection:** Check for null pointer in read() method
- **Action:** Log throttled error, return ERROR status
- **Recovery:** Controller manager will handle error state

### Invalid Joint Names
- **Detection:** Check joint names match expected names
- **Action:** Log error with mismatched names
- **Recovery:** Skip invalid joints, continue with valid ones

## Testing Strategy

### Unit Tests
1. **Hardware Interface Tests:**
   - Test velocity command conversion (wheel velocities → twist)
   - Test joint state parsing
   - Test timeout handling

2. **Firmware Tests:**
   - Verify topic names are correct
   - Test message publishing rates
   - Verify message content

### Integration Tests
1. **Hardware Interface + Firmware:**
   - Publish test joint states, verify hardware interface reads them
   - Send velocity commands from hardware interface, verify firmware receives them
   - Test connection timeout scenarios

2. **Full System Test:**
   - Launch complete system
   - Verify all topics are published
   - Send cmd_vel commands, verify robot moves
   - Verify odometry is published correctly

### Validation Criteria
- ✅ `ros2 topic list` shows standard topic names (no `/ddd/` prefix)
- ✅ `ros2 topic echo /joint_states` shows real encoder data
- ✅ `ros2 topic echo /imu/data_raw` shows real IMU data
- ✅ Publishing to `/cmd_vel` makes robot move
- ✅ No "mock mode" warnings in logs
- ✅ Hardware interface activation succeeds without timeout

## Migration Path

### Phase 1: Firmware Update (Breaking Change)
1. Update firmware topic names
2. Rebuild and flash firmware
3. Test firmware with tests, and Test firmware topics with `ros2 topic list`

### Phase 2: Hardware Interface Update
1. Implement velocity command publishing
2. Update state reading logic
3. Make mock mode fallback only with envireoument variable aviable like operation=dev 
4. Make tests and make Test with updated firmware 

### Phase 3: Launch Configuration
1. Update micro-ROS agent remappings
2. Update controller launch remappings
3.  Make tests and Test complete system if possible with a full automated script.

### Phase 4: Documentation
1. Update README with new topic names
2. Create migration guide
3. Update architecture diagrams

## Performance Considerations

### Publishing Rates
- Joint states: 100 Hz (firmware)
- IMU data: 50 Hz (firmware)
- Velocity commands: 100 Hz (hardware interface)
- Odometry: 50 Hz (controller)

### Latency Budget
- micro-ROS agent: < 1ms
- Hardware interface read/write: < 10ms
- Total control loop: < 20ms

### Resource Usage
- Additional Twist publisher: ~1KB memory
- Topic remapping: negligible CPU overhead
- No significant performance impact expected
