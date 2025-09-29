# Integration Test: Sensor Data Publishing

**Feature**: Sensor Data Publishing Integration with micro-ROS
**Date**: 2025-09-25
**Source**: quickstart.md, contracts/micro-ros-contracts.md

## Test Objective

Verify that the firmware correctly publishes sensor data on all required ROS2 topics with appropriate data rates and content.

## Test Environment

- **Hardware**: Raspberry Pi Pico with ICM-20948 IMU, VL53L0X ToF sensor, INA3221 battery monitor, motor encoders
- **Software**: ROS2 Humble, micro-ROS agent, firmware flashed to Pico
- **Prerequisites**: Firmware built and flashed, micro-ROS agent running, Pico connected

## Test Setup

1. Build and flash firmware to Pico
2. Start micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
3. Verify connection: `ros2 topic list` should show all publisher topics

## Test Cases

### Test Case 1: IMU Data Publishing (/imu/data_raw)

| Test ID | Test Action                            | Expected Behavior                                   | Verification                         |
| ------- | -------------------------------------- | --------------------------------------------------- | ------------------------------------ |
| SP-1.1  | `ros2 topic echo /imu/data_raw --once` | Single sensor_msgs/Imu message received             | Message structure validation         |
| SP-1.2  | `ros2 topic hz /imu/data_raw`          | Publishing rate ~100 Hz                             | Rate measurement over 10 seconds     |
| SP-1.3  | `ros2 topic echo /imu/data_raw`        | Continuous accelerometer, gyroscope data            | Data ranges: accel ±2g, gyro ±250°/s |
| SP-1.4  | Move Pico, observe data changes        | Accelerometer/gyroscope values change appropriately | Qualitative data validation          |

### Test Case 2: Odometry Publishing (/odom)

| Test ID | Test Action                                  | Expected Behavior                         | Verification                     |
| ------- | -------------------------------------------- | ----------------------------------------- | -------------------------------- |
| SP-2.1  | `ros2 topic echo /odom --once`               | Single nav_msgs/Odometry message received | Message structure validation     |
| SP-2.2  | `ros2 topic hz /odom`                        | Publishing rate ~50 Hz                    | Rate measurement over 10 seconds |
| SP-2.3  | Rotate wheels manually, observe odom changes | Pose and twist values update              | Encoder integration validation   |
| SP-2.4  | Check covariance matrices                    | Non-zero covariance values present        | Uncertainty representation       |

### Test Case 3: ToF Sensor Publishing (/tof_sensor)

| Test ID | Test Action                          | Expected Behavior                            | Verification                     |
| ------- | ------------------------------------ | -------------------------------------------- | -------------------------------- |
| SP-3.1  | `ros2 topic echo /tof_sensor --once` | Single sensor_msgs/Range message received    | Message structure validation     |
| SP-3.2  | `ros2 topic hz /tof_sensor`          | Publishing rate ~10 Hz                       | Rate measurement over 10 seconds |
| SP-3.3  | Place object at known distance       | Range reading matches actual distance (±5cm) | Distance accuracy validation     |
| SP-3.4  | Remove object (out of range)         | Range shows maximum value                    | Out-of-range handling            |

### Test Case 4: Battery State Publishing (/battery_state)

| Test ID | Test Action                             | Expected Behavior                                | Verification                     |
| ------- | --------------------------------------- | ------------------------------------------------ | -------------------------------- |
| SP-4.1  | `ros2 topic echo /battery_state --once` | Single sensor_msgs/BatteryState message received | Message structure validation     |
| SP-4.2  | `ros2 topic hz /battery_state`          | Publishing rate ~10 Hz                           | Rate measurement over 10 seconds |
| SP-4.3  | Check voltage/current values            | Reasonable battery values present                | Data range validation            |
| SP-4.4  | Monitor during discharge                | Percentage decreases appropriately               | Battery monitoring functionality |

### Test Case 5: Multi-Topic Coordination

| Test ID | Test Action                       | Expected Behavior                        | Verification                     |
| ------- | --------------------------------- | ---------------------------------------- | -------------------------------- |
| SP-5.1  | `ros2 topic list`                 | All 4 sensor topics present              | Topic discovery                  |
| SP-5.2  | Monitor all topics simultaneously | No timing conflicts or data corruption   | Concurrent publishing validation |
| SP-5.3  | Check timestamps                  | Timestamps are reasonable and increasing | Time synchronization             |

## Performance Requirements

- **IMU**: 100 Hz publishing rate, < 5ms latency
- **Odometry**: 50 Hz publishing rate, < 10ms latency
- **ToF Sensor**: 10 Hz publishing rate, < 50ms latency
- **Battery**: 10 Hz publishing rate, < 50ms latency

## Data Quality Checks

- **IMU**: Accelerometer: ±2g range, Gyroscope: ±250°/s range
- **Odometry**: Pose covariance properly set, frame_id = "odom"
- **ToF**: Range: 0.03-2.0m, field_of_view ≈ 25°, radiation_type = INFRARED
- **Battery**: Voltage: 9-13V typical, Current: ±5A range, Percentage: 0-100%

## Pass Criteria

- All topics publish at specified rates
- Data values are within expected ranges
- Message structures match ROS2 standards
- Timestamps are properly set and monotonic
- No data corruption during concurrent publishing