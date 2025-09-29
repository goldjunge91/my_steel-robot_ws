# Integration Test: Motor Control with ROS2 Twist Commands

**Feature**: Motor Control Integration with micro-ROS
**Date**: 2025-09-25
**Source**: quickstart.md, contracts/micro-ros-contracts.md

## Test Objective

Verify that the firmware correctly processes `geometry_msgs/Twist` messages on `/cmd_vel` topic and produces the expected motor movements for omnidirectional robot control.

## Test Environment

- **Hardware**: Raspberry Pi Pico with 4x DC motors, TB6612FNG drivers, Hall encoders
- **Software**: ROS2 Humble, micro-ROS agent, firmware flashed to Pico
- **Prerequisites**: Firmware built and flashed, micro-ROS agent running, Pico connected

## Test Setup

1. Build and flash firmware to Pico
2. Start micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
3. Verify connection: `ros2 topic list` should show `/cmd_vel` as subscribed topic

## Test Cases

### Test Case 1: Forward Motion

| Test ID | ROS2 Command                                                                                                          | Expected Motor Behavior                    | Verification                                  |
| ------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ | --------------------------------------------- |
| MC-1.1  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"` | All 4 motors forward at proportional speed | Visual inspection of motor rotation           |
| MC-1.2  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"` | All 4 motors forward at full speed         | Visual inspection, PWM duty cycle measurement |

### Test Case 2: Backward Motion

| Test ID | ROS2 Command                                                                                                           | Expected Motor Behavior                     | Verification                        |
| ------- | ---------------------------------------------------------------------------------------------------------------------- | ------------------------------------------- | ----------------------------------- |
| MC-2.1  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"` | All 4 motors backward at proportional speed | Visual inspection of motor rotation |

### Test Case 3: Strafe Right

| Test ID | ROS2 Command                                                                                                           | Expected Motor Behavior                                       | Verification                          |
| ------- | ---------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------- | ------------------------------------- |
| MC-3.1  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"` | Front-left/back-right forward, front-right/back-left backward | Visual inspection for strafing motion |

### Test Case 4: Rotation

| Test ID | ROS2 Command                                                                                                          | Expected Motor Behavior                    | Verification                             |
| ------- | --------------------------------------------------------------------------------------------------------------------- | ------------------------------------------ | ---------------------------------------- |
| MC-4.1  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"` | Left motors forward, right motors backward | Visual inspection for clockwise rotation |

### Test Case 5: Combined Motion

| Test ID | ROS2 Command                                                                                                          | Expected Motor Behavior             | Verification                           |
| ------- | --------------------------------------------------------------------------------------------------------------------- | ----------------------------------- | -------------------------------------- |
| MC-5.1  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"` | Forward motion with slight rotation | Visual inspection of combined movement |

### Test Case 6: Stop Command

| Test ID | ROS2 Command                                                                                                          | Expected Motor Behavior | Verification                        |
| ------- | --------------------------------------------------------------------------------------------------------------------- | ----------------------- | ----------------------------------- |
| MC-6.1  | `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"` | All motors stop         | Visual inspection of motor stopping |

## Safety Checks

- **Timeout Test**: Send command, wait 200ms, verify motors stop automatically
- **Out-of-range Test**: Send Twist with values > 1.0, verify motors don't exceed safe limits
- **Connection Loss Test**: Disconnect micro-ROS agent, verify motors stop within 200ms

## Pass Criteria

- All motor movements match expected directions and relative speeds
- No motor overheating or unusual noises
- Commands are processed within 10ms of receipt
- Safety timeout works correctly
- Encoder feedback is published on `/odom` topic