# Integration Test: Launcher Control with ROS2 JointState Commands

**Feature**: Launcher Control Integration with micro-ROS
**Date**: 2025-09-25
**Source**: quickstart.md, contracts/micro-ros-contracts.md

## Test Objective

Verify that the firmware correctly processes `sensor_msgs/JointState` messages on `/launcher_control` topic and produces the expected pan/tilt servo movements.

## Test Environment

- **Hardware**: Raspberry Pi Pico with pan/tilt servos for launcher control
- **Software**: ROS2 Humble, micro-ROS agent, firmware flashed to Pico
- **Prerequisites**: Firmware built and flashed, micro-ROS agent running, Pico connected

## Test Setup

1. Build and flash firmware to Pico
2. Start micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
3. Verify connection: `ros2 topic list` should show `/launcher_control` as subscribed topic

## Test Cases

### Test Case 1: Pan Movement

| Test ID | ROS2 Command                                                                                                                                                                         | Expected Servo Behavior                        | Verification                            |
| ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------- | --------------------------------------- |
| LC-1.1  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['pan'], position: [0.0], velocity: [], effort: []}"`  | Pan servo moves to center position (0 radians) | Visual inspection of pan servo movement |
| LC-1.2  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['pan'], position: [0.5], velocity: [], effort: []}"`  | Pan servo moves to +0.5 rad position           | Visual inspection, angle measurement    |
| LC-1.3  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['pan'], position: [-0.5], velocity: [], effort: []}"` | Pan servo moves to -0.5 rad position           | Visual inspection, angle measurement    |

### Test Case 2: Tilt Movement

| Test ID | ROS2 Command                                                                                                                                                                         | Expected Servo Behavior                         | Verification                             |
| ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------- | ---------------------------------------- |
| LC-2.1  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['tilt'], position: [0.0], velocity: [], effort: []}"` | Tilt servo moves to center position (0 radians) | Visual inspection of tilt servo movement |
| LC-2.2  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['tilt'], position: [0.3], velocity: [], effort: []}"` | Tilt servo moves to +0.3 rad position           | Visual inspection, angle measurement     |

### Test Case 3: Combined Pan and Tilt

| Test ID | ROS2 Command                                                                                                                                                                                     | Expected Servo Behavior                     | Verification                     |
| ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------- | -------------------------------- |
| LC-3.1  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['pan', 'tilt'], position: [0.2, 0.1], velocity: [], effort: []}"` | Pan moves to 0.2 rad, tilt moves to 0.1 rad | Visual inspection of both servos |

### Test Case 4: Invalid Joint Names

| Test ID | ROS2 Command                                                                                                                                                                            | Expected Behavior                  | Verification                      |
| ------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- | --------------------------------- |
| LC-4.1  | `ros2 topic pub /launcher_control sensor_msgs/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['invalid'], position: [0.0], velocity: [], effort: []}"` | Message ignored, no servo movement | Visual inspection, logged warning |

## Safety Checks

- **Range Limits**: Send JointState with positions outside safe range (±π/2), verify servos don't move beyond limits
- **Velocity/Effort**: Send JointState with velocity/effort values, verify they are ignored (position control only)
- **Empty Message**: Send empty JointState, verify no movement

## Pass Criteria

- Servo movements match expected positions within ±5 degrees accuracy
- Combined pan/tilt movements work correctly
- Invalid joint names are properly ignored
- Safety limits are enforced
- Commands are processed within 50ms of receipt
