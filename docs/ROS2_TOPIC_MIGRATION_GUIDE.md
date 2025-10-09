# ROS2 Topic Standardization Migration Guide

## Overview

This guide documents the breaking changes introduced by the ROS2 topic standardization effort for the robot_xl platform. The changes align the system with ROS2 best practices (REP-105) and ros2_control standards, ensuring proper communication between the Pico firmware, hardware interface, and ROS2 controllers.

**Migration Date**: 2025-10-09  
**Affected Components**: Pico firmware, hardware interfaces, launch files, micro-ROS agent configuration

## Breaking Changes Summary

### 1. Topic Name Changes

All custom `/ddd/*` prefixed topics have been replaced with standard ROS2 topic names according to REP-105.

### 2. Hardware Interface Architecture

The hardware interface now uses standard `geometry_msgs/Twist` messages instead of custom `std_msgs/Float32MultiArray` for velocity commands.

### 3. Mock Mode Removed

The hardware interface no longer falls back to mock mode. Real hardware feedback is required for operation.

## Topic Name Migration Table

| Old Topic Name | New Topic Name | Message Type | Direction | Notes |
|----------------|----------------|--------------|-----------|-------|
| `/ddd/imu` | `/imu/data_raw` | `sensor_msgs/Imu` | Publish | Standard IMU topic name |
| `/ddd/odom` | `/odom` | `nav_msgs/Odometry` | Publish | Standard odometry topic |
| `/ddd/range_tof` | `/sensors/range_tof` | `sensor_msgs/Range` | Publish | VL6180X ToF sensor |
| `/ddd/range` | `/sensors/range_ultrasonic` | `sensor_msgs/Range` | Publish | HC-SR04 ultrasonic sensor |
| `/ddd/illuminance` | `/sensors/illuminance` | `sensor_msgs/Illuminance` | Publish | VL6180X illuminance |
| `/ddd/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Removed alternative topic |
| `/joint_states` | `/joint_states` | `sensor_msgs/JointState` | Publish | No change (already standard) |

### micro-ROS Agent Topic Remapping

The micro-ROS agent automatically adds `/rt/` prefix to all firmware topics. The agent configuration now includes remappings to remove this prefix:

| Firmware Topic (with /rt/) | ROS2 Topic | Direction |
|----------------------------|------------|-----------|
| `/rt/joint_states` | `/joint_states` | Firmware → ROS2 |
| `/rt/imu/data_raw` | `/imu/data_raw` | Firmware → ROS2 |
| `/rt/odom` | `/odom` | Firmware → ROS2 |
| `/rt/sensors/range_tof` | `/sensors/range_tof` | Firmware → ROS2 |
| `/rt/sensors/range_ultrasonic` | `/sensors/range_ultrasonic` | Firmware → ROS2 |
| `/rt/sensors/illuminance` | `/sensors/illuminance` | Firmware → ROS2 |
| `/rt/cmd_vel` | `/cmd_vel` | ROS2 → Firmware |

## Migration Steps

### Step 1: Update Firmware

**IMPORTANT**: The firmware must be updated first. Old firmware will not work with the new system.

1. Navigate to firmware directory:
   ```bash
   cd firmware
   ```

2. Build the updated firmware:
   ```bash
   make build_release
   ```

3. Flash to Pico (put Pico in BOOTSEL mode):
   ```bash
   make flash-release
   ```

4. Verify firmware is running:
   ```bash
   ./monitor_firmware.sh
   ```

### Step 2: Rebuild ROS2 Packages

1. Clean previous builds:
   ```bash
   rm -rf build/ install/ log/
   ```

2. Build updated packages:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select robot_hardware_interfaces robot_bringup robot_controller
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Step 3: Update Launch Commands

No changes required to launch commands. The system uses the same launch files with updated internal configurations.

```bash
# Start micro-ROS agent (auto-detects Pico)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# Or use the Python launcher script
python3 scripts/launch_microros_agent.py

# Launch robot bringup
ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl
```

### Step 4: Verify Topics

After launching the system, verify the new topic names:

```bash
# List all topics
ros2 topic list

# Expected topics (no /ddd/ prefix, no /rt/ prefix):
# /joint_states
# /imu/data_raw
# /odom
# /sensors/range_tof
# /sensors/range_ultrasonic
# /sensors/illuminance
# /cmd_vel
```

### Step 5: Test Functionality

1. **Test joint states**:
   ```bash
   ros2 topic echo /joint_states
   # Should show 4 joints with position and velocity data
   ```

2. **Test IMU data**:
   ```bash
   ros2 topic echo /imu/data_raw
   # Should show orientation, angular_velocity, linear_acceleration
   ```

3. **Test velocity commands**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
   # Robot should move forward
   ```

4. **Test with teleop**:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   # Should control robot with keyboard
   ```

## Troubleshooting

### Issue: No topics appearing

**Symptoms**: `ros2 topic list` shows no robot topics

**Solutions**:
1. Verify micro-ROS agent is running:
   ```bash
   ps aux | grep micro_ros_agent
   ```

2. Check Pico connection:
   ```bash
   ls -l /dev/ttyACM*
   ./scripts/check_pico.sh
   ```

3. Verify firmware is flashed:
   ```bash
   cd firmware && ./monitor_firmware.sh
   # Should see micro-ROS initialization messages
   ```

### Issue: Topics have /rt/ prefix

**Symptoms**: Topics appear as `/rt/joint_states`, `/rt/imu/data_raw`, etc.

**Solutions**:
1. Verify micro-ROS agent launch file has remappings:
   ```bash
   cat src/robot_bringup/launch/microros_agent.launch.py | grep remappings
   ```

2. Restart micro-ROS agent with correct launch file:
   ```bash
   ros2 launch robot_bringup microros_agent.launch.py
   ```

### Issue: Hardware interface fails to activate

**Symptoms**: Error message "Timeout waiting for joint states from firmware"

**Solutions**:
1. Verify firmware is publishing `/joint_states`:
   ```bash
   ros2 topic echo /joint_states --once
   ```

2. Check micro-ROS agent logs for connection issues

3. Increase connection timeout in URDF if needed:
   ```xml
   <param name="connection_timeout_ms">10000</param>
   ```

### Issue: Robot doesn't respond to cmd_vel

**Symptoms**: Publishing to `/cmd_vel` has no effect

**Solutions**:
1. Verify firmware is receiving commands:
   ```bash
   ros2 topic echo /rt/cmd_vel
   # Should show commands when you publish to /cmd_vel
   ```

2. Check hardware interface is publishing:
   ```bash
   ros2 topic info /cmd_vel
   # Should show hardware interface as publisher
   ```

3. Verify controller is running:
   ```bash
   ros2 control list_controllers
   # mecanum_drive_controller should be "active"
   ```

### Issue: "Mock mode" warnings in logs

**Symptoms**: Logs show "mock mode enabled" or similar warnings

**Solutions**:
This should not happen with the new implementation. If you see these warnings:
1. Verify you have the latest code:
   ```bash
   git pull
   colcon build --packages-select robot_hardware_interfaces
   ```

2. Check that firmware is actually publishing data:
   ```bash
   ros2 topic hz /joint_states
   # Should show ~100 Hz
   ```

### Issue: IMU data not available

**Symptoms**: `/imu/data_raw` topic not publishing

**Solutions**:
1. Verify IMU is connected and firmware detects it:
   ```bash
   cd firmware && ./monitor_firmware.sh
   # Look for IMU initialization messages
   ```

2. Check I2C connection (if IMU is on I2C):
   ```bash
   sudo i2cdetect -y 1
   # Should show device at expected address
   ```

3. Verify micro-ROS agent remapping:
   ```bash
   ros2 topic list | grep imu
   # Should show /imu/data_raw (not /rt/imu/data_raw or /ddd/imu)
   ```

## Compatibility Notes

### Backward Compatibility

**There is NO backward compatibility.** The old firmware and new ROS2 packages are incompatible. You must update both together.

### External Tools

The new topic names are compatible with standard ROS2 tools:

- **Nav2**: Works with `/odom`, `/cmd_vel`, `/imu/data_raw`
- **RViz2**: Can visualize all standard topics
- **teleop_twist_keyboard**: Works directly with `/cmd_vel`
- **robot_localization**: Works with `/odom` and `/imu/data_raw`
- **Foxglove**: Can visualize all standard message types

### Custom Scripts

If you have custom scripts that subscribe to old topic names, update them:

```python
# OLD
self.imu_sub = self.create_subscription(Imu, '/ddd/imu', ...)
self.odom_sub = self.create_subscription(Odometry, '/ddd/odom', ...)

# NEW
self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', ...)
self.odom_sub = self.create_subscription(Odometry, '/odom', ...)
```

## Benefits of Migration

### 1. Standard ROS2 Compatibility

- Works seamlessly with Nav2, MoveIt, and other ROS2 packages
- No custom remapping needed for standard tools
- Follows REP-105 (Coordinate Frames for Mobile Platforms)

### 2. Improved Maintainability

- Clear, semantic topic names
- Standard message types (Twist instead of Float32MultiArray)
- Reduced custom code and configuration

### 3. Better Debugging

- Standard tools (rqt, RViz) work out of the box
- Topic names clearly indicate their purpose
- No confusion about `/ddd/` prefix meaning

### 4. Real Hardware Feedback

- No mock mode fallback ensures system operates with real data
- Faster detection of hardware connection issues
- More reliable operation

## Additional Resources

- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [ros2_control Documentation](https://control.ros.org/)
- [micro-ROS Documentation](https://micro.ros.org/)

## Support

If you encounter issues not covered in this guide:

1. Check the firmware logs: `cd firmware && ./monitor_firmware.sh`
2. Check ROS2 logs: `ros2 topic list`, `ros2 topic echo <topic>`
3. Review hardware interface logs in controller manager output
4. Open an issue on the project repository with:
   - Output of `ros2 topic list`
   - Firmware monitor output
   - Controller manager logs
   - Steps to reproduce the issue
