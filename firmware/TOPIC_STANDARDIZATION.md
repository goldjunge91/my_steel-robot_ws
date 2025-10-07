# ROS2 Topic Standardization

This document describes the topic name standardization changes made to the firmware to comply with ROS2 REP-105 conventions.

## Overview

The firmware has been updated to use standard ROS2 topic names without robot-specific prefixes. This improves compatibility with standard ROS2 tools and packages like Nav2, RViz, and robot_localization.

## Changes Made

### 1. Odometry Topic (DDD.cpp)
- **Old:** `/ddd/odom`
- **New:** `/odom`
- **Message Type:** `nav_msgs/Odometry`
- **Frame IDs:** `odom` (parent), `base_link` (child)
- **Requirement:** 1.4

### 2. Velocity Command Topic (DDD.cpp)
- **Old:** `/ddd/cmd_vel` (with alternative subscription)
- **New:** `/cmd_vel` (single subscription)
- **Message Type:** `geometry_msgs/Twist`
- **Requirement:** 1.6

### 3. IMU Topic (ImuAgent.cpp)
- **Old:** `/ddd/imu`
- **New:** `/imu/data_raw`
- **Message Type:** `sensor_msgs/Imu`
- **Frame ID:** `imu_link`
- **Requirement:** 1.3

### 4. ToF Range Sensor Topic (vl6180xAgent.cpp)
- **Old:** `/ddd/range_tof`
- **New:** `/sensors/range_tof`
- **Message Type:** `sensor_msgs/Range`
- **Frame ID:** `tof_link`
- **Requirement:** 1.5

### 5. Illuminance Sensor Topic (vl6180xAgent.cpp)
- **Old:** `/ddd/illuminance`
- **New:** `/sensors/illuminance`
- **Message Type:** `sensor_msgs/Illuminance`
- **Frame ID:** `tof_link`
- **Requirement:** 1.5

### 6. Ultrasonic Range Sensor Topic (HCSR04Agent.cpp)
- **Old:** `/ddd/range`
- **New:** `/sensors/range_ultrasonic`
- **Message Type:** `sensor_msgs/Range`
- **Frame IDs:** `range_front`, `range_back`
- **Requirement:** 1.5

## Topic Naming Conventions

All topics now follow these conventions:

1. **No robot-specific prefixes** - Removed `/ddd/` prefix from all topics
2. **Standard ROS2 names** - Using REP-105 standard names where applicable
3. **Sensor namespace** - All sensor topics grouped under `/sensors/` namespace
4. **Descriptive names** - Sensor topics include sensor type (e.g., `range_tof`, `range_ultrasonic`)

## micro-ROS Agent Topic Mapping

The micro-ROS agent adds a `/rt/` prefix to all firmware topics. The complete topic mapping is:

| Firmware Topic | micro-ROS Topic | ROS2 Topic (after remapping) |
|----------------|-----------------|------------------------------|
| `odom` | `/rt/odom` | `/odom` |
| `cmd_vel` | `/rt/cmd_vel` | `/cmd_vel` |
| `imu/data_raw` | `/rt/imu/data_raw` | `/imu/data_raw` |
| `sensors/range_tof` | `/rt/sensors/range_tof` | `/sensors/range_tof` |
| `sensors/illuminance` | `/rt/sensors/illuminance` | `/sensors/illuminance` |
| `sensors/range_ultrasonic` | `/rt/sensors/range_ultrasonic` | `/sensors/range_ultrasonic` |
| `joint_states` | `/rt/joint_states` | `/joint_states` |

## Building and Flashing

### Build Firmware
```bash
cd firmware
make build
```

The firmware will be saved to `firmware/releases/my_firmware_latest.uf2`.

### Flash to Pico
1. Hold the BOOTSEL button on the Pico while connecting it via USB
2. The Pico will appear as a USB mass storage device
3. Copy the `.uf2` file to the Pico:
   ```bash
   make flash
   ```
   Or manually:
   ```bash
   cp releases/my_firmware_latest.uf2 /media/marco/RPI-RP2/
   ```

### Verify Firmware Boot
Monitor the serial output to verify the firmware boots correctly:
```bash
./monitor_firmware.sh
```

## Testing

### Automated Test Script
Run the automated test script to verify all topics are working correctly:

```bash
# Start micro-ROS agent first
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# In another terminal, run the test script
cd firmware
./test_firmware_topics.sh
```

The script will:
1. Check that all new standard topic names exist
2. Verify deprecated `/ddd/` topics are removed
3. Confirm topics are publishing data
4. Validate message types

### Manual Testing

#### List Topics
```bash
ros2 topic list
```

Expected topics (with `/rt/` prefix from micro-ROS agent):
- `/rt/odom`
- `/rt/cmd_vel`
- `/rt/imu/data_raw`
- `/rt/sensors/range_tof`
- `/rt/sensors/illuminance`
- `/rt/sensors/range_ultrasonic`
- `/rt/joint_states`

#### Echo Topics
```bash
# Odometry
ros2 topic echo /rt/odom

# IMU
ros2 topic echo /rt/imu/data_raw

# Joint States
ros2 topic echo /rt/joint_states

# Sensors
ros2 topic echo /rt/sensors/range_tof
ros2 topic echo /rt/sensors/illuminance
ros2 topic echo /rt/sensors/range_ultrasonic
```

#### Send Velocity Commands
```bash
# Move forward
ros2 topic pub /rt/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Stop
ros2 topic pub /rt/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## Unit Tests

Unit tests have been added to verify topic names are correct:

```bash
cd firmware
make test
```

The test suite includes:
- Topic name validation
- Deprecated prefix detection
- Frame ID verification
- Naming convention compliance

## Troubleshooting

### Topics not appearing
1. Verify the Pico is connected: `ls /dev/ttyACM*`
2. Check micro-ROS agent is running: `pgrep -f micro_ros_agent`
3. Monitor firmware serial output for errors: `./monitor_firmware.sh`

### Deprecated topics still visible
1. Ensure you flashed the latest firmware build
2. Power cycle the Pico (unplug and reconnect)
3. Restart the micro-ROS agent

### No data on topics
1. Check sensor connections (IMU, ToF, ultrasonic)
2. Review firmware logs for initialization errors
3. Verify sensor frame_ids in URDF match firmware

## References

- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [ROS2 Topic Naming Conventions](https://design.ros2.org/articles/topic_and_service_names.html)
- [micro-ROS Documentation](https://micro.ros.org/)

## Requirements Addressed

This implementation addresses the following requirements from the specification:

- **1.1** - Firmware publishes sensor data using standard ROS2 topic names
- **1.2** - Joint states published to `/joint_states`
- **1.3** - IMU data published to `/imu/data_raw`
- **1.4** - Odometry published to `/odom`
- **1.5** - Range sensors use descriptive names under `/sensors/` namespace
- **1.6** - Velocity commands subscribed from `/cmd_vel` only
- **1.7** - No robot-specific prefixes like `ddd` in topic names
