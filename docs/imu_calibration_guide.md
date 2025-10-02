# IMU Calibration Guide - ICM-20948

This guide describes the calibration workflow for the ICM-20948 IMU sensor on the robot.

## Overview

The ICM-20948 IMU requires calibration for:
1. **Gyroscope bias** - Zero-rate offset
2. **Accelerometer** - Scale factors, offsets, misalignment
3. **Magnetometer** - Hard/soft iron distortion (if used)
4. **Noise covariance** - For sensor fusion (robot_localization, etc.)

## Quick Start: Noise Covariance Estimation

**Required:** Robot must be **completely stationary** on flat surface.

### Step 1: Run noise analyzer

```bash
cd /home/marco/ros2_steel_ws/my_steel-robot_ws/firmware/scripts
chmod +x analyze_imu_noise.py

# Make sure micro-ROS agent is running and IMU is publishing
ros2 topic hz /ddd/imu  # Should show ~10 Hz

# Run analyzer (collects 1000 samples = ~100 seconds)
python3 analyze_imu_noise.py
```

### Step 2: Update firmware

The script will output C++ code like:
```cpp
imu_msg_.linear_acceleration_covariance[0] = 0.045;  // X
imu_msg_.linear_acceleration_covariance[4] = 0.038;  // Y
imu_msg_.linear_acceleration_covariance[8] = 0.052;  // Z

imu_msg_.angular_velocity_covariance[0] = 0.00015;  // X
imu_msg_.angular_velocity_covariance[4] = 0.00018;  // Y
imu_msg_.angular_velocity_covariance[8] = 0.00012;  // Z
```

Copy these values to `firmware/src/application/ImuAgent.cpp` constructor.

### Step 3: Rebuild and flash

```bash
cd /home/marco/ros2_steel_ws/my_steel-robot_ws/firmware/build
make -j
# Flash to device
```

---

## Advanced Calibration: Full 6-DOF Calibration

For precise accelerometer/gyro calibration using sphere-fitting method.

### Prerequisites

```bash
# Install imu_tools (ROS2 IMU calibration package)
sudo apt install ros-humble-imu-tools

# Or use imu_tk (more advanced, C++ implementation)
cd ~/
git clone https://github.com/Kyle-ak/imu_tk.git
cd imu_tk
mkdir build && cd build
cmake ..
make
```

### Method 1: Using imu_tk (Recommended)

**Reference:** https://github.com/Kyle-ak/imu_tk

This method uses Ceres solver for precise calibration including:
- Accelerometer scale factors, biases, axis misalignment
- Gyroscope scale factors, biases, axis misalignment

#### Step 1: Collect calibration data

```bash
# Record rosbag while rotating IMU in ALL orientations
# Duration: 2-3 minutes
# Must cover all possible attitudes (sphere surface)
ros2 bag record -o imu_calibration /ddd/imu
```

**Important rotation sequence:**
1. Flat on table (Z up)
2. On each side (6 faces of cube)
3. Slow continuous rotation around X axis
4. Slow continuous rotation around Y axis
5. Slow continuous rotation around Z axis
6. Random orientations to fill sphere

#### Step 2: Convert rosbag to imu_tk format

```python
#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

# Read rosbag and extract IMU data
storage_options = rosbag2_py.StorageOptions(uri='imu_calibration', storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

with open('imu_raw_data.txt', 'w') as f:
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == '/ddd/imu':
            msg = deserialize_message(data, Imu)
            # Format: timestamp accel_x accel_y accel_z gyro_x gyro_y gyro_z
            f.write(f"{timestamp} "
                   f"{msg.linear_acceleration.x} {msg.linear_acceleration.y} {msg.linear_acceleration.z} "
                   f"{msg.angular_velocity.x} {msg.angular_velocity.y} {msg.angular_velocity.z}\n")
```

#### Step 3: Run imu_tk calibration

```bash
cd ~/imu_tk/build
./bin/test_imu_calib ../data/xsens_imu.mat  # Replace with your data file
```

Output will provide calibration matrices that can be applied in firmware.

### Method 2: Using ROS imu_calib (Simpler)

**Reference:** http://wiki.ros.org/imu_calib

```bash
# Install package
sudo apt install ros-humble-imu-calib

# Run calibration node
ros2 launch imu_calib do_calib.launch.py

# Follow on-screen instructions for 6-point calibration
```

---

## Gyroscope Bias Calibration

**Current implementation:** Automatic at startup (in `ensureInitialized()`)

**Problem:** Temperature drift during operation causes bias to change.

### Online Bias Estimation

Add to `ImuAgent::run()`:

```cpp
// When robot is stationary (based on cmd_vel or IMU variance)
if (robot_stationary && sample_count % 1000 == 0) {
    // Update gyro bias estimate
    gyro_bias_x_ = gyro_bias_x_ * 0.9 + gyro.x * 0.1;  // Low-pass filter
    gyro_bias_y_ = gyro_bias_y_ * 0.9 + gyro.y * 0.1;
    gyro_bias_z_ = gyro_bias_z_ * 0.9 + gyro.z * 0.1;
}

// Apply bias correction before publishing
imu_msg_.angular_velocity.x = (gyro.x - gyro_bias_x_) * kDegToRad;
imu_msg_.angular_velocity.y = (gyro.y - gyro_bias_y_) * kDegToRad;
imu_msg_.angular_velocity.z = (gyro.z - gyro_bias_z_) * kDegToRad;
```

---

## Magnetometer Calibration (Future)

The ICM-20948 includes AK09916 magnetometer for full 9-DOF.

### Hard Iron Calibration (Offset)

Caused by permanent magnets near sensor.

```bash
# Collect data while rotating IMU in figure-8 pattern
ros2 bag record -o mag_calibration /ddd/magnetometer

# Process with magneto calibration tool
# https://github.com/kriswiner/MPU9250/wiki/Simple-and-Effective-Magnetometer-Calibration
```

### Soft Iron Calibration (Scale factors)

Caused by ferromagnetic materials distorting Earth's magnetic field.

Requires full sphere fitting with cross-axis terms (more complex).

---

## Integration with robot_localization

Once covariances are calibrated, configure `robot_localization` EKF:

```yaml
# config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    imu0: /ddd/imu
    imu0_config: [false, false, false,  # x, y, z position
                  true,  true,  true,   # roll, pitch, yaw
                  false, false, false,  # x_dot, y_dot, z_dot
                  true,  true,  true,   # roll_dot, pitch_dot, yaw_dot
                  true,  true,  true]   # x_ddot, y_ddot, z_ddot
    
    # Use calibrated covariances from IMU message
    imu0_differential: false
    imu0_relative: false
```

---

## Validation

After calibration, verify results:

```bash
# 1. Check noise levels (should match covariance)
ros2 topic echo /ddd/imu

# 2. Verify gravity norm when stationary (should be 9.81 m/s²)
python3 -c "
import rclpy
from sensor_msgs.msg import Imu
import numpy as np

def cb(msg):
    acc = np.array([msg.linear_acceleration.x, 
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z])
    norm = np.linalg.norm(acc)
    print(f'Gravity norm: {norm:.4f} m/s² (expected 9.81)')

rclpy.init()
node = rclpy.create_node('test')
node.create_subscription(Imu, '/ddd/imu', cb, 10)
rclpy.spin(node)
"

# 3. Check gyro bias (should be near zero when stationary)
ros2 topic echo /ddd/imu | grep angular_velocity -A3
```

---

## References

- [Least Squares Sphere Fit](https://jekel.me/2015/Least-Squares-Sphere-Fit/)
- [imu_tk - Inertial Measurement Unit Toolkit](https://github.com/Kyle-ak/imu_tk)
- [ICM-20948 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
- [STMicroelectronics AN4508 - Accelerometer Calibration](https://www.st.com/resource/en/application_note/dm00119044.pdf)
- [Magnetometer Calibration Guide](https://github.com/kriswiner/MPU9250/wiki/Simple-and-Effective-Magnetometer-Calibration)
