# Technology Stack

> **TODO: This steering document needs approval before being considered final**

## Core Technologies
- **ROS2 Humble** - Primary robotics framework
- **C++17/C11** - System-level programming for hardware interfaces and firmware
- **Python 3** - High-level nodes, scripts, and utilities
- **CMake** - Build system for C++ components and firmware
- **Colcon** - ROS2 workspace build tool

## Hardware Platforms
- **Raspberry Pi 4B (8GB)** - Main compute running Ubuntu 22.04/ROS2
- **Raspberry Pi Pico** - Real-time microcontroller for motor control, Nerf launcher, IMU, and ToF sensor
- **micro-ROS** - Bridge between Pico firmware and ROS2 ecosystem

## Key ROS2 Packages & Dependencies
- **ros2_control** - Hardware abstraction and controller framework
- **ros2_controllers** - Standard controllers (mecanum_drive_controller, diff_drive_controller)
- **robot_localization** - Sensor fusion and EKF for odometry
- **Nav2** - Navigation stack for autonomous movement
- **Gazebo** - Physics simulation for development and testing
- **MoveIt** - Motion planning for manipulator control

## Build System & Automation
- **Colcon** - Primary ROS2 workspace build tool
- **vcs** - Version control system tool for multi-repo management  
- **rosdep** - ROS dependency management
- **Just** - Task runner for common development workflows (currently being updated)
- **CMake** - Build system for firmware and C++ components

## Common Commands

### Workspace Setup
```bash
# Setup environment and install dependencies
./setup.sh

# Import all repositories
vcs import src < src/ros2.repos

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -y --rosdistro=humble

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Development Workflow
```bash
# Source ROS and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Clean build
rm -rf build install log
colcon build --symlink-install

# Build specific packages
colcon build --packages-select robot_hardware robot_bringup

# Run tests
colcon test --packages-select robot_hardware

# Launch robot system
ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl
```

### Firmware Development
```bash
# Build Pico firmware (from firmware directory)
cd firmware
make build

# Flash to Pico (requires BOOTSEL mode)
make flash

# Monitor firmware output
./monitor_firmware.sh

# Test micro-ROS connection
cd ../scripts
./test_firmware_connection.sh
```


### Target Configuration
Configure your deployment target in `.env` file:
```bash
# Copy example configuration
cp .env.example .env

# Edit configuration
TARGET=robot              # For SBC deployment
# TARGET=remote_pc         # For development/simulation

# Robot model configuration
ROBOT_MODEL_NAME=robot_xl  # Primary robot model
DRIVE_TYPE=mecanum         # Drive system type
ROS_DISTRO=humble         # ROS2 distribution
``` 


```bash
# Copy example configuration
cp .env.example .env

# Edit configuration
# TARGET=robot              # For SBC deployment
TARGET=remote_pc         # For development/simulation

# Robot model configuration
ROBOT_MODEL_NAME=robot_xl  # Primary robot model
DRIVE_TYPE=mecanum         # Drive system type
ROS_DISTRO=humble         # ROS2 distribution
``` 

## Development Environment
- **Docker/DevContainer** support available for consistent development
- **VS Code** configuration included with ROS2 extensions
- **Environment variables** via `.env` file for target-specific configuration
- **Cross-platform support** for Windows (WSL2), Linux, and macOS development
- **Remote development** capabilities for working with SBC over SSH

## Firmware Stack
- **Pico SDK** - Raspberry Pi Pico development framework
- **FreeRTOS** - Real-time operating system for Pico
- **micro-ROS** - ROS2 client library for microcontrollers
- **Eigen** - Linear algebra library for sensor processing
- **Custom libraries** - VL53L0X ToF sensor, motor control, servo control

## Computer Vision Stack
- **OpenCV** - Computer vision library for image processing
- **YOLOv5** - Real-time object detection for face recognition
- **USB Video Class (UVC)** - Camera interface for 1080p video capture

## Communication Protocols
- **Serial/UART** - Primary communication between Pi 4B and Pico
- **I2C** - IMU and sensor communication on Pico
- **PWM** - Motor control signals and servo control
- **USB** - Camera interface and development/debugging