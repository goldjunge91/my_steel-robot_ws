# Technology Stack
## Code Implementation Rules

- **Always write code directly to the correct files** - Never create dummy or pseudo code without permission
- **Write production-ready code** - All code should be complete, functional, and ready to use
- **No placeholder implementations** - Avoid stub functions, TODOs, or incomplete logic unless explicitly requested
- **Never create any *.md files, summaries, documentation, analysis or other additional artifacts** (such as README.md summaries, documentation, analysis .md files, project overviews, or automatic reports) without explicit request 
- **Always wait for direct instruction** from the user before generating or adding such content

> **Note**: See `coding-conventions.md` for important rules about code implementation, investigation, and making changes.

## Core Framework

- **ROS2 Humble** - Primary robotics middleware
- **Ubuntu 22.04** - Target OS for SBC and development

## Build System

- **colcon** - ROS2 workspace build tool
- **CMake 3.13+** - C++ package builds
- **ament_cmake** - ROS2 CMake extensions
- **ament_python** - Python package builds

## Firmware

- **Raspberry Pi Pico SDK** - Microcontroller framework
- **FreeRTOS** - Real-time operating system
- **micro-ROS** - ROS2 client library for microcontrollers
- **Eigen** - Linear algebra for odometry calculations
- **CMake** - Firmware build system

## Key Libraries & Dependencies

### Control & Hardware
- `ros2_control` - Hardware abstraction framework
- `ros2_controllers` - Standard controller implementations (mecanum_drive_controller, diff_drive_controller)
- `controller_manager` - Controller lifecycle management
- `hardware_interface` - Hardware plugin interface

### Simulation
- `gazebo_ros2_control` - Gazebo integration
- `gazebo_ros` - ROS2-Gazebo bridge

### Navigation & Localization
- `nav2` - Navigation stack
- `robot_localization` - Sensor fusion (EKF)
- `cartographer` - SLAM implementation

### Utilities
- `xacro` - URDF macro language
- `robot_state_publisher` - TF tree publishing
- `joint_state_publisher` - Joint state management
- `micro_ros_agent` - Bridge between micro-ROS and ROS2

## Code Style & Linting

### C++
- **Standard**: C++17
- **Formatter**: clang-format (Google style base, 4-space indent, 100 char line limit)
- **Linters**: ament_cppcheck, ament_cpplint

### Python
- **Standard**: Python 3
- **Linters**: flake8 (120 char line limit), ament_flake8, ament_pep257
- **Test**: pytest

## Common Commands

### Workspace Build
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build entire workspace
colcon build --symlink-install

# Build specific packages
colcon build --packages-select <package_name>

# Build with dependencies
colcon build --packages-up-to <package_name>

# Source workspace
source install/setup.bash
```

### Testing
```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# Show test results
colcon test-result --all --verbose
```

### Firmware Build & Flash
```bash
# Build firmware (debug)
cd firmware && make build

# Build firmware (release)
cd firmware && make build_release

# Flash to Pico (BOOTSEL mode)
cd firmware && make flash

# Monitor debug output
./firmware/monitor_firmware.sh
```

### Using Just (Task Runner)
```bash
# List all available commands
just

# Build workspace
just build

# Build firmware
just build-firmware

# Flash firmware
just flash-firmware

# Start simulation
just start-gazebo-sim

# Run teleoperation
just run-teleop

# Start micro-ROS agent
just start-microros

# Clean build artifacts
just clean
```

### ROS2 Runtime
```bash
# Launch robot bringup
ros2 launch robot_bringup bringup.launch.py

# Launch with parameters
ros2 launch robot_bringup bringup.launch.py \
  robot_model:=robot_xl \
  drive_type:=mecanum \
  microros:=true \
  serial_port:=/dev/ttyACM0

# Start micro-ROS agent manually
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v

# Teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/mecanum_cont/cmd_vel_unstamped

# List topics
ros2 topic list

# Echo topic
ros2 topic echo /odom

# List controllers
ros2 control list_controllers

# Spawn controller
ros2 run controller_manager spawner <controller_name>
```

### Development Tools
```bash
# Format C++ code
./format.sh

# Check Python style
flake8 .

# Run RViz
rviz2

# Run RViz with config
rviz2 -d src/robot/config/view_robot.rviz

# Check joystick
ros2 run joy joy_enumerate_devices
jstest-gtk
```

## Environment Configuration

Use `.env` file to configure target platform:
- `TARGET=robot` - Raspberry Pi SBC deployment
- `TARGET=remote_pc` - Development workstation with simulation

## Communication Protocols

- **micro-ROS**: USB serial (115200 baud) between Pico and SBC
- **ROS2 DDS**: Inter-process communication on SBC
- **Rosbridge WebSocket**: Web dashboard connectivity (port 9090)
- **Web Video Server**: Camera streaming (port 8080)
