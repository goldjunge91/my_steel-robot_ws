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
- `ros2_controllers` - Standard controller implementations (diff_drive_controller)
- `mecanum_drive_controller` - Custom mecanum drive controller (in robot_controllers package)
- `controller_manager` - Controller lifecycle management
- `hardware_interface` - Hardware plugin interface
- `imu_sensor_broadcaster` - IMU data broadcasting
- `joint_state_broadcaster` - Joint state broadcasting

### Simulation
- `gazebo_ros2_control` - Gazebo integration
- `gazebo_ros` - ROS2-Gazebo bridge

### Navigation & Localization
- `nav2` - Navigation stack
- `robot_localization` - Sensor fusion (EKF)
- `cartographer` - SLAM implementation (planned)

### Utilities
- `xacro` - URDF macro language
- `robot_state_publisher` - TF tree publishing
- `joint_state_publisher` - Joint state management
- `micro_ros_agent` - Bridge between micro-ROS and ROS2
- `teleop_twist_keyboard` - Keyboard teleoperation
- `joy` - Joystick interface

### External Components
- `husarion_components_description` - Component descriptions
- `dynamixel_hardware_interface` - Dynamixel servo interface
- `open_manipulator_x` - Manipulator arm support
- `foxglove-joystick` - Foxglove joystick integration

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

# Flash to Pico (BOOTSEL mode or picotool)
cd firmware && make flash

# Flash release firmware
cd firmware && make flash-release

# Monitor debug output
./firmware/monitor_firmware.sh

# Test firmware topics
./firmware/test_firmware_topics.sh

# Create release
cd firmware && ./create_release.sh
```

### Using Just (Task Runner)
```bash
# List all available commands
just

# Build workspace
just build

# Build only hardware packages
just build-hardware

# Run tests
just run-tests

# Clean build artifacts
just clean

# Build firmware (debug)
just build-firmware

# Build firmware (release)
just build-firmware-release

# Flash firmware (debug)
just flash-firmware

# Flash firmware (release)
just flash-firmware-release

# Monitor firmware output
just monitor-firmware

# Test firmware connection
just test-firmware

# Start simulation
just start-gazebo-sim

# Start simulation with tmux
just start-sim-tmux

# Run teleoperation
just run-teleop

# Start micro-ROS agent (auto-detect)
just start-microros

# Start micro-ROS agent with specific device
just start-microros-dev /dev/ttyACM0

# Check target configuration
just check-target

# Open shell with ROS sourced
just shell
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

# Launch simulation
ros2 launch robot launch_sim.launch.py use_sim_time:=true

# Launch robot on Raspberry Pi
ros2 launch robot launch_robot_pi.launch.py

# Launch joystick control
ros2 launch robot joystick_xbox_mecanum_pico.launch.py

# Start micro-ROS agent manually
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v

# Start micro-ROS agent with Python script (auto-detect)
python3 scripts/launch_microros_agent.py

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

# Check controller status
./scripts/check_and_spawn_controllers.sh
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

# Run RViz clean (without previous state)
./scripts/run_rviz_clean.sh

# Check joystick
ros2 run joy joy_enumerate_devices
jstest-gtk

# Check Pico connection
./scripts/check_pico.sh

# Check micro-ROS and Pico USB
./scripts/check_micro_ros_and_pico_usb0.sh

# Check system status
./scripts/check_status.sh

# Diagnose agent issues
./scripts/diagnose_agent_issues.sh

# Test firmware connection
./scripts/test_firmware_connection.sh

# Proper test procedure
./scripts/proper_test_procedure.sh

# ROS2 verification checks
./scripts/ros2_verification_checks.sh
```

## Environment Configuration

Use `.env` file to configure target platform (see `.env.*.example` for templates):
- `TARGET=robot` - Raspberry Pi SBC deployment
- `TARGET=remote_pc` - Development workstation with simulation

Key environment variables:
- `ROS_DISTRO=humble` - ROS2 distribution
- `ROBOT_MODEL_NAME=robot_xl` - Robot model (only robot_xl supported)
- `DRIVE_TYPE=mecanum` - Drive type (mecanum or diff)
- `SERIAL_PORT=/dev/ttyACM0` - Pico serial port
- `SERIAL_BAUDRATE=115200` - Serial communication baud rate
- `MICROROS=true` - Enable micro-ROS agent
- `PICO_SDK_PATH` - Path to Pico SDK (for firmware builds)

## Communication Protocols

- **micro-ROS**: USB serial (115200 baud default) between Pico and SBC via `/dev/ttyACM0`
- **ROS2 DDS**: Inter-process communication on SBC
- **Rosbridge WebSocket**: Web dashboard connectivity (port 9090, planned)
- **Web Video Server**: Camera streaming (port 8080, planned)

## Hardware Components

### Sensors
- **ICM20948** - 9-DOF IMU (SPI0 on Pico)
- **VL6180X** - Time-of-Flight distance sensor (I2C1 on Pico)
- **RPLiDAR** - 2D LiDAR scanner
- **Camera** - USB camera for vision

### Actuators
- **4x DC Motors** - Mecanum wheels with encoders
- **Motor Drivers** - TB6612 or similar H-bridge drivers
- **Servos** - Dynamixel servos for manipulator (optional)

### Controllers
- **Raspberry Pi 4B** - Main SBC running ROS2
- **Raspberry Pi Pico** - Real-time motor control and sensor interfacing
- **Arduino Nano** - Nerf launcher controller (separate)
