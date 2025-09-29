# Technology Stack

## Core Framework
- **ROS2 Humble**: Primary robotics framework
- **ros2_control**: Hardware abstraction and controller management
- **colcon**: Build system for ROS2 packages
- **Just**: Command runner for development workflows

## Programming Languages
- **C++17**: Firmware and hardware interfaces
- **C11**: Low-level firmware components
- **Python3**: High-level nodes and utilities
- **CMake**: Build configuration

## Hardware Platform
- **SBC**: Raspberry Pi 4B/5 (Ubuntu 22.04/Raspberry Pi OS)
- **MCU**: Raspberry Pi Pico (RP2040)
- **Communication**: USB Serial, micro-ROS protocol

## Key Libraries & Dependencies

### Firmware (Pico)
- **Pico SDK**: Hardware abstraction for RP2040
- **FreeRTOS**: Real-time operating system
- **micro-ROS**: ROS2 communication over serial/USB
- **Eigen**: Linear algebra library

### ROS2 Packages
- **ros2_controllers**: Standard controllers (mecanum_drive_controller)
- **robot_state_publisher**: URDF/TF management
- **controller_manager**: Controller lifecycle management
- **gazebo_ros2_control**: Simulation support

## Common Commands

### Build & Development
```bash
# Build entire workspace
just build

# Build only hardware packages
just build-hardware

# Clean build artifacts
just clean

# Run tests
just run-tests

# Open development shell
just shell
```

### Simulation & Testing
```bash
# Start Gazebo simulation
just start-gazebo-sim

# Launch tmux simulation session
just start-sim-tmux

# Run keyboard teleoperation
just run-teleop
```

### Configuration
```bash
# Check current target configuration
just check-target

# Copy environment template
cp .env.example .env
```

### Firmware (from firmware/ directory)
```bash
# Build firmware
make

# Build release version
make -C build_release

# Flash to Pico (requires picotool)
# Follow firmware/README.md for detailed instructions
```

## Build System Details
- **colcon**: Uses `--symlink-install` for faster development
- **merge-install**: Optional layout for deployment
- **Event handlers**: `console_direct+` for immediate output
- **Dependencies**: Managed via `package.xml` and `rosdep`