# Project Structure

## Workspace Organization
The project follows ROS2 workspace conventions with clear separation between firmware, ROS2 packages, and utilities.

## Root Directory Layout
```
my_steel-robot_ws/
├── src/                    # ROS2 packages (vcs import target)
├── firmware/               # Raspberry Pi Pico firmware
├── lib/                    # External libraries (submodules)
├── docs/                   # Documentation and guides
├── scripts/                # Utility scripts and tools
├── build/                  # Build artifacts (generated)
├── install/                # Installation artifacts (generated)
├── log/                    # Build logs (generated)
├── Justfile               # Command runner recipes
├── .env.example           # Environment configuration template
└── README.md              # Main project documentation
```

## Core ROS2 Packages (src/)

### Hardware & Control
- **`mecabridge_hardware/`**: ros2_control hardware interface for Pico communication
- **`robot_controller/`**: High-level robot control logic
- **`robot_utils/`**: Utility scripts, flashing tools, port discovery

### Robot Definition & Bringup
- **`robot/`**: Main robot package with launch files and configurations
- **`robot_description/`**: URDF/XACRO files, meshes, robot models
- **`robot_bringup/`**: System launch files and orchestration

### External Dependencies
- **`husarion_controllers/`**: Mecanum drive controller
- **`micro-ROS-Agent/`**: Communication bridge for micro-ROS
- **`open_manipulator_x/`**: Manipulator arm support
- **`rosbot_hardware_interfaces/`**: Additional hardware interfaces

## Firmware Structure (firmware/)
```
firmware/
├── src/                   # Main firmware source code
│   ├── hal/              # Hardware abstraction layer
│   ├── include/          # Header files organized by layer
│   ├── lib/              # External libraries (IMU drivers)
│   └── main.cpp          # Main firmware entry point
├── port/                 # FreeRTOS port configuration
├── tests/                # Unit and integration tests
├── build/                # Debug build artifacts
├── build_release/        # Release build artifacts
└── CMakeLists.txt        # Build configuration
```

## External Libraries (lib/)
- **`FreeRTOS-Kernel/`**: Real-time operating system
- **`eigen/`**: Linear algebra library
- **`micro_ros_raspberrypi_pico_sdk/`**: micro-ROS Pico integration
- **`pico-distance-sensor/`**: Distance sensor drivers

## Configuration Patterns

### Environment Configuration
- **`.env`**: Runtime configuration (copied from `.env.example`)
- **Target platforms**: `robot` (SBC) or `remote_pc` (development)
- **Drive mode**: `mecanum` (four-wheel omnidirectional)

### Package Structure Conventions
```
typical_package/
├── package.xml           # ROS2 package manifest
├── CMakeLists.txt        # Build configuration
├── launch/               # Launch files
├── config/               # YAML configurations
├── src/                  # Source code
├── include/              # Header files
└── test/                 # Unit tests
```

## Key Configuration Files
- **`docs/PINMAP.md`**: Authoritative pin mapping documentation
- **`robot_controllers/config/`**: Controller YAML configurations
- **`robot_description/urdf/`**: Robot model definitions
- **`firmware/port/FreeRTOS-Kernel/`**: RTOS configuration

## Build Artifacts
- **`build/`**: Intermediate build files (colcon)
- **`install/`**: Installed packages and binaries
- **`log/`**: Build and test logs
- **`firmware/build*/`**: Firmware build outputs (.uf2, .elf files)

## Documentation Structure
- **`docs/`**: Project-wide documentation
- **`docs/check/`**: Validation and testing guides
- **Package-level**: Each package contains its own README.md