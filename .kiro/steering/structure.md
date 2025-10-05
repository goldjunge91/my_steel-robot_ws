# Project Structure & Organization

> **TODO: This steering document needs approval before being considered final**

## Workspace Layout
```
my_steel-robot_ws/
├── src/                     # ROS2 packages (imported via vcs)
├── firmware/                # Raspberry Pi Pico firmware
├── docs/                    # Documentation and guides
├── scripts/                 # Utility scripts and tools
├── config/                  # Global configuration files
├── lib/                     # External libraries (FreeRTOS, Eigen, etc.)
├── .kiro/steering/          # AI assistant guidance documents
├── Justfile                 # Task automation (being updated)
├── setup.sh                 # Environment setup script
├── .env                     # Environment configuration
└── .env.example             # Environment template
```

## Package Architecture

### Core ROS2 Packages (src/)
- **robot_description/** - URDF/XACRO models, meshes, component configurations
- **robot_bringup/** - Launch files and system orchestration
- **robot_hardware/** - ros2_control SystemInterface for hardware abstraction
- **robot_controllers/** - Controller configurations (mecanum/differential drive)
- **robot_gazebo/** - Simulation assets and launch files

### Specialized Packages
- **robot_localization/** - Sensor fusion and EKF configuration for odometry
- **robot_vision/** - Computer vision nodes (face detection, tracking with OpenCV/YOLOv5)
- **robot_nerf_launcher/** - High-level Nerf launcher control and safety systems
- **robot_autonomy/** - Nav2 configuration and autonomous navigation behaviors
- **robot_utils/** - Development tools, scripts, and utilities

### Hardware Integration
- **mecabridge_hardware/** - Hardware interface bridge for ros2_control
- **robot_hardware_interfaces/** - Custom hardware interface definitions
- **robot-micro-ROS-Agent/** - micro-ROS communication bridge for Pico
- **open_manipulator_x/** - Manipulator arm integration (if applicable)

### External Dependencies (lib/)
- **eigen/** - Linear algebra library for sensor processing
- **FreeRTOS-Kernel/** - Real-time operating system for Pico
- **micro_ros_raspberrypi_pico_sdk/** - micro-ROS SDK for Pico
- **pico-distance-sensor/** - VL53L0X ToF sensor library

## Key Configuration Patterns

### Controller Configuration
```
robot_controllers/config/
├── my_steel/
│   ├── mecanum_drive_controller.yaml
│   └── diff_drive_controller.yaml
└── robot_xl/
    ├── mecanum_drive_controller.yaml
    └── diff_drive_controller.yaml
```

### Launch File Hierarchy
```
robot_bringup/launch/
├── bringup.launch.py        # Main system launch
├── microros.launch.py       # micro-ROS agent
└── simulation.launch.py     # Gazebo simulation
```

### Hardware Description
```
robot_description/
├── urdf/
│   ├── my_steel.urdf.xacro  # Main robot model
│   └── components/          # Modular components
├── meshes/                  # 3D models
└── config/
    └── components_config/   # Hardware-specific configs
```

## Naming Conventions

### Joint Names
- Wheel joints: `front_left_wheel_joint`, `front_right_wheel_joint`, etc.
- Must match between URDF, controller YAML, and hardware interface

### Topic Naming
- Controllers publish to: `/mecanum_cont/cmd_vel_unstamped`
- Hardware feedback: `/joint_states`, `/odom`
- Sensors: `/scan` (LiDAR), `/imu/data_raw` (IMU)
- Custom: `/battery/status`, `/nerf/fire_cmd`

### Package Naming
- Prefix with `robot_` for core functionality
- Use underscores for multi-word names
- Keep names descriptive but concise

## File Organization Rules

### Documentation
- **docs/PINMAP.md** - Authoritative hardware pin mapping
- **README.md** files in each package root
- Architecture docs in **docs/** directory

### Configuration Files
- Environment-specific configs in package **config/** directories
- Global workspace settings in root **config/** directory
- Use YAML for ROS2 parameters and launch configurations
- Environment variables in **.env** file for deployment targets

### Scripts and Tools
- Development scripts in **scripts/** directory
- Robot-specific tools in respective **robot_utils/**
- Executable scripts should have proper shebang and permissions

## Multi-Repository Management
- **src/ros2.repos** - VCS import configuration for all packages
- Each package maintained in separate Git repository under goldjunge91 organization
- Use `vcs import src < src/ros2.repos` to populate workspace
- Submodules in **lib/** directory for external dependencies
