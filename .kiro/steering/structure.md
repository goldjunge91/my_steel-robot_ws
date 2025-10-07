# Project Structure
## Code Implementation Rules

- **Always write code directly to the correct files** - Never create dummy or pseudo code without permission
- **Write production-ready code** - All code should be complete, functional, and ready to use
- **No placeholder implementations** - Avoid stub functions, TODOs, or incomplete logic unless explicitly requested
- **Never create READMEs, summaries, documentation, or other additional artifacts** (such as README.md files, project overviews, or automatic reports) without explicit request
- **Always wait for direct instruction** from the user before generating or adding such content

> **Note**: See `coding-conventions.md` for important rules about code implementation, codebase investigation, and making changes.

## Workspace Organization

This is a ROS2 workspace following standard colcon conventions with additional firmware and documentation directories.

## Top-Level Layout

```
my_steel-robot_ws/
├── src/                    # ROS2 packages
├── firmware/               # Raspberry Pi Pico firmware (separate build)
├── lib/                    # External libraries (FreeRTOS, Pico SDK, Eigen)
├── docs/                   # Project documentation
├── scripts/                # Utility scripts
├── docker/                 # Docker configurations
├── build/                  # Build artifacts (gitignored)
├── install/                # Install artifacts (gitignored)
├── log/                    # Build/runtime logs (gitignored)
├── Justfile               # Task runner commands
└── .env                   # Environment configuration
```

## ROS2 Packages (src/)

### Core Robot Packages

- **robot/** - Main robot package with URDF, launch files, and configs
  - `description/` - URDF/xacro robot models
  - `launch/` - Launch files (simulation, robot state publisher)
  - `config/` - RViz configs, controller parameters
  - `worlds/` - Gazebo world files

- **robot_description/** - Robot URDF definitions and meshes
  - `urdf/` - Xacro files defining robot structure
  - `meshes/` - 3D models (STL/DAE)
  - `config/components_config/` - Component configurations

- **robot_bringup/** - Launch orchestration for hardware/simulation
  - `launch/` - Bringup launch files
  - `config/` - Runtime parameters

- **robot_controller/** - Controller configurations
  - `config/` - YAML configs for mecanum/diff drive controllers

- **robot_hardware_interfaces/** - ros2_control hardware interface
  - `src/` - C++ hardware interface plugin
  - `include/` - Header files
  - `config/` - Hardware parameters

### Functionality Packages

- **robot_gazebo/** - Gazebo simulation assets
- **robot_localization_tool/** - Sensor fusion (EKF) configuration
- **robot_vision/** - Computer vision nodes (face detection)
- **robot_nerf_launcher/** - Nerf launcher control logic
- **robot_utils/** - Utility scripts and tools

### External Dependencies

- **micro-ROS-Agent/** - Bridge between micro-ROS and ROS2
- **open_manipulator_x/** - Manipulator arm support (optional)

## Firmware Structure (firmware/)

```
firmware/
├── src/                    # Source code
│   ├── main.cpp           # Entry point, FreeRTOS setup
│   ├── Agent.h/cpp        # Base agent class
│   ├── uRosBridge.h/cpp   # micro-ROS communication singleton
│   ├── DDD.h/cpp          # Main robot control agent (odometry, cmd_vel)
│   ├── MotorsAgent.h/cpp  # Motor control with PID
│   ├── MotorPID.h/cpp     # PID controller implementation
│   ├── MotorMgr.h/cpp     # Motor hardware abstraction
│   ├── HCSR04Agent.h/cpp  # Ultrasonic sensor agent
│   ├── hal/               # Hardware abstraction layer
│   ├── application/       # Application-specific code
│   └── shared/            # Shared utilities
├── tests/                 # Unit tests (GoogleTest)
├── port/                  # FreeRTOS port configuration
├── releases/              # Compiled .uf2 files (gitignored)
├── CMakeLists.txt         # Build configuration
├── Makefile               # Build wrapper
└── README.md              # Firmware documentation
```

## Documentation (docs/)

- **PINMAP.md** - Authoritative pin mapping (single source of truth)
- **architecture_and_packages.md** - Package responsibilities
- **hardware_setup.md** - Hardware assembly guide
- **imu_calibration_guide.md** - IMU calibration procedures
- **TESTING_*.md** - Testing strategies and results
- **check/** - Additional guides and checklists

## Scripts (scripts/)

Organized by function:
- **Setup**: `setup.sh`, `raspberry_install.sh`, `setup_pico_sdk.sh`
- **Build**: `build.sh`, `build_on_remote.sh`
- **Firmware**: `flash_and_monitor.sh`, `diagnose_firmware.sh`
- **Testing**: `test_*.sh`, `proper_test_procedure.sh`
- **Services**: `install_*_service.sh` for systemd services
- **Analysis**: `analysis/` - Python scripts for codebase analysis

## Libraries (lib/)

External dependencies as git submodules:
- **FreeRTOS-Kernel/** - Real-time OS
- **pico-sdk/** - Raspberry Pi Pico SDK (via environment)
- **micro_ros_raspberrypi_pico_sdk/** - micro-ROS for Pico
- **eigen/** - Linear algebra library
- **pico-distance-sensor/** - Distance sensor library

## Key Configuration Files

### Workspace Root
- **Justfile** - Task automation (preferred over raw commands)
- **.env** - Environment variables (TARGET=robot|remote_pc)
- **.clang-format** - C++ code formatting rules
- **.flake8** - Python linting configuration
- **setup.cfg** - Python package configuration

### Build Artifacts (Gitignored)
- **build/** - Intermediate build files
- **install/** - Installed ROS2 packages
- **log/** - Build and runtime logs

## Package Naming Conventions

- **robot_*** - Project-specific packages
- Prefix indicates function: `robot_bringup`, `robot_hardware_interfaces`, `robot_vision`
- Use underscores, not hyphens in package names

## File Naming Conventions

### C++
- Headers: `.h` extension
- Source: `.cpp` extension
- Classes: PascalCase (e.g., `MotorsAgent.cpp`)

### Python
- Modules: snake_case (e.g., `launch_microros_agent.py`)
- Launch files: `*.launch.py`

### Configuration
- YAML: `*.yaml` (not `.yml`)
- Xacro: `*.xacro`
- World files: `*.world`

## Important Paths

### Pin Mapping
- **Authoritative source**: `docs/PINMAP.md`
- Firmware must match this documentation

### Controller Configs
- Mecanum: `robot_controller/config/{robot_model}/mecanum_drive_controller.yaml`
- Differential: `robot_controller/config/{robot_model}/diff_drive_controller.yaml`

### Launch Files
- Main bringup: `robot_bringup/launch/bringup.launch.py`
- Simulation: `robot/launch/launch_sim.launch.py`

### URDF
- Main robot: `robot_description/urdf/robot.urdf.xacro`
- Components: `robot_description/urdf/components/*.xacro`

## Build Output Locations

- ROS2 packages: `install/lib/<package_name>/`
- Firmware binary: `firmware/build/src/my_firmware.uf2`
- Firmware releases: `firmware/releases/my_firmware_*.uf2`

## Development Workflow

1. Make changes in `src/` or `firmware/src/`
2. Build with `just build` or `just build-firmware`
3. Test in simulation first (`just start-gazebo-sim`)
4. Flash firmware if needed (`just flash-firmware`)
5. Test on hardware with `ros2 launch robot_bringup bringup.launch.py`

## Notes

- Keep joint names consistent across URDF, controller YAML, and hardware interface
- `docs/PINMAP.md` is the single source of truth for pin assignments
- Use `just` commands instead of raw colcon/make commands when available
- Firmware and ROS2 packages have separate build systems
