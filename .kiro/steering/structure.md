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
├── config/                 # Workspace-level configs (joystick, etc.)
├── datasheet/              # Hardware datasheets
├── build/                  # Build artifacts (gitignored)
├── install/                # Install artifacts (gitignored)
├── log/                    # Build/runtime logs (gitignored)
├── _obsolet/               # Obsolete/archived code
├── .devcontainer/          # Dev container configuration
├── .github/                # GitHub workflows and CI
├── .kiro/                  # Kiro AI settings and specs
├── .vscode/                # VS Code configuration
├── Justfile                # Task runner commands
├── .env                    # Environment configuration
├── .env.*.example          # Example environment files
├── format.sh               # Code formatting script
├── setup.sh                # Workspace setup script
└── *.sh                    # Various utility scripts
```

## ROS2 Packages (src/)

### Core Robot Packages

- **robot/** - Main robot package with URDF, launch files, and configs
  - `description/` - URDF/xacro robot models
  - `launch/` - Launch files (simulation, robot state publisher, joystick)
  - `config/` - RViz configs, controller parameters, joystick configs
  - `worlds/` - Gazebo world files
  - `scripts/` - Helper scripts

- **robot_description/** - Robot URDF definitions and meshes
  - `urdf/` - Xacro files defining robot structure
  - `meshes/` - 3D models (STL/DAE)
  - `config/components_config/` - Component configurations
  - Based on Husarion components description

- **robot_bringup/** - Launch orchestration for hardware/simulation
  - `launch/` - Bringup launch files
  - `config/` - Runtime parameters

- **robot_controller/** - Controller configurations
  - `config/` - YAML configs for mecanum/diff drive controllers per robot model
  - `launch/` - Controller spawner launch files

- **robot_hardware_interfaces/** - ros2_control hardware interface
  - `src/` - C++ hardware interface plugin
  - `include/` - Header files
  - `config/` - Hardware parameters
  - Interfaces with micro-ROS firmware on Pico

### Custom Controllers

- **robot_controllers/** - Custom controller implementations
  - `mecanum_drive_controller/` - Custom mecanum drive controller (adapted from ros2_controllers)

### Functionality Packages

- **robot_gazebo/** - Gazebo simulation assets
- **robot_gz_worlds/** - Gazebo world files
- **robot_localization_tool/** - Sensor fusion (EKF) configuration
- **robot_vision/** - Computer vision nodes (face detection)
- **robot_nerf_launcher/** - Nerf launcher control logic
- **robot_utils/** - Utility scripts and tools

### External Dependencies

- **micro-ROS-Agent/** - Bridge between micro-ROS and ROS2
- **open_manipulator_x/** - Manipulator arm support (optional)
- **dynamixel_hardware_interface/** - Dynamixel servo interface
- **foxglove-joystick/** - Foxglove joystick integration
- **rosbot_components_description/** - Component descriptions

## Firmware Structure (firmware/)

```
firmware/
├── src/                    # Source code
│   ├── main.cpp           # Entry point, FreeRTOS setup
│   ├── Agent.h/cpp        # Base agent class
│   ├── BlinkAgent.h/cpp   # Status LED agent
│   ├── uRosBridge.h/cpp   # micro-ROS communication singleton
│   ├── uRosEntities.h/cpp # micro-ROS entity management
│   ├── PubEntities.h/cpp  # Publisher entities
│   ├── DDD.h/cpp          # Main robot control agent (odometry, cmd_vel)
│   ├── MotorsAgent.h/cpp  # Motor control with PID
│   ├── MotorPID.h/cpp     # PID controller implementation
│   ├── MotorMgr.h/cpp     # Motor hardware abstraction
│   ├── PWMManager.h/cpp   # PWM management for motors
│   ├── HCSR04Agent.h/cpp  # Ultrasonic sensor agent (deprecated)
│   ├── GPIOInputMgr.h/cpp # GPIO input management
│   ├── GPIOObserver.h/cpp # GPIO observer pattern
│   ├── hal/               # Hardware abstraction layer
│   │   ├── hardware/      # Hardware-specific implementations
│   │   └── application/   # Application HAL
│   ├── application/       # Application-specific code
│   │   ├── ImuAgent.h/cpp      # ICM20948 IMU agent
│   │   └── vl6180xAgent.h/cpp  # VL6180X ToF sensor agent
│   └── shared/            # Shared utilities
│       └── Vector3f.hpp   # 3D vector math
├── tests/                 # Unit tests (GoogleTest)
├── port/                  # FreeRTOS port configuration
├── releases/              # Compiled .uf2 files (gitignored)
├── build/                 # Debug build artifacts (gitignored)
├── build_release/         # Release build artifacts (gitignored)
├── CMakeLists.txt         # Build configuration
├── Makefile               # Build wrapper
├── README.md              # Firmware documentation
└── TOPIC_STANDARDIZATION.md  # Topic naming conventions
```

## Documentation (docs/)

- **PINMAP.md** - Authoritative pin mapping (single source of truth)
- **architecture_and_packages.md** - Package responsibilities
- **hardware_setup.md** - Hardware assembly guide
- **HARDWARE_SETUP_PLAN.md** - Hardware setup planning
- **imu_calibration_guide.md** - IMU calibration procedures
- **firmware.README.md** - Firmware documentation
- **MANIPULATOR.md** - Manipulator arm documentation
- **TEST_SUMMARY.md** - Test results summary
- **TESTING_AND_MOCKING_STRATEGY.md** - Testing approach
- **TESTING_COMPLETE.md** - Complete testing documentation
- **raspberry_pi_setup_plan.md** - Raspberry Pi setup guide
- **robot_firmware_pico_port_plan.md** - Pico firmware porting plan
- **tmux.md** - Tmux workflow documentation
- **command list.md** - Common commands reference
- **aliases.zsh** - Shell aliases
- **check/** - Additional guides and checklists

## Scripts (scripts/)

Organized by function:
- **Setup**: `setup.sh`, `raspberry_install.sh`, `setup_pico_sdk.sh`, `remote_pc_setup.sh`, `setup_autostart.sh`
- **Build**: `build.sh`, `build_on_remote.sh`
- **Firmware**: `diagnose_firmware.sh` (moved to workspace root)
- **Testing**: `test_*.sh`, `proper_test_procedure.sh`, `ros2_verification_checks.sh`, `simple_gazebo_test.sh`
- **Services**: `install_*_service.sh` for systemd services (robot, microros, foxglove)
- **Launch**: `launch_microros_agent.py`, `start_robot.sh`, `start_minimal.sh`, `start_sim_tmux.sh`, `start_teleop.sh`
- **Monitoring**: `check_*.sh` scripts for status checking
- **Utilities**: `imu_pose.py`, `log_sensor.py`, `range_to_marker.py`, `visualize_range.py`
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
- **.env.*.example** - Example environment configurations for different targets
- **.clang-format** - C++ code formatting rules
- **.flake8** - Python linting configuration
- **setup.cfg** - Python package configuration
- **.ament_cmake_lint.yaml** - Ament CMake linting config
- **.ament_xmllint.yaml** - Ament XML linting config
- **.markdownlint.yaml** - Markdown linting config
- **cspell.config.yaml** - Spell checking config

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
- Current board: robot_digital_v1 (Raspberry Pi Pico)

### Controller Configs
- Mecanum: `robot_controller/config/{robot_model}/mecanum_drive_controller.yaml`
- Differential: `robot_controller/config/{robot_model}/diff_drive_controller.yaml`
- Supported models: robot_xl (only model currently supported)

### Launch Files
- Main bringup: `robot_bringup/launch/bringup.launch.py`
- Simulation: `robot/launch/launch_sim.launch.py`
- Robot on Pi: `robot/launch/launch_robot_pi.launch.py`
- Joystick: `robot/launch/joystick_xbox_mecanum_pico.launch.py`
- Camera: `robot/launch/camera.launch.py`
- Dashboard: `robot/launch/dashboard.launch.py`

### URDF
- Main robot: `robot_description/urdf/robot.urdf.xacro`
- Core: `robot/description/robot_core.xacro`
- Components: `robot/description/*.xacro` (camera, lidar, depth_camera, face, etc.)

## Build Output Locations

- ROS2 packages: `install/lib/<package_name>/`
- Firmware binary (debug): `firmware/build/src/my_firmware.uf2`
- Firmware binary (release): `firmware/build_release/src/my_firmware.uf2`
- Firmware releases: `firmware/releases/my_firmware_*.uf2`

## Development Workflow

1. Make changes in `src/` or `firmware/src/`
2. Build with `just build` or `just build-firmware`
3. Test in simulation first (`just start-gazebo-sim`)
4. Flash firmware if needed (`just flash-firmware` or `just flash-firmware-release`)
5. Monitor firmware with `just monitor-firmware`
6. Start micro-ROS agent with `just start-microros`
7. Test on hardware with `ros2 launch robot_bringup bringup.launch.py`

## Notes

- Keep joint names consistent across URDF, controller YAML, and hardware interface
- `docs/PINMAP.md` is the single source of truth for pin assignments
- Use `just` commands instead of raw colcon/make commands when available
- Firmware and ROS2 packages have separate build systems
