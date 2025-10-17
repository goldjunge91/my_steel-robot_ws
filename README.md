# my_steel Robot Workspace

ROS2 workspace for the my_steel omnidirectional mobile robot platform with mecanum drive and Nerf launcher.

## Overview

The my_steel robot is an educational and research platform built on ROS2 Humble, featuring:

- **Omnidirectional mobility** via 4-wheel mecanum drive
- **Real-time control** using Raspberry Pi Pico with micro-ROS
- **Autonomous navigation** with RPLiDAR and sensor fusion
- **Computer vision** for face detection and tracking
- **Interactive Nerf launcher** with dedicated controller (Arduino Nano/Pro Micro)
- **Remote operation** via Tailscale VPN over 4G/5G networks
- **Optional manipulator arm** support (Open Manipulator X)

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Raspberry Pi Pico SDK (for firmware builds)

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/goldjunge91/my_steel-robot_ws.git
   cd my_steel-robot_ws
   ```

2. Install VCS tool (for dependency management):

   ```bash
   pip3 install vcstool
   # Or on macOS with Homebrew:
   # brew install vcstool
   ```

3. Import dependencies:

   ```bash
   # Import ROS2 packages (src/)
   vcs import src < src/ros2.repos
   
   # Initialize library submodules (lib/)
   git submodule update --init --recursive lib/
   ```

4. Install ROS dependencies:

   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the workspace:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

6. Flash firmware to Pico (see [Firmware](#firmware) section)

### Docker Deployment (Alternative)

For production deployment on Raspberry Pi, Docker provides a containerized solution with all dependencies pre-installed:

**Features:**

- Pre-built ROS2 Humble environment with all dependencies
- Automatic service orchestration with Docker Compose
- Integrated Tailscale VPN for secure remote access
- Persistent logs and configuration
- Automatic restart on failure

**Quick Start:**

```bash
# Pull the pre-built image
docker pull mysteel/robot:humble-arm64

# Copy docker-compose configuration
cp docker/compose.robot-pi.yaml ~/compose.robot-pi.yaml

# Configure Tailscale (optional)
cp docker/.env.robot-pi.example ~/.env
# Edit ~/.env and add your TAILSCALE_AUTHKEY

# Start the robot
docker compose -f ~/compose.robot-pi.yaml up -d

# View logs
docker compose -f ~/compose.robot-pi.yaml logs -f
```

**Documentation:**

- See [docker/README.md](docker/README.md) for complete deployment guide
- Includes build instructions, configuration options, and troubleshooting
- Supports Tailscale VPN for remote access over 4G/5G networks

### Running the Robot

1. Start micro-ROS agent:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
   # Or use auto-detection:
   python3 scripts/launch_microros_agent.py
   ```

2. Launch robot bringup:

   ```bash
   ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl
   ```

3. Control with keyboard:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

## ROS2 Topics

The robot uses standard ROS2 topic names following REP-105:

### Published Topics

| Topic                       | Message Type            | Rate     | Description                       |
| --------------------------- | ----------------------- | -------- | --------------------------------- |
| `/joint_states`             | sensor_msgs/JointState  | 100 Hz   | Wheel encoder feedback            |
| `/imu/data_raw`             | sensor_msgs/Imu         | 50 Hz    | IMU data (ICM20948)               |
| `/odom`                     | nav_msgs/Odometry       | 50 Hz    | Wheel odometry                    |
| `/sensors/range_tof`        | sensor_msgs/Range       | Variable | Time-of-Flight distance (VL6180X) |
| `/sensors/range_ultrasonic` | sensor_msgs/Range       | Variable | Ultrasonic distance (HC-SR04)     |
| `/sensors/illuminance`      | sensor_msgs/Illuminance | Variable | Ambient light (VL6180X)           |
| `/odometry/wheels`          | nav_msgs/Odometry       | 50 Hz    | Controller odometry output        |

### Subscribed Topics

| Topic      | Message Type        | Description                        |
| ---------- | ------------------- | ---------------------------------- |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands for robot motion |

### Topic Architecture

```
┌─────────────────────┐
│ Navigation / Teleop │
└──────────┬──────────┘
           │ /cmd_vel (Twist)
           ▼
┌─────────────────────┐
│ Mecanum Controller  │
└──────────┬──────────┘
           │ velocity commands
           ▼
┌─────────────────────┐
│ Hardware Interface  │──────> /cmd_vel (Twist)
│  (ros2_control)     │<────── /joint_states
└─────────────────────┘        /imu/data_raw
           │
           ▼
┌─────────────────────┐
│  micro-ROS Agent    │
│  (topic remapping)  │
└──────────┬──────────┘
           │ USB Serial
           ▼
┌─────────────────────┐
│  Pico Firmware      │
│  (micro-ROS)        │
└─────────────────────┘
```

**Note**: The micro-ROS agent automatically adds `/rt/` prefix to firmware topics and remaps them to standard names. See [micro-ROS Agent Configuration](#micro-ros-agent-configuration) for details.

## Dependency Management Strategy

This project uses a **hybrid approach** for managing dependencies:

### VCS for ROS2 Packages (src/)

- **What**: Your robot packages that change frequently
- **Why**: Easy for contributors, standard ROS2 workflow
- **How**: Uses `src/ros2.repos` file with `vcs import`

```yaml
# src/ros2.repos example
repositories:
  robot:
    type: git
    url: https://github.com/goldjunge91/robot.git
    version: humble  # Always latest from humble branch
```

**Commands:**

```bash
# Import all packages
vcs import src < src/ros2.repos

# Check status
vcs status src

# Update all packages
vcs pull src

# Switch branches
vcs custom src --args checkout humble
```

### Git Submodules for Libraries (lib/)

- **What**: External libraries that need exact versions
- **Why**: Precise version control, important for firmware builds
- **How**: Traditional git submodules with `lib/lib_repos.repos` as reference

```yaml
# lib/lib_repos.repos example
repositories:
  FreeRTOS-Kernel:
    type: git
    url: https://github.com/FreeRTOS/FreeRTOS-Kernel
    version: V10.6.2  # Exact tag version
```

**Commands:**

```bash
# Initialize submodules
git submodule update --init --recursive lib/

# Update to latest
git submodule update --remote lib/

# Set specific version
cd lib/FreeRTOS-Kernel && git checkout V10.6.2
```

### Why This Hybrid Approach?

| Aspect              | VCS (src/)                   | Submodules (lib/)         |
| ------------------- | ---------------------------- | ------------------------- |
| **Use Case**        | Your ROS2 packages           | External libraries        |
| **Updates**         | Frequent, latest from branch | Rare, specific versions   |
| **Complexity**      | Simple `vcs pull`            | More complex git commands |
| **Contributors**    | Easy `vcs import`            | Automatic with git clone  |
| **Version Control** | Branch-based                 | Commit/tag-based          |
| **Offline Work**    | Requires internet            | Available offline         |

### Directory Structure

```
my_steel-robot_ws/
├── src/                    # ROS2 Packages → VCS managed
│   ├── robot/             # Your packages, frequent updates
│   ├── robot_bringup/     # Development branches (humble/main)
│   └── ros2.repos         # VCS configuration
├── lib/                   # External Libraries → Git Submodules  
│   ├── FreeRTOS-Kernel/  # Stable versions, exact tags
│   ├── eigen/             # Precise version for firmware
│   └── lib_repos.repos    # Reference (not used by git)
└── firmware/              # Pico firmware (separate build)
```

## Architecture

### Hardware Components

- **Raspberry Pi 4B**: Main SBC running ROS2 Humble with Tailscale VPN
- **Raspberry Pi Pico**: Real-time motor control and sensor interfacing (base platform)
- **Arduino Nano / Pro Micro**: Dedicated Nerf launcher controller
- **ICM20948**: 9-DOF IMU (SPI)
- **VL6180X**: Time-of-Flight distance sensor (I2C)
- **YDLIDAR LDS01RR**: 2D LiDAR scanner (360°, 8m range)
- **4x DC Motors (GM3865-520)**: Mecanum wheels with Hall encoders
- **USB Camera**: Computer vision (1080p)
- **Nerf Launcher**: 2x RS2205 brushless motors (flywheel), 2x servos (pan/tilt), trigger mechanism

### Software Stack

- **ROS2 Humble**: Primary robotics middleware
- **ros2_control**: Hardware abstraction framework
- **mecanum_drive_controller**: Custom mecanum drive controller
- **micro-ROS**: ROS2 client library for microcontrollers (Pico, Arduino)
- **FreeRTOS**: Real-time OS on Pico
- **Tailscale**: Zero-config VPN for secure remote access over 4G/5G
- **Nav2**: Navigation stack (optional)
- **SLAM Toolbox**: Karto-based SLAM implementation
- **Gazebo**: Simulation environment

### Package Structure

```
src/
├── robot_description/          # URDF, meshes, component configs
├── robot_bringup/             # Launch files and configurations
├── robot_controller/          # Controller configurations
├── robot_hardware_interfaces/ # ros2_control hardware interface
├── robot_controllers/         # Custom controller implementations
├── robot_gazebo/              # Gazebo simulation assets
├── robot_localization_tool/   # Sensor fusion (EKF)
├── robot_vision/              # Computer vision nodes
├── robot_utils/               # Utility scripts and tools
└── micro-ROS-Agent/           # micro-ROS bridge
```

## Firmware

The Pico firmware is located in the `firmware/` directory and uses micro-ROS for ROS2 communication.

### Building Firmware

```bash
cd firmware

# Debug build
make build

# Release build
make build_release
```

### Flashing Firmware

1. Put Pico in BOOTSEL mode (hold BOOTSEL button while connecting USB)
2. Flash the firmware:

   ```bash
   make flash          # Debug
   make flash-release  # Release
   ```

3. Monitor firmware output:

   ```bash
   ./monitor_firmware.sh
   ```

### Firmware Topics

The firmware publishes and subscribes to the following topics (with `/rt/` prefix added by micro-ROS):

- Publishes: `/joint_states`, `/imu/data_raw`, `/odom`, `/sensors/*`
- Subscribes: `/cmd_vel`

See `firmware/README.md` for detailed firmware documentation.

## micro-ROS Agent Configuration

The micro-ROS agent bridges communication between the Pico firmware and ROS2. It automatically adds `/rt/` prefix to firmware topics and uses remapping to convert them to standard ROS2 names.

### Topic Remapping

| Firmware Topic     | ROS2 Topic      | Direction       |
| ------------------ | --------------- | --------------- |
| `/rt/joint_states` | `/joint_states` | Firmware → ROS2 |
| `/rt/imu/data_raw` | `/imu/data_raw` | Firmware → ROS2 |
| `/rt/odom`         | `/odom`         | Firmware → ROS2 |
| `/rt/sensors/*`    | `/sensors/*`    | Firmware → ROS2 |
| `/cmd_vel`         | `/rt/cmd_vel`   | ROS2 → Firmware |

### Starting the Agent

```bash
# Auto-detect Pico device
python3 scripts/launch_microros_agent.py

# Manual device specification
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# Using launch file (includes remapping)
ros2 launch robot_bringup microros_agent.launch.py
```

## Using Just (Task Runner)

The workspace includes a `Justfile` for common tasks:

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

# Firmware commands
just build-firmware          # Build debug firmware
just build-firmware-release  # Build release firmware
just flash-firmware          # Flash debug firmware
just flash-firmware-release  # Flash release firmware
just monitor-firmware        # Monitor firmware output
just test-firmware           # Test firmware connection

# Simulation
just start-gazebo-sim        # Start Gazebo simulation
just start-sim-tmux          # Start simulation in tmux

# Runtime
just run-teleop              # Start keyboard teleoperation
just start-microros          # Start micro-ROS agent (auto-detect)
just start-microros-dev /dev/ttyACM0  # Start with specific device

# Utilities
just shell                   # Open shell with ROS sourced
just check-target            # Check target configuration
```

## Configuration

### Environment Variables

Configure your target platform in `.env`:

```bash
# Copy from example
cp .env.example .env

# Edit .env
TARGET=robot      # For Raspberry Pi deployment
# TARGET=remote_pc  # For development PC with simulation
```

### Key Parameters

- `ROBOT_MODEL_NAME`: robot_xl (only supported model)
- `DRIVE_TYPE`: mecanum (primary) or diff (legacy)
- `SERIAL_PORT`: /dev/ttyACM0 (Pico serial port)
- `SERIAL_BAUDRATE`: 115200
- `MICROROS`: true (enable micro-ROS agent)

## Development

### Building Specific Packages

```bash
# Build single package
colcon build --packages-select robot_hardware_interfaces

# Build with dependencies
colcon build --packages-up-to robot_bringup

# Build with symlink install (faster for Python)
colcon build --symlink-install
```

### Testing

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select robot_hardware_interfaces

# Show test results
colcon test-result --all --verbose
```

### Code Formatting

```bash
# Format C++ code
./format.sh

# Check Python style
flake8 .
```

## Simulation

Launch Gazebo simulation:

```bash
ros2 launch robot_gazebo launch_sim.launch.py use_sim_time:=true
```

The simulation includes:

- Robot model with mecanum wheels
- Gazebo physics
- ros2_control integration
- Sensor simulation (IMU, LiDAR)

## Troubleshooting

### No topics appearing

1. Check micro-ROS agent is running:

   ```bash
   ps aux | grep micro_ros_agent
   ```

2. Verify Pico connection:

   ```bash
   ls -l /dev/ttyACM*
   ./scripts/check_pico.sh
   ```

3. Monitor firmware:

   ```bash
   cd firmware && ./monitor_firmware.sh
   ```

### Remote connection issues (Tailscale)

1. Check Tailscale status:

   ```bash
   sudo tailscale status
   ```

2. Verify connectivity:

   ```bash
   ping <robot-tailscale-ip>
   ```

3. Check ROS2 domain over Tailscale:

   ```bash
   export ROS_DOMAIN_ID=0
   ros2 topic list
   ```

### Hardware interface fails to activate

1. Verify firmware is publishing:

   ```bash
   ros2 topic echo /joint_states --once
   ```

2. Check controller status:

   ```bash
   ros2 control list_controllers
   ```

3. Review logs:

   ```bash
   ros2 launch robot_bringup bringup.launch.py --log-level debug
   ```

### Robot doesn't respond to commands

1. Test velocity commands:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
   ```

2. Verify controller is active:

   ```bash
   ros2 control list_controllers
   # mecanum_drive_controller should be "active"
   ```

3. Check firmware is receiving commands:

   ```bash
   ros2 topic echo /rt/cmd_vel
   ```

## Migration from Old Topic Names

If you're upgrading from a previous version that used `/ddd/*` topic prefixes, see the [Migration Guide](docs/ROS2_TOPIC_MIGRATION_GUIDE.md) for detailed instructions.

### Quick Migration Summary

- `/ddd/imu` → `/imu/data_raw`
- `/ddd/odom` → `/odom`
- `/ddd/range_tof` → `/sensors/range_tof`
- `/ddd/range` → `/sensors/range_ultrasonic`
- `/ddd/cmd_vel` → `/cmd_vel` (alternative removed)

**Important**: Firmware must be updated to work with new topic names. See migration guide for step-by-step instructions.

## Documentation

- [PINMAP.md](docs/PINMAP.md) - Pin assignments and hardware connections
- [Projekt.md](Projekt.md) - Comprehensive project description (German)
- [SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md) - Detailed system architecture
- [ROS2_TOPIC_MIGRATION_GUIDE.md](docs/ROS2_TOPIC_MIGRATION_GUIDE.md) - Migration guide for topic standardization
- [architecture_and_packages.md](docs/architecture_and_packages.md) - Package responsibilities
- [hardware_setup.md](docs/hardware_setup.md) - Hardware assembly guide
- [firmware.README.md](docs/firmware.README.md) - Firmware documentation

## Contributing

1. Follow ROS2 coding standards
2. Use `clang-format` for C++ code
3. Use `flake8` for Python code
4. Add tests for new features
5. Update documentation

## License

See [LICENSE](LICENSE) file for details.

## Maintainer

- **Owner**: @goldjunge91
- **Repository**: goldjunge91/my_steel-robot_ws

## References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ros2_control Documentation](https://control.ros.org/)
- [micro-ROS Documentation](https://micro.ros.org/)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [REP-103: Standard Units of Measure](https://www.ros.org/reps/rep-0103.html)
