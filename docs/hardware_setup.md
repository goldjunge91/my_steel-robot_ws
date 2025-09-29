# Hardware Preparation and Bringup Guide

This guide explains how to prepare the remote development PC, the single-board computer (SBC) on the robot, and the Raspberry Pi Pico firmware. It also covers hardware checks, flashing procedures, and how to start the full system in simulation or on the real robot.

## 1. Prerequisites

- Ubuntu 22.04 (or matching ROS 2 Humble environment)
- ROS 2 Humble installed (`/opt/ros/humble/setup.bash` available)
- `colcon`, `rosdep`, `tmux`, `git`, `build-essential`
- Pico SDK toolchain (or `picotool`) on the machine that flashes the firmware
- Docker (optional) if the micro-ROS agent shall run in a container
- Access to this workspace and the `ros2.repos` manifest already imported into `src/`

Environment variables:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_steel_ws/my_steel-robot_ws/install/setup.bash  # after building the workspace
```

## 2. Quick Checklist

| Task | Remote PC | SBC | Pico |
|------|-----------|-----|------|
| Update system packages | ✓ | ✓ | — |
| Install ROS 2 Humble | ✓ | ✓ | — |
| Clone workspace & build | ✓ | ✓ | — |
| Micro-ROS agent present | optional | ✓ | — |
| Pico SDK / flashing tools | optional | optional | ✓ |
| Firmware flashed | — | — | ✓ |
| ros2_control bringup | for simulation | ✓ | — |

## 3. Hardware Verification (Robot Bench Test)

Perform these checks before flashing or running the bringup:

1. **Serial Devices**
   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* /dev/gpiochip*
   udevadm info -a -n /dev/ttyACM0  # adjust port
   ```
   Ensure the user is in the `dialout` group (`sudo usermod -aG dialout $USER`).

2. **Power and Wiring**
   - Confirm Pico USB is connected and, if applicable, external power rails are on.
   - Compare wiring against `docs/PINMAP.md`.

3. **I2C / SPI peripherals (optional)**
   ```bash
   sudo apt install -y i2c-tools
   sudo i2cdetect -y 1
   ```

4. **Firmware Sanity**
   - If an older firmware is installed, check that the micro-ROS client enumerates by running the agent and verifying topic output.

## 4. Remote PC Preparation

1. Install dependencies and ROS 2 Humble.
2. Clone the workspace, import repos, and build:
   ```bash
   cd ~/ros2_steel_ws/my_steel-robot_ws
   vcs import src < ros2.repos
   rosdep install --from-paths src --ignore-src -y
   colcon build --symlink-install
   source install/setup.bash
   ```
3. Optional convenience targets (`just` recipes):
   - `just start-gazebo-sim` – Gazebo with mecanum controller
   - `just start-sim-tmux` – tmux session for simulation

## 5. SBC Preparation (Robot Onboard Computer)

1. Ensure ROS 2 Humble and required packages are installed (see README).
2. Copy the workspace or sync via `rsync`/`git`. Build as on the remote PC.
3. Configure micro-ROS agent:
   - Install the agent (`ros-humble-micro-ros-agent` or Docker image).
   - Optionally enable the provided systemd service (`scripts/micro_ros_agent.service`).
4. Confirm USB permissions and `dialout` group membership for the runtime user.

### Common tmux workflow (from `src/robot_bringup/README.md`)

```bash
# Start micro-ROS agent (Docker example)
tmux new-session -d -s sbc_agent -n agent
 tmux send-keys -t sbc_agent:agent 'docker run -it --rm -v /dev:/dev --privileged --net=host \
   microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200 -v6' C-m

# Start bringup with mecanum drive
tmux new-session -d -s sbc_bringup -n bringup
 tmux send-keys -t sbc_bringup:bringup 'source /opt/ros/humble/setup.bash; \
   source ~/ros2_steel_ws/my_steel-robot_ws/install/setup.bash; \
   ros2 launch mecabridge_hardware mecabridge_hardware.launch.py \
     drive_type:=mecanum device:=/dev/ttyACM0 baud_rate:=115200 microros:=true' C-m
```

Adjust `drive_type`, `device`, and `microros` according to the setup. Use `tmux attach -t <session>` to monitor logs.

## 6. Pico Firmware Build and Flash

Firmware sources live in `firmware/`. Two build modes exist (debug and release).

### 6.1 Build

```bash
cd ~/ros2_steel_ws/my_steel-robot_ws/firmware
make clean
make build_release   # produces build_release/src/my_firmware.uf2
```

Use `make build` for a default build in `firmware/build/`.

### 6.2 Flashing via UF2 (Bootsel)

1. Hold BOOTSEL while connecting the Pico to USB; it mounts as `RPI-RP2`.
2. Copy the generated UF2 file:
   ```bash
   cp build_release/src/my_firmware.uf2 /media/$USER/RPI-RP2/
   sync
   ```
3. The board reboots automatically with the new firmware.

### 6.3 Flashing via picotool (optional)

```bash
picotool load -f build_release/src/my_firmware.uf2
picotool reboot
```

### 6.4 Using the Python flashing helper

`robot_utils` provides scripts that select ports automatically:

```bash
ros2 run robot_utils flash_firmware.py --robot-model robot \
  --port /dev/ttyACM0 --file firmware/build_release/src/my_firmware.uf2 --usb
```

The script also supports downloading prebuilt firmware if no `--file` is given.

## 7. Simulation with Controller

### 7.1 Direct launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot launch_sim.launch.py \
  world:=src/robot/worlds/obstacles.world \
  with_gazebo_gui:=true with_rviz:=true headless:=false
```

This launch file loads the mecanum drive controller (`drive_controller`) and `joint_state_broadcaster` after spawning the robot (`src/robot/launch/launch_sim.launch.py`).

### 7.2 tmux convenience

```bash
./scripts/start_sim_tmux.sh
```

Creates windows for simulation, teleop, and monitoring with the correct environment sourcing.

## 8. Real Robot Bringup

1. Confirm firmware is flashed and the micro-ROS agent is running (see Section 5).
2. Launch the hardware interface with correct parameters:

```bash
ros2 launch mecabridge_hardware mecabridge_hardware.launch.py \
  drive_type:=mecanum device:=/dev/ttyACM0 baud_rate:=115200 \
  has_encoders:=false use_mock_hardware:=false
```

The launch file selects the proper controller (`mecanum_cont`) and loads URDF/configs (`src/mecabridge_hardware/launch/mecabridge_hardware.launch.py`).

3. Verify controllers:

```bash
ros2 service call /controller_manager/list_controllers ros2_control_msgs/srv/ListControllers
ros2 topic list
ros2 topic echo /mecanum_drive_controller/odometry --once
```

4. Start teleoperation or navigation nodes as required (e.g., `ros2 run teleop_twist_keyboard teleop_twist_keyboard`).

## 9. Troubleshooting

- **Controller fails to start**: Check `configUSE_CORE_AFFINITY` in the FreeRTOS configuration and ensure test configs do not override it when building pico SDK tests.
- **No serial connection**: Ensure correct port, check `udev` rules (`scripts/99-pico.rules`), and confirm user permissions.
- **micro-ROS agent silent**: Inspect logs with `journalctl -u microros-agent.service -f` or the Docker container output.
- **Simulation drift**: Verify `use_sim_time` parameters and controller YAML in `robot/config/`.

## 10. Reference Files

- `README.md` – workspace overview and quickstart steps
- `src/robot_bringup/README.md` – detailed bringup and tmux automation
- `src/robot/launch/launch_sim.launch.py` – simulation launch definition
- `src/mecabridge_hardware/launch/mecabridge_hardware.launch.py` – hardware bringup
- `docs/PINMAP.md` – wiring reference
- `scripts/start_sim_tmux.sh` – tmux simulation helper

Keep this document under version control and update it whenever new hardware or flashing steps are introduced.

## 11. Package Inventory (src/)

The workspace mixes ROS 2 packages and support repositories under `src/`. Use the table below to orient yourself when wiring, flashing, or launching components.

| Path | Type | Purpose | Key Assets / Notes |
|------|------|---------|--------------------|
| `husarion_controllers/mecanum_drive_controller` | ROS 2 package | ros2_control velocity controller that converts `/cmd_vel` into four wheel speed commands for mecanum bases; publishes odometry and TF | Plugin manifest `mecanum_drive_plugin.xml`; parameters `src/mecanum_drive_controller_parameter.yaml`; library registered in `CMakeLists.txt` |
| `mecabridge_hardware` | ROS 2 package | SystemInterface implementation that bridges ros2_control to the Pico firmware via deterministic serial protocol | Launch files under `launch/`; configs under `config/`; hardware interface code in `src/mecabridge_hardware/`; documented in `README.md` |
| `micro-ROS-Agent/micro_ros_agent` | ROS 2 package (upstream) | Canonical micro-ROS agent executable; allows Pico firmware to attach via serial or UDP | Launch helper `src/robot_bringup/launch/microros_agent.launch.py` uses this package; follow upstream README for options |
| `open_manipulator_x/open_manipulator_x_description` | ROS 2 package | URDF/Xacro, meshes, and kinematic data for the OpenMANIPULATOR-X arm | Use when attaching the manipulator to the chassis; check `urdf/` and `meshes/` |
| `open_manipulator_x/open_manipulator_x_moveit` | ROS 2 package | MoveIt2 configs and launch files for the manipulator | Requires MoveIt planners; see `launch/` |
| `open_manipulator_x/open_manipulator_x_joy` | ROS 2 package | Joystick interface nodes for teleoperating the manipulator | Launch `launch/open_manipulator_x_joy.launch.py` with joystick hardware |
| `robot` | ROS 2 package | Core simulation package: URDF, Gazebo worlds, RViz configs, and top-level launch (`launch_sim.launch.py`) | Description in `README.md`; adjust `description/robot.urdf.xacro`; controllers loaded via `config/my_controllers.yaml` |
| `robot_autonomy` | Support repo | Navigation2, SLAM, deployment scripts (Docker/just) | `nav2/` contains parameter sets; `justfile` orchestrates nav stack; not a catkin package |
| `robot_bringup` | Support repo | Launch and shell scripts to start micro-ROS agent and overall bringup | `launch/microros_agent.launch.py`; tmux automation documented in `README.md`; integrate with controller spawners |
| `robot_controller` | ROS 2 package | Aggregated controller configurations and launch wrappers for the robot family | Depends on `controller_manager`, `mecanum_drive_controller`, `diff_drive_controller`, etc.; see `config/` and `launch/` |
| `robot_description` | ROS 2 package | Shared URDF, meshes, and component configs for the chassis (simulation + hardware) | Primary entrypoint `urdf/multi_drive_robot.urdf.xacro`; keep joint names consistent with controllers |
| `robot_firmware` | External repo | Placeholder for dedicated Pico firmware project (often maintained separately) | Use for alternative/build history; actual firmware in workspace’s top-level `firmware/` directory |
| `robot_gazebo` | Support repo | Additional Gazebo resources or experiments (currently minimal placeholder) | Extend with custom worlds/plugins as needed |
| `robot_hardware` | Support repo | Notes and prototypes around alternative hardware interfaces | Check `README.md`; not an active ROS package yet |
| `robot_localization` | Upstream package snapshot | Contains built artifacts and ignore markers for the `robot_localization` stack | When needed, replace with official release or rebuild from source |
| `robot_nerf_launcher` | Support repo | (Placeholder) Code and documentation for the Nerf launcher attachment | Expand with nodes controlling actuators or sensors for the accessory |
| `robot_utils` | ROS 2 package | Helper scripts/binaries for flashing, serial discovery, deployment | `robot_utils/flash_firmware.py`, `scripts/` utilities; depends on `ament_index_python`; install provides CLI tools |
| `robot_vision` | Support repo | Reserved for camera/vision processing nodes | Document pipeline once implemented (e.g., face detection, AprilTags) |
| `rosbot_hardware_interfaces` | ROS 2 package (upstream) | Husarion’s ros2_control integration for ROSbot platforms—used as reference or dependency | Provides example diff-drive controller, URDF includes, and topics (`README.md`) |
| `serial` | External library | Vendored serial port utility (likely `wjwwood/serial`) for low-level communication | Treat as third-party dependency; do not modify unless updating vendor drop |

### Cross-Package Dependencies

- `mecabridge_hardware` loads controllers defined in `robot/config/my_controllers.yaml` and expects URDFs from `robot_description`.
- `robot` simulation uses `mecanum_drive_controller` for kinematics and optionally spawns the hardware interface in mock mode.
- `robot_controller` launch files spawn controller manager nodes that depend on both `mecabridge_hardware` and `husarion_controllers`.
- `robot_autonomy` assumes `/tf`, `/odom`, and controller topics from either simulation (`robot`) or real hardware (`mecabridge_hardware`).
- `micro-ROS-Agent` must run before `mecabridge_hardware` when the Pico firmware uses micro-ROS transports.

Keep this section updated when new packages are added or deprecated to avoid stale bringup instructions.

## 12. Per-Package File Breakdown (`src/`)

Use this deep dive when you need to locate specific launch files, headers, or implementation details. Paths are relative to `my_steel-robot_ws/src`.

### husarion_controllers/mecanum_drive_controller
- `package.xml`, `CMakeLists.txt`: declares ament-based shared library build.
- Headers under `include/mecanum_drive_controller/`
  - `mecanum_drive_controller.hpp`: main controller class deriving from `controller_interface::ControllerInterface`.
  - `odometry.hpp`, `speed_limiter.hpp`: helper classes used by the controller implementation.
  - `visibility_control.h`: export macros for Windows/Linux builds.
- Sources in `src/`
  - `mecanum_drive_controller.cpp`: lifecycle methods, interface configuration, message subscription, state publishing.
  - `odometry.cpp`, `speed_limiter.cpp`: math helpers for wheel odometry and rate limiting.
  - `mecanum_drive_controller_parameter.yaml`: input for `generate_parameter_library` to create strongly-typed parameters.
- `mecanum_drive_plugin.xml`: registers the controller with pluginlib.
- `doc/userdoc.rst`, `CHANGELOG.rst`: upstream documentation and release history.

### mecabridge_hardware
- `package.xml`, `CMakeLists.txt`: builds the hardware interface library and installs launch/config assets.
- `src/mecabridge_hardware/`
  - `mecabridge_hardware_interface.cpp`: implements `hardware_interface::SystemInterface` lifecycle, read/write loops, and watchdog logic.
  - `mecabridge_serial_protocol.cpp`: CRC-validated framing, command/state payload structures.
  - `wheel.cpp`: per-wheel state bookkeeping (position, velocity, encoder conversions).
  - `mecabridge_hardware.cpp`: plugin entry point.
  - `mecabridge_hardware_interface_temp.cpp`: previous revision kept for reference.
- `config/`
  - `mecabridge_*_controller.yaml`: ros2_control controller definitions (diff, mecanum, four-wheel).
  - `mecabridge_hardware_params.yaml`: hardware-specific parameters (serial device, watchdog, drive type).
  - `mecabridge_hardware.xml`, `robot_hardware.xml`, `fake_robot_hardware.xml`: plugin manifests for ros2_control.
  - `robot_controller_example.yaml`, `mecabridge_example_configs.yaml`: sample controller_manager setups.
- `launch/`: per-drive launch files plus `mecabridge_hardware.launch.py` master launcher accepting arguments (`drive_type`, `use_mock_hardware`, etc.).
- `test_legacy/`: GoogleTest suites and helper headers for protocol/hardware interface regression tests; useful references when writing new tests.
- `scripts/`: developer helpers (build/dev shell, license insertion, header fixes).
- `README.md`, `README_drive_arduino.md`: architecture, serial protocol, tmux workflows.

### micro-ROS-Agent
- Top-level licensing (`LICENSE`, `NOTICE`, third-party list) and contributor guide.
- `micro_ros_agent/package.xml`, `CMakeLists.txt`: standard micro-ROS agent build; depends on `rcl`, `rmw` implementations.
- `micro_ros_agent/README.md`: runtime options (serial, UDP, CAN).
- `snap/snapcraft.yaml`: snap packaging recipe.
- Treat this as an upstream dependency—avoid local modifications unless tracking forks.

### open_manipulator_x
- Multi-package repository: `open_manipulator_x_description`, `open_manipulator_x_moveit`, `open_manipulator_x_joy` each provide `package.xml` + `CMakeLists.txt`.
  - `*_description`: URDF/Xacro, STL meshes for the manipulator; use with `robot_description` if arm is mounted.
  - `*_moveit`: MoveIt2 configs (`config/`, `launch/`) and `.setup_assistant` snapshot.
  - `*_joy`: joystick teleop nodes and launch files for the arm.
- `README.md`: upstream quickstart and wiring.

### robot
- Core mobile base package.
- `package.xml`, `CMakeLists.txt`, `pyproject.toml`: builds both C++ launch support and Python utilities.
- `description/`: modular Xacro (sensors, ros2_control, inertials) aggregated by `robot.urdf.xacro`.
- `config/`: ros2_control controller YAMLs (`my_controllers*.yaml`), teleop configs, Gazebo parameters, RViz layouts.
- `launch/`: bringup entry points (simulation, dashboard, joystick, camera, RPLIDAR, TF relay).
- `scripts/`: helper scripts (wait for service, xacro check).
- `docs/` and `Projekt.md`: requirements, vendor datasheets, workflow notes.
- `todo.md`, `Tasks.md`: outstanding work items.

### robot_autonomy
- Not a ROS package; containerized Nav2/SLAM orchestrations.
- Key directories: `docker/` (compose definitions), `nav2/` (param files, launch scripts), `justfile`/`Makefile` (developer recipes).
- Use when bringing up autonomy stacks on remote machines.

### robot_bringup
- Launch and scripting toolkit around micro-ROS agent and ros2_control bringup.
- `launch/microros_agent.launch.py`: parameterized agent launcher (Docker or native).
- `scripts/run_microros_agent.sh`, `scripts/install_microros_agent.sh`: automation for agent deployment.
- `README.md`: tmux-based operations guide covering SBC and remote PC flows.

### robot_controller
- Pure-Python ROS 2 package (`setup.py`, `setup.cfg`, `package.xml`).
- `launch/controller.launch.py`, `launch/manipulator.launch.py`: spawn controller manager plus specific controllers.
- `test/`: simple pytest hooks verifying xacro and style compliance.
- Acts as glue between hardware interfaces and controllers shipped in this workspace.

### robot_description
- URDF/Xacro + meshes describing the chassis.
- `README.md`: usage instructions.
- `config/components/` (if populated) and sensor macro includes to compose new variants.
- Ensure any joint name changes propagate to matching controller YAMLs.

### robot_firmware
- Placeholder for an external firmware repository (currently only `.git`).
- Actual firmware you build lives in top-level `firmware/` directory; keep this as a reference if tracking upstream history.

### robot_gazebo
- Currently a stub with `README.md`; extend with Gazebo plugins/worlds as simulation demands grow.

### robot_hardware
- Concept notes and potential future hardware interface experiments documented in `README.md` (no code yet).

### robot_localization
- Snapshot of the `robot_localization` stack, mostly populated with build/install artifacts and `COLCON_IGNORE` to avoid accidental builds.
- Replace with a clean source checkout if you need to customize EKF/UKF nodes.

### robot_nerf_launcher
- Placeholder for the Nerf attachment; `README.md` describes intended control interface.

### robot_utils
- Installable Python package providing CLI helpers.
- `robot_utils/flash_firmware.py`: entry point for flashing via UART/USB (downloads firmware if needed).
- `flash_firmware_{uart,usb}.py`, `utils.py`: shared helpers (port discovery, subprocess wrappers).
- `launch/laser_filter.launch.py`: example ROS 2 launch included with the package.
- `README.md`: usage examples and best practices.

### robot_vision
- Placeholder repository; `README.md` outlines planned vision stack (face detection, AprilTags, etc.).

### rosbot_hardware_interfaces
- Upstream ros2_control implementation for ROSbot platforms.
- `README.md`: topic interface summary, launch instructions (`example_diff_drive.launch.py`).
- Useful reference when comparing mecabridge configuration to existing Husarion robots.

### serial
- Vendored serial-port library (git submodule). No ROS build files—used as third-party dependency for firmware flashing or hardware interface code.

### Additional Notes
- Many repositories under `src/` contain their own `.git/` directories, indicating they were imported via `vcs`. Coordinate updates through `ros2.repos` or submodule management rather than editing history inside the workspace.
- `COLCON_IGNORE` files (e.g., under `robot_localization/build/`) prevent colcon from descending into generated artifacts—keep them in place to avoid accidental rebuilds of cached outputs.

Use this breakdown alongside the package inventory to quickly locate implementations, configs, or launch entries when debugging or extending the stack.
