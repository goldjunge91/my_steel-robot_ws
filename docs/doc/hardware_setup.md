---
title: Hardware Setup Guide
summary: Complete hardware preparation and bringup guide
description: Step-by-step guide for preparing the remote PC, Raspberry Pi, and Pico firmware for the my_steel robot
keywords: hardware setup, raspberry pi, pico, firmware flashing, bringup
author: goldjunge91
alpha: true
order: 4
---

<!-- TODO: Uncomment text has to be checked -->
<!-- 

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
   - `just start-sim-tmux` – tmux session for simulation -->
<!-- 

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
``` -->

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

### Cross-Package Dependencies

- `mecabridge_hardware` loads controllers defined in `robot/config/my_controllers.yaml` and expects URDFs from `robot_description`.
- `robot` simulation uses `mecanum_drive_controller` for kinematics and optionally spawns the hardware interface in mock mode.
- `robot_controller` launch files spawn controller manager nodes that depend on both `mecabridge_hardware` and `robot_controllers`.
- `robot_autonomy` assumes `/tf`, `/odom`, and controller topics from either simulation (`robot`) or real hardware (`mecabridge_hardware`).
- `micro-ROS-Agent` must run before `mecabridge_hardware` when the Pico firmware uses micro-ROS transports.

Keep this section updated when new packages are added or deprecated to avoid stale bringup instructions.

## 12. Per-Package File Breakdown (`src/`)

Use this deep dive when you need to locate specific launch files, headers, or implementation details. Paths are relative to `my_steel-robot_ws/src`.

### robot_controllers/mecanum_drive_controller

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

### robot_hardware_interfaces

- Upstream ros2_control implementation for robot platforms.
- `README.md`: topic interface summary, launch instructions (`example_diff_drive.launch.py`).
- Useful reference when comparing mecabridge configuration to existing  robots.
