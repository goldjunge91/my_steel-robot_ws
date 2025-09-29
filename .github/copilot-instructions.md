# my_steel-robot_ws — AI coding agent instructions

Purpose: give an AI agent the minimal, actionable knowledge to be productive in this repo.

Key facts
- Tech: ROS2 (Humble), C++17 (ROS nodes), Python 3.8+ (tools/scripts), Gazebo / gz_ros2_control for sim.
- Embedded: Raspberry Pi Pico (RP2040) firmware in C/C++ using Pico SDK 1.5.x, STM32 HAL compatibility layer
- Build system: workspace-level helpers ./build.sh, ./test.sh, ./setup.sh and standard colcon/ament packages under `src/`.

Quick commands (what humans use)
- Build workspace: ./build.sh
- Run tests: ./test.sh (or run the VS Code task "test")
- Source environment after install: `install/local_setup.sh` (or `local_setup.bash` / `local_setup.ps1`)
- Start simulation: ./start_sim_tmux.sh (tmux session configured for sim + logs)

Architecture highlights (what to inspect first)
- Packages live under `src/` — notable groups: hardware interfaces (`mecabridge_hardware`, `robot_hardware`, `serial`), controllers (`my_steel_controller`, `robot_controller`, `mecanum_drive_controller`), robot descriptions (`robot_description`, `open_manipulator_x_description`), simulation (`robot_gazebo`), firmware (`robot_firmware`).
- Communication surface: ROS2 topics/services/actions between packages; hardware <-> controller uses ros2-control + gz_ros2_control; look for controller configs in `robot_controller` and hardware interface implementations in `mecabridge_hardware`.
- Firmware & serial: `robot_firmware` and `serial` implement the low-level protocol and flashing utilities — changes here usually require hardware flash or simulated serial bridge.
- Embedded firmware: Pico firmware under `src/robot_firmware/lpico/` provides USB CDC communication, motor control (TB6612 drivers), encoder feedback, and shooter control with 300ms watchdog safety system.

Project conventions and patterns
- Packaging: ament_cmake for C++ ROS nodes, ament_python for Python tools. Look for `package.xml` and `CMakeLists.txt` in each package.
- Linting & style: uncrustify / cpplint / cppcheck for C++; flake8 / pep257 / ruff for Python. Tasks are available in workspace tasks and helper scripts.
- Build helpers: prefer top-level scripts (`./build.sh`, `./setup.sh`) which configure env and wrap colcon; this repository keeps many CI/simulation flags inside those scripts.

Integration notes (what often trips newcomers)
- Use `src/ros2.repos` when reproducing workspace externally; it pins external dependencies used by CI/sim.
- Simulation uses Gazebo + gz_ros2_control — when editing controllers or ros2-control interfaces, validate using `start_sim_tmux.sh` to run both ROS nodes and Gazebo plugins together.
- Serial-related changes: unit tests are scarce; prefer running integration in the simulator (serial bridge in `mecabridge_hardware`) to avoid touching physical hardware.

Files & places to read first (examples)
- `src/mecabridge_hardware/` — hardware interface implementations and gz_ros2_control glue
- `src/my_steel_controller/` and `src/robot_controller/` — controller composition and parameter files
- `src/robot_gazebo/` — Gazebo world, robot launch files, simulation plugins
- `src/robot_firmware/lpico/` — Raspberry Pi Pico firmware with STM32 HAL compatibility layer
- `specs/001-description-build-firmware/` — firmware porting specifications, contracts, and design artifacts
- Top-level scripts: `build.sh`, `test.sh`, `setup.sh`, `start_sim_tmux.sh` — they encode common developer workflows

Where to add human notes
- Preserve manual notes between the markers below when merging.

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->

If any of this is unclear or you want deeper examples (for example, a sample flow to change a controller and validate it in sim), tell me which area and I'll expand this file.
# my_steel-robot_ws Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-09-21

## Active Technologies
- C++17 for ROS2 nodes, Python 3.8+ for tooling and scripts + ROS2 Humble, ros2-control, gz_ros2_control, mecanum_drive_controller, gazebo (001-feature-hal-mit)

## Project Structure
```
src/
tests/
```

## Commands
cd src [ONLY COMMANDS FOR ACTIVE TECHNOLOGIES][ONLY COMMANDS FOR ACTIVE TECHNOLOGIES] pytest [ONLY COMMANDS FOR ACTIVE TECHNOLOGIES][ONLY COMMANDS FOR ACTIVE TECHNOLOGIES] ruff check .

## Code Style
C++17 for ROS2 nodes, Python 3.8+ for tooling and scripts: Follow standard conventions


<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->