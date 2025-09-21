# Task List (TDD-first, dependency ordered)

Feature: HAL with Robot Simulation (Gazebo)
Feature dir: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit

Notes:
- [P] indicates tasks that can be executed in parallel where dependencies allow.
- Files referenced are repo-relative paths under the feature directory or `src/` when creating code.

T001 - ✅ COMPLETED: Setup: Workspace bootstrap and dependencies
 - Description: Ensure ROS 2 Humble environment and required packages are available for development and CI.
 - Actions:
   1. ✅ Document required system deps (ros2_control, ros2_controllers, gz_ros2_control) in feature README.
   2. ✅ Confirm dev container or developer machine can run Gazebo and ROS 2 Humble.
 - Path: /workspaces/my_steel-robot_ws/.github (update docs)
 - Depends on: none

T002 - ✅ COMPLETED: Setup: Create `src/mecabridge_hardware` ament_cmake package (scaffold)
 - Description: Add package skeleton with `package.xml`, `CMakeLists.txt`, basic folder layout and a placeholder `mecabridge_node`.
 - Actions:
   1. ✅ Create `src/mecabridge_hardware/package.xml` listing dependencies: rclcpp, hardware_interface, controller_manager, pluginlib, ros2_control, std_srvs.
   2. ✅ Create `src/mecabridge_hardware/CMakeLists.txt` with a target for `mecabridge_node` and the hardware interface library.
   3. ✅ Add `src/mecabridge_hardware/src/mecabridge_node.cpp` that reads parameter `mecabridge_hardware.mode` and logs initial state.
 - Path: /workspaces/my_steel-robot_ws/src/mecabridge_hardware
 - Depends on: T001

T003 [P] - 🔄 IN PROGRESS: Test: Contract test - SwitchMode service
 - Description: Add a failing contract test asserting `/mecabridge/switch_mode` exists with request `mode:string` and response `(success:bool, reason:string)`.
 - Actions:
   1. ✅ Create custom SwitchMode.srv service definition
   2. ✅ Update package.xml and CMakeLists.txt for message generation
   3. 🔄 Create comprehensive contract test for service schema and availability
 - File: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/contracts/tests/test_switch_mode.py
 - Runner: pytest (initial failing test)
 - Depends on: T002

T004 [P] - Test: Contract test - Enable service
 - Description: Add a failing contract test for `/mecabridge/enable` (std_srvs/SetBool) and expected error behaviour if hardware missing.
 - File: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/contracts/tests/test_enable_service.py
 - Depends on: T002

T005 [P] - Model: Implement data model stubs (Robot, HAL, ControllerManager, Wheel, ESC)
 - Description: Create C++ header(s) under `src/mecabridge_hardware/include/mecabridge_hardware/` defining structs/classes per `data-model.md`.
 - Files:
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/include/mecabridge_hardware/robot_state.hpp
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/include/mecabridge_hardware/hal_state.hpp
 - Depends on: T002

T006 - Core: Implement ros2_control hardware_interface skeleton
 - Description: Implement a hardware_interface::SystemInterface (C++) that exposes state and command interfaces for four mecanum wheels and supports `mode` switching between sim and hw backends.
 - Files:
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/src/mecabridge_hw_interface.cpp
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/include/mecabridge_hardware/mecabridge_hw_interface.hpp
 - Key behaviours: configure(), start(), stop(), read(), write(), safe-stop on mode switch
 - Depends on: T002, T005

T007 [P] - Test: Contract tests for telemetry & diagnostics topics
 - Description: Add failing tests asserting `/mecabridge/telemetry` and `/mecabridge/diagnostics` topics are published with expected message types.
 - File: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/contracts/tests/test_telemetry_diagnostics.py
 - Depends on: T006

T008 - Core: Implement `switch_mode` and `enable` services and wiring
 - Description: Implement `/mecabridge/switch_mode` and `/mecabridge/enable` services in `mecabridge_node` and ensure calls trigger hardware interface reconfiguration with safe-stop.
 - Files:
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/src/services.cpp
 - Depends on: T006, T005

T009 - Integration: Example controller config & controller_manager launch
 - Description: Add an example YAML for mecanum controller configuration and a launch file to spin `controller_manager` and load controllers bound to the hardware plugin.
 - Files:
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/config/mecanum_controllers.yaml
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/launch/mecabridge_controllers_launch.py
 - Depends on: T006

T010 - Integration: Gazebo URDF/xacro and sim launch (gz_ros2_control)
 - Description: Provide robot URDF/xacro with ros2_control transmissions and a Gazebo launch that loads gz_ros2_control plugin so the hardware plugin can operate in sim mode.
 - Files:
   - /workspaces/my_steel-robot_ws/src/robot/urdf/mecabridge_robot.urdf.xacro
   - /workspaces/my_steel-robot_ws/src/robot/launch/mecabridge_gazebo.launch.py
 - Depends on: T009

T011 [P] - Test: Integration test - Manual drive in Gazebo
 - Description: Automated test script that launches Gazebo + mecabridge in sim mode, publishes `/cmd_vel`, and verifies pose changes in telemetry.
 - File: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/tests/test_manual_drive_sim.py
 - Depends on: T010, T009, T006

T012 - Core: Teleop adapter wiring (Xbox)
 - Description: Integrate teleop (teleop_twist_joy or custom) and example launch so an Xbox controller can publish `/cmd_vel` to the system.
 - Files:
   - /workspaces/my_steel-robot_ws/src/robot/teleop/ (README + launch)
 - Depends on: T009, T006

T013 - Core: Watchdog & heartbeat implementation
 - Description: Implement watchdog between RPi and Pico with configurable `mecabridge_hardware.watchdog_timeout`, publish diagnostics on timeout and enforce safe disable.
 - Files:
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/src/watchdog.cpp
 - Depends on: T005, T006

T014 - HIL: Serial backend for hardware (RPi ↔ Pico)
 - Description: Implement a robust serial protocol with framing, CRC, and heartbeat. Provide a backend class selectable by the hw interface when `mode=hw`.
 - Files:
   - /workspaces/my_steel-robot_ws/src/mecabridge_hardware/src/serial_backend.cpp
 - CI note: HIL tests must be optional / env gated.
 - Depends on: T013, T006

T015 [P] - Test: HIL smoke test (optional hardware)
 - Description: Run smoke test that exercises ESC commands and encoder feedback on real hardware (or hardware-in-the-loop rig). Mark as optional in CI.
 - File: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/tests/test_hil_smoke.py
 - Depends on: T014

T016 - Docs: README & safety docs in package
 - Description: Add README to `src/mecabridge_hardware` explaining safety defaults, enable/disable, watchdog, and mode switching. Include constitution check summary in PR template.
 - File: /workspaces/my_steel-robot_ws/src/mecabridge_hardware/README.md
 - Depends on: T008, T013

T017 - CI: Add contract-test workflow and HIL gating
 - Description: Add a GitHub Actions workflow to run contract and integration tests on PRs; gate HIL tests behind an env var.
 - Files: .github/workflows/contract-tests.yml
 - Depends on: T003, T004, T007

T018 [P] - Polish: Unit tests and linters
 - Description: Add unit tests for the hardware interface logic and mode switching. Run linters (uncrustify/cpplint) and formatters in CI.
 - Files: tests/ under package and .github workflow updates
 - Depends on: T006, T008

T019 - Polish: Quickstart validation
 - Description: Validate `quickstart.md` steps end-to-end in sim and update commands or examples if needed.
 - File: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/quickstart.md
 - Depends on: T010, T011

T020 - Release prep: versioning & CHANGELOG
 - Description: Update `package.xml` version and add CHANGELOG entry describing the HAL integration and any migration notes.
 - Files: /workspaces/my_steel-robot_ws/src/mecabridge_hardware/package.xml, /workspaces/my_steel-robot_ws/CHANGELOG.md
 - Depends on: T016, T017

---

Parallel execution guidance:
- Run contract tests (T003, T004, T007) in parallel once basic package scaffolding and hw-interface stubs exist.
- Implement T005 (models) and T006 (hw interface) in parallel when different authors work on headers vs implementation.

Agent commands (examples an LLM can execute):
- "Create ament_cmake package skeleton at /workspaces/my_steel-robot_ws/src/mecabridge_hardware with package.xml and CMakeLists.txt."
- "Add class mecabridge_hardware::MecabridgeHWInterface with pluginlib export and implement configure/start/stop/read/write stubs."
- "Add failing pytest at specs/.../contracts/tests/test_switch_mode.py asserting service schema."
