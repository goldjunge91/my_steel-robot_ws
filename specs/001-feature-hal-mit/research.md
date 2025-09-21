# Research: HAL with Robot Simulation (Gazebo)

Feature: Connect robot environment with `mecabridge_hardware` while preserving Gazebo remote simulation and manual drive.

Decision: Use ros2-control for hardware abstraction and provide a single HAL package `mecabridge_hardware` (ament_cmake) that implements the ros2_control hardware_interface and can be toggled between a simulated interface and a hardware interface at runtime via a ROS parameter or lifecycle state.

Rationale:
- ros2-control is the ROS 2 standard for hardware abstraction; using it ensures controller compatibility and follows the project's constitution.
- Implementing a hardware interface that can run in either "sim" or "hw" mode keeps a single code path and reduces duplication.
- Providing clear safety defaults (motors disabled by default, explicit enable) satisfies Safety & Fail-Safe Operation principles.

Alternatives considered:
- Separate simulation and hardware packages: Rejected because it increases maintenance and violates Modularity (single HAL package per hardware) principle.
- Using a custom adaptor outside ros2-control: Rejected due to duplication of existing control loops and reduced interoperability.

Dependencies & Tools to use:
- ROS 2 Humble (existing workspace assumed)
- ros2_control and ros2_controllers packages for mecanum controller
- Gazebo (simulator) with ros2_control gazebo plugins
- ament_cmake package for `mecabridge_hardware`

Open Questions (resolved):
- Q: How to switch between sim and hw without restarting?
  A: Expose a ROS parameter (`/mecabridge_hardware/mode`) and implement dynamic switching that tears down and reinitializes the hw_interface or swaps internal backends. Switching while moving must apply brakes and enter safe stop state first.

Open Questions (remaining):
- Edge-case behavior when hardware selected but device not present: Implement fallback to simulated backend with clear logging and diagnostic status; expose a diagnostic topic and ROS parameter to force safe-fallback.
## Unknowns
- ESC controller integration (effort/velocity interface for launcher motors)
- Network QoS tuning for ROS 2 over WiFi
- Time sync best practices for multi-tier ROS 2 deployments
- Watchdog/heartbeat protocol between RPi and Pico

## Research Tasks
- Best practices for ros2_control + gz_ros2_control in multi-tier setups
- Reliable serial protocol for RPi↔Pico (framing, CRC, heartbeat)
- CycloneDDS QoS config for mixed-reliability topics
- HIL test patterns for safety validation

## Findings (to be filled as research progresses)
- Decision: [TBD]
- Rationale: [TBD]
- Alternatives considered: [TBD]

Next steps (Phase 1):
- Define data model for Robot, Hardware Layer, and Simulation Environment (update `data-model.md`).
- Produce contracts for ROS interfaces and controller parameterization (create `/contracts/`).
- Write quickstart to launch Gazebo and start HAL in both sim and hw modes.
