# ROS Interfaces & Contracts

This file lists the ROS topics, services, parameters and actions that the `mecabridge_hardware` HAL exposes or depends on.

Topics:
- `/mecabridge/telemetry` (publisher) : robot telemetry messages (pose, velocity, battery)
- `/mecabridge/diagnostics` (publisher) : diagnostic messages with hardware status
- `/cmd_vel` (subscriber) : geometry_msgs/Twist for teleop and high-level commands

Services:
- `/mecabridge/enable` (srv) : enable/disable actuators (std_srvs/SetBool) — toggles HAL enabled state
- `/mecabridge/switch_mode` (srv) : switch between sim and hw backends (custom service SwitchMode)

Parameters:
- `mecabridge_hardware.mode` : string, 'sim' or 'hw' (runtime switchable)
- `mecabridge_hardware.watchdog_timeout` : float seconds

Action/Controller Contracts:
- Mecanum controller expects ros2_control interfaces: velocity/position state and command interfaces for four wheels.

Failure Modes & Expected Behavior:
- If hardware not present in `hw` mode: service `/mecabridge/enable` returns false and diagnostics publishes FAULT with reason. HAL should optionally fall back to `sim` mode if `fallback_on_failure` parameter is true.
