# Implementation Plan: HAL mit Robot Simulation (Gazebo + ros2_control Mecanum)

**Branch**: 001-feature-hal-mit  
**Spec**: /workspaces/my_steel-robot_ws/specs/001-feature-hal-mit/spec.md  
**Created**: 2025-09-21  
**Status**: Draft

## 1. Summary
Provide unified simulation + hardware abstraction enabling seamless switching between Gazebo simulation (gz_ros2_control) and physical hardware via `mecabridge_hardware`, using `mecanum_drive_controller` for omnidirectional motion. Ensures reproducible kinematics, consistent controller configuration, and safety-aligned operation.

## 2. Goals
- Full ros2_control integration (simulation + hardware) with mecanum kinematics
- Single source of truth for wheel geometry + control limits reused by both packages
- Launch workflows: sim-only, hardware-only, hybrid (teleop in sim while hardware offline)
- Deterministic controller startup (avoid duplicate spawners / conflicts)
- Teleop + autonomy (nav2 later) foundation with clean interface boundaries

## 3. Non-Goals
- Navigation stack (nav2) tuning and behavior trees (future feature)
- Advanced perception pipelines (face detection details)
- Custom controllers beyond standard `mecanum_drive_controller`
- Power management / battery analytics

## 4. Current State (Findings)
- `mecabridge_hardware` package includes preliminary `mecabridge_hardware_interface.cpp` with wheel command mapping placeholders.
- Tests already reference `mecanum_drive_controller` (integration launch + controller YAML).
- `robot` package contains Gazebo worlds, controller YAMLs (`my_controllers_mecanum*.yaml`), and `ros2_control.xacro` embedding `<plugin name="gazebo_ros2_control" ...>`.
- Duplicate / legacy files (e.g. `launch_sim.launch_old.py`, `gazebo_control_old.xacro`).
- Multiple controller YAML variants → risk of config drift.
- No unified parameter export between hardware and sim (e.g., wheel radius, separation, inversion flags).
- Safety (E-stop / enable) path unclear in HAL interface.

## 5. Target Architecture
```
+----------------------+        +-------------------------+
|  robot (simulation)  |        | mecabridge_hardware     |
|  - URDF/Xacro        |        |  - HardwareInterface    |
|  - ros2_control.xacro|        |  - Serial protocol      |
|  - gz_ros2_control   |        |  - Wheel abstraction    |
+-----------+----------+        +-----------+-------------+
            |                               |
            |  /cmd_vel (Twist)             |
            v                               v
        mecanum_drive_controller (shared YAML config)
                 |                      |
                 +---- joint_state_broadcaster
                 |
           /wheel_joint velocity cmds
                 |
     +-----------+------------+
     |  Sim joints (Gazebo)   | OR | Physical motors (Pico)
     +------------------------+    +-----------------------+
```

## 6. Key Design Decisions
| Area           | Decision                                                                       | Rationale                            |
| -------------- | ------------------------------------------------------------------------------ | ------------------------------------ |
| Controller     | Use `mecanum_drive_controller`                                                 | Standard, tested, avoids custom math |
| Config mgmt    | Single YAML (e.g. `config/mecanum_controllers.yaml`) consumed by both packages | Eliminates drift                     |
| Geometry       | Export wheel params via Xacro + generate shared YAML via script                | Consistency                          |
| Mode switching | Separate launch files; no runtime dynamic swap initially (restart nodes)       | Simplicity & reliability             |
| Safety         | Add enable/disable service in HAL + timeout for stale velocity commands        | Constitution compliance              |
| Testing        | Use Gazebo + integration tests with `ros2 control list_controllers` assertions | Regression safety                    |

## 7. Risks & Mitigations
| Risk                                    | Impact                        | Mitigation                                                  |
| --------------------------------------- | ----------------------------- | ----------------------------------------------------------- |
| Controller param mismatch               | Controller fails to configure | Add schema validation test parsing YAML                     |
| Drift between sim & hardware kinematics | Different motion behavior     | Central param file + generation script                      |
| Serial latency / jitter                 | Motion instability            | HAL smoothing & watchdog (already partially present)        |
| Duplicate controller spawning           | Crash / STRICT switch failure | Ensure only plugin loads controllers; remove extra spawners |
| Incorrect wheel TF frames               | Localization errors           | Add TF sanity test (frame tree + static transforms)         |

## 8. Milestones
1. Erstellen der vereinheitlichten `mecanum_controllers.yaml` (vollständige ros2_control `mecanum_drive_controller` Parameter – keine vereinfachte wheels{} oder command_interfaces Platzhalter) + Entfernen alter Duplikate
2. Implement shared wheel param export (Xacro -> YAML generation script)
3. Harden `mecabridge_hardware_interface` (timeouts, enable, scaling)
4. Gazebo launch cleanup (single entrypoint, remove old file)
5. Add integration test: spawn sim, verify controller active, publish `/cmd_vel` → wheel joint state changes
6. Add hardware-side test stub (mock serial) verifying command mapping
7. Documentation: README updates + quickstart run scripts

## 9. Decisions (Previously Open Questions Resolved)
1. Command Interface: **Velocity only (`hardware_interface::HW_IF_VELOCITY`)**. Rationale: Matches `mecanum_drive_controller` native expectations; keeps HardwareInterface minimal (KISS). Effort/torque concerns delegated to motor driver firmware / ESC closed-loop.
2. Runtime Mode Switch: **No hot-swap in initial implementation**. Two explicit launch paths: `*_sim.launch.py` (Gazebo) and `*_hardware.launch.py` (Pico hardware). Switching requires restarting relevant launch (DRY by sharing a common include for shared nodes).
3. Wheel Geometry Source: **Hybrid (CAD defaults + optional runtime override YAML)**. Resolution order: (a) Override file (if provided & valid) > (b) Generated CAD default YAML > (c) Fallback constants embedded in Xacro (last resort). This supports reproducibility + field calibration without code changes.

## 10. Success Criteria
- `ros2 control list_controllers` shows `mecanum_drive_controller (active)` and `joint_state_broadcaster (active)` in both sim and hardware test mode
- Publishing `cmd_vel` linear.x, angular.z results in correct signed wheel velocity pattern
- Single controller YAML referenced in both packages; no duplicate definitions remain
- HAL returns to safe (zero command) within timeout after last command
- All tests (unit + integration) pass in CI

## 11. Work Breakdown (High-Level)
- Config consolidation
- HAL enhancements (timeouts, enable service, scaling audit)
- Simulation launch + URDF audit
- Shared param generation
- Tests (unit, integration, Gazebo sim) 
- Docs & developer quickstart

Progress Tracking: Phase scaffolding will be captured in tasks.md.

## 12. API & Configuration Contracts

This section defines the stable interfaces introduced or relied upon by this feature (KISS + DRY oriented). Deviations require updating this plan and CHANGELOG entries in affected packages.

### 12.1 Hardware Interface (mecabridge_hardware)
Exports (per wheel joint):
- Command Interfaces: `velocity` (name: `wheel_<fl|fr|rl|rr>_joint` matching URDF)
- State Interfaces: `position`, `velocity` (optional future: `effort` if upstream hardware supplies it)

Command Timeout Behavior:
- Parameter: `~command_stale_timeout_ms` (int, default: 500)
- If (now - last_cmd_time) > timeout: all wheel velocity commands forced to 0.0 on next `write()` cycle.

Enable / Disable:
- Service: `~/enable` (`std_srvs/SetBool`)
      - true → normal operation
      - false → immediately zero commands + suppress further non-zero writes
- Latched internal state is exposed via ROS parameter `~enabled` (read-only at runtime) for observability.

### 12.2 Topics
| Topic | Type | Direction | Notes |
|-------|------|-----------|-------|
| `/cmd_vel` | `geometry_msgs/Twist` (or `TwistStamped`) | Inbound to controller | Consumed by `mecanum_drive_controller` (remap via launch arg if needed) |
| `/joint_states` | `sensor_msgs/JointState` | Outbound | Published by `joint_state_broadcaster` |
| `/tf`, `/tf_static` | TF | Outbound | Standard frame tree (base_link, wheel joints) |
| `~/diagnostics` (optional future) | `diagnostic_msgs/DiagnosticArray` | Outbound | HAL health & watchdog timing |

### 12.3 Services
| Service | Type | Provider | Purpose |
|---------|------|----------|---------|
| `mecabridge_hardware/enable` | `std_srvs/SetBool` | HAL | Enable/disable actuator outputs |
| `controller_manager/list_controllers` | builtin | controller_manager | Introspection (validation tests) |

### 12.4 Parameters (Grouped)
Hardware Node (`mecabridge_hardware`):
| Name | Type | Default | Description |
|------|------|---------|-------------|
| `command_stale_timeout_ms` | int | 500 | Zero outputs after inactivity (safety) |
| `wheel_radius` | double | from geometry resolver | Effective rolling radius (m) |
| `wheel_separation_x` | double | from geometry resolver | Lateral (left-right) half-separation or full span (defined below) |
| `wheel_separation_y` | double | from geometry resolver | Longitudinal (front-rear) half-separation or full span (defined below) |
| `velocity_limit` | double | 3.0 | Max wheel linear surface speed (m/s) |
| `accel_limit` | double | 8.0 | (Optional future) ramp limiting |
| `enabled` | bool | true | Current enable state (read-only after start) |

Controller YAML (`mecanum_controllers.yaml`) – Autoritative (vollständig, keine vereinfachte Struktur):
```
# BLOCK 1: Controller Manager Deklarationen
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz; kann via Launch Override angepasst werden

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController

# BLOCK 2: Mecanum Drive Controller Parameter (Upstream Schema Humble / rolling docs identisch)
mecanum_drive_controller:
  ros__parameters:
    # Sicherheits- / Referenz-Timeout in Sekunden (0.0 würde jede Zyklus-Nullung erzwingen – vermeiden)
    reference_timeout: 0.5

    # Wheel command joint names (velocity command interfaces)
    front_left_wheel_command_joint_name:  "front_left_wheel_joint"
    front_right_wheel_command_joint_name: "front_right_wheel_joint"
    rear_right_wheel_command_joint_name:  "rear_right_wheel_joint"
    rear_left_wheel_command_joint_name:   "rear_left_wheel_joint"

    # Optional: State joints (hier identisch, explizit für Klarheit)
    front_left_wheel_state_joint_name:  "front_left_wheel_joint"
    front_right_wheel_state_joint_name: "front_right_wheel_joint"
    rear_right_wheel_state_joint_name:  "rear_right_wheel_joint"
    rear_left_wheel_state_joint_name:   "rear_left_wheel_joint"

    # Kinematik – Controller erwartet radius + Summe (lx + ly). Lx/Ly (Half-Separations) werden intern dokumentiert,
    # können für Tests/Validierung zusätzlich im Hardware-Paket gehalten werden.
    kinematics:
      base_frame_offset: { x: 0.0, y: 0.0, theta: 0.0 }
      wheels_radius: 0.033
      sum_of_robot_center_projection_on_X_Y_axis: 0.2685  # = |lx| + |ly|

    # Frames & TF
    base_frame_id: base_link
    odom_frame_id: odom
    enable_odom_tf: true
    tf_frame_prefix_enable: true
    tf_frame_prefix: ""

    # Eingabemodus: false => /<controller>/cmd_vel (geometry_msgs/Twist), true => /<controller>/reference (TwistStamped)
    use_stamped_vel: false

    # Kovarianzen (Standardwerte – später kalibrierbar)
    pose_covariance_diagonal:  [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    twist_covariance_diagonal: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
```
Erläuterungen:
- Keine `command_interfaces`, keine vereinfachte `wheels:` Map – diese gehören nicht zum offiziellen Parameter-Schema.
- Geometrie-Auflösung: `wheels_radius` und `sum_of_robot_center_projection_on_X_Y_axis` stammen aus Hybrid-Resolver (vgl. 12.5). Separate Lx/Ly können im Hardware-Paket zur Validierung geführt werden; Controller benötigt nur die Summe.
- Update-Rate konservativ 100 Hz gewählt; Simulation kann höher, Hardware ggf. niedriger – Launch Override nutzen.

### 12.5 Geometry Resolution (Hybrid Strategy)
Resolution Order:
1. Override file (launch arg: `geometry_override_file:=<path>`). If provided & passes validation test.
2. Generated default YAML (e.g., `config/mecanum_geometry_default.yaml`) produced from CAD-linked Xacro macro.
3. Hardcoded fallback constants in Xacro macro (only used to build the default YAML initially).

Validation Rules:
- `wheel_radius > 0`
- `wheel_separation_x > 0`, `wheel_separation_y > 0`
- Optional tolerance: difference between override and default logged if >5% for awareness.

### 12.6 File & Directory Patterns
| File | Purpose |
|------|---------|
| `robot/config/mecanum_controllers.yaml` | Canonical controller + kinematics config (shared) |
| `robot/config/mecanum_geometry_default.yaml` | Generated from CAD (checked into repo) |
| `robot/config/mecanum_geometry_override.example.yaml` | Template for field calibration |
| `mecabridge_hardware/config/mecanum_controllers.yaml` | Symlink or copy (avoid drift) |
| `scripts/generate_mecanum_params.py` | Emits default geometry YAML from Xacro |
| `launch/mecanum_sim.launch.py` | Simulation entrypoint |
| `launch/mecanum_hardware.launch.py` | Hardware entrypoint |
| `launch/_include/mecanum_common.launch.py` | Shared nodes (state publisher, teleop optional) |

DRY Strategy:
- Shared controller YAML physically located once (in `robot/`) and referenced (symlink or installed) by HAL package.
- Common launch include prevents duplicated node argument logic.

### 12.7 Launch Interface
Simulation Launch (`mecanum_sim.launch.py`) Args:
| Arg | Default | Description |
|-----|---------|-------------|
| `world` | `empty` | Gazebo world name / path |
| `headless` | `false` | Skip GUI |
| `geometry_override_file` | "" | Optional absolute/relative path |
| `use_sim_time` | `true` | Set `/use_sim_time` |
| `teleop` | `false` | Start joystick teleop node |

Hardware Launch (`mecanum_hardware.launch.py`) Args:
| Arg | Default | Description |
|-----|---------|-------------|
| `port` | `/dev/ttyACM0` | Serial device |
| `baud` | `115200` | Serial baud rate |
| `geometry_override_file` | "" | Same resolution logic |
| `enable_on_start` | `true` | Call enable service after bring-up |

### 12.8 Twist → Wheel Velocity Mapping (Reference)
Wheel velocity commands (rad/s) derived from body twist:
```
Given body twist (vx, vy, wz), wheel radius r, half-separations Lx (x), Ly (y):

FL = (1/r) * ( vx - vy - (Lx + Ly) * wz )
FR = (1/r) * ( vx + vy + (Lx + Ly) * wz )
RL = (1/r) * ( vx + vy - (Lx + Ly) * wz )
RR = (1/r) * ( vx - vy + (Lx + Ly) * wz )
```
Note: Confirm sign conventions with URDF joint axis orientation. Include unit test asserting pattern for a sample twist (vx=1.0, vy=0, wz=0 → all equal positive; vy=1.0 → FL negative, FR positive, RL positive, RR negative given standard axes).

### 12.9 Test Contracts (Must Remain Valid)
| Test | Assertion |
|------|-----------|
| Geometry validation | All geometry params > 0; override differences logged |
| Controller active | `mecanum_drive_controller` & `joint_state_broadcaster` active within timeout |
| Watchdog | Wheel commands zeroed after timeout |
| Enable toggle | Disabling halts motion, enabling resumes |
| Wheel mapping | Known Twist produces expected wheel sign pattern |

### 12.10 Extensibility Notes
- Adding effort interface later: extend HardwareInterface to export additional command interface but keep velocity primary; update controller YAML only if controller variant demands it.
- Runtime switching (future): Introduce a higher-level multiplexer node; current design isolates concerns to keep complexity low.

---
This API section is now the authoritative contract for implementation tasks and future reviews.
