```markdown
# my_steel Robot — Architektur, Packages & Verantwortlichkeiten
Ziel: Husarion‑ähnliche Gliederung für deinen omnidirektionalen ROS2‑Roboter (Mecanum, SBC = Raspberry Pi 4B / Jetson Nano; Digital board = Raspberry Pi Pico / STM32; Remote PC = Entwicklung / UI).

Empfohlener Root: goldjunge91/my_steel-robot_ws (root workspace)
- Aufbau: my_steel-robot_ws/
  - src/  (via `vcs import < src/ros2.repos`)
  - docs/
  - firmware/ (optional: eigenständiges repo empfohlen)
  - docker/ (autonomy containers)
  - .repos / ros2.repos (für `vcs import`)

Kurz-Glossar
- SBC: Raspberry Pi 4B (oder Jetson Nano)
- Digital‑Board: Raspberry Pi Pico (oder STM32 MCU) — führt Low‑Level Firmware / micro-ROS Client aus
- Remote PC: Laptop/Desktop für dev / UI / Foxglove

----------------------------------------
1) Empfohlene Ordnerstruktur (Kurz)
- my_steel-robot_ws/
  - src/
    - mecabridge_firmware/          # optional separate repo: firmware for Pico/STM32 (micro-ROS client)
    - mecabridge_hardware/          # HW bridge / ros2_control hardware interface or micro-ROS bridge
    - my_steel_description/         # URDF / xacro / meshes / components_config (physical parts)
    - my_steel_controller/          # ros2_control controller YAMLs + plugins (mecanum/diff)
    - my_steel_bringup/             # main bringup launches (aggregate), microros agent launch
    - my_steel_utils/               # helper scripts: flash_firmware, find_device_port, sync
    - my_steel_localization/        # ekf, sensor fusion configs
    - my_steel_teleop/              # teleop_twist_joy configs, xbox drivers
    - my_steel_autonomy/            # Nav2, SLAM toolbox, docker/just recipes, UI integrations (Foxglove)
    - my_steel_sim/                 # Gazebo / Webots worlds and launch wrappers
  - docs/
    - PINMAP.md
    - HW_Schematics.pdf (if available)
  - ros2.repos                       # for `vcs import` (referenced by workspace root)

----------------------------------------
2) Paket‑Matrix (Tabelle)
Hinweis: "Type" beschreibt Rolle; "Runs on" sagt SBC/DigitalBoard/Remote. "Primary Nodes" sind die wichtigsten node‑names oder packages; "Publishes / Subscribes" listet die wichtigsten Topics.

| Package (folder)       | Type                 | Runs on       | Primary nodes / executables                                              | Publishes (examples)                                        | Subscribes (examples)                                    | Key files / notes                                                      |
| ---------------------- | -------------------- | ------------- | ------------------------------------------------------------------------ | ----------------------------------------------------------- | -------------------------------------------------------- | ---------------------------------------------------------------------- |
| mecabridge_firmware/   | firmware (micro-ROS) | Digital-Board | micro_ros_client (firmware)                                              | (via micro-ROS) encoder_ticks, imu_raw, battery, digital IO | motors_cmd (from micro-ROS agent)                        | src/, include/, docs/PINMAP.md, flash scripts. Pinmap here.            |
| mecabridge_hardware/   | hw bridge / HAL      | SBC           | mecabridge_hw_node (ros2_control hardware_interface) or micro-ROS bridge | /joint_states, /odom, /imu/data_raw, /_motors_response      | /cmd_vel, /_motors_cmd (if using micro-ROS)              | config/controllers/*.yaml, src/hardware_interface.cpp or python bridge |
| my_steel_description/  | description (URDF)   | SBC / sim     | robot_state_publisher (xacro/URDF)                                       | TF, robot_description                                       | —                                                        | urdf/*.xacro, meshes/, components_config/                              |
| my_steel_controller/   | controller config    | SBC           | ros2_control_node (launched from bringup)                                | (controller nodes operate on hardware interfaces)           | (controllers subscribe to hardware interfaces / cmd_vel) | config/{mecanum,diff}_drive_controller.yaml                            |
| my_steel_bringup/      | bringup / launch     | SBC           | bringup.launch.py, controller.launch.py, microros.launch.py              | launches micro-ROS agent, ros2_control_node, spawners       | accepts launch args: robot_model, mecanum, microros      | launch/*.py, config/*.yaml                                             |
| my_steel_utils/        | tools / scripts      | SBC / Remote  | flash_firmware (wrapper), find_device_port, sync                         | —                                                           | —                                                        | scripts/flash_firmware.sh, tools/find_port.py                          |
| my_steel_localization/ | sensor fusion        | SBC           | robot_localization (ekf_node)                                            | /odom (fused), /tf                                          | /imu/data, /wheel/odometry, /mag (if present)            | config/ekf.yaml                                                        |
| my_steel_teleop/       | teleop / input       | SBC / Remote  | teleop_twist_joy, xbox_driver                                            | /cmd_vel                                                    | joystick inputs                                          | config/joy.yaml                                                        |
| my_steel_autonomy/     | autonomy / UI        | Remote / SBC  | nav2, slam_toolbox, mapping_node, foxglove bridge                        | /map, /tf, /scan, /path, /nav_msgs/…                        | /odom, /scan, /tf, /cmd_vel (planner output)             | docker-compose/*, justfile, .env                                       |
| my_steel_sim/          | simulation assets    | Remote / Dev  | gazebo launch or webots launch (sim wrapper)                             | simulated /scan, /odom, /camera topics                      | /cmd_vel                                                 | worlds/, launch/sim*.launch.py                                         |
| docs/                  | documentation        | N/A           | —                                                                        | —                                                           | —                                                        | PINMAP.md, README_ARCH.md, schematics                                  |

----------------------------------------
3) Wichtige Topics & Schnittstellen (Datenfluss)
- Sensor → SBC:
  - LiDAR: /scan (publisher on SBC if USB/Serial LIDAR) → SLAM/Nav2 subscribes
  - IMU: /imu/data_raw (publisher by mecabridge_hardware or micro-ROS agent)
  - Camera: /camera/image_raw (/camera/color/image_raw)
  - VL53L0X: optional short-range /tof_distance topic
- SBC → Digital Board:
  - cmd_vel (from controllers/planner) → mapped to low-level commands (/motors_cmd or via ros2_control HW interface)
  - micro-ROS transport: micro-ROS agent runs on SBC and bridges to MCU over serial/UDP; agent exposes topics/services to ROS2.
- Digital Board → SBC:
  - encoder ticks, raw odometry, motor state responses (/ _motors_response), ADC/battery readings
- High-level autonomy:
  - Nav2 / Planner reads /scan, /odom, /tf and outputs cmd_vel
  - UI (Foxglove) connects to ros2 topics via remote VPS / Husarnet / your VPN.

----------------------------------------
4) Launch / Config patterns (Husarion style)
- Bringup pattern:
  - bringup.launch.py:
    - loads robot_description (xacro)
    - sets controller_config path: `{robot_model}/{mecanum|diff}_drive_controller.yaml`
    - launches ros2_control_node with that config
    - spawns joint_state_broadcaster, imu_broadcaster, drive_controller
    - optionally launches micro-ROS agent (args: microros=True, serial_port, serial_baudrate, port)
- Controller YAMLs:
  - Provide both mecanum & diff files under `my_steel_controller/config/<robot_model>/`
  - Ensure `hardware_interface` resource names match URDF joint names and your hardware bridge.

----------------------------------------
5) Deployment & Workflow (recommended)
- Workspace root (my_steel-robot_ws) contains ros2.repos for `vcs import`.
- Development flow:
  1. Build firmware, flash Pico (mecabridge_firmware).
  2. Build ROS workspace: `colcon build --symlink-install`.
  3. On robot SBC: source & run `ros2 launch my_steel_bringup bringup.launch.py robot_model:=my_steel mecanum:=True microros:=True serial_port:=/dev/ttyUSB0`
  4. Start nav stack (docker/just): use my_steel_autonomy scripts (just start-robot).
  5. Visualize via Foxglove or RViz.

----------------------------------------
6) Files you must create / copy from Husarion
- controller launch pattern (controller.launch.py)
- microros launch wrapper (microros_agent.launch.py) with serial/udp args
- example controller YAMLs (mecanum/diff)
- URDF / xacro with components_config includes
- flash_firmware script (platformio / stm32cubeprogrammer wrapper)
- docs/PINMAP.md (firmware authoritative source)

----------------------------------------
7) Empfehlungen & Best Practices
- Firmware repository separate and versioned (tagged) — pin firmware version in my_steel_firmware/README and in my_steel_utils flash wrapper.
- Keep hardware names (joint names, interfaces) identical between URDF, controller YAMLs and hardware_interface.
- Provide a `HUSARION_ROS_BUILD_TYPE`-like env var: `MY_STEEL_ROS_BUILD_TYPE=hardware|simulation`.
- Start with Simulation (my_steel_sim) — validate controllers & URDF first.
- Keep PINMAP.md and schematics in docs/ and link them from all READMEs.

----------------------------------------
8) Weiteres — quick checklist zum Portieren (konkret)
- [ ] Extrahiere Pinmap aus Board / Firmware → docs/PINMAP.md
- [ ] Implementiere mecabridge_hardware: ros2_control hardware_interface or micro-ROS bridge
- [ ] Create controller YAMLs and test in simulation
- [ ] Provide bringup.launch.py that supports microros argument & controller selection
- [ ] Provide flash_firmware script and instructions in my_steel_utils
- [ ] Setup my_steel_autonomy with docker/just scripts for Nav2 and UI
```

Package: my_steel_firmware
Repo/Path: my_steel_firmware/  (separates repo, submodule optional)
Typ: Firmware (MCU)
Läuft auf: Digital‑Board (Raspberry Pico / alternativ STM32)
Hauptnodes: micro-ROS client (falls verfügbar) oder Serial firmware loop
Publishes: /_imu/data_raw (optional), /_encoders, /_motors_response, /battery/status (optional)
Subscribes: /_motors_cmd, /_servos_cmd, /_esc_cmd
Tätigkeit/Aufgabe: Echtzeit Steuerung: PWM für Motoren; Encoder Interrupts; IMU read; Servo/ESC control; safety cutoffs; implementiert micro-ROS client or simple serial protocol. Build: PlatformIO / CMake / Make. CI: cross-compile, create artifact, optional signed-firmware.

Package: my_steel_hardware
Repo/Path: src/my_steel_hardware/
Typ: ROS2 package (hardware_interface or bridge)
Läuft auf: SBC (Raspberry Pi 4B / Jetson)
Hauptnodes: my_steel_hardware_node (C++/Python)
Publishes: /joint_states, /odometry/wheels, /_motors_cmd (if bridging)
Subscribes: /cmd_vel (via controller), /_motors_response (from firmware)
Tätigkeit/Aufgabe: Implementiert ros2_control hardware_interface (preferred) or a bridge node that converts micro‑ROS/serial messages to ros2_control resource values. Provides standard resources names for controller YAMLs. Handles encoder -> odom, motor command translation.

Package: my_steel_controllers
Repo/Path: src/my_steel_controllers/  (mostly configs)
Typ: Config / ros2_control controller YAMLs
Läuft auf: SBC (used by ros2_control_node)
Hauptnodes: none (configs loaded by ros2_control_node)
Publishes: controllers publish to topics (e.g., odom)
Subscribes: cmd_vel
Tätigkeit/Aufgabe: Contain YAML templates: {robot_model}/{mecanum,diff}_drive_controller.yaml, joint_state_broadcaster, imu_broadcaster config. KEEP PATHS same as Husarion for compatibility.

Package: my_steel_bringup
Repo/Path: src/my_steel_bringup/
Typ: Launch/bringup package
Läuft auf: SBC
Hauptnodes: bringup.launch.py, controller.launch.py, microros.launch.py
Publishes: none (launch orchestration)
Subscribes: none
Tätigkeit/Aufgabe: Aggregate launch logic: start ros2_control_node with selected controller_config, spawn controllers, start micro-ROS Agent (serial/udp options), remappings (drive_controller/cmd_vel->cmd_vel), exposes params (robot_model, mecanum, microros, serial_port, serial_baudrate, port). MUST follow Husarion pattern.

Package: my_steel_description
Repo/Path: src/my_steel_description/
Typ: URDF/XACRO and meshes
Läuft auf: SBC / Simulation
Hauptnodes: robot_state_publisher (launched from bringup)
Publishes: /tf, /robot_description (parameter)
Subscribes: none
Tätigkeit/Aufgabe: Modular xacro with components_config (base, nerf_launcher, lidar, manipulator, etc.). Keep components_config pattern for optional modules.

Package: my_steel_localization
Repo/Path: src/my_steel_localization/
Typ: Nodes & configs
Läuft auf: SBC
Hauptnodes: ekf_node (robot_localization), filter nodes
Publishes: /odom (fused), /tf
Subscribes: sensor topics: /imu/data, /wheel/odom, /scan (for SLAM interfacing)
Tätigkeit/Aufgabe: Sensorfusion EKF, provide stable odom for Nav2. Keep parameter sets for tuning.

Package: my_steel_teleop
Repo/Path: src/my_steel_teleop/
Typ: Config / node
Läuft auf: SBC or Remote PC (if joystick connected)
Hauptnodes: teleop_twist_joy (with custom joy.yaml)
Publishes: /cmd_vel
Subscribes: /joy
Tätigkeit/Aufgabe: Xbox controller mapping for manual control.

Package: my_steel_vision
Repo/Path: src/my_steel_vision/
Typ: Node (python/C++)
Läuft auf: SBC (Pi4 or Jetson recommended)
Hauptnodes: face_detector_node, tracker_node
Publishes: /vision/target_pose, /vision/detections
Subscribes: camera/image_raw, /tf, /cmds (for arming)
Tätigkeit/Aufgabe: Camera capture, face detection, tracks target, publishes aim position to nerf launcher controller. On Jetson use optimized libs (TensorRT).

Package: my_steel_nerf
Repo/Path: src/my_steel_nerf/
Typ: High-level control node
Läuft auf: SBC (high-level safety, firing state machine)
Hauptnodes: nerf_controller_node
Publishes: /nerf/servo_cmd, /nerf/fire_cmd
Subscribes: /vision/target_pose, /safety/emergency_stop
Tätigkeit/Aufgabe: Translate target_pose to pan/tilt servo angles and firing sequences. Sends commands to firmware via /_servos_cmd or serial.

Package: lidar_driver (3rd party or wrapper)
Repo/Path: src/lidar_driver/ (or vendor)
Typ: Driver
Läuft auf: SBC
Hauptnodes: lidar_node
Publishes: /scan
Subscribes: none
Tätigkeit/Aufgabe: Serial/USB LiDAR driver (LDS01RR). Set LIDAR_BAUDRATE in .env or launch.

Package: imu_driver
Repo/Path: src/imu_driver/ or integrated in firmware
Typ: Driver
Läuft auf: Digital Board (preferred) or SBC
Hauptnodes: imu_node or firmware publishes imu via micro-ROS
Publishes: /imu/data_raw
Subscribes: none
Tätigkeit/Aufgabe: IMU reads, calibration, timestamping. If on MCU reduces jitter.

Package: my_steel_power
Repo/Path: src/my_steel_power/
Typ: Node
Läuft auf: SBC (I2C reads) or MCU
Hauptnodes: power_monitor_node
Publishes: /battery/status, /battery/cell_volts
Subscribes: none
Tätigkeit/Aufgabe: Read INA3221, publish battery voltage/current for telemetry and safe shutoff.

Package: my_steel_utils
Repo/Path: src/my_steel_utils/
Typ: Tools/Scripts
Läuft auf: SBC / Remote PC
Hauptnodes: scripts: flash_firmware, find_serial_port, sync (rsync wrapper) or just recipes
Publishes: none
Subscribes: none
Tätigkeit/Aufgabe: Helpers to flash firmware, discover serial devices, rsync sync to remote robot, start micro-ROS Agent wrapper. Mirror Husarion's just/flash-firmware.

Package: my_steel_autonomy
Repo/Path: my_steel_autonomy/ (can be a separate repo included by vcs)
Typ: Deployment / Docker / just
Läuft auf: SBC (or Docker host) or Remote PC
Hauptnodes: nav2_container, slam_toolbox, foxglove container or foxglove-websocket
Publishes: /map, /tf, /nav_topics
Subscribes: /scan, /odom, /cmd_vel (for control loop)
Tätigkeit/Aufgabe: Container orchestrations for Nav2/SLAM, Web UI. Provide `just` recipes: flash-firmware, start-rosbot, start-gazebo-sim, connect-vpn.

Package: web_ui (optional)
Repo/Path: my_steel_autonomy/ui/
Typ: Web container (foxglove/rosbridge wrapper)
Läuft auf: SBC (Docker) or Remote PC
Hauptnodes: foxglove‑websocket or custom frontend
Publishes/Subscribes: Webbridge to ROS topics for display/control
Tätigkeit/Aufgabe: Visualize maps, video stream, telemetry, teleop.

Package: my_steel_docs
Repo/Path: docs/
Typ: Documentation
Läuft auf: n/a
Hauptnodes: PINMAP.md, wiring diagrams, safety notes
Publishes: n/a
Subscribes: n/a
Tätigkeit/Aufgabe: Single source of truth for pinout, wiring, connector mapping, BOM.

Package: ci (workflow definitions)
Repo/Path: .github/workflows/
Typ: CI
Läuft auf: GitHub Actions
Hauptnodes: actions build firmware, build packages, run static checks
Publishes: build artifacts (firmware hex/uf2)
Subscribes: n/a
Tätigkeit/Aufgabe: Automate firmware build (PlatformIO/CMake), optionally run unit tests and store artifacts for download.