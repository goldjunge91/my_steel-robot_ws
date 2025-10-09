```text
# my_steel Robot — Architektur, Packages & Verantwortlichkeiten
Ziel: Husarion‑ähnliche Gliederung für deinen omnidirektionalen ROS2‑Roboter (Mecanum, SBC = Raspberry Pi 4B / Jetson Nano; Digital board = Raspberry Pi Pico / STM32; Remote PC = Entwicklung / UI).

Empfohlener Root: goldjunge91/my_steel-robot_ws (root workspace)
```

```markdown
# Paket‑Matrix (Kurzbeschreibung, Rollen & Verantwortlichkeiten)

| Package (folder)     | Typ / Rolle            | Läuft auf      | Haupt‑Artefakte / Nodes                                  | Publishes / Subscribes (Beispiele)                  | Key files / Notes (Wichtig für Doku)                                |
| -------------------- | ---------------------- | -------------- | -------------------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------- |
| robot_firmware/      | Firmware (Pico)        | Digital‑Board  | micro-ROS client oder Serial-Firmware (PlatformIO/CMake) | encoder ticks, imu, battery                         | firmware/src, platformio.ini, docs/PINMAP.md (Autoritativ für Pins) |
| robot_hardware/      | Hardware‑Interface     | SBC            | ros2_control SystemInterface (C++ Plugin) oder Bridge    | /joint_states, /odom, /_motors_response             | src/* hardware plugin, config/ros2_control_params.yaml              |
| robot_description/   | Robot Description      | SBC / Sim      | URDF / XACRO / meshes                                    | TF, robot_description                               | urdf/*.xacro, components_config/                                    |
| robot_controllers/   | Controller Configs     | SBC            | YAMLs für mecanum_drive_controller & diff controller     | controllers publish /odom, /tf_odometry             | config/{robot_model}/{mecanum,diff}_drive_controller.yaml           |
| robot_bringup/       | Launch / Orchestration | SBC            | bringup.launch.py, spawners, microros agent wrapper      | startet ros2_control_node, spawner, micro-ROS agent | launch/*.py, params/                                                |
| robot_localization/  | Sensor Fusion / EKF    | SBC            | robot_localization configs                               | /odom (fused)                                       | config/ekf.yaml                                                     |
| robot_vision/        | Vision (optional)      | SBC/Jetson     | nodes: face_detector, tracker                            | /vision/target_pose                                 | models/, launch/                                                    |
| robot_nerf_launcher/ | High-level Controller  | SBC            | nerf_controller_node                                     | /nerf/fire_cmd, /nerf/servo_cmd                     | safety states, firing sequences                                     |
| robot_power/         | Power Monitor          | SBC / MCU      | power_monitor_node (INA3221)                             | /battery/status                                     | config/power.yaml                                                   |
| robot_utils/         | Tools / Scripts        | SBC / Dev PC   | flash_firmware, find_port, sync helpers                  | —                                                   | scripts/flash_firmware.sh, tools/find_port.py                       |
| robot_autonomy/      | Nav2 / Deployment      | SBC / Docker   | docker/just recipes, Nav2 configs                        | /map, /tf, /path                                    | docker-compose/*, justfile, .env                                    |
| robot_sim/           | Simulation             | Dev / Remote   | Gazebo/Webots assets & launches                          | simulated sensors (/scan, /odom)                    | worlds/, launch/sim*.launch.py                                      |
| docs/                | Dokumentation          | n/a            | PINMAP.md, wiring, deployment                            | —                                                   | PINMAP.md (single source of truth for pinout)                       |
| .github/             | CI / Workflows         | GitHub Actions | firmware-ci.yml, build pipelines                         | artifacts (firmware uf2/hex)                        | CI für Firmware und Paketchecks                                     |

----------------------------------------
3) Wichtige Topics & Schnittstellen (Datenfluss)
- Sensor → SBC:
  - LiDAR → /scan
  - IMU → /imu/data_raw
  - odom (wheel) → /odom/wheel (wenn HW Interface publisht)
- SBC → Digital Board (Pico):
  - /_motors_cmd (oder direkte Serial-Protokolle) → set wheel speeds
  - micro-ROS Agent (Serial/UDP) → bridged topics between SBC und MCU
- Digital Board → SBC:
  - encoder ticks, motor status, battery → /_motors_response, /battery/status
- High-level:
  - Nav2 → consumes /scan, /odom and publishes /cmd_vel
  - controller_manager handles controllers that command hardware interface

----------------------------------------
4) Launch / Config Patterns
- Bringup pattern:
  - bringup.launch.py:
    - lade robot_description (xacro)
    - wähle controller config: `{robot_model}/{mecanum|diff}_drive_controller.yaml`
    - starte ros2_control_node (controller_manager) mit config
    - spawn joint_state_broadcaster, imu_broadcaster, drive_controller
    - optional: starte micro-ROS Agent (args: microros, serial_port, baudrate)
- Controller YAMLs:
  - Lege sowohl mecanum und diff Varianten unter robot_controllers/config/ an.
  - Verwende das vorhandene `mecanum_drive_controller` (ros2_controllers) für Mecanum, konfiguriere wheel names & geometry.
  - DiffDrive: diff_drive_controller oder JointGroupVelocityController + cmdvel_to_wheels als Option.

----------------------------------------
5) Deployment & Workflow
- Empfehlung:
  - Firmware in eigenem Repo (robot_firmware) mit eigener CI (PlatformIO build).
  - Workspace nutzt `ros2.repos` für `vcs import` → einfach reproduzierbar.
  - Entwicklung Flow:
    1. Firmware build & flash (robot_firmware)
    2. colcon build (SBC packages)
    3. bringup auf SBC mit passenden launch-Args (drive_type, microros, serial_port)
    4. Starte Nav2 / SLAM (robot_autonomy)
    5. Visualisiere per Foxglove/RViz

----------------------------------------
6) Dateien die du erstellen / pflegen musst (Priorität)
- configs:
  - robot_controllers/config/my_steel/mecanum_drive_controller.yaml
  - robot_controllers/config/my_steel/diff_drive_controller.yaml
  - robot_hardware/config/ros2_control_params.yaml
- urdf/xacro:
  - robot_description/urdf/my_steel.urdf.xacro (mit ros2_control Block)
- bringup:
  - robot_bringup/launch/bringup.launch.py (Param: drive_type, microros, serial_port)
- firmware:
  - robot_firmware/docs/PINMAP.md (Pinmap authoritative)
  - flashing scripts in robot_utils/scripts/

----------------------------------------
7) Empfehlungen & Best Practices
- Halte joint names und interface‑Namen synchron zwischen URDF, controller YAML und hardware interface.
- Teste zuerst in Simulation (robot_sim).
- Nutze den vorhandenen mecanum_drive_controller (ros2_controllers) statt eigener Implementationen, wenn möglich.
- Firmware: implementiere watchdog & safe stop bei Verbindungsabbruch.
- Dokumentation: PINMAP.md ist Single Source of Truth für Pinbelegung; halte Firmware und docs synchron.

----------------------------------------
8) Quick Checkliste zum Portieren / Finalisieren (Dokumentation)
- [ ] PINMAP.md fertig und in firmware/docs referenziert
- [ ] controller YAMLs (mecanum + diff) vorhanden und in ros2_control params referenziert
- [ ] bringup.launch.py akzeptiert drive_type und lädt passende controller configs
- [ ] robot_hardware dokumentiert: Serial/USB Protokoll, expected topics/services, param list
- [ ] ros2.repos aktualisiert mit allen neuen/umbenannten Repos
```