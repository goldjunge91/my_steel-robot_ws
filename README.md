# my_steel-robot_ws

Workspace für my_steel Roboter Mecanum / Nerf Launcher robot.

Ziel
- Dokumentiere die Workspace-Struktur, Quickstart-Schritte und die wichtigsten Konfigurationsdateien.
- Fokus: Verwendung von ros2_control + dem vorhandenen mecanum_drive_controller (ros2_controllers).
- Modularer Aufbau, damit dieselbe Hardware‑Basis auch als Differential‑Antrieb betrieben werden kann.

```markdown
Workspace-Struktur
my_steel-robot_ws/
├── README.md
├── ros2.repos                # für `vcs import src < ros2.repos`
├── .github/
│   └── workflows/
│       └── firmware-ci.yml   # Firmware CI pipeline
├── robot_firmware/        # Firmware (Raspberry Pi Pico) - separater Repo empfohlen
│   ├── src/
│   ├── include/
│   ├── platformio.ini oder Makefile
│   └── README.md
├── docs/
│   ├── PINMAP.md
│   ├── wiring_schematic.pdf
│   └── deployment.md
├── src/                      # (vcs import wird hierhin klonen)
│   ├── robot_description/   # URDF/xacro, meshes, components_config
│   ├── robot_bringup/       # launches: bringup.launch.py, microros.launch.py
│   ├── robot_gazebo/        # Gazebo / Webots assets
│   ├── robot_controllers/   # controller YAMLs (mecanum/diff), ggf. plugins
│   ├── robot_hardware/      # ros2_control SystemInterface (hardware_interface) oder bridge node
│   ├── robot_localization/  # ekf, sensor fusion config
│   ├── robot_vision/        # face detection, tracking
│   ├── robot_nerf_launcher/ # high-level nerf control, safety
│   ├── robot_power/         # INA3221 monitor node
│   ├── robot_utils/         # flash scripts, serial discovery, helper scripts
│   └── robot_autonomy/      # Nav2 config, docker-compose / foxglove UI, just recipes
└── .env.example               # für docker/just oder .env Optionen
```


Designprinzipien
- Firmware ist eigene repo: robot_firmware → eigene CI (PlatformIO)
- Hardwareinterface (Host) ist C++ SystemInterface (ros2_control)
- Controller YAMLs referenzieren exakt die joint names aus URDF
- Bringup lädt den robot_description & ros2_control params aus package_share

## CLI-Automatisierung (Just)
- `just` - zeigt alle verfügbaren Rezepte
- `just setup-dev` - führt das Devcontainer-Setup-Skript aus
- `just build` - baut alle Pakete mit merge-install Layout
- `just build-hardware` - baut nur robot_hardware + robot samt Abhängigkeiten
- `just run-tests` - startet `colcon test` für Hardware und Robotik-Pakete
- `just start-gazebo-sim` - startet die Gazebo-Simulation mit mecanum Controllern
- `just start-sim-tmux` - nutzt `start_sim_tmux.sh` für eine tmux-basierte Sim-Session
- `just run-teleop` - startet `teleop_twist_keyboard` mit Remap auf `/mecanum_cont/cmd_vel_unstamped`
- `just shell` - öffnet eine Shell mit eingeblendeter ROS/Workspace-Umgebung
- `just clean` - entfernt `build/`, `install/` und `log/`
## Environment Configuration

Before building or deploying, configure your target platform in the `.env` file:

```bash
# Copy from .env.example
cp .env.example .env

# Edit .env to set your target
# TARGET=robot      # For Raspberry Pi SBC deployment
# TARGET=remote_pc  # For development PC with simulation (default)
```

### Target Platforms

- **`TARGET=robot`**: Configures for Raspberry Pi SBC deployment
  - Installs: ROS2 core, hardware interfaces, micro-ROS agent, Pico SDK, GPIO libraries
  - Use for: Robot hardware deployment, SBC setup

- **`TARGET=remote_pc`**: Configures for development workstation
  - Installs: ROS2 core, Gazebo simulation, MoveIt, RQT tools, joystick support
  - Use for: Development, simulation, remote operation

### Quick Setup

1. Set your target in `.env`
2. Run setup: `just setup-dev` or `./setup.sh`
3. Build: `just build`
4. Check configuration: `just check-target`


/mecabridge_hardware -> my_steel-hardware
(ros2_control hardware interface)

goldjunge91/my_steel-robot_ws -> my_steel-workspace (oder behalte my_steel-robot_ws)
(aktuelles Metapaket / root workspace, enthält ros2.repos, launch wrappers → bleibt Root-Workspace repo)

goldjunge91/robot -> wenn darin generischer Code/launch/URDF ist → splitten:

Teile mit URDF/XACRO → my_steel-description
Teile mit bringup → my_steel-bringup
sonst: falls generischer „robot“ nur eine alte Struktur ist, archivieren oder migrieren

Quickstart (Dokumentations-Flow)
1) Dependencies
   - Ubuntu 22.04 (oder passende ROS 2 Distro)
   - ROS 2 (Humble / Rolling / Version passend zu deinen Paketen)
   - PlatformIO (oder Pico toolchain) für Pico‑Firmware
   - Python3, colcon, rosdep

2) Workspace importieren
   - Lege `ros2.repos` im Root an und importiere:
     vcs import src < ros2.repos

3) Build
   source /opt/ros/$ROS_DISTRO/setup.bash
   colcon build --symlink-install
   source install/setup.bash

4) Firmware (falls benötigt)
   - Firmware liegt idealerweise in einem separaten Repo `robot_firmware` (oder als Submodule).
   - Folge der detaillierten Anleitung unter `src/robot_firmware/install.md` (SDK-Setup, Build, Flash, micro-ROS Agent).
   - Flash-Skript (in robot_utils) benutzen:
     ./scripts/flash_firmware.sh --device /dev/ttyUSB0 --board robot_pico

5) Bringup (SBC)
   - Beispiel:
     ros2 launch robot_bringup bringup.launch.py robot_model:=my_steel drive_type:=mecanum microros:=false serial_port:=/dev/ttyACM0
   - Hinweis: `drive_type` kann `mecanum` oder `diff` sein. Bei `mecanum` verwenden wir bevorzugt den vorhandenen ros2_controllers mecanum_drive_controller.

Raspberry Pi (SBC) setup checklist
----------------------------------
Use this checklist when preparing a Raspberry Pi as the SBC for the robot.

- OS: Raspberry Pi OS (64-bit) based on Debian/Ubuntu 22.04 (or a Ubuntu 22.04 image for Pi 4/5). Make sure kernel is up-to-date.
- Install ROS 2 Humble (or the ROS_DISTRO you selected). Follow the official ROS 2 installation for Debian/Ubuntu.
- Install system packages: python3-venv, python3-pip, python3-colcon-common-extensions, git, build-essential, python3-libgpiod, python3-pyudev, rosdep.
- Enable hardware interfaces if used: I2C (for IMU), SPI (if any), camera (v4l2) via raspi-config or config.txt overlays.
- Add udev rules: copy `scripts/udev_rules/99-my_steel.rules` to `/etc/udev/rules.d/` and run `sudo udevadm control --reload-rules && sudo udevadm trigger`.
- Optional: create `.env` in the workspace root (`.env.example` provided) and fill values such as ROS_DISTRO, ROBOT_MODEL_NAME, SERIAL_PORT, SERIAL_BAUDRATE.
- (If using Pico firmware) Install Pico SDK as documented in `setup_pico_sdk.sh` or run that script on the Pi (requires git and build tools).
- (If using micro-ROS) install micro-ROS agent (apt or pip) and create a systemd unit to run it or use the included launch file `robot_bringup/launch/microros_agent.launch.py`.
- (Optional) Add a systemd service: see `scripts/systemd/robot_bringup.service` as a template to run the bringup on boot (update paths & user).

Quick validation on the Pi after wiring and software install
----------------------------------------------------------
Run these to sanity-check the hardware and bringup before attempting to control the robot.

1) Check serial devices and permissions
   ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* /dev/gpiochip*
   udevadm info -a -n /dev/ttyUSB0

2) Check I2C devices (if IMU on the Pico or connected directly)
   sudo apt install -y i2c-tools
   sudo i2cdetect -y 1

3) Check ROS2 environment
   source /opt/ros/$ROS_DISTRO/setup.bash
   ros2 pkg list | grep robot_bringup || true

4) Try loading the URDF (xacro) to ensure xacro is available and the manipulator serial arg is set correctly
   source install/setup.bash
   ros2 launch robot_description load_urdf.launch.py robot_model:=robot_xl components_config:=$(pwd)/src/robot_description/config/robot_xl/basic.yaml

5) If using micro-ROS over serial, test connecting the agent manually and check port
   ros2 run micro_ros_agent micro_ros_agent serial --dev $SERIAL_PORT -b $SERIAL_BAUDRATE


Key config knobs (In README dokumentieren)
- ROBOT_MODEL_NAME: my_steel | robot_xl
- DRIVE_TYPE: mecanum | diff
- MICROROS: true | false
- SERIAL_PORT / SERIAL_BAUDRATE (oder UDP Port für micro-ROS Agent)
- CONTROLLER_CONFIG: path zu robot_controllers/{robot_model}/{mecanum|diff}_drive_controller.yaml

Wichtige Dateien / Orte
- docs/PINMAP.md — single source of truth für Board‑Pinouts (firmware authoritative)
- robot_controllers/config/{mecanum,diff}_drive_controller.yaml
- robot_bringup/launch/bringup.launch.py
- robot_hardware/src/... — ros2_control SystemInterface (oder bridge)
- .github/workflows/firmware-ci.yml — Firmware CI

Was in dieser Doku bewusst weggelassen wurde
- Keine Code‑Änderungen — nur Dokumentation.
- Firmware‑Details sind in `robot_firmware/README.md` zu halten (einheitlich mit PINMAP.md).

Tipps / Empfehlungen
- Halte joint‑Namen in URDF, controller YAMLs und hardware interface identisch.
- Starte in Simulation (robot_gazebo) bevor du echte Hardware anschließt.
- Nutze den vorhandenen mecanum_drive_controller (ros2_controllers) für mecanum; für diffdrive kannst du diff_drive_controller oder JointGroupVelocityController + cmdvel_to_wheels verwenden.

Kontakt / Maintainer
- Owner: @goldjunge91
- Repo: goldjunge91/my_steel-robot_ws
