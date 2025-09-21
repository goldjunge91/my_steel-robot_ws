# my_steel-robot_ws (based on Husarion ROSbot-XL structure)

Prerequisites
- Ubuntu 22.04 (or the ROS2 distro you use)
- ROS2 installed (Humble/… match your code)
- PI Pico toolchain (PlatformIO) for firmware flashing
- Python3, colcon, rosdep

````markdown
my_steel-robot_ws/
├── README.md
├── ros2.repos                # für `vcs import src < ros2.repos`
├── .github/
│   └── workflows/
│       └── firmware-ci.yml   # Firmware CI pipeline
├── firmware/                  # mecabridge_firmware (Pico) - separate repo recommended
│   ├── src/
│   ├── include/
│   ├── platformio.ini or Makefile
│   └── README.md
├── docs/
│   ├── PINMAP.md
│   ├── wiring_schematic.pdf
│   └── deployment.md
├── src/                      # (vcs import wird hierhin klonen)
│   ├── my_steel_description/   # URDF/xacro, meshes, components_config
│   ├── my_steel_bringup/       # launch files: bringup.launch.py, microros.launch.py
│   ├── my_steel_gazebo/       # Gazebo / Webots assets
│   ├── my_steel_controllers/   # controller YAMLs, controller plugins (if any)
│   ├── my_steel_hardware/      # ros2_control hardware_interface or bridge node
│   ├── my_steel_localization/  # ekf, sensor fusion config
│   ├── my_steel_vision/        # face detection, tracking
│   ├── my_steel_nerf_launcher/ # high-level nerf control, safety
│   ├── my_steel_power/         # INA3221 monitor node
│   ├── my_steel_utils/         # flash scripts, serial discovery, helpers scripts,
│   └── my_steel_autonomy/      # Nav2 config, docker-compose / foxglove UI docker/just, start  webui wrappers
└── .env.example               # for docker/just or .env options
```

Quickstart (example)
1) Build ROS workspace
   source /opt/ros/$ROS_DISTRO/setup.bash
   colcon build --symlink-install
   source install/setup.bash

TODO: diesen abschnitt anpassen
----------------------------------------
2) Flash firmware (on robot)
   # example wrapper (see firmware/ README)
   ./scripts/flash_firmware.sh --device /dev/ttyUSB0 --board my_steel

3) Start bringup
   ros2 launch robot_bridge bringup.launch.py robot_model:=my_steel mecanum:=False microros:=True serial_port:=/dev/ttyUSB0

Key config knobs (document these in README)
- ROBOT_MODEL_NAME: my_steel or my_steel_xl (affects controller selection)
- MECANUM: True/False
- MICROROS: True/False
- SERIAL_PORT / SERIAL_BAUDRATE or UDP PORT (for micro-ROS agent)
- CONTROLLER_CONFIG path: robot_bridge/config/{robot_model}/{mecanum|diff}_drive_controller.yaml

TODO: diesen abschnitt anpassen
----------------------------------------
Important files to include
- docs/PINMAP.md (table with MCU pin, signal, connector)
- robot_bridge/config/{robot_model}/diff_drive_controller.yaml
- robot_bridge/launch/bringup.launch.py
- mecabridge_hardware/src/hardware_interface.cpp (or python bridge node)
