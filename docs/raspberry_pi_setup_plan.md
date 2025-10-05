# my_steel Robot - Einfacher Start Guide

## Schnellstart - Alles ist installiert

**WICHTIG: Probleme beheben vor dem Start!**

### Schritt 0: Kritische Fixes

```bash
# 1. micro-ROS Agent installieren
sudo apt update
sudo apt install ros-humble-micro-ros-agent -y

# 2. CycloneDDS Konfiguration deaktivieren (temporär)
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. .env Datei für Raspberry Pi anpassen:
# TARGET=robot              # (aktuell: remote_pc)
# ROBOT_MODEL_NAME=robot_xl ist bereits korrekt
```

### 1. Raspberry Pi starten

```bash
# ROS2 Environment laden
source /opt/ros/humble/setup.bash
cd ~/my_steel-robot_ws

# Wichtige Environment-Variablen setzen
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDX_URI

# WICHTIG: Nach Plugin-Fix neu bauen
colcon build --symlink-install
source install/setup.bash

# Zuerst micro-ROS Agent separat starten (MUSS laufen!)
# Terminal 1:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# Dann in Terminal 2:
# Basis-System starten 
ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl mecanum:=True microros:=False
```

### 2. Raspberry Pi System starten (Terminal 2)

```bash
# ROS2 Environment laden
source /opt/ros/humble/setup.bash
cd ~/my_steel-robot_ws

# Wichtige Environment-Variablen setzen
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset CYCLONEDX_URI

# Workspace laden
source install/setup.bash

# Basis-System starten 
ros2 launch robot_bringup bringup.launch.py robot_model:=robot_xl mecanum:=True microros:=False
```

### 3. Foxglove Bridge starten (Terminal 3)

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

# Foxglove Bridge starten
ros2 run foxglove_bridge foxglove_bridge --port 8765
```

### 4. Remote-PC Setup

```bash
# ROS2 Environment
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Joy-Node starten (Terminal 1)
ros2 run joy joy_node

# Controller für Steuerung (Terminal 2)
ros2 run teleop_twist_joy teleop_node --ros-args -p joy_config:=xbox
```

### 5. Foxglove Studio verbinden

1. Foxglove Studio öffnen
2. "Open connection" → "Foxglove WebSocket" 
3. URL: `ws://[RASPBERRY_PI_IP]:8765`
4. Connect

### 6. RViz2 auf Remote-PC (optional)

```bash
ros2 run rviz2 rviz2
```

## Autostart Setup (Optional)

Für automatischen Start beim Neustart:

```bash
# Alle Services installieren
cd ~/my_steel-robot_ws
chmod +x scripts/setup_autostart.sh
bash scripts/setup_autostart.sh

# Services manuell steuern:
sudo systemctl start microros-agent     # micro-ROS Agent
sudo systemctl start my-steel-robot     # Hauptsystem  
sudo systemctl start foxglove-bridge    # Web Interface

# Status prüfen:
sudo systemctl status microros-agent
sudo systemctl status my-steel-robot
sudo systemctl status foxglove-bridge

# Logs anzeigen:
journalctl -u microros-agent -f
journalctl -u my-steel-robot -f
journalctl -u foxglove-bridge -f
```

## Troubleshooting

### Häufige Probleme:
- **CycloneDDS Fehler**: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` verwenden
- **micro_ros_agent not found**: `sudo apt install ros-humble-micro-ros-agent`
- **Socket buffer size**: CycloneDDS Config deaktivieren
- **Controller manager conflicts**: ROS2 Humble Version prüfen
- **Keine Topics sichtbar**: ROS_DOMAIN_ID=0 setzen
- **Foxglove verbindet nicht**: IP-Adresse und Port 8765 prüfen

### Debug-Befehle:
```bash
ros2 topic list
ros2 node list  
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```