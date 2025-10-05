# Schritt-für-Schritt Anleitung: Robot Hardware Setup

## PHASE 1: Raspberry Pi Setup

### 1. Dependencies installieren
```bash
cd /home/pi/workspace/ros2_dev_ws/my_steel-robot_ws

# Foxglove Bridge
sudo apt update
sudo apt install ros-humble-foxglove-bridge

# Camera driver (falls nicht vorhanden)
sudo apt install ros-humble-v4l2-camera

# Joy/Teleop (für Tests)
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

### 2. Workspace builden
```bash
./build.sh
```

### 3. Scripts ausführbar machen
```bash
chmod +x scripts/setup_robot_env.sh
chmod +x scripts/start_robot.sh
```

### 4. Robot starten
```bash
./scripts/start_robot.sh
```

Das startet:
- Micro-ROS Agent (Pico Kommunikation)
- Controller Manager + Mecanum Drive Controller
- Camera Node (v4l2_camera)
- Robot State Publisher (TF)
- IMU Broadcaster
- Foxglove Bridge (Port 8765)

---

## PHASE 2: Remote PC Setup

### 1. ROS2 Humble installieren (falls nicht vorhanden)
```bash
# Ubuntu 22.04
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Dependencies installieren
```bash
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

### 3. DDS Config kopieren
```bash
# Von Robot kopieren:
scp pi@<ROBOT_IP>:/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws/config/cyclone_dds.xml ~/
```

### 4. Environment setup
```bash
# Setup script vom Robot kopieren
scp pi@<ROBOT_IP>:/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws/scripts/remote_pc_setup_env.sh ~/

# Anpassen: ROBOT_IP eintragen
nano ~/remote_pc_setup_env.sh

# Sourcen
source ~/remote_pc_setup_env.sh
```

### 5. Joystick Launch kopieren
```bash
scp pi@<ROBOT_IP>:/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws/scripts/remote_pc_joystick.launch.py ~/
```

---

## PHASE 3: Testing & Betrieb

### 1. Network Connectivity Test
```bash
# Remote PC: Ping Robot
ping <ROBOT_IP>

# Remote PC: Check ROS2 nodes
source ~/remote_pc_setup_env.sh
ros2 node list

# Expected: Nodes vom Robot sollten sichtbar sein
```

### 2. Topic Discovery Test
```bash
# Remote PC: List topics
ros2 topic list

# Expected output:
# /camera/image_raw
# /imu/data_raw
# /joint_states
# /tf
# /tf_static
# /diff_drive_controller/cmd_vel_unstamped
# /range  (ToF sensor)
```

### 3. Sensor Data Test
```bash
# Remote PC: Check camera
ros2 topic hz /camera/image_raw

# Check IMU
ros2 topic echo /imu/data_raw

# Check ToF
ros2 topic echo /range
```

### 4. Controller Input starten
```bash
# Remote PC: Xbox Controller anschließen, dann:
source ~/remote_pc_setup_env.sh
ros2 launch ~/remote_pc_joystick.launch.py

# LB halten + Left Stick = Fahren
```

### 5. RViz2 starten
```bash
# Remote PC:
ros2 launch robot_description rviz.launch.py
```

### 6. Foxglove Studio
```bash
# Browser oder Foxglove App öffnen
# WebSocket URL: ws://<ROBOT_IP>:8765

# Oder Desktop App:
flatpak install flathub org.foxglove.studio
flatpak run org.foxglove.studio
```

---

## TROUBLESHOOTING

### Robot nicht sichtbar auf Remote PC
```bash
# Check ROS_DOMAIN_ID (muss gleich sein!)
echo $ROS_DOMAIN_ID  # Sollte 42 sein

# Check DDS discovery
ros2 daemon stop
ros2 daemon start
ros2 node list
```

### Kamera startet nicht
```bash
# Check device
ls -l /dev/v4l/by-id/

# Device anpassen in hardware_bringup.launch.py
```

### Pico nicht verbunden
```bash
# Check USB
ls -l /dev/ttyACM*

# Firmware flashen
cd firmware
make flash
```

### Foxglove verbindet nicht
```bash
# Check Port
sudo netstat -tulpn | grep 8765

# Firewall
sudo ufw allow 8765
```

---

## AUTO-START (Optional)

### Systemd Service
```bash
# Create service
sudo nano /etc/systemd/system/robot.service
```

```ini
[Unit]
Description=Robot ROS2 Hardware
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws
ExecStart=/home/pi/workspace/ros2_dev_ws/my_steel-robot_ws/scripts/start_robot.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Enable service
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service

# Check status
sudo systemctl status robot.service
```

---

## WICHTIGE TOPICS

| Topic | Type | Beschreibung |
|-------|------|--------------|
| `/cmd_vel` | Twist | Robot Bewegungskommandos |
| `/camera/image_raw` | Image | Kamera Bild |
| `/imu/data_raw` | Imu | IMU Daten (Beschleunigung, Gyro) |
| `/range` | Range | ToF Distance Sensor |
| `/joint_states` | JointState | Motor Encoder Positionen |
| `/tf` | TF2 | Transformationen |

---

## NETZWERK KONFIGURATION

### WiFi empfohlen
- 5GHz WiFi bevorzugen (weniger Interferenz)
- Static IP für Robot konfigurieren
- Router QoS für niedrige Latenz

### DDS Settings
- Multicast: Einfach, aber nicht in allen Netzwerken
- Unicast: Zuverlässiger, Peer-Liste konfigurieren

### Firewall Ports
- 7400-7500: DDS Discovery & Data
- 8765: Foxglove WebSocket
