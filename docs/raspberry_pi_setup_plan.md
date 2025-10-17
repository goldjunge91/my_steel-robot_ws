# my_steel Robot - Einfacher Start Guide

## Deployment-Optionen

Es gibt zwei Möglichkeiten, den Roboter auf dem Raspberry Pi zu betreiben:

1. **Docker Deployment (Empfohlen)**: Containerisierte Lösung mit allen Abhängigkeiten vorinstalliert
2. **Manuelle Installation**: Traditionelle Installation direkt auf dem System

### Option 1: Docker Deployment (Empfohlen)

**Vorteile:**
- Alle Abhängigkeiten vorinstalliert
- Einfache Updates durch neue Images
- Automatische Service-Orchestrierung
- Integrierte Tailscale VPN-Unterstützung
- Persistente Logs und Konfiguration

**Schnellstart:**
```bash
# Docker und Docker Compose installieren (falls nicht vorhanden)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Neu einloggen für Gruppenänderung

# Docker Compose Konfiguration kopieren
cd ~/my_steel-robot_ws
cp docker/compose.robot-pi.yaml ~/compose.robot-pi.yaml

# Tailscale konfigurieren (optional, für Remote-Zugriff)
cp docker/.env.robot-pi.example ~/.env
# ~/.env bearbeiten und TAILSCALE_AUTHKEY hinzufügen

# Verzeichnisse für Logs und Konfiguration erstellen
sudo mkdir -p /var/log/robot /etc/robot /var/lib/tailscale
sudo chown -R $USER:$USER /var/log/robot /etc/robot /var/lib/tailscale

# Image herunterladen
docker pull mysteel/robot:humble-arm64

# Roboter starten
docker compose -f ~/compose.robot-pi.yaml up -d

# Status prüfen
docker compose -f ~/compose.robot-pi.yaml ps

# Logs anzeigen
docker compose -f ~/compose.robot-pi.yaml logs -f
```

**Systemd Integration (Autostart):**
```bash
# Service-Datei kopieren
sudo cp docker/robot-docker.service /etc/systemd/system/

# Service aktivieren
sudo systemctl daemon-reload
sudo systemctl enable robot-docker.service
sudo systemctl start robot-docker.service

# Status prüfen
sudo systemctl status robot-docker.service
```

**Weitere Informationen:**
- Siehe [docker/README.md](../docker/README.md) für vollständige Dokumentation
- Build-Anleitung, Konfigurationsoptionen und erweiterte Troubleshooting

### Option 2: Manuelle Installation

## Schnellstart - Alles ist installiert

**WICHTIG: Probleme beheben vor dem Start!**

### Schritt 0: Kritische Fixes

```bash
# 1. Fehlende Pakete installieren
sudo apt update
sudo apt install ros-humble-micro-ros-agent ros-humble-usb-cam -y

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

### Docker-spezifische Probleme:

#### Container startet nicht
```bash
# Container-Status prüfen
docker compose -f ~/compose.robot-pi.yaml ps

# Detaillierte Logs anzeigen
docker compose -f ~/compose.robot-pi.yaml logs

# Einzelne Services prüfen
docker compose -f ~/compose.robot-pi.yaml logs microros-agent
docker compose -f ~/compose.robot-pi.yaml logs robot-bringup
```

#### USB-Gerät nicht gefunden (/dev/ttyACM0)
```bash
# Pico-Verbindung prüfen
ls -l /dev/ttyACM*

# Container neu starten (erkennt Geräte neu)
docker compose -f ~/compose.robot-pi.yaml restart

# Berechtigungen prüfen
sudo usermod -aG dialout $USER
# Neu einloggen erforderlich
```

#### Health Checks schlagen fehl
```bash
# Health Check Status prüfen
docker inspect microros-agent | grep -A 10 Health
docker inspect robot-bringup | grep -A 10 Health

# In Container einloggen für Debugging
docker exec -it microros-agent bash
docker exec -it robot-bringup bash

# Innerhalb des Containers:
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 control list_controllers
```

#### Tailscale verbindet nicht
```bash
# Tailscale-Status im Container prüfen
docker exec robot-bringup tailscale status

# Auth Key prüfen
cat ~/.env | grep TAILSCALE_AUTHKEY

# Tailscale-Logs anzeigen
docker compose -f ~/compose.robot-pi.yaml logs robot-bringup | grep tailscale

# Tailscale-State zurücksetzen
sudo rm -rf /var/lib/tailscale/*
docker compose -f ~/compose.robot-pi.yaml restart robot-bringup
```

#### Container-Updates
```bash
# Neues Image herunterladen
docker pull mysteel/robot:humble-arm64

# Container mit neuem Image neu erstellen
docker compose -f ~/compose.robot-pi.yaml up -d --force-recreate

# Alte Images aufräumen
docker image prune -a
```

#### Logs und Debugging
```bash
# Alle Logs anzeigen
docker compose -f ~/compose.robot-pi.yaml logs -f

# Logs eines bestimmten Services
docker compose -f ~/compose.robot-pi.yaml logs -f microros-agent

# Logs auf Host-System
tail -f /var/log/robot/*.log

# Container-Ressourcennutzung
docker stats
```

### Manuelle Installation - Häufige Probleme:
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

### Weitere Dokumentation:
- **Docker Deployment**: Siehe [docker/README.md](../docker/README.md)
- **Allgemeine Architektur**: Siehe [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md)
- **Hardware Setup**: Siehe [hardware_setup.md](hardware_setup.md)