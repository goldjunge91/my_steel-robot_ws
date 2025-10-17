# my_steel Robot - Technischer Stack

# Coding Conventions

## Code-Implementierungsregeln
- **Schreibe Code immer direkt in die korrekten Dateien** - Niemals Dummy- oder Pseudo-Code ohne Erlaubnis erstellen
- **Schreibe produktionsreifen Code** - Aller Code sollte vollständig, funktional und einsatzbereit sein
- **Keine Platzhalter-Implementierungen** - Vermeide Stub-Funktionen, TODOs oder unvollständige Logik, außer explizit angefordert

## Dokumentationsrichtlinien
- **Niemals READMEs, Zusammenfassungen, Dokumentation oder andere zusätzliche Artefakte erstellen** (wie README.md-Dateien, Projektübersichten oder automatische Berichte) ohne explizite Anfrage
- **Immer auf direkte Anweisung warten** vom Benutzer, bevor solche Inhalte generiert oder hinzugefügt werden
- **Fokus auf Code, nicht Dokumentation** - Wenn nach Feature-Implementierung gefragt wird, nur den Code schreiben
- **Ausnahme**: Inline-Code-Kommentare sind akzeptabel und für Klarheit erwünscht

## Was das bedeutet
Bei Anfragen wie "Feature hinzufügen" oder "X implementieren":
- ✅ Schreibe die tatsächliche Implementierung
- ✅ Füge notwendige Imports und Dependencies hinzu
- ✅ Inkludiere Inline-Kommentare für komplexe Logik
- ❌ Erstelle keine README zur Erklärung der Arbeit
- ❌ Schreibe keine Zusammenfassungsdokumente
- ❌ Generiere keine Projektübersichten oder Berichte
- ❌ Erstelle keinen Dummy- oder Pseudo-Code
- ❌ Schreibe keine .md-Datei ohne Erlaubnis

Der Benutzer wird nach Dokumentation fragen, wenn sie benötigt wird.

## Codebase-Untersuchung
- Erkunde relevante Dateien und Verzeichnisse
- Suche nach Schlüsselfunktionen, Klassen oder Variablen zum Problem
- Lese und verstehe relevante Code-Snippets
- Identifiziere die Grundursache des Problems
- Validiere und aktualisiere kontinuierlich das Verständnis beim Sammeln von Kontext

## Code-Änderungen durchführen
- **Vor dem Bearbeiten immer die relevanten Dateiinhalte oder Abschnitte lesen** für vollständigen Kontext
- **Immer Dateien lesen, die in der zu bearbeitenden Datei importiert werden** für vollständigen Kontext
- Stelle sicher, den vollständigen Kontext der Codebase vor Änderungen zu verstehen
- Denke immer über die Auswirkungen der Änderungen nach auf:
  - Die gesamte Codebase
  - Dependencies und Interaktionen mit anderen Code-Teilen
  - Edge Cases und potenzielle Fallstricke
  - Performance, Sicherheit und Wartbarkeit
- Falls ein Patch nicht korrekt angewendet wird, versuche ihn erneut anzuwenden
- Mache kleine, testbare, inkrementelle Änderungen, die logisch aus Untersuchung und Plan folgen

## Umgebungsvariablen
- Wenn erkannt wird, dass ein Projekt eine Umgebungsvariable benötigt (wie API-Key oder Secret), prüfe immer, ob eine `.env`-Datei im Projekt-Root existiert
- Falls sie nicht existiert, erstelle automatisch eine `.env`-Datei mit Platzhalter für die benötigte(n) Variable(n) und informiere den Benutzer
- Tue dies proaktiv, ohne auf Benutzeranfrage zu warten

**ROS2 Humble + Colcon Build-System + micro-ROS Firmware + Docker Deployment**

- **Build**: `colcon build --symlink-install` (ROS2), `make build && make flash` (Firmware)
- **Test**: `colcon test`, `make test` (Firmware Unit-Tests)
- **Deploy**: `docker compose -f docker/compose.robot-pi.yaml up -d`
- **Debug**: `ros2 topic list`, `ros2 control list_controllers`, Foxglove Studio

## Build-System & Architektur

### ROS2 Workspace (Colcon)
- **Build-Tool**: `colcon build --symlink-install --merge-install`
- **ROS2 Distribution**: Humble (Ubuntu 22.04)
- **Dependency Management**: `vcs` für ROS2-Pakete, Git-Submodules für Libraries
- **Package Format**: ament_cmake (C++) und ament_python (Python)

### Firmware Build-System
- **Mikrocontroller**: Raspberry Pi Pico (RP2040)
- **Build-Tool**: CMake + Make
- **SDK**: Pico SDK + FreeRTOS + micro-ROS
- **Compiler**: gcc-arm-none-eabi

## Technologie-Stack

### High-Level (Raspberry Pi 4B)
- **OS**: Ubuntu Server 22.04 LTS
- **Middleware**: ROS2 Humble
- **Control Framework**: ros2_control + ros2_controllers
- **Navigation**: Nav2 + SLAM Toolbox
- **Computer Vision**: OpenCV + dlib
- **Communication**: micro-ROS Agent, Tailscale VPN

### Low-Level (Raspberry Pi Pico)
- **RTOS**: FreeRTOS
- **Communication**: micro-ROS Client
- **Libraries**: Eigen (Mathematik), Pico SDK (Hardware)
- **Protokoll**: USB Serial (CDC) zu ROS2 Host

### Deployment & Container
- **Container**: Docker + Docker Compose
- **Base Image**: `goldjunge491/robot:humble-arm64`
- **Orchestration**: Multi-Container (micro-ROS Agent + Robot Bringup)
- **VPN**: Tailscale für Remote-Zugriff

## Wichtige Frameworks & Libraries

### ROS2 Pakete
- `mecanum_drive_controller` - Omnidirektionale Bewegungssteuerung
- `joint_state_broadcaster` - Gelenkzustände publizieren
- `imu_sensor_broadcaster` - IMU-Daten publizieren
- `robot_state_publisher` - TF-Tree aus URDF
- `micro_ros_agent` - Brücke zu Mikrocontrollern
- `foxglove_bridge` - Web-Dashboard Konnektivität

### Hardware-Interfaces
- **ros2_control**: Hardware-Abstraktion für Motoren/Sensoren
- **SystemInterface**: Custom Hardware-Plugin für Pico-Kommunikation
- **CommandInterface**: Geschwindigkeitsbefehle an Motoren
- **StateInterface**: Encoder-Feedback und Sensordaten

### Firmware Libraries
- **micro-ROS**: ROS2-Client für Mikrocontroller
- **FreeRTOS**: Echtzeit-Task-Scheduling
- **Eigen**: Lineare Algebra für Odometrie-Berechnungen
- **Pico SDK**: Hardware-Abstraktion (GPIO, PWM, SPI, I2C)

## Häufige Build-Kommandos

### Workspace Build
```bash
# Vollständiger Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install

# Einzelnes Paket
colcon build --packages-select robot_description

# Mit Dependencies
colcon build --packages-up-to robot_bringup

# Tests ausführen
colcon test --packages-select robot_hardware_interfaces
colcon test-result --all --verbose
```

### Firmware Build
```bash
# Debug Build
cd src/robot_firmware
make build

# Release Build
make build_release

# Flash auf Pico
make flash

# Tests
make test
make test-verbose
```

### Docker Deployment
```bash
# Auf Raspberry Pi
docker compose -f docker/compose.robot-pi.yaml up -d

# Logs anzeigen
docker compose -f docker/compose.robot-pi.yaml logs -f

# Services stoppen
docker compose -f docker/compose.robot-pi.yaml down
```

### Development Workflow
```bash
# Setup Dependencies
./setup.sh

# Build Workspace
./build.sh

# Start Robot (Pi)
./scripts/start_robot.sh

# Start Simulation (Dev PC)
ros2 launch robot_gazebo simulation.launch.py

# Teleop Control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Konfigurationsdateien

### Controller-Konfiguration
- `src/robot_controller/config/robot_xl/mecanum_drive_controller.yaml`
- Update-Rate: 100 Hz, Geschwindigkeitslimits, Kinematik-Parameter

### Hardware-Interface
- `src/robot_hardware_interfaces/config/ros2_control_params.yaml`
- Serial-Port, Baudrate, Timeout-Werte

### Launch-Parameter
- `robot_model`: robot_xl (Standard)
- `mecanum`: True/False (Antriebstyp)
- `microros`: True/False (micro-ROS Agent starten)
- `camera`: True/False (USB-Kamera aktivieren)

## Testing & Debugging

### Unit Tests
```bash
# Firmware Tests
cd src/robot_firmware
make test

# ROS2 Package Tests
colcon test --packages-select <package_name>
```

### Hardware-Debugging
```bash
# micro-ROS Verbindung prüfen
ros2 topic list | grep rt/

# Controller Status
ros2 control list_controllers

# Hardware-Interface Status
ros2 service call /controller_manager/list_hardware_interfaces

# Firmware Monitor
cd src/robot_firmware
./monitor_firmware.sh
```

### Performance Monitoring
```bash
# Topic-Frequenzen
ros2 topic hz /joint_states
ros2 topic hz /odom

# CPU/Memory Usage
htop
ros2 node info /controller_manager
```

## Entwicklungsumgebung

### Erforderliche Tools
- **ROS2 Humble**: `sudo apt install ros-humble-desktop`
- **Colcon**: `pip3 install colcon-common-extensions`
- **VCS**: `pip3 install vcstool`
- **Pico SDK**: Für Firmware-Entwicklung
- **Docker**: Für Deployment-Tests

### IDE-Integration
- **VS Code**: ROS2-Extension, C++ IntelliSense
- **CLion**: CMake-Integration für Firmware
- **Foxglove Studio**: Robotik-Visualisierung und -Debugging
