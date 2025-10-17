# my_steel Robot - Projektstruktur

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

**ROS2 Workspace mit modularer Paket-Architektur: Core (description, bringup, controller) + Funktional (vision, launcher, utils)**

- **Pakete**: `robot_` Präfix, `_tool/_interfaces/_launcher` Suffix
- **Topics**: REP-105 konform (`/cmd_vel`, `/odom`, `/imu/data_raw`, `/sensors/*`)
- **Launch**: `robot_model:=robot_xl mecanum:=True microros:=False`
- **Hardware**: Pico (FreeRTOS + micro-ROS) ↔ Pi 4B (ROS2 + Navigation)

## Workspace-Organisation

### Root-Level Struktur
```
my_steel-robot_ws/
├── src/                    # ROS2 Pakete (VCS-verwaltet)
├── lib/                    # Externe Libraries (Git Submodules)
├── docker/                 # Container-Deployment
├── scripts/                # Build- und Setup-Skripte
├── docs/                   # Projektdokumentation
├── .kiro/                  # Kiro-Konfiguration
├── build.sh               # Workspace Build-Skript
├── setup.sh               # Dependency Setup
└── src/ros2.repos         # VCS Repository-Konfiguration
```

## ROS2 Paket-Architektur

### Core Robot Pakete
- **`robot_description/`** - URDF/XACRO Robotermodelle, Meshes, Konfigurationen
- **`robot_bringup/`** - Launch-Orchestrierung, System-Startup
- **`robot_controller/`** - ros2_control Konfigurationen (Mecanum/Diff Drive)
- **`robot_hardware_interfaces/`** - Hardware-Abstraktionsschicht für Pico

### Funktionale Pakete
- **`robot_gazebo/`** - Gazebo-Simulation, Welten, Spawn-Logik
- **`robot_localization_tool/`** - EKF Sensorfusion, Odometrie-Verbesserung
- **`robot_vision/`** - Computer Vision, Gesichtserkennung
- **`robot_nerf_launcher/`** - Nerf-Dart-System Steuerung
- **`robot_utils/`** - Hilfsskripte, Tools, Laser-Filter

### Externe Abhängigkeiten
- **`micro-ROS-Agent/`** - Brücke zwischen ROS2 und Mikrocontrollern
- **`open_manipulator_x/`** - Optionaler Roboterarm (MoveIt Integration)
- **`robot_gz_worlds/`** - Gazebo-Welten und -Modelle

### Firmware (Separates Repository)
- **`robot_firmware/`** - Raspberry Pi Pico Firmware (FreeRTOS + micro-ROS)

## Konfigurationskonventionen

### Launch-Dateien Hierarchie
```
robot_bringup/launch/bringup.launch.py          # Haupt-Orchestrierung
├── robot_controller/launch/controller.launch.py # Controller-Management
├── robot_bringup/launch/microros_agent.launch.py # micro-ROS Bridge
└── robot_localization_tool/launch/ekf.launch.py  # Sensorfusion
```

### Controller-Konfigurationen
```
robot_controller/config/robot_xl/
├── mecanum_drive_controller.yaml      # Omnidirektionale Steuerung
├── diff_drive_controller.yaml         # Differentialantrieb (Legacy)
├── mecanum_drive_manipulator_controller.yaml  # Mit Roboterarm
└── diff_drive_manipulator_controller.yaml     # Diff + Arm
```

### Hardware-Konfigurationen
```
robot_description/config/robot_xl/
├── basic.yaml              # Grundkonfiguration
├── autonomy.yaml           # Navigation + SLAM
├── manipulation.yaml       # Mit Roboterarm
├── telepresence.yaml       # Remote-Steuerung
└── xbox_teleop.yaml        # Controller-Mapping
```

## Naming Conventions

### ROS2 Topics (REP-105 konform)
- **Sensoren**: `/imu/data_raw`, `/sensors/range_tof`, `/sensors/illuminance`
- **Bewegung**: `/cmd_vel` (Input), `/odom` (Output), `/joint_states`
- **Hardware**: `/rt/*` (Firmware-Topics mit micro-ROS Prefix)

### Package Naming
- **Präfix**: `robot_` für alle projektspezifischen Pakete
- **Suffix**: `_tool`, `_interfaces`, `_launcher` für funktionale Unterscheidung
- **Modell**: `robot_xl` als Standard-Robotermodell

### Launch-Parameter
- **`robot_model`**: `robot_xl` (einziges unterstütztes Modell)
- **`mecanum`**: `True`/`False` (Antriebstyp)
- **`use_sim`**: `True`/`False` (Simulation vs. Hardware)
- **`microros`**: `True`/`False` (micro-ROS Agent starten)
- **`camera`**: `True`/`False` (USB-Kamera aktivieren)

## Build-System Organisation

### Dependency Management
- **VCS (src/)**: ROS2-Pakete mit `src/ros2.repos`
- **Git Submodules (lib/)**: Externe Libraries (FreeRTOS, Eigen, Pico SDK)
- **rosdep**: System-Abhängigkeiten automatisch installieren

### Build-Targets
```bash
# Vollständiger Workspace
colcon build --symlink-install --merge-install

# Hardware-spezifisch
colcon build --packages-up-to robot_hardware_interfaces

# Simulation-spezifisch  
colcon build --packages-up-to robot_gazebo

# Firmware (separates Build-System)
cd src/robot_firmware && make build
```

## Deployment-Struktur

### Docker-Container Organisation
```
docker/
├── compose.robot-pi.yaml      # Raspberry Pi Deployment
├── compose.simulation.yaml    # Entwicklungs-Simulation
├── compose.robot_xl.yaml      # Vollständiges System
└── Dockerfile.*               # Multi-Stage Builds
```

### Service-Architektur
- **microros-agent**: USB-Serial Brücke zu Pico
- **robot-bringup**: Haupt-ROS2-Stack
- **Optional**: Tailscale VPN für Remote-Zugriff

## Hardware-Mapping (PINMAP.md)

### Raspberry Pi Pico Pin-Zuordnung
- **Motoren**: 4x PWM-Paare (CW/CCW) + Encoder (A/B)
- **Sensoren**: SPI (IMU), I2C (ToF), GPIO (Status-LED)
- **Kommunikation**: USB-CDC für micro-ROS

### ROS2 Hardware-Interface
- **CommandInterface**: Geschwindigkeitsbefehle an Motoren
- **StateInterface**: Encoder-Feedback, Sensordaten
- **Update-Rate**: 100 Hz für Motoren, 50 Hz für Sensoren

## Entwicklungsworkflow

### Lokale Entwicklung
1. **Setup**: `./setup.sh` (Dependencies installieren)
2. **Build**: `./build.sh` (Workspace kompilieren)
3. **Test**: `colcon test` (Unit-Tests ausführen)
4. **Simulation**: `ros2 launch robot_gazebo simulation.launch.py`

### Hardware-Deployment
1. **Firmware**: `cd src/robot_firmware && make flash`
2. **Container**: `docker compose -f docker/compose.robot-pi.yaml up -d`
3. **Monitoring**: Foxglove Studio, RViz, Web-Dashboard

### Code-Organisation
- **C++ Pakete**: `ament_cmake` mit Standard-CMakeLists.txt
- **Python Pakete**: `ament_python` mit setup.py
- **Launch-Dateien**: Python-basiert für Flexibilität
- **Konfiguration**: YAML-Dateien für Parameter

## Dokumentationsstruktur

### Technische Dokumentation
- **`docs/ARCHITECTURE.md`** - System-Architektur und Datenfluss
- **`docs/PINMAP.md`** - Hardware-Pin-Zuordnung (Single Source of Truth)
- **`docs/architecture_and_packages.md`** - Paket-Verantwortlichkeiten

### Benutzer-Dokumentation
- **`README.md`** - Schnellstart und Übersicht
- **`Projekt.md`** - Detaillierte Projektbeschreibung
- **Package-spezifische READMEs** - In jedem src/ Unterordner