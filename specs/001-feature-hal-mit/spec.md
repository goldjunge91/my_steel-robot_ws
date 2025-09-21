# Feature Specification: HAL mit Robot Simulation (Gazebo)

**Feature Branch**: `001-feature-hal-mit`  
**Created**: 2025-09-21  
**Status**: Draft  
**Input**: User description: "Feature HAL mit Robot simulaton entwickeln wir werden unseren robot umgebung, mit dem hardware layer "mecabridge_hardware" verbinden dabei aber nicht die möglichkeit verlieren auf dem remote pc gazebo zu starten und manuell so zu steuern. 

Update: "update the spec.md and add the HAL folder "src/mecabridge_hardware" and the robot that in "src/robot", specify this feature is for gazebo and all around running simulated robot with the full implementation of ros2-control mecanum driver control" 

# Projektbeschreibung: Omnidirektionaler ROS2-Roboter

## 1. Vision & Zielsetzung
Ziel dieses Projekts ist die Entwicklung einer kostengünstigen, modularen und leistungsfähigen mobilen Roboterplattform, die auf dem **Robot Operating System 2 (ROS2)** basiert. Die Plattform dient als Alternative zu kommerziellen Systemen wie dem TurtleBot und kombiniert fortschrittliche Navigationsfähigkeiten mit einer interaktiven, durch Computer Vision gesteuerten Funktion: einem Nerf-Dart-Launcher, der Gesichter erkennen und anvisieren kann.

Das Projekt verbindet anspruchsvolle Robotik-Konzepte wie **autonome Navigation (SLAM)** und Sensorfusion mit unterhaltsamen, interaktiven Elementen.

## 2. Kernfunktionen

* **Omnidirektionale Mobilität:** Dank des Mecanum-Radantriebs kann sich der Roboter frei in alle Richtungen bewegen (vorwärts/rückwärts, seitwärts, diagonal) und auf der Stelle drehen.

* **Autonome Navigation & Kartierung:** Mithilfe eines LiDAR-Sensors und eines 9-Achsen-IMU (Inertial Measurement Unit) ist der Roboter in der Lage, seine Umgebung zu kartieren (SLAM) und sich darin autonom zu lokalisieren und zu bewegen.

* **Hinderniserkennung:** Ein VL53L0X Time-of-Flight-Sensor liefert zusätzliche Distanzdaten zur Erkennung von nahen Objekten und zur Unterstützung der Navigation.

* **Intelligenter Nerf-Launcher:** Eine integrierte USB-Kamera ermöglicht die Implementierung von Computer-Vision-Algorithmen. Das primäre Anwendungsziel ist die **Gesichtserkennung**, um den Nerf-Launcher automatisch auf erkannte Personen auszurichten und auf Befehl abzufeuern.

* **Fernsteuerung & Telemetrie:** Der Roboter kann manuell über einen Xbox-Controller gesteuert werden. Ein Web-Dashboard visualisiert wichtige Telemetriedaten wie Batteriestatus und Sensordaten in Echtzeit.

## 3. Systemarchitektur

Die Architektur ist in zwei Steuerungsebenen unterteilt, um eine effiziente Aufgabenverteilung zu gewährleisten:

1. **High-Level-Steuerung (Raspberry Pi 4B):**

   * **Gehirn des Roboters:** Führt das ROS2-Framework aus.

   * **Aufgaben:** Verarbeitet Daten von LiDAR, Kamera und Abstandssensoren, führt SLAM-Algorithmen aus, plant Pfade, hostet das Web-Dashboard und führt die Gesichtserkennungs-Software aus.

   * **Kommunikation:** Sendet hochrangige Bewegungsbefehle (z.B. "fahre nach links mit 0,5 m/s") an die Low-Level-Steuerung.

2. **Low-Level-Steuerung (Raspberry Pi Pico):**

   * **Echtzeit-Controller:** Verantwortlich für die direkte, präzise Ansteuerung der Hardware.

   * **Aufgaben:** Empfängt Befehle vom Pi 4B und übersetzt sie in exakte PWM-Signale für die vier DC-Motortreiber (TB6612FNG). Steuert die Servos und Brushless-Motoren des Nerf-Launchers.

   * **Vorteil:** Entlastet den Raspberry Pi 4B von Echtzeitaufgaben und sorgt für eine zuverlässige und jitterfreie Motorsteuerung.

## 4. Technische Komponenten

| **Kategorie** | **Komponente** | **Zweck** |
| **Chassis & Antrieb** | 4x DC-Getriebemotoren (GM3865-520) mit Hall-Encodern | Kraftvoller Antrieb und Feedback zur Raddrehung |
|  | 4x 80mm Mecanum-Räder | Ermöglichen die omnidirektionale Bewegung |
|  | 4x TB6612FNG Motortreiber | Ansteuerung der DC-Motoren |
| **Steuerung & Sensorik** | Raspberry Pi 4B (8GB) | High-Level-Steuerung, ROS2, Computer Vision |
|  | Raspberry Pi Pico | Low-Level-Steuerung (Motoren, Servos) |
|  | Lidar LDS01RR | 360°-Umgebungsscans für SLAM |
|  | ICM-20948 (9-DoF IMU) | Erfassung von Beschleunigung, Rotation und Ausrichtung (Sensorfusion) |
|  | VL53L0X Time-of-Flight-Sensor | Präzise Abstandsmessung für Hinderniserkennung |
|  | USB-Kamera (1080p) | Video-Streaming und Input für die Gesichtserkennung |
| **Nerf-Launcher** | 2x RS2205 Brushless-Motoren & 2x 40A ESCs | Beschleunigung der Nerf-Darts |
|  | 1x Digital-Servo (22kg) & 1x 9g Servo | Zielen des Launchers (Pan/Tilt) |
| **Energieversorgung** | 3S Li-Ion Akku-Pack (18650 Zellen) | Mobile Stromversorgung |
|  | 3S Batterieschutzplatine (BMS) | Schutz vor Überladung, Tiefentladung und Kurzschluss |
|  | INA3221 Sensor | Überwachung von Spannung und Stromverbrauch |
| **Bedienung & Interface** | Xbox Controller | Manuelle Fernsteuerung |
|  | 10" / 5" Display | Lokale Anzeige von Statusinformationen oder Kamerabild |
|  | TM1637 LED-Anzeige | Schnelle Anzeige von Statuscodes oder Werten |"

---

## User Scenarios & Testing *(mandatory)*

### Primary User Story
Als Entwickler möchte ich die Roboterumgebung mit dem Hardware-Layer "mecabridge_hardware" verbinden, dabei aber nicht die Möglichkeit verlieren, Gazebo auf dem Remote-PC zu starten und den Roboter manuell zu steuern. Das Feature soll sich auf die Gazebo-Simulation konzentrieren und eine vollständige Implementierung des ros2-control Mecanum-Treibers für die omnidirektionale Steuerung bieten.

### Acceptance Scenarios
1. **Given** der Roboter ist im Simulationsmodus, **When** ich Gazebo auf dem Remote-PC starte, **Then** kann ich den Roboter manuell steuern.
2. **Given** der Roboter ist mit der Hardware verbunden, **When** ich in den Hardware-Modus wechsle, **Then** verwendet das System den mecabridge_hardware Layer für die Steuerung.
3. **Given** die HAL-Integration ist aktiv, **When** ich zwischen Simulation und Hardware wechsle, **Then** bleibt die Funktionalität des Roboters erhalten.

### Edge Cases
- Was passiert, wenn die Hardware nicht verfügbar ist, aber der Hardware-Modus aktiviert wird?
- Wie verhält sich das System beim Wechsel zwischen Modi während der Roboter in Bewegung ist?
- Wie wird mit Fehlern im mecabridge_hardware Layer umgegangen?

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: System MUST allow switching between simulation and hardware modes without restarting the system.
- **FR-002**: System MUST integrate the mecabridge_hardware layer for hardware control of the omnidirectional robot.
- **FR-003**: System MUST maintain the ability to start Gazebo simulation on the remote PC and control the robot manually.
- **FR-004**: System MUST support omnidirectional mobility using Mecanum wheels in both simulation and hardware modes.
- **FR-005**: System MUST provide autonomous navigation and mapping capabilities using LiDAR and IMU sensors.
- **FR-006**: System MUST implement obstacle detection using VL53L0X Time-of-Flight sensor.
- **FR-007**: System MUST enable intelligent Nerf-Launcher functionality with face recognition using USB camera.
- **FR-008**: System MUST allow manual control via Xbox controller.
- **FR-009**: System MUST provide real-time telemetry visualization through a web dashboard.
- **FR-010**: System MUST ensure safe operation with fail-safe mechanisms for actuator control.
- **FR-011**: System MUST implement full ros2-control mecanum driver control for omnidirectional mobility in both simulation and hardware modes.
- **FR-012**: System MUST provide comprehensive gazebo simulation environment for testing and development of the omnidirectional robot.

### Key Entities *(include if feature involves data)*
- **Robot**: Represents the omnidirectional ROS2 robot with Mecanum wheels, sensors, and Nerf-Launcher; has attributes for position, velocity, sensor data, and control modes.
- **Hardware Layer**: Abstraction layer (mecabridge_hardware) that interfaces with physical hardware components like motors, sensors, and controllers; located in src/mecabridge_hardware.
- **Robot Configuration**: Contains robot-specific configurations and models; located in src/robot.
- **Simulation Environment**: Gazebo-based simulation that mirrors the physical robot for testing and development, supporting full ros2-control integration.

---

## Review & Acceptance Checklist
*GATE: Automated checks run during main() execution*

### Content Quality
- [ ] No implementation details (languages, frameworks, APIs)
- [ ] Focused on user value and business needs
- [ ] Written for non-technical stakeholders
- [ ] All mandatory sections completed

### Requirement Completeness
- [ ] No [NEEDS CLARIFICATION] markers remain
- [ ] Requirements are testable and unambiguous  
- [ ] Success criteria are measurable
- [ ] Scope is clearly bounded
