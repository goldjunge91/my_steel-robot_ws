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
|  | TM1637 LED-Anzeige | Schnelle Anzeige von Statuscodes oder Werten |
