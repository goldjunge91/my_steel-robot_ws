---
title: my_steel Robot Workspace
summary: Orientierung und Navigation innerhalb des Projekts
description: Einstiegspunkt für das ROS2-Workspace der my_steel Plattform mit Verweisen auf Hardware- und Software-Dokumentation
keywords: ros2, dokumentation, architektur, navigation
author: goldjunge91
order: 1
---
# Robot Projekt Übersicht

Der Workspace gliedert sich in zwei strikt getrennte Ebenen:

- **Software-Ebene** auf dem Raspberry Pi 4 mit ROS 2 Humble für Navigation, Teleoperation, Computer Vision und Betreiber-Dashboards.
- **Hardware-Ebene** auf den Mikrocontrollern (Raspberry Pi Pico, Arduino Pro Micro) für deterministische Motor- und Launcher-Steuerung.

## System-Layer auf einen Blick

```mermaid
graph LR
    subgraph "Software Layer (Pi 4)"
        ROS[ROS 2 Nodes]
        CTRL[Controller Manager]
        NAV[Navigation / SLAM]
        VISION[Computer Vision]
    end

    subgraph "Kommunikation"
        AGENT[micro-ROS Agent\nUSB Serial]
        BRIDGE[ROS Topics / Services]
    end

    subgraph "Hardware Layer"
        PICO[Pico Firmware\nMotor + Sensor]
        PROMICRO[Pro Micro Firmware\nNerf Launcher]
    end

    ROS -->|/cmd_vel, /tf, /odom| AGENT
    AGENT --> PICO
    PICO -->|Status Topics| AGENT
    ROS -->|Launcher Commands| BRIDGE --> PROMICRO
```

**Prinzip:** Software-Features werden ausschließlich über ROS 2 umgesetzt; alle hardwarenahen Zeitkritischen Abläufe laufen autark in der Firmware.

## Dokumentations-Navigator

| Fokus | Zweck | Nächster Schritt |
|-------|-------|------------------|
| **Systemüberblick** | Ziele, Architektur, Designentscheidungen | [PROJEKT.md](PROJEKT.md) |
| **Hardware bauen** | Verkabelung, Montage, Prüfungen | [hardware_setup.md](hardware_setup.md) |
| **Firmware verstehen** | Pico-Agenten, Datenflüsse, Echtzeit | [FIRMWARE_ARCHITECTURE.md](FIRMWARE_ARCHITECTURE.md) |
| **Pinbelegung** | Zuordnung der Controller-Pins | [PINMAP.md](PINMAP.md) |
| **Raspberry Pi betreiben** | Betriebssystem, Docker, Services | [raspberry_pi_setup_plan.md](raspberry_pi_setup_plan.md) |

## Dokumentation finden

```mermaid
flowchart LR
    README([README.md\nÜbersicht]) --> ARCHI[ARCHITECTURE_INDEX.md]
    README --> HW[hardware_setup.md]
    README --> FW[FIRMWARE_ARCHITECTURE.md]
    README --> PI[raspberry_pi_setup_plan.md]
    README --> PROJ[PROJEKT.md]
    ARCHI --> PINMAP[PINMAP.md]
    PINMAP --> PINR[PINMAP_ROBOT.md]
    PINMAP --> PINN[PINMAP_NERF.md]
```


## Architektur-Skizze

```mermaid
flowchart LR
    subgraph Host[ROS2 Host • Raspberry Pi 4B]
        Nav[Navigation & Autonomie]
        Vision[Computer Vision]
        Bringup[ros2_control Bringup]
    end

    subgraph Firmware[Raspberry Pi Pico • Echtzeit]
        Motors[Motorregelung]
        Sensors[Sensordaten]
        Bridge[micro-ROS Bridge]
    end

    subgraph Launcher[Arduino Pro Micro • Launcher]
        Esc[Brushless ESCs]
        Servos[Servos & Sicherheit]
    end

    Nav --> Bridge
    Vision --> Bridge
    Bringup --> Bridge

    Bridge --> Motors
    Sensors --> Bridge

    Bridge -. UART .-> Launcher
    Launcher --> Esc
    Launcher --> Servos
```

## Wesentliche Kennzahlen

| Kategorie | Hardware Layer | Software Layer |
| --- | --- | --- |
| Recheneinheit | Raspberry Pi 4B (4 GB), Raspberry Pi Pico | Arduino Pro Micro |
| Laufzeit | ROS 2 Humble (rclcpp, Nav2, SLAM Toolbox) | FreeRTOS + bare metal Treiber |
| Schnittstellen | USB CDC, UART, SPI, I²C, PWM | ROS 2 Topics, Services, Actions |

<!-- 
### Topic Architecture

```
┌─────────────────────┐
│ Navigation / Teleop │
└──────────┬──────────┘
           │ /cmd_vel (Twist)
           ▼
┌─────────────────────┐
│ Mecanum Controller  │
└──────────┬──────────┘
           │ velocity commands
           ▼
┌─────────────────────┐
│ Hardware Interface  │──────> /cmd_vel (Twist)
│  (ros2_control)     │<────── /joint_states
└─────────────────────┘        /imu/data_raw
           │
           ▼
┌─────────────────────┐
│  micro-ROS Agent    │
│  (topic remapping)  │
└──────────┬──────────┘
           │ USB Serial
           ▼
┌─────────────────────┐
│  Pico Firmware      │
│  (micro-ROS)        │
└─────────────────────┘
``` -->
