# Feature Specification: HAL with Robot Simulation (Gazebo)

**Feature Branch**: `001-feature-hal-mit`  
**Created**: 2025-09-21  
**Status**: Draft  
**Input**: User description: "Develop the HAL with robot simulation feature: connect our robot environment with the `mecabridge_hardware` layer without losing the ability to start Gazebo on the remote PC and drive the robot manually."  

Update: "Update `spec.md` and add the HAL folder `src/mecabridge_hardware` and the robot under `src/robot`. Clarify that this feature targets Gazebo and the end-to-end simulated robot with a full implementation of the ros2-control mecanum drive controller."

# Project Description: Omnidirectional ROS2 Robot

## 1. Vision & Objectives
The goal of this project is to create an affordable, modular, and capable mobile robot platform built on **Robot Operating System 2 (ROS2)**. The platform is positioned as an alternative to commercial systems such as TurtleBot and combines advanced navigation with an interactive computer-vision-driven feature: a Nerf dart launcher that can detect and aim at faces.

The project brings together advanced robotics concepts such as **autonomous navigation (SLAM)** and sensor fusion with engaging interactive elements.

## 2. Core Capabilities

* **Omnidirectional Mobility:** Thanks to the mecanum wheel drive, the robot can move freely in any direction (forward/backward, sideways, diagonal) and rotate on the spot.

* **Autonomous Navigation & Mapping:** With a LiDAR sensor and a 9-axis IMU, the robot can build maps of its environment (SLAM) and localise itself for autonomous motion.

* **Obstacle Detection:** A VL53L0X time-of-flight sensor provides close-range distance measurements to detect obstacles and support navigation.

* **Intelligent Nerf Launcher:** An integrated USB camera enables computer-vision algorithms. The primary application is **face detection**, allowing the Nerf launcher to automatically aim at detected people and fire on command.

* **Remote Control & Telemetry:** The robot can be driven manually with an Xbox controller. A web dashboard visualises critical telemetry such as battery state and sensor streams in real time.

## 3. System Architecture

The architecture is split into two control layers to distribute responsibilities efficiently:

1. **High-Level Control (Raspberry Pi 4B):**

   * **Robot brain:** Executes the ROS2 framework.

   * **Responsibilities:** Processes LiDAR, camera, and distance sensor data; runs SLAM algorithms; plans paths; hosts the web dashboard; performs face detection.

   * **Communication:** Sends high-level motion commands (for example, "drive left at 0.5 m/s") to the low-level controller.

2. **Low-Level Control (Raspberry Pi Pico):**

   * **Real-time controller:** Responsible for precise hardware actuation.

   * **Responsibilities:** Receives commands from the Pi 4B and converts them into PWM signals for the four DC motor drivers (TB6612FNG). Drives the servos and brushless motors for the Nerf launcher.

   * **Benefit:** Offloads real-time tasks from the Raspberry Pi 4B and guarantees reliable, low-jitter motor control.

## 4. Technical Components

| **Category**           | **Component**                                  | **Purpose**                                              |
|------------------------|-------------------------------------------------|----------------------------------------------------------|
| **Chassis & Drive**    | 4x DC gear motors (GM3865-520) with Hall encoders | Powerful drive with wheel rotation feedback              |
|                        | 4x 80 mm mecanum wheels                         | Enable omnidirectional movement                          |
|                        | 4x TB6612FNG motor drivers                      | Motor control                                            |
| **Control & Sensing**  | Raspberry Pi 4B (8GB)                           | High-level control, ROS2, computer vision                |
|                        | Raspberry Pi Pico                               | Low-level control (motors, servos)                       |
|                        | LiDAR LDS01RR                                   | 360° environment scans for SLAM                          |
|                        | ICM-20948 (9-DoF IMU)                           | Measures acceleration, rotation, and orientation         |
|                        | VL53L0X time-of-flight sensor                   | Precise distance measurement for obstacle detection      |
|                        | USB camera (1080p)                              | Video streaming and face detection input                 |
| **Nerf Launcher**      | 2x RS2205 brushless motors & 2x 40A ESCs        | Accelerate the Nerf darts                                |
|                        | 1x 22 kg digital servo & 1x 9 g servo           | Aim the launcher (pan/tilt)                              |
| **Power**              | 3S Li-ion battery pack (18650 cells)            | Mobile power supply                                      |
|                        | 3S battery management system                    | Protects against overcharge, deep discharge, shorting    |
|                        | INA3221 sensor                                  | Monitors voltage and current draw                        |
| **User Interaction**   | Xbox controller                                 | Manual remote control                                    |
|                        | 10" / 5" display                                | Local status or camera feed                              |
|                        | TM1637 LED display                              | Quick display of status codes or values                  |

---

## User Scenarios & Testing *(mandatory)*

### Primary User Story
As a developer, I want to connect the robot environment to the `mecabridge_hardware` layer while retaining the ability to launch Gazebo on the remote PC and drive the robot manually. The feature focuses on the Gazebo simulation and delivers a full implementation of the ros2-control mecanum drive controller for omnidirectional motion.

### Acceptance Scenarios
1. **Given** the robot runs in simulation mode, **when** I launch Gazebo on the remote PC, **then** I can manually drive the robot.
2. **Given** the robot is connected to the hardware, **when** I switch to hardware mode, **then** the system uses the `mecabridge_hardware` layer for control.
3. **Given** the HAL integration is active, **when** I switch between simulation and hardware, **then** the robot remains fully functional.

### Edge Cases
- What happens if the hardware is unavailable but hardware mode is selected?
- How does the system behave while switching modes if the robot is already moving?
- How does the system react to errors within the `mecabridge_hardware` layer?

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
- **FR-012**: System MUST provide comprehensive Gazebo simulation environment for testing and development of the omnidirectional robot.

### Key Entities *(include if feature involves data)*
- **Robot**: Represents the omnidirectional ROS2 robot with mecanum wheels, sensors, and Nerf launcher; includes attributes for position, velocity, sensor data, and control modes.
- **Hardware Layer**: Abstraction layer (`mecabridge_hardware`) interfacing with physical components such as motors, sensors, and controllers; located in `src/mecabridge_hardware`.
- **Robot Configuration**: Contains robot-specific configurations and models; located in `src/robot`.
- **Simulation Environment**: Gazebo-based simulation mirroring the physical robot for testing and development, supporting full ros2-control integration.

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
