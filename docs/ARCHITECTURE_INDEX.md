# Architecture Documentation Index

This document provides an index to all architecture documentation and Mermaid diagrams in the project.

## Overview

The my_steel robot uses a two-tier architecture with a Raspberry Pi 4B running ROS2 Humble for high-level control and a Raspberry Pi Pico running FreeRTOS with micro-ROS for real-time motor control.

## Architecture Documents

### System-Level

- **[System Architecture](SYSTEM_ARCHITECTURE.md)** - Complete system overview
  - Complete system diagram
  - Layered architecture
  - Data flow diagrams
  - Topic flow
  - Hardware architecture
  - Network architecture
  - Performance metrics
  - Design decisions

### Core Packages

- **[robot_hardware_interfaces](../src/robot_hardware_interfaces/ARCHITECTURE.md)** - Hardware interface implementation
  - Component diagram
  - Class diagram
  - Sequence diagrams (activation, control loop)
  - Data flow
  - State machine
  - Design decisions (Twist vs Float32MultiArray, no mock mode)

- **[robot_bringup](../src/robot_bringup/ARCHITECTURE.md)** - Launch orchestration
  - Launch architecture
  - Launch file hierarchy
  - Sequence diagram (system startup)
  - Data flow
  - Component interaction
  - Launch parameters
  - State machine (launch process)
  - Topic remapping strategy

- **[robot_controller](../src/robot_controller/ARCHITECTURE.md)** - Controller configurations
  - Package structure
  - Controller architecture
  - Data flow
  - Configuration structure
  - Sequence diagram (controller lifecycle)
  - Mecanum drive kinematics
  - Topic remapping
  - State machine (controller states)

- **[robot_description](../src/robot_description/ARCHITECTURE.md)** - URDF/xacro models
  - Package structure
  - URDF structure
  - Link hierarchy
  - Joint types
  - ros2_control configuration
  - Coordinate frames (TF tree)
  - Gazebo integration
  - Component configuration flow
  - Physical properties
  - Xacro macros
  - URDF validation

### Firmware

- **[Pico Firmware](../firmware/ARCHITECTURE.md)** - Real-time firmware
  - System architecture
  - Agent architecture
  - Data flow
  - Sequence diagrams (initialization, control loop)
  - Task scheduling
  - Memory layout
  - Pin configuration
  - PID control flow
  - Odometry calculation
  - Communication protocol
  - Build system

## Quick Reference

### Key Diagrams by Topic

#### System Overview
- [Complete System Diagram](SYSTEM_ARCHITECTURE.md#complete-system-diagram)
- [Layered Architecture](SYSTEM_ARCHITECTURE.md#layered-architecture)
- [Hardware Architecture](SYSTEM_ARCHITECTURE.md#hardware-architecture)

#### Data Flow
- [Command to Motion](SYSTEM_ARCHITECTURE.md#data-flow-command-to-motion)
- [Sensor to State](SYSTEM_ARCHITECTURE.md#data-flow-sensor-to-state)
- [Topic Flow](SYSTEM_ARCHITECTURE.md#topic-flow-diagram)

#### Hardware Interface
- [Component Diagram](../src/robot_hardware_interfaces/ARCHITECTURE.md#component-diagram)
- [Control Loop Sequence](../src/robot_hardware_interfaces/ARCHITECTURE.md#sequence-diagram-control-loop)
- [State Machine](../src/robot_hardware_interfaces/ARCHITECTURE.md#state-machine-diagram)

#### Controllers
- [Controller Architecture](../src/robot_controller/ARCHITECTURE.md#controller-architecture)
- [Mecanum Kinematics](../src/robot_controller/ARCHITECTURE.md#mecanum-drive-kinematics)
- [Controller Lifecycle](../src/robot_controller/ARCHITECTURE.md#sequence-diagram-controller-lifecycle)

#### Firmware
- [Agent Architecture](../firmware/ARCHITECTURE.md#agent-architecture)
- [Control Loop](../firmware/ARCHITECTURE.md#sequence-diagram-control-loop)
- [Task Scheduling](../firmware/ARCHITECTURE.md#task-scheduling)
- [PID Control](../firmware/ARCHITECTURE.md#pid-control-flow)

#### Launch System
- [Launch Architecture](../src/robot_bringup/ARCHITECTURE.md#launch-architecture)
- [System Startup](../src/robot_bringup/ARCHITECTURE.md#sequence-diagram-system-startup)
- [Topic Remapping](../src/robot_bringup/ARCHITECTURE.md#topic-remapping-strategy)

#### Robot Model
- [URDF Structure](../src/robot_description/ARCHITECTURE.md#urdf-structure)
- [TF Tree](../src/robot_description/ARCHITECTURE.md#coordinate-frames-tf-tree)
- [ros2_control Config](../src/robot_description/ARCHITECTURE.md#ros2_control-configuration)

## Diagram Types

### Component Diagrams
Show the static structure of the system with components and their relationships.

**Found in:**
- System Architecture
- Hardware Interfaces
- Bringup
- Controllers

### Sequence Diagrams
Show the dynamic behavior of the system over time with message exchanges.

**Found in:**
- Hardware Interfaces (activation, control loop)
- Bringup (system startup)
- Controllers (lifecycle)
- Firmware (initialization, control loop)

### Data Flow Diagrams
Show how data moves through the system.

**Found in:**
- System Architecture
- Hardware Interfaces
- Controllers
- Firmware

### State Machine Diagrams
Show the states and transitions of system components.

**Found in:**
- Hardware Interfaces
- Bringup
- Controllers
- System Architecture

### Class Diagrams
Show the object-oriented structure of code.

**Found in:**
- Hardware Interfaces
- Firmware

### Flowcharts
Show algorithmic processes and decision logic.

**Found in:**
- Firmware (PID control, odometry)
- Robot Description (URDF validation)

## Viewing Mermaid Diagrams

### In GitHub
GitHub automatically renders Mermaid diagrams in Markdown files.

### In VS Code
Install the "Markdown Preview Mermaid Support" extension:
```bash
code --install-extension bierner.markdown-mermaid
```

### In Browser
Use the Mermaid Live Editor: https://mermaid.live/

### Generate Images
Use the Mermaid CLI:
```bash
npm install -g @mermaid-js/mermaid-cli
mmdc -i ARCHITECTURE.md -o architecture.png
```

## Contributing

When adding new features or modifying architecture:

1. Update the relevant architecture document
2. Add or modify Mermaid diagrams as needed
3. Update this index if adding new documents
4. Ensure diagrams are clear and follow existing style
5. Test that diagrams render correctly

### Diagram Style Guidelines

- Use consistent colors for similar components across diagrams
- Keep diagrams focused on one aspect (don't try to show everything)
- Add notes for important details
- Use subgraphs to group related components
- Label all connections clearly
- Include legends when using custom colors

### Color Conventions

- **Green (#4CAF50)**: Primary/active components
- **Blue (#2196F3)**: Data/communication components
- **Orange (#FF9800)**: Hardware/physical components
- **Purple (#9C27B0)**: Control/management components
- **Red (#f44336)**: Errors/warnings
- **Yellow (#FF9800)**: Warnings/cautions

## Related Documentation

- [README.md](../README.md) - Project overview and quick start
- [ROS2_TOPIC_MIGRATION_GUIDE.md](ROS2_TOPIC_MIGRATION_GUIDE.md) - Topic standardization migration
- [PINMAP.md](PINMAP.md) - Hardware pin assignments
- [hardware_setup.md](hardware_setup.md) - Hardware assembly guide
- [firmware.README.md](firmware.README.md) - Firmware documentation
- [TESTING_AND_MOCKING_STRATEGY.md](TESTING_AND_MOCKING_STRATEGY.md) - Testing approach

## Questions?

For questions about the architecture:
1. Check the relevant architecture document
2. Look at the code in the corresponding package
3. Review the design decisions section
4. Open an issue on GitHub

## Maintenance

This index should be updated whenever:
- New architecture documents are added
- Major architectural changes are made
- New diagram types are introduced
- Package structure changes significantly
