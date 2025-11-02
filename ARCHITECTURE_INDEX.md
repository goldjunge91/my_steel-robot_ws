# Architecture Documentation Index

This document provides an index to all architecture documentation and Mermaid diagrams in the project.

## Overview

The my_steel robot uses a two-tier architecture with a Raspberry Pi 4B running ROS2 Humble for high-level control and a Raspberry Pi Pico running FreeRTOS with micro-ROS for real-time motor control.

## Architecture Documents

### Core Packages

- **robot_hardware** - Hardware interface implementation
  - Component diagram
  - Class diagram
  - Sequence diagrams (activation, control loop)
  - Data flow
  - State machine
  - Design decisions (Twist vs Float32MultiArray, no mock mode)

- **robot_controller** - Controller configurations
  - Package structure
  - Controller architecture
  - Data flow
  - Configuration structure
  - Sequence diagram (controller lifecycle)
  - Mecanum drive kinematics
  - Topic remapping
  - State machine (controller states)

- **robot_description** - URDF/xacro models
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

- **Pico Firmware** - Real-time firmware
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

#### Hardware Interface

- Component Diagram
- Control Loop Sequence
- State Machine

#### Controllers

- Controller Architecture
- Mecanum Kinematics
- Controller Lifecycle

#### Firmware

- Agent Architecture
- Control Loop
- Task Scheduling
- PID Control

#### Robot Model

- URDF Structure
- TF Tree
- ros2_control Config

## Diagram Types

### Component Diagrams

Show the static structure of the system with components and their relationships.

**Found in:**
- Hardware Interfaces
- Controllers

### Sequence Diagrams

Show the dynamic behavior of the system over time with message exchanges.

**Found in:**
- Hardware Interfaces (activation, control loop)
- Controllers (lifecycle)
- Firmware (initialization, control loop)

### Data Flow Diagrams

Show how data moves through the system.

**Found in:**
- Hardware Interfaces
- Controllers
- Firmware

### State Machine Diagrams

Show the states and transitions of system components.

**Found in:**
- Hardware Interfaces
- Controllers

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

Use the Mermaid Live Editor: <https://mermaid.live/>

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

- [PINMAP.md](PINMAP.md) - Hardware pin assignments
- [hardware_setup.md](hardware_setup.md) - Hardware assembly guide

## Maintenance

This index should be updated whenever:
- New architecture documents are added
- Major architectural changes are made
- New diagram types are introduced
- Package structure changes significantly