# Product Overview
## Code Implementation Rules

- **Always write code directly to the correct files** - Never create dummy or pseudo code without permission
- **Write production-ready code** - All code should be complete, functional, and ready to use
- **No placeholder implementations** - Avoid stub functions, TODOs, or incomplete logic unless explicitly requested
- **Never create READMEs, summaries, documentation, or other additional artifacts** (such as README.md files, project overviews, or automatic reports) without explicit request
- **Always wait for direct instruction** from the user before generating or adding such content

> **Note**: See `coding-conventions.md` for important rules about code implementation and documentation.

## my_steel Robot

An omnidirectional mobile robot platform built on ROS2 Humble, featuring mecanum wheel drive and an interactive Nerf dart launcher with computer vision capabilities.

## Core Capabilities

- **Omnidirectional mobility** via 4-wheel mecanum drive system with custom controller
- **Autonomous navigation** using RPLiDAR and sensor fusion (ICM20948 9-DOF IMU)
- **Computer vision** for face detection and tracking
- **Interactive Nerf launcher** with pan/tilt servos and brushless dart acceleration (separate controller)
- **Real-time telemetry** via Foxglove and RViz
- **Optional manipulator arm** support (Open Manipulator X with Dynamixel servos)

## Hardware Architecture

Two-tier control system:
- **High-level (Raspberry Pi 4B)**: ROS2 Humble framework, navigation, SLAM, computer vision, web dashboard
- **Low-level (Raspberry Pi Pico)**: Real-time motor control (4x DC motors with encoders), IMU (ICM20948), ToF sensor (VL6180X), micro-ROS over USB serial

## Current Status

- **Supported robot model**: robot_xl only
- **Supported drive type**: mecanum (primary), differential (legacy support)
- **Firmware**: FreeRTOS-based with micro-ROS client on Raspberry Pi Pico
- **Simulation**: Gazebo Classic with ros2_control integration
- **Development**: Active development with CI/CD via GitHub Actions

## Target Use Cases

- Educational robotics platform
- Alternative to commercial systems like TurtleBot and Husarion ROSbot
- Interactive demonstration of autonomous navigation + computer vision
- Modular base for mecanum or differential drive configurations
- Research platform for mobile manipulation
