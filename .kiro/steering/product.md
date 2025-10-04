# my_steel Robot Project

## Overview
The my_steel robot is a ROS2-based omnidirectional robot platform featuring four mecanum wheels and a nerf launcher. It's designed as a modular system with holonomic movement capabilities.

## Key Features
- **Omnidirectional Movement**: Mecanum wheel configuration for holonomic motion
- **Nerf Launcher**: Integrated projectile launching system with servo control
- **Mecanum Drive System**: Four-wheel mecanum configuration for omnidirectional movement
- **Sensor Integration**: IMU, distance sensors, encoders, and optional camera/LiDAR
- **Dual Deployment**: Runs on Raspberry Pi SBC with Raspberry Pi Pico firmware

## Target Platforms
- **Robot (SBC)**: Raspberry Pi 4B/5 for autonomous operation
- **Remote PC**: Development workstation for simulation and remote control
- **Firmware**: Raspberry Pi Pico for real-time motor control and sensor interfacing

## Primary Use Cases
- Autonomous navigation and mapping
- Remote teleoperation
- Target tracking and engagement (nerf launcher)
- Educational robotics platform
- ROS2 development and testing