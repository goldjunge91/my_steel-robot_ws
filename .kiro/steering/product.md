# Product Overview

> **TODO: This steering document needs approval before being considered final**

## my_steel Robot - Omnidirectional ROS2 Robot Platform

An autonomous mobile robot platform built on ROS2 with omnidirectional movement capabilities and interactive features. The robot combines advanced navigation (SLAM) with computer vision for an entertaining Nerf dart launcher that can detect and target faces.

### Core Features
- **Omnidirectional mobility** via Mecanum wheel drive system
- **Autonomous navigation** using LiDAR SLAM and sensor fusion
- **Interactive Nerf launcher** with computer vision-based face detection
- **Dual-controller architecture** (Raspberry Pi 4B + Pico) for optimal performance
- **Manual control** via Xbox 360 Wireless Controller from Remote-PC or web dashboard
- **Web dashboard** for telemetry, remote monitoring, and manual Nerf launcher control
- **Real-time computer vision** with OpenCV and YOLOv5 for face detection and targeting
- **Real-time SLAM visualization** with RViz2 for navigation and mapping
- **Precise motor control** with Raspberry Pi Pico for real-time movement execution
- **Multi-modal operation** supporting autonomous, manual, and mixed control modes

### Target Use Cases
- Student Project for educational robotics
- Interactive entertainment robot
- Research platform for navigation and computer vision

### Hardware Architecture
- **Remote control station**: Development PC for teleoperation and monitoring
- **High-level control**: Raspberry Pi 4B (8GB) running ROS2 Humble on Ubuntu 22.04
- **Low-level control**: Raspberry Pi Pico for real-time motor control, IMU, and ToF sensor
- **Sensors**: LiDAR (LDS01RR), 9-DOF IMU (ICM-20948), ToF distance sensor (VL53L0X), USB camera (1080p)
- **Mobility**: 4x Mecanum wheels (80mm) with DC gearmotors (GM3865-520) and Hall encoders
- **Nerf launcher**: 2x brushless motors (RS2205) with ESCs, pan/tilt servos for targeting
- **Power**: 3S Li-Ion battery pack (18650 cells) with BMS protection and INA3221 monitoring

The project emphasizes modularity, cost-effectiveness, and educational value while maintaining professional-grade capabilities.