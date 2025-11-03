---
title: Architecture Index
summary: Navigation index for all architecture documentation
description: Quick navigation to find the right documentation for your needs
keywords: architecture, documentation, navigation, index
author: goldjunge91
alpha: true
order: 8
---

# Architecture Documentation Index

## What are you looking for?

### 🔧 **Hardware Setup & Connections**
- **[PINMAP.md](PINMAP.md)** - Pin assignments for Raspberry Pi Pico
- **[hardware_setup.md](hardware_setup.md)** - Complete hardware setup guide

### 💻 **Firmware & Low-Level**
- **[FIRMWARE_ARCHITECTURE.md](FIRMWARE_ARCHITECTURE.md)** - Pico firmware architecture with detailed diagrams

### 🚀 **Project Overview & Vision**
- **[PROJEKT.md](PROJEKT.md)** - Complete project story from idea to implementation (German)
- **[README.md](README.md)** - Quick start and workspace overview

### 🐳 **Deployment & Setup**
- **[raspberry_pi_setup_plan.md](raspberry_pi_setup_plan.md)** - Raspberry Pi deployment guide

## Quick Navigation by Task

| I want to... | Go to... |
|---------------|----------|
| Wire up the hardware | [PINMAP.md](PINMAP.md) |
| Set up the robot from scratch | [hardware_setup.md](hardware_setup.md) |
| Understand the firmware | [FIRMWARE_ARCHITECTURE.md](FIRMWARE_ARCHITECTURE.md) |
| Get the big picture | [PROJEKT.md](PROJEKT.md) |
| Deploy on Raspberry Pi | [raspberry_pi_setup_plan.md](raspberry_pi_setup_plan.md) |
| Quick start development | [README.md](README.md) |

## Architecture at a Glance

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Raspberry Pi 4B │    │ Raspberry Pi    │    │ Arduino Nano/   │
│                 │    │ Pico            │    │ Pro Micro       │
│ • ROS2 Humble   │◄──►│ • FreeRTOS      │    │ • Nerf Launcher │
│ • Navigation    │    │ • micro-ROS     │    │ • Separate      │
│ • Computer      │    │ • Motor Control │    │   Controller    │
│   Vision        │    │ • Sensors       │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Documentation Quality

| Document | Completeness | Technical Depth | Diagrams |
|----------|--------------|-----------------|----------|
| FIRMWARE_ARCHITECTURE.md | ████████████ | ████████████ | ████████████ |
| hardware_setup.md | ████████████ | ████████░░░░ | ████░░░░░░░░ |
| PROJEKT.md | ████████████ | ████████░░░░ | ████████░░░░ |
| PINMAP.md | ████████████ | ████░░░░░░░░ | ░░░░░░░░░░░░ |
| README.md | ████████████ | ████████░░░░ | ████░░░░░░░░ |