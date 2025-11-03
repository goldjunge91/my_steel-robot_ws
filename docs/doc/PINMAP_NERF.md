---
title: Nerf Launcher Pin Assignments (Pro Micro)
summary: MCU pinout mapping for Arduino Pro Micro nerf launcher controller
description: Single source of truth for nerf launcher MCU pin mappings
keywords: pinmap, arduino pro micro, nerf launcher, pin assignments, brushless motors, servos
author: goldjunge91
order: 6
---

## Hardware Info

- **Board**: nerf_launcher_v1
- **MCU**: Arduino Pro Micro (ATmega32U4)
- **Firmware**: Arduino + micro-ROS v0.1
- **Author**: @goldjunge91
- **Date**: 2025-11-03

## Pin Assignment Table

| Pin | Signal | Function | Notes |
|-----|--------|----------|-------|
| D2 | ESC_MOTOR_1 | Brushless Motor 1 | PWM 1000-2000μs |
| D3 | ESC_MOTOR_2 | Brushless Motor 2 | PWM 1000-2000μs |
| D4 | TRIGGER_SERVO | Trigger Mechanism | 22kg digital servo |
| D5 | SERVO_TILT | Tilt Servo | 9g servo (up/down) |
| D6 | UNUSED | - | Available for future use |
| D7 | SAFETY_SWITCH | Hardware Interlock | ⚠️ **MANDATORY** |
| D8 | FIRE_BUTTON | Manual Override | Digital input |
| D9 | LED_STATUS | Status LED | System status |
| D10 | COMM_TX | Serial TX | To main robot |
| D16 | COMM_RX | Serial RX | From main robot |
| A0 | BATTERY_VOLTAGE | Battery Monitor | Analog input |
| A1 | MOTOR_CURRENT | Current Sensor | Analog input |

## Fixed Assignments

!!! danger "DO NOT CHANGE"
    These pins are **safety-critical**:
    
    - **Safety Switch (D7)**: Hardware interlock required
    - **Emergency Stop**: Immediate motor shutdown capability

## System Overview

### Hardware Components
- **2x Brushless Motors**: RS2205 with 40A ESCs (flywheel system)
- **1x Tilt Servo**: 9g servo for vertical aiming (up/down)
- **1x Trigger Servo**: 22kg digital servo for firing mechanism
- **Safety Systems**: Hardware interlock, emergency stop
- **Communication**: Serial UART to main robot, monitoring



## Safety & Implementation

- **Logic Power**: 5V from USB or main robot
- **Motor Power**: 11.1V (3S LiPo from main robot)
- **Safety Timeout**: Auto-shutdown after 30s inactivity
- **Range Limits**: Servo angles and speed restricted

## Firmware Sync

!!! warning "Update Process"
    1. Update this document first
    2. Update `firmware/src/config.h`
    3. Update `firmware/src/pins.h`
    4. Test and tag release

## Version History

- **v0.1** (2025-11-03): Initial nerf launcher pinmap
- **v0.2** (2025-11-03): Cleaned up, removed duplicates