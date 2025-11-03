---
title: Robot Pin Assignments (Pico)
summary: MCU pinout mapping for Raspberry Pi Pico main controller
description: Single source of truth for main robot MCU pin mappings
keywords: pinmap, raspberry pi pico, pin assignments, hardware connections, robot
author: goldjunge91
order: 5
---



## Hardware Info

- **Board**: robot_digital_v1
- **MCU**: Raspberry Pi Pico (RP2040)
- **Firmware**: FreeRTOS + micro-ROS v0.1
- **Author**: @goldjunge91
- **Date**: 2025-11-03

## Pin Assignment Table

| Pin | Signal | Function | Notes |
|-----|--------|----------|-------|
| GP0 | UART0_TX | Debug UART | Reserved |
| GP1 | UART0_RX | Debug UART | Reserved |
| GP2 | VL6180X_SDA | ToF Sensor | ⚠️ **FIXED** |
| GP3 | VL6180X_SCL | ToF Sensor | ⚠️ **FIXED** |
| GP4 | MOTOR_FR_PWM_CW | Front Right Motor | PWM CW |
| GP5 | MOTOR_FR_PWM_CCW | Front Right Motor | PWM CCW |
| GP6 | MOTOR_FL_ENCODER_A | Front Left Encoder | Interrupt |
| GP7 | MOTOR_FL_ENCODER_B | Front Left Encoder | - |
| GP8 | MOTOR_FR_ENCODER_A | Front Right Encoder | - |
| GP9 | MOTOR_FR_ENCODER_B | Front Right Encoder | - |
| GP10 | MOTOR_RL_ENCODER_A | Rear Left Encoder | - |
| GP11 | MOTOR_RL_ENCODER_B | Rear Left Encoder | - |
| GP12 | MOTOR_RR_ENCODER_A | Rear Right Encoder | - |
| GP13 | MOTOR_RR_ENCODER_B | Rear Right Encoder | - |
| GP14 | MOTOR_RL_PWM_CW | Rear Left Motor | PWM CW |
| GP15 | MOTOR_RL_PWM_CCW | Rear Left Motor | PWM CCW |
| GP16 | IMU_MISO | IMU Sensor | ⚠️ **FIXED** |
| GP17 | IMU_CS | IMU Sensor | ⚠️ **FIXED** |
| GP18 | IMU_SCK | IMU Sensor | ⚠️ **FIXED** |
| GP19 | IMU_MOSI | IMU Sensor | ⚠️ **FIXED** |
| GP20 | MOTOR_FL_PWM_CW | Front Left Motor | PWM CW |
| GP21 | MOTOR_FL_PWM_CCW | Front Left Motor | PWM CCW |
| GP22 | MOTOR_RR_PWM_CW | Rear Right Motor | PWM CW |
| GP26 | LED_STATUS | Status LED | Blink |
| GP28 | MOTOR_RR_PWM_CCW | Rear Right Motor | PWM CCW |

## Fixed Assignments

!!! danger "DO NOT CHANGE"
    These pins are **hardware-constrained**:
    
    - **IMU (ICM20948)**: SPI0 → GP16/17/18/19
    - **VL6180X ToF**: I2C1 → GP2/3

## System Overview

### Hardware Components
- **4x Mecanum Motors**: PWM control + Hall encoders
- **IMU Sensor**: ICM20948 9-DOF via SPI0
- **ToF Sensor**: VL6180X distance + light via I2C1
- **Communication**: USB CDC to ROS2 host, UART debug



## Safety & Implementation

- **Motor Power**: Separate 12V rail, common GND
- **Flyback Diodes**: Required on all motor connections
- **Watchdog**: Firmware must implement safety shutdown
- **PWM Frequency**: 20 kHz for motors

## Firmware Sync

!!! warning "Update Process"
    1. Update this document first
    2. Update `firmware/src/board_config.h`
    3. Update `firmware/src/pinmap.h`
    4. Test and tag release

## Version History

- **v0.1** (2025-09-21): Initial pinmap
- **v0.2** (2025-11-03): Cleaned up, removed duplicates