```markdown
# PINMAP (Template) — my_steel Robot (Digital Board: Raspberry Pi Pico / MCU)
This document is the single source of truth for physical pinout mappings between MCU pins, connectors and functional signals.
Fill all fields for your exact board revision. Keep this file in firmware/ and docs/ and reference it from README.

## Versioning
- Board name: my_steel_digital_v1
- Firmware tag: v0.1
- Author: @goldjunge91
- Date: 2025-09-21

## Legend
- MCU_PIN: MCU pin name or physical pin (e.g., GP0, PA0)
- SIGNAL: function (Encoder A/B, PWM, DIR, SERVO_PWM, I2C_SDA)
- CONNECTOR: connector name on the board (J1, JST1, PWR)
- NOTES: any electrical notes, pull-ups, levels (3.3V), timers used (PWM Timer X), interrupts

| MCU_PIN | SIGNAL                | CONNECTOR | NOTES (timer/interrupt/alt) |
|---------|-----------------------|-----------|-----------------------------|
| GP0 / PA0 | MOTOR_FL_ENCODER_A  | J1-1      | encoder A, ext interrupt   |
| GP1 / PA1 | MOTOR_FL_ENCODER_B  | J1-2      | encoder B                 |
| GP2 / PA2 | MOTOR_FL_PWM        | J2-1      | PWM (Timer 1 Channel 1)   |
| GP3 / PA3 | MOTOR_FL_DIR        | J2-2      | Digital output            |
| GP4       | MOTOR_FR_PWM        | J3-1      | PWM Timer 2 Ch1           |
| GP5       | MOTOR_FR_DIR        | J3-2      |                           |
| GP6       | MOTOR_RL_PWM        | J4-1      |                           |
| GP7       | MOTOR_RL_DIR        | J4-2      |                           |
| GP8       | MOTOR_RR_PWM        | J5-1      |                           |
| GP9       | MOTOR_RR_DIR        | J5-2      |                           |
| I2C_SDA   | ICM20948 SDA        | I2C1      | ALT pins, pull-ups 4.7k   |
| I2C_SCL   | ICM20948 SCL        | I2C1      |                           |
| UART_TX   | micro-ROS / Serial TX | CONSOLE  | Serial to SBC: Baud default 576000 |
| UART_RX   | micro-ROS / Serial RX | CONSOLE  |                           |
| ADC0      | BATTERY_VOLTAGE     | BATT_MON  | via voltage divider, calibration coef |
| PWM_SERVO | SERVO_PAN           | SERVO_J1  | 50Hz typical, 3.3V tolerant |
| PWM_SERVO2| SERVO_TILT          | SERVO_J2  |                           |
| GPIO_X    | VL53L0X_XSHUT       | TOF1      | optional reset pin         |

## Notes on wiring & safety
- Motor power must be isolated from MCU power (use common GND).
- TTL levels: MCU 3.3V — ensure any 5V peripherals are level shifted.
- Use proper decoupling and flyback diodes where needed.
- Document connector pin numbering physically on board silkscreen.

## How to update
- Update the firmware pins in `firmware/` source (board_config.h / pinmap.h).
- Bump `PINMAP` version in this file and tag firmware release.
- Keep a CSV export for quick reference: docs/pinmap.csv