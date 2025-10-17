# PINMAP (Template) — my_steel Robot (Digital Board: Raspberry Pi Pico)

Dieses Dokument ist die single source of truth für MCU‑Pinout mappings zwischen MCU‑Pins, Connectors und funktionalen Signalen.
Pflege diese Datei im Firmware‑Repo (robot_firmware/docs/PINMAP.md) und halte sie synchron mit Firmware‑Konstanten (board_config.h / pinmap.h).

## Version / Metadaten
- Board name: robot_digital_v1
- Firmware tag: v0.1
- Author: @goldjunge91
- Date: 2025-09-21

## Legende
- MCU_PIN: MCU pin name (z. B. GP0)
- SIGNAL: Funktion (Encoder A/B, PWM, DIR, SERVO_PWM, I2C_SDA)
- CONNECTOR: Connector oder Header (J1, JST1, PWR)
- NOTES: Timer/Interrupt/Level/Anmerkungen

| MCU_PIN | SIGNAL                | CONNECTOR | NOTES (timer/interrupt/alt)                          |
| ------- | --------------------- | --------- | ---------------------------------------------------- |
| GP0     | UART0_TX              | UART0     | Debug UART TX (reserved)                             |
| GP1     | UART0_RX              | UART0     | Debug UART RX (reserved)                             |
| GP2     | VL6180X_SDA           | I2C1      | VL6180X ToF Sensor SDA (FIXED - DO NOT CHANGE)       |
| GP3     | VL6180X_SCL           | I2C1      | VL6180X ToF Sensor SCL (FIXED - DO NOT CHANGE)       |
| GP4     | MOTOR_FR_PWM_CW       | PICO GP4  | Front Right Motor PWM CW                             |
| GP5     | MOTOR_FR_PWM_CCW      | PICO GP5  | Front Right Motor PWM CCW                            |
| GP6     | MOTOR_FL_ENCODER_A    | PICO GP6  | Front Left Encoder A (ext interrupt)                 |
| GP7     | MOTOR_FL_ENCODER_B    | PICO GP7  | Front Left Encoder B                                 |
| GP8     | MOTOR_FR_ENCODER_A    | PICO GP8  | Front Right Encoder A                                |
| GP9     | MOTOR_FR_ENCODER_B    | PICO GP9  | Front Right Encoder B                                |
| GP10    | MOTOR_RL_ENCODER_A    | PICO GP10 | Rear Left Encoder A                                  |
| GP11    | MOTOR_RL_ENCODER_B    | PICO GP11 | Rear Left Encoder B                                  |
| GP12    | MOTOR_RR_ENCODER_A    | PICO GP12 | Rear Right Encoder A                                 |
| GP13    | MOTOR_RR_ENCODER_B    | PICO GP13 | Rear Right Encoder B                                 |
| GP14    | MOTOR_RL_PWM_CW       | PICO GP14 | Rear Left Motor PWM CW                               |
| GP15    | MOTOR_RL_PWM_CCW      | PICO GP15 | Rear Left Motor PWM CCW                              |
| GP16    | IMU_MISO              | SPI0      | ICM20948 MISO (FIXED - DO NOT CHANGE)                |
| GP17    | IMU_CS                | SPI0      | ICM20948 CS (FIXED - DO NOT CHANGE)                  |
| GP18    | IMU_SCK               | SPI0      | ICM20948 SCK (FIXED - DO NOT CHANGE)                 |
| GP19    | IMU_MOSI              | SPI0      | ICM20948 MOSI (FIXED - DO NOT CHANGE)                |
| GP20    | MOTOR_FL_PWM_CW       | PICO GP20 | Front Left Motor PWM CW                              |
| GP21    | MOTOR_FL_PWM_CCW      | PICO GP21 | Front Left Motor PWM CCW                             |
| GP22    | MOTOR_RR_PWM_CW       | PICO GP22 | Rear Right Motor PWM CW                              |
| GP26    | LED_STATUS            | PICO GP26 | Status LED / Blink LED                               |
| GP28    | MOTOR_RR_PWM_CCW      | PICO GP28 | Rear Right Motor PWM CCW                             |

## Notes and important conflicts

**FINAL PIN ASSIGNMENTS (NO CONFLICTS):**
- UART0 Debug: GP0/GP1 (TX/RX) - RESERVED for debugging
- IMU (ICM20948): SPI0 on GP16(MISO)/GP17(CS)/GP18(SCK)/GP19(MOSI) - **FIXED - DO NOT CHANGE**
  - ROS2 Topic: `/imu/data_raw` (sensor_msgs/Imu)
  - Publishing Rate: ~50 Hz
- VL6180X ToF: I2C1 on GP2(SDA)/GP3(SCL) - **FIXED - DO NOT CHANGE**
  - ROS2 Topics:
    - `/sensors/range_tof` (sensor_msgs/Range) - Time-of-Flight distance
    - `/sensors/illuminance` (sensor_msgs/Illuminance) - Ambient light
- Motors: 4x motors with PWM (CW/CCW) + Encoders
  - Front Left: PWM=GP20/GP21, Encoders=GP6/GP7
  - Front Right: PWM=GP4/GP5, Encoders=GP8/GP9
  - Rear Left: PWM=GP14/GP15, Encoders=GP10/GP11
  - Rear Right: PWM=GP22/GP28, Encoders=GP12/GP13
  - ROS2 Topics:
    - `/joint_states` (sensor_msgs/JointState) - Encoder feedback (publish)
    - `/cmd_vel` (geometry_msgs/Twist) - Velocity commands (subscribe)
    - `/odom` (nav_msgs/Odometry) - Wheel odometry (publish)
  - Publishing Rate: ~100 Hz
- Status LED: GP26

**NERF LAUNCHER - SEPARATE CONTROLLER:**
- Controller: Arduino Nano OR Raspberry Pi Pico (separate firmware)
- Communication: Serial/UART or I2C to main Pico
- Components:
  - 2x Brushless Motors (RS2205) with ESCs
  - 2x Servos (Pan/Tilt)
  - Trigger mechanism
- Firmware: Separate repository/folder (not part of main robot firmware)
- ROS2 Interface: Via main Pico or direct USB connection

- USB console / micro-ROS transport: use TinyUSB / USB CDC (recommended). The Pico will enumerate as a serial device on the host (e.g. `/dev/ttyACM0`).
- Keep `PINMAP.md` and `firmware/.../config.h` synchronized. Update this file first, then `config.h` when changing wiring.

## ROS2 Topic Naming Convention

All firmware topics follow ROS2 best practices (REP-105):

| Sensor/Component | ROS2 Topic | Message Type | Rate | Direction |
|------------------|------------|--------------|------|-----------|
| ICM20948 IMU | `/imu/data_raw` | sensor_msgs/Imu | 50 Hz | Publish |
| VL6180X ToF | `/sensors/range_tof` | sensor_msgs/Range | Variable | Publish |
| VL6180X Light | `/sensors/illuminance` | sensor_msgs/Illuminance | Variable | Publish |
| HC-SR04 Ultrasonic | `/sensors/range_ultrasonic` | sensor_msgs/Range | Variable | Publish |
| Motor Encoders | `/joint_states` | sensor_msgs/JointState | 100 Hz | Publish |
| Wheel Odometry | `/odom` | nav_msgs/Odometry | 50 Hz | Publish |
| Velocity Commands | `/cmd_vel` | geometry_msgs/Twist | 100 Hz | Subscribe |

**Note**: The micro-ROS agent automatically adds `/rt/` prefix to firmware topics. The agent configuration includes remappings to remove this prefix for standard ROS2 compatibility.

**Migration**: Previous topic names using `/ddd/` prefix (e.g., `/ddd/imu`, `/ddd/odom`) have been replaced with standard names. See `docs/ROS2_TOPIC_MIGRATION_GUIDE.md` for details.

## Version / Change
- Board name: robot_digital_v1
- Firmware tag: v0.1
- Author: @goldjunge91
- Date: 2025-09-22 (updated to match config.h)

## Hinweise zur Implementierung
- Motor‑Power muss getrennt von MCU‑Power versorgt werden; gemeinsame GND verwenden.
- MCU läuft 3.3V; Level‑Shifter verwenden, wenn peripherie 5V erwartet.
- Encodertreiber: Prefer interrupts or PIO for high frequency.
- USB‑CDC (TinyUSB) ist empfohlen für Host‑Kommunikation (SBC <-> Pico).
- Serial Baud für micro-ROS: 115200 / 230400 / 460800 / 921600 — dokumentiere in firmware & bringup.

## Wie aktualisieren
- Ändere die Firmware‑Konstanten (board_config.h / pinmap.h) zuerst.
- Aktualisiere Version in dieser Datei und erstelle ein Firmware‑Release-Tag.
- Exportiere eine CSV für schnelle Referenz: docs/pinmap.csv

## Safety Notes
- Verwende Flyback‑Diodes und ausreichende Decoupling‑Kondensatoren an Motor‑Power Rails.
- Schütze MCU‑Pins vor Induktiven Lasten.
- Implementiere Watchdog in Firmware: sichere Motorabschaltung bei fehlender Host‑Kommunikation.

## Changelog
- 2025-10-09: Added ROS2 topic naming documentation, updated sensor topic names to follow REP-105
- 2025-09-21: Template erstellt / initial pinmap