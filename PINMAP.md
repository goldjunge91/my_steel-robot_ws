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

| MCU_PIN | SIGNAL             | CONNECTOR | NOTES (timer/interrupt/alt) |
| ------- | ------------------ | --------- | --------------------------- |
| GP0     | MOTOR_FL_ENCODER_A | J1-1      | encoder A, ext interrupt    |
| GP1     | MOTOR_FL_ENCODER_B | J1-2      | encoder B                   |
| GP2     | MOTOR_FL_PWM       | J2-1      | PWM (Timer X channel)       |
| GP3     | MOTOR_FL_DIR       | J2-2      | digital output              |
| GP4     | MOTOR_FR_PWM       | J3-1      | PWM                         |
| GP5     | MOTOR_FR_DIR       | J3-2      |                             |
| GP6     | MOTOR_RL_PWM       | J4-1      |                             |
| GP7     | MOTOR_RL_DIR       | J4-2      |                             |
| GP8     | MOTOR_RR_PWM       | J5-1      |                             |
| GP9     | MOTOR_RR_DIR       | J5-2      |                             |
| GP10    | UART_TX (console)  | CONSOLE   | USB-CDC TX (to SBC)         |
| GP11    | UART_RX (console)  | CONSOLE   | USB-CDC RX (to SBC)         |
| GP12    | I2C_SDA            | I2C1      | ICM20948 SDA, pull-ups 4.7k |
| GP13    | I2C_SCL            | I2C1      | ICM20948 SCL                |
| ADC0    | BATTERY_VOLTAGE    | BATT_MON  | via voltage divider         |
| GP14    | PWM_SERVO_PAN      | SERVO_J1  | 50Hz typical                |
| GP15    | PWM_SERVO_TILT     | SERVO_J2  |                             |
| GP16    | GPIO_VL53L0X_XSHUT | TOF1      | optional reset              |

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
- 2025-09-21: Template erstellt / initial pinmap