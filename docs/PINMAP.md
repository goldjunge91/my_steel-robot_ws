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
| GP2     | MOTOR_FL_PWM (FL_PWM) | PICO GP2  | PWM output (motor A PWM)                             |
| GP18    | MOTOR_FL_DIR_IN1      | PICO GP18 | Motor direction input (IN1)                          |
| GP19    | MOTOR_FL_DIR_IN2      | PICO GP19 | Motor direction input (IN2)                          |
| GP8     | MOTOR_FL_ENCODER_A    | PICO GP8  | Encoder A (ext interrupt)                            |
| GP9     | MOTOR_FL_ENCODER_B    | PICO GP9  | Encoder B                                            |
| GP3     | MOTOR_FR_PWM (FR_PWM) | PICO GP3  | PWM output (motor B PWM)                             |
| GP17    | MOTOR_FR_DIR_IN1      | PICO GP17 | Motor direction input (IN1)                          |
| GP20    | MOTOR_FR_DIR_IN2      | PICO GP20 | Motor direction input (IN2)                          |
| GP10    | MOTOR_FR_ENCODER_A    | PICO GP10 | Encoder A                                            |
| GP11    | MOTOR_FR_ENCODER_B    | PICO GP11 | Encoder B                                            |
| GP4     | MOTOR_RL_PWM (RL_PWM) | PICO GP4  | PWM output (motor C PWM)                             |
| GP21    | MOTOR_RL_DIR_IN1      | PICO GP21 | Motor direction input (IN1)                          |
| GP22    | MOTOR_RL_DIR_IN2      | PICO GP22 | Motor direction input (IN2)                          |
| GP12    | MOTOR_RL_ENCODER_A    | PICO GP12 | Encoder A                                            |
| GP13    | MOTOR_RL_ENCODER_B    | PICO GP13 | Encoder B                                            |
| GP5     | MOTOR_RR_PWM (RR_PWM) | PICO GP5  | PWM output (motor D PWM)                             |
| GP26    | MOTOR_RR_DIR_IN1      | PICO GP26 | Motor direction input (IN1)                          |
| GP27    | MOTOR_RR_DIR_IN2      | PICO GP27 | Motor direction input (IN2)                          |
| GP6     | MOTOR_RR_ENCODER_A    | PICO GP6  | Encoder A                                            |
| GP7     | MOTOR_RR_ENCODER_B    | PICO GP7  | Encoder B                                            |
| GP28    | TB6612_STBY (STBY)    | PICO GP28 | Shared TB6612 standby pin                            |
| GP14    | ESC1_PWM (ESC1_PIN)   | PICO GP14 | ESC / servo PWM                                      |
| GP15    | ESC2_PWM (ESC2_PIN)   | PICO GP15 | ESC / servo PWM                                      |
| GP16    | GEAR_PIN              | PICO GP16 | Gear / digital output                                |
| GP12    | I2C_SDA               | I2C1      | ICM20948 SDA (shared with RL encoder pin assignment) |
| GP13    | I2C_SCL               | I2C1      | ICM20948 SCL                                         |

## Notes and important conflicts

- The table above was updated to reflect the pin assignments used in `firmware/mecabridge_pico/src/config.h` (the mapping you provided).
- ADC / battery measurement: the previous template used `ADC0` for battery voltage; in the current pinmap many ADC-capable pins (GP26..GP29) are already used (e.g. GP26 is assigned to RR direction pin in `config.h`). The firmware does not currently assign a dedicated ADC pin for battery measurement in `config.h`. Recommendation: choose an unused ADC-capable pin (GP26..GP29) and update `config.h` accordingly, or remap the RR motor pins to free GP26 if battery monitor is required.
- USB console / micro-ROS transport: use TinyUSB / USB CDC (recommended). The Pico will enumerate as a serial device on the host (e.g. `/dev/ttyACM0`).
- Keep `PINMAP.md` and `firmware/.../config.h` synchronized. Update this file first, then `config.h` when changing wiring.

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
- 2025-09-21: Template erstellt / initial pinmap