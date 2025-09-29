#ifndef CONFIG_H
#define CONFIG_H

// === COMMUNICATION INTERFACES ===

// IMU (I2C bus 0)
#define IMU_I2C i2c0
#define IMU_SDA 0
#define IMU_SCL 1
#define IMU_GPIO_IT 2
#define IMU_SAMPLE_FREQ 25 // Hz
#define IMU_ID 0x37
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

// Battery Monitor (INA3221 on I2C bus 0)
#define BATTERY_I2C i2c0
#define BATTERY_ADDR 0x40 // INA3221 default address

// Power Board Communication (UART)
#define PWR_BRD_SERIAL_TX 5
#define PWR_BRD_SERIAL_RX 6
#define PWR_BRD_SERIAL_BAUDRATE 38400
#define PWR_BRD_SERIAL_TIMEOUT 1 // ms

// === MOTOR CONTROL ===

// TB6612 Motor Driver Pins
#define STBY 28 // Common standby for both TB6612s

// Front-Left Motor (TB1 A)
#define FL_IN1 18
#define FL_IN2 19
#define FL_PWM 2
#define ENC_FL_A 8
#define ENC_FL_B 9

// Front-Right Motor (TB1 B)
#define FR_IN1 17
#define FR_IN2 20
#define FR_PWM 3
#define ENC_FR_A 10
#define ENC_FR_B 11

// Rear-Left Motor (TB2 A)
#define RL_IN1 21
#define RL_IN2 22
#define RL_PWM 4
#define ENC_RL_A 12
#define ENC_RL_B 13

// Rear-Right Motor (TB2 B)
#define RR_IN1 26
#define RR_IN2 27
#define RR_PWM 5
#define ENC_RR_A 6
#define ENC_RR_B 7

// === SHOOTER CONTROL ===

#define ESC1_PIN 14
#define ESC2_PIN 15
#define GEAR_PIN 16
#define PAN_PIN 23
#define TILT_PIN 24

// === USER INTERFACE ===

#define GRN_LED 3
#define RD_LED 4
#define PUSH_BUTTON1 11
#define PUSH_BUTTON2 12

// === ON-BOARD PERIPHERALS ===

#define EN_LOC_5V 13 // 5V regulator enable
#define DIP_SW 2

#endif // CONFIG_H