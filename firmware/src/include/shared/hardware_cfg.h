/**
 * @file hardware_cfg.h
 * @author Maciej Kurcius, cleaned by Gemini
 * @brief Hardware pin and peripheral configuration for the Pico board.
 * @version 0.2
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HARDWARE_CFG
#define HARDWARE_CFG

#include <stdint.h>

// --- General Config ---
#define DEFAULT_FIRMWARE_MODE 2  // 0 - normal; 1 - error; 2 - debug

// --- Power Control ---
#define SBC_ETH_CONNECT_TIMEOUT 10  // ms - Note: This seems unused in current code.
#define POWEROFF_DELAY 5000         // ms - Note: This seems unused in current code.

// --- User Interface (Rear Panel) ---
#define GRN_LED 3
#define RD_LED 4
#define PUSH_BUTTON1 11
#define PUSH_BUTTON2 12

// --- On-board Peripherals ---
#define EN_LOC_5V 13 // 5V regulator enable
#define DIP_SW 2

// --- Power Board Communication (UART) ---
#define PWR_BRD_GPIO_INPUT 4
#define PWR_BRD_GPIO_OUTPUT 7
#define PWR_BRD_SERIAL_BAUDRATE 38400
#define PWR_BRD_SERIAL_TX 5
#define PWR_BRD_SERIAL_RX 6
#define PWR_BRD_SERIAL_TIMEOUT 1  // ms

// --- SBC (Single Board Computer) Communication (Pico uses stdio for this) ---
#define SBC_SERIAL_BAUDRATE 460800
#define SBC_SERIAL_RX 10
#define SBC_SERIAL_TX 9

// --- IMU (I2C) ---
#define IMU_I2C i2c0
#define IMU_SDA 0
#define IMU_SCL 1
#define IMU_GPIO_IT 2
#define IMU_SAMPLE_FREQ 25  // Hz
#define IMU_ID 0x37
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

// --- EEPROM (I2C) ---
#define EEPROM_BLOCK_ADDR_0 0x00
#define EEPROM_BLOCK_ADDR_1 0x01
#define EEPROM_BLOCK_ADDR_2 0x02
#define EEPROM_BLOCK_ADDR_3 0x03
#define EEPROM_BLOCK_ADDR_4 0x04
#define EEPROM_BLOCK_ADDR_5 0x05
#define EEPROM_BLOCK_ADDR_6 0x06
#define EEPROM_BLOCK_ADDR_7 0x07
#define EEPROM_DEV_ID 0x50
#define EEPROM_CONTROL_BYTE(DevId, BlockAddr) (DevId | BlockAddr)

// Board version eeprom defines
#define BOARD_VER_MEM_BLOCK 0x00
#define BOARD_VER_MEM_ADDR 0x00
#define BOARD_VER_MEM_SIZE 0x04
#define BOARD_VER_READ_ATTEMPTS 5

// --- Battery Message Types ---
#define BATTERY_CELLS_SERIES 3
#define BATTERY_CELLS_PARALLEL 3
#define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 1
#define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 1

typedef enum {
  unknown_status = 0,
  charging = 1,
  discharging = 2,
  not_charging = 3,
  full = 4
} BatteryStatusTypeDef;

typedef enum {
  unknown_health = 0,
  good = 1,
  overhaet = 2,
  dead = 3,
  overvoltage = 4,
  unspec_failure = 5,
  cold = 6,
  watchdog_timer_expire = 7,
  safety_timer_expire = 8
} BatteryHealthTypeDef;

typedef enum {
  unknown_type = 0,
  NIMH = 1,
  LION = 2,
  LIPO = 3,
  LIFE = 4,
  NICD = 5,
  LIMN = 6
} BatteryTechnologyTypeDef;

typedef struct
{
  // ROS battery msgs variables
  float voltage;
  float temperature;
  float current;
  float charge_current;
  float capacity;
  float design_capacity;
  float percentage;
  float cell_temperature[BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE];
  float cell_voltage[BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE];
  BatteryStatusTypeDef status;
  BatteryHealthTypeDef health;
  BatteryTechnologyTypeDef technology;
  bool present;
} battery_state_queue_t;

/* FIRMWARE MODE */

typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;

#endif /* HARDWARE_CFG */