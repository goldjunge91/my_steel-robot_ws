// Minimal hardware_cfg.h stub for host builds. Fill in pin definitions as
// needed by tests. These are placeholders only.
#pragma once

#define FL_PWM 0
#define FL_IN1 1
#define FL_IN2 2

// Encoder channels
#define ENC_FL_A 10

// Battery constants
#define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 1
#define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 1

// Battery types
typedef enum {
    unknown_type = 0,
    NIMH = 1,
    LION = 2,
    LIPO = 3,
    LIFE = 4,
    NICD = 5,
    LIMN = 6
} BatteryTechnologyTypeDef;

typedef enum { POWER_SUPPLY_STATUS_UNKNOWN = 0 } BatteryStatusTypeDef;
typedef enum { POWER_SUPPLY_HEALTH_UNKNOWN = 0 } BatteryHealthTypeDef;

typedef struct {
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

// Firmware mode
typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;

// Add other mappings required by tests as needed.
