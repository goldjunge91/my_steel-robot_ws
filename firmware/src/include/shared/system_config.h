#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <cstdbool>
#include <cstdint>

#define PWM_MAX 255
#define WATCHDOG_MS 300
#define LOOP_MS 10
#define DEADZONE_DEFAULT 30
#define SLEW_DEFAULT 8

typedef struct {
    uint8_t pwm_max;
    uint16_t watchdog_ms;
    uint8_t loop_ms;
    uint8_t deadzone;
    uint8_t slew_rate;
} system_config_t;

// T024: BatteryStatus data structure
typedef struct {
    int32_t voltage_mv;      // Battery voltage in millivolts
    int32_t current_ma;      // Battery current in milliamps
    uint8_t soc_percent;     // State of charge in percent
    uint32_t last_sample_ms; // Timestamp of the last sample
} BatteryStatus;

#endif // SYSTEM_CONFIG_H
