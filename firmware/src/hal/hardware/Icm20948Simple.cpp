#include "hal/hardware/Icm20948Simple.hpp"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>
#include <cstring>
#include <cstdio>

// ICM-20948 WHO_AM_I register and expected value
#define ICM20948_WHO_AM_I_REG 0x00
#define ICM20948_WHO_AM_I_VAL 0xEA

namespace hal::hardware {

Icm20948Simple::Icm20948Simple(const Config& config)
    : config_(config), initialized_(false) {}

bool Icm20948Simple::initialize() {
    if (config_.bus == nullptr) {
        return false;
    }

    // Initialize I2C
    i2c_init(config_.bus, config_.baudrate_hz);
    gpio_set_function(config_.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config_.scl_pin, GPIO_FUNC_I2C);
    
    if (config_.enable_pullups) {
        gpio_pull_up(config_.sda_pin);
        gpio_pull_up(config_.scl_pin);
    }

    sleep_ms(10);

    // Probe WHO_AM_I register
    uint8_t reg = ICM20948_WHO_AM_I_REG;
    uint8_t value = 0;
    
    int ret = i2c_write_blocking(config_.bus, config_.address, &reg, 1, true);
    if (ret < 0) {
        printf("[ICM20948Simple] Failed to write WHO_AM_I register\n");
        return false;
    }
    
    ret = i2c_read_blocking(config_.bus, config_.address, &value, 1, false);
    if (ret < 0) {
        printf("[ICM20948Simple] Failed to read WHO_AM_I register\n");
        return false;
    }

    if (value != ICM20948_WHO_AM_I_VAL) {
        printf("[ICM20948Simple] WHO_AM_I mismatch: expected 0x%02X, got 0x%02X\n", 
               ICM20948_WHO_AM_I_VAL, value);
        return false;
    }

    printf("[ICM20948Simple] WHO_AM_I check passed: 0x%02X\n", value);
    initialized_ = true;
    return true;
}

bool Icm20948Simple::readAcceleration(shared::Vector3f& accel_g) {
    if (!initialized_) {
        return false;
    }
    
    // Return dummy data for now (safe implementation)
    accel_g.x = 0.0f;
    accel_g.y = 0.0f;
    accel_g.z = 1.0f;  // 1g in Z direction (gravity)
    return true;
}

bool Icm20948Simple::readGyroscope(shared::Vector3f& gyro_dps) {
    if (!initialized_) {
        return false;
    }
    
    // Return dummy data for now (safe implementation)
    gyro_dps.x = 0.0f;
    gyro_dps.y = 0.0f;
    gyro_dps.z = 0.0f;
    return true;
}

bool Icm20948Simple::readTemperature(float& temperature_c) {
    if (!initialized_) {
        return false;
    }
    
    // Return dummy temperature
    temperature_c = 25.0f;  // Room temperature
    return true;
}

}  // namespace hal::hardware