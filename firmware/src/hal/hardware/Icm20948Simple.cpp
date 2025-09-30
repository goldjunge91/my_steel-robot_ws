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
    // DUMMY MODE: Skip hardware initialization for testing
    printf("[ICM20948Simple] DUMMY MODE: Skipping hardware initialization\n");
    printf("[ICM20948Simple] Simulating successful initialization\n");
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