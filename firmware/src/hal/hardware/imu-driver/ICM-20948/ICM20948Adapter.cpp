#include "ICM20948Adapter.h"
#include <cstdint>
#include <cstring>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

// ICM-20948 WHO_AM_I register and expected value
#define ICM20948_WHO_AM_I_REG 0x00
#define ICM20948_WHO_AM_I_VAL 0xEA

ICM20948Adapter::ICM20948Adapter(int i2c_bus, int address) : _i2c_bus(i2c_bus), _address(address) {}

ICM20948Adapter::~ICM20948Adapter() {}

bool ICM20948Adapter::Init() {
    // Initialize hardware I2C on the requested bus (use i2c0/i2c1 based on _i2c_bus)
    i2c_inst_t *i2c = (_i2c_bus == 1) ? i2c1 : i2c0;
    // default to 400kHz
    i2c_init(i2c, 400000);

    // Note: pin configuration for SDA/SCL is expected to be done elsewhere (bsp)

    // Probe WHO_AM_I
    uint8_t reg = ICM20948_WHO_AM_I_REG;
    uint8_t value = 0;
    int ret = i2c_write_blocking(i2c, (uint8_t)_address << 1, &reg, 1, true);
    if (ret < 0)
        return false;
    ret = i2c_read_blocking(i2c, (uint8_t)_address << 1, &value, 1, false);
    if (ret < 0)
        return false;

    if (value != ICM20948_WHO_AM_I_VAL) {
        // WHO_AM_I mismatch, try alternate 7-bit address 0x68/0x69 common variants
        // but adapter was constructed with an address; fail fast
        return false;
    }

    return true;
}

imu_queue_t ICM20948Adapter::LoopHandler() {
    imu_queue_t out;
    // Return zeroed/default data for now
    std::memset(&out, 0, sizeof(out));
    out.Orientation[0] = 1.0f;
    return out;
}
