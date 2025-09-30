#pragma once

#include <cstddef>
#include <cstdint>
#include "hardware/i2c.h"
#include "shared/Vector3f.hpp"

namespace hal::hardware {

class Icm20948Simple {
public:
    struct Config {
        i2c_inst_t* bus;
        uint32_t baudrate_hz;
        uint8_t address;
        uint8_t sda_pin;
        uint8_t scl_pin;
        bool enable_pullups;
    };

    explicit Icm20948Simple(const Config& config);

    bool initialize();
    bool readAcceleration(shared::Vector3f& accel_g);
    bool readGyroscope(shared::Vector3f& gyro_dps);
    bool readTemperature(float& temperature_c);

private:
    Config config_;
    bool initialized_;
};

}  // namespace hal::hardware