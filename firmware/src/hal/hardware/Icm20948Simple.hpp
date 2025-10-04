#pragma once

#include "hardware/spi.h"
#include "shared/Vector3f.hpp"

#include <cstdint>

namespace hal::hardware {

class Icm20948Simple {
public:
    struct Config {
        spi_inst_t* bus;
        uint32_t baudrate_hz;
        uint8_t cs_pin;
        uint8_t sck_pin;
        uint8_t mosi_pin;
        uint8_t miso_pin;
    };

    explicit Icm20948Simple(const Config& config);

    bool initialize();
    bool readAcceleration(shared::Vector3f& accel_g);
    bool readGyroscope(shared::Vector3f& gyro_dps);
    bool readTemperature(float& temperature_c);

private:
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool selectRegisterBank(uint8_t bank);

    Config config_;
    bool initialized_ = false;
    uint8_t current_bank_ = 0xFF;  // Ungültiger Startwert, um Umschaltung zu erzwingen
};

}  // namespace hal::hardware
