#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace hal::hardware {

class Vl6180x {
public:
    struct Config {
        i2c_inst_t* bus;
        uint8_t sda_pin;
        uint8_t scl_pin;
    };

    explicit Vl6180x(const Config& config);

    bool initialize();
    uint8_t readRange();
    float readLux();

private:
    // Low-level I2C-Funktionen, die den Erfolg zurückgeben
    bool writeRegister(uint16_t reg, uint8_t value);
    bool readRegister(uint16_t reg, uint8_t& value);
    bool readRegister16(uint16_t reg, uint16_t& value);

    Config config_;
};

}  // namespace hal::hardware
