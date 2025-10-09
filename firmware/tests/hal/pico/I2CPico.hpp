#pragma once

#include "hal/interfaces/II2CInterface.hpp"

// Only include Pico SDK headers when building for Pico
#ifndef PICO_PLATFORM_HOST
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#endif

namespace hal::pico {

    /**
     * @brief Pico SDK implementation of I2C interface
     *
     * This class wraps real Pico SDK I2C calls for production use.
     * It implements the same interface as I2CMock, allowing seamless
     * switching between real hardware and testing.
     */
    class I2CPico : public hal::interfaces::II2CInterface {
    public:
        I2CPico() = default;
        ~I2CPico() override = default;

        // II2CInterface implementation
        bool init(uint8_t bus_id, uint32_t baudrate_hz) override;
        void configure_pins(uint8_t sda_pin, uint8_t scl_pin, bool enable_pullups) override;

        int write_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            const uint8_t* data,
            size_t length,
            bool nostop
        ) override;

        int read_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t* data,
            size_t length,
            bool nostop
        ) override;

        bool write_read(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t reg_addr,
            uint8_t* data,
            size_t length
        ) override;

    private:
#ifndef PICO_PLATFORM_HOST
        i2c_inst_t* get_i2c_instance(uint8_t bus_id) const;
#endif
    };

}  // namespace hal::pico
