#include "hal/pico/I2CPico.hpp"

namespace hal::pico {

#ifndef PICO_PLATFORM_HOST

    i2c_inst_t*
        I2CPico::get_i2c_instance(uint8_t bus_id) const {
        return bus_id == 0 ? i2c0 : i2c1;
    }

    bool
        I2CPico::init(uint8_t bus_id, uint32_t baudrate_hz) {
        i2c_inst_t* bus = get_i2c_instance(bus_id);
        uint32_t actual_baudrate = i2c_init(bus, baudrate_hz);
        return actual_baudrate > 0;
    }

    void
        I2CPico::configure_pins(uint8_t sda_pin, uint8_t scl_pin, bool enable_pullups) {
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);

        if (enable_pullups) {
            gpio_pull_up(sda_pin);
            gpio_pull_up(scl_pin);
        }
    }

    int
        I2CPico::write_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            const uint8_t* data,
            size_t length,
            bool nostop
        ) {
        i2c_inst_t* bus = get_i2c_instance(bus_id);
        return i2c_write_blocking(bus, device_addr, data, length, nostop);
    }

    int
        I2CPico::read_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t* data,
            size_t length,
            bool nostop
        ) {
        i2c_inst_t* bus = get_i2c_instance(bus_id);
        return i2c_read_blocking(bus, device_addr, data, length, nostop);
    }

    bool
        I2CPico::write_read(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t reg_addr,
            uint8_t* data,
            size_t length
        ) {
        i2c_inst_t* bus = get_i2c_instance(bus_id);

        // Write register address
        int written = i2c_write_blocking(bus, device_addr, &reg_addr, 1, true);
        if (written != 1) {
            return false;
        }

        // Read data
        int read = i2c_read_blocking(bus, device_addr, data, length, false);
        return read == static_cast<int>(length);
    }

#else  // PICO_PLATFORM_HOST

    // Stub implementations for host build (should use mocks instead)
    bool I2CPico::init(uint8_t bus_id, uint32_t baudrate_hz) { return true; }
    void I2CPico::configure_pins(uint8_t sda_pin, uint8_t scl_pin, bool enable_pullups) {}
    int I2CPico::write_blocking(uint8_t bus_id, uint8_t device_addr, const uint8_t* data, size_t length, bool nostop) { return length; }
    int I2CPico::read_blocking(uint8_t bus_id, uint8_t device_addr, uint8_t* data, size_t length, bool nostop) { return length; }
    bool I2CPico::write_read(uint8_t bus_id, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, size_t length) { return true; }

#endif

}  // namespace hal::pico
