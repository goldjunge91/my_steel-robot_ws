#include "hal/mocks/I2CMock.hpp"
#include <cstring>

namespace hal::mocks {

    bool
        I2CMock::init(uint8_t bus_id, uint32_t baudrate_hz) {
        if (simulate_error_) {
            return false;
        }

        bus_id_ = bus_id;
        baudrate_hz_ = baudrate_hz;
        initialized_ = true;
        return true;
    }

    void
        I2CMock::configure_pins(uint8_t sda_pin, uint8_t scl_pin, bool enable_pullups) {
        sda_pin_ = sda_pin;
        scl_pin_ = scl_pin;
        pullups_enabled_ = enable_pullups;
    }

    int
        I2CMock::write_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            const uint8_t* data,
            size_t length,
            bool nostop
        ) {
        if (simulate_error_ || !initialized_ || data == nullptr || length == 0) {
            return -1;
        }

        write_count_++;

        // First byte is register address, rest is data
        if (length >= 2) {
            uint8_t reg_addr = data[0];
            for (size_t i = 1; i < length; i++) {
                device_registers_[device_addr][reg_addr + (i - 1)] = data[i];
            }
        }

        return static_cast<int>(length);
    }

    int
        I2CMock::read_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t* data,
            size_t length,
            bool nostop
        ) {
        if (simulate_error_ || !initialized_ || data == nullptr || length == 0) {
            return -1;
        }

        read_count_++;

        // Return zeros if device/registers not set
        std::memset(data, 0, length);

        return static_cast<int>(length);
    }

    bool
        I2CMock::write_read(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t reg_addr,
            uint8_t* data,
            size_t length
        ) {
        if (simulate_error_ || !initialized_ || data == nullptr || length == 0) {
            return false;
        }

        write_count_++;
        read_count_++;

        // Read consecutive registers starting from reg_addr
        auto device_it = device_registers_.find(device_addr);
        if (device_it != device_registers_.end()) {
            for (size_t i = 0; i < length; i++) {
                auto reg_it = device_it->second.find(reg_addr + i);
                if (reg_it != device_it->second.end()) {
                    data[i] = reg_it->second;
                }
                else {
                    data[i] = 0;  // Default value for unset registers
                }
            }
        }
        else {
            std::memset(data, 0, length);
        }

        return true;
    }

    void
        I2CMock::set_register(uint8_t device_addr, uint8_t reg_addr, uint8_t value) {
        device_registers_[device_addr][reg_addr] = value;
    }

    uint8_t
        I2CMock::get_register(uint8_t device_addr, uint8_t reg_addr) const {
        auto device_it = device_registers_.find(device_addr);
        if (device_it != device_registers_.end()) {
            auto reg_it = device_it->second.find(reg_addr);
            if (reg_it != device_it->second.end()) {
                return reg_it->second;
            }
        }
        return 0;
    }

    void
        I2CMock::set_registers(uint8_t device_addr, uint8_t start_reg, const uint8_t* data, size_t length) {
        for (size_t i = 0; i < length; i++) {
            device_registers_[device_addr][start_reg + i] = data[i];
        }
    }

    void
        I2CMock::simulate_error(bool enable) {
        simulate_error_ = enable;
    }

    void
        I2CMock::reset() {
        device_registers_.clear();
        write_count_ = 0;
        read_count_ = 0;
        initialized_ = false;
    }

}  // namespace hal::mocks
