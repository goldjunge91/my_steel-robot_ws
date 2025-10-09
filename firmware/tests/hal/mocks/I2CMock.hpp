#pragma once

#include "hal/interfaces/II2CInterface.hpp"
#include <map>
#include <vector>

namespace hal::mocks {

    /**
     * @brief Mock implementation of I2C interface for testing
     *
     * This class simulates I2C hardware by maintaining virtual register banks
     * for each device address. Perfect for unit testing I2C-based sensors
     * like the ICM-20948 IMU without real hardware.
     *
     * Features:
     * - Per-device register storage
     * - Realistic read/write operations
     * - Error injection capabilities
     * - Transaction tracking for verification
     */
    class I2CMock : public hal::interfaces::II2CInterface {
    public:
        I2CMock() = default;
        ~I2CMock() override = default;

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

        // Test helper methods

        /**
         * @brief Set a register value for a specific device
         * @param device_addr Device address
         * @param reg_addr Register address
         * @param value Register value
         */
        void set_register(uint8_t device_addr, uint8_t reg_addr, uint8_t value);

        /**
         * @brief Get a register value for a specific device
         * @param device_addr Device address
         * @param reg_addr Register address
         * @return Register value (0 if not set)
         */
        uint8_t get_register(uint8_t device_addr, uint8_t reg_addr) const;

        /**
         * @brief Set multiple consecutive registers (useful for sensor data)
         * @param device_addr Device address
         * @param start_reg Starting register address
         * @param data Pointer to data array
         * @param length Number of bytes to set
         */
        void set_registers(uint8_t device_addr, uint8_t start_reg, const uint8_t* data, size_t length);

        /**
         * @brief Enable/disable error simulation
         * @param enable If true, all operations will fail
         */
        void simulate_error(bool enable);

        /**
         * @brief Reset all register values
         */
        void reset();

        /**
         * @brief Get number of write transactions performed
         */
        size_t get_write_count() const { return write_count_; }

        /**
         * @brief Get number of read transactions performed
         */
        size_t get_read_count() const { return read_count_; }

        /**
         * @brief Check if device was initialized
         */
        bool is_initialized() const { return initialized_; }

    private:
        // Per-device register banks: device_addr -> (reg_addr -> value)
        std::map<uint8_t, std::map<uint8_t, uint8_t>> device_registers_;

        bool initialized_ = false;
        uint8_t bus_id_ = 0;
        uint32_t baudrate_hz_ = 0;
        uint8_t sda_pin_ = 0;
        uint8_t scl_pin_ = 0;
        bool pullups_enabled_ = false;

        bool simulate_error_ = false;
        size_t write_count_ = 0;
        size_t read_count_ = 0;
    };

}  // namespace hal::mocks
