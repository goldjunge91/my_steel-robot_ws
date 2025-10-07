#pragma once

#include <cstddef>
#include <cstdint>

namespace hal::interfaces {

    /**
     * @brief Abstract interface for I2C hardware operations
     *
     * This interface abstracts I2C communication to enable testing
     * without real hardware. Implementations can be:
     * - I2CPico: Real Pico SDK hardware calls
     * - I2CMock: Simulated I2C for unit tests
     */
    class II2CInterface {
    public:
        virtual ~II2CInterface() = default;

        /**
         * @brief Initialize I2C bus with specified baudrate
         * @param bus_id I2C bus identifier (e.g., 0 or 1)
         * @param baudrate_hz Baudrate in Hz (e.g., 400000 for 400kHz)
         * @return true if initialization successful
         */
        virtual bool init(uint8_t bus_id, uint32_t baudrate_hz) = 0;

        /**
         * @brief Configure GPIO pins for I2C function
         * @param sda_pin SDA pin number
         * @param scl_pin SCL pin number
         * @param enable_pullups Enable internal pull-up resistors
         */
        virtual void configure_pins(uint8_t sda_pin, uint8_t scl_pin, bool enable_pullups) = 0;

        /**
         * @brief Write data to I2C device
         * @param bus_id I2C bus identifier
         * @param device_addr 7-bit device address
         * @param data Pointer to data buffer
         * @param length Number of bytes to write
         * @param nostop If true, don't send STOP condition (for repeated start)
         * @return Number of bytes written, or negative on error
         */
        virtual int write_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            const uint8_t* data,
            size_t length,
            bool nostop
        ) = 0;

        /**
         * @brief Read data from I2C device
         * @param bus_id I2C bus identifier
         * @param device_addr 7-bit device address
         * @param data Pointer to buffer for received data
         * @param length Number of bytes to read
         * @param nostop If true, don't send STOP condition
         * @return Number of bytes read, or negative on error
         */
        virtual int read_blocking(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t* data,
            size_t length,
            bool nostop
        ) = 0;

        /**
         * @brief Write register address and read data (common I2C pattern)
         * @param bus_id I2C bus identifier
         * @param device_addr 7-bit device address
         * @param reg_addr Register address to read from
         * @param data Pointer to buffer for received data
         * @param length Number of bytes to read
         * @return true if successful
         */
        virtual bool write_read(
            uint8_t bus_id,
            uint8_t device_addr,
            uint8_t reg_addr,
            uint8_t* data,
            size_t length
        ) = 0;
    };

}  // namespace hal::interfaces
