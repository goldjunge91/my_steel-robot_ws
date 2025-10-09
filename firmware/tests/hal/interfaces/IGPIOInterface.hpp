#pragma once

#include <cstdint>

namespace hal::interfaces {

    /**
     * @brief Abstract interface for GPIO hardware operations
     *
     * This interface abstracts GPIO operations to enable testing
     * without real hardware. Implementations can be:
     * - GPIOPico: Real Pico SDK hardware calls
     * - GPIOMock: Simulated GPIO for unit tests
     */
    class IGPIOInterface {
    public:
        virtual ~IGPIOInterface() = default;

        // GPIO direction constants
        enum Direction {
            INPUT = 0,
            OUTPUT = 1
        };

        // GPIO function constants (matching Pico SDK)
        enum Function {
            FUNC_XIP = 0,
            FUNC_SPI = 1,
            FUNC_UART = 2,
            FUNC_I2C = 3,
            FUNC_PWM = 4,
            FUNC_SIO = 5,
            FUNC_PIO0 = 6,
            FUNC_PIO1 = 7,
            FUNC_GPCK = 8,
            FUNC_USB = 9,
            FUNC_NULL = 0x1f
        };

        /**
         * @brief Initialize a GPIO pin
         * @param pin Pin number
         */
        virtual void init(uint8_t pin) = 0;

        /**
         * @brief Set GPIO pin direction
         * @param pin Pin number
         * @param direction INPUT or OUTPUT
         */
        virtual void set_dir(uint8_t pin, Direction direction) = 0;

        /**
         * @brief Set GPIO pin function
         * @param pin Pin number
         * @param function Function to set (SPI, I2C, PWM, etc.)
         */
        virtual void set_function(uint8_t pin, Function function) = 0;

        /**
         * @brief Set GPIO pin output value
         * @param pin Pin number
         * @param value true for HIGH, false for LOW
         */
        virtual void put(uint8_t pin, bool value) = 0;

        /**
         * @brief Get GPIO pin input value
         * @param pin Pin number
         * @return true if HIGH, false if LOW
         */
        virtual bool get(uint8_t pin) = 0;

        /**
         * @brief Enable pull-up resistor on GPIO pin
         * @param pin Pin number
         */
        virtual void pull_up(uint8_t pin) = 0;

        /**
         * @brief Enable pull-down resistor on GPIO pin
         * @param pin Pin number
         */
        virtual void pull_down(uint8_t pin) = 0;

        /**
         * @brief Disable pull-up/pull-down resistors on GPIO pin
         * @param pin Pin number
         */
        virtual void disable_pulls(uint8_t pin) = 0;

        /**
         * @brief Set pulls on GPIO pin
         * @param pin Pin number
         * @param up Enable pull-up
         * @param down Enable pull-down
         */
        virtual void set_pulls(uint8_t pin, bool up, bool down) = 0;
    };

}  // namespace hal::interfaces
