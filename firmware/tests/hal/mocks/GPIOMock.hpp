#pragma once

#include "hal/interfaces/IGPIOInterface.hpp"
#include <map>

namespace hal::mocks {

    /**
     * @brief Mock implementation of GPIO interface for testing
     *
     * This class simulates GPIO hardware by maintaining virtual pin states.
     * Perfect for unit testing without real hardware.
     *
     * Features:
     * - Pin state tracking (HIGH/LOW)
     * - Direction tracking (INPUT/OUTPUT)
     * - Function tracking (I2C, SPI, PWM, etc.)
     * - Pull-up/pull-down state tracking
     * - Error injection capabilities
     */
    class GPIOMock : public hal::interfaces::IGPIOInterface {
    public:
        GPIOMock() = default;
        ~GPIOMock() override = default;

        // IGPIOInterface implementation
        void init(uint8_t pin) override;
        void set_dir(uint8_t pin, Direction direction) override;
        void set_function(uint8_t pin, Function function) override;
        void put(uint8_t pin, bool value) override;
        bool get(uint8_t pin) override;
        void pull_up(uint8_t pin) override;
        void pull_down(uint8_t pin) override;
        void disable_pulls(uint8_t pin) override;
        void set_pulls(uint8_t pin, bool up, bool down) override;

        // Test helper methods

        /**
         * @brief Check if a pin was initialized
         */
        bool is_initialized(uint8_t pin) const;

        /**
         * @brief Get the direction of a pin
         */
        Direction get_direction(uint8_t pin) const;

        /**
         * @brief Get the function of a pin
         */
        Function get_function(uint8_t pin) const;

        /**
         * @brief Check if pull-up is enabled
         */
        bool is_pullup_enabled(uint8_t pin) const;

        /**
         * @brief Check if pull-down is enabled
         */
        bool is_pulldown_enabled(uint8_t pin) const;

        /**
         * @brief Reset all pin states
         */
        void reset();

    private:
        struct PinState {
            bool initialized = false;
            bool value = false;
            Direction direction = INPUT;
            Function function = FUNC_SIO;
            bool pullup = false;
            bool pulldown = false;
        };

        std::map<uint8_t, PinState> pin_states_;
    };

}  // namespace hal::mocks
