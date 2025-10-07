#pragma once

#include "hal/interfaces/IGPIOInterface.hpp"

// Only include Pico SDK headers when building for Pico
#ifndef PICO_PLATFORM_HOST
#include <hardware/gpio.h>
#endif

namespace hal::pico {

    /**
     * @brief Pico SDK implementation of GPIO interface
     *
     * This class wraps real Pico SDK GPIO calls for production use.
     * It implements the same interface as GPIOMock, allowing seamless
     * switching between real hardware and testing.
     */
    class GPIOPico : public hal::interfaces::IGPIOInterface {
    public:
        GPIOPico() = default;
        ~GPIOPico() override = default;

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

    private:
#ifndef PICO_PLATFORM_HOST
        static gpio_function convert_function(Function func);
#endif
    };

}  // namespace hal::pico
