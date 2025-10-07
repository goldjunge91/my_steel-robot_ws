#include "hal/pico/GPIOPico.hpp"

namespace hal::pico {

#ifndef PICO_PLATFORM_HOST

    gpio_function
        GPIOPico::convert_function(Function func) {
        switch (func) {
        case FUNC_XIP:    return GPIO_FUNC_XIP;
        case FUNC_SPI:    return GPIO_FUNC_SPI;
        case FUNC_UART:   return GPIO_FUNC_UART;
        case FUNC_I2C:    return GPIO_FUNC_I2C;
        case FUNC_PWM:    return GPIO_FUNC_PWM;
        case FUNC_SIO:    return GPIO_FUNC_SIO;
        case FUNC_PIO0:   return GPIO_FUNC_PIO0;
        case FUNC_PIO1:   return GPIO_FUNC_PIO1;
        case FUNC_GPCK:   return GPIO_FUNC_GPCK;
        case FUNC_USB:    return GPIO_FUNC_USB;
        case FUNC_NULL:   return GPIO_FUNC_NULL;
        default:          return GPIO_FUNC_SIO;
        }
    }

    void
        GPIOPico::init(uint8_t pin) {
        gpio_init(pin);
    }

    void
        GPIOPico::set_dir(uint8_t pin, Direction direction) {
        gpio_set_dir(pin, direction == OUTPUT);
    }

    void
        GPIOPico::set_function(uint8_t pin, Function function) {
        gpio_set_function(pin, convert_function(function));
    }

    void
        GPIOPico::put(uint8_t pin, bool value) {
        gpio_put(pin, value);
    }

    bool
        GPIOPico::get(uint8_t pin) {
        return gpio_get(pin);
    }

    void
        GPIOPico::pull_up(uint8_t pin) {
        gpio_pull_up(pin);
    }

    void
        GPIOPico::pull_down(uint8_t pin) {
        gpio_pull_down(pin);
    }

    void
        GPIOPico::disable_pulls(uint8_t pin) {
        gpio_disable_pulls(pin);
    }

    void
        GPIOPico::set_pulls(uint8_t pin, bool up, bool down) {
        gpio_set_pulls(pin, up, down);
    }

#else  // PICO_PLATFORM_HOST

    // Stub implementations for host build (should use mocks instead)
    void GPIOPico::init(uint8_t pin) {}
    void GPIOPico::set_dir(uint8_t pin, Direction direction) {}
    void GPIOPico::set_function(uint8_t pin, Function function) {}
    void GPIOPico::put(uint8_t pin, bool value) {}
    bool GPIOPico::get(uint8_t pin) { return false; }
    void GPIOPico::pull_up(uint8_t pin) {}
    void GPIOPico::pull_down(uint8_t pin) {}
    void GPIOPico::disable_pulls(uint8_t pin) {}
    void GPIOPico::set_pulls(uint8_t pin, bool up, bool down) {}

#endif

}  // namespace hal::pico
