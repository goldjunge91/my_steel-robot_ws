#include "hal/mocks/GPIOMock.hpp"

namespace hal::mocks {

    void
        GPIOMock::init(uint8_t pin) {
        pin_states_[pin].initialized = true;
    }

    void
        GPIOMock::set_dir(uint8_t pin, Direction direction) {
        pin_states_[pin].direction = direction;
    }

    void
        GPIOMock::set_function(uint8_t pin, Function function) {
        pin_states_[pin].function = function;
    }

    void
        GPIOMock::put(uint8_t pin, bool value) {
        pin_states_[pin].value = value;
    }

    bool
        GPIOMock::get(uint8_t pin) {
        auto it = pin_states_.find(pin);
        if (it != pin_states_.end()) {
            return it->second.value;
        }
        return false;
    }

    void
        GPIOMock::pull_up(uint8_t pin) {
        pin_states_[pin].pullup = true;
        pin_states_[pin].pulldown = false;
    }

    void
        GPIOMock::pull_down(uint8_t pin) {
        pin_states_[pin].pullup = false;
        pin_states_[pin].pulldown = true;
    }

    void
        GPIOMock::disable_pulls(uint8_t pin) {
        pin_states_[pin].pullup = false;
        pin_states_[pin].pulldown = false;
    }

    void
        GPIOMock::set_pulls(uint8_t pin, bool up, bool down) {
        pin_states_[pin].pullup = up;
        pin_states_[pin].pulldown = down;
    }

    bool
        GPIOMock::is_initialized(uint8_t pin) const {
        auto it = pin_states_.find(pin);
        return it != pin_states_.end() && it->second.initialized;
    }

    GPIOMock::Direction
        GPIOMock::get_direction(uint8_t pin) const {
        auto it = pin_states_.find(pin);
        if (it != pin_states_.end()) {
            return it->second.direction;
        }
        return INPUT;
    }

    GPIOMock::Function
        GPIOMock::get_function(uint8_t pin) const {
        auto it = pin_states_.find(pin);
        if (it != pin_states_.end()) {
            return it->second.function;
        }
        return FUNC_SIO;
    }

    bool
        GPIOMock::is_pullup_enabled(uint8_t pin) const {
        auto it = pin_states_.find(pin);
        return it != pin_states_.end() && it->second.pullup;
    }

    bool
        GPIOMock::is_pulldown_enabled(uint8_t pin) const {
        auto it = pin_states_.find(pin);
        return it != pin_states_.end() && it->second.pulldown;
    }

    void
        GPIOMock::reset() {
        pin_states_.clear();
    }

}  // namespace hal::mocks
