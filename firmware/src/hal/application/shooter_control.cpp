#include "application/shooter_control.h"
#include "hardware/gpio.h"
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>

ShooterControl::ShooterControl(uint8_t esc1_pin, uint8_t esc2_pin, uint8_t gear_pin,
                               uint8_t pan_pin, uint8_t tilt_pin)
    : esc1_pin_(esc1_pin), esc2_pin_(esc2_pin), gear_pin_(gear_pin), pan_pin_(pan_pin),
      tilt_pin_(tilt_pin) {
    state_.pan_position = 0.0f;
    state_.tilt_position = 0.0f;
    state_.pan_target = 0.0f;
    state_.tilt_target = 0.0f;
    state_.is_firing = false;
    state_.flywheels_enabled = false;
    state_.gear_high = false;
    state_.armed = false;
    state_.last_update_ms = 0;
}

void ShooterControl::init() {
    gpio_init(esc1_pin_);
    gpio_set_dir(esc1_pin_, GPIO_OUT);
    gpio_init(esc2_pin_);
    gpio_set_dir(esc2_pin_, GPIO_OUT);
    gpio_init(gear_pin_);
    gpio_set_dir(gear_pin_, GPIO_OUT);
    gpio_init(pan_pin_);
    gpio_set_dir(pan_pin_, GPIO_OUT);
    gpio_init(tilt_pin_);
    gpio_set_dir(tilt_pin_, GPIO_OUT);
    update();
}

void ShooterControl::set_pan(float pan_rad) {
    state_.pan_target = pan_rad;
    // For servo, set PWM
    // Placeholder: gpio_put(pan_pin_, pan_rad > 0 ? 1 : 0);
}

void ShooterControl::set_tilt(float tilt_rad) {
    state_.tilt_target = tilt_rad;
    // Placeholder
}

void ShooterControl::fire() {
    state_.is_firing = true;
    // Trigger fire
    // Placeholder
}

void ShooterControl::update() {
    // Update positions
    state_.pan_position = state_.pan_target;
    state_.tilt_position = state_.tilt_target;
    // Update GPIOs
    gpio_put(esc1_pin_, state_.flywheels_enabled ? 1 : 0);
    gpio_put(esc2_pin_, state_.flywheels_enabled ? 1 : 0);
    gpio_put(gear_pin_, state_.gear_high ? 1 : 0);
    gpio_put(pan_pin_, state_.pan_position > 0 ? 1 : 0);
    gpio_put(tilt_pin_, state_.tilt_position > 0 ? 1 : 0);
}

void ShooterControl::enable_flywheels(bool enable) { state_.flywheels_enabled = enable; }

void ShooterControl::set_gear_state(bool high) { state_.gear_high = high; }

void ShooterControl::arm() {
    state_.armed = true;
    enable_flywheels(true);
    set_gear_state(true); // Assume high gear for armed
}

void ShooterControl::disarm() {
    state_.armed = false;
    enable_flywheels(false);
    set_gear_state(false); // Low gear for disarmed
}
