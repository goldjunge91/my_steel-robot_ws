#ifndef SHOOTER_CONTROL_H
#define SHOOTER_CONTROL_H

#include "shared/shooter_types.h"

class ShooterControl {
  public:
    ShooterControl(uint8_t esc1_pin, uint8_t esc2_pin, uint8_t gear_pin, uint8_t pan_pin,
                   uint8_t tilt_pin);
    void init();
    void set_pan(float pan_rad);
    void set_tilt(float tilt_rad);
    void fire();
    void update();
    void enable_flywheels(bool enable);
    void set_gear_state(bool high);
    void arm();
    void disarm();

  private:
    uint8_t esc1_pin_;
    uint8_t esc2_pin_;
    uint8_t gear_pin_;
    uint8_t pan_pin_;
    uint8_t tilt_pin_;
    LauncherState state_;
};

#endif // SHOOTER_CONTROL_H