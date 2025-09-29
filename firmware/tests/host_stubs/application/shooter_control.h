// Minimal stub for application/shooter_control.h
#pragma once

class ShooterControl {
public:
    void enable_flywheels(bool enable);
    void set_gear_state(bool state);
    void set_pan(float pan);
    void set_tilt(float tilt);
    void fire();
};