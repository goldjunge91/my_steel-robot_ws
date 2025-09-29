#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <cstdbool>
#include <cstdint>

// T021: MotorControllerState data structure
typedef struct {
    float pwm_duty[4];     // PWM duty cycle for each motor
    bool dir_pin_state[4]; // Direction pin state for each motor
    bool enabled;          // True if the motor controller is enabled
    uint32_t error_flags;  // Error flags for the motor controller
} MotorControllerState;

// T022: EncoderState data structure
typedef struct {
    int32_t ticks;              // Encoder tick count
    float velocity;             // Wheel velocity in rad/s
    uint32_t last_timestamp_ms; // Timestamp of the last encoder update
} EncoderState;

#endif // MOTOR_TYPES_H
