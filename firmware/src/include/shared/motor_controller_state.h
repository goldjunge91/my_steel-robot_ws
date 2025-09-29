#ifndef MOTOR_CONTROLLER_STATE_H
#define MOTOR_CONTROLLER_STATE_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_COUNT 4  /**< Number of motors in the system */

/**
 * @brief Data structure representing the current state of the motor controller
 */
typedef struct {
    float pwm_duty[MOTOR_COUNT];        /**< PWM duty cycle for each motor (-1.0 to 1.0) */
    bool dir_pin_state[MOTOR_COUNT];    /**< Direction pin state for each motor */
    bool enabled;                       /**< True if the motor controller is enabled */
    uint32_t error_flags;               /**< Error flags for the motor controller */
} MotorControllerState;

/**
 * @brief Error flags for motor controller
 */
typedef enum {
    MOTOR_ERROR_NONE = 0,
    MOTOR_ERROR_OVERCURRENT = (1 << 0),
    MOTOR_ERROR_OVERTEMP = (1 << 1),
    MOTOR_ERROR_STALL = (1 << 2),
    MOTOR_ERROR_ENCODER = (1 << 3),
    MOTOR_ERROR_COMMUNICATION = (1 << 4)
} MotorErrorFlags;

/**
 * @brief Initialize motor controller state to default values
 * @param state Pointer to MotorControllerState structure to initialize
 */
void motor_controller_state_init(MotorControllerState *state);

/**
 * @brief Set PWM duty cycle for a specific motor
 * @param state Pointer to MotorControllerState structure
 * @param motor_index Motor index (0-3)
 * @param duty PWM duty cycle (-1.0 to 1.0, negative values reverse direction)
 */
void motor_controller_set_duty(MotorControllerState *state, uint8_t motor_index, float duty);

/**
 * @brief Enable or disable the motor controller
 * @param state Pointer to MotorControllerState structure
 * @param enabled True to enable, false to disable
 */
void motor_controller_set_enabled(MotorControllerState *state, bool enabled);

/**
 * @brief Set error flag
 * @param state Pointer to MotorControllerState structure
 * @param error_flag Error flag to set
 */
void motor_controller_set_error(MotorControllerState *state, MotorErrorFlags error_flag);

/**
 * @brief Clear error flag
 * @param state Pointer to MotorControllerState structure
 * @param error_flag Error flag to clear
 */
void motor_controller_clear_error(MotorControllerState *state, MotorErrorFlags error_flag);

/**
 * @brief Clear all error flags
 * @param state Pointer to MotorControllerState structure
 */
void motor_controller_clear_all_errors(MotorControllerState *state);

/**
 * @brief Check if motor controller has any errors
 * @param state Pointer to MotorControllerState structure
 * @return True if any error flags are set
 */
bool motor_controller_has_errors(const MotorControllerState *state);

#endif // MOTOR_CONTROLLER_STATE_H