#ifndef LAUNCHER_STATE_H
#define LAUNCHER_STATE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Data structure representing the current state of the launcher system
 */
typedef struct {
    float pan_position;         /**< Current pan position in radians */
    float tilt_position;        /**< Current tilt position in radians */
    float pan_target;           /**< Target pan position in radians */
    float tilt_target;          /**< Target tilt position in radians */
    bool is_firing;             /**< True if the launcher is currently firing */
    uint32_t last_update_ms;    /**< Timestamp of the last update in milliseconds */
} LauncherState;

/**
 * @brief Initialize launcher state to default values
 * @param state Pointer to LauncherState structure to initialize
 */
void launcher_state_init(LauncherState *state);

/**
 * @brief Update launcher state with new target positions
 * @param state Pointer to LauncherState structure
 * @param pan_target New target pan position in radians
 * @param tilt_target New target tilt position in radians
 */
void launcher_state_set_target(LauncherState *state, float pan_target, float tilt_target);

/**
 * @brief Update launcher firing state
 * @param state Pointer to LauncherState structure
 * @param is_firing New firing state
 */
void launcher_state_set_firing(LauncherState *state, bool is_firing);

/**
 * @brief Update current positions (called by control loop)
 * @param state Pointer to LauncherState structure
 * @param pan_position Current pan position in radians
 * @param tilt_position Current tilt position in radians
 */
void launcher_state_update_position(LauncherState *state, float pan_position, float tilt_position);

#endif // LAUNCHER_STATE_H