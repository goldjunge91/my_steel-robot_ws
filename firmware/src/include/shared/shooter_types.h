#ifndef SHOOTER_TYPES_H
#define SHOOTER_TYPES_H

#include <cstdbool>
#include <cstdint>

// T020: LauncherState data structure
typedef struct {
    float pan_position;      // Current pan position in radians
    float tilt_position;     // Current tilt position in radians
    float pan_target;        // Target pan position in radians
    float tilt_target;       // Target tilt position in radians
    bool is_firing;          // True if the launcher is currently firing
    bool flywheels_enabled;  // True if flywheels are enabled
    bool gear_high;          // True if gear is in high position
    bool armed;              // True if the shooter is armed
    uint32_t last_update_ms; // Timestamp of the last update
} LauncherState;

#endif // SHOOTER_TYPES_H