// Minimal stub for ImuLib_cfg.h to satisfy host builds of tests that include
// firmware headers. Extend as necessary for test code.
#pragma once

#include <stdint.h>

// Example typedefs and placeholders expected by firmware headers
typedef struct ImuData {
    float accel[3];
    float gyro[3];
    float mag[3];
} ImuData;

typedef struct imu_queue_t {
    float Orientation[4];
    float AngularVelocity[3];
    float LinearAcceleration[3];
} imu_queue_t;

static inline void ImuLib_init(void) {}
static inline int ImuLib_available(void) { return 0; }
