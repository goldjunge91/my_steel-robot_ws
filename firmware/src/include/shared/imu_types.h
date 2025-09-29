#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <cstdint>

typedef struct imu_queue_t {
    float Orientation[4];
    float AngularVelocity[3];
    float LinearAcceleration[3];
} imu_queue_t;

#endif // IMU_TYPES_H
