#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stddef.h>

#define SERVO_COUNT 2

#ifdef __cplusplus
extern "C" {
#endif

void servo_driver_init(void);
void servo_driver_set_targets(const float *targets, size_t target_count);

#ifdef __cplusplus
}
#endif

#endif // SERVO_DRIVER_H
