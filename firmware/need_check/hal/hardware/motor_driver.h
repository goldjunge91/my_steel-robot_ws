#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#define MOTOR_COUNT 4

#ifdef __cplusplus
extern "C" {
#endif

void motor_driver_init(void);
void motor_driver_apply_commands(const float *commands, size_t command_count);
void motor_driver_stop_all(void);
void motor_driver_handle_timeout(uint64_t now_us);
void motor_driver_update_measurements(double dt);
void motor_driver_get_joint_state(double *positions_out, double *velocities_out);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H
