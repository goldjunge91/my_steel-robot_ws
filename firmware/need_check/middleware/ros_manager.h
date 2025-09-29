#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROS_MANAGER_WHEEL_COUNT 4
#define ROS_MANAGER_SERVO_COUNT 2

typedef struct
{
    size_t size;
    double position[ROS_MANAGER_WHEEL_COUNT];
    double velocity[ROS_MANAGER_WHEEL_COUNT];
} ros_motor_state_t;

typedef struct
{
    float voltage;
    float temperature;
    float current;
    float charge;
    float capacity;
    float design_capacity;
    float percentage;
    uint8_t status;
    uint8_t health;
    uint8_t technology;
    bool present;
} ros_battery_state_t;

bool ros_manager_init(void);
void ros_manager_spin_some(void);

QueueHandle_t ros_manager_get_motor_setpoint_queue(void);
QueueHandle_t ros_manager_get_motor_state_queue(void);
QueueHandle_t ros_manager_get_imu_queue(void);
QueueHandle_t ros_manager_get_battery_queue(void);

#ifdef __cplusplus
}
#endif

#endif // ROS_MANAGER_H
