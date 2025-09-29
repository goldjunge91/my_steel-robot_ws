// Stub micro_ros_cfg.h for host builds. Provides minimal definitions to satisfy test compilation.

#pragma once

#include <stdint.h>
#include <cstdlib>

// Basic types
typedef int rcl_ret_t;
typedef struct rcl_timer_t rcl_timer_t;
typedef enum { Default = 0, Ok = 1, Error = 2, InvalidInput = 3, Pending = 4 } uRosFunctionStatus;
typedef enum { NotCreated = 0, Created = 1, Destroyed = 3 } uRosEntitiesStatus;

// Firmware mode
typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;
extern FirmwareModeTypeDef firmware_mode;

// Constants
#define MOT_CMD_MSG_LEN 4
#define MOT_RESP_MSG_LEN 4
#define FRONT_LEFT_MOTOR_NAME "fl_wheel_joint"
#define FRONT_RIGHT_MOTOR_NAME "fr_wheel_joint"
#define REAR_LEFT_MOTOR_NAME "rl_wheel_joint"
#define REAR_RIGHT_MOTOR_NAME "rr_wheel_joint"

// Queue types
typedef struct {
    uint8_t size = 4;
    double velocity[4];
    double position[4];
} motor_state_queue_t;

typedef struct {
    float Orientation[4];
    float AngularVelocity[3];
    float LinearAcceleration[3];
} imu_queue_t;

typedef struct {
    // odom fields
    double pose_x;
    double pose_y;
    double pose_theta;
    double linear_vel;
    double angular_vel;
} odom_queue_t;

typedef struct {
    float range;
    uint8_t field_of_view;
} range_queue_t;

typedef struct {
    // ROS battery msgs variables
    float voltage;
    float temperature;
    float current;
    float charge_current;
    float capacity;
    float design_capacity;
    float percentage;
    float cell_temperature[1];
    float cell_voltage[1];
    int status;
    int health;
    int technology;
    bool present;
} battery_state_queue_t;

// Message types (minimal stubs)
typedef struct {
    struct {
        struct {
            char *data;
            size_t capacity;
            size_t size;
        } frame_id;
    } header;
    struct {
        char **data;
        size_t capacity;
        size_t size;
    } name;
    struct {
        double *data;
        size_t capacity;
        size_t size;
    } position;
    struct {
        double *data;
        size_t capacity;
        size_t size;
    } velocity;
    struct {
        double *data;
        size_t capacity;
        size_t size;
    } effort;
} sensor_msgs__msg__JointState;

typedef struct {
    struct {
        float *data;
        size_t capacity;
        size_t size;
    } data;
} std_msgs__msg__Float32MultiArray;

// Externs
extern double SetpointBuffer[4];
extern bool SetpointAvailable;
extern motor_state_queue_t MotorStateBuffer;
extern bool MotorStateAvailable;
extern imu_queue_t ImuBuffer;
extern bool ImuAvailable;
extern battery_state_queue_t BatteryStateBuffer;
extern bool BatteryStateAvailable;
extern odom_queue_t OdomBuffer;
extern bool OdomAvailable;
extern range_queue_t TofSensorBuffer;
extern bool TofSensorAvailable;

// Functions
void ErrorLoop(const char *func) {}
uRosFunctionStatus uRosPingAgent(void) { return Ok; }
uRosFunctionStatus uRosPingAgent(uint8_t arg_timeout, uint8_t arg_attempts) { return Ok; }
uRosFunctionStatus uRosLoopHandler(uRosFunctionStatus arg_agent_ping_status) { return Ok; }
void uRosMotorsCmdCallback(const void *arg_input_message) {}
void uRosTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {}
uRosEntitiesStatus uRosCreateEntities(void) { return Created; }
uRosEntitiesStatus uRosDestroyEntities(void) { return Destroyed; }
void MotorsResponseMsgInit(sensor_msgs__msg__JointState *arg_message) {
    static char frame_id[] = "motors_response";
    arg_message->header.frame_id.data = frame_id;
    arg_message->header.frame_id.capacity = sizeof(frame_id);
    arg_message->header.frame_id.size = sizeof(frame_id);
    arg_message->name.capacity = MOT_RESP_MSG_LEN;
    arg_message->name.size = MOT_RESP_MSG_LEN;
    static char *names[MOT_RESP_MSG_LEN] = {FRONT_LEFT_MOTOR_NAME, FRONT_RIGHT_MOTOR_NAME, REAR_LEFT_MOTOR_NAME, REAR_RIGHT_MOTOR_NAME};
    arg_message->name.data = names;
    arg_message->position.capacity = MOT_RESP_MSG_LEN;
    arg_message->position.size = MOT_RESP_MSG_LEN;
    arg_message->position.data = (double *)malloc(MOT_RESP_MSG_LEN * sizeof(double));
    arg_message->velocity.capacity = MOT_RESP_MSG_LEN;
    arg_message->velocity.size = MOT_RESP_MSG_LEN;
    arg_message->velocity.data = (double *)malloc(MOT_RESP_MSG_LEN * sizeof(double));
    arg_message->effort.capacity = MOT_RESP_MSG_LEN;
    arg_message->effort.size = MOT_RESP_MSG_LEN;
    arg_message->effort.data = (double *)malloc(MOT_RESP_MSG_LEN * sizeof(double));
}
void MotorsCmdMsgInit(std_msgs__msg__Float32MultiArray *arg_message) {
    arg_message->data.capacity = MOT_CMD_MSG_LEN;
    arg_message->data.size = MOT_CMD_MSG_LEN;
    arg_message->data.data = (float *)malloc(MOT_CMD_MSG_LEN * sizeof(float));
}
void uRosCmdVelCallback(const void *arg_input_message) {}
void uRosLauncherControlCallback(const void *arg_input_message) {}
void uRosLauncherFireCallback(const void *arg_input_message) {}

// Macros
#define RCCHECK(fn)                                                                                \
    {                                                                                              \
        rcl_ret_t temp_rc = fn;                                                                    \
        if (temp_rc != 0) {                                                                        \
        }                                                                                          \
    }
#define RCSOFTCHECK(fn)                                                                            \
    {                                                                                              \
        rcl_ret_t temp_rc = fn;                                                                    \
        if (temp_rc != 0) {                                                                        \
        }                                                                                          \
    }