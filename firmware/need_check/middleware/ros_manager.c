#include "ros_manager.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_srvs/srv/trigger.h>

#include "pico/stdlib.h"

#include "motor_driver.h"
#include "pico_uart_transports.h"
#include "servo_driver.h"
#include "pico/unique_id.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STATUS_LED_PIN 25

#define FRONT_LEFT_MOTOR_NAME "fl_wheel_joint"
#define FRONT_RIGHT_MOTOR_NAME "fr_wheel_joint"
#define REAR_LEFT_MOTOR_NAME "rl_wheel_joint"
#define REAR_RIGHT_MOTOR_NAME "rr_wheel_joint"

#define MOT_RESP_MSG_LEN 4
#define MOT_CMD_MSG_LEN 4
#define SERVO_CMD_MSG_LEN 2

static rcl_publisher_t imu_publisher;
static rcl_publisher_t motor_state_publisher;
static rcl_publisher_t battery_publisher;
static rcl_subscription_t motors_cmd_subscriber;
static rcl_subscription_t servo_cmd_subscriber;
static rcl_timer_t timer;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__JointState motor_state_msg;
static sensor_msgs__msg__BatteryState battery_msg;
static std_msgs__msg__Float32MultiArray motors_cmd_msg;
static std_msgs__msg__Float32MultiArray servo_cmd_msg;
static rcl_service_t cpu_id_service;
static std_srvs__srv__Trigger_Request cpu_id_request;
static std_srvs__srv__Trigger_Response cpu_id_response;

static double motor_positions[MOTOR_COUNT] = {0};
static double motor_velocities[MOTOR_COUNT] = {0};
static uint64_t last_timer_time_us = 0;

static const size_t message_index_map[MOT_RESP_MSG_LEN] = {
    [0] = 3, // rr
    [1] = 2, // rl
    [2] = 1, // fr
    [3] = 0  // fl
};

static void error_led_blink(void);
static void init_messages(void);
static void motors_cmd_callback(const void *msgin);
static void servo_cmd_callback(const void *msgin);
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
static void cpu_id_service_callback(const void *req, void *res);

static void error_led_blink(void)
{
    while (true)
    {
        gpio_put(STATUS_LED_PIN, 1);
        sleep_ms(200);
        gpio_put(STATUS_LED_PIN, 0);
        sleep_ms(200);
    }
}

static QueueHandle_t motor_setpoint_queue = NULL;
static QueueHandle_t motor_state_queue = NULL;
static QueueHandle_t imu_queue = NULL;
static QueueHandle_t battery_queue = NULL;

#define MOTOR_SETPOINT_QUEUE_LENGTH 1
#define MOTOR_STATE_QUEUE_LENGTH 1
#define IMU_QUEUE_LENGTH 1
#define BATTERY_QUEUE_LENGTH 1

bool ros_manager_init(void)
{
#if defined(USE_UART_TRANSPORT) && USE_UART_TRANSPORT == 1
    rmw_uros_set_custom_transport(true, NULL, pico_uart_transport_open, pico_uart_transport_close,
                                  pico_uart_transport_write, pico_uart_transport_read);
#else
    rmw_uros_set_custom_transport(true, NULL, pico_usb_transport_open, pico_usb_transport_close,
                                  pico_usb_transport_write, pico_usb_transport_read);
#endif

    allocator = rcl_get_default_allocator();

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_node_init_default(&node, "rosbot_pico_node", "", &support) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    memset(&imu_msg, 0, sizeof(imu_msg));
    memset(&motor_state_msg, 0, sizeof(motor_state_msg));
    memset(&battery_msg, 0, sizeof(battery_msg));
    memset(&motors_cmd_msg, 0, sizeof(motors_cmd_msg));
    memset(&servo_cmd_msg, 0, sizeof(servo_cmd_msg));

    init_messages();

    std_srvs__srv__Trigger_Request__init(&cpu_id_request);
    std_srvs__srv__Trigger_Response__init(&cpu_id_response);

    if (rclc_publisher_init_best_effort(&imu_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                        "imu/data_raw") != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_publisher_init_best_effort(&motor_state_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                        "motors_response") != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_publisher_init_best_effort(&battery_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
                                        "battery_state") != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_subscription_init_best_effort(&motors_cmd_subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                           "motors_cmd") != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_subscription_init_best_effort(&servo_cmd_subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                           "servo_cmd") != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    // Executor needs enough slots for timer + 2 subscriptions + service
    if (rclc_executor_init(&executor, &support.context, 4, &allocator) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }
    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }
    if (rclc_executor_add_subscription(&executor, &motors_cmd_subscriber, &motors_cmd_msg,
                                       &motors_cmd_callback, ON_NEW_DATA) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }
    if (rclc_executor_add_subscription(&executor, &servo_cmd_subscriber, &servo_cmd_msg,
                                       &servo_cmd_callback, ON_NEW_DATA) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_service_init_default(&cpu_id_service, &node,
                                  ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                  "get_cpu_id") != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rclc_executor_add_service(&executor, &cpu_id_service, &cpu_id_request, &cpu_id_response,
                                  cpu_id_service_callback) != RCL_RET_OK)
    {
        error_led_blink();
        return false;
    }

    if (rmw_uros_sync_session(1000) != RMW_RET_OK)
    {
        error_led_blink();
        return false;
    }

    gpio_put(STATUS_LED_PIN, 1);
    last_timer_time_us = time_us_64();

    if (motor_setpoint_queue == NULL)
    {
        motor_setpoint_queue = xQueueCreate(MOTOR_SETPOINT_QUEUE_LENGTH, sizeof(float) * MOTOR_COUNT);
    }
    if (motor_state_queue == NULL)
    {
        motor_state_queue = xQueueCreate(MOTOR_STATE_QUEUE_LENGTH, sizeof(ros_motor_state_t));
    }
    if (imu_queue == NULL)
    {
        imu_queue = xQueueCreate(IMU_QUEUE_LENGTH, sizeof(sensor_msgs__msg__Imu));
    }
    if (battery_queue == NULL)
    {
        battery_queue = xQueueCreate(BATTERY_QUEUE_LENGTH, sizeof(ros_battery_state_t));
    }

    return true;
}

void ros_manager_spin_some(void)
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

static void init_messages(void)
{
    static rosidl_runtime_c__String msg_name_tab[MOT_RESP_MSG_LEN];
    static double msg_data_tab[3][MOT_RESP_MSG_LEN];

    motor_state_msg.position.data = msg_data_tab[0];
    motor_state_msg.position.capacity = MOT_RESP_MSG_LEN;
    motor_state_msg.position.size = MOT_RESP_MSG_LEN;

    motor_state_msg.velocity.data = msg_data_tab[1];
    motor_state_msg.velocity.capacity = MOT_RESP_MSG_LEN;
    motor_state_msg.velocity.size = MOT_RESP_MSG_LEN;

    motor_state_msg.effort.data = msg_data_tab[2];
    motor_state_msg.effort.capacity = MOT_RESP_MSG_LEN;
    motor_state_msg.effort.size = MOT_RESP_MSG_LEN;

    static char frame_id[] = "motors_response";
    motor_state_msg.header.frame_id.data = frame_id;
    motor_state_msg.header.frame_id.size = strlen(frame_id);
    motor_state_msg.header.frame_id.capacity = strlen(frame_id) + 1;

    static const char *names[MOT_RESP_MSG_LEN] = {
        REAR_RIGHT_MOTOR_NAME,
        REAR_LEFT_MOTOR_NAME,
        FRONT_RIGHT_MOTOR_NAME,
        FRONT_LEFT_MOTOR_NAME};

    for (size_t i = 0; i < MOT_RESP_MSG_LEN; i++)
    {
        msg_name_tab[i].data = (char *)names[i];
        msg_name_tab[i].size = strlen(names[i]);
        msg_name_tab[i].capacity = msg_name_tab[i].size + 1;
    }

    motor_state_msg.name.data = msg_name_tab;
    motor_state_msg.name.size = MOT_RESP_MSG_LEN;
    motor_state_msg.name.capacity = MOT_RESP_MSG_LEN;

    static float motor_cmd_data[MOT_CMD_MSG_LEN] = {0};
    motors_cmd_msg.data.data = motor_cmd_data;
    motors_cmd_msg.data.capacity = MOT_CMD_MSG_LEN;
    motors_cmd_msg.data.size = 0;

    static float servo_cmd_data[SERVO_CMD_MSG_LEN] = {0};
    servo_cmd_msg.data.data = servo_cmd_data;
    servo_cmd_msg.data.capacity = SERVO_CMD_MSG_LEN;
    servo_cmd_msg.data.size = 0;

    static char imu_frame_id[] = "imu_link";
    imu_msg.header.frame_id.data = imu_frame_id;
    imu_msg.header.frame_id.size = strlen(imu_frame_id);
    imu_msg.header.frame_id.capacity = strlen(imu_frame_id) + 1;

    static char battery_frame_id[] = "battery_link";
    battery_msg.header.frame_id.data = battery_frame_id;
    battery_msg.header.frame_id.size = strlen(battery_frame_id);
    battery_msg.header.frame_id.capacity = strlen(battery_frame_id) + 1;
}

static void motors_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (motor_setpoint_queue != NULL)
    {
        float setpoints[MOTOR_COUNT] = {0};
        size_t count = msg->data.size > MOTOR_COUNT ? MOTOR_COUNT : msg->data.size;
        for (size_t i = 0; i < count; i++)
        {
            setpoints[i] = msg->data.data[i];
        }
        xQueueOverwrite(motor_setpoint_queue, setpoints);
    }
}

static void servo_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    servo_driver_set_targets(msg->data.data, msg->data.size);
}

static void timer_callback(rcl_timer_t *timer_handle, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer_handle == NULL)
    {
        return;
    }

    gpio_put(STATUS_LED_PIN, !gpio_get(STATUS_LED_PIN));

    uint64_t now = time_us_64();
    double dt = (double)(now - last_timer_time_us) / 1e6;
    if (dt <= 0.0)
    {
        dt = 0.1;
    }
    last_timer_time_us = now;

    motor_driver_update_measurements(dt);
    motor_driver_get_joint_state(motor_positions, motor_velocities);

    if (rmw_uros_epoch_synchronized())
    {
        motor_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        motor_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
        imu_msg.header.stamp = motor_state_msg.header.stamp;
        battery_msg.header.stamp = motor_state_msg.header.stamp;
    }
    else
    {
        motor_state_msg.header.stamp.sec = 0;
        motor_state_msg.header.stamp.nanosec = 0;
        imu_msg.header.stamp.sec = 0;
        imu_msg.header.stamp.nanosec = 0;
        battery_msg.header.stamp.sec = 0;
        battery_msg.header.stamp.nanosec = 0;
    }

    for (size_t i = 0; i < MOT_RESP_MSG_LEN; i++)
    {
        size_t motor_index = message_index_map[i];
        motor_state_msg.position.data[i] = motor_positions[motor_index];
        motor_state_msg.velocity.data[i] = motor_velocities[motor_index];
        motor_state_msg.effort.data[i] = 0.0;
    }

    if (motor_state_queue != NULL)
    {
        ros_motor_state_t state = {0};
        state.size = MOT_RESP_MSG_LEN;
        for (size_t i = 0; i < MOT_RESP_MSG_LEN; i++)
        {
            state.position[i] = motor_state_msg.position.data[i];
            state.velocity[i] = motor_state_msg.velocity.data[i];
        }
        xQueueOverwrite(motor_state_queue, &state);
    }

    {
        rcl_ret_t _rc = rcl_publish(&motor_state_publisher, &motor_state_msg, NULL);
        (void)_rc; // intentionally ignore publish result for now
    }
    {
        rcl_ret_t _rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
        (void)_rc;
    }

    if (imu_queue != NULL)
    {
        sensor_msgs__msg__Imu imu_copy = imu_msg;
        xQueueOverwrite(imu_queue, &imu_copy);
    }

    battery_msg.voltage = 12.0;
    battery_msg.percentage = 0.8;
    {
        rcl_ret_t _rc = rcl_publish(&battery_publisher, &battery_msg, NULL);
        (void)_rc;
    }

    if (battery_queue != NULL)
    {
        ros_battery_state_t battery = {
            .voltage = battery_msg.voltage,
            .temperature = battery_msg.temperature,
            .current = battery_msg.current,
            .charge = battery_msg.charge,
            .capacity = battery_msg.capacity,
            .design_capacity = battery_msg.design_capacity,
            .percentage = (float)battery_msg.percentage,
            .status = (uint8_t)battery_msg.power_supply_status,
            .health = (uint8_t)battery_msg.power_supply_health,
            .technology = (uint8_t)battery_msg.power_supply_technology,
            .present = battery_msg.present};
        xQueueOverwrite(battery_queue, &battery);
    }

    motor_driver_handle_timeout(now);
}

static void cpu_id_service_callback(const void *req, void *res)
{
    (void)req;

    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    static char buffer[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
    char *write_ptr = buffer;
    for (size_t i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i)
    {
        write_ptr += snprintf(write_ptr, 3, "%02X", board_id.id[i]);
    }

    std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
    response->success = true;
    response->message.data = buffer;
    response->message.size = strlen(buffer);
    response->message.capacity = strlen(buffer) + 1;
}

QueueHandle_t ros_manager_get_motor_setpoint_queue(void)
{
    return motor_setpoint_queue;
}

QueueHandle_t ros_manager_get_motor_state_queue(void)
{
    return motor_state_queue;
}

QueueHandle_t ros_manager_get_imu_queue(void)
{
    return imu_queue;
}

QueueHandle_t ros_manager_get_battery_queue(void)
{
    return battery_queue;
}
