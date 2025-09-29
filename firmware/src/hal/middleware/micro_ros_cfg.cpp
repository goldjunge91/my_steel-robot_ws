/**
 * @file micro_ros_cfg.cpp
 * @author Maciej Kurcius, refactored by Gemini
 * @brief
 * @version 0.    // QOS best effort
    if (OdomAvailable) {
      odom_queue_t queue_odom = OdomBuffer;
      OdomAvailable = false;
      if (rmw_uros_epoch_synchronized()) {
        odom_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      odom_msg.header.frame_id.data = (char *)"odom";
      odom_msg.pose.pose.position.x = queue_odom.x;
      odom_msg.pose.pose.position.y = queue_odom.y;
      odom_msg.pose.pose.orientation.z = sin(queue_odom.theta / 2);
      odom_msg.pose.pose.orientation.w = cos(queue_odom.theta / 2);
      odom_msg.twist.twist.linear.x = queue_odom.vx;
      odom_msg.twist.twist.angular.z = queue_odom.vtheta;
      RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
    // QOS best effort
    if (TofSensorAvailable) {
      range_queue_t queue_tof = TofSensorBuffer;
      TofSensorAvailable = false;
      if (rmw_uros_epoch_synchronized()) {
        tof_sensor_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        tof_sensor_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      tof_sensor_msg.header.frame_id.data = (char *)"tof_link";
      tof_sensor_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
      tof_sensor_msg.field_of_view = 0.1;
      tof_sensor_msg.min_range = 0.02;
      tof_sensor_msg.max_range = 4.0;
      tof_sensor_msg.range = queue_tof.range;
      RCSOFTCHECK(rcl_publish(&tof_sensor_publisher, &tof_sensor_msg, NULL));
    }2025-09-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "middleware/micro_ros_cfg.h"
#include "application/shooter_control.h"
#include "hardware/watchdog.h"
#include "pico/critical_section.h"
#include "pico/unique_id.h"
#include <cmath>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <stdio.h>

extern critical_section_t data_lock;
extern float battery_voltage;

// ROS PUBLISHERS
rcl_publisher_t imu_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t tof_sensor_publisher;
rcl_publisher_t battery_state_publisher;
// ROS SUBSCRIPTIONS
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t launcher_control_subscriber;
rcl_subscription_t launcher_fire_subscriber;
// ROS MESSAGES
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Range tof_sensor_msg;
sensor_msgs__msg__BatteryState battery_state_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__JointState launcher_control_msg;
std_msgs__msg__Bool launcher_fire_msg;
// ROS SERVICES
rcl_service_t get_cpu_id_service;
// ROS REQUESTS AND RESPONSES
std_srvs__srv__Trigger_Request get_cpu_id_service_request;
std_srvs__srv__Trigger_Response get_cpu_id_service_response;
// ROS
rcl_init_options_t init_options;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
uRosFunctionStatus ping_agent_status;
// REST
extern FirmwareModeTypeDef firmware_mode;
extern ShooterControl shooter;

// External references to global buffer variables defined in main.cpp
extern double SetpointBuffer[4];
extern bool SetpointAvailable;
extern motor_state_queue_t MotorStateBuffer;
extern bool MotorStateAvailable;
extern imu_queue_t ImuBuffer;
extern bool ImuAvailable;
extern battery_state_queue_t BatteryStateBuffer;
extern bool BatteryStateAvailable;

void uRosShooterCmdCallback(const void *arg_input_message) {
    const std_msgs__msg__Int8MultiArray *msg =
        (const std_msgs__msg__Int8MultiArray *)arg_input_message;
    if (msg->data.size >= 2) {
        shooter.enable_flywheels(msg->data.data[0] != 0);
        shooter.set_gear_state(msg->data.data[1] != 0);
    }
}

void uRosLauncherControlCallback(const void *arg_input_message) {
    const sensor_msgs__msg__JointState *msg =
        (const sensor_msgs__msg__JointState *)arg_input_message;
    if (msg->name.size >= 2 && msg->position.size >= 2) {
        float pan = msg->position.data[0];
        float tilt = msg->position.data[1];
        shooter.set_pan(pan);
        shooter.set_tilt(tilt);
    }
}

void uRosLauncherFireCallback(const void *arg_input_message) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)arg_input_message;
    if (msg->data) {
        shooter.fire();
    }
}

void uRosTimerCallback(rcl_timer_t *arg_timer, int64_t arg_last_call_time) {
    RCLC_UNUSED(arg_last_call_time);
    static battery_state_queue_t battery_state_queue;
    static imu_queue_t queue_imu;
    if (arg_timer != NULL) {
        // QOS default
        critical_section_enter_blocking(&data_lock);
        if (BatteryStateAvailable) {
            battery_state_queue = BatteryStateBuffer;
            BatteryStateAvailable = false;
            if (rmw_uros_epoch_synchronized()) {
                battery_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
                battery_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
            }
            battery_state_msg.voltage = battery_state_queue.voltage;
            battery_state_msg.temperature = battery_state_queue.temperature;
            battery_state_msg.current = battery_state_queue.current;
            battery_state_msg.charge = battery_state_queue.charge_current;
            battery_state_msg.capacity = battery_state_queue.capacity;
            battery_state_msg.design_capacity = battery_state_queue.design_capacity;
            battery_state_msg.percentage = battery_state_queue.percentage;
            battery_state_msg.power_supply_status = battery_state_queue.status;
            battery_state_msg.power_supply_health = battery_state_queue.health;
            battery_state_msg.power_supply_technology = battery_state_queue.technology;
            battery_state_msg.present = battery_state_queue.present;
            battery_state_msg.cell_temperature.capacity =
                BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
            battery_state_msg.cell_temperature.size = BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
            battery_state_msg.cell_temperature.data = battery_state_queue.cell_temperature;
            battery_state_msg.cell_voltage.capacity = BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
            battery_state_msg.cell_voltage.size = BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
            battery_state_msg.cell_voltage.data = battery_state_queue.cell_voltage;
            RCSOFTCHECK(rcl_publish(&battery_state_publisher, &battery_state_msg, NULL));
            battery_voltage = battery_state_queue.voltage;
        }
        critical_section_exit(&data_lock);
        // Publish odom
        critical_section_enter_blocking(&data_lock);
        if (OdomAvailable) {
            // Fill odom_msg from OdomBuffer
            if (rmw_uros_epoch_synchronized()) {
                odom_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
                odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
            }
            odom_msg.header.frame_id.data = (char *)"odom";
            odom_msg.pose.pose.position.x = OdomBuffer.pose_x;
            odom_msg.pose.pose.position.y = OdomBuffer.pose_y;
            odom_msg.pose.pose.orientation.z = sin(OdomBuffer.pose_theta / 2);
            odom_msg.pose.pose.orientation.w = cos(OdomBuffer.pose_theta / 2);
            odom_msg.twist.twist.linear.x = OdomBuffer.linear_vel;
            odom_msg.twist.twist.angular.z = OdomBuffer.angular_vel;
            RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
            OdomAvailable = false;
        }
        critical_section_exit(&data_lock);
        // Publish tof_sensor
        critical_section_enter_blocking(&data_lock);
        if (TofSensorAvailable) {
            if (rmw_uros_epoch_synchronized()) {
                tof_sensor_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
                tof_sensor_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
            }
            tof_sensor_msg.header.frame_id.data = (char *)"tof_link";
            tof_sensor_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND; // or LASER
            tof_sensor_msg.field_of_view = TofSensorBuffer.field_of_view;
            tof_sensor_msg.min_range = 0.02;
            tof_sensor_msg.max_range = 4.0;
            tof_sensor_msg.range = TofSensorBuffer.range;
            RCSOFTCHECK(rcl_publish(&tof_sensor_publisher, &tof_sensor_msg, NULL));
            TofSensorAvailable = false;
        }
        critical_section_exit(&data_lock);
    }
}

void uRosGetIdCallback(const void *req, void *res) {
    (void)req; // Unused parameter

    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);

    char cpu_id_buffer[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1] = {0};
    char *hex_ptr = cpu_id_buffer;
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i) {
        sprintf(hex_ptr, "%02x", board_id.id[i]);
        hex_ptr += 2;
    }

    static char out_buffer[100];
    snprintf(out_buffer, sizeof(out_buffer), "{\"cpu_id\": \"%s\"}", cpu_id_buffer);

    std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
    response->success = true;
    response->message.data = out_buffer;
    response->message.size = strlen(out_buffer);
}

uRosEntitiesStatus uRosCreateEntities(void) {
    uint8_t ros_msgs_cnt = 0;
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(
        rcl_init_options_set_domain_id(&init_options, UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    if (firmware_mode == fw_debug) {
        printf("Created support with option domain_id=%d\n",
               UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV);
    }

    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    if (firmware_mode == fw_debug) {
        printf("Created node `%s`\n", NODE_NAME);
    }

    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), uRosTimerCallback));
    ros_msgs_cnt++;
    if (firmware_mode == fw_debug) {
        printf("Created timer\n");
    }

    RCCHECK(rclc_subscription_init_default(&cmd_vel_subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           "cmd_vel"));
    ros_msgs_cnt++;
    if (firmware_mode == fw_debug) {
        printf("Created 'cmd_vel' subscriber\n");
    }

    RCCHECK(rclc_subscription_init_default(
        &launcher_control_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "launcher_control"));
    ros_msgs_cnt++;
    if (firmware_mode == fw_debug) {
        printf("Created 'launcher_control' subscriber\n");
    }

    RCCHECK(rclc_subscription_init_default(&launcher_fire_subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                           "launcher/fire"));
    ros_msgs_cnt++;
    if (firmware_mode == fw_debug) {
        printf("Created 'launcher/fire' subscriber\n");
    }

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
    if (firmware_mode == fw_debug) {
        printf("Created 'imu/data_raw' publisher.\n");
    }

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    if (firmware_mode == fw_debug) {
        printf("Created 'odom' publisher.\n");
    }

    RCCHECK(rclc_publisher_init_default(&tof_sensor_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                        "tof_sensor"));
    if (firmware_mode == fw_debug) {
        printf("Created 'tof_sensor' publisher.\n");
    }

    RCCHECK(rclc_publisher_init_default(&battery_state_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
                                        "battery_state"));
    if (firmware_mode == fw_debug) {
        printf("Created 'battery_state' publisher.\n");
    }

    std_srvs__srv__Trigger_Request__init(&get_cpu_id_service_request);
    std_srvs__srv__Trigger_Response__init(&get_cpu_id_service_response);
    RCCHECK(rclc_service_init_default(&get_cpu_id_service, &node,
                                      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                      "get_cpu_id"));
    ros_msgs_cnt++;
    if (firmware_mode == fw_debug) {
        printf("Created 'get_cpu_id_service' service.\n");
    }

    RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
                                           &uRosCmdVelCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &launcher_control_subscriber,
                                           &launcher_control_msg, &uRosLauncherControlCallback,
                                           ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &launcher_fire_subscriber, &launcher_fire_msg,
                                           &uRosLauncherFireCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_service(&executor, &get_cpu_id_service, &get_cpu_id_service_request,
                                      &get_cpu_id_service_response, uRosGetIdCallback));
    if (firmware_mode == fw_debug) {
        printf("Executor started\n");
    }

    RCCHECK(rmw_uros_sync_session(1000));
    if (firmware_mode == fw_debug) {
        printf("Clocks synchronised\n");
    }

    nav_msgs__msg__Odometry__init(&odom_msg);
    sensor_msgs__msg__Range__init(&tof_sensor_msg);

    return Created;
}

uRosEntitiesStatus uRosDestroyEntities(void) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_publisher_fini(&tof_sensor_publisher, &node));
    RCCHECK(rcl_publisher_fini(&battery_state_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&launcher_control_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&launcher_fire_subscriber, &node));
    RCCHECK(rcl_service_fini(&get_cpu_id_service, &node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    RCCHECK(rcl_init_options_fini(&init_options));
    if (firmware_mode == fw_debug) {
        printf("Destroyed all microros entities.\n");
    }
    return Destroyed;
}

void MotorsResponseMsgInit(sensor_msgs__msg__JointState *arg_message) {
    static rosidl_runtime_c__String msg_name_tab[MOT_RESP_MSG_LEN];
    static double msg_data_tab[3][MOT_RESP_MSG_LEN];
    char *frame_id = (char *)"motors_response";
    arg_message->position.data = msg_data_tab[0];
    arg_message->position.capacity = arg_message->position.size = MOT_RESP_MSG_LEN;
    arg_message->velocity.data = msg_data_tab[1];
    arg_message->velocity.capacity = arg_message->velocity.size = MOT_RESP_MSG_LEN;
    arg_message->effort.data = msg_data_tab[2];
    arg_message->effort.capacity = arg_message->effort.size = MOT_RESP_MSG_LEN;
    arg_message->header.frame_id.data = frame_id;
    arg_message->header.frame_id.capacity = arg_message->header.frame_id.size =
        strlen((const char *)frame_id);
    msg_name_tab->capacity = msg_name_tab->size = MOT_RESP_MSG_LEN;
    msg_name_tab[0].data = (char *)REAR_RIGHT_MOTOR_NAME;
    msg_name_tab[1].data = (char *)REAR_LEFT_MOTOR_NAME;
    msg_name_tab[2].data = (char *)FRONT_RIGHT_MOTOR_NAME;
    msg_name_tab[3].data = (char *)FRONT_LEFT_MOTOR_NAME;
    for (uint8_t i = 0; i < MOT_RESP_MSG_LEN; i++) {
        msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
    }
    arg_message->name.capacity = arg_message->name.size = MOT_RESP_MSG_LEN;
    arg_message->name.data = msg_name_tab;
}

void MotorsCmdMsgInit(std_msgs__msg__Float32MultiArray *arg_message) {
    static float data[MOT_CMD_MSG_LEN] = {0, 0, 0, 0};
    arg_message->data.capacity = MOT_CMD_MSG_LEN;
    arg_message->data.size = MOT_CMD_MSG_LEN;
    arg_message->data.data = (float *)data;
}
