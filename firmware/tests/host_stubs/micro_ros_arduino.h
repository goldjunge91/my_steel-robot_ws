// Minimal stub for host builds: micro_ros_arduino.h
// This file is only used when building tests on the host. It provides
// the minimal declarations required by test code that depends on
// micro_ros_arduino.h. Expand as needed.

#pragma once

// Provide a minimal rcl types header compatibility layer expected by includes
#include <stdint.h>

// Placeholder macros/types used by firmware headers
#define PICO_ERROR_NONE 0

typedef int rcl_ret_t;

static inline rcl_ret_t rmw_uros_ping_agent(int timeout_ms, int attempts) { return 0; }

static inline bool rmw_uros_epoch_synchronized() { return true; }

static inline int64_t rmw_uros_epoch_millis() { return 0; }

static inline int64_t rmw_uros_epoch_nanos() { return 0; }

static inline rcl_ret_t rmw_uros_sync_session(int timeout_ms) { return 0; }

static inline rcl_ret_t rmw_uros_set_context_entity_destroy_session_timeout(void *rmw_context,
                                                                            int timeout) {
    return 0;
}

// Basic ROS types
typedef struct rcl_allocator_t {
    int dummy;
} rcl_allocator_t;
typedef struct rcl_init_options_t {
    int dummy;
} rcl_init_options_t;
typedef struct rclc_support_t {
    int dummy;
} rclc_support_t;
typedef struct rcl_node_t {
    int dummy;
} rcl_node_t;
typedef struct rcl_timer_t {
    int dummy;
} rcl_timer_t;
typedef struct rcl_publisher_t {
    int dummy;
} rcl_publisher_t;
typedef struct rcl_subscription_t {
    int dummy;
} rcl_subscription_t;
typedef struct rcl_service_t {
    int dummy;
} rcl_service_t;
typedef struct rclc_executor_t {
    int dummy;
} rclc_executor_t;
typedef struct rcl_context_t {
    int dummy;
} rcl_context_t;

// Function stubs
static inline rcl_allocator_t rcutils_get_default_allocator() { return {}; }
static inline rcl_init_options_t rcl_get_zero_initialized_init_options() { return {}; }
static inline rcl_ret_t rcl_init_options_init(rcl_init_options_t *options,
                                              rcl_allocator_t allocator) {
    return 0;
}
static inline rcl_ret_t rcl_init_options_set_domain_id(rcl_init_options_t *options,
                                                       size_t domain_id) {
    return 0;
}
static inline rcl_ret_t rclc_support_init_with_options(rclc_support_t *support, int argc,
                                                       char **argv, rcl_init_options_t *options,
                                                       rcl_allocator_t *allocator) {
    return 0;
}
static inline rcl_ret_t rclc_node_init_default(rcl_node_t *node, const char *name,
                                               const char *namespace_, rclc_support_t *support) {
    return 0;
}
static inline rcl_ret_t rclc_timer_init_default(rcl_timer_t *timer, rclc_support_t *support,
                                                int64_t period,
                                                void (*callback)(rcl_timer_t *, int64_t)) {
    return 0;
}
static inline rcl_ret_t rclc_subscription_init_default(rcl_subscription_t *subscription,
                                                       rcl_node_t *node, void *type_support,
                                                       const char *topic_name) {
    return 0;
}
static inline rcl_ret_t rclc_publisher_init_default(rcl_publisher_t *publisher, rcl_node_t *node,
                                                    void *type_support, const char *topic_name) {
    return 0;
}
static inline rcl_ret_t rclc_service_init_default(rcl_service_t *service, rcl_node_t *node,
                                                  void *type_support, const char *service_name) {
    return 0;
}
static inline rcl_ret_t rclc_executor_init(rclc_executor_t *executor, rclc_support_t *support,
                                           size_t number_of_handles, rcl_allocator_t *allocator) {
    return 0;
}
static inline rcl_ret_t rclc_executor_add_timer(rclc_executor_t *executor, rcl_timer_t *timer) {
    return 0;
}
static inline rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *executor,
                                                       rcl_subscription_t *subscription) {
    return 0;
}
static inline rcl_ret_t rclc_executor_add_service(rclc_executor_t *executor,
                                                  rcl_service_t *service) {
    return 0;
}
static inline rcl_ret_t rcl_publish(rcl_publisher_t *publisher, void *ros_message,
                                    void *allocation) {
    return 0;
}
static inline rcl_ret_t rcl_subscription_fini(rcl_subscription_t *subscription, rcl_node_t *node) {
    return 0;
}
static inline rcl_ret_t rcl_publisher_fini(rcl_publisher_t *publisher, rcl_node_t *node) {
    return 0;
}
static inline rcl_ret_t rcl_service_fini(rcl_service_t *service, rcl_node_t *node) { return 0; }
static inline rcl_ret_t rcl_timer_fini(rcl_timer_t *timer) { return 0; }
static inline rcl_ret_t rclc_executor_fini(rclc_executor_t *executor) { return 0; }
static inline rcl_ret_t rcl_node_fini(rcl_node_t *node) { return 0; }
static inline rcl_ret_t rclc_support_fini(rclc_support_t *support) { return 0; }
static inline rcl_ret_t rcl_init_options_fini(rcl_init_options_t *init_options) { return 0; }
static inline void *rcl_context_get_rmw_context(rcl_context_t *context) { return nullptr; }

// Message init stubs
static inline void nav_msgs__msg__Odometry__init(void *msg) {}
static inline void sensor_msgs__msg__Range__init(void *msg) {}

// Type support stubs
static inline void *
rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist() {
    return nullptr;
}
static inline void *
rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__JointState() {
    return nullptr;
}
static inline void *rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Bool() {
    return nullptr;
}
static inline void *rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Imu() {
    return nullptr;
}
static inline void *
rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Odometry() {
    return nullptr;
}
static inline void *
rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__Range() {
    return nullptr;
}
static inline void *
rosidl_typesupport_c__get_message_type_support_handle__sensor_msgs__msg__BatteryState() {
    return nullptr;
}
static inline void *
rosidl_typesupport_c__get_service_type_support_handle__std_srvs__srv__Trigger() {
    return nullptr;
}

// Service message init
static inline void std_srvs__srv__Trigger_Request__init(void *msg) {}
static inline void std_srvs__srv__Trigger_Response__init(void *msg) {}

// Error handling
static inline void ErrorLoop(const char *func) {}

// Callback stubs
static inline void uRosCmdVelCallback(const void *msg) {}

// Provide any other small compatibility helpers here as tests require.
