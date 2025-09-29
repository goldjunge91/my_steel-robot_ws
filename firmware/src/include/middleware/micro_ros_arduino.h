// Minimal stub for micro_ros_arduino.h to allow building on Pico without full Arduino
// This stub intentionally keeps content minimal; integrate proper micro-ROS Arduino client if
// runtime features are required.
#ifndef MICRO_ROS_ARDUINO_STUB_H
#define MICRO_ROS_ARDUINO_STUB_H

// Provide a macro used by some codepaths; real implementation is provided by micro-ROS Arduino
#define RMW_UXRCE_TRANSPORT_CUSTOM 0

// No-op init function placeholder
inline void micro_ros_init_transport() {}

// Minimal rmw_uros stubs used by microros plumbing in the example. These return default values
// so the code compiles; proper implementations come from micro-ROS middleware.
inline int rmw_uros_ping_agent(int /*timeout*/, int /*attempts*/) { return 0; }
inline bool rmw_uros_epoch_synchronized() { return false; }
inline unsigned long rmw_uros_epoch_millis() { return 0; }
inline unsigned long rmw_uros_epoch_nanos() { return 0; }
inline int rmw_uros_sync_session(int /*timeout*/) { return 0; }
inline void rmw_uros_set_context_entity_destroy_session_timeout(void * /*ctx*/, int /*t*/) {}

// NVIC_SystemReset placeholder (ARM-specific reset). On Pico, you might want to call
// watchdog_reboot or similar.
inline void NVIC_SystemReset() { /* no-op in host build */ }

#endif // MICRO_ROS_ARDUINO_STUB_H
