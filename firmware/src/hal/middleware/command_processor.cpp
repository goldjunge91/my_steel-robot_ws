#include "middleware/command_processor.h"
#include "middleware/control_loop.h"
#include "middleware/usb_interface.h"
#include "pico/critical_section.h"
#include "pico/time.h"
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <string.h>

extern critical_section_t data_lock;
extern uint64_t last_cmd_time;

// External buffer references
extern double SetpointBuffer[4];
extern bool SetpointAvailable;

void uRosCmdVelCallback(const void *arg_input_message) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)arg_input_message;
    // Convert Twist to wheel velocities in rad/s * 1000
    // Assuming wheel radius 0.05m, track width 0.2m or something
    // For simplicity, linear.x in m/s to rad/s, then *1000
    float wheel_radius = 0.05f;                                            // placeholder
    float track_width = 0.2f;                                              // placeholder
    float linear_vel = msg->linear.x / wheel_radius;                       // rad/s
    float angular_vel = msg->angular.z * track_width / (2 * wheel_radius); // rad/s
    // For mecanum, FL = linear + angular, FR = linear - angular, etc.
    critical_section_enter_blocking(&data_lock);
    SetpointBuffer[0] = (linear_vel + angular_vel) * 1000; // FL
    SetpointBuffer[1] = (linear_vel - angular_vel) * 1000; // FR
    SetpointBuffer[2] = (linear_vel + angular_vel) * 1000; // RL
    SetpointBuffer[3] = (linear_vel - angular_vel) * 1000; // RR
    SetpointAvailable = true;
    last_cmd_time = time_us_32();
    critical_section_exit(&data_lock);
}

void command_processor_task(void) {
    char command[128];
    if (usb_interface_get_command(command, sizeof(command))) {
        if (strncmp(command, "M", 1) == 0) {
            int fl, fr, rl, rr;
            sscanf(command, "M FL:%d FR:%d RL:%d RR:%d", &fl, &fr, &rl, &rr);
            control_loop_set_motor_speeds_int(fl, fr, rl, rr);
            usb_interface_send_response("OK");
        } else if (strcmp(command, "STOP") == 0) {
            control_loop_set_motor_speeds_int(0, 0, 0, 0);
            usb_interface_send_response("STOPPED");
        } else {
            usb_interface_send_response("ERROR: INVALID_CMD");
        }
    }
}