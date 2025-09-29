// host_globals.cpp: provide minimal definitions of firmware-global symbols
// used by host tests that include firmware implementation files directly.

#include "ImuLib_cfg.h"
#include "hardware_cfg.h"
#include "pico/critical_section.h"
#include <stdbool.h>
#include <stdint.h>

// critical_section_t comes from pico/critical_section.h (stubbed for host builds)
critical_section_t data_lock;

uint64_t last_cmd_time = 0;

double SetpointBuffer[4] = {0.0, 0.0, 0.0, 0.0};
bool SetpointAvailable = false;

// Minimal placeholders for other globals referenced by micro_ros_cfg.cpp
float battery_voltage = 0.0f;

// Buffers and availability flags used by micro_ros_cfg.cpp (simple placeholders)
bool MotorStateAvailable = false;
bool ImuAvailable = false;
bool BatteryStateAvailable = false;
bool OdomAvailable = false;
bool TofSensorAvailable = false;

// Firmware mode
FirmwareModeTypeDef firmware_mode = fw_normal;

// Define simple structs referenced by micro_ros_cfg.cpp if not present (lightweight)
struct motor_state_queue_t {
    uint8_t size = 4;
    double velocity[4];
    double position[4];
};

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

motor_state_queue_t MotorStateBuffer;
imu_queue_t ImuBuffer;
battery_state_queue_t BatteryStateBuffer;
odom_queue_t OdomBuffer;
range_queue_t TofSensorBuffer;

// Stub for ShooterControl
class ShooterControl {
  public:
    void enable_flywheels(bool enable) {}
    void set_gear_state(bool state) {}
    void set_pan(float pan) {}
    void set_tilt(float tilt) {}
    void fire() {}
};

ShooterControl shooter;

// Provide noop implementations for functions that may be called
extern "C" void BoardPheripheralsInit(void) {}
