#include "middleware/control_loop.h"
#include "hardware/config.h"
#include "hardware/motors.h"
#include "middleware/micro_ros_cfg.h"
#include "pico/critical_section.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "application/watchdog_manager.h"
#include "shared/hardware_cfg.h"
#include <cmath>

extern critical_section_t data_lock;
extern uint64_t last_cmd_time;
extern float battery_voltage;

// External buffer references
extern double SetpointBuffer[4];
extern bool SetpointAvailable;
extern odom_queue_t OdomBuffer;
extern bool OdomAvailable;
extern range_queue_t TofSensorBuffer;
extern bool TofSensorAvailable;

// Constants
const float WHEEL_RADIUS = 0.05f;
const float TRACK_WIDTH = 0.2f;

// Global motor objects
TimebaseTimerClass timebase_timer;
MotorClass motor_fl;
MotorClass motor_fr;
MotorClass motor_rl;
MotorClass motor_rr;

extern "C" {

void control_loop_init(void)
{
  // The MotorClass constructor handles all hardware initialization.
  // Note: Default directions are set based on the original STM32 code's conventions.
  motor_fl = MotorClass(FL_PWM, FL_IN1, FL_IN2, ENC_FL_A, -1, &timebase_timer, pio0, 0);
  motor_fr = MotorClass(FR_PWM, FR_IN1, FR_IN2, ENC_FR_A, 1, &timebase_timer, pio0, 1);
  motor_rl = MotorClass(RL_PWM, RL_IN1, RL_IN2, ENC_RL_A, -1, &timebase_timer, pio0, 2);
  motor_rr = MotorClass(RR_PWM, RR_IN1, RR_IN2, ENC_RR_A, 1, &timebase_timer, pio0, 3);
}

void control_loop_update(void)
{
  // Check for new setpoints
  critical_section_enter_blocking(&data_lock);
  if (SetpointAvailable) {
    control_loop_set_motor_setpoints(SetpointBuffer);
    SetpointAvailable = false;
  } else {
    // Safety stop if no command for 200ms or battery critical
    uint64_t now = time_us_32();
    if (now - last_cmd_time > 200000 || battery_voltage < 10.5f) {
      double zero_setpoints[4] = {0, 0, 0, 0};
      control_loop_set_motor_setpoints(zero_setpoints);
    }
  }
  critical_section_exit(&data_lock);

  // Run PID loop for all motors. This needs to be called periodically.
  motor_fl.PidLoopHandler();
  motor_fr.PidLoopHandler();
  motor_rl.PidLoopHandler();
  motor_rr.PidLoopHandler();

  // Compute odometry
  critical_section_enter_blocking(&data_lock);
  float avg_vel = (motor_fl.GetVelocity() + motor_fr.GetVelocity() + motor_rl.GetVelocity() +
    motor_rr.GetVelocity()) /
    4.0f / 1000.0f;                 // rad/s
  float linear_vel = avg_vel * WHEEL_RADIUS;
  float angular_vel = ((motor_fr.GetVelocity() + motor_rr.GetVelocity() - motor_fl.GetVelocity() -
    motor_rl.GetVelocity()) /
    4.0f / 1000.0f) *
    WHEEL_RADIUS / (TRACK_WIDTH / 2.0f);
  OdomBuffer.linear_vel = linear_vel;
  OdomBuffer.angular_vel = angular_vel;
  // Integrate pose (simple, no proper integration)
  static uint32_t last_time = 0;
  uint32_t now = time_us_32() / 1000;
  float dt = (now - last_time) / 1000.0f;
  if (last_time != 0) {
    OdomBuffer.pose_x += linear_vel * dt * cos(OdomBuffer.pose_theta);
    OdomBuffer.pose_y += linear_vel * dt * sin(OdomBuffer.pose_theta);
    OdomBuffer.pose_theta += angular_vel * dt;
  }
  last_time = now;
  OdomAvailable = true;
  critical_section_exit(&data_lock);

  // ToF sensor (placeholder)
  critical_section_enter_blocking(&data_lock);
  TofSensorBuffer.range = 1.0f;           // meters
  TofSensorBuffer.field_of_view = 0.1f;   // radians
  TofSensorAvailable = true;
  critical_section_exit(&data_lock);

  watchdog_manager_update();
}

void control_loop_set_motor_setpoints(const double * setpoints)
{
  // Setpoint is in rad/s, MotorClass expects rad/s * 1000
  motor_fl.SetPidSetpoint((float)setpoints[0]);
  motor_fr.SetPidSetpoint((float)setpoints[1]);
  motor_rl.SetPidSetpoint((float)setpoints[2]);
  motor_rr.SetPidSetpoint((float)setpoints[3]);
}

void control_loop_set_motor_speeds_int(int fl, int fr, int rl, int rr)
{
  // This function bypasses PID and sets raw PWM values.
  // Useful for low-level testing.
  motor_fl.SetMove(fl);
  motor_fr.SetMove(fr);
  motor_rl.SetMove(rl);
  motor_rr.SetMove(rr);
}

} // extern "C"
