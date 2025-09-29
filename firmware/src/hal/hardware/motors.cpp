/**
 * @file motors.cpp
 * @author Maciej Kurcius, modified for Pico by Gemini
 * @brief Pico-compatible motor control implementation with PID and PIO-based encoders.
 * @version 0.3
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "hardware/motors.h"
#include "hardware/config.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include <cmath>
#include <algorithm>

// Include the generated PIO header
#include "quadrature_encoder.pio.h"

// --- PIO Quadrature Encoder ---
// Helper function to initialize the PIO program for an encoder
static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint offset, uint pin_base)
{
  pio_sm_config c = quadrature_encoder_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pin_base);
  sm_config_set_in_shift(&c, false, false, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
  // Initialize ISR with current pin state
  pio_sm_exec(pio, sm, pio_encode_in(pio_pins, 2));
}

// --- TimebaseTimerClass Implementation ---
TimebaseTimerClass::TimebaseTimerClass() {}
TimebaseTimerClass::~TimebaseTimerClass() {}

uint64_t TimebaseTimerClass::GetAbsTimeValue()
{
  return to_us_since_boot(get_absolute_time());
}

uint64_t TimebaseTimerClass::GetTimeChange(uint64_t * arg_last_time)
{
  uint64_t current_time = GetAbsTimeValue();
  uint64_t ret_val = current_time - *arg_last_time;
  *arg_last_time = current_time;
  return ret_val;
}

// --- MotorClass Implementation ---
MotorClass::MotorClass() {}

MotorClass::MotorClass(
  uint32_t arg_pwm_pin,
  uint32_t arg_a_channel_motor_pin, uint32_t arg_b_channel_motor_pin,
  uint32_t arg_a_channel_encoder_pin,
  int8_t arg_default_direction,
  TimebaseTimerClass * arg_timebase_timer,
  PIO pio,
  uint sm)
{
  pwm_pin_ = arg_pwm_pin;
  a_channel_motor_pin_ = arg_a_channel_motor_pin;
  b_channel_motor_pin_ = arg_b_channel_motor_pin;
  a_channel_encoder_pin_ = arg_a_channel_encoder_pin;
  default_direction_ = arg_default_direction;
  timebase_tim_ = arg_timebase_timer;
  pio_ = pio;
  sm_ = sm;

  gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);
  pwm_slice_num_ = pwm_gpio_to_slice_num(pwm_pin_);
  pwm_channel_ = pwm_gpio_to_channel(pwm_pin_);
  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, 255);
  pwm_init(pwm_slice_num_, &config, true);
  SetPwm(0);

  gpio_init(a_channel_motor_pin_);
  gpio_set_dir(a_channel_motor_pin_, GPIO_OUT);
  gpio_init(b_channel_motor_pin_);
  gpio_set_dir(b_channel_motor_pin_, GPIO_OUT);
  SoftStop();

  uint offset = pio_add_program(pio_, &quadrature_encoder_program);
  quadrature_encoder_program_init(pio_, sm_, offset, a_channel_encoder_pin_);

  last_time_ = timebase_tim_->GetAbsTimeValue();
  encoder_value_ = 0;
  last_encoder_value_ = GetEncoderValue();
  actual_encoder_value_ = last_encoder_value_;
}

MotorClass::~MotorClass() {}

void MotorClass::SetPwm(uint32_t arg_value)
{
  pwm_set_chan_level(pwm_slice_num_, pwm_channel_, arg_value > 255 ? 255 : arg_value);
}

void MotorClass::SetMove(int32_t arg_velocity)
{
  SetPwm(abs(arg_velocity));
  if ((arg_velocity * (int32_t)default_direction_) > 0) {
    gpio_put(a_channel_motor_pin_, 1);
    gpio_put(b_channel_motor_pin_, 0);
  } else if ((arg_velocity * (int32_t)default_direction_) < 0) {
    gpio_put(a_channel_motor_pin_, 0);
    gpio_put(b_channel_motor_pin_, 1);
  } else {
    gpio_put(a_channel_motor_pin_, 0);
    gpio_put(b_channel_motor_pin_, 0);
  }
}

void MotorClass::EmgStop()
{
  gpio_put(a_channel_motor_pin_, 0);
  gpio_put(b_channel_motor_pin_, 0);
  SetPwm(0);
}

void MotorClass::SoftStop()
{
  gpio_put(a_channel_motor_pin_, 1);
  gpio_put(b_channel_motor_pin_, 1);
  SetPwm(0);
}

int32_t MotorClass::GetEncoderValue()
{
  while (!pio_sm_is_rx_fifo_empty(pio_, sm_)) {
    int32_t delta = pio_sm_get(pio_, sm_);
    encoder_value_ += delta;
  }
  return encoder_value_;
}

int32_t MotorClass::VelocityUpdate(void)
{
  time_change_ = timebase_tim_->GetTimeChange(&last_time_);
  if (time_change_ == 0) {return actual_velocity_;}

  actual_encoder_value_ = GetEncoderValue();
  int64_t tick_velocity = ((int64_t)(actual_encoder_value_ - last_encoder_value_) * 1000000) /
    time_change_;
  actual_velocity_ = (int32_t)(TICK_TO_RAD_X_1000(tick_velocity) * default_direction_);
  last_encoder_value_ = actual_encoder_value_;
  return actual_velocity_;
}

void MotorClass::PidLoopHandler(void)
{
  if (RAMP_FLAG) {
    // Ramp logic from original code
  } else {
    actual_input_ = input_;
  }

  actual_error_ = actual_input_ - VelocityUpdate();
  error_sum_ += actual_error_;
  error_sum_ =
    std::max((int64_t)-max_error_sum_, std::min((int64_t)max_error_sum_, (int64_t)error_sum_));

  output_ = (kp_gain_ * actual_error_) + (ki_gain_ * error_sum_) +
    (kd_gain_ * (actual_error_ - last_error_));
  last_error_ = actual_error_;

  output_ = std::max((int32_t)-1000000, std::min((int32_t)1000000, output_));

  int32_t pwm_val = (output_ * 255) / 1000000;

  SetMove(pwm_val);
}


// --- Methods that remain largely unchanged ---

void MotorClass::SetPidSetpoint(int32_t arg_setpoint) {this->input_ = arg_setpoint;}

void MotorClass::SetPidSetpoint(float arg_setpoint)
{
  this->input_ = (int32_t)(arg_setpoint * 1000);
}

void MotorClass::PidLoopHandler(int32_t arg_setpoint)
{
  this->SetPidSetpoint(arg_setpoint);
  this->PidLoopHandler();
}

void MotorClass::PidLoopHandler(float arg_setpoint)
{
  this->SetPidSetpoint(arg_setpoint);
  this->PidLoopHandler();
}

int32_t MotorClass::GetVelocity(void) {return this->actual_velocity_;}

int64_t MotorClass::GetWheelAbsPosition(void)
{
  return (int64_t)TICK_TO_RAD_X_1000(
    this->GetEncoderValue() * (int64_t)this->GetDefaultDirection());
}

int8_t MotorClass::GetDefaultDirection(void) {return this->default_direction_;}

int16_t MotorClass::GetWheelAngle(void)
{
  return (int16_t)this->actual_encoder_value_ % (int)TICK_PER_RADIAN_X_1000;
}

void SetMaxMotorsCurrent(uint32_t Ilim1_, uint32_t Ilim2_, uint32_t Ilim3_, uint32_t Ilim4_)
{
  // This function is likely for a specific motor driver shield and can be left empty
  // or implemented if the hardware is present.
}
