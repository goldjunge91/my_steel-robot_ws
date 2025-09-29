#include "servo_driver.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PAN_SERVO_PIN 23
#define TILT_SERVO_PIN 24

#define SERVO_PWM_WRAP 24999
#define SERVO_PWM_CLKDIV 100.0f
#define SERVO_FRAME_US 20000.0f
#define SERVO_MIN_PULSE_US 1000.0f
#define SERVO_MAX_PULSE_US 2000.0f
#define SERVO_MIN_DEG -90.0f
#define SERVO_MAX_DEG 90.0f

typedef struct
{
    uint8_t pin;
    uint slice;
    uint channel;
    float target_deg;
} servo_hw_t;

static servo_hw_t servos[SERVO_COUNT] = {
    [0] = {PAN_SERVO_PIN, 0, 0, 0.0f},
    [1] = {TILT_SERVO_PIN, 0, 0, 0.0f},
};

static uint16_t angle_to_level(float angle_deg);
static float clampf(float value, float min_value, float max_value);

void servo_driver_init(void)
{
    for (size_t i = 0; i < SERVO_COUNT; i++)
    {
        servo_hw_t *servo = &servos[i];
        gpio_set_function(servo->pin, GPIO_FUNC_PWM);
        servo->slice = pwm_gpio_to_slice_num(servo->pin);
        servo->channel = pwm_gpio_to_channel(servo->pin);

        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, SERVO_PWM_CLKDIV);
        pwm_config_set_wrap(&config, SERVO_PWM_WRAP);
        pwm_init(servo->slice, &config, true);
        pwm_set_chan_level(servo->slice, servo->channel, angle_to_level(servo->target_deg));
    }
}

void servo_driver_set_targets(const float *targets, size_t target_count)
{
    if (targets == NULL)
    {
        return;
    }

    size_t count = target_count > SERVO_COUNT ? SERVO_COUNT : target_count;
    for (size_t i = 0; i < count; i++)
    {
        servo_hw_t *servo = &servos[i];
        servo->target_deg = targets[i];
        uint16_t level = angle_to_level(servo->target_deg);
        pwm_set_chan_level(servo->slice, servo->channel, level);
    }
}

static uint16_t angle_to_level(float angle_deg)
{
    float clamped = clampf(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    float ratio = (clamped - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
    float pulse = SERVO_MIN_PULSE_US + ratio * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    float duty = pulse / SERVO_FRAME_US;
    float level_f = duty * (SERVO_PWM_WRAP + 1);
    if (level_f < 0.0f)
    {
        level_f = 0.0f;
    }
    if (level_f > SERVO_PWM_WRAP)
    {
        level_f = SERVO_PWM_WRAP;
    }
    return (uint16_t)(level_f);
}

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}
