#include "motor_driver.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STBY_PIN 28

#define FL_IN1 18
#define FL_IN2 19
#define FL_PWM 2
#define ENC_FL_A 8
#define ENC_FL_B 9

#define FR_IN1 17
#define FR_IN2 20
#define FR_PWM 3
#define ENC_FR_A 10
#define ENC_FR_B 11

#define RL_IN1 21
#define RL_IN2 22
#define RL_PWM 4
#define ENC_RL_A 12
#define ENC_RL_B 13

#define RR_IN1 26
#define RR_IN2 27
#define RR_PWM 5
#define ENC_RR_A 6
#define ENC_RR_B 7

#define MOTOR_PWM_WRAP 999
#define MOTOR_PWM_CLKDIV 6.25f
#define ENCODER_TICKS_PER_REV 2048.0
#define COMMAND_TIMEOUT_US (500 * 1000)

typedef struct
{
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t pwm_pin;
    uint8_t encoder_a_pin;
    uint8_t encoder_b_pin;
    uint pwm_slice;
    uint pwm_channel;
    volatile int32_t ticks;
    uint8_t encoder_state;
    bool invert_direction;
} motor_hw_t;

static motor_hw_t motors[MOTOR_COUNT] = {
    [0] = {FL_IN1, FL_IN2, FL_PWM, ENC_FL_A, ENC_FL_B, 0, 0, 0, 0, false},
    [1] = {FR_IN1, FR_IN2, FR_PWM, ENC_FR_A, ENC_FR_B, 0, 0, 0, 0, true},
    [2] = {RL_IN1, RL_IN2, RL_PWM, ENC_RL_A, ENC_RL_B, 0, 0, 0, 0, false},
    [3] = {RR_IN1, RR_IN2, RR_PWM, ENC_RR_A, ENC_RR_B, 0, 0, 0, 0, true},
};

static motor_hw_t *encoder_motor_lookup[32] = {0};

static double motor_positions[MOTOR_COUNT] = {0};
static double motor_velocities[MOTOR_COUNT] = {0};
static int32_t last_tick_counts[MOTOR_COUNT] = {0};
static float motor_commands[MOTOR_COUNT] = {0};
static uint64_t last_cmd_time_us = 0;
static bool encoder_irq_initialized = false;

static const int8_t quadrature_table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

static void motor_apply_command(size_t index, float command);
static void encoder_isr(uint gpio, uint32_t events);

void motor_driver_init(void)
{
    gpio_init(STBY_PIN);
    gpio_set_dir(STBY_PIN, GPIO_OUT);
    gpio_put(STBY_PIN, 1);

    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor_hw_t *motor = &motors[i];
        gpio_init(motor->in1_pin);
        gpio_set_dir(motor->in1_pin, GPIO_OUT);
        gpio_put(motor->in1_pin, 0);

        gpio_init(motor->in2_pin);
        gpio_set_dir(motor->in2_pin, GPIO_OUT);
        gpio_put(motor->in2_pin, 0);

        gpio_set_function(motor->pwm_pin, GPIO_FUNC_PWM);
        motor->pwm_slice = pwm_gpio_to_slice_num(motor->pwm_pin);
        motor->pwm_channel = pwm_gpio_to_channel(motor->pwm_pin);

        pwm_config config = pwm_get_default_config();
        pwm_config_set_clkdiv(&config, MOTOR_PWM_CLKDIV);
        pwm_config_set_wrap(&config, MOTOR_PWM_WRAP);
        pwm_init(motor->pwm_slice, &config, true);
        pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, 0);

        gpio_init(motor->encoder_a_pin);
        gpio_set_dir(motor->encoder_a_pin, GPIO_IN);
        gpio_pull_up(motor->encoder_a_pin);

        gpio_init(motor->encoder_b_pin);
        gpio_set_dir(motor->encoder_b_pin, GPIO_IN);
        gpio_pull_up(motor->encoder_b_pin);

        motor->encoder_state = (gpio_get(motor->encoder_a_pin) << 1) | gpio_get(motor->encoder_b_pin);
        motor->ticks = 0;

        encoder_motor_lookup[motor->encoder_a_pin] = motor;
        encoder_motor_lookup[motor->encoder_b_pin] = motor;

        uint32_t events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
        if (!encoder_irq_initialized)
        {
            gpio_set_irq_enabled_with_callback(motor->encoder_a_pin, events, true, encoder_isr);
            encoder_irq_initialized = true;
        }
        else
        {
            gpio_set_irq_enabled(motor->encoder_a_pin, events, true);
        }
        gpio_set_irq_enabled(motor->encoder_b_pin, events, true);
    }

    memset(motor_positions, 0, sizeof(motor_positions));
    memset(motor_velocities, 0, sizeof(motor_velocities));
    memset(last_tick_counts, 0, sizeof(last_tick_counts));
    memset(motor_commands, 0, sizeof(motor_commands));
    last_cmd_time_us = time_us_64();
}

void motor_driver_apply_commands(const float *commands, size_t command_count)
{
    if (commands == NULL)
    {
        return;
    }

    size_t count = command_count > MOTOR_COUNT ? MOTOR_COUNT : command_count;
    for (size_t i = 0; i < count; i++)
    {
        motor_apply_command(i, commands[i]);
    }

    for (size_t i = count; i < MOTOR_COUNT; i++)
    {
        motor_apply_command(i, 0.0f);
    }

    last_cmd_time_us = time_us_64();
}

void motor_driver_stop_all(void)
{
    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor_apply_command(i, 0.0f);
    }
}

void motor_driver_handle_timeout(uint64_t now_us)
{
    if (last_cmd_time_us == 0)
    {
        return;
    }

    if (now_us - last_cmd_time_us > COMMAND_TIMEOUT_US)
    {
        for (size_t i = 0; i < MOTOR_COUNT; i++)
        {
            if (fabsf(motor_commands[i]) > 1e-3f)
            {
                motor_apply_command(i, 0.0f);
            }
        }
    }
}

void motor_driver_update_measurements(double dt)
{
    const double ticks_to_rad = (2.0 * M_PI) / ENCODER_TICKS_PER_REV;
    if (dt <= 0.0)
    {
        return;
    }

    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor_hw_t *motor = &motors[i];
        int32_t current_ticks = motor->ticks;
        int32_t delta_ticks = current_ticks - last_tick_counts[i];
        last_tick_counts[i] = current_ticks;

        double delta_rad = delta_ticks * ticks_to_rad;
        motor_positions[i] += delta_rad;
        motor_velocities[i] = delta_rad / dt;
    }
}

void motor_driver_get_joint_state(double *positions_out, double *velocities_out)
{
    if (positions_out != NULL)
    {
        memcpy(positions_out, motor_positions, sizeof(motor_positions));
    }
    if (velocities_out != NULL)
    {
        memcpy(velocities_out, motor_velocities, sizeof(motor_velocities));
    }
}

static void motor_apply_command(size_t index, float command)
{
    if (index >= MOTOR_COUNT)
    {
        return;
    }

    motor_hw_t *motor = &motors[index];
    float adjusted = motor->invert_direction ? -command : command;
    float magnitude = fabsf(adjusted);

    if (magnitude < 0.01f)
    {
        gpio_put(motor->in1_pin, 0);
        gpio_put(motor->in2_pin, 0);
        pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, 0);
    }
    else
    {
        if (magnitude > 1.0f)
        {
            magnitude = 1.0f;
        }
        bool forward = adjusted >= 0.0f;
        gpio_put(motor->in1_pin, forward ? 1 : 0);
        gpio_put(motor->in2_pin, forward ? 0 : 1);
        uint16_t level = (uint16_t)(magnitude * (MOTOR_PWM_WRAP + 1));
        if (level > MOTOR_PWM_WRAP)
        {
            level = MOTOR_PWM_WRAP;
        }
        pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, level);
    }

    motor_commands[index] = command;
}

static void encoder_isr(uint gpio, uint32_t events)
{
    gpio_acknowledge_irq(gpio, events);

    motor_hw_t *motor = encoder_motor_lookup[gpio];
    if (motor == NULL)
    {
        return;
    }

    uint8_t new_state = (gpio_get(motor->encoder_a_pin) << 1) | gpio_get(motor->encoder_b_pin);
    uint8_t index = ((motor->encoder_state << 2) | new_state) & 0x0F;
    motor->ticks += quadrature_table[index];
    motor->encoder_state = new_state;
}
