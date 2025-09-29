#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "motor_driver.h"
#include "servo_driver.h"
#include "ros_manager.h"
#include "app_tasks.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>

// Provide sensible defaults if the build system did not set the debug options.
#ifndef ENABLE_DEBUG_UART
#define ENABLE_DEBUG_UART 1
#endif

#ifndef DEBUG_USE_UART1
#define DEBUG_USE_UART1 0
#endif

static void micro_ros_task(void *parameters)
{
    (void)parameters;
    const TickType_t delay_ticks = pdMS_TO_TICKS(10);
    for (;;)
    {
        ros_manager_spin_some();
        vTaskDelay(delay_ticks);
    }
}

// Create a small debug UART task that prints human-readable status
// This sends readable logs on uart0 (GPIO0 TX / GPIO1 RX) at 115200
// so you can monitor them with picocom on /dev/ttyUSB0.
#if ENABLE_DEBUG_UART
static void debug_uart_task(void *parameters)
{
    (void)parameters;
    const uint LED_PIN_DBG = 25;
    const uint uart_baud = 115200;
// Initialize UART0 (safe to call multiple times)
#if DEBUG_USE_UART1
    uart_init(uart1, uart_baud);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
#else
    uart_init(uart0, uart_baud);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
#endif

    char buf[128];
    /* Optional debug toggle pin to provide a visible pulse when ALIVE is sent.
        This is helpful for hardware checks with a multimeter or LED. */
    const uint DEBUG_TOGGLE_PIN = 2;
    gpio_init(DEBUG_TOGGLE_PIN);
    gpio_set_dir(DEBUG_TOGGLE_PIN, GPIO_OUT);
    gpio_put(DEBUG_TOGGLE_PIN, 0);
    TickType_t last = xTaskGetTickCount();
    for (;;)
    {
        // uptime in ms
        uint32_t uptime_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        int n = snprintf(buf, sizeof(buf), "ALIVE uptime=%lu ms\r\n", (unsigned long)uptime_ms);
        if (n > 0)
        {
// uart_puts appends no extra newline; use uart_write_blocking for full control
#if DEBUG_USE_UART1
            uart_write_blocking(uart1, (const uint8_t *)buf, (size_t)n);
#else
            uart_write_blocking(uart0, (const uint8_t *)buf, (size_t)n);
#endif
            /* short pulse to indicate a send (5ms) */
            gpio_put(DEBUG_TOGGLE_PIN, 1);
            sleep_ms(5);
            gpio_put(DEBUG_TOGGLE_PIN, 0);
            /* Also emit a USB-CDC stdio heartbeat so we can confirm firmware is alive
               even if the hardware UART isn't visible. This will appear on /dev/ttyACM0
               (but will interfere with micro-ROS binary transport if Agent is attached).
               Use only for debugging. */
            printf("ALIVE usb uptime=%lu ms\r\n", (unsigned long)uptime_ms);
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // ENABLE_DEBUG_UART

int main(void)
{
    stdio_init_all();

    const uint status_led_pin = 25;
    gpio_init(status_led_pin);
    gpio_set_dir(status_led_pin, GPIO_OUT);
    gpio_put(status_led_pin, 0);

    motor_driver_init();
    servo_driver_init();

    if (!ros_manager_init())
    {
        // If ROS manager initialization fails, don't lock the firmware in a
        // blocking blink loop — continue so the debug UART task can run and
        // provide diagnostics over the hardware UART. In production you may
        // want to handle this differently.
        printf("ros_manager_init() failed — continuing for debug.\r\n");
        fflush(stdout);
        // Briefly flash the status LED to indicate an error occurred
        for (int i = 0; i < 3; ++i)
        {
            gpio_put(status_led_pin, 1);
            sleep_ms(150);
            gpio_put(status_led_pin, 0);
            sleep_ms(150);
        }
        // continue startup to allow debug task / RTOS scheduler to run
    }

    if (xTaskCreate(micro_ros_task, "uROS", 4096, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
    {
        while (true)
        {
            gpio_put(status_led_pin, 1);
            sleep_ms(200);
            gpio_put(status_led_pin, 0);
            sleep_ms(200);
        }
    }

    app_tasks_start();

#if ENABLE_DEBUG_UART
    if (xTaskCreate(debug_uart_task, "dbgUART", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)
    {
        // continue even if debug task creation fails
    }
#endif

    vTaskStartScheduler();

    while (true)
    {
        sleep_ms(1000);
    }

    return 0;
}
