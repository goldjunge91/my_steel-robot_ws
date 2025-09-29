#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "motor_driver.h"
#include "servo_driver.h"
#include "ros_manager.h"

#include <stdio.h>
#include <string.h>

#define MOTOR_TASK_NAME "MotorCtrl"
#define MOTOR_TASK_STACK (configMINIMAL_STACK_SIZE + 256)
#define MOTOR_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define MOTOR_COMMAND_TIMEOUT_MS 500
#define MOTOR_CONTROL_PERIOD_MS 10

#define DIAG_TASK_NAME "Diag"
#define DIAG_TASK_STACK (configMINIMAL_STACK_SIZE + 128)
#define DIAG_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define DIAG_PERIOD_MS 5000

static void motor_control_task(void *parameters);
static void diagnostics_task(void *parameters);

void app_tasks_start(void)
{
    if (xTaskCreate(motor_control_task,
                    MOTOR_TASK_NAME,
                    MOTOR_TASK_STACK,
                    NULL,
                    MOTOR_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        /* TODO:In production we might log an error; for now we simply loop forever. */
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (xTaskCreate(diagnostics_task,
                    DIAG_TASK_NAME,
                    DIAG_TASK_STACK,
                    NULL,
                    DIAG_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void motor_control_task(void *parameters)
{
    (void)parameters;

    QueueHandle_t setpoint_queue = ros_manager_get_motor_setpoint_queue();
    float active_command[MOTOR_COUNT] = {0.0f};
    TickType_t last_command_tick = xTaskGetTickCount();
    const TickType_t control_delay = pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS);
    const TickType_t command_timeout = pdMS_TO_TICKS(MOTOR_COMMAND_TIMEOUT_MS);

    for (;;)
    {
        if (setpoint_queue != NULL)
        {
            /* Receive new command if available without blocking longer than control period. */
            if (xQueueReceive(setpoint_queue, active_command, control_delay) == pdPASS)
            {
                last_command_tick = xTaskGetTickCount();
            }
            else
            {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_command_tick) > command_timeout)
                {
                    for (size_t i = 0; i < MOTOR_COUNT; i++)
                    {
                        active_command[i] = 0.0f;
                    }
                }
            }
        }

        motor_driver_apply_commands(active_command, MOTOR_COUNT);
    }
}

static void diagnostics_task(void *parameters)
{
    (void)parameters;
    const TickType_t delay_ticks = pdMS_TO_TICKS(DIAG_PERIOD_MS);
    char task_list[256];

    for (;;)
    {
        memset(task_list, 0, sizeof(task_list));
#if (configUSE_TRACE_FACILITY == 1) && (INCLUDE_vTaskList == 1)
        vTaskList(task_list);
        printf("\n[Diag] Task snapshot\n%s\n", task_list);
#else
        printf("[Diag] Scheduler alive\n");
        (void)task_list;
#endif
        vTaskDelay(delay_ticks);
    }
}
