/***
 * Step 2: Motors + Simple IMU (no sensors)
 */

#include "BlinkAgent.h"
#include "FreeRTOS.h"
#include "MotorsAgent.h"
#include "application/ImuAgent.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "task.h"
#include "uRosBridge.h"

#include <stdio.h>

extern "C" {
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
}

#ifndef ENABLE_DEBUG_HEARTBEAT
#define ENABLE_DEBUG_HEARTBEAT 1
#endif

#ifndef DEBUG_HEARTBEAT_INTERVAL_MS
#define DEBUG_HEARTBEAT_INTERVAL_MS 1000
#endif

// Standard Task priority
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// LED PAD to use - Built-in LED on Pico
#define BLINK_LED_PAD 25  // Built-in LED
#define CONN_LED_PAD 25   // Built-in LED (same for both)

// Left Motor
#define LEFT_PWR_CW 9
#define LEFT_PWR_CCW 8
#define LEFT_ROTENC_A 14
#define LEFT_ROTENV_B 15

// Right Motor
#define RIGHT_PWR_CW 6
#define RIGHT_PWR_CCW 7
#define RIGHT_ROTENC_A 12
#define RIGHT_ROTENV_B 13

// PID
#define KP 0.55
#define KI 0.019
#define KD 0.24

char ROBOT_NAME[] = "ddd_step2";

#if ENABLE_DEBUG_HEARTBEAT
static void debugHeartbeatTask(void *params) {
    (void)params;
    const TickType_t delay_ticks = pdMS_TO_TICKS(DEBUG_HEARTBEAT_INTERVAL_MS);
    for (;;) {
        uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "ALIVE uptime=%lu ms\r\n", (unsigned long)uptime_ms);
        if (n > 0) {
            uart_write_blocking(uart0, (const uint8_t *)buf, (size_t)n);
        }
        vTaskDelay(delay_ticks);
    }
}
#endif

/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params) {
    BlinkAgent blink(BLINK_LED_PAD);

    printf("Boot task started for %s\n", ROBOT_NAME);

    blink.start("Blink", TASK_PRIORITY);
    printf("Blink started\n");

    printf("Starting Motors...\n");
    MotorsAgent motors;
    motors.addMotor(0, LEFT_PWR_CW, LEFT_PWR_CCW, LEFT_ROTENC_A, LEFT_ROTENV_B);
    motors.addMotor(1, RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENV_B);
    motors.configAllPID(KP, KI, KD);
    motors.start("Motors", TASK_PRIORITY);
    printf("Motors started\n");

    printf("Starting Simple IMU...\n");
    // Create IMU instance but start bridge before starting the IMU task so
    // the bridge can create publishers (via setuRosEntities) and be ready
    // to accept published messages.
    static application::ImuAgent imu;  // Static to ensure it stays alive
    imu.setFrameId("imu_link");

    // Start up a uROS Bridge (IMU only for debugging)
    printf("Starting micro-ROS Bridge...\n");
    uRosBridge *bridge = uRosBridge::getInstance();
    bridge->setuRosEntities(&imu);  // Only IMU to debug publisher issue
    bridge->setLed(CONN_LED_PAD);
    bridge->start("Bridge", TASK_PRIORITY + 2);
    printf("micro-ROS Bridge started\n");
    // WICHTIG: Warte bis die Session bereit ist
    printf("Waiting for micro-ROS session to be ready...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));  // 3 Sekunden warten
    printf("Session should be ready now\n");
    // Now start the IMU task so it can publish to an active bridge
    imu.start("IMU", TASK_PRIORITY);
    printf("Simple IMU started\n");

    for (;;) {
        printf("Main task running...\n");
        vTaskDelay(10000);
    }
}

/***
 * Launch the tasks and scheduler
 */
void vLaunch(void) {
    // Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

#if ENABLE_DEBUG_HEARTBEAT
    xTaskCreate(debugHeartbeatTask, "DbgBeat", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

/***
 * Main
 * @return
 */
int main(void) {
    // Setup serial over UART and give a few seconds to settle before we start
    stdio_init_all();
    sleep_ms(2000);
    printf("GO STEP2\n");
    fflush(stdout);

    // Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0 (STEP2):\n", rtos_name);
    vLaunch();

    return 0;
}