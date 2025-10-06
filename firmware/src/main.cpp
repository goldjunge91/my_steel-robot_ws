// /***
//  * Control Turntable and Led Lights
//  *
//  * Uses FreeRTOS Task
//  * Jon Durrant
//  * 15-Aug-2022
//  */

// #include "BlinkAgent.h"
// #include "DDD.h"
// #include "FreeRTOS.h"
// #include "HCSR04Agent.h"
// #include "MotorsAgent.h"
// #include "PubEntities.h"
// #include "application/ImuAgent.h"
// #include "application/vl6180xAgent.hpp"
// #include "hardware/i2c.h"
// #include "hardware/uart.h"
// #include "pico/stdlib.h"
// #include "task.h"
// #include "uRosBridge.h"

// #include <stdio.h>

// extern "C" {
// #include "pico/stdio.h"
// #include "pico/stdio/driver.h"
// #include "pico/stdio_uart.h"
// }

// #ifndef ENABLE_DEBUG_HEARTBEAT
// #define ENABLE_DEBUG_HEARTBEAT 1
// #endif

// #ifndef DEBUG_HEARTBEAT_INTERVAL_MS
// #define DEBUG_HEARTBEAT_INTERVAL_MS 5000
// #endif

// // Standard Task priority
// #define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// // LED PAD to use
// #define BLINK_LED_PAD 2
// #define CONN_LED_PAD 3

// // Left Motor
// #define LEFT_PWR_CW 9
// #define LEFT_PWR_CCW 8
// #define LEFT_ROTENC_A 14
// #define LEFT_ROTENV_B 15

// // Right Motor
// #define RIGHT_PWR_CW 6
// #define RIGHT_PWR_CCW 7
// #define RIGHT_ROTENC_A 12
// #define RIGHT_ROTENV_B 13

// // PID
// #define KP 0.55
// #define KI 0.019
// #define KD 0.24

// // IMU (SPI) Pins
// #define IMU_SPI_PORT spi0
// #define IMU_CS_PIN 17
// #define IMU_SCK_PIN 18
// #define IMU_MOSI_PIN 19
// #define IMU_MISO_PIN 16

// // VL6180X (I2C) Pins
// #define VL6180X_I2C_PORT i2c1
// #define VL6180X_SDA_PIN 2
// #define VL6180X_SCL_PIN 3

// char ROBOT_NAME[] = "ddd";

// static void init_stdio_ports() {
//     // Initialise all configured stdio drivers (USB + UART if enabled in CMake).
//     stdio_init_all();
//     // Force stdio UART onto GP0/GP1 so legacy serial monitors keep working.
//     stdio_uart_init_full(uart0, 115200, 0, 1);
// }

// #if ENABLE_DEBUG_HEARTBEAT
// static void debugHeartbeatTask(void *params) {
//     (void)params;
//     const TickType_t delay_ticks = pdMS_TO_TICKS(DEBUG_HEARTBEAT_INTERVAL_MS);
//     for (;;) {
//         uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
//         printf("[HEARTBEAT] Uptime: %lu ms\n", (unsigned long)uptime_ms);
//         vTaskDelay(delay_ticks);
//     }
// }
// #endif

// /***
//  * Main task to boot the other Agents
//  * @param params - unused
//  */
// void mainTask(void *params) {
//     printf("mainTask started. Initializing agents...\n");
//     // KORREKTUR: Alle Agenten als 'static' deklarieren, um Stack Overflow zu vermeiden
//     static BlinkAgent blink(BLINK_LED_PAD);

//     static MotorsAgent motors;
//     motors.addMotor(0, LEFT_PWR_CW, LEFT_PWR_CCW, LEFT_ROTENC_A, LEFT_ROTENV_B);
//     motors.addMotor(1, RIGHT_PWR_CW, RIGHT_PWR_CCW, RIGHT_ROTENC_A, RIGHT_ROTENV_B);
//     motors.configAllPID(KP, KI, KD);

//     static HCSR04Agent range;
//     range.addSensor(0, "range_front");
//     range.addSensor(18, "range_back");

//     // Konfiguration für den VL6180X Time-of-Flight Sensor (I2C)
//     hal::hardware::Vl6180x::Config tof_cfg{};
//     tof_cfg.bus = VL6180X_I2C_PORT;
//     tof_cfg.sda_pin = VL6180X_SDA_PIN;
//     tof_cfg.scl_pin = VL6180X_SCL_PIN;
//     static application::Vl6180xAgent tof(tof_cfg);

//     // Konfiguration für die IMU (SPI)
//     hal::hardware::Icm20948Simple::Config imu_cfg{};
//     imu_cfg.bus = IMU_SPI_PORT;
//     imu_cfg.cs_pin = IMU_CS_PIN;
//     imu_cfg.sck_pin = IMU_SCK_PIN;
//     imu_cfg.mosi_pin = IMU_MOSI_PIN;
//     imu_cfg.miso_pin = IMU_MISO_PIN;
//     static application::ImuAgent imu(imu_cfg);
//     imu.setFrameId("imu_link");

//     static DDD ddd;
//     ddd.setMotorsAgent(&motors);
//     ddd.setHCSR04Agent(&range);
//     ddd.setImuAgent(&imu);
//     ddd.setVl6180xAgent(&tof);

//     // Starten der Agenten-Tasks
//     printf("Starting BlinkAgent...\n");
//     blink.start("Blink", TASK_PRIORITY);

//     printf("Starting MotorsAgent...\n");
//     motors.start("Motors", TASK_PRIORITY);

//     printf("Starting HCSR04Agent...\n");
//     range.start("Range", TASK_PRIORITY);

//     printf("Starting Vl6180xAgent...\n");
//     tof.start("VL6180X", TASK_PRIORITY);

//     printf("Starting DDD Agent...\n");
//     ddd.start("DDD", TASK_PRIORITY);

//     // Starten der uROS Bridge
//     printf("Starting uRosBridge...\n");
//     static uRosBridge *bridge = uRosBridge::getInstance();
//     bridge->setuRosEntities(&ddd);
//     bridge->setLed(CONN_LED_PAD);
//     bridge->start("Bridge", TASK_PRIORITY + 2);
//     printf("micro-ROS bridge task started.\n");

//     // Warten, bis die micro-ROS-Sitzung bereit ist, bevor der IMU-Task gestartet wird
//     printf("Waiting for micro-ROS session...\n");
//     while (!uRosBridge::getInstance()->isSessionReady()) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
//     printf("micro-ROS session is ready.\n");

//     printf("Starting ImuAgent...\n");
//     imu.start("IMU", TASK_PRIORITY);
//     printf("All agents started.\n");

//     // Dieser Task hat seine Aufgabe erledigt und kann sich nun schlafen legen.
//     for (;;) {
//         vTaskDelay(portMAX_DELAY);
//     }
// }

// /***
//  * Launch the tasks and scheduler
//  */
// void vLaunch(void) {
//     // Erhöhe den Stack für den mainTask, um die Erstellung aller Objekte sicherzustellen
//     TaskHandle_t task;
//     xTaskCreate(mainTask, "MainThread", 2048, NULL, TASK_PRIORITY, &task);

// #if ENABLE_DEBUG_HEARTBEAT
//     xTaskCreate(
//         debugHeartbeatTask, "DbgBeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
// #endif

//     /* Start the tasks and timer running. */
//     vTaskStartScheduler();
// }

// /***
//  * Main
//  * @return
//  */
// int main(void) {
//     init_stdio_ports();
//     sleep_ms(2000);
//     printf("\n\n-- Booting %s firmware --\n", ROBOT_NAME);

//     // Start tasks and scheduler
//     const char *rtos_name = "FreeRTOS";
//     printf("Starting %s on core 0...\n", rtos_name);
//     vLaunch();

//     return 0;  // Should not be reached
// }

/***
 * Control Turntable and Led Lights
 *
 * Uses FreeRTOS Task
 * Jon Durrant
 * 15-Aug-2022
 */

#include "BlinkAgent.h"
#include "DDD.h"
#include "FreeRTOS.h"
#include "HCSR04Agent.h"
#include "MotorsAgent.h"
#include "PubEntities.h"
#include "application/ImuAgent.h"
#include "application/vl6180xAgent.hpp"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "task.h"
#include "uRosBridge.h"

#include <stdio.h>

extern "C" {
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/stdio_usb.h"  // Nötig für die manuelle Initialisierung
}

#ifndef ENABLE_DEBUG_HEARTBEAT
#define ENABLE_DEBUG_HEARTBEAT 1
#endif

#ifndef DEBUG_HEARTBEAT_INTERVAL_MS
#define DEBUG_HEARTBEAT_INTERVAL_MS 5000
#endif

// Standard Task priority
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// LED PAD to use
#define BLINK_LED_PAD 26
#define CONN_LED_PAD 26

// Front Left Motor (Motor 0)
#define FRONT_LEFT_PWR_CW 20
#define FRONT_LEFT_PWR_CCW 21
#define FRONT_LEFT_ROTENC_A 6
#define FRONT_LEFT_ROTENV_B 7

// Front Right Motor (Motor 1)
#define FRONT_RIGHT_PWR_CW 4
#define FRONT_RIGHT_PWR_CCW 5
#define FRONT_RIGHT_ROTENC_A 8
#define FRONT_RIGHT_ROTENV_B 9

// Rear Left Motor (Motor 2)
#define REAR_LEFT_PWR_CW 14
#define REAR_LEFT_PWR_CCW 15
#define REAR_LEFT_ROTENC_A 10
#define REAR_LEFT_ROTENV_B 11

// Rear Right Motor (Motor 3)
#define REAR_RIGHT_PWR_CW 22
#define REAR_RIGHT_PWR_CCW 28
#define REAR_RIGHT_ROTENC_A 12
#define REAR_RIGHT_ROTENV_B 13

// PID
#define KP 0.55
#define KI 0.019
#define KD 0.24

// IMU (SPI) Pins DONT CHANGE IT
#define IMU_SPI_PORT spi0
#define IMU_CS_PIN 17
#define IMU_SCK_PIN 18
#define IMU_MOSI_PIN 19
#define IMU_MISO_PIN 16

// VL6180X (I2C) Pins DONT CHANGE IT
#define VL6180X_I2C_PORT i2c1
#define VL6180X_SDA_PIN 2
#define VL6180X_SCL_PIN 3



char ROBOT_NAME[] = "robot_xl";

#if ENABLE_DEBUG_HEARTBEAT
static void debugHeartbeatTask(void *params) {
    (void)params;
    const TickType_t delay_ticks = pdMS_TO_TICKS(DEBUG_HEARTBEAT_INTERVAL_MS);
    for (;;) {
        uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
        printf("[HEARTBEAT] Uptime: %lu ms\n", (unsigned long)uptime_ms);
        vTaskDelay(delay_ticks);
    }
}
#endif

/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params) {
    printf("mainTask started. Initializing agents...\n");
    // KORREKTUR: Alle Agenten als 'static' deklarieren, um Stack Overflow zu vermeiden
    static BlinkAgent blink(BLINK_LED_PAD);

    static MotorsAgent motors;
    motors.addMotor(0, FRONT_LEFT_PWR_CW, FRONT_LEFT_PWR_CCW, FRONT_LEFT_ROTENC_A, FRONT_LEFT_ROTENV_B);
    motors.addMotor(1, FRONT_RIGHT_PWR_CW, FRONT_RIGHT_PWR_CCW, FRONT_RIGHT_ROTENC_A, FRONT_RIGHT_ROTENV_B);
    motors.addMotor(2, REAR_LEFT_PWR_CW, REAR_LEFT_PWR_CCW, REAR_LEFT_ROTENC_A, REAR_LEFT_ROTENV_B);
    motors.addMotor(3, REAR_RIGHT_PWR_CW, REAR_RIGHT_PWR_CCW, REAR_RIGHT_ROTENC_A, REAR_RIGHT_ROTENV_B);
    motors.configAllPID(KP, KI, KD);

    static HCSR04Agent range;
    range.addSensor(0, "range_front");
    range.addSensor(18, "range_back");

    // Konfiguration für den VL6180X Time-of-Flight Sensor (I2C)
    hal::hardware::Vl6180x::Config tof_cfg{};
    tof_cfg.bus = VL6180X_I2C_PORT;
    tof_cfg.sda_pin = VL6180X_SDA_PIN;
    tof_cfg.scl_pin = VL6180X_SCL_PIN;
    static application::Vl6180xAgent tof(tof_cfg);

    // Konfiguration für die IMU (SPI)
    hal::hardware::Icm20948Simple::Config imu_cfg{};
    imu_cfg.bus = IMU_SPI_PORT;
    imu_cfg.baudrate_hz = 1000 * 1000;  // 1MHz
    imu_cfg.cs_pin = IMU_CS_PIN;
    imu_cfg.sck_pin = IMU_SCK_PIN;
    imu_cfg.mosi_pin = IMU_MOSI_PIN;
    imu_cfg.miso_pin = IMU_MISO_PIN;
    static application::ImuAgent imu(imu_cfg);
    imu.setFrameId("imu_link");

    static DDD ddd;
    ddd.setMotorsAgent(&motors);
    ddd.setHCSR04Agent(&range);
    ddd.setImuAgent(&imu);
    ddd.setVl6180xAgent(&tof);

    // Starten der Agenten-Tasks
    printf("Starting BlinkAgent...\n");
    blink.start("Blink", TASK_PRIORITY);

    printf("Starting MotorsAgent...\n");
    motors.start("Motors", TASK_PRIORITY);

    printf("Starting HCSR04Agent...\n");
    range.start("Range", TASK_PRIORITY);

    printf("Starting Vl6180xAgent...\n");
    tof.start("VL6180X", TASK_PRIORITY);

    printf("Starting ImuAgent...\n");
    imu.start("IMU", TASK_PRIORITY);

    printf("Starting DDD Agent...\n");
    ddd.start("DDD", TASK_PRIORITY);

    // Starten der uROS Bridge
    printf("Starting uRosBridge...\n");
    static uRosBridge *bridge = uRosBridge::getInstance();
    bridge->setuRosEntities(&ddd);
    bridge->setLed(CONN_LED_PAD);
    bridge->start("Bridge", TASK_PRIORITY + 2);
    printf("micro-ROS bridge task started.\n");

    printf("All agents started. MainTask will now suspend.\n");

    // Dieser Task hat seine Aufgabe erledigt und kann sich nun schlafen legen.
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

/***
 * Launch the tasks and scheduler
 */
void vLaunch(void) {
    // Erhöhe den Stack für den mainTask, um die Erstellung aller Objekte sicherzustellen
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 2048, NULL, TASK_PRIORITY, &task);

#if ENABLE_DEBUG_HEARTBEAT
    xTaskCreate(
        debugHeartbeatTask, "DbgBeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

/***
 * Main
 * @return
 */
int main(void) {
    // Stellt sicher, dass die grundlegende Hardware initialisiert ist
    stdio_init_all();

    // *** KORRIGIERT: USB manuell für micro-ROS initialisieren, UART für printf ***
    stdio_usb_init();                           // Macht USB für den Transport verfügbar
    stdio_uart_init_full(uart0, 115200, 0, 1);  // Leitet printf auf UART um

    sleep_ms(2000);
    printf("\n\n-- Booting %s firmware (Debug on UART0) --\n", ROBOT_NAME);

    // Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0...\n", rtos_name);
    vLaunch();

    return 0;  // Should not be reached
}
