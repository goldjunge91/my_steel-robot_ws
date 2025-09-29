// Main entry point for the Pico firmware.
// Refactored for modern C++ style by Gemini.

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <cstdio>

// Define the built-in LED pin explicitly (GP25 on Pico)
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

// Application-specific headers
#include "application/shooter_control.h"
#include "application/watchdog_manager.h"
#include "hardware/ImuLib_cfg.h"
#include "hardware/bsp.h"
#include "hardware/config.h"
#include "middleware/command_processor.h"
#include "middleware/control_loop.h"
#include "middleware/micro_ros_cfg.h"
#include "middleware/usb_interface.h"
#include "shared/hardware_cfg.h"

// --- Global Data Buffers ---
// These are shared with other parts of the system (e.g., micro-ROS) via extern declarations.
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;
double SetpointBuffer[4] = {0};
bool SetpointAvailable = false;
motor_state_queue_t MotorStateBuffer;
bool MotorStateAvailable = false;
imu_queue_t ImuBuffer;
bool ImuAvailable = false;
battery_state_queue_t BatteryStateBuffer;
bool BatteryStateAvailable = false;
odom_queue_t OdomBuffer;
bool OdomAvailable = false;
range_queue_t TofSensorBuffer;
bool TofSensorAvailable = false;

// --- Critical Section for Thread Safety ---
critical_section_t data_lock;

// --- Safety Timers ---
uint64_t last_cmd_time = 0;

// --- Battery Status ---
float battery_voltage = 12.0f;

// --- Connection Status ---
bool connection_lost = false;
bool connection_alive = true;

// Forward declarations
void control_loop_core1();

/**
 * @brief Timer callback to blink the onboard LED.
 *
 * @param t Pointer to the repeating_timer structure.
 * @return true to continue the timer, false to stop.
 */
bool blink_timer_callback(struct repeating_timer *t) {
    static bool led_state = true;
    gpio_put(PICO_DEFAULT_LED_PIN, led_state);
    led_state = !led_state;
    printf("LED blinked, state: %d\n", led_state);
    return true; // Keep repeating
}

/**
 * @class Application
 * @brief Wraps the entire robot firmware application, managing initialization and the main loop.
 */
class Application {
  public:
    /**
     * @brief Construct a new Application object and initializes peripheral drivers.
     */
    Application()
        : shooter(ESC1_PIN, ESC2_PIN, GEAR_PIN, PAN_PIN, TILT_PIN),
          imu(0, IMU_ADDR_A) // I2C bus 0, Address A from hardware_cfg.h
    {}

    /**
     * @brief Initializes all hardware and software subsystems.
     */
    void init() {
        // Initialize stdio for USB communication
        stdio_init_all();
        printf("Pico Firmware Initializing...\n");

        // Initialize the onboard LED
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

        // Set up a repeating timer to blink the LED, indicating the system is alive.
        // This is more robust than blinking in the main loop, as it will continue
        // even if the main loop gets blocked.
        static repeating_timer_t timer;
        if (!add_repeating_timer_ms(1000, blink_timer_callback, NULL, &timer)) {
            printf("Failed to add blink timer\n");
        } else {
            printf("Blink timer added successfully\n");
        }

        // Initialize core board hardware (GPIO, I2C, UART)
        BoardPheripheralsInit();

        // Initialize control systems
        control_loop_init();
        shooter.init();

        // Initialize IMU and report status
        if (!imu.Init()) {
            printf("IMU Init Failed!\n");
        }

        // Initialize watchdog as the last step
        watchdog_manager_init();

        // Initialize critical section for thread safety
        critical_section_init(&data_lock);

        // Launch motor control on Core1
        multicore_launch_core1(control_loop_core1);

        printf("Initialization complete. Entering main loop.\n");
    }

    /**
     * @brief Runs the main application loop forever.
     */
    void run() {
        while (true) {
            run_once();
        }
    }

  private:
    /**
     * @brief Executes a single iteration of the main application loop.
     */
    void run_once() {
        // Poll for external data and commands
        PowerBoardSerial.UartProtocolLoopHandler();
        command_processor_task(); // Handles commands from USB

        // Update all control loops (now on Core1)
        // control_loop_update(); // Moved to Core1

        // Read IMU data and make it available for micro-ROS
        ImuBuffer = imu.LoopHandler();
        ImuAvailable = true;

        // The LED is now blinked by a timer interrupt.
    }

    // --- Member Variables ---
    ShooterControl shooter;
    ImuDriver imu;
};

// --- Core1 Function ---
void control_loop_core1() {
    while (true) {
        control_loop_update();
        sleep_ms(1); // 1ms loop for 1kHz
    }
}

// --- Main Entry Point ---
int main() {
    Application app;
    app.init();
    app.run(); // This call never returns
    return 0;
}