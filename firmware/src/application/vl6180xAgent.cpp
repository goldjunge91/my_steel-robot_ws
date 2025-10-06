#include "application/vl6180xAgent.hpp"

#include "hardware/i2c.h"  // Für i2c_deinit/init
#include "uRosBridge.h"

#include <cstdio>

namespace application {

using hal::hardware::Vl6180x;

// Standard-Konfiguration, falls keine übergeben wird
Vl6180x::Config makeDefaultVl6180xConfig() {
    Vl6180x::Config cfg{};
    cfg.bus = i2c1;
    cfg.sda_pin = 2;
    cfg.scl_pin = 3;
    return cfg;
}

Vl6180xAgent::Vl6180xAgent() : Vl6180xAgent(makeDefaultVl6180xConfig()) {}

Vl6180xAgent::Vl6180xAgent(const Vl6180x::Config &config) : config_(config), sensor_(config) {
    sensor_msgs__msg__Range__init(&range_msg_);
    rosidl_runtime_c__String__assign(&range_msg_.header.frame_id, "tof_link");
    range_msg_.radiation_type = sensor_msgs__msg__Range__INFRARED;
    range_msg_.field_of_view = 0.1;  // Geschätzter Wert
    range_msg_.min_range = 0.0;
    range_msg_.max_range = 0.2;  // Der Sensor ist für kurze Distanzen

    sensor_msgs__msg__Illuminance__init(&illuminance_msg_);
    rosidl_runtime_c__String__assign(&illuminance_msg_.header.frame_id, "tof_link");
}

Vl6180xAgent::~Vl6180xAgent() {
    sensor_msgs__msg__Range__fini(&range_msg_);
    sensor_msgs__msg__Illuminance__fini(&illuminance_msg_);
}

void Vl6180xAgent::createEntities(rcl_node_t *node, rclc_support_t *support) {
    (void)support;
    printf("[Vl6180xAgent] Creating entities...\n");
    rclc_publisher_init_default(&range_publisher_,
                                node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                                "range_tof");

    rclc_publisher_init_default(&illuminance_publisher_,
                                node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Illuminance),
                                "illuminance");
    entities_active_ = 2;
}

void Vl6180xAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support) {
    (void)support;
    rcl_publisher_fini(&range_publisher_, node);
    rcl_publisher_fini(&illuminance_publisher_, node);
    entities_active_ = 0;
}

uint Vl6180xAgent::getHandles() {
    // This agent only provides publishers and no subscriptions/timers
    // so it does not require executor handles.
    return 0;
}

void Vl6180xAgent::addToExecutor(rclc_executor_t *executor) {
    (void)executor;  // No subscriptions or timers to add
}
uint Vl6180xAgent::getCount() {
    return entities_active_;
}

void Vl6180xAgent::run() {
    printf("[Vl6180xAgent] Task started.\n");

    for (;;) {
        if (!ensureInitialized()) {
            printf("[Vl6180xAgent] Retrying initialization in 2s...\n");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        uint8_t range = sensor_.readRange();
        float lux = sensor_.readLux();

        if (entities_active_ > 0 && uRosBridge::getInstance()->isSessionReady()) {
            int64_t timestamp = rmw_uros_epoch_nanos();

            // Range-Nachricht füllen und senden
            range_msg_.header.stamp.sec = timestamp / 1000000000;
            range_msg_.header.stamp.nanosec = timestamp % 1000000000;
            range_msg_.range = static_cast<float>(range) / 1000.0f;  // in Meter umrechnen
            uRosBridge::getInstance()->publish(&range_publisher_, &range_msg_, this, nullptr);

            // Illuminance-Nachricht füllen und senden
            illuminance_msg_.header.stamp.sec = timestamp / 1000000000;
            illuminance_msg_.header.stamp.nanosec = timestamp % 1000000000;
            illuminance_msg_.illuminance = lux;
            uRosBridge::getInstance()->publish(
                &illuminance_publisher_, &illuminance_msg_, this, nullptr);
        }
        vTaskDelay(pdMS_TO_TICKS(publish_period_ms_));
    }
}

configSTACK_DEPTH_TYPE Vl6180xAgent::getMaxStackSize() {
    return 1024;
}

bool Vl6180xAgent::ensureInitialized() {
    if (initialized_) {
        return true;
    }

    // Harter Reset des I2C-Busses direkt vor der Initialisierung, um Timing-Probleme im RTOS zu
    // vermeiden
    i2c_deinit(config_.bus);
    sleep_ms(20);                   // Kurze Pause
    i2c_init(config_.bus, 100000);  // Neuinitialisierung mit sicheren 100kHz
    gpio_set_function(config_.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config_.scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(config_.sda_pin);
    gpio_pull_up(config_.scl_pin);
    sleep_ms(20);  // Weitere Pause

    printf("[Vl6180xAgent] Attempting to initialize sensor...\n");
    initialized_ = sensor_.initialize();
    if (!initialized_) {
        printf("[Vl6180xAgent] Initialization failed.\n");
    } else {
        printf("[Vl6180xAgent] Sensor successfully initialized.\n");
    }
    return initialized_;
}

}  // namespace application
