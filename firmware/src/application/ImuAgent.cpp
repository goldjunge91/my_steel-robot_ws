#include "application/ImuAgent.h"

#include <cstdio>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "uRosBridge.h"
// FreeRTOS Includes
#include "FreeRTOS.h"
#include "task.h"

namespace {
// KORREKTUR: 'constexpr' ist hier der beste Typ, da es sich um eine Compile-Time-Konstante handelt.
// Der ursprüngliche Fehler war ein Phantomfehler des IntelliSense-Parsers.
constexpr uint8_t kDefaultAddress = 0x68;
constexpr uint32_t kDefaultBaudrateHz = 400000;
constexpr float kGtoMetersPerSecond2 = 9.80665f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

struct DefaultPins {
    static uint8_t sda() {
#ifdef PICO_DEFAULT_I2C_SDA_PIN
        return PICO_DEFAULT_I2C_SDA_PIN;
#else
        return 4;
#endif
    }

    static uint8_t scl() {
#ifdef PICO_DEFAULT_I2C_SCL_PIN
        return PICO_DEFAULT_I2C_SCL_PIN;
#else
        return 5;
#endif
    }
};

hal::hardware::Icm20948Simple::Config makeDefaultConfigInternal() {
    hal::hardware::Icm20948Simple::Config cfg{};
    
    // KORREKTUR für "identifier 'i2cPICO_DEFAULT_I2C' is undefined":
    // Diese Logik ist bereits korrekt und robust, da sie Standard-Makros prüft
    // und auf ein bekanntes Default (i2c0) zurückfällt.
#if defined(i2c_default)
    cfg.bus = i2c_default;
#elif defined(PICO_DEFAULT_I2C_INSTANCE)
    cfg.bus = PICO_DEFAULT_I2C_INSTANCE;
#else
    cfg.bus = i2c0;
#endif

    cfg.baudrate_hz = kDefaultBaudrateHz;
    cfg.address = kDefaultAddress;
    cfg.sda_pin = DefaultPins::sda();
    cfg.scl_pin = DefaultPins::scl();
    cfg.enable_pullups = true;
    return cfg;
}

}  // namespace

namespace application {

using hal::hardware::Icm20948Simple;
using shared::Vector3f;

ImuAgent::ImuAgent()
    : ImuAgent(makeDefaultConfigInternal()) {}

ImuAgent::ImuAgent(const hal::hardware::Icm20948Simple::Config &config)
     // KORREKTUR für "function returning function is not allowed" (Phantomfehler):
     // Verwendung von {} Initialisierung ist moderner C++ und vermeidet Parser-Fehldeutungen.
    : config_(config), sensor_{config_} {
    sensor_msgs__msg__Imu__init(&imu_msg_);

    for (double &v : imu_msg_.orientation_covariance) { v = 0.0; }
    imu_msg_.orientation_covariance[0] = -1.0;

    for (double &v : imu_msg_.angular_velocity_covariance) { v = 0.0; }
    imu_msg_.angular_velocity_covariance[0] = 0.02;
    imu_msg_.angular_velocity_covariance[4] = 0.02;
    imu_msg_.angular_velocity_covariance[8] = 0.02;

    for (double &v : imu_msg_.linear_acceleration_covariance) { v = 0.0; }
    imu_msg_.linear_acceleration_covariance[0] = 0.05;
    imu_msg_.linear_acceleration_covariance[4] = 0.05;
    imu_msg_.linear_acceleration_covariance[8] = 0.05;

    // KORREKTUR für "identifier '__null' is undefined":
    // In C++ sollte immer 'nullptr' anstelle von 'NULL' oder internen Makros verwendet werden.
    imu_msg_.header.frame_id.data = nullptr;
    imu_msg_.header.frame_id.size = 0;
    imu_msg_.header.frame_id.capacity = 0;

    imu_publisher_ = rcl_get_zero_initialized_publisher();
}

ImuAgent::~ImuAgent() {
    sensor_msgs__msg__Imu__fini(&imu_msg_);
}

// ... (Restlicher Code bleibt unverändert) ...

void ImuAgent::setFrameId(const char *frame_id) {
    if (frame_id == nullptr) { return; }
    if (!rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, frame_id)) {
        printf("[ImuAgent] Failed to set frame_id to %s\n", frame_id);
    }
    frame_id_set_ = true;
}

void ImuAgent::createEntities(rcl_node_t *node, rclc_support_t *support) {
    (void)support;
    if (!frame_id_set_) { setFrameId("imu_link"); }
    rcl_ret_t ret = rclc_publisher_init_default(
        &imu_publisher_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/ddd/imu");
    if (ret != RCL_RET_OK) { printf("[ImuAgent] Failed to create publisher: %d\n", ret); return; }
    entities_active_ = 1;
}

void ImuAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support) {
    (void)support;
    if (entities_active_ != 0) {
        rcl_ret_t ret = rcl_publisher_fini(&imu_publisher_, node);
        if (ret != RCL_RET_OK) { printf("[ImuAgent] Failed to destroy publisher: %d\n", ret); }
    }
    imu_publisher_ = rcl_get_zero_initialized_publisher();
    entities_active_ = 0;
}

uint ImuAgent::getCount() { return entities_active_; }

void ImuAgent::run() {
    for (;;) {
        if (!ensureInitialized()) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
        Vector3f accel{}, gyro{};
        float temperature = 0.0f;
        bool accel_ok = sensor_.readAcceleration(accel);
        bool gyro_ok = sensor_.readGyroscope(gyro);
        bool temp_ok = sensor_.readTemperature(temperature);
        if (accel_ok && gyro_ok) {
            if (!temp_ok) { temperature = 0.0f; }
            populateMessage(accel, gyro, temperature);
            if (entities_active_ != 0) {
                int64_t timestamp = rmw_uros_epoch_nanos();
                imu_msg_.header.stamp.sec = timestamp / 1000000000;
                imu_msg_.header.stamp.nanosec = timestamp % 1000000000;
                if (!uRosBridge::getInstance()->publish(&imu_publisher_, &imu_msg_, this, nullptr)) {
                    printf("[ImuAgent] Failed to queue IMU publish\n");
                }
            }
        } else {
            initialized_ = false;
            printf("[ImuAgent] Sensor read failed, reinitialising...\n");
        }
        vTaskDelay(pdMS_TO_TICKS(publish_period_ms_));
    }
}

configSTACK_DEPTH_TYPE ImuAgent::getMaxStackSize() { return 512; }

bool ImuAgent::ensureInitialized() {
    if (initialized_) { return true; }
    initialized_ = sensor_.initialize();
    if (!initialized_) { printf("[ImuAgent] Initialization failed\n"); } 
    else { printf("[ImuAgent] Sensor initialised\n"); }
    return initialized_;
}

void ImuAgent::populateMessage(const Vector3f &accel_g, const Vector3f &gyro_dps, float temperature_c) {
    (void)temperature_c;
    imu_msg_.linear_acceleration.x = accel_g.x * kGtoMetersPerSecond2;
    imu_msg_.linear_acceleration.y = accel_g.y * kGtoMetersPerSecond2;
    imu_msg_.linear_acceleration.z = accel_g.z * kGtoMetersPerSecond2;
    imu_msg_.angular_velocity.x = gyro_dps.x * kDegToRad;
    imu_msg_.angular_velocity.y = gyro_dps.y * kDegToRad;
    imu_msg_.angular_velocity.z = gyro_dps.z * kDegToRad;
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
}

// KORREKTUR für "a pointer to a bound function may only be used to call the function":
// Dieser Fehler tritt auf, wenn man eine Member-Funktion wie 'run()' direkt an FreeRTOS' xTaskCreate übergibt.
// Die Lösung ist eine statische "Wrapper"-Funktion. Der Code, der den Task erstellt, sollte so aussehen:
/*
void ImuAgent::startTask() {
    xTaskCreate(
        ImuAgent::taskRunner, // 1. Adresse der statischen Funktion
        "ImuAgentTask",
        getMaxStackSize(),
        this, // 2. Zeiger auf das aktuelle Objekt als Argument übergeben
        1,
        &task_handle_
    );
}

// 3. Die statische Wrapper-Funktion
void ImuAgent::taskRunner(void* pvParameters) {
    // Wandle den void-Zeiger zurück in einen ImuAgent-Zeiger
    ImuAgent* agent = static_cast<ImuAgent*>(pvParameters);
    // Rufe die eigentliche Member-Funktion auf diesem Objekt auf
    agent->run();
    // Der Task sollte hier niemals enden. Falls doch, lösche ihn.
    vTaskDelete(NULL);
}
*/

}  // namespace application