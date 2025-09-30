// #include "application/ImuAgent.h"

// #include "uRosBridge.h"

// #include <cstdio>

// namespace {
// // Standard-Konstanten
// constexpr uint32_t kDefaultBaudrateHz = 4000000;  // 4 MHz für SPI
// constexpr float kGtoMetersPerSecond2 = 9.80665f;
// constexpr float kPi = 3.14159265358979323846f;
// constexpr float kDegToRad = kPi / 180.0f;

// // Standard-Pins für SPI0
// constexpr uint8_t SPI_SCK_PIN = 18;
// constexpr uint8_t SPI_MOSI_PIN = 19;
// constexpr uint8_t SPI_MISO_PIN = 16;
// constexpr uint8_t SPI_CS_PIN = 17;

// hal::hardware::Icm20948Simple::Config makeDefaultConfigInternal() {
//     hal::hardware::Icm20948Simple::Config cfg{};
//     cfg.bus = spi0;
//     cfg.baudrate_hz = kDefaultBaudrateHz;
//     cfg.cs_pin = SPI_CS_PIN;
//     cfg.sck_pin = SPI_SCK_PIN;
//     cfg.mosi_pin = SPI_MOSI_PIN;
//     cfg.miso_pin = SPI_MISO_PIN;
//     return cfg;
// }

// }  // namespace

// namespace application {

// using hal::hardware::Icm20948Simple;
// using shared::Vector3f;

// ImuAgent::ImuAgent() : ImuAgent(makeDefaultConfigInternal()) {}

// ImuAgent::ImuAgent(const Icm20948Simple::Config &config) : config_(config), sensor_(config) {
//     sensor_msgs__msg__Imu__init(&imu_msg_);
//     // Kovarianzen initialisieren
//     imu_msg_.orientation_covariance[0] = -1.0;  // Orientierung nicht bereitgestellt
//     for (size_t i = 1; i < 9; ++i) imu_msg_.orientation_covariance[i] = 0;

//     imu_msg_.angular_velocity_covariance[0] = 0.02;
//     imu_msg_.angular_velocity_covariance[4] = 0.02;
//     imu_msg_.angular_velocity_covariance[8] = 0.02;

//     imu_msg_.linear_acceleration_covariance[0] = 0.05;
//     imu_msg_.linear_acceleration_covariance[4] = 0.05;
//     imu_msg_.linear_acceleration_covariance[8] = 0.05;
// }

// ImuAgent::~ImuAgent() {
//     sensor_msgs__msg__Imu__fini(&imu_msg_);
// }

// void ImuAgent::setFrameId(const char *frame_id) {
//     if (frame_id == nullptr) return;
//     rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, frame_id);
// }

// void ImuAgent::createEntities(rcl_node_t *node, rclc_support_t *support) {
//     (void)support;
//     printf("[ImuAgent] Creating entities...\n");
//     rclc_publisher_init_default(
//         &imu_publisher_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/ddd/imu");
//     entities_active_ = 1;
// }

// void ImuAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support) {
//     (void)support;
//     if (entities_active_ > 0) {
//         rcl_publisher_fini(&imu_publisher_, node);
//     }
//     entities_active_ = 0;
// }

// uint ImuAgent::getCount() {
//     return entities_active_;
// }

// uint ImuAgent::getHandles() {
//     return 0;  // Keine Subscriptions
// }

// void ImuAgent::run() {
//     for (;;) {
//         if (!ensureInitialized()) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//             continue;
//         }

//         Vector3f accel{}, gyro{};
//         if (sensor_.readAcceleration(accel) && sensor_.readGyroscope(gyro)) {
//             populateMessage(accel, gyro);

//             if (uRosBridge::getInstance()->isSessionReady() && entities_active_ > 0) {
//                 int64_t timestamp = rmw_uros_epoch_nanos();
//                 imu_msg_.header.stamp.sec = timestamp / 1000000000;
//                 imu_msg_.header.stamp.nanosec = timestamp % 1000000000;
//                 uRosBridge::getInstance()->publish(&imu_publisher_, &imu_msg_, this, nullptr);
//             }
//         } else {
//             printf("[ImuAgent] Sensor read failed, re-initializing...\n");
//             initialized_ = false;
//         }

//         vTaskDelay(pdMS_TO_TICKS(publish_period_ms_));
//     }
// }

// configSTACK_DEPTH_TYPE ImuAgent::getMaxStackSize() {
//     return 1024;
// }

// bool ImuAgent::ensureInitialized() {
//     if (initialized_) return true;
//     initialized_ = sensor_.initialize();
//     if (!initialized_) {
//         printf("[ImuAgent] Initialization failed\n");
//     }
//     return initialized_;
// }

// void ImuAgent::populateMessage(const Vector3f &accel_g, const Vector3f &gyro_dps) {
//     imu_msg_.linear_acceleration.x = accel_g.x * kGtoMetersPerSecond2;
//     imu_msg_.linear_acceleration.y = accel_g.y * kGtoMetersPerSecond2;
//     imu_msg_.linear_acceleration.z = accel_g.z * kGtoMetersPerSecond2;

//     imu_msg_.angular_velocity.x = gyro_dps.x * kDegToRad;
//     imu_msg_.angular_velocity.y = gyro_dps.y * kDegToRad;
//     imu_msg_.angular_velocity.z = gyro_dps.z * kDegToRad;

//     // Orientierung wird nicht berechnet, daher auf "nicht vorhanden" setzen
//     imu_msg_.orientation.x = 0.0;
//     imu_msg_.orientation.y = 0.0;
//     imu_msg_.orientation.z = 0.0;
//     imu_msg_.orientation.w = 1.0;
// }

// }  // namespace application

#include "application/ImuAgent.h"

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "task.h"
#include "uRosBridge.h"

#include <cstdio>

namespace application {

using hal::hardware::Icm20948Simple;
using shared::Vector3f;

ImuAgent::ImuAgent(const hal::hardware::Icm20948Simple::Config& config) :
    config_(config), sensor_(config) {
    sensor_msgs__msg__Imu__init(&imu_msg_);
    // ... (restlicher Konstruktor-Code bleibt gleich) ...
    imu_msg_.orientation_covariance[0] = -1.0;  // Bedeutet: Orientierung wird nicht geliefert
}

ImuAgent::~ImuAgent() {
    sensor_msgs__msg__Imu__fini(&imu_msg_);
}

void ImuAgent::setFrameId(const char* frame_id) {
    if (frame_id == nullptr) return;
    rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, frame_id);
}

void ImuAgent::createEntities(rcl_node_t* node, rclc_support_t* support) {
    (void)support;
    rclc_publisher_init_default(
        &imu_publisher_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/ddd/imu");
    entities_active_ = 1;
}

void ImuAgent::destroyEntities(rcl_node_t* node, rclc_support_t* support) {
    (void)support;
    if (entities_active_ > 0) {
        rcl_publisher_fini(&imu_publisher_, node);
    }
    entities_active_ = 0;
}

uint ImuAgent::getCount() {
    return entities_active_;
}
uint ImuAgent::getHandles() {
    return 0;
}
void ImuAgent::addToExecutor(rclc_executor_t* executor) {
    (void)executor;
}

void ImuAgent::run() {
    printf("[ImuAgent] Task gestartet.\n");
    for (;;) {
        if (!ensureInitialized()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        Vector3f accel{}, gyro{};
        bool accel_ok = sensor_.readAcceleration(accel);
        bool gyro_ok = sensor_.readGyroscope(gyro);

        if (accel_ok && gyro_ok) {
            // *** HIER: DEBUG-AUSGABE HINZUGEFÜGT ***
            printf(
                "[IMU DEBUG] Accel(g): x=%.2f, y=%.2f, z=%.2f | Gyro(dps): x=%.2f, y=%.2f, "
                "z=%.2f\n",
                accel.x,
                accel.y,
                accel.z,
                gyro.x,
                gyro.y,
                gyro.z);

            if (uRosBridge::getInstance()->isSessionReady() && entities_active_ > 0) {
                populateMessage(accel, gyro);
                uRosBridge::getInstance()->publish(&imu_publisher_, &imu_msg_, this, nullptr);
            }
        } else {
            printf("[ImuAgent] Fehler beim Lesen der Sensordaten.\n");
            initialized_ = false;  // Erzwingt Neu-Initialisierung
        }

        vTaskDelay(pdMS_TO_TICKS(publish_period_ms_));
    }
}

configSTACK_DEPTH_TYPE ImuAgent::getMaxStackSize() {
    return 1024;
}

bool ImuAgent::ensureInitialized() {
    if (initialized_) return true;
    printf("[ImuAgent] Initialisiere Sensor...\n");
    initialized_ = sensor_.initialize();
    if (initialized_) {
        printf("[ImuAgent] Sensor erfolgreich initialisiert.\n");
    } else {
        printf("[ImuAgent] Sensor-Initialisierung fehlgeschlagen.\n");
    }
    return initialized_;
}

void ImuAgent::populateMessage(const Vector3f& accel_g, const Vector3f& gyro_dps) {
    constexpr float kGtoMetersPerSecond2 = 9.80665f;
    constexpr float kDegToRad = 3.1415926535 / 180.0f;

    int64_t timestamp = rmw_uros_epoch_nanos();
    imu_msg_.header.stamp.sec = timestamp / 1000000000;
    imu_msg_.header.stamp.nanosec = timestamp % 1000000000;

    imu_msg_.linear_acceleration.x = accel_g.x * kGtoMetersPerSecond2;
    imu_msg_.linear_acceleration.y = accel_g.y * kGtoMetersPerSecond2;
    imu_msg_.linear_acceleration.z = accel_g.z * kGtoMetersPerSecond2;

    imu_msg_.angular_velocity.x = gyro_dps.x * kDegToRad;
    imu_msg_.angular_velocity.y = gyro_dps.y * kDegToRad;
    imu_msg_.angular_velocity.z = gyro_dps.z * kDegToRad;

    // Wir liefern keine absolute Orientierung, daher bleibt dies eine Einheitsquaternion
    imu_msg_.orientation.x = 0.0;
    imu_msg_.orientation.y = 0.0;
    imu_msg_.orientation.z = 0.0;
    imu_msg_.orientation.w = 1.0;
}

}  // namespace application
