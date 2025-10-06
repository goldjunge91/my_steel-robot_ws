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

namespace application
{

using hal::hardware::Icm20948Simple;
using shared::Vector3f;

ImuAgent::ImuAgent(const hal::hardware::Icm20948Simple::Config& config) : config_(config), sensor_(config)
{
  sensor_msgs__msg__Imu__init(&imu_msg_);

  // Orientation: not provided (set first element to -1.0 per ROS2 convention)
  imu_msg_.orientation_covariance[0] = -1.0;
  for (size_t i = 1; i < 9; ++i)
  {
    imu_msg_.orientation_covariance[i] = 0.0;
  }

  // Angular velocity covariance (rad/s)² - Calibrated from real sensor data
  // NOTE: X-axis shows higher noise (0.034), possibly due to mechanical stress or vibration
  imu_msg_.angular_velocity_covariance[0] = 0.033653;  // X variance (calibrated)
  imu_msg_.angular_velocity_covariance[4] = 0.002124;  // Y variance (calibrated)
  imu_msg_.angular_velocity_covariance[8] = 0.000277;  // Z variance (calibrated)
  for (size_t i = 0; i < 9; ++i)
  {
    if (i != 0 && i != 4 && i != 8)
    {
      imu_msg_.angular_velocity_covariance[i] = 0.0;  // Off-diagonal = 0
    }
  }

  // Linear acceleration covariance (m/s²)² - Calibrated from real sensor data
  // NOTE: Y-axis shows abnormally high noise (3.71), investigate mounting or connection
  imu_msg_.linear_acceleration_covariance[0] = 0.482400;  // X variance (calibrated)
  imu_msg_.linear_acceleration_covariance[4] = 3.712792;  // Y variance (calibrated - HIGH!)
  imu_msg_.linear_acceleration_covariance[8] = 0.246898;  // Z variance (calibrated)
  for (size_t i = 0; i < 9; ++i)
  {
    if (i != 0 && i != 4 && i != 8)
    {
      imu_msg_.linear_acceleration_covariance[i] = 0.0;  // Off-diagonal = 0
    }
  }
}

ImuAgent::~ImuAgent()
{
  sensor_msgs__msg__Imu__fini(&imu_msg_);
}

void ImuAgent::setFrameId(const char* frame_id)
{
  if (frame_id == nullptr)
    return;
  rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, frame_id);
}

void ImuAgent::createEntities(rcl_node_t* node, rclc_support_t* support)
{
  (void)support;
  printf("[ImuAgent] Creating entities...\n");
  rclc_publisher_init_default(&imu_publisher_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
  entities_active_ = 1;
  printf("[ImuAgent]   ✅ Publisher: imu/data_raw (sensor_msgs/Imu)\n");
}

void ImuAgent::destroyEntities(rcl_node_t* node, rclc_support_t* support)
{
  (void)support;
  printf("[ImuAgent] Destroying entities...\n");
  if (entities_active_ > 0)
  {
    rcl_publisher_fini(&imu_publisher_, node);
    printf("[ImuAgent]   ✅ Publisher destroyed\n");
  }
  entities_active_ = 0;
}

uint ImuAgent::getCount()
{
  return entities_active_;
}
uint ImuAgent::getHandles()
{
  return 0;
}
void ImuAgent::addToExecutor(rclc_executor_t* executor)
{
  (void)executor;
}

void ImuAgent::run()
{
  printf("[ImuAgent] ✅ Task started\n");
  static uint32_t sample_count = 0;
  static uint32_t publish_count = 0;
  static uint32_t error_count = 0;

  for (;;)
  {
    if (!ensureInitialized())
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    Vector3f accel{}, gyro{};
    bool accel_ok = sensor_.readAcceleration(accel);
    bool gyro_ok = sensor_.readGyroscope(gyro);

    if (accel_ok && gyro_ok)
    {
      sample_count++;

      // Check if all values are exactly zero (indicates sensor problem)
      bool all_zero =
          (accel.x == 0.0f && accel.y == 0.0f && accel.z == 0.0f && gyro.x == 0.0f && gyro.y == 0.0f && gyro.z == 0.0f);

      // Only log every 50th sample to reduce spam, OR if all values are zero (error condition)
      if ((sample_count % 50) == 0 || all_zero)
      {
        printf("[ImuAgent] Sample #%u: Accel(g): x=%.2f, y=%.2f, z=%.2f | Gyro(dps): x=%.2f, y=%.2f, z=%.2f\n",
               (unsigned)sample_count, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

        if (all_zero && sample_count > 100)
        {
          printf("[ImuAgent] ⚠️  WARNING: All values are zero after %u samples - sensor may not be working!\n",
                 (unsigned)sample_count);
        }
      }

      if (uRosBridge::getInstance()->isSessionReady() && entities_active_ > 0)
      {
        populateMessage(accel, gyro);
        if (uRosBridge::getInstance()->publish(&imu_publisher_, &imu_msg_, this, nullptr))
        {
          publish_count++;
          if ((publish_count % 100) == 0)
          {
            printf("[ImuAgent] ✅ Published %u messages\n", (unsigned)publish_count);
          }
        }
      }
    }
    else
    {
      error_count++;
      printf("[ImuAgent] ❌ Error #%u: Sensor read failed (accel=%d, gyro=%d)\n", (unsigned)error_count, accel_ok,
             gyro_ok);
      initialized_ = false;  // Force re-initialization
    }

    vTaskDelay(pdMS_TO_TICKS(publish_period_ms_));
  }
}

configSTACK_DEPTH_TYPE ImuAgent::getMaxStackSize()
{
  return 1024;
}

bool ImuAgent::ensureInitialized()
{
  if (initialized_)
    return true;

  // Rate-limit initialization attempts: only try every 10 seconds after first failure
  static uint32_t last_attempt_ms = 0;
  static uint32_t attempt_count = 0;
  uint32_t now_ms = to_ms_since_boot(get_absolute_time());

  // First attempt or enough time has passed
  if (attempt_count == 0 || (now_ms - last_attempt_ms) >= 10000)
  {
    printf("[ImuAgent] 🔧 Initializing ICM-20948 sensor (attempt #%u)...\n", ++attempt_count);
    last_attempt_ms = now_ms;

    initialized_ = sensor_.initialize();
    if (initialized_)
    {
      printf("[ImuAgent] ✅ Sensor initialized successfully after %u attempts\n", attempt_count);
      attempt_count = 0;  // Reset counter on success
    }
    else
    {
      printf("[ImuAgent] ❌ Sensor initialization FAILED (will retry in 10 seconds)\n");
    }
  }

  return initialized_;
}

void ImuAgent::populateMessage(const Vector3f& accel_g, const Vector3f& gyro_dps)
{
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
