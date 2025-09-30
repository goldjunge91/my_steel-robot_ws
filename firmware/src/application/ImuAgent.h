#pragma once

#include "Agent.h"
#include "uRosEntities.h"
#include "hal/hardware/Icm20948Simple.hpp"
#include "shared/Vector3f.hpp"

#include "FreeRTOS.h"
#include "task.h"

extern "C" {
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/time_sync.h>
#include "rosidl_runtime_c/string_functions.h"
}

namespace application {

class ImuAgent : public Agent, public uRosEntities {
public:
    ImuAgent();
    explicit ImuAgent(const hal::hardware::Icm20948Simple::Config &config);
    ~ImuAgent() override;

    void setFrameId(const char *frame_id);

    void createEntities(rcl_node_t *node, rclc_support_t *support) override;
    void destroyEntities(rcl_node_t *node, rclc_support_t *support) override;
    uint getCount() override;
    uint getHandles() override;
    void addToExecutor(rclc_executor_t *executor) override;

protected:
    void run() override;
    configSTACK_DEPTH_TYPE getMaxStackSize() override;

private:
    bool ensureInitialized();
    void populateMessage(const shared::Vector3f &accel_g,
                         const shared::Vector3f &gyro_dps,
                         float temperature_c);

    hal::hardware::Icm20948Simple::Config config_;
    hal::hardware::Icm20948Simple sensor_;

    sensor_msgs__msg__Imu imu_msg_;
    rcl_publisher_t imu_publisher_;

    bool initialized_ = false;
    uint32_t publish_period_ms_ = 20;
    uint entities_active_ = 0;
    bool frame_id_set_ = false;
};

}  // namespace application
