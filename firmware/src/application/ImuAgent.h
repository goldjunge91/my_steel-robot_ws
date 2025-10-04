#pragma once

#include "Agent.h"
#include "hal/hardware/Icm20948Simple.hpp"
#include "shared/Vector3f.hpp"
#include "uRosEntities.h"

extern "C" {
#include "rosidl_runtime_c/string_functions.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
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
    void populateMessage(const shared::Vector3f &accel_g, const shared::Vector3f &gyro_dps);

    hal::hardware::Icm20948Simple::Config config_;
    hal::hardware::Icm20948Simple sensor_;

    sensor_msgs__msg__Imu imu_msg_;
    rcl_publisher_t imu_publisher_;

    bool initialized_ = false;
    uint32_t publish_period_ms_ = 100;  // 10Hz
    uint entities_active_ = 0;
};

}  // namespace application
