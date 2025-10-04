#pragma once

#include "Agent.h"
#include "hal/hardware/vl6180x/vl6180x_driver.hpp"
#include "uRosEntities.h"

extern "C" {
#include "rosidl_runtime_c/string_functions.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
#include <sensor_msgs/msg/illuminance.h>
#include <sensor_msgs/msg/range.h>
}

namespace application {

class Vl6180xAgent : public Agent, public uRosEntities {
public:
    Vl6180xAgent();
    explicit Vl6180xAgent(const hal::hardware::Vl6180x::Config& config);
    ~Vl6180xAgent() override;

    void setRangeFrameId(const char* frame_id);
    void setIlluminanceFrameId(const char* frame_id);

    // uRosEntities interface
    void createEntities(rcl_node_t* node, rclc_support_t* support) override;
    void destroyEntities(rcl_node_t* node, rclc_support_t* support) override;
    uint getCount() override;

    // --- NEU: Fehlende Funktionen hinzugefügt ---
    uint getHandles() override;
    void addToExecutor(rclc_executor_t* executor) override;

protected:
    void run() override;
    configSTACK_DEPTH_TYPE getMaxStackSize() override;

private:
    bool ensureInitialized();

    hal::hardware::Vl6180x::Config config_;
    hal::hardware::Vl6180x sensor_;

    sensor_msgs__msg__Range range_msg_;
    sensor_msgs__msg__Illuminance illuminance_msg_;
    rcl_publisher_t range_publisher_;
    rcl_publisher_t illuminance_publisher_;

    bool initialized_ = false;
    uint32_t publish_period_ms_ = 200;
    uint entities_active_ = 0;
};

}  // namespace application
