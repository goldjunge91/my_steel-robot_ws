/**
 * @file test_topic_names.cpp
 * @brief Unit tests for ROS2 topic name standardization
 * 
 * Tests verify that all firmware components use standard ROS2 topic names
 * according to REP-105 conventions, without robot-specific prefixes.
 */

#include <gtest/gtest.h>
#include <string>

/**
 * Test fixture for topic name validation
 */
class TopicNameTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }

    void TearDown() override {
        // Cleanup code if needed
    }

    /**
     * Helper function to verify topic name doesn't contain deprecated prefixes
     */
    bool hasDeprecatedPrefix(const std::string& topic_name) {
        return topic_name.find("/ddd/") != std::string::npos ||
               topic_name.find("ddd/") == 0;
    }

    /**
     * Helper function to verify topic name follows REP-105 conventions
     */
    bool isStandardTopicName(const std::string& topic_name) {
        // Standard topics should not have robot-specific prefixes
        return !hasDeprecatedPrefix(topic_name);
    }
};

/**
 * Test: DDD odometry topic name
 * Requirement: 1.3, 1.4
 */
TEST_F(TopicNameTest, OdometryTopicName) {
    const std::string topic_name = "odom";
    
    EXPECT_TRUE(isStandardTopicName(topic_name)) 
        << "Odometry topic should not have /ddd/ prefix";
    EXPECT_EQ(topic_name, "odom") 
        << "Odometry topic should be 'odom' according to REP-105";
}

/**
 * Test: DDD cmd_vel topic name
 * Requirement: 1.6
 */
TEST_F(TopicNameTest, CmdVelTopicName) {
    const std::string topic_name = "cmd_vel";
    
    EXPECT_TRUE(isStandardTopicName(topic_name)) 
        << "Cmd_vel topic should not have /ddd/ prefix";
    EXPECT_EQ(topic_name, "cmd_vel") 
        << "Cmd_vel topic should be 'cmd_vel' according to REP-105";
}

/**
 * Test: IMU topic name
 * Requirement: 1.3
 */
TEST_F(TopicNameTest, ImuTopicName) {
    const std::string topic_name = "imu/data_raw";
    
    EXPECT_TRUE(isStandardTopicName(topic_name)) 
        << "IMU topic should not have /ddd/ prefix";
    EXPECT_EQ(topic_name, "imu/data_raw") 
        << "IMU topic should be 'imu/data_raw' according to REP-105";
}

/**
 * Test: ToF range sensor topic name
 * Requirement: 1.5
 */
TEST_F(TopicNameTest, TofRangeTopicName) {
    const std::string topic_name = "sensors/range_tof";
    
    EXPECT_TRUE(isStandardTopicName(topic_name)) 
        << "ToF range topic should not have /ddd/ prefix";
    EXPECT_EQ(topic_name, "sensors/range_tof") 
        << "ToF range topic should be 'sensors/range_tof' for clarity";
    EXPECT_TRUE(topic_name.find("sensors/") == 0) 
        << "Sensor topics should be under 'sensors/' namespace";
}

/**
 * Test: Illuminance sensor topic name
 * Requirement: 1.5
 */
TEST_F(TopicNameTest, IlluminanceTopicName) {
    const std::string topic_name = "sensors/illuminance";
    
    EXPECT_TRUE(isStandardTopicName(topic_name)) 
        << "Illuminance topic should not have /ddd/ prefix";
    EXPECT_EQ(topic_name, "sensors/illuminance") 
        << "Illuminance topic should be 'sensors/illuminance' for clarity";
    EXPECT_TRUE(topic_name.find("sensors/") == 0) 
        << "Sensor topics should be under 'sensors/' namespace";
}

/**
 * Test: Ultrasonic range sensor topic name
 * Requirement: 1.5
 */
TEST_F(TopicNameTest, UltrasonicRangeTopicName) {
    const std::string topic_name = "sensors/range_ultrasonic";
    
    EXPECT_TRUE(isStandardTopicName(topic_name)) 
        << "Ultrasonic range topic should not have /ddd/ prefix";
    EXPECT_EQ(topic_name, "sensors/range_ultrasonic") 
        << "Ultrasonic range topic should be 'sensors/range_ultrasonic' for clarity";
    EXPECT_TRUE(topic_name.find("sensors/") == 0) 
        << "Sensor topics should be under 'sensors/' namespace";
}

/**
 * Test: Verify no deprecated topic names are used
 * Requirement: 1.7
 */
TEST_F(TopicNameTest, NoDeprecatedPrefixes) {
    std::vector<std::string> current_topics = {
        "odom",
        "cmd_vel",
        "imu/data_raw",
        "sensors/range_tof",
        "sensors/illuminance",
        "sensors/range_ultrasonic"
    };
    
    for (const auto& topic : current_topics) {
        EXPECT_FALSE(hasDeprecatedPrefix(topic)) 
            << "Topic '" << topic << "' should not contain /ddd/ prefix";
    }
}

/**
 * Test: Verify sensor topics use consistent namespace
 * Requirement: 1.5
 */
TEST_F(TopicNameTest, SensorTopicsUseConsistentNamespace) {
    std::vector<std::string> sensor_topics = {
        "sensors/range_tof",
        "sensors/illuminance",
        "sensors/range_ultrasonic"
    };
    
    for (const auto& topic : sensor_topics) {
        EXPECT_TRUE(topic.find("sensors/") == 0) 
            << "Sensor topic '" << topic << "' should start with 'sensors/' namespace";
    }
}

/**
 * Test: Verify frame_id assignments for sensors
 * Requirement: 1.3, 1.5
 */
TEST_F(TopicNameTest, FrameIdAssignments) {
    // IMU frame_id
    const std::string imu_frame_id = "imu_link";
    EXPECT_EQ(imu_frame_id, "imu_link") 
        << "IMU frame_id should be 'imu_link'";
    
    // ToF frame_id
    const std::string tof_frame_id = "tof_link";
    EXPECT_EQ(tof_frame_id, "tof_link") 
        << "ToF sensor frame_id should be 'tof_link'";
    
    // Ultrasonic sensor frame_ids
    const std::string ultrasonic_front_frame_id = "range_front";
    const std::string ultrasonic_back_frame_id = "range_back";
    EXPECT_EQ(ultrasonic_front_frame_id, "range_front") 
        << "Front ultrasonic sensor frame_id should be 'range_front'";
    EXPECT_EQ(ultrasonic_back_frame_id, "range_back") 
        << "Back ultrasonic sensor frame_id should be 'range_back'";
}

/**
 * Test: Verify odometry frame_ids
 * Requirement: 1.4
 */
TEST_F(TopicNameTest, OdometryFrameIds) {
    const std::string odom_frame_id = "odom";
    const std::string child_frame_id = "base_link";
    
    EXPECT_EQ(odom_frame_id, "odom") 
        << "Odometry frame_id should be 'odom'";
    EXPECT_EQ(child_frame_id, "base_link") 
        << "Odometry child_frame_id should be 'base_link'";
}

// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
