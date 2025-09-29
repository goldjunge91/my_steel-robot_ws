#include <gtest/gtest.h>

// We include the header to get the declarations, then include the implementation
// file directly to test the helper initialization functions on host.

#include "middleware/micro_ros_cfg.h"
#ifndef HOST_BUILD
#include "middleware/micro_ros_cfg.cpp"
#endif

TEST(MicroRosCfgHost, MotorsCmdMsgInitSetsSizeAndCapacity) {
  std_msgs__msg__Float32MultiArray msg;
  MotorsCmdMsgInit(&msg);
  EXPECT_EQ(msg.data.size, MOT_CMD_MSG_LEN);
  EXPECT_EQ(msg.data.capacity, MOT_CMD_MSG_LEN);
}

TEST(MicroRosCfgHost, MotorsResponseMsgInitNamesAndSizes) {
  sensor_msgs__msg__JointState msg;
  MotorsResponseMsgInit(&msg);
  EXPECT_EQ(msg.position.size, MOT_RESP_MSG_LEN);
  EXPECT_EQ(msg.velocity.size, MOT_RESP_MSG_LEN);
  EXPECT_EQ(msg.effort.size, MOT_RESP_MSG_LEN);
  EXPECT_EQ(msg.name.size, MOT_RESP_MSG_LEN);
  // Check that frame id points to string "motors_response"
  EXPECT_STREQ(msg.header.frame_id.data, "motors_response");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
