#include <gtest/gtest.h>

// Provide minimal stubs/mocks for USB interface and control loop functions used
// by command_processor.cpp. The production implementations live in
// src/middleware/usb_interface.cpp and src/middleware/control_loop.cpp; for
// host-side tests we override the functions with test doubles here.

extern "C" {
// command_processor expects these C functions to exist:
bool usb_interface_get_command(char *out_buf, size_t bufsize);
void usb_interface_send_response(const char *resp);
void control_loop_set_motor_speeds_int(int fl, int fr, int rl, int rr);
}

#include "middleware/command_processor.cpp" // include implementation directly for host test

static std::string last_response;
static std::string queued_cmd;

extern "C" bool usb_interface_get_command(char *out_buf, size_t bufsize) {
  if (queued_cmd.empty()) return false;
  strncpy(out_buf, queued_cmd.c_str(), bufsize - 1);
  out_buf[bufsize - 1] = '\0';
  // consume command
  queued_cmd.clear();
  return true;
}

extern "C" void usb_interface_send_response(const char *resp) {
  last_response = resp ? resp : "";
}

static int last_fl, last_fr, last_rl, last_rr;
extern "C" void control_loop_set_motor_speeds_int(int fl, int fr, int rl, int rr) {
  last_fl = fl; last_fr = fr; last_rl = rl; last_rr = rr;
}

TEST(CommandProcessorHost, ParseAndExecuteMCommand) {
  queued_cmd = std::string("M FL:10 FR:20 RL:30 RR:40");
  last_response.clear();
  command_processor_task();
  EXPECT_EQ(last_response, "OK");
  EXPECT_EQ(last_fl, 10);
  EXPECT_EQ(last_fr, 20);
  EXPECT_EQ(last_rl, 30);
  EXPECT_EQ(last_rr, 40);
}

TEST(CommandProcessorHost, StopCommandStopsMotors) {
  queued_cmd = std::string("STOP");
  last_response.clear();
  // set non-zero initial values
  last_fl = 1; last_fr = 2; last_rl = 3; last_rr = 4;
  command_processor_task();
  EXPECT_EQ(last_response, "STOPPED");
  EXPECT_EQ(last_fl, 0);
  EXPECT_EQ(last_fr, 0);
  EXPECT_EQ(last_rl, 0);
  EXPECT_EQ(last_rr, 0);
}

TEST(CommandProcessorHost, InvalidCommandReturnsError) {
  queued_cmd = std::string("UNKNOWN_CMD");
  last_response.clear();
  command_processor_task();
  EXPECT_EQ(last_response, "ERROR: INVALID_CMD");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
