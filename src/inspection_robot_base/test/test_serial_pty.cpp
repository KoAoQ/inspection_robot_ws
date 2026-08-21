#include <gtest/gtest.h>
#ifdef __linux__
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include "inspection_robot_base/serial_transport.hpp"
using namespace inspection_robot_base;
TEST(SerialTransport, PseudoTerminalRoundTrip) {
  int master_fd=-1, slave_fd=-1; char slave_name[256]{};
  ASSERT_EQ(openpty(&master_fd,&slave_fd,slave_name,nullptr,nullptr),0);
  close(slave_fd);
  SerialTransport transport(SerialConfig{slave_name,115200,20});
  ASSERT_TRUE(transport.open()) << transport.lastError();
  const std::array<std::uint8_t,4> inbound{{0x7B,0x01,0x02,0x7D}};
  ASSERT_EQ(::write(master_fd,inbound.data(),inbound.size()),static_cast<ssize_t>(inbound.size()));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::vector<std::uint8_t> read;
  ASSERT_TRUE(transport.readAvailable(read,64)) << transport.lastError();
  ASSERT_EQ(read.size(),inbound.size());
  EXPECT_EQ(read[0],0x7B);
  const std::array<std::uint8_t,3> outbound{{0xAA,0xBB,0xCC}};
  ASSERT_TRUE(transport.write(outbound.data(),outbound.size())) << transport.lastError();
  std::array<std::uint8_t,3> received{};
  ASSERT_EQ(::read(master_fd,received.data(),received.size()),static_cast<ssize_t>(received.size()));
  EXPECT_EQ(received,outbound);
  transport.close(); close(master_fd);
}
#else
TEST(SerialTransport, PseudoTerminalUnsupported) { GTEST_SKIP(); }
#endif
