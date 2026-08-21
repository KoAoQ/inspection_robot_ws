#include <gtest/gtest.h>

#include "inspection_robot_base/base_fault_manager.hpp"
using namespace inspection_robot_base;
TEST(FaultManager, DisconnectClearsFreshnessAndReconnectWaitsForNewFeedback) {
  using namespace std::chrono_literals;
  BaseFaultManager f(500ms, 300ms);
  auto t = BaseFaultManager::Clock::now();
  f.notifySerialConnected(t);
  f.notifyMotionFeedback(t);
  f.notifyValidCommand(t);
  EXPECT_TRUE(f.canMove(t));
  f.notifySerialDisconnected();
  f.notifySerialConnected(t + 10ms);
  EXPECT_FALSE(f.canMove(t + 10ms));
  EXPECT_EQ(f.snapshot(t + 10ms).state, BaseState::kWaitingForFeedback);
  f.notifyMotionFeedback(t + 20ms);
  EXPECT_FALSE(f.canMove(t + 20ms));
}
TEST(FaultManager, FeedbackTimeoutLatchesAndResetRequiresFreshFeedback) {
  using namespace std::chrono_literals;
  BaseFaultManager f(500ms, 300ms);
  auto t = BaseFaultManager::Clock::now();
  f.notifySerialConnected(t);
  f.notifyMotionFeedback(t);
  f.notifyValidCommand(t);
  f.evaluate(t + 301ms);
  EXPECT_TRUE(f.faultLatched());
  EXPECT_EQ(f.fault(), FaultCode::kFeedbackTimeout);
  EXPECT_FALSE(f.resetIfHealthy(t + 301ms));
}
TEST(FaultManager, CommandTimeoutStopsButDoesNotLatch) {
  using namespace std::chrono_literals;
  BaseFaultManager f(100ms, 500ms);
  auto t = BaseFaultManager::Clock::now();
  f.notifySerialConnected(t);
  f.notifyMotionFeedback(t);
  f.notifyValidCommand(t);
  EXPECT_FALSE(f.canMove(t + 101ms));
  f.evaluate(t + 101ms);
  EXPECT_FALSE(f.faultLatched());
}
