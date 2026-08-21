#include <gtest/gtest.h>

#include <limits>

#include "inspection_robot_base/base_io_worker.hpp"
using namespace inspection_robot_base;
TEST(IoPolicy, SendsZeroWheneverMovementNotAllowed) {
  VelocityCommand r{0.2, 0.0, 0.5};
  auto z = BaseIoWorker::chooseOutput(r, false);
  EXPECT_DOUBLE_EQ(z.vx, 0.0);
  EXPECT_DOUBLE_EQ(z.vy, 0.0);
  EXPECT_DOUBLE_EQ(z.wz, 0.0);
  auto p = BaseIoWorker::chooseOutput(r, true);
  EXPECT_DOUBLE_EQ(p.vx, 0.2);
}
