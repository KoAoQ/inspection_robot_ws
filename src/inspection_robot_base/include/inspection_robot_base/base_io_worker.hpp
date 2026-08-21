#pragma once
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include "inspection_robot_base/base_fault_manager.hpp"
#include "inspection_robot_base/byte_transport.hpp"
#include "inspection_robot_base/control_state.hpp"
#include "inspection_robot_base/hardware_state.hpp"
#include "inspection_robot_base/motion_protocol.hpp"
#include "inspection_robot_base/odometry_integrator.hpp"
#include "inspection_robot_base/serial_frame_router.hpp"
namespace inspection_robot_base {
struct BaseIoConfig {
  double loop_hz{50.0};
  std::chrono::milliseconds reconnect_period{1000};
  std::chrono::milliseconds command_timeout{500};
  std::chrono::milliseconds feedback_timeout{300};
  std::chrono::milliseconds first_feedback_timeout{1000};
  double max_vx{0.5}, max_vy{0.0}, max_wz{2.0};
  bool auto_enable_mcu_safety{true};
  bool allow_mcu_safety_disable{false};
  std::uint8_t motion_reserved_byte{0};
  std::size_t max_read_bytes{512};
  OdometryScale odom_scale{};
};
class BaseIoWorker {
public:
  BaseIoWorker(std::unique_ptr<IByteTransport> transport,
               std::shared_ptr<SharedControlState> control,
               std::shared_ptr<HardwareStateStore> hardware,
               BaseIoConfig config);
  ~BaseIoWorker();
  void start();
  void stop();
  bool running() const { return running_; }
  // Exposed as a pure policy helper for unit tests and design review.
  static VelocityCommand chooseOutput(const VelocityCommand & requested, bool can_move);
private:
  using Clock = std::chrono::steady_clock;
  void run();
  bool connectAndHandshake(Clock::time_point now);
  void disconnectForFault(FaultCode reason);
  bool sendMotion(const VelocityCommand & command);
  bool sendZero();
  bool processDeviceRequests();
  void readAndProcess(Clock::time_point now);
  void processFrame(const ValidatedFrame & frame, Clock::time_point now);
  static bool finite(const VelocityCommand & command);
  VelocityCommand clamp(const VelocityCommand & command) const;
  void refreshPublishedSnapshot(Clock::time_point now);
  std::unique_ptr<IByteTransport> transport_;
  std::shared_ptr<SharedControlState> control_;
  std::shared_ptr<HardwareStateStore> hardware_;
  BaseIoConfig config_;
  BaseFaultManager fault_manager_;
  SerialFrameRouter router_;
  OdometryIntegrator odometry_;
  HardwareSnapshot local_hw_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  Clock::time_point next_reconnect_{}, connected_since_{};
  Clock::time_point last_motion_integrate_{};
  std::uint64_t seen_command_seq_{0}, seen_reset_seq_{0};
  VelocityCommand latest_command_{};
};
}
