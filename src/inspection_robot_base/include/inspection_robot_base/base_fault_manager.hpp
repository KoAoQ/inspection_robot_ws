#pragma once
#include <chrono>
#include <cstdint>
namespace inspection_robot_base {
enum class BaseState : std::uint8_t { kDisconnected=0, kWaitingForFeedback=1, kReady=2, kFaultLatched=3 };
enum class FaultCode : std::uint16_t { kNone=0, kInvalidCommand=1, kFeedbackTimeout=2, kSerialIo=3, kEmergencyStop=4, kMcuSafetyConfig=5 };
struct EvaluationResult { bool feedback_timeout_transition{false}; };
struct FaultSnapshot {
  BaseState state{BaseState::kDisconnected};
  FaultCode fault{FaultCode::kNone};
  bool serial_open{false}, feedback_healthy{false}, command_healthy{false}, emergency_stop{false};
};
class BaseFaultManager {
public:
  using Clock = std::chrono::steady_clock;
  BaseFaultManager(std::chrono::milliseconds command_timeout, std::chrono::milliseconds feedback_timeout);
  void notifySerialConnected(Clock::time_point now);
  void notifySerialDisconnected(FaultCode reason = FaultCode::kNone);
  void notifyMotionFeedback(Clock::time_point now);
  void notifyValidCommand(Clock::time_point now);
  void invalidateCommand();
  void notifyInvalidCommand();
  void notifySerialIoFailure();
  void notifyMcuSafetyConfigFailure();
  void setEmergencyStop(bool active);
  EvaluationResult evaluate(Clock::time_point now);
  bool resetIfHealthy(Clock::time_point now);
  bool canMove(Clock::time_point now) const;
  bool feedbackHealthy(Clock::time_point now) const;
  bool commandHealthy(Clock::time_point now) const;
  bool faultLatched() const { return state_ == BaseState::kFaultLatched; }
  FaultCode fault() const { return fault_; }
  FaultSnapshot snapshot(Clock::time_point now) const;
private:
  void clearFreshness();
  void latch(FaultCode fault);
  std::chrono::milliseconds command_timeout_, feedback_timeout_;
  BaseState state_{BaseState::kDisconnected};
  FaultCode fault_{FaultCode::kNone};
  bool serial_open_{false}, have_feedback_{false}, have_command_{false}, emergency_stop_{false};
  Clock::time_point last_feedback_{}, last_command_{};
};
}
