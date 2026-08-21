#include "inspection_robot_base/base_fault_manager.hpp"
namespace inspection_robot_base {
BaseFaultManager::BaseFaultManager(std::chrono::milliseconds c,std::chrono::milliseconds f):command_timeout_(c),feedback_timeout_(f){}
void BaseFaultManager::clearFreshness(){have_feedback_=false;have_command_=false;last_feedback_={};last_command_={};}
void BaseFaultManager::notifySerialConnected(Clock::time_point){serial_open_=true;clearFreshness();if(!faultLatched())state_=BaseState::kWaitingForFeedback;}
void BaseFaultManager::notifySerialDisconnected(FaultCode reason){serial_open_=false;clearFreshness();if(reason!=FaultCode::kNone)latch(reason);else if(!faultLatched())state_=BaseState::kDisconnected;}
void BaseFaultManager::notifyMotionFeedback(Clock::time_point now){have_feedback_=true;last_feedback_=now;if(serial_open_&&!faultLatched())state_=BaseState::kReady;}
void BaseFaultManager::notifyValidCommand(Clock::time_point now){have_command_=true;last_command_=now;}
void BaseFaultManager::invalidateCommand(){have_command_=false;last_command_={};}
void BaseFaultManager::notifyInvalidCommand(){latch(FaultCode::kInvalidCommand);} void BaseFaultManager::notifySerialIoFailure(){notifySerialDisconnected(FaultCode::kSerialIo);} void BaseFaultManager::notifyMcuSafetyConfigFailure(){latch(FaultCode::kMcuSafetyConfig);}
void BaseFaultManager::setEmergencyStop(bool a){emergency_stop_=a;if(a)latch(FaultCode::kEmergencyStop);}
bool BaseFaultManager::feedbackHealthy(Clock::time_point now)const{return serial_open_&&have_feedback_&&(now-last_feedback_)<=feedback_timeout_;}
bool BaseFaultManager::commandHealthy(Clock::time_point now)const{return have_command_&&(now-last_command_)<=command_timeout_;}
EvaluationResult BaseFaultManager::evaluate(Clock::time_point now){
  EvaluationResult result;
  if(faultLatched()) return result;
  if(!serial_open_){state_=BaseState::kDisconnected;return result;}
  if(!have_feedback_){state_=BaseState::kWaitingForFeedback;return result;}
  if(!feedbackHealthy(now)){latch(FaultCode::kFeedbackTimeout);result.feedback_timeout_transition=true;return result;}
  state_=BaseState::kReady;return result;
}
bool BaseFaultManager::resetIfHealthy(Clock::time_point now){if(!serial_open_||emergency_stop_||!feedbackHealthy(now))return false;fault_=FaultCode::kNone;state_=BaseState::kReady;invalidateCommand();return true;}
bool BaseFaultManager::canMove(Clock::time_point now)const{return state_==BaseState::kReady&&!emergency_stop_&&feedbackHealthy(now)&&commandHealthy(now);}
FaultSnapshot BaseFaultManager::snapshot(Clock::time_point now)const{return FaultSnapshot{state_,fault_,serial_open_,feedbackHealthy(now),commandHealthy(now),emergency_stop_};}
void BaseFaultManager::latch(FaultCode f){if(!faultLatched())fault_=f;state_=BaseState::kFaultLatched;}
}
