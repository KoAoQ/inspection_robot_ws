#include "inspection_robot_base/base_io_worker.hpp"
#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>
#include "inspection_robot_base/charging_protocol.hpp"
#include "inspection_robot_base/device_control_protocol.hpp"
#include "inspection_robot_base/motion_protocol.hpp"
#include "inspection_robot_base/ultrasonic_protocol.hpp"
namespace inspection_robot_base {
BaseIoWorker::BaseIoWorker(std::unique_ptr<IByteTransport> t,std::shared_ptr<SharedControlState> c,std::shared_ptr<HardwareStateStore> h,BaseIoConfig cfg)
:transport_(std::move(t)),control_(std::move(c)),hardware_(std::move(h)),config_(cfg),fault_manager_(cfg.command_timeout,cfg.feedback_timeout),odometry_(cfg.odom_scale){}
BaseIoWorker::~BaseIoWorker(){stop();}
void BaseIoWorker::start(){if(running_.exchange(true))return;thread_=std::thread(&BaseIoWorker::run,this);} void BaseIoWorker::stop(){if(!running_.exchange(false))return;if(thread_.joinable())thread_.join();if(transport_->isOpen()){sendZero();transport_->close();}}
VelocityCommand BaseIoWorker::chooseOutput(const VelocityCommand& r,bool can_move){return can_move?r:VelocityCommand{};}
bool BaseIoWorker::finite(const VelocityCommand& c){return std::isfinite(c.vx)&&std::isfinite(c.vy)&&std::isfinite(c.wz);} VelocityCommand BaseIoWorker::clamp(const VelocityCommand& c)const{return {std::clamp(c.vx,-config_.max_vx,config_.max_vx),std::clamp(c.vy,-config_.max_vy,config_.max_vy),std::clamp(c.wz,-config_.max_wz,config_.max_wz)};}
bool BaseIoWorker::sendMotion(const VelocityCommand& c){auto f=MotionProtocol::encodeVelocity(c.vx,c.vy,c.wz,MotionCommandContext{0,config_.motion_reserved_byte});if(!f)return false;return transport_->write(f->data(),f->size());} bool BaseIoWorker::sendZero(){return sendMotion({});}
void BaseIoWorker::disconnectForFault(FaultCode reason){
  if(transport_->isOpen()) sendZero();
  transport_->close();
  router_.clear();
  latest_command_={};
  control_->invalidateCommand();
  seen_command_seq_=control_->snapshot().command_seq;
  fault_manager_.notifySerialDisconnected(reason);
  next_reconnect_=Clock::now()+config_.reconnect_period;
}
bool BaseIoWorker::connectAndHandshake(Clock::time_point now){
  if(!transport_->open()){next_reconnect_=now+config_.reconnect_period;return false;}
  transport_->flushInput();
  router_.clear();
  latest_command_={};
  control_->invalidateCommand();
  seen_command_seq_=control_->snapshot().command_seq;
  fault_manager_.notifySerialConnected(now);
  connected_since_=now;
  local_hw_.have_motion=false;
  local_hw_.have_ultrasonic=false;
  local_hw_.have_charging=false;
  if(!sendZero()){disconnectForFault(FaultCode::kSerialIo);return false;}if(config_.auto_enable_mcu_safety){auto f=DeviceControlProtocol::setMcuSafetyEnabled(true);if(!transport_->write(f.data(),f.size())){fault_manager_.notifyMcuSafetyConfigFailure();disconnectForFault(FaultCode::kMcuSafetyConfig);return false;}local_hw_.mcu_safety_requested=true;}return true;}
bool BaseIoWorker::processDeviceRequests(){auto q=control_->drainDeviceRequests();for(const auto& r:q){DeviceControlProtocol::Frame f{};switch(r.type){case DeviceRequestType::kRecharge:f=DeviceControlProtocol::setRechargeEnabled(r.enabled);break;case DeviceRequestType::kMcuSafety:if(!r.enabled&&!config_.allow_mcu_safety_disable)continue;f=DeviceControlProtocol::setMcuSafetyEnabled(r.enabled);local_hw_.mcu_safety_requested=r.enabled;break;case DeviceRequestType::kLight:f=DeviceControlProtocol::setLight(r.enabled,r.r,r.g,r.b);break;}if(!transport_->write(f.data(),f.size()))return false;}return true;}
void BaseIoWorker::processFrame(const ValidatedFrame& frame,Clock::time_point now){if(frame.type()==FrameType::kMotionFeedback){auto m=MotionProtocol::decode(frame);if(!m)return;local_hw_.motion=*m;local_hw_.have_motion=true;local_hw_.motion_time=now;++local_hw_.motion_seq;fault_manager_.notifyMotionFeedback(now);double dt=0.0;if(last_motion_integrate_.time_since_epoch().count()!=0)dt=std::chrono::duration<double>(now-last_motion_integrate_).count();last_motion_integrate_=now;local_hw_.odometry=odometry_.update(m->vx,m->vy,m->wz,dt);}else if(frame.type()==FrameType::kUltrasonic){auto u=UltrasonicProtocol::decode(frame);if(u){local_hw_.ultrasonic=*u;local_hw_.have_ultrasonic=true;local_hw_.ultrasonic_time=now;++local_hw_.ultrasonic_seq;}}else if(frame.type()==FrameType::kCharging){auto c=ChargingProtocol::decode(frame);if(c){local_hw_.charging=*c;local_hw_.have_charging=true;local_hw_.charging_time=now;++local_hw_.charging_seq;}}}
void BaseIoWorker::readAndProcess(Clock::time_point now){std::vector<std::uint8_t> bytes;if(!transport_->readAvailable(bytes,config_.max_read_bytes)){disconnectForFault(FaultCode::kSerialIo);return;}if(!bytes.empty())router_.pushBytes(bytes.data(),bytes.size());while(auto f=router_.popNext())processFrame(*f,now);}
void BaseIoWorker::refreshPublishedSnapshot(Clock::time_point now){local_hw_.router_stats=router_.stats();local_hw_.fault=fault_manager_.snapshot(now);local_hw_.serial_error=transport_->lastError();hardware_->update(local_hw_);}
void BaseIoWorker::run(){const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0/std::max(1.0,config_.loop_hz)));next_reconnect_=Clock::now();auto next=Clock::now();while(running_){auto now=Clock::now();auto ctl=control_->snapshot();fault_manager_.setEmergencyStop(ctl.emergency_stop);
    if(!transport_->isOpen()){if(now>=next_reconnect_)connectAndHandshake(now);refreshPublishedSnapshot(now);next+=period;std::this_thread::sleep_until(next);continue;}
    readAndProcess(now);if(!transport_->isOpen()){refreshPublishedSnapshot(now);next+=period;std::this_thread::sleep_until(next);continue;}
    if(ctl.reset_seq!=seen_reset_seq_){seen_reset_seq_=ctl.reset_seq;if(fault_manager_.resetIfHealthy(now)){control_->invalidateCommand();seen_command_seq_=control_->snapshot().command_seq;latest_command_={};}}
    if(ctl.command_seq!=seen_command_seq_){seen_command_seq_=ctl.command_seq;if(ctl.have_command){if(!finite(ctl.command)){fault_manager_.notifyInvalidCommand();latest_command_={};control_->invalidateCommand();seen_command_seq_=control_->snapshot().command_seq;}else{latest_command_=clamp(ctl.command);fault_manager_.notifyValidCommand(now);}}else{latest_command_={};fault_manager_.invalidateCommand();}}
    auto evaluation=fault_manager_.evaluate(now);
    if(evaluation.feedback_timeout_transition){disconnectForFault(FaultCode::kFeedbackTimeout);refreshPublishedSnapshot(now);next+=period;std::this_thread::sleep_until(next);continue;}
    if(!local_hw_.have_motion && connected_since_.time_since_epoch().count()!=0 &&
       (now-connected_since_)>config_.first_feedback_timeout){
      disconnectForFault(FaultCode::kFeedbackTimeout);refreshPublishedSnapshot(now);next+=period;std::this_thread::sleep_until(next);continue;
    }
    if(!processDeviceRequests()){disconnectForFault(FaultCode::kSerialIo);refreshPublishedSnapshot(now);next+=period;std::this_thread::sleep_until(next);continue;}
    auto out=chooseOutput(latest_command_,fault_manager_.canMove(now));
    if(!sendMotion(out)){disconnectForFault(FaultCode::kSerialIo);}
    refreshPublishedSnapshot(now);next+=period;auto after=Clock::now();if(next<after)next=after;std::this_thread::sleep_until(next);
  }}
}
