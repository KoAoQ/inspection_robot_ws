#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "inspection_robot_base/base_io_worker.hpp"
#include "inspection_robot_base/checksum.hpp"
using namespace inspection_robot_base;
using namespace std::chrono_literals;

class FakeTransport final : public IByteTransport {
public:
  bool open() override { open_=true; return true; }
  void close() override { open_=false; }
  bool isOpen() const override { return open_; }
  bool flushInput() override { return true; }
  bool readAvailable(std::vector<std::uint8_t> & out, std::size_t) override {
    out.clear(); if(!open_) return false;
    std::vector<std::uint8_t> f(24,0); f[0]=0x7B; f[22]=xorBcc(f.data(),22); f[23]=0x7D; out=f; return true;
  }
  bool write(const std::uint8_t * data,std::size_t size) override {
    if(!open_) return false; std::lock_guard<std::mutex> l(mu_); writes_.emplace_back(data,data+size); return true;
  }
  std::string lastError() const override { return {}; }
  std::vector<std::vector<std::uint8_t>> writes() const { std::lock_guard<std::mutex> l(mu_); return writes_; }
private:
  std::atomic<bool> open_{false}; mutable std::mutex mu_; std::vector<std::vector<std::uint8_t>> writes_;
};

static bool motionNonzero(const std::vector<std::uint8_t>& f){
  if(f.size()!=11 || f[0]!=0x7B || f[10]!=0x7D) return false;
  return f[3]||f[4]||f[5]||f[6]||f[7]||f[8];
}
int main(){
  auto raw=new FakeTransport();
  auto control=std::make_shared<SharedControlState>();
  auto hardware=std::make_shared<HardwareStateStore>();
  BaseIoConfig cfg; cfg.loop_hz=100.0; cfg.command_timeout=50ms; cfg.feedback_timeout=200ms; cfg.reconnect_period=20ms; cfg.auto_enable_mcu_safety=true;
  BaseIoWorker worker(std::unique_ptr<IByteTransport>(raw),control,hardware,cfg);
  worker.start();
  std::this_thread::sleep_for(40ms); // handshake + fresh feedback
  control->submitVelocity({0.2,0.0,0.0});
  std::this_thread::sleep_for(40ms);
  auto w1=raw->writes(); bool saw_nonzero=false; for(const auto& f:w1) saw_nonzero |= motionNonzero(f); assert(saw_nonzero);
  std::this_thread::sleep_for(90ms); // command timeout; feedback remains fresh
  auto w2=raw->writes(); assert(!w2.empty());
  // Last motion command must be zero; worker continuously writes zero when movement is not authorized.
  bool checked=false; for(auto it=w2.rbegin();it!=w2.rend();++it){if(it->size()==11 && (*it)[0]==0x7B && (*it)[10]==0x7D){assert(!motionNonzero(*it));checked=true;break;}} assert(checked);
  worker.stop();
  return 0;
}
