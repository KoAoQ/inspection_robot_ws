#include "inspection_robot_base/serial_transport.hpp"
#include <algorithm>
#include <exception>
namespace inspection_robot_base {
SerialTransport::SerialTransport(SerialConfig c):config_(std::move(c)){}
void SerialTransport::setError(const std::string& v){std::lock_guard<std::mutex> l(error_mutex_);last_error_=v;}
std::string SerialTransport::lastError() const{std::lock_guard<std::mutex> l(error_mutex_);return last_error_;}
bool SerialTransport::open(){try{if(serial_.isOpen())return true;serial_.setPort(config_.port);serial_.setBaudrate(config_.baud_rate);auto timeout=serial::Timeout::simpleTimeout(config_.timeout_ms);serial_.setTimeout(timeout);serial_.open();if(!serial_.isOpen()){setError("serial port did not open");return false;}setError("");return true;}catch(const std::exception& e){setError(e.what());try{serial_.close();}catch(...){ }return false;}}
void SerialTransport::close(){try{if(serial_.isOpen())serial_.close();}catch(const std::exception& e){setError(e.what());}}
bool SerialTransport::isOpen() const{try{return serial_.isOpen();}catch(...){return false;}}
bool SerialTransport::flushInput(){try{if(!serial_.isOpen())return false;serial_.flushInput();return true;}catch(const std::exception& e){setError(e.what());return false;}}
bool SerialTransport::readAvailable(std::vector<std::uint8_t>& out,std::size_t max_bytes){out.clear();try{if(!serial_.isOpen())return false;auto n=std::min(serial_.available(),max_bytes);if(n==0)return true;out.resize(n);auto got=serial_.read(out.data(),n);out.resize(got);return true;}catch(const std::exception& e){setError(e.what());try{serial_.close();}catch(...){ }return false;}}
bool SerialTransport::write(const std::uint8_t* data,std::size_t size){try{if(!serial_.isOpen()){setError("write requested while serial closed");return false;}auto n=serial_.write(data,size);if(n!=size){setError("short serial write");serial_.close();return false;}return true;}catch(const std::exception& e){setError(e.what());try{serial_.close();}catch(...){ }return false;}}
}
