#include "inspection_robot_base/base_driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include "inspection_robot_base/serial_transport.hpp"
namespace inspection_robot_base {
namespace {
geometry_msgs::msg::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
std::uint8_t toByte(float v) {
  return static_cast<std::uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f);
}
}  // namespace
BaseDriverNode::BaseDriverNode(const rclcpp::NodeOptions& options)
    : Node("inspection_robot_base", options) {
  auto port = declare_parameter<std::string>("serial.port", "/dev/inspection_robot_controller");
  auto baud = declare_parameter<int>("serial.baud_rate", 115200);
  auto timeout = declare_parameter<int>("serial.timeout_ms", 20);
  BaseIoConfig cfg;
  cfg.loop_hz = declare_parameter<double>("io.loop_hz", 50.0);
  cfg.reconnect_period =
      std::chrono::milliseconds(declare_parameter<int>("serial.reconnect_period_ms", 1000));
  cfg.command_timeout =
      std::chrono::milliseconds(declare_parameter<int>("safety.command_timeout_ms", 500));
  cfg.feedback_timeout =
      std::chrono::milliseconds(declare_parameter<int>("safety.feedback_timeout_ms", 300));
  cfg.first_feedback_timeout =
      std::chrono::milliseconds(declare_parameter<int>("safety.first_feedback_timeout_ms", 1000));
  cfg.max_vx = declare_parameter<double>("limits.max_vx", 0.5);
  cfg.max_vy = declare_parameter<double>("limits.max_vy", 0.0);
  cfg.max_wz = declare_parameter<double>("limits.max_wz", 2.0);
  cfg.auto_enable_mcu_safety = declare_parameter<bool>("mcu.auto_enable_safety", true);
  cfg.allow_mcu_safety_disable = declare_parameter<bool>("mcu.allow_safety_disable", false);
  allow_mcu_safety_disable_ = cfg.allow_mcu_safety_disable;
  cfg.motion_reserved_byte =
      static_cast<std::uint8_t>(declare_parameter<int>("mcu.motion_reserved_byte", 0));
  cfg.odom_scale.x = declare_parameter<double>("odom.scale_x", 1.0);
  cfg.odom_scale.y = declare_parameter<double>("odom.scale_y", 1.0);
  cfg.odom_scale.yaw_positive = declare_parameter<double>("odom.scale_yaw_positive", 1.0);
  cfg.odom_scale.yaw_negative = declare_parameter<double>("odom.scale_yaw_negative", 1.0);
  odom_frame_ = declare_parameter<std::string>("frames.odom", "odom");
  base_frame_ = declare_parameter<std::string>("frames.base", "base_footprint");
  imu_frame_ = declare_parameter<std::string>("frames.imu", "gyro_link");
  ultrasonic_frames_ = {"ultrasonic_a_link",
                        "ultrasonic_b_link",
                        "ultrasonic_c_link",
                        "ultrasonic_d_link",
                        "ultrasonic_e_link",
                        "ultrasonic_f_link"};
  ultrasonic_fov_ = declare_parameter<double>("ultrasonic.field_of_view", 0.52);
  ultrasonic_min_ = declare_parameter<double>("ultrasonic.min_range", 0.02);
  ultrasonic_max_ = declare_parameter<double>("ultrasonic.max_range", 5.3);
  control_ = std::make_shared<SharedControlState>();
  hardware_ = std::make_shared<HardwareStateStore>();
  auto transport = std::make_unique<SerialTransport>(
      SerialConfig{port, static_cast<std::uint32_t>(baud), static_cast<std::uint32_t>(timeout)});
  worker_ = std::make_unique<BaseIoWorker>(std::move(transport), control_, hardware_, cfg);
  auto cmd_qos = rclcpp::QoS(1).reliable().durability_volatile();
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/base/safe_cmd_vel",
      cmd_qos,
      std::bind(&BaseDriverNode::onVelocity, this, std::placeholders::_1));
  estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/base/emergency_stop",
      rclcpp::QoS(1).reliable(),
      std::bind(&BaseDriverNode::onEmergencyStop, this, std::placeholders::_1));
  light_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
      "/base/light_rgb", 10, std::bind(&BaseDriverNode::onLight, this, std::placeholders::_1));
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "/base/reset_fault",
      std::bind(&BaseDriverNode::onReset, this, std::placeholders::_1, std::placeholders::_2));
  recharge_srv_ = create_service<std_srvs::srv::SetBool>(
      "/base/set_recharge",
      std::bind(&BaseDriverNode::onRecharge, this, std::placeholders::_1, std::placeholders::_2));
  mcu_safety_srv_ = create_service<std_srvs::srv::SetBool>(
      "/base/set_mcu_safety",
      std::bind(&BaseDriverNode::onMcuSafety, this, std::placeholders::_1, std::placeholders::_2));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 20);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 20);
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  for (std::size_t i = 0; i < 6; ++i)
    range_pubs_[i] = create_publisher<sensor_msgs::msg::Range>(
        "/ultrasonic/" + std::string(1, static_cast<char>('a' + i)), 10);
  charging_pub_ =
      create_publisher<inspection_robot_interfaces::msg::ChargingState>("/base/charging_state", 10);
  status_pub_ = create_publisher<inspection_robot_interfaces::msg::BaseStatus>("/base/status", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  publish_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                     std::bind(&BaseDriverNode::publishState, this));
  worker_->start();
  RCLCPP_INFO(
      get_logger(),
      "inspection_robot_base V3 worker started; only /base/safe_cmd_vel can command hardware");
}
BaseDriverNode::~BaseDriverNode() {
  if (worker_)
    worker_->stop();
}
void BaseDriverNode::onVelocity(const geometry_msgs::msg::Twist::SharedPtr m) {
  control_->submitVelocity({m->linear.x, m->linear.y, m->angular.z});
}
void BaseDriverNode::onEmergencyStop(const std_msgs::msg::Bool::SharedPtr m) {
  control_->setEmergencyStop(m->data);
}
void BaseDriverNode::onReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
  control_->requestReset();
  r->success = true;
  r->message =
      "reset request queued; base will clear fault only if serial, feedback, and e-stop conditions "
      "are healthy";
}
void BaseDriverNode::onRecharge(const std::shared_ptr<std_srvs::srv::SetBool::Request> q,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
  control_->enqueueDeviceRequest({DeviceRequestType::kRecharge, q->data, 0, 0, 0});
  r->success = true;
  r->message = "recharge request queued to single I/O writer";
}
void BaseDriverNode::onMcuSafety(const std::shared_ptr<std_srvs::srv::SetBool::Request> q,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
  if (!q->data && !allow_mcu_safety_disable_) {
    r->success = false;
    r->message = "MCU safety disable rejected by base configuration";
    return;
  }
  control_->enqueueDeviceRequest({DeviceRequestType::kMcuSafety, q->data, 0, 0, 0});
  r->success = true;
  r->message = "MCU safety request queued to single I/O writer";
}
void BaseDriverNode::onLight(const std_msgs::msg::ColorRGBA::SharedPtr m) {
  bool enabled = m->a > 0.0f;
  control_->enqueueDeviceRequest(
      {DeviceRequestType::kLight, enabled, toByte(m->r), toByte(m->g), toByte(m->b)});
}
void BaseDriverNode::publishMotion(const HardwareSnapshot& s) {
  auto stamp = now();
  nav_msgs::msg::Odometry o;
  o.header.stamp = stamp;
  o.header.frame_id = odom_frame_;
  o.child_frame_id = base_frame_;
  o.pose.pose.position.x = s.odometry.x;
  o.pose.pose.position.y = s.odometry.y;
  o.pose.pose.orientation = yawQuaternion(s.odometry.yaw);
  o.twist.twist.linear.x = s.motion.vx;
  o.twist.twist.linear.y = s.motion.vy;
  o.twist.twist.angular.z = s.motion.wz;
  odom_pub_->publish(o);
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = imu_frame_;
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity.x = s.motion.gyro_x;
  imu.angular_velocity.y = s.motion.gyro_y;
  imu.angular_velocity.z = s.motion.gyro_z;
  imu.linear_acceleration.x = s.motion.accel_x;
  imu.linear_acceleration.y = s.motion.accel_y;
  imu.linear_acceleration.z = s.motion.accel_z;
  imu_pub_->publish(imu);
  sensor_msgs::msg::BatteryState b;
  b.header.stamp = stamp;
  b.voltage = s.motion.battery_voltage;
  b.present = true;
  battery_pub_->publish(b);
}
void BaseDriverNode::publishUltrasonic(const HardwareSnapshot& s) {
  auto stamp = now();
  for (std::size_t i = 0; i < 6; ++i) {
    sensor_msgs::msg::Range r;
    r.header.stamp = stamp;
    r.header.frame_id = ultrasonic_frames_[i];
    r.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    r.field_of_view = ultrasonic_fov_;
    r.min_range = ultrasonic_min_;
    r.max_range = ultrasonic_max_;
    r.range = static_cast<float>(s.ultrasonic.ranges_m[i]);
    range_pubs_[i]->publish(r);
  }
}
void BaseDriverNode::publishCharging(const HardwareSnapshot& s) {
  inspection_robot_interfaces::msg::ChargingState m;
  m.header.stamp = now();
  m.current_a = s.charging.current_a;
  m.infrared_state = s.charging.infrared_state;
  m.charging = s.charging.charging;
  m.charge_mode_set = s.charging.charge_mode_set;
  charging_pub_->publish(m);
}
void BaseDriverNode::publishStatus(const HardwareSnapshot& s) {
  inspection_robot_interfaces::msg::BaseStatus m;
  m.header.stamp = now();
  m.state = static_cast<std::uint8_t>(s.fault.state);
  m.fault_code = static_cast<std::uint16_t>(s.fault.fault);
  m.serial_connected = s.fault.serial_open;
  m.feedback_healthy = s.fault.feedback_healthy;
  m.command_healthy = s.fault.command_healthy;
  m.emergency_stop = s.fault.emergency_stop;
  m.mcu_safety_requested = s.mcu_safety_requested;
  m.motion_frames = s.motion_seq;
  m.ultrasonic_frames = s.ultrasonic_seq;
  m.charging_frames = s.charging_seq;
  m.checksum_errors = s.router_stats.checksum_errors;
  m.framing_errors = s.router_stats.framing_errors;
  m.last_serial_error = s.serial_error;
  status_pub_->publish(m);
  diagnostic_msgs::msg::DiagnosticArray d;
  d.header.stamp = m.header.stamp;
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "inspection_robot/base";
  st.hardware_id = "inspection_robot_s200";
  st.level = (m.state == inspection_robot_interfaces::msg::BaseStatus::STATE_READY)
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : ((m.state == inspection_robot_interfaces::msg::BaseStatus::STATE_FAULT_LATCHED)
                        ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                        : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  st.message = (st.level == 0) ? "ready" : ((st.level == 2) ? "fault latched" : "not ready");
  d.status.push_back(st);
  diagnostics_pub_->publish(d);
}
void BaseDriverNode::publishState() {
  auto s = hardware_->snapshot();
  if (s.have_motion && s.motion_seq != published_motion_seq_) {
    published_motion_seq_ = s.motion_seq;
    publishMotion(s);
  }
  if (s.have_ultrasonic && s.ultrasonic_seq != published_ultrasonic_seq_) {
    published_ultrasonic_seq_ = s.ultrasonic_seq;
    publishUltrasonic(s);
  }
  if (s.have_charging && s.charging_seq != published_charging_seq_) {
    published_charging_seq_ = s.charging_seq;
    publishCharging(s);
  }
  publishStatus(s);
}
}  // namespace inspection_robot_base
