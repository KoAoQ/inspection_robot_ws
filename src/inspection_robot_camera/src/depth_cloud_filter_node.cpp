#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class DepthCloudFilterNode : public rclcpp::Node {
 public:
  DepthCloudFilterNode()
      : Node("depth_cloud_filter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/depth/points");
    output_topic_ =
        declare_parameter<std::string>("output_topic", "/camera/depth/points_filtered");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    max_publish_rate_ = declare_parameter<double>("max_publish_rate", 12.0);
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.10);
    leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.05);
    min_x_ = declare_parameter<double>("min_x", 0.10);
    max_x_ = declare_parameter<double>("max_x", 3.00);
    min_y_ = declare_parameter<double>("min_y", -2.00);
    max_y_ = declare_parameter<double>("max_y", 2.00);
    min_z_ = declare_parameter<double>("min_z", 0.03);
    max_z_ = declare_parameter<double>("max_z", 1.60);
    min_points_ = declare_parameter<int>("min_points", 50);

    if (leaf_size_ <= 0.0 || max_publish_rate_ < 0.0 || min_x_ >= max_x_ ||
        min_y_ >= max_y_ || min_z_ >= max_z_) {
      throw std::runtime_error("invalid depth cloud filter parameters");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        std::bind(&DepthCloudFilterNode::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "filtering %s -> %s in %s at <= %.1f Hz, voxel %.3f m",
                input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(),
                max_publish_rate_, leaf_size_);
  }

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (max_publish_rate_ > 0.0 && have_last_publish_) {
      const double elapsed =
          std::chrono::duration<double>(steady_now - last_publish_).count();
      if (elapsed < 1.0 / max_publish_rate_) {
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    const float leaf = static_cast<float>(leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*downsampled);

    sensor_msgs::msg::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*downsampled, downsampled_msg);
    downsampled_msg.header = msg->header;

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      const auto transform = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          tf2::durationFromSec(transform_timeout_sec_));
      tf2::doTransform(downsampled_msg, transformed, transform);
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "depth cloud transform unavailable: %s", error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed, *target_cloud);

    pcl::PointCloud<pcl::PointXYZ> filtered;
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(target_cloud);
    crop.setMin(Eigen::Vector4f(static_cast<float>(min_x_), static_cast<float>(min_y_),
                               static_cast<float>(min_z_), 1.0F));
    crop.setMax(Eigen::Vector4f(static_cast<float>(max_x_), static_cast<float>(max_y_),
                               static_cast<float>(max_z_), 1.0F));
    crop.filter(filtered);

    if (static_cast<int>(filtered.size()) < min_points_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(filtered, output);
    output.header = transformed.header;
    pub_->publish(output);
    last_publish_ = steady_now;
    have_last_publish_ = true;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double max_publish_rate_{12.0};
  double transform_timeout_sec_{0.10};
  double leaf_size_{0.05};
  double min_x_{0.10};
  double max_x_{3.00};
  double min_y_{-2.00};
  double max_y_{2.00};
  double min_z_{0.03};
  double max_z_{1.60};
  int min_points_{50};
  bool have_last_publish_{false};
  std::chrono::steady_clock::time_point last_publish_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
