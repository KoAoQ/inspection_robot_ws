#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "inspection_robot_base/base_driver_node.hpp"
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::executors::MultiThreadedExecutor exec;auto node=std::make_shared<inspection_robot_base::BaseDriverNode>();exec.add_node(node);exec.spin();rclcpp::shutdown();return 0;}
