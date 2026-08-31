cmake_minimum_required(VERSION 3.8)
project(inspection_robot_camera)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(depth_cloud_filter_node src/depth_cloud_filter_node.cpp)
ament_target_dependencies(
  depth_cloud_filter_node
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_include_directories(depth_cloud_filter_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(depth_cloud_filter_node ${PCL_LIBRARIES})

install(TARGETS depth_cloud_filter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
