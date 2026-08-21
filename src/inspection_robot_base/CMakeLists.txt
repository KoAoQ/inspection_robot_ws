cmake_minimum_required(VERSION 3.8)
project(inspection_robot_base)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(inspection_robot_interfaces REQUIRED)
find_package(serial REQUIRED)

add_library(
  ${PROJECT_NAME}_core
  src/serial_frame_router.cpp
  src/motion_protocol.cpp
  src/ultrasonic_protocol.cpp
  src/charging_protocol.cpp
  src/device_control_protocol.cpp
  src/serial_transport.cpp
  src/odometry_integrator.cpp
  src/base_fault_manager.cpp
  src/control_state.cpp
  src/hardware_state.cpp
  src/base_io_worker.cpp)
target_include_directories(
  ${PROJECT_NAME}_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                              $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core serial)

add_executable(base_driver_node src/base_driver_node.cpp src/main.cpp)
target_link_libraries(base_driver_node ${PROJECT_NAME}_core)
ament_target_dependencies(
  base_driver_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  inspection_robot_interfaces)

install(
  TARGETS ${PROJECT_NAME}_core base_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol_router test/test_protocol_router.cpp)
  if(TARGET test_protocol_router)
    target_link_libraries(test_protocol_router ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  if(TARGET test_fault_manager)
    target_link_libraries(test_fault_manager ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_io_worker_logic test/test_io_worker_logic.cpp)
  if(TARGET test_io_worker_logic)
    target_link_libraries(test_io_worker_logic ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_serial_pty test/test_serial_pty.cpp)
  if(TARGET test_serial_pty)
    target_link_libraries(test_serial_pty ${PROJECT_NAME}_core util)
    ament_target_dependencies(test_serial_pty serial)
  endif()
endif()
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
