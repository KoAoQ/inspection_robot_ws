cmake_minimum_required(VERSION 3.8)
project(inspection_robot_bringup)
find_package(ament_cmake REQUIRED)
install(PROGRAMS
  scripts/closed_loop_segment.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
