cmake_minimum_required(VERSION 3.14)
project(sar_rerun)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rerun_sdk REQUIRED)

add_executable(rerun_bridge src/rerun_bridge.cpp)
target_link_libraries(rerun_bridge rerun_sdk)
ament_target_dependencies(rerun_bridge
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)

install(TARGETS rerun_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
