cmake_minimum_required(VERSION 3.16)
project(sar_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

add_library(sar_control SHARED
  src/arduino_comms.cpp
  src/sar_system.cpp
)

target_include_directories(sar_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sar_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(sar_control Python3::Python)

pluginlib_export_plugin_description_file(hardware_interface sar_control.xml)

add_executable(test_hardware src/test_hardware.cpp)
ament_target_dependencies(test_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(test_hardware Python3::Python)

install(TARGETS test_hardware DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS sar_control
  EXPORT export_sar_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_sar_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
