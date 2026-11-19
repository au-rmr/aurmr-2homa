cmake_minimum_required(VERSION 3.8)
project(tahoma_control)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(bondcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tahoma_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
add_executable(motion_server src/motion_server.cpp)
target_link_libraries(motion_server
  bondcpp::bondcpp rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${control_msgs_TARGETS} ${controller_manager_msgs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS} ${tahoma_interfaces_TARGETS})
install(TARGETS motion_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Streamed joint velocities: the last check, inside controller_manager's loop (S7).
add_library(streaming_velocity_controller SHARED src/streaming_velocity_controller.cpp)
target_link_libraries(streaming_velocity_controller
  controller_interface::controller_interface hardware_interface::hardware_interface
  pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools ${std_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)
install(TARGETS streaming_velocity_controller
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_package()
