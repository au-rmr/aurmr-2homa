cmake_minimum_required(VERSION 3.8)
project(perception_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/BinItem.srv
  srv/GetItem.srv
  srv/DetectGrasp.srv
  DEPENDENCIES geometry_msgs sensor_msgs)
ament_package()
