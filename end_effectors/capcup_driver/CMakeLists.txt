cmake_minimum_required(VERSION 3.8)
project(capcup_driver)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS nodes/cap_bridge nodes/contact_detector nodes/haptic_search nodes/vacuum
        nodes/cap_dashboard
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
