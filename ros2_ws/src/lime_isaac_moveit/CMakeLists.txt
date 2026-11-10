cmake_minimum_required(VERSION 3.8)
project(lime_isaac_moveit)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/arm_demo.py DESTINATION lib/${PROJECT_NAME} RENAME arm_demo)

ament_package()
