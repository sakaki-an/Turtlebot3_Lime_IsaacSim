cmake_minimum_required(VERSION 3.8)
project(lime_isaac_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mission.py DESTINATION lib/${PROJECT_NAME} RENAME mission)

ament_package()
