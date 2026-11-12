cmake_minimum_required(VERSION 3.8)
project(lime_isaac_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mission.py DESTINATION lib/${PROJECT_NAME} RENAME mission)
install(PROGRAMS scripts/scan_fix.py DESTINATION lib/${PROJECT_NAME} RENAME scan_fix)
install(PROGRAMS scripts/patrol.py DESTINATION lib/${PROJECT_NAME} RENAME patrol)
install(PROGRAMS scripts/tidy.py DESTINATION lib/${PROJECT_NAME} RENAME tidy)
# patrol and tidy import these two (from the folder they are installed in)
install(FILES scripts/mission.py scripts/lime_bt.py DESTINATION lib/${PROJECT_NAME})

ament_package()
