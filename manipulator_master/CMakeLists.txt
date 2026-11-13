cmake_minimum_required(VERSION 3.8)
project(manipulator_master)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/web_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_server
)

ament_package()
