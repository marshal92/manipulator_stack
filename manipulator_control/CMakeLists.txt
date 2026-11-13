cmake_minimum_required(VERSION 3.8)
project(manipulator_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(manipulator_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(teleop_manager src/teleop_manager.cpp)
ament_target_dependencies(teleop_manager
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  control_msgs
  trajectory_msgs
  moveit_msgs
  tf2
  tf2_ros
  kdl_parser
  urdf
  Eigen3
)
target_link_libraries(teleop_manager Eigen3::Eigen)

add_executable(tactical_server src/tactical_server.cpp)
ament_target_dependencies(tactical_server
  rclcpp
  rclcpp_action
  geometry_msgs
  shape_msgs
  moveit_msgs
  manipulator_interfaces
)

add_executable(control_hub src/control_hub.cpp)
ament_target_dependencies(control_hub
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_msgs
  manipulator_interfaces
  nlohmann_json
)

add_executable(trajectory_bridge src/trajectory_bridge.cpp)
ament_target_dependencies(trajectory_bridge
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
)

install(TARGETS
  teleop_manager
  tactical_server
  control_hub
  trajectory_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
