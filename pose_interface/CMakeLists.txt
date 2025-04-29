cmake_minimum_required(VERSION 3.8)
project(pose_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

# Executable
add_executable(pose_interface
  src/pose_interface.cpp)
ament_target_dependencies(pose_interface
  rclcpp
  tf2_ros
  geometry_msgs)

# Install
install(TARGETS pose_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
