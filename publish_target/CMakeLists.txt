cmake_minimum_required(VERSION 3.8)
project(publish_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # ✅ 꼭 필요
find_package(sensor_msgs REQUIRED)
find_package(bumblebee_interfaces REQUIRED)

add_executable(publish_target_node src/pose_transform_node.cpp)

ament_target_dependencies(publish_target_node
  rclcpp
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  bumblebee_interfaces
)

install(TARGETS publish_target_node DESTINATION lib/${PROJECT_NAME})

ament_package()

