cmake_minimum_required(VERSION 3.8)
project(bumblebee_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(DIRECTORY launch rviz ros2_control
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
