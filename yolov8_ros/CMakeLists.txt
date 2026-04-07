cmake_minimum_required(VERSION 3.5)
project(yolov8_ros)

# 패키지 종속성 설정
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bumblebee_interfaces REQUIRED)

# ✅ bumblebee_interfaces의 메시지 종속성 설정
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(bumblebee_interfaces)


ament_package()

