cmake_minimum_required(VERSION 3.8)
project(asv_perception_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Homography.msg"
  "msg/Classification.msg"
  "msg/ClassificationArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  DEPENDENCIES sensor_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
