cmake_minimum_required(VERSION 3.8)
project(asv_perception_obstacleid_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(obstacle_projection_component src/ObstacleProjectionNode.cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(asv_perception_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)

# include_directories(include)
include_directories(
  include
  include/detail/
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  /opt/ros/humble/include/tf2_geometry_msgs/
  /opt/ros/humble/include/tf2/
  /opt/ros/humble/include/tf2_ros/
  /opt/ros/humble/include/tf2_msgs/
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(obstacle_projection_component ${PCL_LIBRARIES})

rclcpp_components_register_node(
  obstacle_projection_component
  PLUGIN "obstacle_id::ObstacleProjectionNode"
  EXECUTABLE obstacle_projection
)

ament_target_dependencies(
  obstacle_projection_component
  rclcpp
  rclcpp_components
  sensor_msgs
  asv_perception_interfaces
  cv_bridge
)

ament_export_include_directories(include)

ament_export_targets(export_obstacle_projection_component)
install(
  TARGETS obstacle_projection_component
  EXPORT export_obstacle_projection_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
