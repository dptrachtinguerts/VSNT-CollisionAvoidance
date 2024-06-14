cmake_minimum_required(VERSION 3.8)
project(asv_perception_obstacleid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(asv_perception_obstacleid plugins.xml)

find_package(pcl_conversions REQUIRED)
find_package(PCL 1.3 REQUIRED)

find_package(asv_perception_interfaces REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rclcpp_components REQUIRED)


add_executable(ObstacleProjectionNodelet src/ObstacleProjectionNodelet.cpp)
ament_target_dependencies(ObstacleProjectionNodelet rclcpp pcl_conversions pluginlib cv_bridge asv_perception_interfaces rclcpp_components)

add_executable(RadarToPointCloudNodelet src/RadarToPointCloudNodelet.cpp)
ament_target_dependencies(RadarToPointCloudNodelet rclcpp pcl_conversions pluginlib)

install(TARGETS 
  ObstacleProjectionNodelet 
  RadarToPointCloudNodelet
  DESTINATION lib/$(PROJECT_NAME)
)

install(
  DIRECTORY include/
  DESTINATION include
)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/humble/include/tf2_geometry_msgs/
  /opt/ros/humble/include/tf2/
  /opt/ros/humble/include/tf2_ros/
  /opt/ros/humble/include/tf2_msgs/
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

ament_export_include_directories(
  include
)

# rclcpp_components_register_node(
#     ObstacleProjectionNodelet_component
#     PLUGIN "obstacle_id::ObstacleProjectionNodelet"
#     EXECUTABLE ObstacleProjectionNodelet
# )

ament_package()
