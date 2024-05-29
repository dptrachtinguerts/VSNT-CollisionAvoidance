from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    video_publisher_node = Node(
        package="asv_perception_classification",
        executable="video_publisher"
    )

    ld.add_action(video_publisher_node)
    return ld