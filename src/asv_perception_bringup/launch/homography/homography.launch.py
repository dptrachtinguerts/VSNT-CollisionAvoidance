from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    homography_node = Node(
        package="asv_perception_homography",
        executable="homography",
        parameters=[
            {"radar_img_w": 1024},
            {"yaw": 4.},
            {"pitch": 80.29},
            {"roll": -2.96},
            {"fovy": 46.3},
            {"tx": 2.5},
            {"ty": 6.2},
            {"tz": -4.4},
            {"rbg_frame_id": "camera"},
            {"radarimg_frame_id": "radar"},
            {"radar_frame_id": "base_link"},
            {"radar_range": 500}
    ])

    ld.add_action(homography_node)
    return ld