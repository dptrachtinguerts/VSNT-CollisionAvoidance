from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    n_inputs = 2
    source_list = ["/home/vicente/videos_for_yolo/VideoCamera.mp4","/home/vicente/dados_usp/video_douglao.mkv"]

    video_publisher_nodes = []

    for i in range(n_inputs):
        video_publisher_nodes.append(Node(
            package="asv_perception_classification",
            executable="video_publisher",
            name="video_publisher_" + str(i),
            parameters=[{"idx": i},
                        {"source": source_list[i]}
                        ]
        ))

    yolo_subscriber_node = Node(
        package="asv_perception_classification",
        executable="yolo_subscriber",
        parameters=[{"n_inputs": n_inputs}]
    )

    classification_debug_node = Node(
        package="asv_perception_classification",
        executable="classification_debug",
        parameters=[{"n_inputs": n_inputs}]
    )

    for node in video_publisher_nodes:
        ld.add_action(node)
    ld.add_action(yolo_subscriber_node)
    ld.add_action(classification_debug_node)

    return ld