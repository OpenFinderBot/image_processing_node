import os

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_processing_node",
                namespace=os.environ["IMAGE_PROCESSING_NODE_NAMESPACE"],
                executable="image_processing_node",
                name="image_processing_node",
                output="screen",
                parameters=[os.environ["IMAGE_PROCESSING_NODE_PARAMETERS"]],
                # remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
            )
        ]
    )
