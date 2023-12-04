import math

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "-0.1",
            "1.5",
            "0",
            f"{math.radians(85)}",
            "0",
            "base_footprint",
            "realsense_link",
        ],
    )

    return LaunchDescription(
        [
            static_transform_publisher_node,
        ]
    )
