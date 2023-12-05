from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_tf_publisher_node = Node(
        package="modot_description",
        executable="camera_tf_publisher",
        name="camera_tf_publisher",
        output="screen",
        remappings=[("accel", "/realsense/accel/sample")],
        parameters=[
            {
                "base_frame": "base_footprint",
                "camera_frame": "realsense_link",
                "imu_frame": "realsense_accel_optical_frame",
                "camera_x_wrt_base": 0.0,
                "camera_y_wrt_base": -0.1,
                "camera_z_wrt_base": 1.5,
            }
        ],
    )

    return LaunchDescription(
        [
            camera_tf_publisher_node,
        ]
    )
