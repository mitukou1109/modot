from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    modot_description_share_dir = get_package_share_directory("modot_description")
    modot_recognition_share_dir = get_package_share_directory("modot_recognition")
    realsense2_camera_share_dir = get_package_share_directory("realsense2_camera")

    realsense2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [realsense2_camera_share_dir, "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "realsense",
            "camera_namespace": "",
            "enable_sync": "true",
            "enable_accel": "true",
            "pointcloud.enable": "true",
        }.items(),
    )

    transform_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [modot_description_share_dir, "launch", "description_bringup.launch.py"]
            )
        )
    )

    vidvipo_yolov2_tiny_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [modot_recognition_share_dir, "launch", "vidvipo_yolov2_tiny.launch.py"]
            )
        )
    )

    obstacle_detector_node = Node(
        package="modot_recognition",
        executable="obstacle_detector",
        name="obstacle_detector",
        output="screen",
        remappings=[("point_cloud", "/realsense/depth/color/points")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [modot_recognition_share_dir, "rviz", "recognition.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            realsense2_camera_launch,
            transform_publisher_launch,
            # vidvipo_yolov2_tiny_launch,
            obstacle_detector_node,
            rviz_node,
        ]
    )
