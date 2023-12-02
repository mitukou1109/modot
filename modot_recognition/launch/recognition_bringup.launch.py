from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    modot_recognition_share_dir = get_package_share_directory("modot_recognition")
    realsense2_camera_share_dir = get_package_share_directory("realsense2_camera")

    realsense2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [realsense2_camera_share_dir, "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={"pointcloud.enable": "true", "camera_namespace": ""}.items(),
    )

    vidvipo_yolov2_tiny_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [modot_recognition_share_dir, "launch", "vidvipo_yolov2_tiny.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            realsense2_camera_launch,
            vidvipo_yolov2_tiny_launch,
        ]
    )
