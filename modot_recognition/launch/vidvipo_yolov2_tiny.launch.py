from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    modot_recognition_share_dir = get_package_share_directory("modot_recognition")
    darknet_ros_share_dir = get_package_share_directory("darknet_ros")
    darknet_ros_config_dir = PathJoinSubstitution(
        [modot_recognition_share_dir, "config", "darknet_ros"]
    )

    yolo_weights_path = PathJoinSubstitution(
        [modot_recognition_share_dir, "yolo_network_config", "weights"]
    )
    yolo_config_path = PathJoinSubstitution(
        [modot_recognition_share_dir, "yolo_network_config", "cfg"]
    )
    ros_param_file = PathJoinSubstitution([darknet_ros_config_dir, "ros.yaml"])
    network_param_file = PathJoinSubstitution(
        [darknet_ros_config_dir, "vidvipo-yolov2-tiny.yaml"]
    )

    rmw_fastrtps_group = GroupAction(
        [
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE",
                PathJoinSubstitution(
                    [
                        modot_recognition_share_dir,
                        "config",
                        "DEFAULT_FASTRTPS_PROFILES.xml",
                    ]
                ),
            ),
            SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "1"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        ]
    )

    darknet_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [darknet_ros_share_dir, "launch", "darknet_ros.launch.py"]
            )
        ),
        launch_arguments={
            "yolo_weights_path": yolo_weights_path,
            "yolo_config_path": yolo_config_path,
            "ros_param_file": ros_param_file,
            "network_param_file": network_param_file,
        }.items(),
    )

    return LaunchDescription(
        [
            rmw_fastrtps_group,
            darknet_ros_launch,
        ]
    )
