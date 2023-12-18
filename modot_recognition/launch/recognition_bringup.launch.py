from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    modot_recognition_share_dir = get_package_share_directory("modot_recognition")
    realsense2_camera_share_dir = get_package_share_directory("realsense2_camera")

    # realsense_config_file = PathJoinSubstitution(
    #     [
    #         modot_recognition_share_dir,
    #         "config",
    #         "realsense2_camera",
    #         "realsense.yaml",
    #     ]
    # )
    realsense_config_file = (
        f"'{modot_recognition_share_dir}/config/realsense2_camera/realsense.yaml'"
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
            "config_file": realsense_config_file,
        }.items(),
    )

    realsense_tf_publisher_node = Node(
        package="modot_description",
        executable="camera_tf_publisher",
        name="realsense_tf_publisher",
        output="screen",
        remappings=[("accel", "/realsense/accel/sample")],
        parameters=[
            {
                "camera_frame": "realsense_link",
                "imu_frame": "realsense_accel_optical_frame",
                "camera_x": 0.1,
                "camera_y": -0.1,
                "camera_z": 1.5,
            }
        ],
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
            rmw_fastrtps_group,
            realsense2_camera_launch,
            realsense_tf_publisher_node,
            # vidvipo_yolov2_tiny_launch,
            obstacle_detector_node,
            rviz_node,
        ]
    )
