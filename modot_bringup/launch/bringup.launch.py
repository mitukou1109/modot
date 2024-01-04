from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    modot_bringup_share_dir = get_package_share_directory("modot_bringup")
    modot_recognition_share_dir = get_package_share_directory("modot_recognition")
    realsense2_camera_share_dir = get_package_share_directory("realsense2_camera")

    realsense_config_file = PathJoinSubstitution(
        [
            modot_bringup_share_dir,
            "config",
            "realsense2_camera",
            "realsense.yaml",
        ]
    )

    realsense_tf_publisher_config_file = PathJoinSubstitution(
        [
            modot_bringup_share_dir,
            "config",
            "realsense_tf_publisher.yaml",
        ]
    )

    yolo_model = PathJoinSubstitution(
        [
            modot_bringup_share_dir,
            "yolo_model",
            "vidvipo_yolov8n_2023-05-19_full_integer_quant_edgetpu.tflite",
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
            "pointcloud.stream_filter": "-1",
            "pointcloud.stream_index_filter": "-1",
            "log_level": "warn",
            "config_file": realsense_config_file,
        }.items(),
    )

    realsense_tf_publisher_node = Node(
        package="modot_description",
        executable="camera_tf_publisher_node",
        name="realsense_tf_publisher",
        output="screen",
        remappings=[("accel", "/realsense/accel/sample")],
        parameters=[realsense_tf_publisher_config_file],
    )

    yolo_detector_node = Node(
        package="ultralytics_ros",
        executable="detector",
        name="yolo_detector",
        output="screen",
        remappings=[("image_raw", "/realsense/color/image_raw")],
        parameters=[
            {
                "yolo_model": yolo_model,
                "model_conf_threshold": 0.3,
                "model_iou_threshold": 0.5,
                "model_image_size": "416",
            }
        ],
    )

    obstacle_detector_node = Node(
        package="modot_recognition",
        executable="obstacle_detector_node",
        name="obstacle_detector",
        output="screen",
        remappings=[("point_cloud", "/realsense/depth/color/points")],
    )

    face_identifier_node = Node(
        package="face_recognition_ros",
        executable="face_identifier",
        name="face_identifier",
        output="screen",
        remappings=[("image_raw", "/realsense/color/image_raw")],
    )

    sound_notifier_node = Node(
        package="modot_notification",
        executable="sound_notifier",
        name="sound_notifier",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([modot_bringup_share_dir, "rviz", "default.rviz"]),
        ],
    )

    return LaunchDescription(
        [
            realsense2_camera_launch,
            realsense_tf_publisher_node,
            yolo_detector_node,
            obstacle_detector_node,
            face_identifier_node,
            sound_notifier_node,
            rviz_node,
        ]
    )
