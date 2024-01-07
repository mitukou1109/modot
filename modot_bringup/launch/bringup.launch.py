from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    modot_bringup_share_dir = get_package_share_directory("modot_bringup")

    realsense_config_file = PathJoinSubstitution(
        [
            modot_bringup_share_dir,
            "config",
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

    realsense_container = ComposableNodeContainer(
        name="realsense_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                name="realsense",
                namespace="",
                parameters=[realsense_config_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="modot_recognition",
                plugin="modot_recognition::ObstacleDetector",
                name="obstacle_detector",
                remappings=[("point_cloud", "/realsense/depth/color/points")],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
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

    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0", "--baud", "115200"],
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
            realsense_container,
            realsense_tf_publisher_node,
            yolo_detector_node,
            face_identifier_node,
            sound_notifier_node,
            micro_ros_agent_node,
            rviz_node,
        ]
    )
