import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    high_confidence_thresh = LaunchConfiguration('high_confidence_thresh')
    low_confidence_thresh = LaunchConfiguration('low_confidence_thresh')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true, physical wall time if false'
    )

    declare_yolo_model = DeclareLaunchArgument(
        'yolo_model_path',
        default_value='yolo11n.pt',
        description='Pretrained lightweight YOLO model path (e.g. yolo11n.pt, yolov8n.pt, yolo26n.pt)'
    )

    declare_high_thresh = DeclareLaunchArgument(
        'high_confidence_thresh',
        default_value='0.80',
        description='Threshold for automatic evidence capture trigger'
    )

    declare_low_thresh = DeclareLaunchArgument(
        'low_confidence_thresh',
        default_value='0.40',
        description='Threshold for uncertain evidence active repositioning'
    )

    # 1. Spatial Evidence Logger Node
    evidence_logger_node = Node(
        package='forensic_robot',
        executable='evidence_logger',
        name='evidence_logger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'global_frame': 'map',
            'robot_frame': 'base_footprint'
        }]
    )

    # 2. Computer Vision Forensic Candidate Detector Node
    vision_detector_node = Node(
        package='forensic_robot',
        executable='vision_detector',
        name='vision_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yolo_model_path': yolo_model_path,
            'high_confidence_thresh': high_confidence_thresh,
            'low_confidence_thresh': low_confidence_thresh,
            'auto_capture': True,
            'capture_cooldown_sec': 3.0,
            'image_topic': '/camera/image_raw'
        }]
    )

    # 3. Active Repositioning Viewpoint Node
    active_repositioner_node = Node(
        package='forensic_robot',
        executable='active_repositioner',
        name='active_repositioner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'reposition_angle_deg': 35.0,
            'angular_speed_rad_s': 0.40,
            'reposition_cooldown_sec': 8.0,
            'settle_time_sec': 0.8
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_yolo_model,
        declare_high_thresh,
        declare_low_thresh,
        evidence_logger_node,
        vision_detector_node,
        active_repositioner_node
    ])
