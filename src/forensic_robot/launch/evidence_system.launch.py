import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Spatial Evidence Logger Node
    evidence_logger_node = Node(
        package='forensic_robot',
        executable='evidence_logger',
        name='evidence_logger',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Computer Vision Forensic Detector Node
    vision_detector_node = Node(
        package='forensic_robot',
        executable='vision_detector',
        name='vision_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Active Repositioning / Viewpoint Planner Node
    active_repositioner_node = Node(
        package='forensic_robot',
        executable='active_repositioner',
        name='active_repositioner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        evidence_logger_node,
        vision_detector_node,
        active_repositioner_node
    ])
