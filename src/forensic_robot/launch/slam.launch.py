import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_forensic_robot = get_package_share_directory('forensic_robot')
    slam_params_file = os.path.join(
        pkg_forensic_robot, 'config', 'mapper_params_online_async.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=slam_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        start_async_slam_toolbox_node
    ])
