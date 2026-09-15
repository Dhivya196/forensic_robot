import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_forensic_robot = get_package_share_directory('forensic_robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # World file
    default_world_path = os.path.join(pkg_forensic_robot, 'worlds', 'crime_scene.world')
    rviz_config_path = os.path.join(pkg_forensic_robot, 'config', 'rviz_config.rviz')

    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world file to load'
    )

    use_rviz = LaunchConfiguration('rviz')
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    # 1. Include Robot State Publisher (RSP)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forensic_robot, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Start Gazebo Sim (Harmonic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 2 ', world]}.items()
    )

    # 3. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'forensic_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ]
    )

    # 4. ROS-GZ Bridge for Sensors and Control
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity command
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # 2D LiDAR Scan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/camera', '/camera/image_raw')
        ]
    )

    # 5. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_rviz_cmd,
        rsp,
        gazebo,
        spawn_robot,
        bridge,
        rviz_node
    ])
