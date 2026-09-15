import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_forensic_robot = get_package_share_directory('forensic_robot')

    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for physical LiDAR (e.g. /dev/ttyUSB0 or /dev/ttyAMA0)'
    )

    # 1. Robot State Publisher (TF Tree for physical robot)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_forensic_robot, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 2. L298N Physical Motor Driver Node (GPIO control for 4 DC Motors)
    l298n_driver_node = Node(
        package='forensic_robot',
        executable='l298n_driver',
        name='l298n_driver',
        output='screen',
        parameters=[{
            'in1_pin': 24,
            'in2_pin': 23,
            'ena_pin': 12,
            'in3_pin': 22,
            'in4_pin': 27,
            'enb_pin': 13,
            'wheel_radius': 0.0325,
            'wheel_base': 0.200,
            'min_pwm_duty': 25.0
        }]
    )

    # 3. 2D LiDAR Driver (RPLiDAR A1/A2/A3/C1 standard driver)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_serial_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # 4. USB / V4L2 / Raspberry Pi Camera Node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_optical_link'
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw')
        ]
    )

    return LaunchDescription([
        declare_lidar_port,
        rsp,
        l298n_driver_node,
        rplidar_node,
        camera_node
    ])
