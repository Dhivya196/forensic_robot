import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'forensic_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # URDF / Xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro')) + glob(os.path.join('urdf', '*.urdf'))),
        # Worlds
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.world')) + glob(os.path.join('worlds', '*.sdf'))),
        # Config (Nav2, SLAM, RViz)
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Forensic Robot Team',
    maintainer_email='investigator@forensic-robot.local',
    description='ROS 2 package for forensic scene mapping, autonomous navigation, and spatial evidence localization robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = forensic_robot.motor_controller:main',
            'evidence_capture = forensic_robot.evidence_capture:main',
            'l298n_driver = forensic_robot.l298n_driver:main',
            'test_motors = forensic_robot.test_motors:main',
            'evidence_logger = forensic_robot.evidence_logger:main',
            'vision_detector = forensic_robot.vision_detector:main',
            'active_repositioner = forensic_robot.active_repositioner:main',
        ],
    },
)
