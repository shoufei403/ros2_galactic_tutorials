#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    # usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('turtlesim_controller'),
            'params',
            'turtlesim_controller.yaml'))


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to parameter file to load'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
        #     launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        # ),

        Node(
            package='turtlesim_controller',
            executable='turtlesim_controller_node',
            parameters=[param_dir],
            arguments=[],
            output='screen'),
    ])
