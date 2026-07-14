#!/usr/bin/env python3
"""ROS2 port of go2_description/launch/spawn.launch.

Uploads the robot description and spawns the model into a running Gazebo
Classic instance via gazebo_ros spawn_entity.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_description')
    robot_name = LaunchConfiguration('robot_name')

    upload = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'upload.launch.py')),
        launch_arguments={'task_period': LaunchConfiguration('task_period')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='go2'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('task_period', default_value='0.004'),
        upload,
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             name=['spawn_', robot_name],
             arguments=['-topic', 'robot_description', '-entity', robot_name,
                        '-x', '0', '-y', '0', '-z', '1']),
    ])
