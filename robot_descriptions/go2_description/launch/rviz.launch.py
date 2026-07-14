#!/usr/bin/env python3
"""ROS2 port of go2_description/launch/rviz.launch."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_description')

    upload = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'upload.launch.py')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz_conf',
                              default_value=os.path.join(pkg, 'rviz', 'conf.rviz')),
        upload,
        # tf -> tf2: static_transform_publisher (ROS2 uses --frame-id / --child-frame-id)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='world_broadcaster',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'world']),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_conf'), '-f', 'world']),
    ])
