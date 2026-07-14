#!/usr/bin/env python3
"""ROS2 port of go1_description/launch/upload.launch.

Runs xacro on the Go1 URDF and publishes it on the `robot_description`
parameter of the robot_state_publisher node. The SRDF is exposed on the
`robot_semantic_description` parameter (kept for parity with ROS1).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('go1_description')
    task_period = LaunchConfiguration('task_period')
    load_force_sensors = LaunchConfiguration('load_force_sensors')

    urdf_xacro = os.path.join(pkg, 'robots', 'go1.urdf.xacro')
    srdf_xacro = os.path.join(pkg, 'robots', 'go1.srdf.xacro')

    robot_description = ParameterValue(
        Command(['ros2', 'run', 'xacro', 'xacro', ' ', urdf_xacro,
                 ' task_period:=', task_period,
                 ' load_force_sensors:=', load_force_sensors]),
        value_type=str)
    robot_semantic = ParameterValue(
        Command(['ros2', 'run', 'xacro', 'xacro', ' ', srdf_xacro]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='go1'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('task_period', default_value='0.001'),
        DeclareLaunchArgument('load_force_sensors', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'robot_semantic_description': robot_semantic,
                'use_sim_time': True,
            }],
        ),
    ])
