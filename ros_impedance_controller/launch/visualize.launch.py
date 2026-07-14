#!/usr/bin/env python3
"""ROS2 port of visualize.launch — RViz2 + joint_state_publisher_gui + robot_state_publisher."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    locosim_dir = os.environ.get('LOCOSIM_DIR', os.path.expanduser('~/ros2_ws/src/locosim_ros2'))
    robot_name = LaunchConfiguration('robot_name')
    test_joints = LaunchConfiguration('test_joints')

    urdf_path = [os.path.join(locosim_dir, 'robot_urdf', ''), robot_name, '.urdf']
    robot_description = ParameterValue(
        Command(['xacro ', *urdf_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='ur5'),
        DeclareLaunchArgument('test_joints', default_value='true'),
        DeclareLaunchArgument('rviz_conf',
                              default_value=os.path.join(locosim_dir, 'robot_control', 'rviz', 'config.rviz')),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', condition=IfCondition(test_joints)),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_conf'), '-f', 'base_link']),
    ])
