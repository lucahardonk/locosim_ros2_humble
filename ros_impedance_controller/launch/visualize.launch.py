#!/usr/bin/env python3
"""ROS2 port of visualize.launch — RViz2 + joint_state_publisher_gui + robot_state_publisher.

Uses the ``xacro`` Python API inside an ``OpaqueFunction`` (instead of
``Command([FindExecutable('xacro'), ...])``) so that a missing xacro fails with
a clear ``ModuleNotFoundError`` rather than the misleading
``executable '[<TextSubstitution object ...>]' not found on the PATH``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def _nodes(context, *args, **kwargs):
    locosim_dir = os.environ.get(
        'LOCOSIM_DIR', os.path.expanduser('~/ros2_ws/src/locosim_ros2'))
    robot_name = LaunchConfiguration('robot_name').perform(context)
    test_joints = LaunchConfiguration('test_joints')
    rviz_conf = LaunchConfiguration('rviz_conf')

    urdf_path = os.path.join(locosim_dir, 'robot_urdf', robot_name + '.urdf')
    robot_description = xacro.process_file(urdf_path).toxml()

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', condition=IfCondition(test_joints)),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_conf, '-f', 'base_link']),
    ]


def generate_launch_description():
    locosim_dir = os.environ.get(
        'LOCOSIM_DIR', os.path.expanduser('~/ros2_ws/src/locosim_ros2'))
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='ur5'),
        DeclareLaunchArgument('test_joints', default_value='true'),
        DeclareLaunchArgument('rviz_conf',
                              default_value=os.path.join(locosim_dir, 'robot_control', 'rviz', 'config.rviz')),

        OpaqueFunction(function=_nodes),
    ])
