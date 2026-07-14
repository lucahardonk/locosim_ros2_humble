#!/usr/bin/env python3
"""ROS2 port of climbingrobot_description/launch/upload.launch.

Runs xacro on the climbingrobot2 URDF and publishes it on the
`robot_description` parameter of the robot_state_publisher node.

Design notes
------------
* Uses the ``xacro`` *Python API* inside an ``OpaqueFunction`` (same pattern as
  go1_description/launch/upload.launch.py) rather than
  ``Command([FindExecutable('xacro'), ...])`` so that a missing xacro fails with
  a clear ``ModuleNotFoundError`` instead of a misleading FindExecutable error.
* Comments are stripped from the resulting URDF (``_strip_xml_comments``)
  because gazebo_ros2_control re-injects the URDF into the embedded
  controller_manager as a ``--param robot_description:=<urdf>`` override and
  rcl's parser chokes on ``:``/``#`` characters inside XML comments.
* ROS1 mapped the launch ``spawn_*`` args onto the xacro ``anchor*`` args
  (the anchor pose is baked into the URDF; the model is then spawned at the
  world origin). We keep that mapping here.
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def _strip_xml_comments(xml):
    """Remove all XML comments (<!-- ... -->) from a URDF/SRDF string.

    Required because gazebo_ros2_control reads this URDF back from the
    robot_state_publisher `robot_description` parameter and re-injects it into
    the embedded controller_manager as a command-line parameter override; rcl's
    argument parser aborts on special characters (``:``, ``#``) that appear
    inside XML comments. Comments are non-semantic in URDF so stripping is safe.
    """
    return re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)


def _spawn_state_publisher(context, *args, **kwargs):
    pkg = get_package_share_directory('climbingrobot_description')

    robot_name = LaunchConfiguration('robot_name').perform(context)
    task_period = LaunchConfiguration('task_period').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)
    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    spawn_2x = LaunchConfiguration('spawn_2x').perform(context)
    spawn_2y = LaunchConfiguration('spawn_2y').perform(context)
    spawn_2z = LaunchConfiguration('spawn_2z').perform(context)
    double_propeller = LaunchConfiguration('double_propeller').perform(context)

    urdf_xacro = os.path.join(pkg, 'urdf', robot_name + '.xacro')

    robot_description = _strip_xml_comments(xacro.process_file(
        urdf_xacro,
        mappings={
            'robot_name': robot_name,
            'task_period': task_period,
            # ROS1 upload.launch mapped spawn_* -> anchor* (anchor pose baked
            # into the URDF; robot is spawned at the world origin).
            'anchorX': spawn_x,
            'anchorY': spawn_y,
            'anchorZ': spawn_z,
            'anchor2X': spawn_2x,
            'anchor2Y': spawn_2y,
            'anchor2Z': spawn_2z,
            'double_propeller': double_propeller,
        },
    ).toxml())

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='climbingrobot2'),
        DeclareLaunchArgument('task_period', default_value='0.001'),
        # anchor1 (base_controller_fixed passes these as spawn_x/y/z)
        DeclareLaunchArgument('spawn_x', default_value='0.2'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='20.0'),
        # anchor2 (defaults match conf.robot_params['climbingrobot2'])
        DeclareLaunchArgument('spawn_2x', default_value='0.2'),
        DeclareLaunchArgument('spawn_2y', default_value='5.0'),
        DeclareLaunchArgument('spawn_2z', default_value='20.0'),
        DeclareLaunchArgument('double_propeller', default_value='false'),

        OpaqueFunction(function=_spawn_state_publisher),
    ])
