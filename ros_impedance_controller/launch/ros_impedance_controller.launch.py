#!/usr/bin/env python3
"""
ROS2 (Humble) + Gazebo Classic launch file for the ros_impedance_controller.

Ported from ros_impedance_controller.launch (ROS1). It:
  1. Loads the robot URDF (via the <robot>_description package upload launch)
     into the robot_state_publisher `robot_description` parameter.
  2. Starts Gazebo Classic (gzserver + gzclient) with the requested world,
     loading the gazebo_ros_init/factory system plugins.
  3. Spawns the robot into Gazebo (spawn_entity.py).
  4. Spawns the ros2_control controllers (joint_state_broadcaster and
     ros_impedance_controller) via the controller_manager spawner.
  5. Optionally starts RViz2.

NOTE: with gazebo_ros2_control the controller_manager is provided by the
`libgazebo_ros2_control.so` plugin declared in the robot URDF, so we do not
start a standalone ros2_control_node here.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ric = get_package_share_directory('ros_impedance_controller')

    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    task_period = LaunchConfiguration('task_period')
    pid_discrete = LaunchConfiguration('pid_discrete_implementation')

    declared_args = [
        DeclareLaunchArgument('robot_name', default_value='go1'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz_conf',
                              default_value=os.path.join(pkg_ric, 'config', 'operator_floating.rviz')),
        DeclareLaunchArgument('world_name', default_value='solo.world'),
        DeclareLaunchArgument('real_robot', default_value='false'),
        DeclareLaunchArgument('task_period', default_value='0.001'),
        DeclareLaunchArgument('pid_discrete_implementation', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.3'),
        DeclareLaunchArgument('spawn_R', default_value='0.0'),
        DeclareLaunchArgument('spawn_P', default_value='0.0'),
        DeclareLaunchArgument('spawn_Y', default_value='0.0'),
    ]

    # Make the world models discoverable by Gazebo
    gazebo_model_path = os.path.join(pkg_ric, 'worlds', 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    world_path = PythonExpression(
        ["'", os.path.join(pkg_ric, 'worlds', ''), "' + '", world_name, "'"])

    # 1 - Upload the robot description (delegated to <robot>_description package)
    upload_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PythonExpression([
                "__import__('ament_index_python.packages', fromlist=['get_package_share_directory'])"
                ".get_package_share_directory('", robot_name, "_description') + '/launch/upload.launch.py'"
            ])
        ),
        launch_arguments={'task_period': task_period}.items(),
    )

    # 2 - Start Gazebo Classic (gzserver + gzclient) with ROS2 system plugins
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', '--pause', world_path],
        output='screen',
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'], output='screen', condition=IfCondition(gui),
    )

    # 3 - Spawn the robot into Gazebo from the robot_description topic
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', robot_name,
                   '-x', LaunchConfiguration('spawn_x'),
                   '-y', LaunchConfiguration('spawn_y'),
                   '-z', LaunchConfiguration('spawn_z'),
                   '-R', LaunchConfiguration('spawn_R'),
                   '-P', LaunchConfiguration('spawn_P'),
                   '-Y', LaunchConfiguration('spawn_Y')],
    )

    # 4 - Spawn the ros2_control controllers (via controller_manager)
    jsb_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )
    ric_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['ros_impedance_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # 5 - robot_state_publisher is started by <robot>_description/upload.launch.py
    #     (included above as `upload_description`) with the robot_description
    #     parameter set from xacro. Do NOT start a second one here — it would
    #     collide on the node name and have an empty robot_description.

    # 6 - RViz2
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_conf'), '-f', 'world'],
        condition=IfCondition(rviz), output='screen',
    )

    # Ordering: load joint_state_broadcaster after the robot is spawned, then
    # the impedance controller after the broadcaster.
    delay_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[jsb_spawner]))
    delay_ric = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[ric_spawner]))

    return LaunchDescription(declared_args + [
        upload_description,
        gzserver,
        gzclient,
        spawn_entity,
        delay_jsb,
        delay_ric,
        rviz_node,
    ])
