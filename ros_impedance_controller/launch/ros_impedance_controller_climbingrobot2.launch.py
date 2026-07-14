#!/usr/bin/env python3
"""
ROS2 (Humble) + Gazebo Classic launch for the climbingrobot2 (two-rope + MPC +
propellers). Ported from
ros_impedance_controller/launch/ros_impedance_controller_climbingrobot2.launch.

It:
  1. Loads the robot URDF (via climbingrobot_description/launch/upload.launch.py)
     into the robot_state_publisher `robot_description` parameter. The anchor
     poses are baked into the URDF from spawn_x/y/z + spawn_2x/2y/2z.
  2. Starts Gazebo Classic (gzserver + gzclient) with the requested world,
     loading the gazebo_ros_init/factory system plugins.
  3. Spawns the robot into Gazebo at the world origin (the anchors carry the
     pose) and spawns the climbing wall (climb_wall2.xacro) at mountain_pos_x.
  4. Spawns the ros2_control controllers (joint_state_broadcaster and
     ros_impedance_controller) via the controller_manager spawner. The
     controller_manager itself is provided by the libgazebo_ros2_control.so
     plugin declared in the robot URDF (gazebo/gazebo.xacro), so no standalone
     ros2_control_node is started here.
  5. Optionally starts RViz2.

`base_controller_fixed.startSimulator` invokes this file with
`robot_name:=climbingrobot2 spawn_x:=.. spawn_y:=.. spawn_z:=..
use_torque_control:=.. [world_name:=..]`.
"""

import os
import tempfile
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import xacro


def _strip_xml_comments(xml):
    return re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)


def _spawn_wall(context, *args, **kwargs):
    """Process climb_wall2.xacro and spawn it into Gazebo from a temp file.

    spawn_entity.py has no '-string' option, so we materialise the processed
    URDF to a temporary file and spawn it with '-file'.
    """
    pkg_desc = get_package_share_directory('climbingrobot_description')
    wall_xacro = os.path.join(pkg_desc, 'urdf', 'climb_wall2.xacro')

    spawn_z = LaunchConfiguration('spawn_z').perform(context)
    wall_inclination = LaunchConfiguration('wall_inclination').perform(context)
    obstacle = LaunchConfiguration('obstacle').perform(context)
    obstacle_location_x = LaunchConfiguration('obstacle_location_x').perform(context)
    obstacle_location_y = LaunchConfiguration('obstacle_location_y').perform(context)
    obstacle_location_z = LaunchConfiguration('obstacle_location_z').perform(context)
    obstacle_size_x = LaunchConfiguration('obstacle_size_x').perform(context)
    obstacle_size_y = LaunchConfiguration('obstacle_size_y').perform(context)
    obstacle_size_z = LaunchConfiguration('obstacle_size_z').perform(context)
    mountain_pos_x = LaunchConfiguration('mountain_pos_x').perform(context)

    wall_urdf = _strip_xml_comments(xacro.process_file(
        wall_xacro,
        mappings={
            'anchorZ': spawn_z,
            'wall_inclination': wall_inclination,
            'obstacle': obstacle,
            'obstacle_location_x': obstacle_location_x,
            'obstacle_location_y': obstacle_location_y,
            'obstacle_location_z': obstacle_location_z,
            'obstacle_size_x': obstacle_size_x,
            'obstacle_size_y': obstacle_size_y,
            'obstacle_size_z': obstacle_size_z,
        },
    ).toxml())

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='_climb_wall2.urdf',
                                      delete=False, encoding='utf-8')
    tmp.write(wall_urdf)
    tmp.close()

    return [
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-file', tmp.name, '-entity', 'climb_wall',
                       '-x', mountain_pos_x],
        ),
    ]


def generate_launch_description():
    pkg_ric = get_package_share_directory('ros_impedance_controller')
    pkg_desc = get_package_share_directory('climbingrobot_description')

    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    task_period = LaunchConfiguration('task_period')

    declared_args = [
        DeclareLaunchArgument('robot_name', default_value='climbingrobot2'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz_conf',
                              default_value=os.path.join(pkg_desc, 'rviz', 'conf.rviz')),
        DeclareLaunchArgument('world_name', default_value='climbingrobot2.world'),
        DeclareLaunchArgument('task_period', default_value='0.001'),
        DeclareLaunchArgument('use_torque_control', default_value='true'),
        # anchor1 (passed by base_controller_fixed.startSimulator)
        DeclareLaunchArgument('spawn_x', default_value='0.2'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='20.0'),
        # anchor2 (defaults match conf.robot_params['climbingrobot2'])
        DeclareLaunchArgument('spawn_2x', default_value='0.2'),
        DeclareLaunchArgument('spawn_2y', default_value='5.0'),
        DeclareLaunchArgument('spawn_2z', default_value='20.0'),
        DeclareLaunchArgument('double_propeller', default_value='false'),
        # wall / obstacle
        DeclareLaunchArgument('wall_inclination', default_value='0.0'),
        DeclareLaunchArgument('mountain_pos_x', default_value='-0.05'),
        DeclareLaunchArgument('obstacle', default_value='false'),
        DeclareLaunchArgument('obstacle_location_x', default_value='-0.5'),
        DeclareLaunchArgument('obstacle_location_y', default_value='2.5'),
        DeclareLaunchArgument('obstacle_location_z', default_value='-6.0'),
        DeclareLaunchArgument('obstacle_size_x', default_value='1.5'),
        DeclareLaunchArgument('obstacle_size_y', default_value='1.5'),
        DeclareLaunchArgument('obstacle_size_z', default_value='0.866'),
    ]

    def _prepend_env(var, path):
        cur = os.environ.get(var, '')
        os.environ[var] = path + (':' + cur if cur else '')

    # Let Gazebo Classic resolve package:// mesh URIs used in the URDF/wall and
    # find the world models/media.
    _prepend_env('GAZEBO_MODEL_PATH', os.path.join(pkg_ric, 'worlds', 'models'))
    for _pkg in ('climbingrobot_description', 'ros_impedance_controller'):
        try:
            _share_parent = os.path.dirname(get_package_share_directory(_pkg))
            _prepend_env('GAZEBO_MODEL_PATH', _share_parent)
            _prepend_env('GAZEBO_RESOURCE_PATH', _share_parent)
        except Exception:  # noqa: BLE001
            pass

    world_path = PathJoinSubstitution([os.path.join(pkg_ric, 'worlds'), world_name])

    # 1 - Upload the robot description (bakes the anchor poses into the URDF).
    upload_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'upload.launch.py')
        ),
        launch_arguments={
            'robot_name': robot_name,
            'task_period': task_period,
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_2x': LaunchConfiguration('spawn_2x'),
            'spawn_2y': LaunchConfiguration('spawn_2y'),
            'spawn_2z': LaunchConfiguration('spawn_2z'),
            'double_propeller': LaunchConfiguration('double_propeller'),
        }.items(),
    )

    # 2 - Start Gazebo Classic with ROS2 system plugins.
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen',
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'], output='screen', condition=IfCondition(gui),
    )

    # 3 - Spawn the robot at the world origin (anchors carry the pose).
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', robot_name,
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
    )

    # 3b - Spawn the climbing wall (processed from climb_wall2.xacro).
    spawn_wall = OpaqueFunction(function=_spawn_wall)

    # 4 - Spawn the ros2_control controllers via controller_manager.
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

    # 5 - RViz2 (fixed frame = world, which is the URDF root link).
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
        spawn_wall,
        delay_jsb,
        delay_ric,
        rviz_node,
    ])
