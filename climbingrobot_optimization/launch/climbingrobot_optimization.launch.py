"""
Launch file bringing up the jump optimiser + MPC controller together.

The jump optimiser plans a trajectory (publishing the reference CoM and nominal
rope forces); the MPC node consumes that reference and closes the loop.  Topics
are remapped so the planner outputs feed straight into the controller inputs.

Usage
-----
    ros2 launch climbingrobot_optimization climbingrobot_optimization.launch.py
    ros2 launch climbingrobot_optimization climbingrobot_optimization.launch.py \\
        use_propellers:=true control_rate_hz:=50.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_propellers = LaunchConfiguration("use_propellers")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    mass = LaunchConfiguration("mass")
    anchor_distance = LaunchConfiguration("anchor_distance")
    n_dyn = LaunchConfiguration("N_dyn")

    args = [
        DeclareLaunchArgument("use_propellers", default_value="false"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("mass", default_value="5.08"),
        DeclareLaunchArgument("anchor_distance", default_value="5.0"),
        DeclareLaunchArgument("N_dyn", default_value="30"),
    ]

    jump_node = Node(
        package="climbingrobot_optimization",
        executable="jump_optimizer_node",
        name="jump_optimizer_node",
        output="screen",
        parameters=[{
            "mass": mass,
            "anchor_distance": anchor_distance,
            "N_dyn": n_dyn,
            "solve_on_start": True,
        }],
    )

    mpc_node = Node(
        package="climbingrobot_optimization",
        executable="mpc_controller_node",
        name="mpc_controller_node",
        output="screen",
        parameters=[{
            "mass": mass,
            "anchor_distance": anchor_distance,
            "N_dyn": n_dyn,
            "control_rate_hz": control_rate_hz,
            "use_propellers": use_propellers,
        }],
        # feed the planner outputs into the controller inputs
        remappings=[
            ("/mpc_controller_node/set_reference_com",
             "/jump_optimizer_node/reference_com"),
            ("/mpc_controller_node/set_reference_frl",
             "/jump_optimizer_node/reference_frl"),
            ("/mpc_controller_node/set_reference_frr",
             "/jump_optimizer_node/reference_frr"),
            ("/mpc_controller_node/set_jump_time",
             "/jump_optimizer_node/jump_time"),
        ],
    )

    return LaunchDescription(args + [jump_node, mpc_node])
