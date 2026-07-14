# robot_control (ROS2 Humble port)

This package holds the locosim control library. Following the project rule that
**core logic (kinematics, Pinocchio, control math) must not change** and only the
ROS middleware layer is ported, the translation strategy is:

* Everything that is pure Python / NumPy / Pinocchio math (kinematics,
  dynamics, optimisation, filters, trajectory generation, the MATLAB-generated
  code, etc.) is reused **unchanged** from the ROS1 sources.
* Only the modules that touch the ROS middleware are rewritten for `rclpy`.

## Fully ported ROS-facing modules (in this package)

| ROS2 file | Ported from | Notes |
|-----------|-------------|-------|
| `robot_control/components/controller_manager.py` | `base_controllers/components/controller_manager.py` | `rospy` publishers/service proxies → `rclpy` publishers/clients; `controller_manager_msgs` `SwitchController` now uses `activate_controllers`/`deactivate_controllers`. |
| `robot_control/utils/pidManager.py` | `base_controllers/utils/pidManager.py` | `rospy.ServiceProxy` → `node.create_client`; uses `ros_impedance_controller` `set_pids`/`pid` interfaces (built with `rosidl_generate_interfaces`). |
| `robot_control/utils/ros_publish.py` | `base_controllers/utils/ros_publish.py` | `rospy` node/publishers/`Time`/`Duration` → `rclpy` equivalents; RViz auto-start via `ros2 launch` subprocess; `geometry_msgs/Point` built with keyword ctor. |
| `robot_control/utils/common_functions.py` | `base_controllers/utils/common_functions.py` | Model loaders (`getRobotModel`/`getRobotModelFloating`) regenerate the URDF via the `xacro` CLI + `ament_index_python`; launch/spawn/static-transform helpers use `ros2 launch`/`ros2 run`/`tf2_ros`. All plotting helpers are copied verbatim. |
| `robot_control/utils/utils.py` | `base_controllers/utils/utils.py` | Pure helper class; the ROS1 global param-server methods are no-ops in ROS2 (no global param server). |
| `robot_control/base_controller.py` | `base_controllers/base_controller.py` | Floating-base controller. `rospy` node → `rclpy` node spun by a `MultiThreadedExecutor` in a background thread; publishers/subscriptions/service clients → `rclpy`; `tf.TransformBroadcaster` → `tf2_ros`; `roslaunch` → `ros2 launch` subprocess; `rospkg` → `ament_index_python`. Adds a `main()` entry point. |
| `robot_control/quadruped_controller.py` | `base_controllers/quadruped_controller.py` | Quadruped controller (Go1/Aliengo/Solo/HyQ). Subscribers/tf/`Rate`/shutdown handling → `rclpy` (see the module docstring); `gazebo_ros.gazebo_interface.set_model_configuration_client(...)` → the `/gazebo/set_model_configuration` service (`gazebo_msgs/srv/SetModelConfiguration`); `PidManager(joint_names)` → `PidManager(node, joint_names)`. All WBC/IK/kinematics math is unchanged. Adds a `main()` entry point. |

The reusable ROS interface layer above is shared by the robot-specific
controllers. Pure-math dependencies pulled in by the quadruped stack
(`utils/math_tools.py`, `utils/kin_dyn_utils.py`, `utils/custom_robot_wrapper.py`,
`utils/optimTools.py`, `params.py`, `components/whole_body_controller.py`,
`components/imu_utils.py`, `components/inverse_kinematics/*`,
`components/leg_odometry/*`) are copied **verbatim** (only the
`base_controllers.` → `robot_control.` import namespace is rewritten).

### Console entry points

```bash
ros2 run robot_control quadruped_controller   # Go1/Aliengo/Solo/HyQ high-level controller
ros2 run robot_control base_controller        # generic floating-base controller
```

> **Status:** the controllers above are ported and syntax-checked (`python3 -m
> py_compile`). They still need to be `colcon build`-ed and run against a live
> Gazebo simulation in a ROS2 workspace to be validated end-to-end.

## rospy → rclpy conversion recipe (for the remaining controllers)

The remaining un-ported files under `base_controllers/` (e.g. `ur5_generic.py`,
the `climbingrobot_controller/` family, and other robot-specific controllers)
mix a large amount of
unchanged control math with a thin rospy layer. They are ported by applying the
**same mechanical substitutions** demonstrated in the three modules above:

| ROS1 (rospy) | ROS2 (rclpy) |
|--------------|--------------|
| `import rospy as ros` | `import rclpy` (+ `from rclpy.node import Node`) |
| `ros.init_node('name')` | `rclpy.init()` then `node = rclpy.create_node('name')` |
| `ros.Publisher(topic, T, queue_size=n)` | `node.create_publisher(T, topic, n)` |
| `ros.Subscriber(topic, T, cb)` | `node.create_subscription(T, topic, cb, qos)` |
| `ros.ServiceProxy(name, S)` | `node.create_client(S, name)` (+ `call_async`) |
| `ros.Service(name, S, cb)` | `node.create_service(S, name, cb)` |
| `ros.get_param('p')` | `node.declare_parameter('p', default)` + `node.get_parameter('p')` |
| `ros.Rate(hz)` / `rate.sleep()` | `node.create_rate(hz)` (spun in an executor) |
| `ros.Time.now()` | `node.get_clock().now().to_msg()` |
| `ros.Duration(s)` | `rclpy.duration.Duration(seconds=s)` |
| `ros.loginfo/logwarn/logerr` | `node.get_logger().info/warn/error` |
| `ros.is_shutdown()` | `not rclpy.ok()` |
| `ros.signal_shutdown()` | `node.destroy_node()` + `rclpy.shutdown()` |
| `SomeSrvRequest()` | `SomeSrv.Request()` |
| `geometry_msgs/Point(x, y, z)` | `Point(x=.., y=.., z=..)` |

Robot-specific controller entry points should be registered in `setup.py`
(`console_scripts`) as they are ported, e.g.
`quadruped_controller = robot_control.quadruped_controller:main`.

The control algorithms themselves (Pinocchio calls, WBC, IK, ESM, filters,
integrators) are copied verbatim — only the middleware calls above change.
