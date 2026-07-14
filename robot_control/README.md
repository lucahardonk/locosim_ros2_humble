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

These three modules are the reusable ROS interface layer shared by the
robot-specific controllers.

## rospy → rclpy conversion recipe (for the remaining controllers)

The remaining files under `base_controllers/` (e.g. `base_controller.py`,
`quadruped_controller.py`, `ur5_generic.py`, the `climbingrobot_controller/`
family, `components/`, and `utils/common_functions.py`) mix a large amount of
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
