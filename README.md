# locosim — ROS2 Humble port

ROS2 **Humble** / **Gazebo Classic (gazebo11)** port of
[locosim](https://github.com/mfocchi/locosim) (branch `develop`), for
**Ubuntu 22.04 native**.

The port follows one guiding rule: **the core robotics logic (kinematics,
Pinocchio dynamics, PID / control math) is preserved byte-for-byte; only the ROS
middleware layer is translated.** See [`INSTALL.md`](./INSTALL.md) for setup and
build instructions.

## Translation mapping applied

| ROS1 | ROS2 (Humble) |
|------|----------------|
| `rospy` | `rclpy` |
| `roscpp` (`ros::NodeHandle`) | `rclcpp` (`rclcpp::Node`) |
| `ros_control` (`controller_interface::Controller`) | `ros2_control` (`controller_interface::ControllerInterface` lifecycle: `on_init/on_configure/on_activate/on_deactivate/update`) |
| `hardware_interface::RobotHW` | `ros2_control` `hardware_interface::SystemInterface` |
| `.launch` (XML) | `.launch.py` (Python) |
| `package.xml` format 2 | `package.xml` format 3 |
| `catkin` / `catkin_package` | `ament_cmake` / `ament_python` |
| `libgazebo_ros_control.so` (`DefaultRobotHWSim`) | `libgazebo_ros2_control.so` (`gazebo_ros2_control/GazeboSystem`) |
| `tf` | `tf2_ros` |
| `.msg`/`.srv` via `message_generation` | `rosidl_generate_interfaces` |
| `ROS_INFO`/`ROS_ERROR` | `RCLCPP_INFO`/`RCLCPP_ERROR` |
| `ros::Publisher` (realtime) | `realtime_tools::RealtimePublisher` / `RealtimeBuffer` |
| controller plugin export | `PLUGINLIB_EXPORT_CLASS` + `pluginlib` xml |

Gazebo **Classic** is kept intentionally (`gazebo_ros`, gazebo11); the Ignition /
`gz` stack is **not** used.

## Packages in this workspace

### Fully hand-translated

* **`ros_impedance_controller/`** — the core ros2_control effort controller.
  Full lifecycle `ControllerInterface`, `RealtimeBuffer<Command>`,
  `RealtimePublisher`, identical PID math (discrete + continuous branches);
  `set_pids`/`generic_float` services and `pid`/`EffortPid` messages via
  `rosidl_generate_interfaces`; pluginlib export; ROS2 launch files that bring up
  Gazebo Classic, spawn the robot and chain the controller spawners; worlds/config
  copied unchanged.
* **`robot_descriptions/go1_description/`** — the **fully-worked reference robot**.
  URDF/xacro extended with a `ros2_control` system block + `GazeboSystem` plugin;
  transmissions replaced by ros2_control; `gazebo.xacro` updated to
  `libgazebo_ros2_control.so` and ROS2 Gazebo Classic sensor plugin syntax;
  controller-manager YAML; ROS2 launch files (upload/rviz/spawn); meshes copied
  unchanged.
* **`robot_hardware_interfaces/base_hardware_interface/`** — WoLF base hardware
  helper ported to `ament_cmake`/`rclcpp` (data model + SRDF/URDF loaders +
  custom ground-truth / contact-switch handle registries). See its
  [README](./robot_hardware_interfaces/README.md).
* **`robot_control/`** — the control library as an `ament_python` package with
  the core ROS-facing modules fully ported to `rclpy`
  (`controller_manager.py`, `pidManager.py`, `ros_publish.py`,
  `gripper_manager.py`) plus the documented `rospy → rclpy` recipe for the
  remaining robot-specific controllers. See its [README](./robot_control/README.md).

### Documented as out-of-scope for this simulation workspace

* Real-robot HAL packages (`go1_hardware_interface`, `aliengo_hardware_interface`,
  `ur_driver`, `zed_wrapper`) — vendor-SDK / physical-hardware only; already
  `CATKIN_IGNORE`d in the ROS1 repo. See
  [`robot_hardware_interfaces/README.md`](./robot_hardware_interfaces/README.md).
* Additional robot descriptions (aliengo, anymal, hyq, solo, ur, z1, tractor, …):
  each follows the **same mechanical URDF/xacro + gazebo + controller-yaml
  translation demonstrated on `go1_description`**; `go1` is provided as the
  complete reference to replicate.
* MATLAB-generated C code and pure-math Python (IK, WBC, ESM, filters,
  integrators, `common_functions.py`) are reused unchanged — they contain no ROS
  middleware calls.

## Verification performed

* All 32 XML / xacro / world / package.xml files parse as well-formed XML.
* All Python modules and launch files compile (`python3 -m py_compile`).
* A full `colcon build` requires a machine with ROS2 Humble + Gazebo Classic
  installed (see `INSTALL.md`); it was not run here because this environment has
  no ROS2 toolchain.
