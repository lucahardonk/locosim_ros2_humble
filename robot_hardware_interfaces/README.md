# robot_hardware_interfaces (ROS2 Humble port)

This folder contains the hardware-interface packages of locosim.

## Packages included in the ROS2 workspace

### `base_hardware_interface`
Fully ported to ROS2 (`ament_cmake`). It provides the WoLF base hardware
data model (`WolfRobotHwInterface`) plus the custom `GroundTruthInterface` and
`ContactSwitchSensorInterface` helper registries and the SRDF/URDF loading
utilities.

**Porting notes**
* `roscpp` → `rclcpp`; `ROS_ERROR_NAMED` / `ROS_DEBUG_STREAM_NAMED` → `RCLCPP_ERROR` / `RCLCPP_DEBUG_STREAM`.
* `ros::NodeHandle::getParam("/robot_description", ...)` → `rclcpp::Node::get_parameter_or("robot_description", ...)`.
* The ROS1 `hardware_interface::RobotHW` + `HardwareResourceManager` registration
  model does not exist in ros2_control. In ROS2 a real hardware plugin derives from
  `hardware_interface::SystemInterface` and exposes state/command through
  `export_state_interfaces()` / `export_command_interfaces()`. The custom
  `GroundTruth`/`ContactSwitch` handles were therefore reduced to lightweight
  name→handle registries (see the headers) that a concrete `SystemInterface`
  subclass can build upon.

## Real-robot HAL packages — intentionally NOT built (out of scope)

The following packages talk to physical quadruped hardware through the vendored
Unitree `unitree_legged_sdk` (LCM/UDP low-level SDK) and are **real-hardware
only**. In the ROS1 repository they are already disabled with a `CATKIN_IGNORE`
marker and are never used for the Gazebo Classic simulation that this workspace
targets:

* `go1_hardware_interface` (contains `go1_hal`, depends on `unitree_legged_sdk`)
* `aliengo_hardware_interface` (contains `aliengo_hal`, depends on `unitree_legged_sdk`)
* `ur_driver`, `zed_wrapper` — external vendor drivers (Universal Robots / Stereolabs ZED)

They are kept out of this simulation workspace on purpose. Porting them to
ros2_control would require:
1. building each as a `hardware_interface::SystemInterface` plugin exported via
   `PLUGINLIB_EXPORT_CLASS` with a `<ros2_control>` tag referencing the plugin in
   the robot URDF;
2. replacing the ROS1 `RobotHW::read()/write()` loop with the ros2_control
   `on_init/on_configure/on_activate/read/write` lifecycle;
3. a ROS2-compatible build of the corresponding vendor SDK.

The Gazebo simulation path used by locosim does not need them: in simulation the
`gazebo_ros2_control/GazeboSystem` plugin (declared in each robot's
`gazebo.xacro` / `ros2_control.xacro`) provides the hardware interface.
