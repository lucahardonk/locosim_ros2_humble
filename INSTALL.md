# locosim ROS2 Humble — Ubuntu 22.04 Installation

This workspace is the ROS2 **Humble** port of locosim, targeting **Ubuntu 22.04
native** and **Gazebo Classic (gazebo11)** — *not* Ignition / Gazebo Sim.

## Prerequisites (apt)

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rqt* \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  libpinocchio-dev \
  python3-pin
```

> `srdfdom` is required by `base_hardware_interface`; install it with
> `sudo apt install -y ros-humble-srdfdom` if it is not already pulled in.

## Python deps (pip)

```bash
pip3 install numpy scipy matplotlib
```

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# copy or clone locosim_ros2 here, e.g.:
#   cp -r /path/to/locosim_ros2/* ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run (Go1 reference robot, Gazebo Classic)

```bash
# terminal 1 — bring up Gazebo Classic + spawn Go1 + start ros2_control controllers
ros2 launch ros_impedance_controller ros_impedance_controller.launch.py robot_name:=go1

# (visualization only — RViz + robot_state_publisher, no Gazebo)
ros2 launch ros_impedance_controller visualize.launch.py robot_name:=go1
```

## Notes

* Source your ROS2 environment (`source /opt/ros/humble/setup.bash`) before building.
* Run `rosdep install --from-paths src --ignore-src -r -y` from `~/ros2_ws` to
  resolve any remaining package dependencies automatically.
* Gazebo Classic is used on purpose (`gazebo_ros`, `libgazebo_ros2_control.so`,
  `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`). Do **not** install the
  Ignition/`gz` stack for this workspace.
