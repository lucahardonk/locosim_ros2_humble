# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi

ROS2 (Humble) port of base_controllers/base_controller_fixed.py
--------------------------------------------------------------------------
This is the fixed-base counterpart of base_controller.py (used by the
climbing-robot and jumpleg controllers, which subclass BaseControllerFixed).
The rospy -> rclpy translation mirrors base_controller.py exactly:
  * rospy node                    -> rclpy.init() + rclpy.create_node(); spun by a
    MultiThreadedExecutor in a background thread (see startExecutor()).
  * rospy.Publisher/Subscriber    -> node.create_publisher / create_subscription.
  * rospy.ServiceProxy            -> node.create_client + call_async, waited on by
    _call_service() (the background executor drives the response).
  * roslaunch python API          -> `ros2 launch ...` via subprocess.Popen.
  * rospkg.RosPack().get_path(pkg)-> ament get_package_share_directory(pkg).
  * gazebo_msgs *Request wrappers -> the ROS2 `.Request()` factory of the service.

COMPATIBILITY CAVEATS (need runtime verification against `ros2 service list`):
  * Gazebo simulation-control services move to the ROOT namespace in gazebo_ros2
    (Gazebo Classic, humble): /pause_physics, /unpause_physics (std_srvs/Empty).
  * set_model_state -> /set_entity_state (gazebo_msgs/SetEntityState) via the
    gazebo_ros_state plugin; apply_body_wrench -> /apply_link_wrench
    (gazebo_msgs/ApplyLinkWrench) via gazebo_ros_force_system; set_model_configuration
    and get/set_physics_properties have no direct gazebo_ros2 equivalent.
  * checkRosControllerRunning() does not exist in the ROS2 common_functions; it is
    reimplemented here against the controller_manager `list_controllers` service and
    degrades gracefully (warns + returns True) when unavailable.
  _call_service() returns None when a service is missing, so an un-migrated call
  disables that feature instead of crashing the controller.
All kinematics / dynamics / Pinocchio / control math is unchanged.
"""

from __future__ import print_function
import os
import subprocess
import threading
import time
import sys

import numpy as np
from numpy import nan
from termcolor import colored
import matplotlib.pyplot as plt

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile

from ament_index_python.packages import get_package_share_directory

# messages for topic subscribers
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

# gazebo messages / services
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import ApplyBodyWrench

# other utils
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.pidManager import PidManager
from base_controllers.utils.utils import Utils
from base_controllers.utils.math_tools import *
from base_controllers.utils.math_tools import Math
from base_controllers.utils.common_functions import getRobotModel, plotJoint
from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import base_controllers.params as conf

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)

# robots can be ur5 and jumpleg; to load ur5 you need to set this xacro path in loadModelAndPublishers
robotName = "ur5"


class BaseControllerFixed(threading.Thread):
    """
        This Class can be used to simulate fixed-base robots (e.g. manipulators,
        or robots anchored to the world such as the climbing robot / jumpleg).
        ROS2 Humble port; see module docstring for the rospy->rclpy mapping.
        All docstrings/attributes are unchanged from the ROS1 version.
    """

    def __init__(self, robot_name="ur5", external_conf=None):
        threading.Thread.__init__(self)

        if (external_conf is not None):
            conf.robot_params = external_conf.robot_params

        self.robot_name = robot_name

        # rclpy node (spun by a background executor, see startExecutor())
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(self.robot_name + '_base_controller_fixed')
        self._executor = None
        self._executor_thread = None

        self.base_offset = np.array([conf.robot_params[self.robot_name]['spawn_x'],
                                     conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z']])
        self.u = Utils()
        self.math_utils = Math()
        self.contact_flag = False
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        # send data to param server
        self.verbose = conf.verbose
        self.use_torque_control = True

        print("Initialized fixed basecontroller---------------------------------------------------------------")

    # ------------------------------------------------------------------ helpers
    def startExecutor(self):
        """Spin the node in a background thread so subscription callbacks fire
        (rospy did this implicitly)."""
        if self._executor is not None:
            return
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

    def _call_service(self, client, request, timeout=5.0):
        """Synchronous service call: wait for service, call_async, block on the
        future (the background executor drives the response)."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().warn("service %s not available" % client.srv_name)
            return None
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.001)
        return future.result() if future.done() else None

    def _now(self):
        return self.node.get_clock().now()

    def _time_sec(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _sleep(self, seconds):
        time.sleep(seconds)

    def _check_ros_controller_running(self, controller_name, robot_name):
        """ROS2 replacement for common_functions.checkRosControllerRunning().
        Queries the controller_manager `list_controllers` service and returns
        True if `controller_name` is active. Degrades gracefully (warns +
        returns True) if controller_manager_msgs / the service are unavailable,
        so the startup does not hard-fail on setups where the check cannot run."""
        try:
            from controller_manager_msgs.srv import ListControllers
        except Exception:
            self.node.get_logger().warn(
                "controller_manager_msgs not available; skipping controller check")
            return True
        cm_service = '/' + robot_name + '/controller_manager/list_controllers'
        client = self.node.create_client(ListControllers, cm_service)
        if not client.wait_for_service(timeout_sec=5.0):
            # try global namespace as a fallback
            client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
            if not client.wait_for_service(timeout_sec=5.0):
                self.node.get_logger().warn(
                    "controller_manager list_controllers service not available; skipping check")
                return True
        resp = self._call_service(client, ListControllers.Request())
        if resp is None:
            return True
        for ctrl in resp.controller:
            if ctrl.name == controller_name and ctrl.state == 'active':
                return True
        return False

    # ------------------------------------------------------------------ startup
    def startSimulator(self, world_name=None, additional_args=None, launch_file=None):
        # clean up previous processes (force-kill and wait so the Gazebo master
        # TCP port is released before relaunch)
        os.system("killall -9 rviz2 gzserver gzclient 2>/dev/null")
        self._sleep(1.0)

        print(colored('Adding gazebo model path!', 'blue'))
        custom_models_path = get_package_share_directory('ros_impedance_controller') + "/worlds/models/"
        if os.getenv("GAZEBO_MODEL_PATH") is not None:
            os.environ["GAZEBO_MODEL_PATH"] += ":" + custom_models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = custom_models_path

        # ROS1 roslaunch -> ROS2 `ros2 launch`. In ROS2 the framework launch file
        # lives in ros_impedance_controller as a .launch.py.
        if launch_file == 'standard':
            launch_name = 'start_framework.launch.py'
        elif launch_file is None:
            launch_name = 'ros_impedance_controller_' + self.robot_name + '.launch.py'
        else:
            launch_name = launch_file

        cli_args = ["ros2", "launch", "ros_impedance_controller", launch_name,
                    'robot_name:=' + self.robot_name,
                    'spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z']),
                    'use_torque_control:=' + str(self.use_torque_control).lower()]
        if additional_args is not None:
            cli_args.extend(additional_args)
        if world_name is not None:
            print(colored("Setting custom model: " + str(world_name), "blue"))
            cli_args.append('world_name:=' + str(world_name))

        self.sim_process = subprocess.Popen(cli_args)
        self._sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))

    def loadModelAndPublishers(self, xacro_path=None, additional_urdf_args=None, markers_time_to_live=0.):

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True, markers_time_to_live=markers_time_to_live)

        qos = QoSProfile(depth=1)
        self.pub_des_jstate = self.node.create_publisher(JointState, "/command", qos)

        # freeze base and pause simulation service clients.
        # See module docstring for the ROS1 -> ROS2 gazebo service name/type caveats.
        self.reset_world = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        self.set_physics_client = self.node.create_client(SetPhysicsProperties, '/gazebo/set_physics_properties')
        self.get_physics_client = self.node.create_client(GetPhysicsProperties, '/gazebo/get_physics_properties')
        self.pause_physics_client = self.node.create_client(Empty, '/pause_physics')
        self.unpause_physics_client = self.node.create_client(Empty, '/unpause_physics')
        self.reset_joints_client = self.node.create_client(SetModelConfiguration, '/gazebo/set_model_configuration')
        self.apply_body_wrench = self.node.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')

        self.u.putIntoGlobalParamServer("verbose", self.verbose)

        # subscribers
        self.sub_jstate = self.node.create_subscription(
            JointState, "/" + self.robot_name + "/joint_states", self._receive_jstate, qos)

        # start spinning so callbacks fire
        self.startExecutor()

        if (self.use_torque_control):
            self.pid = PidManager(self.node, self.joint_names)

        # Loading a robot model of robot (Pinocchio)
        if xacro_path is None:
            print(colored("setting default xacro path", "blue"))
            xacro_path = get_package_share_directory(
                self.robot_name + '_description') + '/urdf/' + self.robot_name + '.xacro'
        else:
            print(colored(f"loading custom xacro path:  : {xacro_path}", "blue"))
        self.robot = getRobotModel(self.robot_name, generate_urdf=True, xacro_path=xacro_path,
                                   additional_urdf_args=additional_urdf_args)

    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        # No need to change the convention because in the HW interface we use our
        # convention (see ros_impedance_contoller_xx.yaml)
        msg = JointState()
        msg.position = list(map(float, q_des))
        msg.velocity = list(map(float, qd_des))
        msg.effort = list(map(float, tau_ffwd))
        self.pub_des_jstate.publish(msg)

    def deregister_node(self):
        print("deregistering nodes")
        self.ros_pub.deregister_node()
        try:
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    def startupProcedure(self):
        if (self.use_torque_control):
            if not self._check_ros_controller_running("ros_impedance_controller", self.robot_name):
                print(colored('Error: you need to launch the ros impedance controller in torque mode!', 'red'))
                sys.exit()
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                                 conf.robot_params[self.robot_name]['kd'],
                                 np.zeros(self.robot.na))
        print(colored("Startup accomplished -----------------------", "red"))

    def initVars(self):

        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.q_des = np.zeros(self.robot.na)
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd = np.zeros(self.robot.na)

        self.x_ee = np.zeros(3)
        self.x_ee_des = np.zeros(3)

        self.contactForceW = np.zeros(3)
        self.contactMomentW = np.zeros(3)

        self.time = 0.

        # log vars
        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_ffwd_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.x_ee_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.contactForceW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.log_counter = 0

        self.ikin = robotKinematics(self.robot, conf.robot_params[self.robot_name]['ee_frame'])

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:, self.log_counter] = self.q
            self.qd_des_log[:, self.log_counter] = self.qd_des
            self.qd_log[:, self.log_counter] = self.qd
            self.tau_ffwd_log[:, self.log_counter] = self.tau_ffwd
            self.tau_log[:, self.log_counter] = self.tau
            self.x_ee_log[:, self.log_counter] = self.x_ee
            self.x_ee_des_log[:, self.log_counter] = self.x_ee_des
            self.contactForceW_log[:, self.log_counter] = self.contactForceW
            self.time_log[self.log_counter] = self.time
            self.log_counter += 1

    def reset_joints(self, q0, joint_names=None):
        # create the message
        req_reset_joints = SetModelConfiguration.Request()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        if joint_names is None:
            req_reset_joints.joint_names = self.joint_names
        else:
            req_reset_joints.joint_names = joint_names
        req_reset_joints.joint_positions = list(map(float, q0))
        self._call_service(self.reset_joints_client, req_reset_joints)
        print(colored(f"---------Resetting Joints to: " + str(q0), "blue"))

    def setSimSpeed(self, dt_sim=0.001, max_update_rate=1000, iters=50):
        physics_req = SetPhysicsProperties.Request()
        physics_req.time_step = dt_sim
        physics_req.max_update_rate = float(max_update_rate)
        physics_req.ode_config.sor_pgs_iters = iters
        physics_req.ode_config.sor_pgs_w = 1.3
        physics_req.ode_config.contact_surface_layer = 0.001
        physics_req.ode_config.contact_max_correcting_vel = 100.
        physics_req.ode_config.erp = 0.2
        physics_req.ode_config.max_contacts = 20
        physics_req.gravity.z = -9.81
        self._call_service(self.set_physics_client, physics_req)


def talker(p):
    p.start()
    p.startSimulator()
    if (robotName == 'ur5'):
        p.loadModelAndPublishers(get_package_share_directory('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro')
    else:
        p.loadModelAndPublishers()
    p.initVars()
    p.startupProcedure()
    p._sleep(1.0)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    # loop frequency
    rate = p.node.create_rate(1 / conf.robot_params[p.robot_name]['dt'])

    # control loop
    while rclpy.ok():
        p.q_des = np.copy(p.q_des_q0)
        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        # log variables
        p.logData()

        # wait for synchronization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4)  # to avoid issues of dt 0.0009999


def main(args=None):
    p = BaseControllerFixed(robotName)

    try:
        talker(p)
    except (KeyboardInterrupt, RuntimeError):
        p.deregister_node()
        if conf.plotting:
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names)
            plotJoint('torque', 1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                      p.tau_ffwd_log, p.joint_names)


if __name__ == '__main__':
    main()
