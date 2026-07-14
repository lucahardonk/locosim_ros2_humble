# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 04-12-2022
#
# ROS2 (Humble) port of base_controllers/components/controller_manager.py
# --------------------------------------------------------------------------
# Translation notes (rospy -> rclpy):
#   * rospy.Publisher(...)         -> node.create_publisher(msg, topic, qos)
#   * rospy.ServiceProxy(...)      -> node.create_client(srv, name)
#   * controller_manager_msgs SwitchController/LoadController:
#       - ROS1 request fields  start_controllers/stop_controllers
#         become in ROS2 Humble  activate_controllers/deactivate_controllers.
#       - strictness enum BEST_EFFORT == 1 (SwitchController.Request.BEST_EFFORT).
# The control logic (which controller to load/switch, message payloads) is kept
# identical to the ROS1 version.
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from termcolor import colored

# controller manager management (same package name in ROS2)
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController

from base_controllers.components.gripper_manager import GripperManager


class ControllerManager():
    def __init__(self, node: Node, robot_name, conf):
        self.node = node
        self.robot_name = robot_name
        self.conf = conf
        self.control_type = conf['control_type']
        self.gripper_sim = conf['gripper_sim']
        self.gripper_type = conf['gripper_type']
        self.real_robot = conf['real_robot']
        self.number_of_joints = len(conf['joint_names'])

        if (self.control_type == 'torque'):
            print(colored("Controller Manager: torque", "blue"))
        if (self.control_type == 'position'):
            print(colored("Controller Manager: position", "blue"))

    def initPublishers(self, robot_name):
        qos = QoSProfile(depth=1)
        # publisher for ros_impedance_controller
        self.pub_full_jstate = self.node.create_publisher(JointState, "/command", qos)
        # specific publisher for joint_group_pos_controller that publishes only position
        self.pub_reduced_des_jstate = self.node.create_publisher(
            Float64MultiArray, "/" + robot_name + "/joint_group_pos_controller/command", 10)

        self.switch_controller_srv = self.node.create_client(
            SwitchController, "/" + self.robot_name + "/controller_manager/switch_controller")
        self.load_controller_srv = self.node.create_client(
            LoadController, "/" + self.robot_name + "/controller_manager/load_controller")

        #  different controllers are available from the real robot and in simulation in case of position control
        if self.real_robot:
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller"]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller"]
        self.active_controller = self.available_controllers[0]

        # switch to the selected controller
        if (self.conf['control_mode'] == "trajectory"):
            if (self.real_robot):
                self.switch_controller("scaled_pos_joint_traj_controller")
            else:
                self.switch_controller("pos_joint_traj_controller")
        else:  # control_mode point
            if self.control_type == 'position':
                self.switch_controller("joint_group_pos_controller")

        # instantiate the gripper manager that will read soft gripper param from param server
        self.gm = GripperManager(self.gripper_type, self.real_robot, self.conf['dt'], node=self.node)

    def send_full_jstate(self, q_des, qd_des, tau_ffwd):
        # No need to change the convention because in the HW interface we use our convention (see ros_impedance_controller_xx.yaml)
        msg = JointState()
        if self.gripper_sim:
            msg.position = np.append(q_des, self.gm.getDesGripperJoints())
            msg.velocity = np.append(qd_des, np.zeros(self.gm.number_of_fingers))
            msg.effort = np.append(tau_ffwd, np.zeros(self.gm.number_of_fingers))
        else:
            msg.position = q_des
            msg.velocity = qd_des
            msg.effort = tau_ffwd
        self.pub_full_jstate.publish(msg)

    def send_reduced_des_jstate(self, q_des):
        msg = Float64MultiArray()
        if self.gripper_sim and not self.real_robot:
            msg.data = np.append(q_des, self.gm.getDesGripperJoints())
        else:
            msg.data = q_des
        self.pub_reduced_des_jstate.publish(msg)

    def sendReference(self, q_des, qd_des=None, tau_ffwd=None):
        if (self.control_type == 'torque'):
            if qd_des is None:
                qd_des = np.zeros(self.number_of_joints)
            if tau_ffwd is None:
                tau_ffwd = np.zeros(self.number_of_joints)
            self.send_full_jstate(q_des, qd_des, tau_ffwd)
        else:
            self.send_reduced_des_jstate(q_des)

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        print('Available controllers: ', self.available_controllers)
        print('Controller manager: loading ', target_controller)

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        print('Controller manager:Switching off  :  ', other_controllers)

        # Load the target controller
        load_req = LoadController.Request()
        load_req.name = target_controller
        self.load_controller_srv.wait_for_service()
        self.load_controller_srv.call_async(load_req)

        # Switch controllers (ROS2 field names: activate/deactivate_controllers)
        switch_req = SwitchController.Request()
        switch_req.deactivate_controllers = other_controllers
        switch_req.activate_controllers = [target_controller]
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        self.switch_controller_srv.wait_for_service()
        self.switch_controller_srv.call_async(switch_req)
        self.active_controller = target_controller
