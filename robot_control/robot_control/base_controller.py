# -*- coding: utf-8 -*-
"""
Created on 3 May 2022
@author: mfocchi

ROS2 (Humble) port of base_controllers/base_controller.py
--------------------------------------------------------------------------
Translation notes (rospy -> rclpy):
  * rospy.init_node / rospy node          -> rclpy.init() + rclpy.create_node(); the
    node is spun by a MultiThreadedExecutor running in a background thread so that
    subscription callbacks fire while the control loop runs (rospy did this
    implicitly).
  * rospy.Publisher(topic, T, queue)      -> node.create_publisher(T, topic, qos)
  * rospy.Subscriber(topic, T, cb)        -> node.create_subscription(T, topic, cb, qos)
  * rospy.ServiceProxy(name, S)           -> node.create_client(S, name); synchronous
    ROS1 calls become create_client + call_async waited on via the executor
    (see _call_service()).
  * tf.TransformBroadcaster                -> tf2_ros.TransformBroadcaster (+ TransformStamped)
  * tf.transformations.euler_from_quaternion -> _euler_from_quaternion() (Pinocchio)
  * roslaunch python API (startSimulator)  -> `ros2 launch ...` via subprocess
  * rospkg.RosPack().get_path(pkg)         -> ament get_package_share_directory(pkg)
  * ros.Time.now() / ros.Duration / ros.Rate / ros.is_shutdown / ros.get_time
    -> node.get_clock() / rclpy.duration.Duration / node.create_rate / rclpy.ok /
       node clock seconds.
  * gazebo_msgs SetPhysicsPropertiesRequest / SetModelStateRequest wrappers become
    the ROS2 `.Request()` factory of the corresponding service type.
All kinematics / dynamics / Pinocchio / control math is unchanged.
"""
from __future__ import print_function

import copy
import os
import subprocess
import threading
import time

import numpy as np
import pinocchio as pin
from termcolor import colored

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile

import tf2_ros
from ament_index_python.packages import get_package_share_directory

from gazebo_msgs.msg import ContactsState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetPhysicsProperties
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ros_impedance_controller.msg import EffortPid

from robot_control.utils.ros_publish import RosPub
from robot_control.utils.pidManager import PidManager
from robot_control.utils.math_tools import *
from robot_control.utils.common_functions import *
import robot_control.params as conf

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)
robotName = "solo"


def _euler_from_quaternion(quaternion):
    """quaternion = [x, y, z, w] -> np.array([roll, pitch, yaw]).
    Replaces tf.transformations.euler_from_quaternion using Pinocchio."""
    q = pin.Quaternion(quaternion[3], quaternion[0], quaternion[1], quaternion[2])
    q.normalize()
    return pin.rpy.matrixToRpy(q.toRotationMatrix())


class BaseController(threading.Thread):
    """
    Simulate floating base robots with an under-actuated base (quadrupeds,
    mobile robots). ROS2 Humble port; see module docstring for the rospy->rclpy
    mapping. All docstrings/attributes are unchanged from the ROS1 version.
    """

    def __init__(self, robot_name="hyq", launch_file=None, external_conf=None, broadcast_world=True):
        threading.Thread.__init__(self)
        if (external_conf is not None):
            conf.robot_params = external_conf.robot_params
        self.robot_name = robot_name

        # rclpy node (spun by a background executor, see startExecutor())
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(self.robot_name + '_base_controller')
        self._executor = None
        self._executor_thread = None

        self.base_offset = np.array([conf.robot_params[self.robot_name]['spawn_x'],
                                     conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z'],
                                     conf.robot_params[self.robot_name].get('spawn_R', 0.),
                                     conf.robot_params[self.robot_name].get('spawn_P', 0.),
                                     conf.robot_params[self.robot_name].get('spawn_Y', 0.)])

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.u = Utils()
        self.math_utils = Math()
        self.verbose = conf.verbose
        self.custom_locosim_launch_file = False
        self.use_ground_truth_contacts = False
        self.apply_external_wrench = False
        self.time_external_wrench = 0.6
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.use_torque_control = False
        self.real_robot = conf.robot_params[self.robot_name].get('real_robot', False)
        self.broadcast_world = broadcast_world
        self.publish_contact_gt_in_wf = False
        print("Initialized basecontroller---------------------------------------------------------------")

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
        # simple wall-clock sleep (control loop uses the ROS clock via create_rate)
        time.sleep(seconds)

    # ------------------------------------------------------------------ startup
    def startSimulator(self, world_name=None, launch_file=None, additional_args=None):
        print(colored('Adding gazebo model path!', 'blue'))
        custom_models_path = get_package_share_directory('ros_impedance_controller') + "/worlds/models/"
        if os.getenv("GAZEBO_MODEL_PATH") is not None:
            os.environ["GAZEBO_MODEL_PATH"] += ":" + custom_models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = custom_models_path

        # clean up previous processes
        os.system("killall rviz2 gzserver gzclient 2>/dev/null")

        cli_args = ["ros2", "launch", "ros_impedance_controller", "ros_impedance_controller.launch.py",
                    'robot_name:=' + self.robot_name,
                    'spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z']),
                    'spawn_R:=' + str(conf.robot_params[self.robot_name].get('spawn_R', 0.)),
                    'spawn_P:=' + str(conf.robot_params[self.robot_name].get('spawn_P', 0.)),
                    'spawn_Y:=' + str(conf.robot_params[self.robot_name].get('spawn_Y', 0.)),
                    'real_robot:=' + str(self.real_robot)]
        if world_name is not None:
            print(colored("Setting custom model: " + str(world_name), "blue"))
            cli_args.append('world_name:=' + str(world_name))
        if additional_args is not None:
            cli_args.extend(additional_args)

        self.sim_process = subprocess.Popen(cli_args)
        self._sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))

    def loadModelAndPublishers(self, xacro_path=None):
        # Loading a robot model (Pinocchio)
        if xacro_path is None:
            xacro_path = get_package_share_directory(
                self.robot_name + '_description') + '/robots/' + self.robot_name + '.urdf.xacro'
        else:
            print("loading custom xacro path: ", xacro_path)
        self.robot = getRobotModelFloating(self.robot_name, xacro_path=xacro_path)

        # visual publisher only (joint states come from the controller)
        self.ros_pub = RosPub(self.robot_name, only_visual=True)

        qos = QoSProfile(depth=1)
        self.pub_des_jstate = self.node.create_publisher(JointState, "/command", qos)

        # gazebo service clients
        self.reset_world = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        self.set_physics_client = self.node.create_client(SetPhysicsProperties, '/gazebo/set_physics_properties')
        self.get_physics_client = self.node.create_client(GetPhysicsProperties, '/gazebo/get_physics_properties')
        self.pause_physics_client = self.node.create_client(Empty, '/gazebo/pause_physics')
        self.unpause_physics_client = self.node.create_client(Empty, '/gazebo/unpause_physics')
        self.apply_body_wrench = self.node.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')

        self.u.putIntoGlobalParamServer("verbose", self.verbose)

    def initSubscribers(self):
        qos = QoSProfile(depth=1)
        # NOTE (ROS1->ROS2 topic mapping): the ROS2 low-level controller_manager
        # publishes joint_states/effort_pid at GLOBAL scope (not under /<robot>/),
        # so subscribe globally. Gazebo-plugin topics below stay under /<robot>/.
        self.sub_jstate = self.node.create_subscription(
            JointState, "/joint_states", self._receive_jstate, qos)
        self.sub_pid_effort = self.node.create_subscription(
            EffortPid, "/effort_pid", self._receive_pid_effort, qos)
        self.sub_pose = self.node.create_subscription(
            Odometry, "/" + self.robot_name + "/ground_truth", self._receive_pose, qos)

        if self.use_ground_truth_contacts:
            self.sub_contact_lf = self.node.create_subscription(
                ContactsState, "/" + self.robot_name + "/lf_foot_bumper", self._receive_contact_lf, qos)
            self.sub_contact_rf = self.node.create_subscription(
                ContactsState, "/" + self.robot_name + "/rf_foot_bumper", self._receive_contact_rf, qos)
            self.sub_contact_lh = self.node.create_subscription(
                ContactsState, "/" + self.robot_name + "/lh_foot_bumper", self._receive_contact_lh, qos)
            self.sub_contact_rh = self.node.create_subscription(
                ContactsState, "/" + self.robot_name + "/rh_foot_bumper", self._receive_contact_rh, qos)

        # start spinning so callbacks fire
        self.startExecutor()

    # ------------------------------------------------------------------ callbacks
    def _receive_contact_lf(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["LF"], grf, self.grForcesLocal_gt)

    def _receive_contact_rf(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["RF"], grf, self.grForcesLocal_gt)

    def _receive_contact_lh(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["LH"], grf, self.grForcesLocal_gt)

    def _receive_contact_rh(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["RH"], grf, self.grForcesLocal_gt)

    def _receive_pose(self, msg):
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w
        self.euler = np.array(_euler_from_quaternion(self.quaternion))
        # unwrap
        self.euler, self.euler_old = unwrap_vector(self.euler, self.euler_old)

        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z
        self.basePoseW[self.u.sp_crd["AX"]] = self.euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = self.euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = self.euler[2]

        self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z

        # compute orientation matrix
        self.b_R_w = self.math_utils.rpyToRot(self.euler)
        if self.broadcast_world:
            self._broadcast_world_transform()

    def _broadcast_world_transform(self):
        t = TransformStamped()
        t.header.stamp = self._now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        lin = self.u.linPart(self.basePoseW)
        t.transform.translation.x = float(lin[0])
        t.transform.translation.y = float(lin[1])
        t.transform.translation.z = float(lin[2])
        t.transform.rotation.x = float(self.quaternion[0])
        t.transform.rotation.y = float(self.quaternion[1])
        t.transform.rotation.z = float(self.quaternion[2])
        t.transform.rotation.w = float(self.quaternion[3])
        self.broadcaster.sendTransform(t)

    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def _receive_pid_effort(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.tau_fb[joint_idx] = msg.effort_pid[msg_idx]

    # ------------------------------------------------------------------ commands
    def send_des_jstate(self, q_des, qd_des, tau_ffwd, soft_limits=0.9, clip_commands=False):
        msg = JointState()
        msg.name = self.joint_names
        if clip_commands:
            msg.position = list(map(float, np.clip(q_des, self.robot.model.lowerPositionLimit[-self.robot.na:] * soft_limits, self.robot.model.upperPositionLimit[-self.robot.na:] * soft_limits)))
            msg.velocity = list(map(float, np.clip(qd_des, -self.robot.model.velocityLimit[-self.robot.na:] * soft_limits, self.robot.model.velocityLimit[-self.robot.na:] * soft_limits)))
            msg.effort = list(map(float, np.clip(tau_ffwd, -self.robot.model.effortLimit[-self.robot.na:] * soft_limits, self.robot.model.effortLimit[-self.robot.na:] * soft_limits)))
        else:
            msg.position = list(map(float, q_des))
            msg.velocity = list(map(float, qd_des))
            msg.effort = list(map(float, tau_ffwd))
        self.pub_des_jstate.publish(msg)

    def deregister_node(self):
        print("deregistering nodes")
        self.ros_pub.deregister_node()
        os.system("pkill -f gzserver 2>/dev/null")
        os.system("pkill -f gzclient 2>/dev/null")
        try:
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    def get_contact(self):
        return self.W_contacts

    def get_pose(self):
        return self.basePoseW

    def get_jstate(self):
        return self.q

    # ------------------------------------------------------------------ gazebo
    def setGravity(self, value):
        physics_props = self._call_service(self.get_physics_client, GetPhysicsProperties.Request())
        if physics_props is None:
            return
        req = SetPhysicsProperties.Request()
        req.time_step = physics_props.time_step
        req.max_update_rate = physics_props.max_update_rate
        req.ode_config = physics_props.ode_config
        req.gravity = physics_props.gravity
        req.gravity.z = value
        self._call_service(self.set_physics_client, req)

    def freezeBase(self, flag, basePoseW=None, baseTwistW=None):
        print(colored("Freezing base", "blue"))
        if flag:
            self.setGravity(0.)
        else:
            self.setGravity(-9.81)
        req_reset_world = SetModelState.Request()
        model_state = ModelState()
        model_state.model_name = self.robot_name
        if basePoseW is None:
            lin = self.u.linPart(self.basePoseW)
            model_state.pose.position.x = float(lin[0])
            model_state.pose.position.y = float(lin[1])
            model_state.pose.position.z = float(lin[2])
            model_state.pose.orientation.x = float(self.quaternion[0])
            model_state.pose.orientation.y = float(self.quaternion[1])
            model_state.pose.orientation.z = float(self.quaternion[2])
            model_state.pose.orientation.w = float(self.quaternion[3])
        else:
            lin = self.u.linPart(basePoseW)
            model_state.pose.position.x = float(lin[0])
            model_state.pose.position.y = float(lin[1])
            model_state.pose.position.z = float(lin[2])
            quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.u.angPart(basePoseW)))
            model_state.pose.orientation.x = float(quaternion.x)
            model_state.pose.orientation.y = float(quaternion.y)
            model_state.pose.orientation.z = float(quaternion.z)
            model_state.pose.orientation.w = float(quaternion.w)

        twist = self.baseTwistW if baseTwistW is None else baseTwistW
        lin = self.u.linPart(twist)
        ang = self.u.angPart(twist)
        model_state.twist.linear.x = float(lin[0])
        model_state.twist.linear.y = float(lin[1])
        model_state.twist.linear.z = float(lin[2])
        model_state.twist.angular.x = float(ang[0])
        model_state.twist.angular.y = float(ang[1])
        model_state.twist.angular.z = float(ang[2])

        req_reset_world.model_state = model_state
        self._call_service(self.reset_world, req_reset_world)

    def mapBaseToWorld(self, B_var):
        W_var = self.b_R_w.transpose().dot(B_var) + self.u.linPart(self.basePoseW)
        return W_var

    def updateKinematics(self):
        self.gen_velocities[:3] = self.b_R_w.dot(self.u.linPart(self.baseTwistW))
        self.gen_velocities[3:6] = self.b_R_w.dot(self.u.angPart(self.baseTwistW))
        self.gen_velocities[6:] = self.qd
        self.neutral_fb_jointstate[7:] = self.q
        pin.forwardKinematics(self.robot.model, self.robot.data, self.neutral_fb_jointstate, self.gen_velocities)
        pin.computeJointJacobians(self.robot.model, self.robot.data)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        ee_frames = conf.robot_params[self.robot_name]['ee_frames']
        for leg in range(4):
            self.B_contacts[leg] = self.robot.framePlacement(self.neutral_fb_jointstate,
                                                             self.robot.model.getFrameId(ee_frames[leg]),
                                                             update_kinematics=False).translation.copy()
            self.W_contacts[leg] = self.mapBaseToWorld(self.B_contacts[leg].transpose())
        if self.use_ground_truth_contacts:
            for leg in range(4):
                self.w_R_lowerleg[leg] = self.b_R_w.transpose().dot(self.robot.data.oMf[self.lowerleg_index[leg]].rotation)

        for leg in range(4):
            self.J[leg] = self.robot.frameJacobian(self.neutral_fb_jointstate,
                                                   self.robot.model.getFrameId(ee_frames[leg]),
                                                   update=False,
                                                   ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, 6+leg*3:6+leg*3+3]
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])
            try:
                self.J_inv[leg] = np.linalg.inv(self.J[leg])
                self.wJ_inv[leg] = self.J_inv[leg].dot(self.b_R_w)
            except np.linalg.linalg.LinAlgError:
                self.J_inv[leg][:, :] = 0.
                self.wJ_inv[leg][:, :] = 0.

        self.configuration[:3] = self.u.linPart(self.basePoseW)
        self.configuration[3:7] = self.quaternion
        self.configuration[7:] = self.q

        self.M = self.robot.mass(self.configuration)
        self.h = pin.nonLinearEffects(self.robot.model, self.robot.data, self.configuration, self.gen_velocities)
        self.h_joints = self.h[6:]
        self.g = self.robot.gravity(self.configuration)
        self.g_joints = self.g[6:]

        self.estimateContactForces()

        self.comPosB, self.comVelB = copy.deepcopy(self.robot.robotComB(self.q, self.qd))
        self.comPoseW = copy.deepcopy(self.basePoseW)
        self.comPoseW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"]+3] = self.robot.robotComW(self.configuration)
        W_base_to_com = self.u.linPart(self.comPoseW) - self.u.linPart(self.basePoseW)
        self.comTwistW = pin.SE3(np.eye(3), W_base_to_com).action.dot(self.baseTwistW)

        self.centroidalInertiaB = self.robot.centroidalInertiaB(self.configuration, self.gen_velocities)
        self.compositeRobotInertiaB = self.robot.compositeRobotInertiaB(self.configuration)

    def estimateContactForces(self):
        for leg in range(4):
            grf = self.wJ_inv[leg].T.dot(self.u.getLegJointState(leg, self.h_joints - self.tau))
            self.u.setLegJointState(leg, grf, self.grForcesW)
            if self.contact_normal[leg].dot(grf) >= conf.robot_params[self.robot_name]['force_th']:
                self.contact_state[leg] = True
            else:
                self.contact_state[leg] = False

        if self.use_ground_truth_contacts:
            for leg in range(4):
                grfLocal_gt = self.u.getLegJointState(leg, self.grForcesLocal_gt)
                if self.publish_contact_gt_in_wf:
                    grf_gt = grfLocal_gt
                else:
                    grf_gt = self.w_R_lowerleg[leg] @ grfLocal_gt
                self.u.setLegJointState(leg, grf_gt, self.grForcesW_gt)
                if self.contact_normal[leg].dot(grf_gt) >= conf.robot_params[self.robot_name]['force_th']:
                    self.contact_state[leg] = True
                else:
                    self.contact_state[leg] = False

    def applyForce(self, Fx, Fy, Fz, Mx, My, Mz, duration, link_name="base_link"):
        from geometry_msgs.msg import Wrench, Point
        req = ApplyBodyWrench.Request()
        req.body_name = self.robot_name + "::" + link_name
        req.reference_frame = "world"
        req.reference_point = Point(x=0., y=0., z=0.)
        wrench = Wrench()
        wrench.force.x = float(Fx)
        wrench.force.y = float(Fy)
        wrench.force.z = float(Fz)
        wrench.torque.x = float(Mx)
        wrench.torque.y = float(My)
        wrench.torque.z = float(Mz)
        req.wrench = wrench
        if duration >= 0:
            req.duration = Duration(seconds=duration).to_msg()
        self._call_service(self.apply_body_wrench, req)

    def startupProcedure(self):
        self.pid = PidManager(self.node, self.joint_names)
        self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'],
                             conf.robot_params[self.robot_name]['kd'],
                             np.zeros(self.robot.na))

        if (self.robot_name == 'hyq'):
            self.gravity_comp = np.array(
                [24.2571, 1.92, 50.5, 21.3801, -2.08377, -44.9598, 24.2, 1.92, 50.5739, 21.3858, -2.08365, -44.9615])
            print("reset posture...")
            self.freezeBase(1, basePoseW=self.base_offset)
            start_t = self._time_sec()
            while self._time_sec() - start_t < 1.0:
                self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                self._sleep(0.01)
            if self.verbose:
                print("q err prima freeze base", (self.q - self.q_des))
            print("put on ground and start compensating gravity...")
            self.freezeBase(0)
            self._sleep(0.5)
            if self.verbose:
                print("q err pre grav comp", (self.q - self.q_des))
            start_t = self._time_sec()
            while self._time_sec() - start_t < 1.0:
                self.send_des_jstate(self.q_des, self.qd_des, self.gravity_comp)
                self._sleep(0.01)
            if self.verbose:
                print("q err post grav comp", (self.q - self.q_des))
            print("starting com controller (no joint PD)...")
            self.pid.setPDs(0.0, 0.0, 0.0)
            self.use_torque_control = True

        if (self.robot_name == 'aliengo' or self.robot_name == 'go1'):
            start_t = self._time_sec()
            while self._time_sec() - start_t < 0.5:
                self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                self._sleep(0.01)

        if (self.robot_name == 'solo'):
            start_t = self._time_sec()
            while self._time_sec() - start_t < 0.5:
                self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                self._sleep(0.01)
        print(colored("finished startup -- starting controller", "red"))

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.comPoseW = np.zeros(6)
        self.comTwistsW = np.zeros(6)
        self.stance_legs = np.array([True, True, True, True])
        self.centroidalInertiaB = np.eye(3)
        self.compositeRobotInertiaB = np.eye(3)
        self.gen_velocities = np.zeros(self.robot.nv)
        self.neutral_fb_jointstate = pin.neutral(self.robot.model)
        self.configuration = pin.neutral(self.robot.model)
        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.tau_fb = np.zeros(self.robot.na)
        self.q_des = np.zeros(self.robot.na)
        self.quaternion = np.array([0., 0., 0., 1.])
        self.euler_old = np.zeros(3)
        self.q_des = conf.robot_params[self.robot_name]['q_0']
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd = np.zeros(self.robot.na)
        self.gravity_comp = np.zeros(self.robot.na)
        self.b_R_w = np.eye(3)
        self.grForcesW = np.zeros(self.robot.na)
        self.grForcesLocal_gt = np.zeros(self.robot.na)
        self.grForcesW_gt = np.zeros(self.robot.na)

        self.J = self.u.listOfArrays(4, np.zeros((3, 3)))
        self.wJ = self.u.listOfArrays(4, np.zeros((3, 3)))
        self.J_inv = self.u.listOfArrays(4, np.zeros((3, 3)))
        self.wJ_inv = self.u.listOfArrays(4, np.zeros((3, 3)))
        self.W_contacts = self.u.listOfArrays(4, np.zeros((3, 3)))
        self.W_contacts_des = self.u.full_listOfArrays(4, 3)
        self.B_contacts = self.u.listOfArrays(4, np.zeros((3, 3)))
        self.B_contacts_des = self.u.full_listOfArrays(4, 3)
        self.contact_state = self.u.full_listOfArrays(4, 1, 0, False)
        self.contact_normal = self.u.listOfArrays(4, np.array([0., 0., 1]))
        self.w_R_lowerleg = self.u.listOfArrays(4, np.eye(3))

        # log vars
        bufsize = conf.robot_params[self.robot_name]['buffer_size']
        self.basePoseW_log = np.full((6, bufsize), np.nan)
        self.baseTwistW_log = np.full((6, bufsize), np.nan)
        self.comPoseW_log = np.full((6, bufsize), np.nan)
        self.comTwistW_log = np.full((6, bufsize), np.nan)
        self.q_des_log = np.full((self.robot.na, bufsize), np.nan)
        self.q_log = np.full((self.robot.na, bufsize), np.nan)
        self.qd_des_log = np.full((self.robot.na, bufsize), np.nan)
        self.qd_log = np.full((self.robot.na, bufsize), np.nan)
        self.tau_ffwd_log = np.full((self.robot.na, bufsize), np.nan)
        self.tau_log = np.full((self.robot.na, bufsize), np.nan)
        self.grForcesW_log = np.full((self.robot.na, bufsize), np.nan)
        self.time_log = np.full((bufsize), np.nan)
        self.constr_viol_log = np.full((4, bufsize), np.nan)

        self.time = np.zeros(1)
        self.loop_time = conf.robot_params[self.robot_name]['dt']
        self.log_counter = 0

        # order: lf lh rf rh
        if self.use_ground_truth_contacts:
            self.lowerleg_index = [0]*4
            self.lowerleg_frame_names = []
            for f in self.robot.model.frames:
                if 'lower' in f.name:
                    self.lowerleg_frame_names.append(f.name)
            self.lowerleg_frame_names = self.u.mapLegListToRos(self.lowerleg_frame_names)
            for legid in self.u.leg_map.keys():
                leg = self.u.leg_map[legid]
                self.lowerleg_index[leg] = self.robot.model.getFrameId(self.lowerleg_frame_names[leg])

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.basePoseW_log[:, self.log_counter] = self.basePoseW
            self.baseTwistW_log[:, self.log_counter] = self.baseTwistW
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:, self.log_counter] = self.q
            self.qd_des_log[:, self.log_counter] = self.qd_des
            self.qd_log[:, self.log_counter] = self.qd
            self.tau_ffwd_log[:, self.log_counter] = self.tau_ffwd
            self.tau_log[:, self.log_counter] = self.tau
            self.grForcesW_log[:, self.log_counter] = self.grForcesW
            self.time_log[self.log_counter] = self.time
            self.log_counter += 1

    def sync_check(self):
        new_time = self._time_sec()
        if hasattr(self, 'ros_time'):
            self.loop_time = new_time - self.ros_time
        self.ros_time = new_time

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
    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()

    # loop frequency
    rate = p.node.create_rate(1 / conf.robot_params[p.robot_name]['dt'])

    while rclpy.ok():
        p.updateKinematics()

        if p.use_torque_control:
            p.pid.setPDs(0, 0, 0)
            p.tau_ffwd = conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des, p.q) - conf.robot_params[p.robot_name]['kd'] * p.qd + p.gravity_comp
        else:
            p.tau_ffwd = np.zeros(p.robot.na)

        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd, clip_commands=p.real_robot)

        p.logData()

        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.contact_state[leg] * p.u.getLegJointState(leg, p.grForcesW / (6 * p.robot.robotMass)), "green")
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW_gt / (6 * p.robot.robotMass)), "red")
        p.ros_pub.publishVisual()

        rate.sleep()
        p.time = np.round(p.time + np.array([p.loop_time]), 4)

        if (p.apply_external_wrench and p.time > p.time_external_wrench):
            print("START APPLYING EXTERNAL WRENCH")
            p.applyForce(0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.05)
            p.apply_external_wrench = False


def main(args=None):
    p = BaseController(robotName)
    try:
        talker(p)
    except (KeyboardInterrupt, RuntimeError):
        p.deregister_node()
        if conf.plotting:
            plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
            plotFrame('position', time_log=p.time_log, Pose_log=p.basePoseW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, joint_names=p.joint_names)


if __name__ == '__main__':
    main()
