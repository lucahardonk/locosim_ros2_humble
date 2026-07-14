# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino

ROS2 (Humble) port of
base_controllers/climbingrobot_controller/climbingrobot_controller2.py
----------------------------------------------------------------------
This is the two-rope ALPINE climbing robot controller (MPC + propellers).
Subclasses the ROS2 ``BaseControllerFixed``. The rospy -> rclpy translation
mirrors base_controller_fixed.py and the sibling climbingrobot_controller.py:
  * ``import rospy as ros``                 -> use self.node (rclpy) + helpers.
  * ros.Rate / ros.is_shutdown / ros.Time  -> node.create_rate / rclpy.ok / self._now.
  * tf.TransformBroadcaster.sendTransform  -> tf2_ros.TransformBroadcaster + TransformStamped
                                              (see _send_tf()).
  * rospy.ServiceProxy / proxy calls        -> node.create_client + _call_service().
  * apply_body_wrench proxy call            -> ApplyBodyWrench.Request() + _call_service().
  * pause/unpause proxy calls               -> _call_service(pause_physics_client, Empty.Request()).

MATLAB engine -> climbingrobot_optimization
-------------------------------------------
The original controller called a MATLAB engine (``self.eng.optimize_cpp_mex`` /
``optimize_cpp_mpc*_mex``). This port replaces those calls with the pure-Python
``climbingrobot_optimization`` package (which uses a compiled C++ kernel when
available, and a NumPy fallback otherwise):
  * eng.optimize_cpp_mex(p0, pf, Fleg_max, Fr_max, Fr_min, mu, optim_params)
        -> jump_optimizer.optimize_cpp(p0, pf, Fleg_max, Fr_max, mu, params)
           (NOTE: there is no separate Fr_min argument; the lower bound is -Fr_max)
  * eng.optimize_cpp_mpc_mex(...)                 -> mpc_controller.optimize_cpp_mpc(...)
  * eng.optimize_cpp_mpc_no_constraints_mex(...)  -> mpc_controller.optimize_cpp_mpc_no_constraints(...)
  * eng.optimize_cpp_mpc_propellers_mex(...)      -> mpc_controller.optimize_cpp_mpc_propellers(...)
The MATLAB ``optim_params`` struct is replaced by a ``Params`` dataclass; the
returned MATLAB struct fields (p, psi, l1, l2, time, Fr_l, Fr_r, Fleg, Tf,
T_th, achieved_target) map 1:1 to keys of the returned ``solution`` dict.

FIRST-PASS PORT: this is gazebo-coupled and must be runtime tested. It requires
the climbingrobot_description package and the climbingrobot_optimization package
to be built/available. All kinematics / dynamics / Pinocchio / control math is
unchanged.

Not ported (intentionally, matching the optimization-package conversion notes):
  * rosbag recording (SAVE_BAG) and the pandas noise-sweep CSV logging (ADD_NOISE)
    are heavy offline-analysis paths; the relevant imports are done lazily so the
    default configuration (SAVE_BAG=False, ADD_NOISE=False) runs without them.
"""

from __future__ import print_function

import os
import math

import numpy as np
import numpy.matlib  # noqa: F401  (provides numpy.matlib.repmat used below)
import scipy.linalg  # noqa: F401
from scipy.linalg import block_diag
from numpy import nan
import pinocchio as pin
import matplotlib.pyplot as plt
import scipy.io.matlab as mio
from termcolor import colored

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Wrench, Point
from gazebo_msgs.msg import ContactsState
from std_srvs.srv import Empty

from base_controllers.utils.math_tools import *
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear
from base_controllers.base_controller_fixed import BaseControllerFixed
import base_controllers.params as conf

# jump optimization / MPC backend (replaces the MATLAB engine calls)
from climbingrobot_optimization import jump_optimizer, mpc_controller
from climbingrobot_optimization.params import Params

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)
robotName = "climbingrobot2"


class ClimbingrobotController(BaseControllerFixed):
    def __init__(self, robot_name="ur5"):
        self.EXTERNAL_FORCE = False
        self.landing = False  # using landing controller
        self.impedance_landing = True
        self.MPC_control = True
        self.PLOT_MPC = False
        self.type_of_disturbance = 'none'  # 'none', 'impulse', 'const'
        self.MPC_uses_constraints = True
        self.PROPELLERS = True
        self.TYPE_OF_JUMP = 'upward'  # 'upward', 'downward'
        self.USE_PROPELLERS_FOR_LEG_REORIENT = False  # true use propeller to reorient the leg
        self.JUMP_LENGTH_MULTIPLIER = 1.  # 0.75 /1 /1.25
        self.MULTIPLE_JUMPS = False  # use this for paper to generate targets in an ellipsoid around p0,
        self.SAVE_BAG = False  # does not show rope vectors
        self.ADD_NOISE = False  # creates multiple jumps / adds noise to velocity or disturbance
        self.OBSTACLE_AVOIDANCE = False
        self.obstacle_location = np.array([0, 2.5, -6])
        self.obstacle_size = np.array([1.5, 1.5, 0.866])

        self.rope_index = np.array([2, 8])  # 'wire_base_prismatic_r', 'wire_base_prismatic_l',
        self.leg_index = np.array([12, 13, 14])
        self.wheel_index = np.array([16, 18])  # 'wheel_joint_l',  'wheel_joint_r'
        self.hip_pitch_joint = 12
        self.hip_roll_joint = 13
        self.base_passive_joints = np.array([3, 4, 5, 9, 10, 11])
        self.anchor_passive_joints = np.array([0, 1, 6, 7])
        self.impulse_start_count = 0  # start disturbance at different point of the flight phase

        if self.landing == True:
            robot_name += 'landing'
            self.force_scale = 150.
        else:
            self.landing = False
            self.force_scale = 60.
        self.landing_joints = np.array([15, 17])
        self.mountain_thickness = 0.1  # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        super().__init__(robot_name=robot_name)
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def _send_tf(self, translation, quat, child, parent):
        """ROS2 replacement for tf.TransformBroadcaster.sendTransform(trans, quat,
        time, child, parent). quat is (x, y, z, w)."""
        t = TransformStamped()
        t.header.stamp = self._now().to_msg()
        t.header.frame_id = parent.lstrip('/')
        t.child_frame_id = child.lstrip('/')
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.broadcaster.sendTransform(t)

    def apply_propeller_moment(self, Mz):
        # create force per to ropes plane
        arm = np.linalg.norm(self.hoist_l_pos - self.base_pos)
        force = self.w_R_b[:, 0] * Mz / (2 * arm)
        self.ros_pub.add_arrow(self.hoist_l_pos, force / (10 * self.force_scale), "green", scale=1.5)  # left should be positive
        self.ros_pub.add_arrow(self.hoist_r_pos, -force / (10 * self.force_scale), "green", scale=1.5)  # right should be negative
        wrench = Wrench()
        wrench.force.x = 0.
        wrench.force.y = 0.
        wrench.force.z = 0.
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = float(Mz)
        self.pub_prop_force.publish(wrench)

    def apply_propeller_force(self, ext_force):
        # create force per to ropes plane
        self.prop_forceW = self.n_bar * ext_force
        self.ros_pub.add_arrow(self.base_pos, self.prop_forceW / self.force_scale, "blue", scale=1.5)
        wrench = Wrench()
        wrench.force.x = float(self.prop_forceW[0])
        wrench.force.y = float(self.prop_forceW[1])
        wrench.force.z = float(self.prop_forceW[2])
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = 0.
        self.pub_prop_force.publish(wrench)

    def applyWrench(self, Fx=0, Fy=0, Fz=0, Mx=0, My=0, Mz=0, time_interval=0, start_time=None):
        from gazebo_msgs.srv import ApplyBodyWrench
        wrench = Wrench()
        wrench.force.x = float(Fx)
        wrench.force.y = float(Fy)
        wrench.force.z = float(Fz)
        wrench.torque.x = float(Mx)
        wrench.torque.y = float(My)
        wrench.torque.z = float(Mz)
        reference_frame = "world"  # you can apply forces only in this frame because this service is buggy
        req = ApplyBodyWrench.Request()
        req.body_name = self.robot_name + "::base_link"
        req.reference_frame = reference_frame
        req.reference_point = Point(x=0., y=0., z=0.)
        req.wrench = wrench
        if start_time is not None:
            # start_time is a builtin_interfaces/Time message (see talker)
            req.start_time = start_time
        req.duration = Duration(seconds=float(time_interval)).to_msg()
        self._call_service(self.apply_body_wrench, req)

    def loadModelAndPublishers(self, xacro_path=None):
        xacro_path = get_package_share_directory('climbingrobot_description') + '/urdf/' + self.robot_name + '.xacro'
        # Pass each xacro arg as a separate list element so that
        # _parse_xacro_mappings splits them correctly (the old concatenated-
        # string form caused only the first key:=value to be parsed).
        additional_urdf_args = 'anchorX:=' + str(conf.robot_params[self.robot_name]['spawn_x'])
        additional_urdf_args += ' anchorY:=' + str(conf.robot_params[self.robot_name]['spawn_y'])
        additional_urdf_args += ' anchorZ:=' + str(conf.robot_params[self.robot_name]['spawn_z'])
        additional_urdf_args += ' anchor2X:=' + str(conf.robot_params[self.robot_name]['spawn_2x'])
        additional_urdf_args += ' anchor2Y:=' + str(conf.robot_params[self.robot_name]['spawn_2y'])
        additional_urdf_args += ' anchor2Z:=' + str(conf.robot_params[self.robot_name]['spawn_2z'])
        super().loadModelAndPublishers(xacro_path=xacro_path, additional_urdf_args=additional_urdf_args,
                                       markers_time_to_live=conf.robot_params[self.robot_name]['dt'])

        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        qos = QoSProfile(depth=1)
        self.sub_contact = self.node.create_subscription(
            ContactsState, "/" + self.robot_name + "/foot_bumper", self._receive_contact, qos)
        if self.landing:
            self.sub_contact_l = self.node.create_subscription(
                ContactsState, "/" + self.robot_name + "/foot_landing_l_bumper", self._receive_contact_landing_l, qos)
            self.sub_contact_r = self.node.create_subscription(
                ContactsState, "/" + self.robot_name + "/foot_landing_r_bumper", self._receive_contact_landing_r, qos)
        # NOTE: the MATLAB engine start (self.eng = matlab.engine.start_matlab()) is
        # removed; the optimizer is now the pure-Python climbingrobot_optimization package.
        if self.PROPELLERS:
            self.pub_prop_force = self.node.create_publisher(Wrench, "/base_force", qos)
        if self.SAVE_BAG:
            print(colored("SAVE_BAG requested but rosbag recording is not ported to ROS2 in this controller.", "yellow"))
        if self.ADD_NOISE:
            import pandas as pd  # lazy import: only the offline noise-sweep path needs pandas
            try:
                os.system('rm *.csv')
            except Exception:
                pass
            print(colored('CREATING NEW CSV TO STORE NOISE TESTS', 'blue'))
            columns = ['test_nr', 'ideal_target', 'optim_target', 'landing_location', 'landing_error', 'relative_error', 'energy', 'rmse']
            if self.type_of_disturbance != 'none':
                columns.append('base_dist')
            self.df = pd.DataFrame(columns=columns)

    def getRobotMass(self):
        robot_link_masses = []
        # get link masses supported by joints
        for idx in self.robot.model.inertias:
            robot_link_masses.append(idx.mass)
        # the robot is supported after this joint
        total_robot_mass = sum(robot_link_masses[self.robot.model.getJointId('wire_base_yaw_l'):])
        return total_robot_mass

    def generateDisturbanceOnHemiSphere(self, min, max):
        # sample_spherical(npoints, ndim=3):
        direction = np.random.randn(3)
        direction /= np.linalg.norm(direction, axis=0)
        # hemisphere
        if direction[2] > 0:
            direction[2] *= -1
        # sample magnitude
        amp = min + max * np.random.uniform(low=0, high=1, size=1)
        return amp * direction

    def generateWindDisturbance(self, n_test, amp):
        directions = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]])
        return amp * directions[n_test, :]

    def updateKinematicsDynamics(self):

        if self.ADD_NOISE:  # add white gaussian noise
            # it is better to scale the 5% i.e, the variance after sampling with a higher variance value
            self.state_der_noise = np.array([0.01, 0.2, 0.2]) * np.random.normal(0., 1)
        else:
            self.state_der_noise = np.zeros(3)

        # q is continuously updated
        self.robot.computeAllTerms(self.q, self.qd)
        # joint space inertia matrix
        self.M = self.robot.mass(self.q)
        # bias terms
        self.h = self.robot.nle(self.q, self.qd)
        # gravity terms
        self.g = self.robot.gravity(self.q)
        # compute ee position  in the world frame
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in a workdframe with the origin attached to the base frame origin
        self.anchor_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor')).translation
        self.anchor_pos2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('anchor_2')).translation
        self.anchor_distance_y = (self.anchor_pos2 - self.anchor_pos)[1]
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation

        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.base_rpy = self.math_utils.rot2eul(self.w_R_b)

        self.Jb = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('base_link'), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self.omega_b = self.Jb[3:, :].dot(self.qd)
        self.base_vel = self.Jb[:3, :].dot(self.qd)

        self.hoist_l_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, -0.05, 0.05]))
        self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, 0.05, 0.05]))

        self.rope_direction = (p.hoist_l_pos - p.anchor_pos) / np.linalg.norm(p.hoist_l_pos - p.anchor_pos)
        self.rope_direction2 = (p.hoist_r_pos - p.anchor_pos2) / np.linalg.norm(p.hoist_r_pos - p.anchor_pos2)

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
        self.Jleg = self.J[:, self.leg_index]
        self.dJdq = self.robot.frameClassicAcceleration(self.q, self.qd, None, self.robot.model.getFrameId(frame_name), False).linear

        w_R_wire = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire')).rotation
        w_R_wire2 = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire_2')).rotation

        self.mat2Gazebo = self.anchor_pos
        self.base_pos_mat = self.base_pos - self.mat2Gazebo
        self.psi = math.atan2(self.base_pos_mat[0], -self.base_pos_mat[2])
        # use geometric intuition for psid
        n_par = (self.anchor_pos - self.anchor_pos2) / np.linalg.norm(self.anchor_pos - self.anchor_pos2)
        rope2_axis = (self.base_pos - self.anchor_pos2) / np.linalg.norm(self.base_pos - self.anchor_pos2)
        self.n_bar = np.cross(n_par, rope2_axis) / np.linalg.norm(np.cross(n_par, rope2_axis))
        self.psid = (self.n_bar.dot(self.base_vel)) / np.linalg.norm(
            np.cross(n_par, self.base_pos - self.anchor_pos2)) + self.state_der_noise[0]

        # WF matlab to WF Gazebo offset
        hoist_distance = np.linalg.norm(self.hoist_l_pos - self.hoist_r_pos)

        # to get the matlab state from the gazebo prismatic joints we need to consider that the gazebo joints is in zero config
        # when the rope is 2.5 m half of anchor distance (startup at the point in the middle of the anchors)
        self.l_1 = self.q[p.rope_index[1]] - hoist_distance / 2 + self.anchor_distance_y / 2
        self.l_1d = self.qd[p.rope_index[1]] + self.state_der_noise[1]
        self.l_2 = self.q[p.rope_index[0]] - hoist_distance / 2 + self.anchor_distance_y / 2
        self.l_2d = self.qd[p.rope_index[0]] + self.state_der_noise[2]

        # compute com variables
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)

        # from ground truth
        self.com = self.robot.robotComW(self.q)

        # the mountain is always wrt to world
        mountain_pos = np.array([- self.mountain_thickness / 2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        self._send_tf(mountain_pos, (0.0, 0.0, 0.0, 1.0), '/wall', '/world')
        if self.OBSTACLE_AVOIDANCE:
            self._send_tf(mountain_pos, (0.0, 0.0, 0.0, 1.0), '/pillar', '/world')

        if p.landing:
            self.x_landing_l = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wheel_l')).translation
            self.x_landing_r = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wheel_r')).translation
            # kinematics of middle point
            self.x_p = 0.5 * (self.x_landing_l + self.x_landing_r)
            self.J_landing_l = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('wheel_l'), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
            self.J_landing_r = self.robot.frameJacobian(self.q, self.robot.model.getFrameId('wheel_r'), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]

    def _receive_contact(self, msg):
        self.contactForceW = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW = self.robot.framePlacement(self.q, self.robot.model.getFrameId("lower_link")).rotation.dot(grf)

    def _receive_contact_landing_l(self, msg):
        self.contactForceW_l = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW_l = self.robot.framePlacement(self.q, self.robot.model.getFrameId("wheel_l")).rotation.dot(grf)

    def _receive_contact_landing_r(self, msg):
        self.contactForceW_r = np.zeros(3)
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW_r = self.robot.framePlacement(self.q, self.robot.model.getFrameId("wheel_r")).rotation.dot(grf)

    def initVars(self):
        super().initVars()
        self.contactForceW_l = np.zeros(3)
        self.contactForceW_r = np.zeros(3)
        self.qdd_des = np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        self.base_rpy = np.zeros(3)
        self.Fr_l_fbk = 0
        self.Fr_r_fbk = 0
        self.Fr_l = 0
        self.Fr_r = 0
        self.prop_force = 0
        self.touch_down_detected_l = False
        self.touch_down_detected_r = False
        self.optimal_control_traj_finished = False
        self.MPC_tracking_error = []

        # init new logged vars here
        self.com_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.simp_model_state_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_rpy_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.time_jump_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_1d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_2d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.psid_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_vel_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.prop_force_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        w_R_wall = self.math_utils.eul2Rot(np.array([0, -conf.robot_params[p.robot_name]['wall_inclination'], 0]))
        self.wall_normal = w_R_wall[:, 0].copy()  # take X axis, use copy to keep it contiguous

        self.mpc_index = 0
        self.mpc_index_old = 0
        self.mpc_index_ffwd = 0  # updated only when we stop recomputing mpc

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.simp_model_state_log[:, self.log_counter] = np.array([self.psi, self.l_1, self.l_2])
            self.base_pos_log[:, self.log_counter] = self.base_pos
            self.base_rpy_log[:, self.log_counter] = self.base_rpy
            self.Fr_l_log[self.log_counter] = self.Fr_l
            self.Fr_r_log[self.log_counter] = self.Fr_r
            self.Fr_l_fbk_log[self.log_counter] = self.Fr_l_fbk
            self.Fr_r_fbk_log[self.log_counter] = self.Fr_r_fbk
            self.l_1d_log[self.log_counter] = self.l_1d
            self.l_2d_log[self.log_counter] = self.l_2d
            self.psid_log[self.log_counter] = self.psid
            self.base_vel_log[:, self.log_counter] = self.base_vel
            if self.PROPELLERS:
                self.prop_force_log[self.log_counter] = self.prop_force

        super().logData()

    def deregister_node(self):
        super().deregister_node()
        os.system("pkill -9 gzserver 2>/dev/null")
        os.system("pkill -9 gzclient 2>/dev/null")

    def startupProcedure(self):
        # set PD gains
        super().startupProcedure()

    def plotStuff(self):
        if p.numberOfJumps < 2:  # do plots only for one jump
            print("PLOTTING")
            print(colored("The initial p0_x and mountain_pitch can be different by the desired ones computed by optim, even if we started optim from actual p0, "
                          "because the robot sags a bit due to leg reorientation", "red"))
            actual_com = p.base_pos_log - p.mat2Gazebo.reshape(3, 1)  # mat2Gazebo is WF in matlab
            time_gazebo = p.time_log - p.start_logging
            plotJoint('position', time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            if not p.MULTIPLE_JUMPS:
                plot3D('basePos', 2, ['X', 'Y', 'Z'], time_gazebo, actual_com, p.ref_time, p.ref_com)
            plot3D('states_test_' + str(p.n_test), 3, ['psi', 'l1', 'l2'], time_gazebo, p.simp_model_state_log, p.ref_time, np.vstack((p.ref_psi, p.ref_l_1, p.ref_l_2)))

            # plot wheels
            if p.landing:
                plt.figure()
                plt.subplot(2, 1, 1)
                plt.ylabel("wheel_pos_l")
                plt.plot(time_gazebo, p.q_des_log[p.wheel_index[0], :], color='red')
                plt.plot(time_gazebo, p.q_log[p.wheel_index[0], :], color='blue')
                plt.grid()
                plt.subplot(2, 1, 2)
                plt.ylabel("wheel_pos_r")
                plt.plot(time_gazebo, p.q_des_log[p.wheel_index[1], :], color='red')
                plt.plot(time_gazebo, p.q_log[p.wheel_index[1], :], color='blue')
                plt.grid()

                plt.figure()
                plt.subplot(2, 1, 1)
                plt.ylabel("wheel_vel_l")
                plt.plot(time_gazebo, p.qd_des_log[p.wheel_index[0], :], color='red')
                plt.plot(time_gazebo, p.qd_log[p.wheel_index[0], :], color='blue')
                plt.grid()
                plt.subplot(2, 1, 2)
                plt.ylabel("wheel_vel_r")
                plt.plot(time_gazebo, p.qd_des_log[p.wheel_index[1], :], color='red')
                plt.plot(time_gazebo, p.qd_log[p.wheel_index[1], :], color='blue')
                plt.grid()

            # save data
            if p.MPC_control:
                filename = f'test_gazebo_MPC_{p.MPC_control}_constraints_{p.MPC_uses_constraints}_dist_{p.type_of_disturbance}.mat'
            else:
                filename = f'test_gazebo_MPC_{p.MPC_control}_dist_{p.type_of_disturbance}.mat'
            mio.savemat(filename, {'ref_time': p.ref_time, 'ref_com': p.ref_com,
                                   'time_gazebo': time_gazebo, 'actual_com': actual_com,
                                   'ref_psi': p.ref_psi, 'ref_l_1': p.ref_l_1, 'ref_l_2': p.ref_l_2,
                                   'psi': p.simp_model_state_log[0, :], 'l_1': p.simp_model_state_log[1, :], 'l_2': p.simp_model_state_log[2, :],
                                   'psid': p.psid_log, 'l_1d': p.l_1d_log, 'l_2d': p.l_2d_log,
                                   'mu': p.mu, 'Fleg': p.Fleg, 'Fr_max': p.Fr_max,
                                   'Fr_l0': p.Fr_l0, 'Fr_r0': p.Fr_r0,
                                   'Fr_l': p.Fr_l_log, 'Fr_r': p.Fr_r_log})

    def getIndex(self, t):
        try:
            # get index
            a_bool = self.jumps[self.jumpNumber]["time"] >= t
            idx = min([i for (i, val) in enumerate(a_bool) if val]) - 1
            if idx == -1:
                return 0
            else:
                return idx
        except:
            return -1

    def getImpulseAngle(self):
        angle_hip_roll = math.atan2(self.jumps[self.jumpNumber]["Fleg"][1],
                                    self.jumps[self.jumpNumber]["Fleg"][0])
        angle_hip_pitch = math.atan2(self.jumps[self.jumpNumber]["Fleg"][2], self.jumps[self.jumpNumber]["Fleg"][0])
        print(colored(f"Start orienting leg to (pitch, roll)  : {angle_hip_roll, angle_hip_roll}", "blue"))
        angle_hip_pitch += -1.57
        return angle_hip_pitch, angle_hip_roll

    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchors
    def computeJointVariables(self, p):
        if conf.robot_params[self.robot_name]['wall_inclination'] > 0.:  # TODO missing normal in matlab wall_constraint!
            p[0] = (-p[2]) * math.tan(conf.robot_params[self.robot_name]['wall_inclination'])  # spawn_x is for the anchor point which is shifted wrt the wall
            print(f"adjusting initial position to be consistent with wall: {p}")

        mountain_wire_pitch_l = math.atan2(p[0], -p[2])
        mountain_wire_pitch_r = math.atan2(p[0], -p[2])

        mountain_wire_roll_l = -math.atan2(-p[2], p[1])
        mountain_wire_roll_r = math.atan2(-p[2], self.anchor_distance_y - p[1])
        # this is an approximation cause I should compute the real rope length considering the hoist distance
        wire_base_prismatic_l = np.linalg.norm(p) - self.anchor_distance_y * 0.5
        wire_base_prismatic_r = math.sqrt(p[0] * p[0] + (self.anchor_distance_y - p[1]) * (self.anchor_distance_y - p[1]) + p[2] * p[2]) - self.anchor_distance_y * 0.5

        wire_base_roll_l = -mountain_wire_roll_l
        wire_base_roll_r = -mountain_wire_roll_r
        return [mountain_wire_pitch_r, mountain_wire_roll_r, wire_base_prismatic_r, 0., wire_base_roll_r, 0.,
                mountain_wire_pitch_l, mountain_wire_roll_l, wire_base_prismatic_l, 0., wire_base_roll_l, 0.]

    def computeOrientationControl(self, des_roll, des_pitch):
        # compute desired orientation
        w_R_des = self.math_utils.eul2Rot(np.array([des_roll, 0., des_pitch]))
        # compute rotation matrix from actual orientation of ee to the desired
        b_R_des = self.w_R_b.T.dot(w_R_des)
        # compute the angle-axis representation of the associated orientation error
        delta_theta = math.atan2(np.sqrt(pow(b_R_des[2, 1] - b_R_des[1, 2], 2) + pow(b_R_des[0, 2] - b_R_des[2, 0], 2) + pow(b_R_des[1, 0] - b_R_des[0, 1], 2)),
                                 b_R_des[0, 0] + b_R_des[1, 1] + b_R_des[2, 2] - 1)
        # compute the axis (deal with singularity)
        if delta_theta == 0.0:
            e_error_o = np.zeros(3)
        else:
            r_hat = 1 / (2 * np.sin(delta_theta)) * np.array([b_R_des[2, 1] - b_R_des[1, 2], b_R_des[0, 2] - b_R_des[2, 0], b_R_des[1, 0] - b_R_des[0, 1]])
            # compute the orientation error
            e_error_o = delta_theta * r_hat
        # the error is expressed in the end-effector frame; map it in the world frame
        w_error_o = self.w_R_b.dot(e_error_o)

        # compute the virtual moment (angular part of the wrench) to realize the orientation task
        W_Gamma_des = np.multiply(conf.robot_params[p.robot_name]['Ko'], w_error_o) + np.multiply(conf.robot_params[p.robot_name]['Do'], -self.omega_b)
        # selection matrix to remove the pitch
        S = np.array([[1, 0, 0], [0, 0, 1]])
        # map to BF and remove the pitch
        B_Gamma_desRY = S.dot(self.w_R_b.dot(W_Gamma_des))
        # build jacobian (3x1)
        J_p = np.hstack(((self.math_utils.skew(p.hoist_l_pos - p.base_pos).dot(self.rope_direction)).reshape(3, 1), (self.math_utils.skew(p.hoist_r_pos - p.base_pos).dot(self.rope_direction2)).reshape(3, 1)))
        # map it to BF and remove pitch
        B_J_pRY = S.dot(self.w_R_b.dot(J_p))

        f_r_fbk = np.linalg.inv(B_J_pRY).dot(B_Gamma_desRY)

        return 0, 0  # f_r_fbk[0], f_r_fbk[1]

    def detectTouchDown(self):
        force_th = 10.
        if not self.touch_down_detected_l and (self.wall_normal.dot(self.contactForceW_l) > force_th):
            self.touch_down_detected_l = True
        if not self.touch_down_detected_r and (self.wall_normal.dot(self.contactForceW_r) > force_th):
            self.touch_down_detected_r = True

        if self.touch_down_detected_l and self.touch_down_detected_r:
            print(colored("TouchDown Detected", "blue"))
            # sample com pos
            self.x_tilde0 = self.wall_normal.reshape(1, 3) @ (self.com)
            return True
        else:
            return False

    def computeLandingControl(self):
        # compute relative position wrt base and xp and project on landing leg plane (supposed to be aligned with wall normal)
        x_tilde = self.wall_normal.dot(self.com)
        xd_tilde = self.wall_normal.dot(self.base_vel)

        # compute impedance law for com
        K_l = 10.
        D_l = 2 * math.sqrt(10. * self.getRobotMass())
        f_com = K_l * (self.x_tilde0 - x_tilde) - D_l * xd_tilde
        if f_com < 0:
            f_com = 0.
        f_com_vec = self.wall_normal * f_com
        self.ros_pub.add_arrow(self.base_pos, f_com_vec / self.force_scale, "red", scale=1.5)

        # map into feet landing forces
        A = np.zeros((6, 6))
        # sum linear forces
        A[:3, :3] = np.eye(3)
        A[:3, 3:] = np.eye(3)
        A[3:, :3] = self.math_utils.skew(self.x_landing_l - self.com)
        A[3:, 3:] = self.math_utils.skew(self.x_landing_r - self.com)

        # keep recomputing gravity comp
        self.Fr_r_actual = 2 * self.g[p.rope_index[1]]
        self.Fr_l_actual = self.g[p.rope_index[1]]

        b = np.zeros(6)
        b[:3] = - self.Fr_r_actual - self.Fr_l_actual - self.getRobotMass() * self.robot.model.gravity.vector[:3] + f_com_vec
        b[3:] = -np.cross(self.hoist_l_pos - self.com, self.rope_direction * self.Fr_l_actual) - np.cross(self.hoist_r_pos - self.com, self.rope_direction2 * self.Fr_r_actual)

        F_l = np.linalg.pinv(A).dot(b)  # Fl_l, Fl_r

        # build jacobian extracting columns from geom landing jacobians
        Jl = block_diag(self.J_landing_l[:, 15].reshape(3, 1), self.J_landing_r[:, 17].reshape(3, 1))

        # TODO uncomment
        # tau = -Jl.T.dot(F_l)
        tau = np.zeros(2)
        return tau

    def computeLateralManeuverVelocity(self, vel_base, omega_base=np.array([0., 0, 0.])):
        y_axis_base = self.w_R_b[:, 1]
        v_ll = y_axis_base.dot(vel_base - self.math_utils.skew(self.x_landing_l - p.base_pos).dot(omega_base))
        v_lr = y_axis_base.dot(vel_base - self.math_utils.skew(self.x_landing_r - p.base_pos).dot(omega_base))
        v_rl = self.rope_direction.dot(vel_base - self.math_utils.skew(self.hoist_l_pos - p.base_pos).dot(omega_base))
        v_rr = self.rope_direction2.dot(vel_base - self.math_utils.skew(self.hoist_r_pos - p.base_pos).dot(omega_base))
        wheel_radius = 0.15 / 2.
        # wheel speed is positive CW ( from top) but the center is moving in opposite direction (Y positive)
        return v_ll / wheel_radius, v_lr / wheel_radius, v_rl, v_rr

    def resetRope(self):
        print(colored(f"RESTORING ROPE PD", "red"))
        # enable PD for rope and reset the PD reference to the new estension
        self.q_des[p.rope_index[0]] = np.copy(p.q[p.rope_index[0]])
        self.q_des[p.rope_index[1]] = np.copy(p.q[p.rope_index[1]])
        # stop applying rope forces and restore PD gains on rope joints
        self.Fr_r = 0.
        self.Fr_l = 0.
        self.tau_ffwd[p.rope_index] = np.zeros(2)
        self.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)

    def printParams(self, p0, pf):
        print(colored(f"p0: {p0}", "red"))
        print(colored(f"pf: {pf}", "red"))
        print(colored(f"Fleg_max: {self.Fleg_max}", "red"))
        print(colored(f"Fr_max: {self.Fr_max}", "red"))
        print(colored(f"mu: {self.mu}", "red"))
        print(colored(f"jump_clearance: {self.optim_params['jump_clearance']}", "red"))
        print(colored(f"mass: {self.optim_params['m']}", "red"))
        print(colored(f"obstacle_avoidance: {self.optim_params['obstacle_avoidance']}", "red"))
        print(colored(f"obstacle_location: {self.optim_params['obstacle_location']}", "red"))
        print(colored(f"obstacle_size: {self.optim_params['obstacle_size']}", "red"))
        print(colored(f"num_params: {self.optim_params['num_params']}", "red"))
        print(colored(f"int_method: {self.optim_params['int_method']}", "red"))
        print(colored(f"N_dyn: {self.optim_params['N_dyn']}", "red"))
        print(colored(f"FRICTION_CONE: {self.optim_params['FRICTION_CONE']}", "red"))
        print(colored(f"int_steps: {self.optim_params['int_steps']}", "red"))
        print(colored(f"contact_normal: {self.optim_params['contact_normal']}", "red"))
        print(colored(f"b: {self.optim_params['b']}", "red"))
        print(colored(f"p_a1: {self.optim_params['p_a1']}", "red"))
        print(colored(f"p_a2: {self.optim_params['p_a2']}", "red"))
        print(colored(f"g: {self.optim_params['g']}", "red"))
        print(colored(f"w1: {self.optim_params['w1']}", "red"))
        print(colored(f"w2: {self.optim_params['w2']}", "red"))
        print(colored(f"w3: {self.optim_params['w3']}", "red"))
        print(colored(f"w4: {self.optim_params['w4']}", "red"))
        print(colored(f"w5: {self.optim_params['w5']}", "red"))
        print(colored(f"w6: {self.optim_params['w6']}", "red"))
        print(colored(f"T_th: {self.optim_params['T_th']}", "red"))

    def _build_params(self, source):
        """Build a climbingrobot_optimization Params dataclass from a plain dict
        (the ROS2 replacement for the MATLAB optim_params struct)."""
        params = Params()
        for key in ('m', 'g', 'num_params', 'int_method', 'N_dyn', 'int_steps', 'T_th',
                    'w1', 'w2', 'w3', 'w4', 'w5', 'w6', 'FRICTION_CONE',
                    'obstacle_avoidance', 'jump_clearance', 'b'):
            if key in source:
                setattr(params, key, source[key])
        params.p_a1 = np.asarray(source['p_a1'], dtype=float).reshape(3)
        params.p_a2 = np.asarray(source['p_a2'], dtype=float).reshape(3)
        params.contact_normal = np.asarray(source['contact_normal'], dtype=float).reshape(3)
        if 'obstacle_location' in source:
            params.obstacle_location = np.asarray(source['obstacle_location'], dtype=float).reshape(3)
        if 'obstacle_size' in source:
            params.obstacle_size = np.asarray(source['obstacle_size'], dtype=float).reshape(3)
        # types that must be int / bool for the optimizer
        params.num_params = int(params.num_params)
        params.N_dyn = int(params.N_dyn)
        params.int_steps = int(params.int_steps)
        params.FRICTION_CONE = bool(params.FRICTION_CONE)
        return params

    def initOptim(self, p0, pf):
        # offline optim vars
        self.Fleg_max = 300.
        if self.TYPE_OF_JUMP == 'upward':
            self.Fr_max = 90.  # had to increase because of slopes downward jumps it used to be 90
            self.Fr_min = 0.  # had to increase because of slopes downward jumps it used to be 0
        if self.TYPE_OF_JUMP == 'downward':
            self.Fr_max = 190.  # had to increase because of slopes downward jumps it used to be 90
            self.Fr_min = 15.  # had to increase because of slopes downward jumps it used to be 0
        self.mu = 0.8
        self.optim_params = {}
        self.optim_params['jump_clearance'] = 1.

        if self.OBSTACLE_AVOIDANCE:
            # I hard code it otherwise does not converge cause it is very sensitive
            p0 = np.array([0.5, 0.5, -6])
            self.Fr_max = 120.

        if self.landing:
            self.optim_params['m'] = 15.07  # I need to hardcode it otherwise it does not converge
            # I hardcode this because wall inclination is non 0 so we start from 0.5
            p0 = np.array([0.5, 2.5, -6])
            pf = np.array([0.5, 4, -4])
            self.Fleg_max = 600.
            self.Fr_max = 300.
        else:
            self.optim_params['m'] = self.getRobotMass()

        # if terrain is inclined we consider only the Y,Z component of the pf and need to compute a target consistent with the wall!
        if conf.robot_params[p.robot_name]['wall_inclination'] > 0.:  # TODO missing normal in matlab wall_constraint!
            pf[0] = (-pf[2]) * math.tan(conf.robot_params[p.robot_name]['wall_inclination']) + conf.robot_params[p.robot_name]['spawn_x']
            print(f"adjusting landing target to be consistent with wall: {pf}")

        self.optim_params['obstacle_avoidance'] = self.OBSTACLE_AVOIDANCE
        self.optim_params['obstacle_location'] = np.asarray(self.obstacle_location, dtype=float).reshape(3)
        self.optim_params['obstacle_size'] = np.asarray(self.obstacle_size, dtype=float).reshape(3)
        self.optim_params['num_params'] = 4.
        self.optim_params['int_method'] = 'rk4'
        self.optim_params['N_dyn'] = 30.
        self.optim_params['FRICTION_CONE'] = 1.
        self.optim_params['int_steps'] = 5.
        self.optim_params['contact_normal'] = np.array([1, 0, 0], dtype=float)
        self.optim_params['b'] = self.anchor_distance_y
        self.optim_params['p_a1'] = np.array([0., 0., 0.])
        self.optim_params['p_a2'] = np.array([0., self.optim_params['b'], 0.])
        self.optim_params['g'] = 9.81
        self.optim_params['w1'] = 1.  # smooth
        if not p.MULTIPLE_JUMPS:
            self.optim_params['w2'] = 0.  # hoist work
        else:
            self.optim_params['w2'] = 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
        self.optim_params['w3'] = 0.
        self.optim_params['w4'] = 0.
        self.optim_params['w5'] = 0.
        self.optim_params['w6'] = 0.
        self.optim_params['T_th'] = 0.05

        # Build the Params dataclass and run the offline optimization.
        # ROS1: self.matvars = self.eng.optimize_cpp_mex(p0, pf, Fleg_max, Fr_max, Fr_min, mu, optim_params)
        # ROS2: jump_optimizer.optimize_cpp(p0, pf, Fleg_max, Fr_max, mu, params)  (no Fr_min arg)
        self.params = self._build_params(self.optim_params)
        try:
            self.matvars = jump_optimizer.optimize_cpp(
                np.asarray(p0, dtype=float), np.asarray(pf, dtype=float),
                self.Fleg_max, self.Fr_max, self.mu, self.params,
                use_cpp=True, max_iter=400, verbose=True)
        except Exception as e:
            print(colored(f"Issue in calling jump_optimizer.optimize_cpp: {e}", "red"))
            raise

        # extract variables (MATLAB struct field -> solution dict key)
        self.ref_com = self.matvars['p']
        self.ref_psi = self.matvars['psi']
        self.ref_l_1 = self.matvars['l1']
        self.ref_l_2 = self.matvars['l2']
        self.ref_time = self.matvars['time']
        self.Fr_l0 = self.matvars['Fr_l']
        self.Fr_r0 = self.matvars['Fr_r']
        self.Fleg = self.matvars['Fleg']
        # this is computed integrating the dynamics with dt and can be different from the reference
        self.targetPos = self.ref_com[:, -1]  # output of optimization
        self.targetPosIdeal = self.ref_com[:, -1]

        print(colored(f"offline optimization accomplished, p0:{p0}, target(rough integr):{self.targetPos}", "blue"))
        print(colored(f"target (fine integr.) is:{self.matvars['achieved_target']}", "blue"))

        self.jumps = [{"time": self.ref_time, "thrustDuration": self.matvars['T_th'], "p0": p0,
                       "targetPos": self.targetPos, "Fleg": self.Fleg,
                       "Fr_r": self.Fr_r0, "Fr_l": self.Fr_l0, "Tf": self.matvars['Tf']}]

        # MPC vars (need to perform optim to know Tf)
        self.mpc_N = int(0.4 * self.optim_params['N_dyn'])
        self.Fr_max_mpc = 100.

        self.optim_params_mpc = {}
        self.optim_params_mpc['int_method'] = 'rk4'
        self.optim_params_mpc['int_steps'] = 5.
        self.optim_params_mpc['contact_normal'] = np.array([1., 0., 0.])
        self.optim_params_mpc['b'] = self.anchor_distance_y
        self.optim_params_mpc['p_a1'] = np.array([0., 0., 0.])
        self.optim_params_mpc['p_a2'] = np.array([0., self.optim_params_mpc['b'], 0.])
        self.optim_params_mpc['g'] = 9.81
        self.optim_params_mpc['m'] = self.getRobotMass()
        self.optim_params_mpc['w1'] = 1.
        self.optim_params_mpc['w2'] = 0.000001
        self.optim_params_mpc['N_dyn'] = self.optim_params['N_dyn']
        self.optim_params_mpc['mpc_dt'] = self.matvars['Tf'] / (self.optim_params['N_dyn'] - 1)

        # Params dataclass for MPC (mirrors optim_params_mpc struct)
        self.params_mpc = self._build_params(self.optim_params_mpc)
        self.params_mpc.mpc_dt = self.optim_params_mpc['mpc_dt']

        self.deltaFr_l = np.zeros((int(self.mpc_N)))
        self.deltaFr_r = np.zeros((int(self.mpc_N)))
        self.propeller_force = np.zeros((int(self.mpc_N)))

        if self.landing:
            self.Fr_max_mpc = 150.  # this is not used in the mpc it creates issues
        if self.PLOT_MPC:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)

    def computeMPC(self, delta_t):
        # after the thrust we start MPC, it will start from time 0.05 so the index will start from 2
        if self.getIndex(delta_t) != -1:
            self.mpc_index = self.getIndex(delta_t)
        else:  # whenever the MPC should not be updated anymore use delta_t to increment mpc_index_ffwd
            if (delta_t % self.optim_params_mpc['mpc_dt']) < 0.001:  # increment mpc_index_ffwd every mpc_dt
                self.mpc_index_ffwd += 1
                if self.mpc_index_ffwd > (self.mpc_N - 1):  # reference is finished keep the last computed one
                    self.mpc_index_ffwd = self.mpc_N - 1

        # This is better for const dist cause it keeps optimizing till the end
        if (self.mpc_index != self.mpc_index_old):  # do optim only every dtMPC not every dt
            # reduce MPC horizon gradually at the end
            if ((self.mpc_index + self.mpc_N) >= len(self.ref_time)):
                self.mpc_N -= 1
            # eval ref
            ref_com = self.ref_com[:, self.mpc_index:self.mpc_index + self.mpc_N]
            Fr_l0 = self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N]
            Fr_r0 = self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N]
            actual_t = self.ref_time[self.mpc_index]

            actual_state = np.array([self.psi, self.l_1, self.l_2, self.psid, self.l_1d, self.l_2d])

            self._call_service(self.pause_physics_client, Empty.Request())
            # perform optimization (MATLAB *_mex -> mpc_controller.*)
            if p.PROPELLERS:
                x, exitflag, fun = mpc_controller.optimize_cpp_mpc_propellers(
                    actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N,
                    self.params_mpc, use_cpp=True, verbose=False)
                # extract optim vars
                self.deltaFr_l = x[:self.mpc_N]
                self.deltaFr_r = x[self.mpc_N:2 * self.mpc_N]
                self.propeller_force = x[2 * self.mpc_N:3 * self.mpc_N]
            else:
                if self.MPC_uses_constraints:
                    x, exitflag, fun = mpc_controller.optimize_cpp_mpc(
                        actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N,
                        self.params_mpc, use_cpp=True, verbose=False)
                else:
                    x, exitflag, fun = mpc_controller.optimize_cpp_mpc_no_constraints(
                        actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc, self.mpc_N,
                        self.params_mpc, use_cpp=True, verbose=False)
                # extract optim vars
                self.deltaFr_l = x[:self.mpc_N]
                self.deltaFr_r = x[self.mpc_N:]

            # store tracking error for RMSE computation
            tracking_error = self.ref_com[:, self.mpc_index] - (self.base_pos - p.anchor_pos)
            self.MPC_tracking_error.append(np.linalg.norm(tracking_error))
            # online plot MPC
            if self.PLOT_MPC:
                self.onlinePlotMPC(self.deltaFr_l, self.deltaFr_r)
            self._call_service(self.unpause_physics_client, Empty.Request())

        self.mpc_index_old = self.mpc_index

        return self.deltaFr_l[self.mpc_index_ffwd], self.deltaFr_r[self.mpc_index_ffwd], self.propeller_force[self.mpc_index_ffwd]

    def onlinePlotMPC(self, deltaFr_l, deltaFr_r):
        # debug
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_label("delta Frl")
        self.ax2.set_label("delta Frr")
        self.ax1.grid()
        self.ax2.grid()
        # MPC action (red)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_l, "or-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_r, "or-")
        # full action (black)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_l, "ok-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_r, "ok-")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def generateTargetPoints(self, p0):
        # generate points in an ellipse of axis a b = 2.5 around p0
        alpha = np.deg2rad(45)
        theta = np.linspace(alpha, alpha + 2 * np.pi, 9)
        # main axes ellipse
        a = (self.anchor_distance_y - 1.5) / 2 / np.cos(alpha)
        b = 2
        y = a * np.cos(theta[:-1])
        z = b * np.sin(theta[:-1])

        Rx = np.array([[np.cos(-alpha), -np.sin(-alpha)], [np.sin(-alpha), np.cos(-alpha)]])
        # sets 0 coord equal to p0
        cw = numpy.matlib.repmat(p0.reshape(3, 1).copy(), 1, 8)
        cw[1:, :] += Rx.dot(np.vstack((y, z)))
        return cw

    def computeJumpEnergyConsumption(self):
        # get index
        lift_off_idx = np.max(np.where((self.time_log - self.start_logging) <= self.jumps[self.jumpNumber]["thrustDuration"]))
        impulse_work = 0.5 * self.optim_params['m'] * self.base_vel_log[:, lift_off_idx].dot(self.base_vel_log[:, lift_off_idx])  # ekin at liftoff
        # this integral is done on a rough discretization dt
        touch_down_idx = np.max(np.where((self.time_log - self.start_logging) < p.jumps[p.jumpNumber]["Tf"]))
        hoist_work = 0.
        for i in range(touch_down_idx):
            hoist_work = hoist_work + (
                abs(self.Fr_r_log[i] * self.l_2d_log[i]) + abs(self.Fr_l_log[i] * self.l_1d_log[i])) * conf.robot_params[p.robot_name]['dt']
        return impulse_work + hoist_work


def talker(p):
    p.start()
    additional_args = ['spawn_2x:=' + str(conf.robot_params[p.robot_name]['spawn_2x']),
                       'spawn_2y:=' + str(conf.robot_params[p.robot_name]['spawn_2y']),
                       'spawn_2z:=' + str(conf.robot_params[p.robot_name]['spawn_2z']),
                       'obstacle:=' + str(p.OBSTACLE_AVOIDANCE),
                       'obstacle_location_x:=' + str(p.obstacle_location[0]),
                       'obstacle_location_y:=' + str(p.obstacle_location[1]),
                       'obstacle_location_z:=' + str(p.obstacle_location[2]),
                       'obstacle_size_x:=' + str(p.obstacle_size[0]),
                       'obstacle_size_y:=' + str(p.obstacle_size[1]),
                       'obstacle_size_z:=' + str(p.obstacle_size[2]),
                       'wall_inclination:=' + str(conf.robot_params[p.robot_name]['wall_inclination']),
                       'double_propeller:=' + str(p.USE_PROPELLERS_FOR_LEG_REORIENT)]
    if p.landing:
        world_name = "climbingrobot2landing.world"
    else:
        world_name = "climbingrobot2.world"
    # ROS1 used the full path to the .launch file; in ROS2 both the climbingrobot2 and
    # climbingrobot2landing robot_names share the same launch file.
    launch_file = 'ros_impedance_controller_climbingrobot2.launch.py'
    p.startSimulator(world_name=world_name, additional_args=additional_args, launch_file=launch_file)
    p.loadModelAndPublishers()

    p.startupProcedure()
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    # loop frequency
    rate = p.node.create_rate(1 / conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()

    # 3 --- whole pipeline with variable Fr_l Fr_r and the Fleg x,y,z
    # single jump; p0 is defined wrt anchor1 pos in matlab convention
    p0 = np.array([0.28, 2.5, -6.10104])  # there is singularity for px = 0!

    if p.MULTIPLE_JUMPS:
        landingW = p.generateTargetPoints(p0)
        if p.ADD_NOISE:
            landingW = np.repeat(landingW, repeats=50, axis=1)
    else:
        if p.OBSTACLE_AVOIDANCE:
            p0 = np.array([0.5, 0.5, -6])
            # middle one for reviewer reply R1.5 (NB the obstacle is large Rx = 1.5 but you need to add 0.2 due to the anchor_pos(x)
            landingW = np.array([1.7, 2.5, -6]).reshape(3, 1)
        else:
            if p.TYPE_OF_JUMP == 'upward':
                landingW = np.array([0.28, 4, p.JUMP_LENGTH_MULTIPLIER * (-4)]).reshape(3, 1)
            elif p.TYPE_OF_JUMP == 'downward':
                landingW = np.array([0.28, 4, p.JUMP_LENGTH_MULTIPLIER * (-12)]).reshape(3, 1)
            else:
                print("wrong TYPE_OF_JUMP")
        if p.ADD_NOISE:
            if p.type_of_disturbance == 'impulse':
                number_of_tests = 100
            else:
                number_of_tests = 6
            landingW = np.matlib.repmat(landingW, 1, number_of_tests)

    for p.n_test in range(landingW.shape[1]):
        pf = landingW[:, p.n_test]
        print(colored(f"---------------Ideal Reference landing test # {p.n_test}: {pf}", "green"))

        # jump parameters
        if p.MULTIPLE_JUMPS:
            p.startJump = 2.5  # wait more for longer jumps to initialize
        else:
            p.startJump = 2.5
        p.orientTime = 1.0
        p.stateMachine = 'idle'
        p.jumpNumber = 0
        p.numberOfJumps = 1
        p.start_logging = np.inf

        # set the rope base joint variables to initialize in p0 position, the leg ones are defined in params.yaml
        p.q_des[:12] = p.computeJointVariables(p0)

        p.setSimSpeed(dt_sim=0.001, max_update_rate=300, iters=1500)

        while rclpy.ok():

            # update the kinematics
            p.updateKinematicsDynamics()

            # multiple jumps state machine
            if (p.stateMachine == 'idle') and (p.time >= p.startJump) and (p.jumpNumber < p.numberOfJumps):
                # first run optim and fill in jump variable
                p._call_service(p.pause_physics_client, Empty.Request())
                p.initOptim(p.base_pos - p.mat2Gazebo, pf)
                p._call_service(p.unpause_physics_client, Empty.Request())
                p.des_leg_orient = p.getImpulseAngle()

                # set the end of orienting
                p.end_orienting = p.startJump + p.orientTime
                p.end_thrusting = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"]
                p.start_logging = p.end_orienting
                p.stateMachine = 'orienting_leg'  # this phase only waits is not doing anything

            if (p.stateMachine == 'orienting_leg'):
                # use propellers (review)
                if p.USE_PROPELLERS_FOR_LEG_REORIENT:
                    p.q_des[p.hip_pitch_joint] = p.des_leg_orient[0]
                    p.q_des[p.hip_roll_joint] = 0.  # set leg straight
                    # reorient base yaw to be p.des_leg_orient[1]
                    rpy = p.math_utils.rot2eul(p.w_R_b)
                    Mz = 30. * (p.des_leg_orient[0] - rpy[2])
                    p.apply_propeller_moment(Mz)
                else:
                    p.q_des[p.hip_pitch_joint] = p.des_leg_orient[0]
                    p.q_des[p.hip_roll_joint] = p.des_leg_orient[1]

                if (p.time >= p.end_orienting):
                    print(colored(f"Stop orienting leg", "blue"))
                    print("\033[34m" + "---------Starting jump  number ", p.jumpNumber, " to optimized target: ",
                          p.jumps[p.jumpNumber]["targetPos"], " from actual p0 : ", p.base_pos - p.mat2Gazebo)
                    print(colored(f"Start trusting", "blue"))
                    p.tau_ffwd = np.zeros(p.robot.na)
                    p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravity in the virtual joint to go exactly there
                    p.pid.setPDjoint(p.base_passive_joints, 0., 0., 0.)
                    p.pid.setPDjoint(p.leg_index, 0., 0., 0.)
                    print(colored(f"ZERO LEG AND ROPE PD", "red"))
                    p.stateMachine = 'thrusting'
                    p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                    p.w_Fleg = p.jumps[p.jumpNumber]["Fleg"]

            if (p.stateMachine == 'thrusting'):
                # apply leg impulse for thrust duration
                p.tau_ffwd[p.leg_index] = -p.Jleg.T.dot(p.w_Fleg)
                # plot Fleg
                p.ros_pub.add_arrow(p.x_ee, p.w_Fleg / p.force_scale, "red", scale=2.5)

                # start also applying forces to ropes
                delta_t = p.time - p.end_orienting
                p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)]
                p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)]

                # plot rope forces
                p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
                p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
                p.tau_ffwd[p.rope_index[0]] = p.Fr_r
                p.tau_ffwd[p.rope_index[1]] = p.Fr_l

                if (p.time > p.end_thrusting):
                    print(colored("Stop Trhusting", "blue"))
                    print(colored(f"RESTORING LEG PD", "red"))
                    # reenable the PDs of default values for landing and reset the torque on the leg (stop applying impulse)
                    p.pid.setPDjoint(p.base_passive_joints, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)
                    # reenable leg pd
                    p.pid.setPDjoint(p.leg_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)
                    p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                    p.stateMachine = 'flying'

                    # retract leg and move landing elements
                    p.q_des[p.leg_index[2]] = 0.25

                    # manage lander retracting leg
                    if p.landing:
                        # extend landing joints
                        p.tau_ffwd[p.landing_joints] = np.zeros(2)
                        if p.impedance_landing:
                            p.q_des[p.landing_joints] = np.array([-0.6, 0.6])
                            p.stateMachine = 'flying_and_reorient_lander'
                    print(colored("Start " + p.stateMachine, "blue"))

                    # add impulsive disturbance
                    p.delayed_start = 0.
                    if p.type_of_disturbance == 'impulse':
                        p.dist_duration = 0.1
                        p.base_dist = np.array([50., -50., 30.])

                        if p.ADD_NOISE:
                            if (p.n_test % 10) == 0:
                                p.impulse_start_count += 1
                                print(colored(f'APPLYING IMPULSE AT {p.impulse_start_count*10}% of the flying phase\n', 'red'))
                                p.delayed_start = p.impulse_start_count * (p.jumps[p.jumpNumber]["Tf"] - p.jumps[p.jumpNumber]["thrustDuration"]) / 10
                            p.base_dist = p.generateDisturbanceOnHemiSphere(25, 25)
                            print(colored(f"generated disturbance direction {p.base_dist}", "red"))

                    # add constant disturbance
                    if p.type_of_disturbance == 'const':
                        p.dist_duration = p.jumps[p.jumpNumber]["Tf"] - p.jumps[p.jumpNumber]["thrustDuration"]
                        p.base_dist = np.array([7., -7., 0.])
                        if p.ADD_NOISE:
                            p.base_dist = p.generateWindDisturbance(p.n_test, 7)

                    if p.type_of_disturbance != 'none':
                        start_time = (p._now() + Duration(seconds=float(p.delayed_start))).to_msg()
                        p.applyWrench(p.base_dist[0], p.base_dist[1], p.base_dist[2], time_interval=p.dist_duration, start_time=start_time)

            if (p.stateMachine == 'flying'):
                # after the thrust we start MPC it will start from time 0.05 so the index should be 12
                # applying forces to ropes
                delta_t = p.time - p.end_orienting
                if p.MPC_control:
                    deltaFr_l0, deltaFr_r0, prop_force = p.computeMPC(delta_t)
                    if p.PROPELLERS:
                        p.apply_propeller_force(prop_force)
                else:
                    deltaFr_l0 = 0.
                    deltaFr_r0 = 0.

                p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
                p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0

                # plot rope forces
                p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
                p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)

                p.tau_ffwd[p.rope_index[0]] = p.Fr_r
                p.tau_ffwd[p.rope_index[1]] = p.Fr_l
                end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["Tf"]

                if (p.time >= end_flying):
                    print(colored("Stop Flying", "blue"))
                    # reset the qdes; reset the rope PD because the Fr are finished
                    p.resetRope()

                    energy = p.computeJumpEnergyConsumption()
                    p.jumpNumber += 1
                    if (p.jumpNumber < p.numberOfJumps):
                        p.stateMachine = 'idle'
                        # reset for multiple jumps
                        p.startJump = p.time
                    else:
                        landing_location = p.base_pos - p.mat2Gazebo
                        print(colored(f" real landing (in matlab convention) is: {landing_location}", "blue"))
                        print(colored(f" while from optim it should be  {p.targetPos}", "blue"))

                        print(colored(f" the landing error is  {np.linalg.norm(landing_location - p.targetPos)}", "blue"))
                        jump_length = np.linalg.norm(p0[:2] - p.targetPos[:2])
                        MSE = np.square(np.array(p.MPC_tracking_error)).mean()
                        RMSE = math.sqrt(MSE)
                        print(colored(
                            f" the relative landing error (norm per jump lenghth)  is {100*np.linalg.norm(landing_location - p.targetPos) / jump_length}%",
                            "blue"))
                        print(colored(f" the energy consumption is  {energy}", "blue"))
                        print(colored(f" the rmse of MPC tracking error is  {RMSE}", "blue"))
                        print(colored(f" the leg impulse  is  {p.Fleg}", "blue"))
                        print(colored(f" the norm of the leg impulse  is  {np.linalg.norm(p.Fleg)}", "blue"))
                        if p.ADD_NOISE:
                            import pandas as pd  # lazy import for the offline noise sweep
                            data = {'test_nr': p.n_test, 'ideal_target': landingW[:, p.n_test], 'optim_target': p.targetPos, 'landing_location': landing_location, 'landing_error': np.linalg.norm(landing_location - p.targetPos),
                                    'relative_error': np.linalg.norm(landing_location - p.targetPos) / jump_length, 'energy': energy, 'rmse': RMSE}
                            if p.type_of_disturbance != 'none':
                                data['base_dist'] = p.base_dist
                            df_dict = pd.DataFrame([data])
                            p.df = pd.concat([p.df, df_dict], ignore_index=True)
                            filename = f'noise_multiple_{p.MULTIPLE_JUMPS}_dist_{p.type_of_disturbance}.csv'
                            p.df.to_csv(filename, index=None)
                        # fundamental: save everything before initVars! cause it will be deleted
                        else:
                            p.plotStuff()
                        # reset for the next jump
                        if p.MULTIPLE_JUMPS or p.ADD_NOISE:
                            p.startupProcedure()
                            p.initVars()
                            p.q_des = np.copy(p.q_des_q0)

                        break
            # this is the same as flying but with the lander
            if (p.stateMachine == 'flying_and_reorient_lander'):
                # applying forces to ropes, when time is finished just reset rope length (only once!) and wait for tf
                delta_t = p.time - p.end_orienting

                if p.MPC_control:
                    deltaFr_l0, deltaFr_r0, p.prop_force = p.computeMPC(delta_t)
                    if p.PROPELLERS:
                        p.apply_propeller_force(p.prop_force)
                else:
                    deltaFr_l0 = 0.
                    deltaFr_r0 = 0.

                if p.type_of_disturbance != 'none':
                    if ((delta_t - p.delayed_start) >= 0) and ((delta_t - p.delayed_start) < p.dist_duration):
                        p.ros_pub.add_arrow(p.base_pos, p.base_dist / 10., "white", scale=1.5)

                if not p.optimal_control_traj_finished:
                    if p.getIndex(delta_t) == -1:
                        # start again pid gains and reset qdes
                        p.resetRope()
                        p.optimal_control_traj_finished = True
                    else:
                        p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
                        p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0
                    # check for early td and in case reset rope
                    if p.detectTouchDown():
                        p.resetRope()
                        print(colored("Early TD detected, Start landing", "blue"))
                        p.stateMachine = 'landing'
                        p.start_landing = p.time
                else:  # you are checking for delayed TD you have already reset rope and restored PD
                    if p.detectTouchDown():
                        print(colored("Start landing", "blue"))
                        p.stateMachine = 'landing'
                        p.start_landing = p.time

                # plot rope forces
                p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
                p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
                p.tau_ffwd[p.rope_index[0]] = p.Fr_r
                p.tau_ffwd[p.rope_index[1]] = p.Fr_l
                end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["Tf"]

                # reorient legs to land parallel to the wall
                rpy = p.math_utils.rot2eul(p.w_R_b)
                p.q_des[p.landing_joints] = np.array([-0.8, 0.8]) - np.array([rpy[2], rpy[2]])

            if (p.stateMachine == 'landing'):
                if (p.time < (p.start_landing + 1.)):
                    p.tau_ffwd[p.landing_joints] = p.computeLandingControl()
                else:
                    print(colored("Start lateral maneuvering", "blue"))
                    p.pid.setPDjoint(p.wheel_index, 0.1, 0.01, 0.)
                    p.stateMachine = 'lateral_maneuvering'
                    p.start_lateral_maneuver = p.time

            if (p.stateMachine == 'lateral_maneuvering'):
                if p.time < (p.start_lateral_maneuver + 3.5):
                    wheel_l, wheel_r, v_r_l, v_r_r = p.computeLateralManeuverVelocity(np.array([0., -0.7, 0.]))
                    p.qd_des[p.rope_index] = [v_r_r, v_r_l]
                    p.qd_des[p.wheel_index] = [wheel_l, wheel_r]
                    # integrate positions
                    p.q_des[p.wheel_index] += conf.robot_params[p.robot_name]['dt'] * p.qd_des[p.wheel_index]
                    p.q_des[p.rope_index] += conf.robot_params[p.robot_name]['dt'] * p.qd_des[p.rope_index]
                else:
                    p.qd_des[p.wheel_index] = np.zeros(2)
                p.prop_force = (-25.)  # push against the wall
                p.apply_propeller_force(p.prop_force)

            # plot ropes as green arrows
            if not p.SAVE_BAG:
                p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)  # arope, already in gazebo
                p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos - p.anchor_pos2), "green", scale=3.)  # arope, already in gazebo
            # plot contact forces on landing legs
            if p.landing:
                p.ros_pub.add_arrow(p.x_landing_l, p.contactForceW_l / p.force_scale, "blue", scale=1.5)
                p.ros_pub.add_arrow(p.x_landing_r, p.contactForceW_r / p.force_scale, "blue", scale=1.5)
            # plot contact force on retractable leg
            p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=2.5)

            # plot target position (whenever is available)
            try:
                p.ros_pub.add_marker(p.mat2Gazebo + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.3, alpha=1.)
                p.ros_pub.add_marker(p.mat2Gazebo + p.targetPosIdeal, color="green", radius=0.5, alpha=0.5)
            except:
                pass
            p.ros_pub.add_marker(p.x_ee, radius=0.05)
            p.ros_pub.publishVisual(delete_markers=False)

            # send commands to gazebo
            p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4)  # to avoid issues of dt 0.0009999
            if (p.time > p.start_logging):
                p.logData()
            # wait for synconization of the control loop
            rate.sleep()


def plot3D(name, figure_id, label, time_log, var, time_mat=None, var_mat=None):
    fig = plt.figure()
    fig.suptitle(name, fontsize=20)

    plt.subplot(3, 1, 1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    plt.grid(True)
    plt.legend(['act', 'ref'])

    plt.subplot(3, 1, 2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])

    plt.subplot(3, 1, 3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])


p = None


def main(args=None):
    global p
    p = ClimbingrobotController(robotName)

    try:
        talker(p)
    except (KeyboardInterrupt, RuntimeError):
        if rclpy.ok():
            rclpy.shutdown()
        p.deregister_node()
    finally:
        p.deregister_node()
        if p.landing:  # for the landing test you should press Ctrl C to stop everything
            p.plotStuff()


if __name__ == '__main__':
    main()
