# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino

ROS2 (Humble) port of base_controllers/climbingrobot_controller/climbingrobot_controller.py
--------------------------------------------------------------------------------------------
Subclasses the ROS2 BaseControllerFixed. The rospy -> rclpy translation mirrors
base_controller.py / base_controller_fixed.py:
  * `import rospy as ros`                 -> use self.node (rclpy) + helpers.
  * ros.Rate / ros.is_shutdown / ros.Time -> node.create_rate / rclpy.ok / self._now.
  * tf.TransformBroadcaster.sendTransform(trans, quat, time, child, parent)
                                          -> tf2_ros.TransformBroadcaster + TransformStamped
                                             (see _send_tf()).
  * rospy.ServiceProxy(...)               -> node.create_client + _call_service().
  * apply_body_wrench proxy call          -> ApplyBodyWrench.Request() + _call_service().
  * roslaunch scriptapi (spawnMountain)   -> `ros2 launch`/spawn via subprocess; NOTE that
                                             spawnMountain is a deprecated code path (the wall
                                             is spawned from the launch/world file instead).
FIRST-PASS PORT: this is gazebo- and MATLAB-trajectory-coupled and must be runtime
tested. It also requires the climbingrobot_description package and the MATLAB .mat
trajectory files (test_optim_marco.mat, ...) present in the working directory.
All kinematics / dynamics / Pinocchio / control math is unchanged.
"""

from __future__ import print_function
import os
import math

import numpy as np
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
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.msg import ContactsState
from std_srvs.srv import Empty

from base_controllers.utils.math_tools import *
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear
from base_controllers.base_controller_fixed import BaseControllerFixed
import base_controllers.params as conf

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)
robotName = "climbingrobot"


class ClimbingrobotController(BaseControllerFixed):

    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.EXTERNAL_FORCE = False
        self.LOAD_MATLAB_TRAJ = True
        self.MARCO_APPROACH = True

        if self.robot_name == 'climbingrobot':
            self.rope_index = 2
            self.wire_yaw_joint = 5
            self.hip_roll_joint = 7
            self.leg_index = np.array([6, 7, 8])
            self.base_passive_joints = np.array([3, 4, 5])
            self.anchor_passive_joints = np.array([0, 1])

        if self.robot_name == 'climbingrobot_slider':
            self.rope_index = 3
            self.slider_index = 0
            self.wire_yaw_joint = 6
            self.hip_roll_joint = 8
            self.leg_index = np.array([7, 8, 9])
            self.base_passive_joints = np.array([4, 5, 6])
            self.anchor_passive_joints = np.array([1, 2])

        self.mountain_thickness = 0.1  # TODO call the launch file passing this parameter
        self.r_leg = 0.3
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

    def applyForce(self, Fx, Fy, Fz, interval):
        from gazebo_msgs.srv import ApplyBodyWrench
        wrench = Wrench()
        wrench.force.x = float(Fx)
        wrench.force.y = float(Fy)
        wrench.force.z = float(Fz)
        wrench.torque.x = 0.
        wrench.torque.y = 0.
        wrench.torque.z = 0.
        reference_frame = "world"  # you can apply forces only in this frame because this service is buggy
        req = ApplyBodyWrench.Request()
        req.body_name = self.robot_name + "::base_link"
        req.reference_frame = reference_frame
        req.reference_point = Point(x=0., y=0., z=0.)
        req.wrench = wrench
        req.duration = Duration(seconds=interval).to_msg()
        self._call_service(self.apply_body_wrench, req)

    def loadModelAndPublishers(self, xacro_path=None):
        super().loadModelAndPublishers(xacro_path=xacro_path)
        # pause/unpause/reset_joints clients are created by BaseControllerFixed
        # (reset_joints_client). tf broadcaster + contact subscriber are climbing-specific.
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        qos = QoSProfile(depth=1)
        self.sub_contact = self.node.create_subscription(
            ContactsState, "/" + self.robot_name + "/foot_bumper", self._receive_contact, qos)

    def computeRopeAngle(self, point):
        rope_direction = (self.anchor_pos - point) / np.linalg.norm(self.anchor_pos - point)
        # angle of the point wrt the vertical line passing through the anchor
        alpha_anchor1 = math.acos(rope_direction[2])
        beta = math.atan2(point[0] + 0.0001, point[1])  # added small amount to have normal1 perpendicular to wall when 0, 0
        normal1 = np.array([math.sin(beta), math.cos(beta), math.sin(alpha_anchor1)])
        return beta, normal1

    def rot2phi_theta(self, R):
        theta = math.atan2(R[2, 0], R[2, 2])
        phi = math.atan2(-R[1, 2], -R[0, 2])  # -cos(phi) * sin(theta) , -sin(phi) * sin(theta)
        # returns  pitch = theta,  yaw = phi
        return theta, phi

    def updateKinematicsDynamics(self):
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
        self.base_pos = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).translation
        self.w_R_b = self.robot.framePlacement(self.q, self.robot.model.getFrameId('base_link')).rotation
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation

        # compute jacobian of the end effector in the world frame (take only the linear part and the actuated joints part)
        self.J = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), True, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
        self.Jleg = self.J[:, self.leg_index]
        self.dJdq = self.robot.frameClassicAcceleration(self.q, self.qd, None, self.robot.model.getFrameId(frame_name), False).linear

        w_R_wire = self.robot.framePlacement(self.q, self.robot.model.getFrameId('wire')).rotation
        self.theta, self.phi = self.rot2phi_theta(w_R_wire)
        self.l = self.q[p.rope_index]
        self.ldot = self.qd[p.rope_index]

        # compute com variables according to a frame located at the foot
        robotComB = pin.centerOfMass(self.robot.model, self.robot.data, self.q)

        # from ground truth
        self.com = self.robot.robotComW(self.q)
        mountain_pos = np.array([conf.robot_params[self.robot_name]['spawn_x'] - self.mountain_thickness / 2, conf.robot_params[self.robot_name]['spawn_y'], 0.0])
        self._send_tf(mountain_pos, (0.0, 0.0, 0.0, 1.0), '/wall', '/world')
        self._send_tf(mountain_pos, (0.0, 0.0, 0.0, 1.0), '/pillar', '/world')
        self._send_tf(mountain_pos, (0.0, 0.0, 0.0, 1.0), '/pillar2', '/world')

    def _receive_contact(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] = msg.states[0].wrenches[0].force.y
        grf[2] = msg.states[0].wrenches[0].force.z
        self.contactForceW = self.robot.framePlacement(self.q, self.robot.model.getFrameId("lower_link")).rotation.dot(grf)

    def initVars(self):
        super().initVars()
        self.qdd_des = np.zeros(self.robot.na)
        self.base_accel = np.zeros(3)
        # init new logged vars here
        self.com_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        if self.robot_name == 'climbingrobot_slider':
            self.simp_model_state_log = np.empty((4, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        else:
            self.simp_model_state_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.ldot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.time_jump_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.rope_normal = np.zeros(3)

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            if self.robot_name == 'climbingrobot_slider':
                self.simp_model_state_log[:, self.log_counter] = np.array([self.theta, self.phi, self.l, self.q[self.slider_index]])
            else:
                self.simp_model_state_log[:, self.log_counter] = np.array([self.theta, self.phi, self.l])
            self.ldot_log[self.log_counter] = self.ldot
            self.base_pos_log[:, self.log_counter] = self.base_pos
            self.time_jump_log[self.log_counter] = self.time - self.end_thrusting
            pass
        super().logData()

    def deregister_node(self):
        super().deregister_node()
        os.system("pkill -9 gzserver 2>/dev/null")
        os.system("pkill -9 gzclient 2>/dev/null")

    def resetBase(self, p0=None):
        # create the message
        req_reset_joints = SetModelConfiguration.Request()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        req_reset_joints.joint_names = self.joint_names
        req_reset_joints.joint_positions = conf.robot_params[self.robot_name]['q_0'].tolist()

        if p0 is None:
            # recompute the theta angle to have the anchor attached to the wall
            req_reset_joints.joint_positions[self.anchor_passive_joints[0]] = math.atan2(self.r_leg, conf.robot_params[self.robot_name]['q_0'][p.rope_index])
        else:  # do IK on p0 overwrite the first anchor joints and the rope joint
            if self.robot_name == 'climbingrobot_slider':
                req_reset_joints.joint_positions[self.anchor_passive_joints[0]], req_reset_joints.joint_positions[self.anchor_passive_joints[1]], req_reset_joints.joint_positions[self.rope_index] = self.computeJointVariables(p0, conf.robot_params[self.robot_name]['q_0'][self.slider_index])
            else:
                req_reset_joints.joint_positions[self.anchor_passive_joints[0]], req_reset_joints.joint_positions[self.anchor_passive_joints[1]], req_reset_joints.joint_positions[self.rope_index] = self.computeJointVariables(p0)
            beta1, self.rope_normal = self.computeRopeAngle(p0)
            beta2, max_Fut, max_Fun = self.getImpulseAngle()
            print("\033[34m" + "impulses are: Fun : ", max_Fun, " and  Fut : ", max_Fut)
            # set on wire yaw joint
            p.q_des[self.wire_yaw_joint] = (np.pi / 2 - beta1) + beta2
            req_reset_joints.joint_positions[self.wire_yaw_joint] = p.q_des[self.wire_yaw_joint]

        # send request and get response (in this case none)
        req_reset_joints.joint_positions = list(map(float, req_reset_joints.joint_positions))
        self._call_service(self.reset_joints_client, req_reset_joints)

        print(colored(f"---------Resetting Base", "blue"))

    def spawnMountain(self):
        # DEPRECATED: the wall is spawned from the launch/world file instead. In ROS1
        # this used the roslaunch scriptapi to run gazebo_ros spawn_model. In ROS2 the
        # equivalent is a `ros2 run gazebo_ros spawn_entity.py` call.
        import subprocess
        args = ['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'mountain', '-topic', 'climb_wall',
                '-x', str(conf.robot_params[self.robot_name]['spawn_x'] - self.mountain_thickness / 2),
                '-y', str(conf.robot_params[self.robot_name]['spawn_y'])]
        self.mountain_process = subprocess.Popen(args)

    def startupProcedure(self):
        if not self.EXTERNAL_FORCE:
            p.resetBase()
        super().startupProcedure()

    def plotStuff(self):
        if p.numberOfJumps < 2:  # do plots only for one jump
            print("PLOTTING")
            traj_gazebo = p.base_pos_log - p.anchor_pos.reshape(3, 1)  # is in anchor frame
            time_gazebo = p.time_log - p.start_logging
            plotJoint('position', time_gazebo, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
            plot3D('basePos', 2, ['X', 'Y', 'Z'], time_gazebo, traj_gazebo, p.matvars['solution'].time, p.matvars['solution'].p)
            plot3D('states', 3, ['theta', 'phi', 'l'], time_gazebo, p.simp_model_state_log[:3, :])
            vars = {'solution': p.matvars['solution'], 'T_th': p.matvars['T_th'], 'mu': p.matvars['mu'],
                    'Fun_max': p.matvars['Fun_max'], 'Fr_max': p.matvars['Fr_max'], 'p0': p.matvars['p0'], 'pf': p.matvars['pf'],
                    'time_gazebo': time_gazebo,
                    'traj_gazebo': traj_gazebo,
                    'theta_gazebo': p.simp_model_state_log[0, :],
                    'phi_gazebo': p.simp_model_state_log[1, :],
                    'l_gazebo': p.simp_model_state_log[2, :]}

            if p.robot_name == 'climbingrobot_slider':
                vars['sy_gazebo'] = p.simp_model_state_log[3, :]
            mio.savemat('test_gazebo.mat', vars)

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
        if p.MARCO_APPROACH:
            max_Fut = self.jumps[self.jumpNumber]["Fut"][self.getIndex(self.jumps[self.jumpNumber]["thrustDuration"] / 2)]
            max_Fun = self.jumps[self.jumpNumber]["Fun"][self.getIndex(self.jumps[self.jumpNumber]["thrustDuration"] / 2)]
            angle = math.atan2(max_Fut, max_Fun)
        else:
            angle = math.atan2(self.jumps[self.jumpNumber]["Fut"],
                               self.jumps[self.jumpNumber]["Fun"])

        return angle, max_Fut, max_Fun

    def computeJointVariables(self, p, slider=None):
        if self.robot_name == 'climbingrobot_slider':
            wire_base_prismatic = math.sqrt(p[0] * p[0] + (p[1] - slider) * (p[1] - slider) + p[2] * p[2])
            mountain_wire_pitch = math.atan2(p[0], -p[2])
            mountain_wire_roll = math.asin((p[1] - slider) / wire_base_prismatic)
        else:
            wire_base_prismatic = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
            mountain_wire_pitch = math.atan2(p[0], -p[2])
            mountain_wire_roll = math.asin(p[1] / wire_base_prismatic)
        return [mountain_wire_pitch, mountain_wire_roll, wire_base_prismatic]


def talker(p):

    p.start()
    if p.robot_name == 'climbingrobot_slider':
        xacro_path = get_package_share_directory('climbingrobot_description') + '/urdf/' + p.robot_name + '.xacro'
    else:
        xacro_path = None

    p.startSimulator(world_name="climbingrobot.world")
    p.loadModelAndPublishers(xacro_path=xacro_path)

    p.startupProcedure()
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    # loop frequency
    rate = p.node.create_rate(1 / conf.robot_params[p.robot_name]['dt'])
    p.updateKinematicsDynamics()
    p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravity in the virtual joint to go exactly there
    p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

    # override with matlab stuff
    if p.LOAD_MATLAB_TRAJ:
        if p.MARCO_APPROACH:
            # single jump
            p.matvars = mio.loadmat('test_optim_marco.mat', squeeze_me=True, struct_as_record=False)
            p.jumps = [{"time": p.matvars['solution'].time, "thrustDuration": p.matvars['T_th'], "p0": p.matvars['p0'], "targetPos": p.matvars['pf'], "Fun": p.matvars['solution'].Fun, "Fut": p.matvars['solution'].Fut, "Fr": p.matvars['solution'].Fr, "K_rope": 0.0, "Tf": p.matvars['solution'].Tf}]
        else:  # my approach
            p.matvars = mio.loadmat('test_optim.mat', squeeze_me=True, struct_as_record=False)
            p.jumps = [{"thrustDuration": p.matvars['T_th'], "p0": p.matvars['p0'], "targetPos": p.matvars['pf'], "Fun": p.matvars['solution'].Fun, "Fut": p.matvars['solution'].Fut, "Fr": p.matvars['solution'].Fr, "K_rope": 0.0, "Tf": p.matvars['solution'].Tf - p.matvars['T_th']}]

    # jump parameters
    p.startJump = 0.5
    p.orientTime = 0.5
    p.stateMachine = 'idle'
    p.jumpNumber = 0
    p.numberOfJumps = len(p.jumps)
    p.start_logging = np.inf

    while rclpy.ok():

        # update the kinematics
        p.updateKinematicsDynamics()

        # multiple jumps state machine
        if (p.stateMachine == 'idle') and (p.time >= p.startJump) and (p.jumpNumber < p.numberOfJumps):
            print("\033[34m" + "---------Starting jump  number ", p.jumpNumber, " to target: ", p.jumps[p.jumpNumber]["targetPos"], " from p0 : ", p.jumps[p.jumpNumber]["p0"])

            p.l_0 = p.l
            if p.jumpNumber == 0:  # do only once
                p.resetBase(p.jumps[p.jumpNumber]["p0"])
            if (p.EXTERNAL_FORCE):
                print(colored("Start applying force", "red"))
                p.applyForce(p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"], 0., p.jumps[p.jumpNumber]["thrustDuration"])
                p.stateMachine = 'thrusting'
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                p.end_thrusting = p.startJump + p.jumps[p.jumpNumber]["thrustDuration"]
                print(colored("Start Thrusting with EXT FORCE", "blue"))
            else:
                print(colored(f"Start orienting leg to  : {p.getImpulseAngle()[0]}", "blue"))
                p.stateMachine = 'orienting_leg'
                # set the end of orienting
                p.end_orienting = p.startJump + p.orientTime
                p.end_thrusting = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"]

            if p.MARCO_APPROACH:
                p.start_logging = p.end_orienting
            else:
                p.start_logging = p.end_thrusting

        if (p.stateMachine == 'orienting_leg') and (p.time >= p.end_orienting):
            print(colored(f"Start trusting", "blue"))
            p.tau_ffwd = np.zeros(p.robot.na)
            p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravity in the virtual joint to go exactly there
            p.pid.setPDjoint(p.base_passive_joints, 0., 0., 0.)
            p.pid.setPDjoint(p.leg_index, 0., 0., 0.)
            print(colored(f"ZERO LEG AND ROPE PD", "red"))
            p.stateMachine = 'thrusting'
            if p.MARCO_APPROACH:
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)

        if (p.stateMachine == 'thrusting'):
            if (p.EXTERNAL_FORCE):
                p.ros_pub.add_arrow(p.base_pos, np.array([p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"], 0.]) / 50., "red", scale=4.5)
            else:
                if not p.MARCO_APPROACH:
                    p.b_Fu_xy = np.array([p.jumps[p.jumpNumber]["Fun"], p.jumps[p.jumpNumber]["Fut"]])
                else:
                    Fu = np.array([p.jumps[p.jumpNumber]["Fun"][p.getIndex(p.time - p.end_orienting)], p.jumps[p.jumpNumber]["Fut"][p.getIndex(p.time - p.end_orienting)]])
                    Fu_norm = np.linalg.norm(Fu)
                    p.b_Fu_xy = np.hstack((Fu_norm, 0.))

                    # start already to use the rope
                    Fr = p.jumps[p.jumpNumber]["Fr"][p.getIndex(p.time - p.end_orienting)]
                    p.tau_ffwd[p.rope_index] = Fr
                    rope_direction = (p.base_pos - p.anchor_pos) / np.linalg.norm(p.base_pos - p.anchor_pos)
                    p.ros_pub.add_arrow(p.base_pos, rope_direction * Fr / 50., "red", scale=4.5)

                # using all 3 leg joints to generate impulse
                p.w_Fu = p.w_R_b.dot(np.hstack((p.b_Fu_xy, 0)))
                p.tau_ffwd[p.leg_index] = - p.Jleg.T.dot(p.w_Fu)
                p.ros_pub.add_arrow(p.x_ee, p.w_Fu / 50., "red", scale=4.5)

            if (p.time > p.end_thrusting):
                print(colored("Stop Trhusting", "blue"))
                p.stateMachine = 'flying'
                print(colored(f"RESTORING LEG PD", "red"))
                p.pid.setPDjoint(p.rope_index, 0., 0., 0.)
                # reenable  the PDs of default values for landing and reset the torque on the leg
                p.pid.setPDjoint(p.base_passive_joints, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)
                # reenable leg pd
                p.pid.setPDjoint(p.leg_index, conf.robot_params[p.robot_name]['kp'], conf.robot_params[p.robot_name]['kd'], 0.)
                p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                print(colored("Start Flying", "blue"))

        if (p.stateMachine == 'flying'):
            # keep extending rope
            if p.LOAD_MATLAB_TRAJ:
                if not p.MARCO_APPROACH:
                    delta_t = p.time - p.end_thrusting
                else:
                    delta_t = p.time - p.end_orienting
                Fr = p.jumps[p.jumpNumber]["Fr"][p.getIndex(delta_t)]
            else:
                Fr = -p.jumps[p.jumpNumber]["K_rope"] * (p.l - p.l_0)
            p.tau_ffwd[p.rope_index] = Fr
            rope_direction = (p.base_pos - p.anchor_pos) / np.linalg.norm(p.base_pos - p.anchor_pos)
            p.ros_pub.add_arrow(p.base_pos, rope_direction * Fr / 50., "red", scale=4.5)

            if p.robot_name == 'climbingrobot_slider':
                p.q_des[p.slider_index] += 0.001

            if (p.EXTERNAL_FORCE):
                end_flying = p.startJump + p.jumps[p.jumpNumber]["thrustDuration"] + p.jumps[p.jumpNumber]["Tf"]
            else:
                end_flying = p.startJump + p.orientTime + p.jumps[p.jumpNumber]["thrustDuration"] + p.jumps[p.jumpNumber]["Tf"]

            if (p.time >= end_flying):
                print(colored("Stop Flying", "blue"))
                # reset the qdes
                p.stateMachine = 'idle'
                print(colored(f"RESTORING ROPE PD", "red"))
                p.q_des[p.rope_index] = np.copy(p.q[p.rope_index])
                p.l_0 = np.copy(p.l)
                p.tau_ffwd[p.rope_index] = 0
                p.pid.setPDjoint(p.rope_index, conf.robot_params[p.robot_name]['kp'][p.rope_index], conf.robot_params[p.robot_name]['kd'][p.rope_index], 0.)
                p.jumpNumber += 1

                if (p.jumpNumber < p.numberOfJumps):
                    # reset for multiple jumps
                    p.startJump = p.time
                else:
                    p._call_service(p.pause_physics_client, Empty.Request())
                    break

        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        if (p.time >= p.start_logging):
            p.logData()

        # plot  rope bw base link and anchor
        if p.robot_name == 'climbingrobot_slider':
            rope_attach_point = p.robot.framePlacement(p.q, p.robot.model.getFrameId('sliding_anchor')).translation
        else:
            rope_attach_point = p.anchor_pos

        p.ros_pub.add_arrow(rope_attach_point, (p.base_pos - rope_attach_point), "green", scale=3.)  # rope, already in gazebo
        p.ros_pub.add_marker(p.anchor_pos + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.8)

        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        if (p.jumpNumber == p.numberOfJumps):
            p.ros_pub.add_marker(p.x_ee, color="green", radius=0.15)
        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        p.ros_pub.publishVisual()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4)  # to avoid issues of dt 0.0009999


def plot3D(name, figure_id, label, time_log, var, time_mat=None, var_mat=None):
    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)

    plt.subplot(3, 1, 1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

    plt.subplot(3, 1, 2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])

    plt.subplot(3, 1, 3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    plt.grid()
    plt.legend(['sim', 'matlab'])


p = None


def main(args=None):
    global p
    p = ClimbingrobotController(robot_name=robotName)

    try:
        talker(p)
    except (KeyboardInterrupt, RuntimeError):
        if rclpy.ok():
            rclpy.shutdown()
        p.deregister_node()
    finally:
        p.plotStuff()


if __name__ == '__main__':
    main()
