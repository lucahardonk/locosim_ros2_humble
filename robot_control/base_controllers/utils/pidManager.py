# ROS2 (Humble) port of base_controllers/utils/pidManager.py
# --------------------------------------------------------------------------
# Translation notes (rospy -> rclpy):
#   * rospy.wait_for_service / rospy.ServiceProxy -> node.create_client + wait_for_service
#   * service call now uses call_async(); the message payloads and the PID
#     selection logic (per-robot, per-leg, per-joint) are unchanged.
#   * The custom interfaces SetPids / Pid still come from the
#     ros_impedance_controller package (built with rosidl_generate_interfaces).
#     In ROS2 the request wrapper is SetPids.Request (there is no separate
#     SetPidsRequest type as in ROS1).
import copy
import numpy as np
from termcolor import colored

from ros_impedance_controller.srv import SetPids
from ros_impedance_controller.msg import Pid


class PidManager:

    def __init__(self, node, jnames):
        print("Initializing PID Manager")
        self.node = node
        self.joint_names = jnames
        self.set_pd_service = self.node.create_client(SetPids, "/set_pids")
        if not self.set_pd_service.wait_for_service(timeout_sec=5.0):
            print(colored("PID Manager: Service call /set_pids non available, check ros_impedance_controller node", "red"))
        self.joint_pid = Pid()
        self.joint_pid_log = len(jnames) * [Pid()]
        self.req_msg = SetPids.Request()
        print(colored("PID Manager initialized", "red"))

    def getPDs(self):
        return self.joint_pid_log

    def setPDs(self, kp, kd, ki=0):
        """
         Set the same values of PID for all the joints of the robot
         @kp: proportional gain (scalar)
         @kd: derivative gain (scalar)
         @kp: integral gain (scalar)
         """
        # create the message
        self.req_msg.data = []

        # fill in the message with des values for kp kd
        for i in range(len(self.joint_names)):
            self.joint_pid.joint_name = self.joint_names[i]
            self.joint_pid.p_value = kp
            self.joint_pid.d_value = kd
            self.joint_pid.i_value = ki
            self.req_msg.data += [copy.deepcopy(self.joint_pid)]
            self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

        # send request and get response (in this case none)
        self.set_pd_service.call_async(self.req_msg)

    def setPDleg(self, legid, kp, kd, ki):
        """
        Set PDs the same values for the joints of a leg of a quadruped
        @kp: proportional gain (scalar)
        @kd: derivative gain (scalar)
        @kp: integral gain (scalar)
        """
        # create the message
        self.req_msg.data = []

        # fill in the message with des values for kp kd
        for i in range(len(self.joint_names)):

            if (legid == 0):
                if ((self.joint_names[i] == 'lf_haa_joint') or (self.joint_names[i] == 'lf_hfe_joint') or (
                        self.joint_names[i] == 'lf_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

            if (legid == 1):
                if ((self.joint_names[i] == 'rf_haa_joint') or (self.joint_names[i] == 'rf_hfe_joint') or (
                        self.joint_names[i] == 'rf_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

            if (legid == 2):
                if ((self.joint_names[i] == 'lh_haa_joint') or (self.joint_names[i] == 'lh_hfe_joint') or (
                        self.joint_names[i] == 'lh_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

            if (legid == 3):
                if ((self.joint_names[i] == 'rh_haa_joint') or (self.joint_names[i] == 'rh_hfe_joint') or (
                        self.joint_names[i] == 'rh_kfe_joint')):
                    self.joint_pid.joint_name = self.joint_names[i]
                    self.joint_pid.p_value = kp
                    self.joint_pid.d_value = kd
                    self.joint_pid.i_value = ki
                    self.joint_pid_log[i] = copy.deepcopy(self.joint_pid)

        self.req_msg.data = self.joint_pid_log

        # send request and get response (in this case none)
        self.set_pd_service.call_async(self.req_msg)

    def setPDjoint(self, joint_idx, kp, kd, ki):
        """
        Set value of PID for a specific joint or a set of joints
        @joint_idx: (int) index of the joint /(array) of indices of the set of joints
        @kp: proportional gain (int/array)
        @kd: derivative gain (int/array)
        @kp: integral gain (int/array)
        """
        # create the message
        self.req_msg.data = []
        if isinstance(joint_idx, int):
            # fill in the message with des values for kp kd
            self.joint_pid.joint_name = self.joint_names[joint_idx]
            self.joint_pid.p_value = float(kp)
            self.joint_pid.d_value = float(kd)
            self.joint_pid.i_value = float(ki)
            self.joint_pid_log[joint_idx] = copy.deepcopy(self.joint_pid)
        else:
            for joint in joint_idx:
                # fill in the message with des values for kp kd
                self.joint_pid.joint_name = self.joint_names[joint]
                # ROS2 rosidl messages strictly require native python float;
                # numpy.float64 (what config gain arrays contain) is rejected
                # with "The 'x_value' field must be of type 'float'". Cast.
                if not isinstance(kp, np.ndarray):
                    self.joint_pid.p_value = float(kp)
                else:
                    self.joint_pid.p_value = float(kp[joint])

                if not isinstance(kd, np.ndarray):
                    self.joint_pid.d_value = float(kd)
                else:
                    self.joint_pid.d_value = float(kd[joint])

                if not isinstance(ki, np.ndarray):
                    self.joint_pid.i_value = float(ki)
                else:
                    self.joint_pid.i_value = float(ki[joint])

                self.joint_pid_log[joint] = copy.deepcopy(self.joint_pid)

        self.req_msg.data = copy.deepcopy(self.joint_pid_log)

        # send request and get response (in this case none)
        self.set_pd_service.call_async(self.req_msg)

    def setPDjoints(self, kp, kd, ki):
        """
        Set array of values of PID for all joints
        @kp: proportional gain (array)
        @kd: derivative gain (array)
        @ki: integral gain (array)
        """
        for joint_idx in range(len(self.joint_names)):
            # fill in the message with des values for kp kd
            self.joint_pid.joint_name = self.joint_names[joint_idx]
            # ROS2 rosidl messages strictly require native python float;
            # numpy.float64 (config gain arrays) would raise
            # "The 'x_value' field must be of type 'float'". Cast.
            self.joint_pid.p_value = float(kp[joint_idx])
            self.joint_pid.d_value = float(kd[joint_idx])
            self.joint_pid.i_value = float(ki[joint_idx])
            self.joint_pid_log[joint_idx] = copy.deepcopy(self.joint_pid)

        self.req_msg.data = copy.deepcopy(self.joint_pid_log)

        # send request and get response (in this case none)
        self.set_pd_service.call_async(self.req_msg)

    def __repr__(self):
        string = f"Joint PID controller"
        for joint_pid in self.joint_pid_log:
            string += f"\njoint name: {joint_pid.joint_name} \t Kp: {joint_pid.p_value} \t Kd: {joint_pid.d_value} \t Ki: {joint_pid.i_value}"
        return string
