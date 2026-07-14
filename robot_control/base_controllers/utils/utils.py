# ROS2 (Humble) port of base_controllers/utils/utils.py
# --------------------------------------------------------------------------
# Translation notes (rospy -> rclpy):
#   * ROS1 has a global parameter server; ROS2 parameters are node-scoped.
#     The `putIntoParamServer` / `putIntoGlobalParamServer` helpers therefore
#     become lightweight no-ops (they only ever stored debug flags that nothing
#     in the simulation path reads back). The `roslib`/`rospy` XMLRPC helpers
#     (`get_param_server`, `make_caller_id`, `succeed`) are dropped for the same
#     reason.
# Everything else in this class is pure NumPy index/geometry bookkeeping and is
# copied verbatim from the ROS1 source.
from __future__ import print_function

import copy
import os

import matplotlib.pyplot as plt
import numpy as np


class Utils:

    def __init__(self):

        self.leg_map = {
            "LF": 0,
            "LH": 1,
            "RF": 2,
            "RH": 3
        }
        self.crd = {
            "X": 0,
            "Y": 1,
            "Z": 2,
        }

        self.sp_crd = {
            "LX": 0,
            "LY": 1,
            "LZ": 2,
            "AX": 3,
            "AY": 4,
            "AZ": 5,
        }

    def getSegment(self, var, index, size):
        return var[index:index+size]

    def linPart(self, var):
        index = self.sp_crd["LX"]
        return var[index:index+3]

    def angPart(self, var):
        index = self.sp_crd["AX"]
        return var[index:index+3]

    ########################################################################
    # manage param server
    #
    # ROS2 has no global parameter server (parameters are node-scoped), so the
    # ROS1 param-server helpers are reduced to no-ops. They only ever stored
    # debug flags that nothing in the simulation control path reads back.
    ########################################################################

    def putIntoParamServer(self, data):
        # No-op in ROS2 (no global parameter server). Kept for API compatibility.
        print("putIntoParamServer is a no-op in ROS2 (data=%s)" % (data,))

    def putIntoGlobalParamServer(self, label, data, verbose=False):
        # No-op in ROS2 (no global parameter server). Kept for API compatibility.
        if verbose:
            print("putIntoGlobalParamServer is a no-op in ROS2 (label=%s)" % label)

#########################################################################

    def getIdx(self, leg, coord):
        return self.leg_map[leg]*3 + self.crd[coord]

    def setLegJointState(self, legid,  input, jointState):
        if isinstance(legid, str):
            jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3] = input
        elif isinstance(legid, int):
            jointState[legid*3:legid*3+3] = input

    def getLegJointState(self, legid,  jointState):
        if isinstance(legid, str):
            return jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3]
        elif isinstance(legid, int):
            return jointState[legid * 3:legid * 3 + 3]

    def spy(self, var):
        plt.spy(var)
        plt.show()

    def detectLiftOff(self, swing, idx, leg):
        if ((swing[leg, idx-1] == 0) and (swing[leg, idx] == 1)):
            return True
        else:
            return False

    def detectTouchDown(self, swing, idx, leg):
        if ((swing[leg, idx] == 1) and (swing[leg, idx+1] == 0)):
            return True
        else:
            return False

    def detectHapticTouchDown(self, grForcesW, leg, force_th):
        grfleg = self.getLegJointState(leg, grForcesW)
        if grfleg[2] >= force_th:
            return True
        else:
            return False

    def mapFromRos(self, ros_in):
        return ros_in

    def mapToRos(self, ros_in):
        return ros_in

    def mapIndexToRos(self, index_in: object) -> object:
        return index_in

    def mapLegListToRos(self, list: object) -> object:
        return list

    def get_dict_keys(dict):
        names = list(dict.keys())
        names.sort()
        return names

    def tic():
        # Homemade version of matlab tic and toc functions
        import time
        global startTime_for_tictoc
        startTime_for_tictoc = time.time()

    def toc():
        import time
        if 'startTime_for_tictoc' in globals():
            print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
        else:
            print("Toc: start time not set")

    def full_listOfArrays(self, length, rows, cols=0, value=np.nan):
        # create a list of length independent np.ndarrays of shape (rows, cols)
        # with all the entries set to value
        if cols == 0:
            a = np.full(rows, value)
        else:
            a = np.full((rows, cols), value)
        return self.listOfArrays(length, a)

    def listOfArrays(self, length, array):
        # create a list of length independent np.ndarrays
        L = []
        for i in range(length):
            L.append(copy.deepcopy(array))
        return L
