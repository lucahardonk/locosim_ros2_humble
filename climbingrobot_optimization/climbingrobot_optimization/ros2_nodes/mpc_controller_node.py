#!/usr/bin/env python3
"""
mpc_controller_node
===================

ROS 2 (Humble) node that wraps the *online* receding-horizon MPC solver
(:func:`climbingrobot_optimization.mpc_controller.optimize_cpp_mpc`).

This is the online-callable equivalent of the ``mpc_loop.m`` inner loop: at a
fixed rate it takes the current measured robot state and the previously planned
reference CoM trajectory, solves a short-horizon correction to the nominal rope
forces, and publishes the corrected forces.

As with :mod:`jump_optimizer_node`, I/O uses ROS parameters and
``std_msgs`` topics so the package stays a pure ``ament_python`` package with no
custom ``rosidl`` interfaces.

Reference input
---------------
Latch the planner output before starting control:
  ~/set_reference_com (std_msgs/Float64MultiArray) : flattened 3 x N_dyn CoM ref
  ~/set_reference_frl (std_msgs/Float64MultiArray) : nominal rope-1 forces
  ~/set_reference_frr (std_msgs/Float64MultiArray) : nominal rope-2 forces
  ~/set_jump_time     (std_msgs/Float64)           : flight time Tf (sets mpc_dt)

State feedback
--------------
  ~/state (std_msgs/Float64MultiArray) : current [psi, l1, l2, psid, l1d, l2d]

Output
------
  ~/rope_forces (std_msgs/Float64MultiArray) : corrected [Fr_l, Fr_r] (2 x mpc_N)

Parameters
----------
Fr_max          : max rope force [N]                 (default 90.0)
mass            : robot mass [kg]                     (default 5.08)
anchor_distance : anchor separation [m]               (default 5.0)
N_dyn           : discretisation knots of the plan    (default 30)
horizon_frac    : mpc_N = round(horizon_frac * N_dyn) (default 0.4)
control_rate_hz : MPC solve rate [Hz]                 (default 20.0)
use_propellers  : enable the propeller-augmented MPC  (default False)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension

from climbingrobot_optimization.params import Params
from climbingrobot_optimization import mpc_controller, dynamics


def _to_multiarray(arr):
    arr = np.asarray(arr, dtype=float)
    msg = Float64MultiArray()
    msg.layout.dim = []
    for i, n in enumerate(arr.shape):
        dim = MultiArrayDimension()
        dim.label = "dim%d" % i
        dim.size = int(n)
        dim.stride = int(np.prod(arr.shape[i:]))
        msg.layout.dim.append(dim)
    msg.data = arr.flatten(order="C").tolist()
    return msg


class MpcControllerNode(Node):
    def __init__(self):
        super().__init__("mpc_controller_node")

        self.declare_parameter("Fr_max", 90.0)
        self.declare_parameter("mass", 5.08)
        self.declare_parameter("anchor_distance", 5.0)
        self.declare_parameter("N_dyn", 30)
        self.declare_parameter("horizon_frac", 0.4)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("use_propellers", False)

        self.params = Params.normal_test()
        self.params.m = float(self.get_parameter("mass").value)
        self.params.b = float(self.get_parameter("anchor_distance").value)
        self.params.p_a2 = np.array([0.0, self.params.b, 0.0])
        self.params.N_dyn = int(self.get_parameter("N_dyn").value)

        self.Fr_max = float(self.get_parameter("Fr_max").value)
        self.mpc_N = int(round(float(self.get_parameter("horizon_frac").value)
                               * self.params.N_dyn))
        self.use_propellers = bool(self.get_parameter("use_propellers").value)

        # reference / state buffers (populated over topics)
        self.ref_com = None      # 3 x N_dyn
        self.Fr_l0 = None        # N_dyn
        self.Fr_r0 = None        # N_dyn
        self.state = None        # 6-vector
        self.Tf = None
        self.t = 0.0

        # subscriptions
        self.create_subscription(Float64MultiArray, "~/set_reference_com",
                                 self._on_ref_com, 1)
        self.create_subscription(Float64MultiArray, "~/set_reference_frl",
                                 self._on_ref_frl, 1)
        self.create_subscription(Float64MultiArray, "~/set_reference_frr",
                                 self._on_ref_frr, 1)
        self.create_subscription(Float64, "~/set_jump_time", self._on_tf, 1)
        self.create_subscription(Float64MultiArray, "~/state", self._on_state, 1)

        # output
        self.pub_forces = self.create_publisher(Float64MultiArray, "~/rope_forces", 1)

        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1e-3), self._control_step)

        self.get_logger().info(
            "mpc_controller_node ready (mpc_N=%d, propellers=%s, C++ kernel: %s)"
            % (self.mpc_N, self.use_propellers,
               "yes" if dynamics.kernel_available() else "numpy fallback"))

    # -- callbacks ------------------------------------------------------
    def _on_ref_com(self, msg):
        data = np.array(msg.data, dtype=float)
        self.ref_com = data.reshape(3, -1)

    def _on_ref_frl(self, msg):
        self.Fr_l0 = np.array(msg.data, dtype=float)

    def _on_ref_frr(self, msg):
        self.Fr_r0 = np.array(msg.data, dtype=float)

    def _on_tf(self, msg):
        self.Tf = float(msg.data)
        self.params.mpc_dt = self.Tf / (self.params.N_dyn - 1)

    def _on_state(self, msg):
        self.state = np.array(msg.data, dtype=float)

    # -- control loop ---------------------------------------------------
    def _ready(self):
        return (self.ref_com is not None and self.Fr_l0 is not None
                and self.Fr_r0 is not None and self.state is not None
                and self.params.mpc_dt > 0.0)

    def _control_step(self):
        if not self._ready():
            return
        try:
            if self.use_propellers:
                x, flag, fval = mpc_controller.optimize_cpp_mpc_propellers(
                    self.state, self.t, self.ref_com, self.Fr_l0, self.Fr_r0,
                    self.Fr_max, self.mpc_N, self.params)
            else:
                x, flag, fval = mpc_controller.optimize_cpp_mpc(
                    self.state, self.t, self.ref_com, self.Fr_l0, self.Fr_r0,
                    self.Fr_max, self.mpc_N, self.params)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("MPC solve failed: %s" % exc)
            return

        delta_Fr_l = x[:self.mpc_N]
        delta_Fr_r = x[self.mpc_N:2 * self.mpc_N]
        # corrected forces = nominal + MPC correction (clipped to the horizon)
        Fr_l = self.Fr_l0[:self.mpc_N] + delta_Fr_l
        Fr_r = self.Fr_r0[:self.mpc_N] + delta_Fr_r
        self.pub_forces.publish(_to_multiarray(np.vstack([Fr_l, Fr_r])))

        self.get_logger().debug(
            "MPC exit=%d cost=%.4f |dFr_l|=%.2f |dFr_r|=%.2f"
            % (flag, fval, np.max(np.abs(delta_Fr_l)), np.max(np.abs(delta_Fr_r))))


def main(args=None):
    rclpy.init(args=args)
    node = MpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
