#!/usr/bin/env python3
"""
jump_optimizer_node
===================

ROS 2 (Humble) node that wraps the *offline* jump optimal-control solver
(:func:`climbingrobot_optimization.jump_optimizer.optimize_cpp`).

This is the online-callable equivalent of running
``optimal_control_2_ropes.m`` once to plan a jump.  Because the ALPINE project
does not ship custom ROS interface (.srv / .msg) packages, this node exposes
its I/O through **parameters** (inputs) and **std_msgs / geometry_msgs topics**
(outputs), which keeps the package a pure ``ament_python`` package with no
``rosidl`` build step.  If you already have a *_msgs* package, swap the
publishers/subscribers for your own message types.

Trigger
-------
Publish an empty message on ``~/plan_jump`` (``std_msgs/Empty``) to (re)solve
the jump.  The node also solves once automatically at start-up if
``solve_on_start`` is true.

Inputs (ROS parameters)
-----------------------
p0                 : [x, y, z] start CoM position          (default [0.5, 2.5, -6.0])
pf                 : [x, y, z] target CoM position         (default [0.5, 4.0, -4.0])
Fleg_max           : max leg impulse force [N]             (default 300.0)
Fr_max             : max rope retraction force [N]         (default 90.0)
mu                 : friction coefficient                  (default 0.8)
mass               : robot mass [kg]                       (default 5.08)
anchor_distance    : distance between the two anchors [m]  (default 5.0)
N_dyn              : number of discretisation knots        (default 30)
obstacle_avoidance : enable obstacle avoidance constraint  (default False)
solve_on_start     : solve once when the node starts       (default True)

Outputs (topics)
----------------
~/reference_com   (std_msgs/Float64MultiArray) : flattened 3 x N_dyn CoM ref
~/reference_frl   (std_msgs/Float64MultiArray) : nominal rope-1 forces (N_dyn)
~/reference_frr   (std_msgs/Float64MultiArray) : nominal rope-2 forces (N_dyn)
~/jump_time       (std_msgs/Float64)           : flight time Tf [s]
~/achieved_target (geometry_msgs/Point)        : simulated landing CoM position
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point

from climbingrobot_optimization.params import Params
from climbingrobot_optimization import jump_optimizer, dynamics


def _to_multiarray(arr):
    """Pack a numpy array (1-D or 2-D) into a Float64MultiArray."""
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


class JumpOptimizerNode(Node):
    def __init__(self):
        super().__init__("jump_optimizer_node")

        # --- declare parameters -----------------------------------------
        self.declare_parameter("p0", [0.5, 2.5, -6.0])
        self.declare_parameter("pf", [0.5, 4.0, -4.0])
        self.declare_parameter("Fleg_max", 300.0)
        self.declare_parameter("Fr_max", 90.0)
        self.declare_parameter("mu", 0.8)
        self.declare_parameter("mass", 5.08)
        self.declare_parameter("anchor_distance", 5.0)
        self.declare_parameter("N_dyn", 30)
        self.declare_parameter("obstacle_avoidance", False)
        self.declare_parameter("solve_on_start", True)

        # --- publishers --------------------------------------------------
        self.pub_ref_com = self.create_publisher(Float64MultiArray, "~/reference_com", 1)
        self.pub_ref_frl = self.create_publisher(Float64MultiArray, "~/reference_frl", 1)
        self.pub_ref_frr = self.create_publisher(Float64MultiArray, "~/reference_frr", 1)
        self.pub_tf = self.create_publisher(Float64, "~/jump_time", 1)
        self.pub_target = self.create_publisher(Point, "~/achieved_target", 1)

        # --- trigger subscription ---------------------------------------
        self.create_subscription(Empty, "~/plan_jump", self._on_trigger, 1)

        self.get_logger().info(
            "jump_optimizer_node ready (C++ kernel: %s)"
            % ("yes" if dynamics.kernel_available() else "numpy fallback"))

        if self.get_parameter("solve_on_start").value:
            # defer to allow the executor to spin up publishers
            self.create_timer(0.5, self._solve_once)
            self._solved_on_start = False

    # ------------------------------------------------------------------
    def _build_params(self):
        p = Params.normal_test()
        p.m = float(self.get_parameter("mass").value)
        p.b = float(self.get_parameter("anchor_distance").value)
        p.p_a2 = np.array([0.0, p.b, 0.0])
        p.N_dyn = int(self.get_parameter("N_dyn").value)
        p.obstacle_avoidance = bool(self.get_parameter("obstacle_avoidance").value)
        return p

    def _solve_once(self):
        if getattr(self, "_solved_on_start", True):
            return
        self._solved_on_start = True
        self.plan_jump()

    def _on_trigger(self, _msg):
        self.plan_jump()

    # ------------------------------------------------------------------
    def plan_jump(self):
        p = self._build_params()
        p0 = np.array(self.get_parameter("p0").value, dtype=float)
        pf = np.array(self.get_parameter("pf").value, dtype=float)
        Fleg_max = float(self.get_parameter("Fleg_max").value)
        Fr_max = float(self.get_parameter("Fr_max").value)
        mu = float(self.get_parameter("mu").value)

        self.get_logger().info("Solving jump: p0=%s -> pf=%s" % (p0.tolist(), pf.tolist()))
        try:
            sol = jump_optimizer.optimize_cpp(p0, pf, Fleg_max, Fr_max, mu, p, verbose=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("Jump optimisation failed: %s" % exc)
            return

        flag = sol.get("problem_solved", 0)
        self.get_logger().info(
            "solved flag=%d Tf=%.3f achieved=%s"
            % (flag, sol["Tf"], np.round(sol["achieved_target"], 3).tolist()))

        # publish results
        self.pub_ref_com.publish(_to_multiarray(sol["p"]))
        self.pub_ref_frl.publish(_to_multiarray(sol["Fr_l"]))
        self.pub_ref_frr.publish(_to_multiarray(sol["Fr_r"]))
        tf_msg = Float64(); tf_msg.data = float(sol["Tf"]); self.pub_tf.publish(tf_msg)
        tgt = Point()
        tgt.x, tgt.y, tgt.z = [float(v) for v in sol["achieved_target"]]
        self.pub_target.publish(tgt)


def main(args=None):
    rclpy.init(args=args)
    node = JumpOptimizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
