"""
Parameter container for the ALPINE climbing robot two-rope model.

Direct Python translation of the ``params`` struct used throughout the
MATLAB code in ``climbing_robots2/matlab/optimal_control``.

WORLD FRAME is attached to anchor 1 (``p_a1``).  Anchor 2 (``p_a2``) is a
distance ``b`` away along the world Y axis.

State vector convention (used everywhere):
    x = [psi, l1, l2, psid, l1d, l2d]
where
    psi  : swing angle of the rope plane about the anchor axis  [rad]
    l1   : length of rope 1 (from anchor 1)                     [m]
    l2   : length of rope 2 (from anchor 2)                     [m]
    *d   : corresponding time derivatives

Optimisation decision vector for the offline jump problem:
    z = [Fleg_x, Fleg_y, Fleg_z, Tf, Fr_l(0..N_dyn-1), Fr_r(0..N_dyn-1)]
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Params:
    # --- model constants --------------------------------------------------
    m: float = 5.08                # robot mass [kg]
    g: float = 9.81                # gravity [m/s^2]
    b: float = 5.0                 # anchor distance (== anchor_distance) [m]
    p_a1: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    p_a2: np.ndarray = field(default_factory=lambda: np.array([0.0, 5.0, 0.0]))
    contact_normal: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))

    # --- discretisation / integration ------------------------------------
    num_params: int = 4            # Fleg(3) + Tf(1)
    N_dyn: int = 30                # number of knots in the discretisation
    int_method: str = "rk4"        # 'rk4' or 'eul'
    int_steps: int = 5             # sub-steps per knot (0 => plain integration)
    T_th: float = 0.05             # leg thrust duration [s]

    # --- cost weights (offline jump optimiser) ----------------------------
    w1: float = 1.0                # diff(Fr) smoothing term
    w2: float = 0.0                # hoist work term
    w3: float = 0.0                # (unused / terminal)
    w4: float = 0.0                # (unused)
    w5: float = 0.0                # (unused)
    w6: float = 0.0                # (unused)

    # --- constraints / scenario flags -------------------------------------
    FRICTION_CONE: bool = True
    obstacle_avoidance: bool = False
    jump_clearance: float = 1.0
    obstacle_location: np.ndarray = field(default_factory=lambda: np.array([-0.5, 2.5, -6.0]))
    obstacle_size: np.ndarray = field(default_factory=lambda: np.array([1.5, 1.5, 0.866]))

    # --- MPC specific ------------------------------------------------------
    mpc_dt: float = 0.0            # set at run time = Tf / (N_dyn - 1)

    def __post_init__(self):
        # keep anchors / vectors as float numpy arrays (column-vector safe)
        self.p_a1 = np.asarray(self.p_a1, dtype=float).reshape(3)
        self.p_a2 = np.asarray(self.p_a2, dtype=float).reshape(3)
        self.contact_normal = np.asarray(self.contact_normal, dtype=float).reshape(3)
        self.obstacle_location = np.asarray(self.obstacle_location, dtype=float).reshape(3)
        self.obstacle_size = np.asarray(self.obstacle_size, dtype=float).reshape(3)

    # convenience factory reproducing the "normal" test in
    # optimal_control_2_ropes.m
    @staticmethod
    def normal_test():
        p = Params()
        p.m = 5.08
        p.b = 5.0
        p.p_a1 = np.array([0.0, 0.0, 0.0])
        p.p_a2 = np.array([0.0, p.b, 0.0])
        p.obstacle_avoidance = False
        p.jump_clearance = 1.0
        return p

    @staticmethod
    def landing_test():
        p = Params()
        p.m = 15.07
        p.b = 5.0
        p.p_a1 = np.array([0.0, 0.0, 0.0])
        p.p_a2 = np.array([0.0, p.b, 0.0])
        p.obstacle_avoidance = False
        p.jump_clearance = 1.0
        return p

    @staticmethod
    def obstacle_avoidance_test():
        p = Params()
        p.m = 5.08
        p.b = 5.0
        p.p_a1 = np.array([0.0, 0.0, 0.0])
        p.p_a2 = np.array([0.0, p.b, 0.0])
        p.obstacle_avoidance = True
        p.jump_clearance = 1.0
        p.obstacle_location = np.array([-0.5, 2.5, -6.0])
        p.obstacle_size = np.array([1.5, 1.5, 0.866])
        return p
