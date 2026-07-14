"""
climbingrobot_optimization
==========================

Python (+ optional C++) port of the MATLAB MPC controller and jump optimal
control code from the ALPINE ``climbing_robots2`` repository.

Modules
-------
params            : Params dataclass (model / discretisation / cost weights)
kinematics        : forward_kin, compute_jacobian, compute_position_velocity,
                    compute_state_from_cartesian
dynamics          : dynamics, integrate_dynamics, compute_rollout (C++ accel.)
jump_optimizer    : offline jump optimal control (SciPy SLSQP)
mpc_controller    : online receding-horizon MPC (SciPy SLSQP)
casadi_backend    : CasADi / IPOPT alternative solver
"""

from .params import Params
from . import kinematics
from . import dynamics
from . import jump_optimizer
from . import mpc_controller

__all__ = ["Params", "kinematics", "dynamics", "jump_optimizer", "mpc_controller"]
__version__ = "1.0.0"
