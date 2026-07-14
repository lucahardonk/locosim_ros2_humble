"""
Online MPC controller for the ALPINE climbing robot.

Faithful translation of:
    matlab/optimal_control/mpc/eval_pos_vel_mpc.m
    matlab/optimal_control/mpc/cost_mpc.m
    matlab/optimal_control/mpc/cost_mpc_propellers.m
    matlab/optimal_control/mpc/constraints_mpc.m
    matlab/optimal_control/mpc/optimize_cpp_mpc.m
    matlab/optimal_control/mpc/optimize_cpp_mpc_no_constraints.m
    matlab/optimal_control/mpc/optimize_cpp_mpc_propellers.m

The MPC tracks a reference CoM trajectory by optimising rope-force
corrections (delta_Fr_l, delta_Fr_r) — plus optional propeller forces — over
a receding horizon of ``mpc_N`` knots, using single shooting.  As in the
offline problem this is a nonlinear program (the rollout is nonlinear), so
SciPy SLSQP is used as the direct equivalent of MATLAB fmincon('sqp').

The public entry points reproduce the MATLAB signatures:
    optimize_cpp_mpc(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params)
    optimize_cpp_mpc_no_constraints(...)
    optimize_cpp_mpc_propellers(...)
returning (x, exitflag, final_cost).
"""

import numpy as np
from scipy.optimize import minimize, Bounds

from .kinematics import compute_position_velocity
from .dynamics import compute_rollout


# ---------------------------------------------------------------------------
# eval_pos_vel_mpc.m
# ---------------------------------------------------------------------------
def eval_pos_vel_mpc(state0, actual_t, Fr_l0, Fr_r0, delta_Fr_l, delta_Fr_r,
                     mpc_N, params, propeller_force=None, use_cpp=True):
    Fr_l0 = np.asarray(Fr_l0, float)
    Fr_r0 = np.asarray(Fr_r0, float)
    if len(Fr_l0) < mpc_N or len(Fr_r0) < mpc_N:
        raise ValueError("eval_pos_vel_mpc: input shorter than mpc_N")

    Fr_l0_mpc = Fr_l0[:mpc_N]
    Fr_r0_mpc = Fr_r0[:mpc_N]

    if propeller_force is None:
        states, t = compute_rollout(state0, actual_t, params.mpc_dt, mpc_N,
                                    Fr_l0_mpc + delta_Fr_l, Fr_r0_mpc + delta_Fr_r,
                                    np.zeros(3), params.int_method, params.int_steps,
                                    params, use_cpp=use_cpp)
    else:
        states, t = compute_rollout(state0, actual_t, params.mpc_dt, mpc_N,
                                    Fr_l0_mpc + delta_Fr_l, Fr_r0_mpc + delta_Fr_r,
                                    np.zeros(3), params.int_method, params.int_steps,
                                    params, extra_forces=propeller_force, use_cpp=use_cpp)

    psi, l1, l2, psid, l1d, l2d = states
    p, pd = compute_position_velocity(params, psi, l1, l2, psid, l1d, l2d)
    return p, pd, t


# ---------------------------------------------------------------------------
# cost_mpc.m / cost_mpc_propellers.m
# ---------------------------------------------------------------------------
def cost_mpc(x, state0, actual_t, ref_com, Fr_l0, Fr_r0, mpc_N, params,
             propellers=False, use_cpp=True):
    ref_com = np.asarray(ref_com, float)
    if ref_com.shape[1] < mpc_N:
        raise ValueError("cost_mpc: ref_com shorter than mpc_N")

    delta_Fr_l = x[:mpc_N]
    delta_Fr_r = x[mpc_N:2 * mpc_N]
    propeller_forces = x[2 * mpc_N:3 * mpc_N] if propellers else None

    ref_com_mpc = ref_com[:, :mpc_N]
    p, _, _ = eval_pos_vel_mpc(state0, actual_t, Fr_l0, Fr_r0, delta_Fr_l, delta_Fr_r,
                               mpc_N, params, propeller_forces, use_cpp=use_cpp)

    # cartesian tracking: sum of squared column-wise 2-norms
    tracking_cart = np.sum(np.sum((ref_com_mpc - p) ** 2, axis=0))
    smooth = np.sum(np.diff(delta_Fr_l) ** 2) + np.sum(np.diff(delta_Fr_r) ** 2)
    if propellers:
        smooth += np.sum(np.diff(propeller_forces) ** 2)

    return params.w1 * tracking_cart + params.w2 * smooth


# ---------------------------------------------------------------------------
# constraints_mpc.m  -> ineq <= 0  (Fr0 + delta_Fr <= 0 unilaterality)
# ---------------------------------------------------------------------------
def constraints_mpc(x, Fr_max, Fr_l0, Fr_r0, mpc_N):
    delta_Fr_l = x[:mpc_N]
    delta_Fr_r = x[mpc_N:2 * mpc_N]
    Fr_l0 = np.asarray(Fr_l0, float)[:mpc_N]
    Fr_r0 = np.asarray(Fr_r0, float)[:mpc_N]

    ineq = []
    for i in range(mpc_N):
        ineq.append(Fr_l0[i] + delta_Fr_l[i])   # <= 0
        ineq.append(Fr_r0[i] + delta_Fr_r[i])   # <= 0
    return np.asarray(ineq)


# ---------------------------------------------------------------------------
# solver core shared by the three MATLAB entry points
# ---------------------------------------------------------------------------
def _solve(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params,
           propellers=False, use_constraints=True, use_cpp=True,
           max_iter=200, verbose=False):
    mpc_N = int(mpc_N)
    propeller_force_max = 100.0

    if propellers:
        x0 = np.zeros(3 * mpc_N)
        lb = np.concatenate((-Fr_max * np.ones(2 * mpc_N), -propeller_force_max * np.ones(mpc_N)))
        ub = np.concatenate((Fr_max * np.ones(2 * mpc_N), propeller_force_max * np.ones(mpc_N)))
    else:
        x0 = np.zeros(2 * mpc_N)
        lb = -Fr_max * np.ones(2 * mpc_N)
        ub = Fr_max * np.ones(2 * mpc_N)
    bounds = Bounds(lb, ub)

    cons = []
    if use_constraints:
        # NOTE: SLSQP inequality constraints follow the convention g(x) >= 0.
        # The MATLAB constraints_mpc returns (Fr0 + delta) which must be <= 0
        # (unilateral rope forces), so we negate.  We use the classic dict-style
        # constraint interface rather than NonlinearConstraint: the latter makes
        # SLSQP stall at x0 for this problem (search direction collapses to 0).
        def con_fun(x):
            return -constraints_mpc(x, Fr_max, Fr_l0, Fr_r0, mpc_N)
        cons = [{"type": "ineq", "fun": con_fun}]

    res = minimize(
        cost_mpc, x0,
        args=(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, mpc_N, params, propellers, use_cpp),
        method="SLSQP", bounds=bounds, constraints=cons,
        options={"maxiter": max_iter, "ftol": 1e-6, "disp": verbose},
    )

    exitflag = 1 if res.success else (0 if res.status == 9 else -2)
    return res.x, exitflag, res.fun


# ---------------------------------------------------------------------------
# public MATLAB-equivalent entry points
# ---------------------------------------------------------------------------
def optimize_cpp_mpc(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params,
                     use_cpp=True, verbose=False):
    """optimize_cpp_mpc.m — MPC with unilateral force constraints."""
    return _solve(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params,
                  propellers=False, use_constraints=True, use_cpp=use_cpp, verbose=verbose)


def optimize_cpp_mpc_no_constraints(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max,
                                    mpc_N, params, use_cpp=True, verbose=False):
    """optimize_cpp_mpc_no_constraints.m — MPC with bounds only."""
    return _solve(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params,
                  propellers=False, use_constraints=False, use_cpp=use_cpp, verbose=verbose)


def optimize_cpp_mpc_propellers(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max,
                                mpc_N, params, use_cpp=True, verbose=False):
    """optimize_cpp_mpc_propellers.m — MPC with propeller forces + constraints."""
    return _solve(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max, mpc_N, params,
                  propellers=True, use_constraints=True, use_cpp=use_cpp, verbose=verbose)
