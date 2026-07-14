"""
Offline jump trajectory optimal control for the ALPINE climbing robot.

Faithful translation of:
    matlab/optimal_control/cost.m
    matlab/optimal_control/constraints.m
    matlab/optimal_control/optimize_cpp.m          (fmincon 'sqp' -> SciPy SLSQP)
    matlab/optimal_control/eval_solution.m
    matlab/optimal_control/computeJumpEnergyConsumption.m

Decision vector (same layout as MATLAB):
    z = [Fleg_x, Fleg_y, Fleg_z, Tf, Fr_l(0..N-1), Fr_r(0..N-1)]

The original code used MATLAB ``fmincon`` with the SQP algorithm and a
single-shooting rollout.  This is reproduced with ``scipy.optimize.minimize``
(method='SLSQP'), which is the closest freely-available SQP solver and
handles the nonlinear inequality constraints + bounds directly.  A CasADi /
IPOPT backend is provided in ``casadi_backend.py`` for a MATLAB-free,
gradient-exact alternative.
"""

import numpy as np
from scipy.optimize import minimize, Bounds

from .kinematics import compute_state_from_cartesian, compute_position_velocity
from .dynamics import compute_rollout


# ---------------------------------------------------------------------------
# helpers to slice the decision vector
# ---------------------------------------------------------------------------
def _unpack(z, params):
    Fleg = np.array([z[0], z[1], z[2]])
    Tf = z[3]
    np_ = params.num_params
    N = params.N_dyn
    Fr_l = np.asarray(z[np_:np_ + N])
    Fr_r = np.asarray(z[np_ + N:np_ + 2 * N])
    return Fleg, Tf, Fr_l, Fr_r


# ---------------------------------------------------------------------------
# cost.m
# ---------------------------------------------------------------------------
def cost(z, p0, pf, params, use_cpp=True):
    Fleg, Tf, Fr_l, Fr_r = _unpack(z, params)
    p0 = np.asarray(p0, float).reshape(3)
    dt_dyn = Tf / (params.N_dyn - 1)

    state0 = compute_state_from_cartesian(params, p0)
    states, _ = compute_rollout(state0, 0.0, dt_dyn, params.N_dyn, Fr_l, Fr_r, Fleg,
                                params.int_method, params.int_steps, params, use_cpp=use_cpp)
    l1d = states[4, :]
    l2d = states[5, :]

    # hoist work (rough grid), motor not regenerating
    hoist_work = np.sum(np.abs(Fr_l * l1d) * dt_dyn) + np.sum(np.abs(Fr_r * l2d) * dt_dyn)
    # smoothness (matches active line in cost.m: sum(diff(Fr_r)) + sum(diff(Fr_l)))
    smooth = np.sum(np.diff(Fr_r)) + np.sum(np.diff(Fr_l))

    return params.w2 * hoist_work + params.w1 * smooth


# ---------------------------------------------------------------------------
# constraints.m  -> returns ineq array (MATLAB convention: ineq <= 0)
# ---------------------------------------------------------------------------
def constraints(z, p0, pf, Fleg_max, Fr_max, mu, params, use_cpp=True, return_solution=False):
    Fleg, Tf, Fr_l, Fr_r = _unpack(z, params)
    p0 = np.asarray(p0, float).reshape(3)
    pf = np.asarray(pf, float).reshape(3)
    N = params.N_dyn

    dt_dyn = Tf / (N - 1)
    state0 = compute_state_from_cartesian(params, p0)
    states, t = compute_rollout(state0, 0.0, dt_dyn, N, Fr_l, Fr_r, Fleg,
                                params.int_method, params.int_steps, params, use_cpp=use_cpp)
    psi, l1, l2 = states[0, :], states[1, :], states[2, :]
    p = compute_position_velocity(params, psi, l1, l2)
    p_f = p[:, -1]

    ineq = []

    # 1 - wall / obstacle constraints (N_dyn of them)
    if params.obstacle_avoidance:
        center = params.obstacle_location
        radii = params.obstacle_size
        a_y = radii[0] ** 2 / radii[1] ** 2
        a_z = radii[0] ** 2 / radii[2] ** 2
        radius = radii[0]
        for i in range(N):
            arg = radius ** 2 - a_z * (p[2, i] - center[2]) ** 2 - a_y * (p[1, i] - center[1]) ** 2
            if arg > 0:
                ineq.append(-p[0, i] + center[0] + np.sqrt(arg) + params.jump_clearance)
            else:
                ineq.append(-p[0, i])
    else:
        for i in range(N):
            ineq.append(-p[0, i])

    # 2 - retraction force constraints: number_of_constr = 0 (handled by bounds)

    # 3 - Fleg impulse constraints
    contact_normal = params.contact_normal
    contact_tang_y = np.cross(np.cross(contact_normal, np.array([0., 1., 0.])), contact_normal)
    contact_tang_z = np.cross(np.cross(contact_normal, np.array([0., 0., 1.])), contact_normal)
    Fun = contact_normal @ Fleg
    Futy = contact_tang_y @ Fleg
    Futz = contact_tang_z @ Fleg
    Fut_norm = np.sqrt(Futy ** 2 + Futz ** 2)
    Fun_min = 0.0

    ineq.append(-Fun + Fun_min)                 # unilateral (Fun >= Fun_min)
    ineq.append(np.linalg.norm(Fleg) - Fleg_max)  # max leg force
    if params.FRICTION_CONE:
        ineq.append(Fut_norm - mu * Fun)        # friction cone

    # 4 - final point constraint (initial_final_constraints == 1)
    fixed_slack = 0.02
    ineq.append(np.linalg.norm(p_f - pf) - fixed_slack)

    # 5 - via point / jump clearance (only when not obstacle avoidance)
    if not params.obstacle_avoidance:
        mid = N // 2 - 1  # MATLAB p(1, N_dyn/2) -> 0-based index
        ineq.append(-p[0, mid] + params.jump_clearance)

    ineq = np.asarray(ineq)

    if return_solution:
        solution_constr = {
            "p": p, "psi": psi, "l1": l1, "l2": l2,
            "psid": states[3, :], "l1d": states[4, :], "l2d": states[5, :],
            "time": t, "final_error_discrete": np.linalg.norm(p[:, -1] - pf),
        }
        return ineq, solution_constr
    return ineq


# ---------------------------------------------------------------------------
# optimize_cpp.m
# ---------------------------------------------------------------------------
def optimize_cpp(p0, pf, Fleg_max, Fr_max, mu, params, use_cpp=True,
                 max_iter=400, verbose=True):
    """Solve the offline jump optimal control problem.

    Returns a ``solution`` dict analogous to the MATLAB struct.
    """
    p0 = np.asarray(p0, float).reshape(3)
    pf = np.asarray(pf, float).reshape(3)
    dt = 0.001
    constr_tolerance = 1e-3
    N = params.N_dyn

    x0_state = compute_state_from_cartesian(params, p0)
    T_pend = 2 * np.pi * np.sqrt(x0_state[1] / params.g) / 4.0  # quarter pendulum period

    Fr_l0 = np.zeros(N)
    Fr_r0 = np.zeros(N)
    z0 = np.concatenate(([Fleg_max, Fleg_max, Fleg_max, T_pend], Fr_l0, Fr_r0))

    lb = np.concatenate(([-Fleg_max, -Fleg_max, -Fleg_max, 0.01], -Fr_max * np.ones(N), -Fr_max * np.ones(N)))
    ub = np.concatenate(([Fleg_max, Fleg_max, Fleg_max, np.inf], np.zeros(N), np.zeros(N)))
    bounds = Bounds(lb, ub)

    # SciPy SLSQP expects inequality constraints g(x) >= 0.
    # MATLAB constraints return ineq <= 0, so we negate.
    def con_fun(z):
        return -constraints(z, p0, pf, Fleg_max, Fr_max, mu, params, use_cpp=use_cpp)

    # Classic dict-style inequality constraints (g(x) >= 0) for SLSQP.  This is
    # more robust than NonlinearConstraint for this single-shooting problem.
    cons = [{"type": "ineq", "fun": con_fun}]

    res = minimize(
        cost, z0, args=(p0, pf, params, use_cpp),
        method="SLSQP", bounds=bounds, constraints=cons,
        options={"maxiter": max_iter, "ftol": constr_tolerance, "disp": verbose},
    )

    z = res.x
    solution = eval_solution(z, dt, p0, pf, params, use_cpp=use_cpp)
    solution["T_th"] = params.T_th
    solution["cost"] = res.fun
    # SciPy: res.success -> map to MATLAB-like EXITFLAG (1 solved, -2 infeasible, 0 maxiter)
    if res.success:
        solution["problem_solved"] = 1
    elif "iteration" in res.message.lower() or res.status == 9:
        solution["problem_solved"] = 0
    else:
        solution["problem_solved"] = -2
    solution["optim_output"] = {"message": res.message, "nit": getattr(res, "nit", None),
                                "status": res.status}

    ineq, solution_constr = constraints(z, p0, pf, Fleg_max, Fr_max, mu, params,
                                        use_cpp=use_cpp, return_solution=True)
    solution["c"] = ineq
    solution["solution_constr"] = solution_constr
    solution["constr_tolerance"] = constr_tolerance
    return solution


# ---------------------------------------------------------------------------
# eval_solution.m
# ---------------------------------------------------------------------------
def eval_solution(z, dt, p0, pf, params, use_cpp=True):
    Fleg, Tf, Fr_l, Fr_r = _unpack(z, params)
    p0 = np.asarray(p0, float).reshape(3)
    pf = np.asarray(pf, float).reshape(3)
    N = params.N_dyn

    # resample inputs onto the fine grid (matches eval_solution.m loop)
    n_samples = int(np.floor(Tf / dt))
    Fr_l_fine = np.zeros(n_samples)
    Fr_r_fine = np.zeros(n_samples)
    rough_count = 0
    t_ = 0.0
    for i in range(n_samples):
        t_ += dt
        if t_ >= (n_samples * dt / (N - 1)):
            rough_count += 1
            t_ = 0.0
        rc = min(rough_count, N - 1)
        Fr_l_fine[i] = Fr_l[rc]
        Fr_r_fine[i] = Fr_r[rc]

    state0 = compute_state_from_cartesian(params, p0)

    dt_dyn = Tf / (N - 1)
    states, t = compute_rollout(state0, 0.0, dt_dyn, N, Fr_l, Fr_r, Fleg,
                                params.int_method, params.int_steps, params, use_cpp=use_cpp)
    psi, l1, l2, psid, l1d, l2d = states
    p, pd = compute_position_velocity(params, psi, l1, l2, psid, l1d, l2d)

    states_fine, t_fine = compute_rollout(state0, 0.0, dt, n_samples, Fr_l_fine, Fr_r_fine, Fleg,
                                          params.int_method, 0, params, use_cpp=use_cpp)
    psi_f, l1_f, l2_f, psid_f, l1d_f, l2d_f = states_fine
    p_fine, pd_fine = compute_position_velocity(params, psi_f, l1_f, l2_f, psid_f, l1d_f, l2d_f)

    solution = {}
    d = np.diff(p, axis=1)
    solution["path_length"] = float(np.sum(np.sqrt(np.sum(d ** 2, axis=0))))
    solution["initial_error"] = float(np.linalg.norm(p[:, 0] - p0))
    solution["final_error_real"] = float(np.linalg.norm(p[:, -1] - pf))

    solution["Fleg"] = Fleg
    solution["Fr_l"] = Fr_l
    solution["Fr_r"] = Fr_r
    solution["p"] = p
    solution["pd"] = pd
    solution["psi"] = psi
    solution["l1"] = l1
    solution["l2"] = l2
    solution["psid"] = psid
    solution["l1d"] = l1d
    solution["l2d"] = l2d
    solution["time"] = t

    solution["Fr_l_fine"] = Fr_l_fine
    solution["Fr_r_fine"] = Fr_r_fine
    solution["p_fine"] = p_fine
    solution["pd_fine"] = pd_fine
    solution["psi_fine"] = psi_f
    solution["l1_fine"] = l1_f
    solution["l2_fine"] = l2_f
    solution["psid_fine"] = psid_f
    solution["l1d_fine"] = l1d_f
    solution["l2d_fine"] = l2d_f
    solution["time_fine"] = t_fine

    solution["Tf"] = Tf
    solution["achieved_target"] = p_fine[:, -1]

    m = params.m
    solution["Ekin0x"] = m / 2 * pd_fine[0, 0] ** 2
    solution["Ekin0y"] = m / 2 * pd_fine[1, 0] ** 2
    solution["Ekin0z"] = m / 2 * pd_fine[2, 0] ** 2
    solution["Ekin0"] = m / 2 * pd_fine[:, 0] @ pd_fine[:, 0]
    solution["Ekinfx"] = m / 2 * pd_fine[0, -1] ** 2
    solution["Ekinfy"] = m / 2 * pd_fine[1, -1] ** 2
    solution["Ekinfz"] = m / 2 * pd_fine[2, -1] ** 2
    solution["Ekinf"] = m / 2 * pd_fine[:, -1] @ pd_fine[:, -1]

    Ekin = m / 2 * np.sum(pd_fine ** 2, axis=0)
    solution["Ekin"] = Ekin
    solution["intEkin"] = float(np.sum(Ekin * dt))
    return solution


# ---------------------------------------------------------------------------
# computeJumpEnergyConsumption.m
# ---------------------------------------------------------------------------
def compute_jump_energy_consumption(solution, params):
    dt_dyn = solution["Tf"] / (params.N_dyn - 1)
    time = solution["time"]
    T_th = solution.get("T_th", params.T_th)
    idx = np.where(time <= T_th)[0]
    impulse_end_idx = idx[-1] if idx.size else 0
    impulse_work = solution["Ekin"][min(impulse_end_idx, len(solution["Ekin"]) - 1)]

    hoist_work = 0.0
    for i in range(len(time)):
        hoist_work += (abs(solution["Fr_l"][i] * solution["l1d"][i])
                       + abs(solution["Fr_r"][i] * solution["l2d"][i])) * dt_dyn

    tf = solution["time_fine"]
    dt = tf[1] - tf[0]
    hoist_work_fine = 0.0
    for i in range(len(tf)):
        hoist_work_fine += (abs(solution["Fr_l_fine"][i] * solution["l1d_fine"][i])
                            + abs(solution["Fr_r_fine"][i] * solution["l2d_fine"][i])) * dt

    return impulse_work, hoist_work, hoist_work_fine
