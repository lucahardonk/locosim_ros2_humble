"""
Rigid-body dynamics + numerical integration of the two-rope ALPINE model.

Faithful NumPy translation of:
    matlab/optimal_control/dynamics.m
    matlab/optimal_control/evalImpulse.m
    matlab/optimal_control/integrate_dynamics.m
    matlab/optimal_control/computeRollout.m

The ``computeRollout`` function is the numerical hot-spot (it is evaluated
by the optimiser thousands of times).  A drop-in C++/pybind11 accelerated
implementation is provided in ../cpp_kernel; if it has been built and is
importable it is used automatically unless ``use_cpp=False`` is passed.
"""

import numpy as np

from .kinematics import forward_kin, compute_jacobian

# ---------------------------------------------------------------------------
# optional C++ acceleration
# ---------------------------------------------------------------------------
try:
    import climbingrobot_kernel as _kernel  # built from cpp_kernel/
    _HAVE_KERNEL = True
except Exception:  # pragma: no cover - kernel is optional
    _kernel = None
    _HAVE_KERNEL = False


def eval_impulse(t, Fleg, params):
    """Leg thrust force active only while t <= T_th. evalImpulse.m."""
    Fleg = np.asarray(Fleg, dtype=float).reshape(3)
    if t <= params.T_th:
        return Fleg.copy()
    return np.zeros(3)


def dynamics(t, x, Fr_l, Fr_r, Fleg, params, extra_force=None):
    """State derivative dxdt = f(x, u). Faithful port of dynamics.m.

    x = [psi, l1, l2, psid, l1d, l2d]
    Fr_l, Fr_r : scalar rope retraction forces (<= 0)
    Fleg       : 3-vector leg impulse force (applied while t <= T_th)
    extra_force: optional scalar force perpendicular to the rope plane
    """
    x = np.asarray(x, dtype=float).reshape(6)
    psi, l1, l2, psid, l1d, l2d = x

    b = params.b
    px, py, pz = forward_kin(params, psi, l1, l2)

    px_l1 = px / l1
    n_pz_l1 = -pz / l1
    px_l1_sinpsi = px / l1 / np.sin(psi)
    py2b = py * 2.0 * b

    # mass equation and rope constraints (A_dyn * qdd = Ftot/m - b_dyn)
    A_dyn = np.array([
        [l1 * n_pz_l1,
         px_l1 - (l1 * np.sin(psi) * (py2b / (b ** 2 * l1) - py2b ** 2 / (2 * b ** 2 * l1 ** 3))) / (2 * px_l1_sinpsi),
         (l2 * py2b * np.sin(psi)) / (2 * b ** 2 * l1 * px_l1_sinpsi)],
        [0.0,
         l1 / b,
         -l2 / b],
        [l1 * px_l1,
         (l1 * np.cos(psi) * (py2b / (b ** 2 * l1) - py2b ** 2 / (2 * b ** 2 * l1 ** 3))) / (2 * px_l1_sinpsi) - n_pz_l1,
         -(l2 * py2b * np.cos(psi)) / (2 * b ** 2 * l1 * px_l1_sinpsi)],
    ])

    e = (l1d * b ** 2 - l1d * l1 ** 2 + 2 * l2d * l1 * l2 - l1d * l2 ** 2)
    poly = (4 * l1 ** 4 * l1d ** 2 - 8 * l1 ** 3 * l2 * l1d * l2d + 4 * l1 ** 2 * l2 ** 2 * l2d ** 2
            - 6 * l1 ** 2 * l1d ** 2 * py2b - 2 * l1 ** 2 * l2d ** 2 * py2b
            + 8 * l1 * l2 * l1d * l2d * py2b + 3 * l1d ** 2 * py2b ** 2)

    b_dyn = np.array([
        (2 * l1d * n_pz_l1 * psid - l1 * psid ** 2 * px_l1
         - (np.sin(psi) * poly) / (4 * b ** 2 * l1 ** 3 * px_l1_sinpsi)
         - (py2b ** 2 * np.sin(psi) * e ** 2) / (16 * b ** 4 * l1 ** 5 * px_l1_sinpsi ** 3)
         + (psid * py2b * np.cos(psi) * e) / (2 * b ** 2 * l1 ** 2 * px_l1_sinpsi)
         + (l1d * py2b * np.sin(psi) * e) / (2 * b ** 2 * l1 ** 3 * px_l1_sinpsi)),
        (l1d ** 2 - l2d ** 2) / b,
        (l1 * n_pz_l1 * psid ** 2 + 2 * l1d * psid * px_l1
         + (np.cos(psi) * poly) / (4 * b ** 2 * l1 ** 3 * px_l1_sinpsi)
         + (py2b ** 2 * np.cos(psi) * e ** 2) / (16 * b ** 4 * l1 ** 5 * px_l1_sinpsi ** 3)
         - (l1d * py2b * np.cos(psi) * e) / (2 * b ** 2 * l1 ** 3 * px_l1_sinpsi)
         + (psid * py2b * np.sin(psi) * e) / (2 * b ** 2 * l1 ** 2 * px_l1_sinpsi)),
    ])

    p = np.array([px, py, pz])
    J = compute_jacobian(p, params)

    if extra_force is None or extra_force == 0.0:
        extra_force_val = 0.0
        n_bar = np.array([1.0, 0.0, 0.0])
    else:
        extra_force_val = float(extra_force)
        n_par = (params.p_a1 - params.p_a2) / np.linalg.norm(params.p_a1 - params.p_a2)
        rope2_axis = (p - params.p_a2) / l2
        cross_val = np.cross(n_par, rope2_axis)
        n_bar = cross_val / np.linalg.norm(cross_val)

    Ftot = params.m * np.array([0.0, 0.0, -params.g]) + J @ np.array([Fr_l, Fr_r]) + extra_force_val * n_bar

    Fleg = np.asarray(Fleg, dtype=float).reshape(3)
    if np.linalg.norm(Fleg) > 0.0:
        Ftot = Ftot + eval_impulse(t, Fleg, params)

    y = np.linalg.solve(A_dyn, (Ftot / params.m - b_dyn))
    return np.array([psid, l1d, l2d, y[0], y[1], y[2]])


def integrate_dynamics(x0, t0, dt, n_steps, Fr_l, Fr_r, Fleg, method, params, extra_forces=None):
    """Integrate the dynamics over ``n_steps`` knots. integrate_dynamics.m.

    Returns (x_final, t_final, x_vec (6 x n_steps), t_vec (n_steps,)).
    ``Fr_l`` and ``Fr_r`` are indexable per step; ``extra_forces`` optional.
    """
    x0 = np.asarray(x0, dtype=float).reshape(6)
    Fr_l = np.atleast_1d(np.asarray(Fr_l, dtype=float))
    Fr_r = np.atleast_1d(np.asarray(Fr_r, dtype=float))
    if extra_forces is None:
        extra_forces = np.zeros(n_steps)
    else:
        extra_forces = np.atleast_1d(np.asarray(extra_forces, dtype=float))

    t_ = t0
    x_ = x0.copy()
    x_vec = np.zeros((6, n_steps))
    t_vec = np.zeros(n_steps)
    x_vec[:, 0] = x_
    t_vec[0] = t_

    if method == "eul":
        for i in range(n_steps - 1):
            x_ = x_ + dt * dynamics(t_, x_, Fr_l[i], Fr_r[i], Fleg, params, extra_forces[i])
            t_ = t_ + dt
            x_vec[:, i + 1] = x_
            t_vec[i + 1] = t_
    elif method == "rk4":
        h = dt
        for i in range(n_steps - 1):
            k1 = dynamics(t_, x_, Fr_l[i], Fr_r[i], Fleg, params, extra_forces[i])
            k2 = dynamics(t_ + 0.5 * h, x_ + 0.5 * h * k1, Fr_l[i], Fr_r[i], Fleg, params, extra_forces[i])
            k3 = dynamics(t_ + 0.5 * h, x_ + 0.5 * h * k2, Fr_l[i], Fr_r[i], Fleg, params, extra_forces[i])
            k4 = dynamics(t_ + h, x_ + h * k3, Fr_l[i], Fr_r[i], Fleg, params, extra_forces[i])
            x_ = x_ + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4) * h
            t_ = t_ + h
            x_vec[:, i + 1] = x_
            t_vec[i + 1] = t_
    else:
        raise ValueError("Unknown integration method: %s" % method)

    return x_, t_, x_vec, t_vec


def compute_rollout(x0, t0, dt_dyn, N_dyn, Fr_l, Fr_r, Fleg, int_method, int_steps,
                    params, extra_forces=None, use_cpp=True):
    """Single-shooting rollout of the trajectory. computeRollout.m.

    Returns (states_rough (6 x N_dyn), t_rough (N_dyn,)).
    Uses the C++ kernel when available (unless ``use_cpp=False``).
    """
    x0 = np.asarray(x0, dtype=float).reshape(6)
    Fr_l = np.atleast_1d(np.asarray(Fr_l, dtype=float))
    Fr_r = np.atleast_1d(np.asarray(Fr_r, dtype=float))
    Fleg = np.asarray(Fleg, dtype=float).reshape(3)
    if extra_forces is None:
        extra_forces = np.zeros(N_dyn)
    else:
        extra_forces = np.atleast_1d(np.asarray(extra_forces, dtype=float))

    if use_cpp and _HAVE_KERNEL:
        states, t = _kernel.compute_rollout(
            x0, t0, dt_dyn, int(N_dyn), Fr_l, Fr_r, Fleg, int_method, int(int_steps),
            float(params.m), float(params.g), float(params.b),
            np.asarray(params.p_a1, float), np.asarray(params.p_a2, float),
            float(params.T_th), extra_forces)
        return np.asarray(states), np.asarray(t)

    states_rough = np.zeros((6, N_dyn))
    t_rough = np.zeros(N_dyn)

    if int_steps == 0:
        _, _, states_rough, t_rough = integrate_dynamics(
            x0, 0.0, dt_dyn, N_dyn, Fr_l, Fr_r, Fleg, int_method, params, extra_forces)
    else:
        dt_step = dt_dyn / float(int_steps - 1)
        for i in range(N_dyn):
            if i >= 1:
                xf, tf, _, _ = integrate_dynamics(
                    states_rough[:, i - 1], t_rough[i - 1], dt_step, int_steps,
                    Fr_l[i - 1] * np.ones(int_steps), Fr_r[i - 1] * np.ones(int_steps),
                    Fleg, int_method, params, extra_forces[i - 1] * np.ones(int_steps))
                states_rough[:, i] = xf
                t_rough[i] = tf
            else:
                states_rough[:, i] = x0
                t_rough[i] = t0

    return states_rough, t_rough


def kernel_available():
    """True if the compiled C++ rollout kernel is importable."""
    return _HAVE_KERNEL
