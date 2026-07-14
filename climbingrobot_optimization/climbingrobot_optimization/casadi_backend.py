"""
CasADi / IPOPT backend for the ALPINE jump + MPC problems.

This is a MATLAB-free, gradient-exact alternative to the SciPy SLSQP path in
``jump_optimizer.py`` / ``mpc_controller.py``.  The two-rope dynamics are
re-implemented with CasADi ``SX`` symbolics (identical formulas to
``dynamics.py``) and integrated with a symbolic single-shooting RK4/Euler
rollout, so IPOPT gets analytic derivatives.

Recommended by the porting map for the offline optimal control (CasADi +
IPOPT) and usable online for the MPC as well.
"""

import numpy as np

try:
    import casadi as ca
    _HAVE_CASADI = True
except Exception:  # pragma: no cover
    ca = None
    _HAVE_CASADI = False


def _require_casadi():
    if not _HAVE_CASADI:
        raise ImportError("casadi is not installed; `pip install casadi`")


# ---------------------------------------------------------------------------
# symbolic kinematics / dynamics (mirror kinematics.py + dynamics.py)
# ---------------------------------------------------------------------------
def _ca_forward_kin(params, psi, l1, l2):
    b = params.b
    root = ca.sqrt(1.0 - (b ** 2 + l1 ** 2 - l2 ** 2) ** 2 / (4.0 * b ** 2 * l1 ** 2))
    px = l1 * ca.sin(psi) * root
    py = (b ** 2 + l1 ** 2 - l2 ** 2) / (2.0 * b)
    pz = -l1 * ca.cos(psi) * root
    return px, py, pz


def _ca_dynamics(x, Fr_l, Fr_r, Fleg, t, params, extra_force=0.0):
    psi, l1, l2 = x[0], x[1], x[2]
    psid, l1d, l2d = x[3], x[4], x[5]
    b = params.b

    px, py, pz = _ca_forward_kin(params, psi, l1, l2)
    px_l1 = px / l1
    n_pz_l1 = -pz / l1
    px_l1_sinpsi = px / l1 / ca.sin(psi)
    py2b = py * 2.0 * b

    # Build A_dyn / b_dyn with vertcat/horzcat so the expression type
    # (SX or MX) follows the inputs (Opti variables are MX).
    A_row0 = ca.horzcat(
        l1 * n_pz_l1,
        px_l1 - (l1 * ca.sin(psi) * (py2b / (b ** 2 * l1) - py2b ** 2 / (2 * b ** 2 * l1 ** 3))) / (2 * px_l1_sinpsi),
        (l2 * py2b * ca.sin(psi)) / (2 * b ** 2 * l1 * px_l1_sinpsi))
    A_row1 = ca.horzcat(l1 * 0.0, l1 / b, -l2 / b)
    A_row2 = ca.horzcat(
        l1 * px_l1,
        (l1 * ca.cos(psi) * (py2b / (b ** 2 * l1) - py2b ** 2 / (2 * b ** 2 * l1 ** 3))) / (2 * px_l1_sinpsi) - n_pz_l1,
        -(l2 * py2b * ca.cos(psi)) / (2 * b ** 2 * l1 * px_l1_sinpsi))
    A_dyn = ca.vertcat(A_row0, A_row1, A_row2)

    e = (l1d * b ** 2 - l1d * l1 ** 2 + 2 * l2d * l1 * l2 - l1d * l2 ** 2)
    poly = (4 * l1 ** 4 * l1d ** 2 - 8 * l1 ** 3 * l2 * l1d * l2d + 4 * l1 ** 2 * l2 ** 2 * l2d ** 2
            - 6 * l1 ** 2 * l1d ** 2 * py2b - 2 * l1 ** 2 * l2d ** 2 * py2b
            + 8 * l1 * l2 * l1d * l2d * py2b + 3 * l1d ** 2 * py2b ** 2)

    b0 = (2 * l1d * n_pz_l1 * psid - l1 * psid ** 2 * px_l1
          - (ca.sin(psi) * poly) / (4 * b ** 2 * l1 ** 3 * px_l1_sinpsi)
          - (py2b ** 2 * ca.sin(psi) * e ** 2) / (16 * b ** 4 * l1 ** 5 * px_l1_sinpsi ** 3)
          + (psid * py2b * ca.cos(psi) * e) / (2 * b ** 2 * l1 ** 2 * px_l1_sinpsi)
          + (l1d * py2b * ca.sin(psi) * e) / (2 * b ** 2 * l1 ** 3 * px_l1_sinpsi))
    b1 = (l1d ** 2 - l2d ** 2) / b
    b2 = (l1 * n_pz_l1 * psid ** 2 + 2 * l1d * psid * px_l1
          + (ca.cos(psi) * poly) / (4 * b ** 2 * l1 ** 3 * px_l1_sinpsi)
          + (py2b ** 2 * ca.cos(psi) * e ** 2) / (16 * b ** 4 * l1 ** 5 * px_l1_sinpsi ** 3)
          - (l1d * py2b * ca.cos(psi) * e) / (2 * b ** 2 * l1 ** 3 * px_l1_sinpsi)
          + (psid * py2b * ca.sin(psi) * e) / (2 * b ** 2 * l1 ** 2 * px_l1_sinpsi))
    b_dyn = ca.vertcat(b0, b1, b2)

    px_, py_, pz_ = px, py, pz
    d1 = ca.vertcat(px_ - params.p_a1[0], py_ - params.p_a1[1], pz_ - params.p_a1[2])
    d2 = ca.vertcat(px_ - params.p_a2[0], py_ - params.p_a2[1], pz_ - params.p_a2[2])
    J = ca.horzcat(d1 / ca.norm_2(d1), d2 / ca.norm_2(d2))

    grav = ca.vertcat(0.0, 0.0, -params.g * params.m)
    Ftot = grav + ca.mtimes(J, ca.vertcat(Fr_l, Fr_r))
    # leg impulse active while t <= T_th (t is a numeric grid time here)
    fleg_active = 1.0 if t <= params.T_th else 0.0
    fleg_vec = ca.vertcat(*[float(v) for v in np.asarray(Fleg, float).reshape(3)])
    Ftot = Ftot + fleg_active * fleg_vec

    qdd = ca.solve(A_dyn, Ftot / params.m - b_dyn)
    return ca.vertcat(psid, l1d, l2d, qdd[0], qdd[1], qdd[2])


def _ca_rollout(x0, t0, dt_dyn, N_dyn, Fr_l, Fr_r, Fleg, params, int_steps, extra_forces=None):
    method = params.int_method
    x = x0
    t = t0
    states = [x]
    if int_steps and int_steps > 0:
        dt_step = dt_dyn / float(int_steps - 1)
        for i in range(N_dyn - 1):
            xi = states[i]
            ti = t0 + i * dt_dyn
            ef = 0.0 if extra_forces is None else extra_forces[i]
            for _ in range(int_steps - 1):
                xi = _rk_step(xi, ti, dt_step, Fr_l[i], Fr_r[i], Fleg, params, method, ef)
                ti = ti + dt_step
            states.append(xi)
    else:
        for i in range(N_dyn - 1):
            ti = t0 + i * dt_dyn
            ef = 0.0 if extra_forces is None else extra_forces[i]
            states.append(_rk_step(states[i], ti, dt_dyn, Fr_l[i], Fr_r[i], Fleg, params, method, ef))
    return ca.horzcat(*states)


def _rk_step(x, t, h, Fr_l, Fr_r, Fleg, params, method, extra_force):
    if method == "eul":
        return x + h * _ca_dynamics(x, Fr_l, Fr_r, Fleg, t, params, extra_force)
    k1 = _ca_dynamics(x, Fr_l, Fr_r, Fleg, t, params, extra_force)
    k2 = _ca_dynamics(x + 0.5 * h * k1, Fr_l, Fr_r, Fleg, t + 0.5 * h, params, extra_force)
    k3 = _ca_dynamics(x + 0.5 * h * k2, Fr_l, Fr_r, Fleg, t + 0.5 * h, params, extra_force)
    k4 = _ca_dynamics(x + h * k3, Fr_l, Fr_r, Fleg, t + h, params, extra_force)
    return x + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


# ---------------------------------------------------------------------------
# MPC via CasADi / IPOPT
# ---------------------------------------------------------------------------
def optimize_cpp_mpc_casadi(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, Fr_max,
                            mpc_N, params, propellers=False, use_constraints=True):
    """CasADi/IPOPT MPC solve. Returns (x, exitflag, final_cost)."""
    _require_casadi()
    mpc_N = int(mpc_N)
    ref_com = np.asarray(ref_com, float)[:, :mpc_N]
    Fr_l0 = np.asarray(Fr_l0, float)[:mpc_N]
    Fr_r0 = np.asarray(Fr_r0, float)[:mpc_N]
    x0 = np.asarray(actual_state, float).reshape(6)

    opti = ca.Opti()
    dFl = opti.variable(mpc_N)
    dFr = opti.variable(mpc_N)
    prop = opti.variable(mpc_N) if propellers else None

    Fl = ca.DM(Fr_l0) + dFl
    Fr = ca.DM(Fr_r0) + dFr
    extra = [prop[i] for i in range(mpc_N)] if propellers else None

    states = _ca_rollout(ca.DM(x0), actual_t, params.mpc_dt, mpc_N,
                         [Fl[i] for i in range(mpc_N)], [Fr[i] for i in range(mpc_N)],
                         ca.DM([0.0, 0.0, 0.0]), params, params.int_steps, extra)

    # cartesian positions
    tracking = 0
    for i in range(mpc_N):
        px, py, pz = _ca_forward_kin(params, states[0, i], states[1, i], states[2, i])
        p_i = ca.vertcat(px, py, pz)
        r_i = ca.DM(ref_com[:, i])
        tracking = tracking + ca.sumsqr(r_i - p_i)

    smooth = ca.sumsqr(ca.diff(dFl)) + ca.sumsqr(ca.diff(dFr))
    if propellers:
        smooth = smooth + ca.sumsqr(ca.diff(prop))
    opti.minimize(params.w1 * tracking + params.w2 * smooth)

    # bounds
    opti.subject_to(opti.bounded(-Fr_max, dFl, Fr_max))
    opti.subject_to(opti.bounded(-Fr_max, dFr, Fr_max))
    if propellers:
        opti.subject_to(opti.bounded(-100.0, prop, 100.0))
    if use_constraints:
        opti.subject_to(Fl <= 0)
        opti.subject_to(Fr <= 0)

    opti.set_initial(dFl, 0)
    opti.set_initial(dFr, 0)
    if propellers:
        opti.set_initial(prop, 0)

    opti.solver("ipopt", {"print_time": False, "ipopt": {"print_level": 0, "max_iter": 200}})
    try:
        sol = opti.solve()
        exitflag = 1
        xl = np.asarray(sol.value(dFl)).reshape(-1)
        xr = np.asarray(sol.value(dFr)).reshape(-1)
        cost_val = float(sol.value(opti.f))
        if propellers:
            xp = np.asarray(sol.value(prop)).reshape(-1)
            return np.concatenate((xl, xr, xp)), exitflag, cost_val
        return np.concatenate((xl, xr)), exitflag, cost_val
    except RuntimeError:
        xl = np.asarray(opti.debug.value(dFl)).reshape(-1)
        xr = np.asarray(opti.debug.value(dFr)).reshape(-1)
        if propellers:
            xp = np.asarray(opti.debug.value(prop)).reshape(-1)
            return np.concatenate((xl, xr, xp)), -2, float("nan")
        return np.concatenate((xl, xr)), -2, float("nan")


def casadi_available():
    return _HAVE_CASADI
