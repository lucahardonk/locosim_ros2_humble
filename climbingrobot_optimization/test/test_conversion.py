"""
Regression / conversion-correctness tests for climbingrobot_optimization.

Run with::

    cd climbingrobot_optimization
    PYTHONPATH=$PWD/climbingrobot_optimization pytest test -v

(The PYTHONPATH addition is only needed to expose the compiled C++ kernel
``climbingrobot_kernel`` when it is built in-tree; the tests transparently fall
back to the pure-NumPy implementation when the kernel is not importable.)
"""

import numpy as np
import pytest

from climbingrobot_optimization.params import Params
from climbingrobot_optimization import dynamics, jump_optimizer, mpc_controller


# --------------------------------------------------------------------------
# fixtures
# --------------------------------------------------------------------------
@pytest.fixture(scope="module")
def solved_jump():
    p = Params.normal_test()
    p0 = np.array([0.5, 2.5, -6.0])
    pf = np.array([0.5, 4.0, -4.0])
    sol = jump_optimizer.optimize_cpp(p0, pf, 300.0, 90.0, 0.8, p, verbose=False)
    return p, sol


# --------------------------------------------------------------------------
# C++ kernel vs NumPy
# --------------------------------------------------------------------------
def test_kernel_matches_numpy():
    """The C++ rollout kernel must match the NumPy rollout to ~machine eps."""
    if not dynamics.kernel_available():
        pytest.skip("C++ kernel not built; skipping parity test")

    p = Params.normal_test()
    p.mpc_dt = 0.04
    x0 = np.array([0.08, 6.5, 6.5, 0.0, 0.0, 0.0])
    N = 20
    Fr_l = -10.0 * np.ones(N)
    Fr_r = -20.0 * np.ones(N)
    Fleg = np.array([50.0, 10.0, 80.0])

    states_cpp, t_cpp = dynamics.compute_rollout(
        x0, 0.0, p.mpc_dt, N, Fr_l, Fr_r, Fleg, p.int_method, p.int_steps, p,
        use_cpp=True)
    states_np, t_np = dynamics.compute_rollout(
        x0, 0.0, p.mpc_dt, N, Fr_l, Fr_r, Fleg, p.int_method, p.int_steps, p,
        use_cpp=False)

    assert np.allclose(states_cpp, states_np, atol=1e-10)
    assert np.allclose(t_cpp, t_np, atol=1e-12)


# --------------------------------------------------------------------------
# jump optimiser
# --------------------------------------------------------------------------
def test_jump_converges(solved_jump):
    _, sol = solved_jump
    assert sol["problem_solved"] == 1
    # target is reached to within a few cm
    err = np.linalg.norm(sol["achieved_target"] - np.array([0.5, 4.0, -4.0]))
    assert err < 0.1, "landing error too large: %.3f m" % err
    assert sol["Tf"] > 0.0


def test_jump_forces_unilateral(solved_jump):
    """Rope forces must be retraction-only (<= 0) within tolerance."""
    _, sol = solved_jump
    assert np.all(sol["Fr_l"] <= 1e-6)
    assert np.all(sol["Fr_r"] <= 1e-6)


# --------------------------------------------------------------------------
# MPC controller
# --------------------------------------------------------------------------
def test_mpc_produces_correction(solved_jump):
    """A perturbed initial state must yield a non-trivial, cost-reducing
    correction (regression test for the SLSQP zero-correction bug)."""
    p, sol = solved_jump
    N = p.N_dyn
    p.mpc_dt = sol["Tf"] / (N - 1)
    mpc_N = int(0.4 * N)
    ref_com = sol["p"]
    Fr_l0, Fr_r0 = sol["Fr_l"], sol["Fr_r"]

    state = np.array([sol["psi"][0], sol["l1"][0] + 0.2, sol["l2"][0] - 0.2,
                      sol["psid"][0], sol["l1d"][0], sol["l2d"][0]])

    cost0 = mpc_controller.cost_mpc(np.zeros(2 * mpc_N), state, 0.0, ref_com,
                                    Fr_l0, Fr_r0, mpc_N, p)
    x, flag, fval = mpc_controller.optimize_cpp_mpc(
        state, 0.0, ref_com, Fr_l0, Fr_r0, 90.0, mpc_N, p)

    assert flag == 1
    assert np.max(np.abs(x)) > 1.0, "MPC returned a near-zero correction"
    assert fval < cost0, "MPC did not reduce the tracking cost"


def test_mpc_respects_unilateral_constraint(solved_jump):
    p, sol = solved_jump
    N = p.N_dyn
    p.mpc_dt = sol["Tf"] / (N - 1)
    mpc_N = int(0.4 * N)
    ref_com = sol["p"]
    Fr_l0, Fr_r0 = sol["Fr_l"], sol["Fr_r"]
    state = np.array([sol["psi"][0], sol["l1"][0] + 0.2, sol["l2"][0] - 0.2,
                      sol["psid"][0], sol["l1d"][0], sol["l2d"][0]])

    x, flag, _ = mpc_controller.optimize_cpp_mpc(
        state, 0.0, ref_com, Fr_l0, Fr_r0, 90.0, mpc_N, p)
    delta_Fr_l = x[:mpc_N]
    delta_Fr_r = x[mpc_N:2 * mpc_N]
    # Fr0 + delta must stay <= 0 (small numerical tolerance)
    assert np.all(Fr_l0[:mpc_N] + delta_Fr_l <= 1e-3)
    assert np.all(Fr_r0[:mpc_N] + delta_Fr_r <= 1e-3)
