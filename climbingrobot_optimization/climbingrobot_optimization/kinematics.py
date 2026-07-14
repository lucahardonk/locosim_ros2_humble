"""
Kinematics of the two-rope ALPINE climbing robot.

Faithful NumPy translation of:
    matlab/optimal_control/forwardKin.m
    matlab/optimal_control/computeJacobian.m
    matlab/optimal_control/computePositionVelocity.m
    matlab/optimal_control/computeStateFromCartesian.m

The math is kept identical to the MATLAB source so that results match the
original optimiser to numerical precision.
"""

import numpy as np


def forward_kin(params, psi, l1, l2, psid=None, l1d=None, l2d=None):
    """Forward kinematics for the 2-rope model.

    Returns (px, py, pz) if velocities are not supplied, otherwise
    (px, py, pz, pdx, pdy, pdz).  Mirrors forwardKin.m.
    """
    b = params.b
    sqrt_arg = 1.0 - (b ** 2 + l1 ** 2 - l2 ** 2) ** 2 / (4.0 * b ** 2 * l1 ** 2)
    root = np.sqrt(sqrt_arg)

    px = l1 * np.sin(psi) * root
    py = (b ** 2 + l1 ** 2 - l2 ** 2) / (2.0 * b)
    pz = -l1 * np.cos(psi) * root

    if psid is None:
        return px, py, pz

    px_l1 = px / l1
    n_pz_l1 = -pz / l1
    # px_l1_sinpsi = px / l1 / sin(psi) has a removable singularity at psi = 0.
    # Since px = l1*sin(psi)*root, this ratio equals `root` exactly for every psi
    # (including the psi = 0 limit), so use the analytic form to avoid 0/0 -> NaN.
    px_l1_sinpsi = root
    py2b = py * 2.0 * b

    pdx = (l1d * px_l1 + l1 * n_pz_l1 * psid
           + (py2b * np.sin(psi) * (l1d * b ** 2 - l1d * l1 ** 2 + 2 * l2d * l1 * l2 - l1d * l2 ** 2))
           / (4.0 * b ** 2 * l1 ** 2 * px_l1_sinpsi))
    pdy = (l1 * l1d - l2 * l2d) / b
    pdz = (l1 * psid * px_l1 - l1d * n_pz_l1
           - (py2b * np.cos(psi) * (l1d * b ** 2 - l1d * l1 ** 2 + 2 * l2d * l1 * l2 - l1d * l2 ** 2))
           / (4.0 * b ** 2 * l1 ** 2 * px_l1_sinpsi))
    return px, py, pz, pdx, pdy, pdz


def compute_jacobian(p, params):
    """3x2 Jacobian mapping rope forces -> Cartesian force. computeJacobian.m."""
    p = np.asarray(p, dtype=float).reshape(3)
    d1 = p - params.p_a1
    d2 = p - params.p_a2
    col1 = d1 / np.linalg.norm(d1)
    col2 = d2 / np.linalg.norm(d2)
    return np.column_stack((col1, col2))


def compute_position_velocity(params, psi, l1, l2, psid=None, l1d=None, l2d=None):
    """Vectorised forward kinematics over a trajectory. computePositionVelocity.m.

    ``psi, l1, l2`` (and optionally the velocities) are 1-D arrays of equal
    length ``n``.  Returns ``p`` (3 x n) and, if velocities supplied, ``pd``
    (3 x n); otherwise just ``p``.
    """
    psi = np.atleast_1d(np.asarray(psi, dtype=float))
    l1 = np.atleast_1d(np.asarray(l1, dtype=float))
    l2 = np.atleast_1d(np.asarray(l2, dtype=float))
    n = psi.shape[0]

    if psid is not None:
        psid = np.atleast_1d(np.asarray(psid, dtype=float))
        l1d = np.atleast_1d(np.asarray(l1d, dtype=float))
        l2d = np.atleast_1d(np.asarray(l2d, dtype=float))
        p = np.zeros((3, n))
        pd = np.zeros((3, n))
        for i in range(n):
            px, py, pz, pdx, pdy, pdz = forward_kin(
                params, psi[i], l1[i], l2[i], psid[i], l1d[i], l2d[i])
            p[:, i] = (px, py, pz)
            pd[:, i] = (pdx, pdy, pdz)
        return p, pd

    p = np.zeros((3, n))
    for i in range(n):
        px, py, pz = forward_kin(params, psi[i], l1[i], l2[i])
        p[:, i] = (px, py, pz)
    return p


def compute_state_from_cartesian(params, p, pd=None):
    """Inverse kinematics: cartesian (p, pd) -> state. computeStateFromCartesian.m."""
    p = np.asarray(p, dtype=float).reshape(3)

    psi = np.arctan2(p[0], -p[2])
    l1 = np.linalg.norm(p - params.p_a1)
    l2 = np.linalg.norm(p - params.p_a2)

    if pd is not None:
        pd = np.asarray(pd, dtype=float).reshape(3)
        n_par = (params.p_a1 - params.p_a2) / np.linalg.norm(params.p_a1 - params.p_a2)
        rope2_axis = (p - params.p_a2) / l2
        cross_val = np.cross(n_par, rope2_axis)
        n_bar = cross_val / np.linalg.norm(cross_val)

        psid = (n_bar @ pd) / np.linalg.norm(np.cross(n_par, p - params.p_a2))
        # project velocity along each rope axis (matches MATLAB comment / code)
        l1d = (p - params.p_a1) @ pd
        l2d = (p - params.p_a2) @ pd
    else:
        psid = 0.0
        l1d = 0.0
        l2d = 0.0

    return np.array([psi, l1, l2, psid, l1d, l2d])
