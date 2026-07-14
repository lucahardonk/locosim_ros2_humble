// dynamics_kernel.cpp — see dynamics_kernel.hpp for description.

#include "dynamics_kernel.hpp"

#include <cmath>
#include <stdexcept>

namespace climbingrobot {

namespace {

// Solve 3x3 linear system A x = rhs via Cramer's rule (fixed size, fast).
std::array<double, 3> solve3x3(const double A[3][3], const std::array<double, 3>& rhs) {
    const double det =
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    const double inv_det = 1.0 / det;

    // adjugate * rhs (column replacement / Cramer)
    double d0[3][3], d1[3][3], d2[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) { d0[i][j] = A[i][j]; d1[i][j] = A[i][j]; d2[i][j] = A[i][j]; }
    for (int i = 0; i < 3; ++i) { d0[i][0] = rhs[i]; d1[i][1] = rhs[i]; d2[i][2] = rhs[i]; }

    auto det3 = [](const double M[3][3]) {
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
               M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    };

    return { det3(d0) * inv_det, det3(d1) * inv_det, det3(d2) * inv_det };
}

double norm3(const std::array<double, 3>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

void forward_kin(const ModelParams& prm, double psi, double l1, double l2,
                 double& px, double& py, double& pz) {
    const double b = prm.b;
    const double root =
        std::sqrt(1.0 - std::pow(b * b + l1 * l1 - l2 * l2, 2) / (4.0 * b * b * l1 * l1));
    px = l1 * std::sin(psi) * root;
    py = (b * b + l1 * l1 - l2 * l2) / (2.0 * b);
    pz = -l1 * std::cos(psi) * root;
}

State dynamics(double t, const State& x, double Fr_l, double Fr_r,
               const std::array<double, 3>& Fleg, const ModelParams& prm,
               double extra_force) {
    const double psi = x[0], l1 = x[1], l2 = x[2];
    const double psid = x[3], l1d = x[4], l2d = x[5];
    const double b = prm.b;

    double px, py, pz;
    forward_kin(prm, psi, l1, l2, px, py, pz);

    const double px_l1 = px / l1;
    const double n_pz_l1 = -pz / l1;
    // px_l1_sinpsi = px / l1 / sin(psi) has a removable singularity at psi = 0.
    // Since px = l1*sin(psi)*root, this ratio equals `root` exactly for every psi
    // (including the psi = 0 limit), so use the analytic form to avoid 0/0 -> NaN.
    const double root =
        std::sqrt(1.0 - std::pow(b * b + l1 * l1 - l2 * l2, 2) / (4.0 * b * b * l1 * l1));
    const double px_l1_sinpsi = root;
    const double py2b = py * 2.0 * b;

    double A[3][3];
    A[0][0] = l1 * n_pz_l1;
    A[0][1] = px_l1 - (l1 * std::sin(psi) * (py2b / (b * b * l1) - py2b * py2b / (2 * b * b * l1 * l1 * l1))) / (2 * px_l1_sinpsi);
    A[0][2] = (l2 * py2b * std::sin(psi)) / (2 * b * b * l1 * px_l1_sinpsi);
    A[1][0] = 0.0;
    A[1][1] = l1 / b;
    A[1][2] = -l2 / b;
    A[2][0] = l1 * px_l1;
    A[2][1] = (l1 * std::cos(psi) * (py2b / (b * b * l1) - py2b * py2b / (2 * b * b * l1 * l1 * l1))) / (2 * px_l1_sinpsi) - n_pz_l1;
    A[2][2] = -(l2 * py2b * std::cos(psi)) / (2 * b * b * l1 * px_l1_sinpsi);

    const double e = (l1d * b * b - l1d * l1 * l1 + 2 * l2d * l1 * l2 - l1d * l2 * l2);
    const double poly = (4 * std::pow(l1, 4) * l1d * l1d - 8 * std::pow(l1, 3) * l2 * l1d * l2d
                         + 4 * l1 * l1 * l2 * l2 * l2d * l2d - 6 * l1 * l1 * l1d * l1d * py2b
                         - 2 * l1 * l1 * l2d * l2d * py2b + 8 * l1 * l2 * l1d * l2d * py2b
                         + 3 * l1d * l1d * py2b * py2b);

    std::array<double, 3> b_dyn;
    b_dyn[0] = (2 * l1d * n_pz_l1 * psid - l1 * psid * psid * px_l1
                - (std::sin(psi) * poly) / (4 * b * b * l1 * l1 * l1 * px_l1_sinpsi)
                - (py2b * py2b * std::sin(psi) * e * e) / (16 * std::pow(b, 4) * std::pow(l1, 5) * std::pow(px_l1_sinpsi, 3))
                + (psid * py2b * std::cos(psi) * e) / (2 * b * b * l1 * l1 * px_l1_sinpsi)
                + (l1d * py2b * std::sin(psi) * e) / (2 * b * b * l1 * l1 * l1 * px_l1_sinpsi));
    b_dyn[1] = (l1d * l1d - l2d * l2d) / b;
    b_dyn[2] = (l1 * n_pz_l1 * psid * psid + 2 * l1d * psid * px_l1
                + (std::cos(psi) * poly) / (4 * b * b * l1 * l1 * l1 * px_l1_sinpsi)
                + (py2b * py2b * std::cos(psi) * e * e) / (16 * std::pow(b, 4) * std::pow(l1, 5) * std::pow(px_l1_sinpsi, 3))
                - (l1d * py2b * std::cos(psi) * e) / (2 * b * b * l1 * l1 * l1 * px_l1_sinpsi)
                + (psid * py2b * std::sin(psi) * e) / (2 * b * b * l1 * l1 * px_l1_sinpsi));

    // Jacobian columns (rope unit vectors)
    const std::array<double, 3> p = {px, py, pz};
    std::array<double, 3> d1 = {p[0] - prm.p_a1[0], p[1] - prm.p_a1[1], p[2] - prm.p_a1[2]};
    std::array<double, 3> d2 = {p[0] - prm.p_a2[0], p[1] - prm.p_a2[1], p[2] - prm.p_a2[2]};
    const double n1 = norm3(d1), n2 = norm3(d2);

    std::array<double, 3> n_bar = {1.0, 0.0, 0.0};
    if (extra_force != 0.0) {
        std::array<double, 3> n_par = {prm.p_a1[0] - prm.p_a2[0], prm.p_a1[1] - prm.p_a2[1], prm.p_a1[2] - prm.p_a2[2]};
        const double npn = norm3(n_par);
        n_par = {n_par[0] / npn, n_par[1] / npn, n_par[2] / npn};
        std::array<double, 3> rope2 = {d2[0] / n2, d2[1] / n2, d2[2] / n2};
        std::array<double, 3> cr = {n_par[1] * rope2[2] - n_par[2] * rope2[1],
                                    n_par[2] * rope2[0] - n_par[0] * rope2[2],
                                    n_par[0] * rope2[1] - n_par[1] * rope2[0]};
        const double ncr = norm3(cr);
        n_bar = {cr[0] / ncr, cr[1] / ncr, cr[2] / ncr};
    }

    std::array<double, 3> Ftot;
    for (int i = 0; i < 3; ++i) {
        Ftot[i] = (d1[i] / n1) * Fr_l + (d2[i] / n2) * Fr_r + extra_force * n_bar[i];
    }
    Ftot[2] += prm.m * (-prm.g);

    const double flegnorm = norm3(Fleg);
    if (flegnorm > 0.0 && t <= prm.T_th) {
        Ftot[0] += Fleg[0]; Ftot[1] += Fleg[1]; Ftot[2] += Fleg[2];
    }

    std::array<double, 3> rhs = {Ftot[0] / prm.m - b_dyn[0],
                                 Ftot[1] / prm.m - b_dyn[1],
                                 Ftot[2] / prm.m - b_dyn[2]};
    std::array<double, 3> y = solve3x3(A, rhs);

    return {psid, l1d, l2d, y[0], y[1], y[2]};
}

static State add_scaled(const State& x, double s, const State& d) {
    State r;
    for (int i = 0; i < 6; ++i) r[i] = x[i] + s * d[i];
    return r;
}

State integrate_dynamics(const State& x0, double t0, double dt, int n_steps,
                         const std::vector<double>& Fr_l,
                         const std::vector<double>& Fr_r,
                         const std::array<double, 3>& Fleg,
                         const std::string& method, const ModelParams& prm,
                         const std::vector<double>& extra_forces,
                         double& t_final) {
    State x = x0;
    double t = t0;
    for (int i = 0; i < n_steps - 1; ++i) {
        const double ef = extra_forces.empty() ? 0.0 : extra_forces[i];
        if (method == "eul") {
            State k = dynamics(t, x, Fr_l[i], Fr_r[i], Fleg, prm, ef);
            x = add_scaled(x, dt, k);
        } else if (method == "rk4") {
            const double h = dt;
            State k1 = dynamics(t, x, Fr_l[i], Fr_r[i], Fleg, prm, ef);
            State k2 = dynamics(t + 0.5 * h, add_scaled(x, 0.5 * h, k1), Fr_l[i], Fr_r[i], Fleg, prm, ef);
            State k3 = dynamics(t + 0.5 * h, add_scaled(x, 0.5 * h, k2), Fr_l[i], Fr_r[i], Fleg, prm, ef);
            State k4 = dynamics(t + h, add_scaled(x, h, k3), Fr_l[i], Fr_r[i], Fleg, prm, ef);
            for (int j = 0; j < 6; ++j) x[j] += (h / 6.0) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
        } else {
            throw std::invalid_argument("Unknown integration method: " + method);
        }
        t += dt;
    }
    t_final = t;
    return x;
}

void compute_rollout(const State& x0, double t0, double dt_dyn, int N_dyn,
                     const std::vector<double>& Fr_l,
                     const std::vector<double>& Fr_r,
                     const std::array<double, 3>& Fleg,
                     const std::string& method, int int_steps,
                     const ModelParams& prm,
                     const std::vector<double>& extra_forces,
                     std::vector<double>& states_out,
                     std::vector<double>& t_out) {
    states_out.assign(6 * N_dyn, 0.0);
    t_out.assign(N_dyn, 0.0);

    auto set_col = [&](int i, const State& s) {
        for (int j = 0; j < 6; ++j) states_out[j * N_dyn + i] = s[j];
    };

    if (int_steps == 0) {
        // plain integration over N_dyn knots
        State x = x0;
        double t = t0;
        set_col(0, x);
        t_out[0] = t;
        for (int i = 0; i < N_dyn - 1; ++i) {
            const double ef = extra_forces.empty() ? 0.0 : extra_forces[i];
            if (method == "eul") {
                State k = dynamics(t, x, Fr_l[i], Fr_r[i], Fleg, prm, ef);
                x = add_scaled(x, dt_dyn, k);
            } else {
                const double h = dt_dyn;
                State k1 = dynamics(t, x, Fr_l[i], Fr_r[i], Fleg, prm, ef);
                State k2 = dynamics(t + 0.5 * h, add_scaled(x, 0.5 * h, k1), Fr_l[i], Fr_r[i], Fleg, prm, ef);
                State k3 = dynamics(t + 0.5 * h, add_scaled(x, 0.5 * h, k2), Fr_l[i], Fr_r[i], Fleg, prm, ef);
                State k4 = dynamics(t + h, add_scaled(x, h, k3), Fr_l[i], Fr_r[i], Fleg, prm, ef);
                for (int j = 0; j < 6; ++j) x[j] += (h / 6.0) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
            }
            t += dt_dyn;
            set_col(i + 1, x);
            t_out[i + 1] = t;
        }
        return;
    }

    const double dt_step = dt_dyn / static_cast<double>(int_steps - 1);
    State prev = x0;
    set_col(0, x0);
    t_out[0] = t0;
    for (int i = 1; i < N_dyn; ++i) {
        std::vector<double> frl(int_steps, Fr_l[i - 1]);
        std::vector<double> frr(int_steps, Fr_r[i - 1]);
        std::vector<double> ef;
        if (!extra_forces.empty()) ef.assign(int_steps, extra_forces[i - 1]);
        double tf = 0.0;
        State xf = integrate_dynamics(prev, t_out[i - 1], dt_step, int_steps, frl, frr,
                                      Fleg, method, prm, ef, tf);
        set_col(i, xf);
        t_out[i] = tf;
        prev = xf;
    }
}

}  // namespace climbingrobot
