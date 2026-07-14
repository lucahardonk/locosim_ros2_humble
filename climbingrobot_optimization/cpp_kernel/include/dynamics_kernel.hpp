// dynamics_kernel.hpp
//
// Dependency-free C++ implementation of the performance-critical numerical
// kernel of the ALPINE two-rope climbing-robot model: the single-shooting
// dynamics rollout (dynamics.m / integrate_dynamics.m / computeRollout.m).
//
// The rollout is evaluated thousands of times inside the SQP/IPOPT solvers,
// so it is the correct target for a native implementation.  All linear
// algebra is fixed-size (3x3 solve, 3x2 Jacobian) and hand-rolled, so the
// kernel has NO external dependencies (no Eigen) and builds anywhere.
//
// Units / conventions match the MATLAB source exactly:
//   state x = [psi, l1, l2, psid, l1d, l2d]

#ifndef CLIMBINGROBOT_DYNAMICS_KERNEL_HPP
#define CLIMBINGROBOT_DYNAMICS_KERNEL_HPP

#include <array>
#include <vector>
#include <string>

namespace climbingrobot {

using State = std::array<double, 6>;

struct ModelParams {
    double m;            // mass [kg]
    double g;            // gravity [m/s^2]
    double b;            // anchor distance [m]
    std::array<double, 3> p_a1;
    std::array<double, 3> p_a2;
    double T_th;         // leg thrust duration [s]
};

// Forward kinematics: (psi,l1,l2) -> (px,py,pz)
void forward_kin(const ModelParams& prm, double psi, double l1, double l2,
                 double& px, double& py, double& pz);

// State derivative dxdt = f(x,u). Mirrors dynamics.m.
State dynamics(double t, const State& x, double Fr_l, double Fr_r,
               const std::array<double, 3>& Fleg, const ModelParams& prm,
               double extra_force);

// Integrate n_steps knots with method "eul" or "rk4". Returns final state;
// optionally fills the full (6 x n_steps) trajectory (column-major flattened).
State integrate_dynamics(const State& x0, double t0, double dt, int n_steps,
                         const std::vector<double>& Fr_l,
                         const std::vector<double>& Fr_r,
                         const std::array<double, 3>& Fleg,
                         const std::string& method, const ModelParams& prm,
                         const std::vector<double>& extra_forces,
                         double& t_final);

// Single-shooting rollout. Mirrors computeRollout.m.
//   states_out : 6 x N_dyn (column-major flattened, size 6*N_dyn)
//   t_out      : N_dyn
void compute_rollout(const State& x0, double t0, double dt_dyn, int N_dyn,
                     const std::vector<double>& Fr_l,
                     const std::vector<double>& Fr_r,
                     const std::array<double, 3>& Fleg,
                     const std::string& method, int int_steps,
                     const ModelParams& prm,
                     const std::vector<double>& extra_forces,
                     std::vector<double>& states_out,
                     std::vector<double>& t_out);

}  // namespace climbingrobot

#endif  // CLIMBINGROBOT_DYNAMICS_KERNEL_HPP
