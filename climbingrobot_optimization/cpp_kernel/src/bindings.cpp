// bindings.cpp — pybind11 module exposing the C++ rollout kernel to Python.
//
// Exposed as the `climbingrobot_kernel` module.  dynamics.py imports it and
// uses `compute_rollout` transparently when available.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "dynamics_kernel.hpp"

namespace py = pybind11;
using namespace climbingrobot;

static ModelParams make_params(double m, double g, double b,
                               py::array_t<double> p_a1, py::array_t<double> p_a2,
                               double T_th) {
    ModelParams prm;
    prm.m = m; prm.g = g; prm.b = b; prm.T_th = T_th;
    auto a1 = p_a1.unchecked<1>();
    auto a2 = p_a2.unchecked<1>();
    for (int i = 0; i < 3; ++i) { prm.p_a1[i] = a1(i); prm.p_a2[i] = a2(i); }
    return prm;
}

static std::vector<double> to_vec(py::array_t<double> a) {
    auto r = a.unchecked<1>();
    std::vector<double> v(r.shape(0));
    for (py::ssize_t i = 0; i < r.shape(0); ++i) v[i] = r(i);
    return v;
}

// Returns (states: numpy 6 x N_dyn, t: numpy N_dyn)
py::tuple py_compute_rollout(py::array_t<double> x0, double t0, double dt_dyn, int N_dyn,
                             py::array_t<double> Fr_l, py::array_t<double> Fr_r,
                             py::array_t<double> Fleg, const std::string& method,
                             int int_steps, double m, double g, double b,
                             py::array_t<double> p_a1, py::array_t<double> p_a2,
                             double T_th, py::array_t<double> extra_forces) {
    ModelParams prm = make_params(m, g, b, p_a1, p_a2, T_th);

    State x0_arr{};
    auto x0r = x0.unchecked<1>();
    for (int i = 0; i < 6; ++i) x0_arr[i] = x0r(i);

    std::array<double, 3> fleg{};
    auto fl = Fleg.unchecked<1>();
    for (int i = 0; i < 3; ++i) fleg[i] = fl(i);

    std::vector<double> frl = to_vec(Fr_l);
    std::vector<double> frr = to_vec(Fr_r);
    std::vector<double> ef = to_vec(extra_forces);

    std::vector<double> states_out, t_out;
    compute_rollout(x0_arr, t0, dt_dyn, N_dyn, frl, frr, fleg, method, int_steps,
                    prm, ef, states_out, t_out);

    py::array_t<double> states({6, N_dyn});
    auto sm = states.mutable_unchecked<2>();
    for (int j = 0; j < 6; ++j)
        for (int i = 0; i < N_dyn; ++i)
            sm(j, i) = states_out[j * N_dyn + i];

    py::array_t<double> tarr(N_dyn);
    auto tm = tarr.mutable_unchecked<1>();
    for (int i = 0; i < N_dyn; ++i) tm(i) = t_out[i];

    return py::make_tuple(states, tarr);
}

PYBIND11_MODULE(climbingrobot_kernel, mod) {
    mod.doc() = "C++ dynamics rollout kernel for the ALPINE climbing robot";
    mod.def("compute_rollout", &py_compute_rollout,
            py::arg("x0"), py::arg("t0"), py::arg("dt_dyn"), py::arg("N_dyn"),
            py::arg("Fr_l"), py::arg("Fr_r"), py::arg("Fleg"), py::arg("method"),
            py::arg("int_steps"), py::arg("m"), py::arg("g"), py::arg("b"),
            py::arg("p_a1"), py::arg("p_a2"), py::arg("T_th"), py::arg("extra_forces"),
            "Single-shooting rollout (returns states 6xN_dyn and time vector).");
}
