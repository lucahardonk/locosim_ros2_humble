# climbingrobot_optimization

Python (+ optional C++) port of the MATLAB **MPC controller** and **jump optimal
control** code from the ALPINE [`climbing_robots2`](https://github.com/mfocchi/climbing_robots2)
repository, packaged as a ROS 2 **Humble** `ament_python` package.

The package provides:

* an **offline jump optimiser** (`jump_optimizer`) — plans a leg-impulse + rope
  force trajectory that flings the robot from a start CoM to a target CoM;
* an **online receding-horizon MPC** (`mpc_controller`) — corrects the nominal
  rope forces to track the planned CoM trajectory under disturbances;
* a **C++ rollout kernel** (`cpp_kernel`) that accelerates the single-shooting
  dynamics rollout (the optimiser hot-spot) by ~300× while matching the NumPy
  reference to machine precision;
* an optional **CasADi / IPOPT backend** (`casadi_backend`) as an alternative to
  the default SciPy SLSQP solvers;
* two **ROS 2 nodes** wrapping the planner and the controller.

---

## Layout

```
climbingrobot_optimization/
├── climbingrobot_optimization/        # python package
│   ├── params.py                      # Params dataclass (model / discretisation / weights)
│   ├── kinematics.py                  # forward kin, Jacobian, cartesian<->state
│   ├── dynamics.py                    # dynamics, integrator, rollout (C++ accelerated)
│   ├── jump_optimizer.py              # offline jump optimal control (SLSQP)
│   ├── mpc_controller.py              # online receding-horizon MPC (SLSQP)
│   ├── casadi_backend.py              # optional CasADi/IPOPT solver
│   └── ros2_nodes/
│       ├── jump_optimizer_node.py     # ROS 2 planner node
│       └── mpc_controller_node.py     # ROS 2 controller node
├── cpp_kernel/                        # C++ rollout kernel + pybind11 bindings
│   ├── include/dynamics_kernel.hpp
│   ├── src/dynamics_kernel.cpp
│   ├── src/bindings.cpp
│   └── CMakeLists.txt
├── launch/climbingrobot_optimization.launch.py
├── test/test_conversion.py
├── package.xml  setup.py  setup.cfg  resource/climbingrobot_optimization
```

---

## State & decision-vector conventions

State vector (used everywhere):

```
x = [psi, l1, l2, psid, l1d, l2d]
```

* `psi`  – swing angle of the rope plane about the anchor axis [rad]
* `l1`   – length of rope 1 (from anchor 1) [m]
* `l2`   – length of rope 2 (from anchor 2) [m]
* `*d`   – time derivatives

World frame is attached to anchor 1; anchor 2 is at `[0, b, 0]`.

Offline jump decision vector:

```
z = [Fleg_x, Fleg_y, Fleg_z, Tf, Fr_l(0..N-1), Fr_r(0..N-1)]
```

MPC decision vector (single shooting over `mpc_N` knots):

```
[delta_Fr_l(mpc_N), delta_Fr_r(mpc_N)]           # + propeller_forces(mpc_N) for the propeller variant
```

---

## Dependencies

Python (pip or rosdep):

```
numpy  scipy          # required
casadi  osqp          # optional (casadi_backend)
pybind11  cmake        # only to build the C++ kernel
```

ROS 2 (Humble): `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`.

---

## Building the C++ kernel (optional but recommended)

The pure-Python code runs out of the box; the kernel is only for speed. It is
dependency-free (no Eigen) and builds with any C++14 compiler.

```bash
cd cpp_kernel
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
# produces climbingrobot_kernel.<ext>.so
```

`dynamics.compute_rollout(...)` and both optimisers auto-detect the compiled
module `climbingrobot_kernel` on the `PYTHONPATH` and use it automatically;
pass `use_cpp=False` to force the NumPy path. `dynamics.kernel_available()`
reports which path is active.

---

## Quick start (plain Python)

```python
import numpy as np
from climbingrobot_optimization.params import Params
from climbingrobot_optimization import jump_optimizer, mpc_controller

p  = Params.normal_test()
p0 = np.array([0.5, 2.5, -6.0])
pf = np.array([0.5, 4.0, -4.0])

# 1) offline plan
sol = jump_optimizer.optimize_cpp(p0, pf, Fleg_max=300.0, Fr_max=90.0, mu=0.8, params=p)
print("solved", sol["problem_solved"], "Tf", sol["Tf"], "landing", sol["achieved_target"])

# 2) online MPC correction for a (perturbed) measured state
p.mpc_dt = sol["Tf"] / (p.N_dyn - 1)
mpc_N = int(0.4 * p.N_dyn)
state = np.array([sol["psi"][0], sol["l1"][0] + 0.2, sol["l2"][0] - 0.2,
                  sol["psid"][0], sol["l1d"][0], sol["l2d"][0]])
delta, exitflag, cost = mpc_controller.optimize_cpp_mpc(
    state, 0.0, sol["p"], sol["Fr_l"], sol["Fr_r"], Fr_max=90.0, mpc_N=mpc_N, params=p)
```

---

## ROS 2 usage

```bash
# in a colcon workspace: <ws>/src/climbingrobot_optimization
colcon build --packages-select climbingrobot_optimization
source install/setup.bash

# bring up planner + controller together
ros2 launch climbingrobot_optimization climbingrobot_optimization.launch.py

# or run individually
ros2 run climbingrobot_optimization jump_optimizer_node
ros2 run climbingrobot_optimization mpc_controller_node
```

The planner publishes the reference CoM (`~/reference_com`), nominal rope forces
(`~/reference_frl`, `~/reference_frr`) and flight time (`~/jump_time`). The
launch file remaps those onto the controller's `~/set_reference_*` inputs. Feed
the current state on `/mpc_controller_node/state` (a 6-element
`std_msgs/Float64MultiArray`) and read corrected forces on
`/mpc_controller_node/rope_forces`.

> The nodes use `std_msgs`/`geometry_msgs` + parameters instead of custom
> `.srv`/`.msg` types so the package stays a pure `ament_python` package with no
> `rosidl` build step. Swap in your own interfaces if your stack already
> defines them.

---

## Tests

```bash
PYTHONPATH=$PWD:$PWD/climbingrobot_optimization pytest test -v
```

Validates C++/NumPy rollout parity, jump convergence & unilateral force limits,
and that the MPC produces a cost-reducing, constraint-respecting correction.
