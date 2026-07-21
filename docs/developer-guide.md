# FIGAROH Developer Guide

## Overview

FIGAROH is a Python toolbox for **robot dynamics identification** and
**geometric calibration** built on top of Pinocchio.  It provides:

- Base parameter identification from measured joint trajectories
- Optimal excitation trajectory generation (via IPOPT)
- Geometric calibration with marker-based pose measurements
- Physical consistency enforcement for identified parameters

Version 0.4.3 introduces an **optional CasADi computational backend** that
replaces numerical differentiation with analytical automatic differentiation
(AD), yielding **1.6–3× speedup** on optimal trajectory generation.

---

## 1. Prerequisites

### 1.1 Required

| Package | Version | Channel | Notes |
|---------|---------|---------|-------|
| Python | ≥ 3.8 | — | Tested on 3.12 |
| pinocchio | ≥ 2.6 | conda-forge | `pin` from PyPI **does not** support CasADi |
| numpy | ≥ 1.24 | PyPI / conda-forge | |
| scipy | ≥ 1.11 | PyPI / conda-forge | |
| cyipopt | ≥ 1.1 | conda-forge | IPOPT Python interface |
| numdifftools | ≥ 0.9 | PyPI | Numerical gradient (numerical backend) |
| matplotlib | ≥ 3.7 | PyPI / conda-forge | |
| pyyaml | ≥ 6 | PyPI / conda-forge | Configuration parsing |

### 1.2 Optional — CasADi Backend

| Package | Version | Channel | Notes |
|---------|---------|---------|-------|
| casadi | ≥ 3.6.7 | conda-forge | Symbolic AD framework |
| pinocchio | ≥ 4.0 | **conda-forge only** | Must replace `pin` from PyPI |

> ⚠️ **Critical**: The PyPI `pin` package does **NOT** include compiled
> CasADi Python bindings (`pinocchio_pywrap_casadi.so`).  You **must** use
> conda-forge `pinocchio ≥ 4.0` for the CasADi backend.  The default
> `numerical` backend works with either distribution.

---

## 2. Installation

### 2.1 Quick Start (numerical backend only)

```bash
pip install figaroh
```

This installs the default `numerical` backend.  The CasADi backend will not be
available.

### 2.2 Full Installation with CasADi (pixi, recommended)

[pixi](https://pixi.sh) is the project's package manager.  It manages both
PyPI and conda-forge dependencies in a single lock file.

```bash
git clone https://github.com/your-org/figaroh-plus.git
cd figaroh-plus
pixi install                    # base environment (numerical backend)
pixi run setup-casadi           # add CasADi backend + OpenMP
```

Verify:

```bash
pixi run python -c "from figaroh.backend import CasadiBackend; print('OK')"
```

### 2.3 Full Installation with conda

```bash
conda create -n figaroh python=3.12
conda activate figaroh

# Install pinocchio from conda-forge (replaces pip 'pin')
conda install -c conda-forge pinocchio>=4.0 casadi>=3.6.7

# Install figaroh and remaining dependencies
pip install figaroh[casadi]
```

### 2.4 Verification Script

```bash
python -c "
from figaroh.backend import NumericalBackend, CasadiBackend

# Numerical backend always available
nb = NumericalBackend()
print('Numerical backend:', nb.name)

# CasADi — may fail if on PyPI pin
try:
    cb = CasadiBackend()
    print('CasADi backend:', cb.name, '(conda-forge pinocchio detected)')
except ImportError as e:
    print('CasADi NOT available:', e)
"
```

---

## 3. Project Structure

```
figaroh-plus/
├── src/figaroh/               # Main library
│   ├── backend/               # ★ NEW: Backend abstraction layer
│   │   ├── __init__.py        #     Lazy CasadiBackend import
│   │   ├── base.py            #     Backend ABC + create_backend()
│   │   ├── numerical.py       #     NumericalBackend (cyipopt + nd)
│   │   └── casadi.py          #     CasadiBackend (cpin + cs.nlpsol)
│   ├── optimal/               # Optimal trajectory generation
│   │   ├── base_optimal_trajectory.py
│   │   ├── config.py          # Configuration loader (backend key)
│   │   └── contraints.py      # TrajectoryConstraintManager
│   ├── identification/        # Parameter identification
│   ├── calibration/           # Geometric calibration
│   ├── tools/                 # Regressor, IPOPT, QR, spline
│   └── utils/                 # Cubic spline, config parser
├── figaroh-examples/          # Submodule: robot examples
│   └── examples/ur10/         # UR10 identification + calibration
├── tests/                     # Unit & integration tests
├── docs/                      # Documentation
│   └── superpowers/
│       ├── specs/             # Design documents
│       └── plans/             # Implementation plans
├── pyproject.toml             # Build config + dependencies
├── pixi.toml                  # pixi environment definitions
└── comet.yaml                 # CI / guard configuration
```

---

## 4. Backend Architecture

### 4.1 Design

```
                    ┌──────────────────────────┐
                    │  BaseOptimalTrajectory     │
                    │  backend="numerical"       │
                    └────────────┬─────────────┘
                                 │
                    ┌────────────▼─────────────┐
                    │     Backend (ABC)         │
                    │  build_regressor(q,v,a)   │
                    │  create_solver(nlp, opts) │
                    └────────────┬─────────────┘
                                 │
              ┌──────────────────┴──────────────────┐
              ▼                                     ▼
   ┌──────────────────────┐          ┌──────────────────────────┐
   │  NumericalBackend     │          │  CasadiBackend           │
   │  (default)            │          │  (optional)              │
   ├──────────────────────┤          ├──────────────────────────┤
   │ regressor:            │          │ regressor:               │
   │  for-loop Python      │          │  pinocchio.casadi →      │
   │  pin.computeJoint     │          │  cs.Function.map(N)      │
   │  TorqueRegressor      │          │                          │
   │ solver:               │          │ solver:                  │
   │  cyipopt.Problem       │          │  cs.nlpsol('ipopt',nlp) │
   │  + nd.Gradient        │          │  → analytical gradient   │
   │  + nd.Jacobian        │          │  → analytical Jacobian   │
   └──────────────────────┘          └──────────────────────────┘
```

### 4.2 Selection

Backends are selected with **three-level precedence**:

| Priority | Method | Example |
|----------|--------|---------|
| 1 (highest) | Programmatic argument | `BaseOptimalTrajectory(backend="casadi")` |
| 2 | YAML config key | `backend: casadi` in config file |
| 3 (default) | Hardcoded fallback | `"numerical"` |

```yaml
# config/robot_config.yaml
identification:
  backend: casadi  # ← overrides default "numerical"
```

```python
from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

# Use config default (numerical)
traj = BaseOptimalTrajectory(robot, active_joints, "config.yaml")

# Force CasADi regardless of config
traj = BaseOptimalTrajectory(robot, active_joints, "config.yaml", backend="casadi")

# Pass a pre-configured backend instance
from figaroh.backend import CasadiBackend
traj = BaseOptimalTrajectory(robot, active_joints, "config.yaml", backend=CasadiBackend(robot))
```

---

## 5. Running Tests

### 5.1 All Tests

```bash
pixi run test
```

Expected output:
```
231 passed, 2 skipped, 14 failed
```

The 14 failures are **pre-existing** test-isolation issues in
`test_qr_decomposition.py` (2) and `test_robotvisualization.py` (12) —
unrelated to the backend changes.

### 5.2 Backend-Specific Tests

```bash
# Numerical backend consistency
pixi run python -m pytest tests/unit/test_backend.py -v

# Configuration parsing
pixi run python -m pytest tests/unit/test_config.py -v

# Quick smoke test
pixi run python -m pytest tests/unit/test_backend.py -q
```

### 5.3 Run Specific Test

```bash
pixi run python -m pytest tests/unit/test_backend.py::TestBackendABC -v
```

---

## 6. Running Benchmarks

### 6.1 Quick Regressor Comparison

```bash
pixi run python scripts/ur10_benchmark.py
```

Output compares `NumericalBackend` vs `CasadiBackend` regressor build time
across sample sizes (50–1000).

### 6.2 Full Trajectory Optimization

```python
from figaroh.tools.load_robot import load_robot
from examples.ur10.utils.ur10_tools import OptimalTrajectoryIPOPT
import time

robot = load_robot("ur10_robot.urdf", package_dirs="../../models", load_by_urdf=True)
active_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# Numerical backend
t0 = time.perf_counter()
traj_num = OptimalTrajectoryIPOPT(robot, active_joints, "config.yaml", backend="numerical")
traj_num.initialize()
result = traj_num.solve(stack_reps=1)
print(f"Numerical: {time.perf_counter()-t0:.1f}s, success={result is not None}")

# CasADi backend
t0 = time.perf_counter()
traj_cas = OptimalTrajectoryIPOPT(robot, active_joints, "config.yaml", backend="casadi")
traj_cas.initialize()
result = traj_cas.solve(stack_reps=1)
print(f"CasADi: {time.perf_counter()-t0:.1f}s, success={result is not None}")
```

### 6.3 Expected Performance (UR10, 6-DOF)

| n_wps | freq | Vars | Cons | Numerical | CasADi | Speedup |
|-------|------|------|------|-----------|--------|---------|
| 4 | 20 Hz | 18 | 1470 / 762 | 113 s | **71 s** | 1.6× |
| 8 | 50 Hz | 42 | 14 448 | ~30 min* | ~3 min* | ~10×* |

*\*extrapolated from per-iteration timing*

---

## 7. Fourier Trajectory Optimization

FIGAROH v0.4.3+ 新增**傅里叶级数激励轨迹**优化，作为三次样条的替代方案。使用全符号化 CasADi NLP 管线 + D-最优目标函数，条件数通常比样条低 15-30%。

- 配置、运行、故障排查等完整指南：**[fourier-optimal-trajectory.md](fourier-optimal-trajectory.md)**
- 快速配置示例：

```yaml
optimal_trajectory:
  trajectory:
    type: "fourier"
    fourier:
      n_harmonics: 5
      n_samples: 200
```

> 傅里叶优化需要 CasADi backend（`backend: "casadi"`）和 pinocchio.casadi 绑定。

---

## 8. Configuration Reference

### 11.1 Unified YAML Format (recommended)

```yaml
robot:
  properties:
    joints:
      active_joints:
        - shoulder_pan_joint
        - shoulder_lift_joint
        - elbow_joint
        - wrist_1_joint
        - wrist_2_joint
        - wrist_3_joint

problem:
  backend: numerical          # "numerical" (default) or "casadi"
  is_external_wrench: false
  is_joint_torques: true
  has_friction: true
  has_actuator_inertia: true
  has_joint_offset: false
  soft_lim: 0.05

trajectory:
  waypoints: 8                # number of trajectory waypoints
  frequency: 50               # sampling frequency (Hz)
  segment_duration: 2.0       # seconds per segment
  max_attempts: 500           # feasible init search attempts

constraints:
  joint_position:
    enabled: true
  joint_velocity:
    enabled: true
  joint_torque:
    enabled: true

output:
  directory: results/
```

### 11.2 Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `backend` | `"numerical"` | Computation backend |
| `n_wps` | 8 | Number of waypoints |
| `freq` | 50 | Sampling frequency (Hz) |
| `t_s` | 2.0 | Segment duration (s) |
| `soft_lim` | 0.05 | Soft joint limit margin (rad) |
| `max_attempts` | 500 | Feasible init search attempts |
| `tol` | 1e-6 | Column elimination tolerance |
| `has_friction` | `false` | Include viscous + static friction parameters |
| `has_actuator_inertia` | `false` | Include actuator inertia parameters |
| `has_joint_offset` | `false` | Include joint offset parameters |

### 11.3 CasADi Solver Options

The CasADi backend maps `IPOPTConfig` options to `cs.nlpsol` options
automatically:

| IPOPTConfig | cs.nlpsol | Description |
|-------------|-----------|-------------|
| `tolerance` | `ipopt.tol` | Convergence tolerance |
| `acceptable_tolerance` | `ipopt.acceptable_tol` | Fallback tolerance |
| `max_iterations` | `ipopt.max_iter` | Max IPOPT iterations |
| `max_cpu_time` | `ipopt.max_cpu_time` | Max CPU time (s) |
| `print_level` | `ipopt.print_level` | Output verbosity (0–12) |
| `mu_strategy` | `ipopt.mu_strategy` | Barrier parameter strategy |
| `linear_solver` | `ipopt.linear_solver` | Linear solver (`mumps`, etc.) |
| `hessian_approximation` | `ipopt.hessian_approximation` | Hessian method |

---

## 9. CasADi Function Caching

The CasADi backend caches compiled symbolic Functions in
`~/.figaroh/casadi_cache/` to avoid rebuilding the regressor graph on
every run.  Cache keys are derived from the robot model's name, joint
count, and total mass (a SHA-256 fingerprint).

| Scenario | Time |
|----------|------|
| First build (no cache) | ~0.05 s (C++ native) |
| Cache hit (subsequent runs) | ~0.03 s |
| Cache miss (model changed) | ~0.05 s (rebuild + save) |

The cache is invalidated automatically when:
- The robot model changes (different URDF)
- The cache version is bumped (`_CACHE_VERSION` in `casadi.py`)

To clear the cache manually:
```bash
rm -rf ~/.figaroh/casadi_cache/
```

---

## 10. Architecture Insight: Regressor as Jacobian

The regressor matrix **H** is mathematically the **Jacobian of inverse
dynamics w.r.t. the inertial parameter vector**:

```
H(q, dq, ddq) = ∂ RNEA(q, dq, ddq) / ∂ π

where π = [Lxx, Lxy, Lxz, Lyy, Lyz, Lzz, lx, ly, lz, m]
      per body (10 barycentric parameters)
```

This means:
1. The regressor is NOT a separate computation — it's the result of
   auto-differentiating RNEA w.r.t. parameters
2. Because `pinocchio.casadi` builds RNEA inside the CasADi SX graph,
   `cs.jacobian(cpin.rnea(...), params)` gives the **analytical
   regressor** with a full derivative chain
3. `cpin.computeJointTorqueRegressor` is a C++ optimisation of this
   Jacobian — it returns the same SX expression but with a more
   efficient internal implementation

This insight comes from the MATLAB CasADi identification pipeline
(`/home/tyche/Documents/identification/x/`) which explicitly uses
`jacobian(dyn.tau, param_vec)` for the regressor and saves the
resulting CasADi Functions to `.casadi` files for reuse.

For figaroh, this means the regressor `W_fun(q, v, a)` is **fully
differentiable** — we can compute `cs.gradient(cond(W), X)` analytically
through the entire chain: spline → RNEA → regressor → condition number.
The only remaining bottleneck is the column-elimination step (QR
pivoting), which is wrapped as a numpy Callback.

---

## 11. Troubleshooting

### 11.1 `ImportError: CasADi backend requires conda-forge pinocchio`

**Cause**: You installed `pin` from PyPI (`pip install pin`), which lacks
compiled CasADi bindings.

**Fix**:

```bash
pip uninstall pin
pixi add --feature casadi casadi pinocchio
# or
conda install -c conda-forge casadi pinocchio
```

### 11.2 `ModuleNotFoundError: No module named 'pinocchio.casadi'`

**Cause**: conda-forge pinocchio is installed but its version doesn't
include CasADi support (pinocchio < 4.0).

**Fix**:

```bash
conda install -c conda-forge "pinocchio>=4.0"
```

### 11.3 `EXIT: Converged to a point of local infeasibility`

**Cause**: The random feasible-initial-trajectory search exhausted all
attempts without finding a valid starting point.

**Fixes** (try in order):

1. Reduce waypoints: set `n_wps: 4` in config
2. Reduce frequency: set `freq: 20` in config
3. Increase max attempts: set `max_attempts: 2000`
4. Relax soft limits: set `soft_lim: 0.1`

### 11.4 Numerical path is faster than CasADi

For problems with **very few variables** (< 10), the CasADi NLP construction
overhead (~35 s) may outweigh the per-iteration speedup.  CasADi's advantage
grows with problem size — expect 3–10× speedup for 30+ variable problems.

### 11.5 `Assertion 'has_derivative()' failed`

**Cause**: A `cs.Callback` was constructed without `{"enable_fd": True}`.

**Fix**: Ensure all Callback nodes pass `{"enable_fd": True}` to
`self.construct()`.

---

## 12. API Reference

### 12.1 `figaroh.backend`

```python
from figaroh.backend import Backend, create_backend, NumericalBackend, CasadiBackend
```

#### `Backend` (ABC)

| Method | Signature | Returns |
|--------|-----------|---------|
| `build_regressor` | `(q, v, a, identif_config)` | `np.ndarray` shape `(N*nv, n_param)` |
| `create_solver` | `(nlp_def, opts)` | `Callable` → result dict |
| `name` | (property) | `"numerical"` or `"casadi"` |

#### `create_backend(backend, robot=None, **kwargs) → Backend`

Factory function.  Accepts:
- `"numerical"` → `NumericalBackend`
- `"casadi"` → `CasadiBackend`
- `Backend` instance → passthrough
- Duck-typed object with `.name` attribute → passthrough
- Unknown string → `ValueError`

### 12.2 `BaseOptimalTrajectory`

```python
BaseOptimalTrajectory(
    robot,                    # RobotWrapper
    active_joints,            # List[str]
    config_file,              # str
    backend="numerical",      # BackendType
)
```

Key methods:
- `initialize()` — set up spline, constraints, base parameters
- `solve(stack_reps=2)` — run IPOPT optimization
- `plot_results()` — visualise trajectories
- `save_results(output_dir)` — export to YAML + CSV

---

## 13. Development Workflow

### 13.1 Environment Setup

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/your-org/figaroh-plus.git
cd figaroh-plus

# Install all features (numerical + casadi + dev)
pixi install -e casadi
pixi install -e dev

# Run tests
pixi run test

# Run linters
pixi run lint
```

### 13.2 Making Changes

1. Create a feature branch: `git checkout -b feature/my-change`
2. Run tests before modifying: `pixi run test`
3. Implement changes following TDD: test → code → refactor
4. Run full test suite: `pixi run test`
5. Run UR10 benchmark if backend code changed: `python scripts/ur10_benchmark.py`

### 13.3 Adding a New Backend

1. Subclass `Backend` in a new file under `src/figaroh/backend/`
2. Implement all 5 abstract methods (`build_regressor`, `create_solver`, `name`)
3. Register in `__init__.py`
4. Add to `create_backend()` factory
5. Write tests in `tests/unit/test_backend.py`
