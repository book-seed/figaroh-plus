# Fourier Excitation Trajectory Configuration

FIGAROH supports **Fourier series** trajectory optimization as an alternative to the default cubic spline parameterization. This is enabled via the `trajectory_type` field in the unified configuration format.

Fourier trajectories use the **FourierOptimizationStrategy**, which constructs a fully symbolic CasADi NLP (SX/MX hybrid) to solve the D-optimal excitation trajectory problem. The resulting trajectories achieve significantly lower condition numbers compared to random trajectories.

> **Note:** This feature requires the CasADi backend. See [Environment Requirements](#environment-requirements) below.

---

## YAML Configuration

In the unified configuration format, set `trajectory.type` to `"fourier"` and configure Fourier-specific parameters under `trajectory.fourier`:

```yaml
# config/robot_config.yaml
robot:
  name: "my_robot"
  urdf_path: "models/robot.urdf"
  properties:
    joints:
      active_joints:
        - joint1
        - joint2
        - joint3

problem:
  backend: "casadi"             # Fourier requires CasADi backend
  soft_lim: 0.05
  has_friction: true
  has_actuator_inertia: true

trajectory:
  type: "fourier"               # "spline" (default) or "fourier"
  fourier:
    n_harmonics: 5              # Number of harmonic terms (default: 5)
    fourier_frequency: null     # Base frequency; null = 2*pi / T
    n_samples: 200              # Number of sampling points (default: 200)
    reg_lambda: 1.0e-6          # Regularization coefficient (default: 1e-6)
    tanh_alpha_opt: 10          # Tanh steepness during optimization (default: 10)
    tanh_alpha_id: 100          # Tanh steepness for identification (default: 100)

constraints:
  joint_position:
    enabled: true
  joint_velocity:
    enabled: true
  joint_torque:
    enabled: true
```

### Fourier Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `n_harmonics` | `5` | Number of sine/cosine harmonic pairs per joint |
| `fourier_frequency` | `null` | Base angular frequency; `null` computes `2*pi / T` from segment duration |
| `n_samples` | `200` | Number of discrete sampling points for evaluation |
| `reg_lambda` | `1.0e-6` | Tikhonov regularization on the information matrix to guarantee SPD for Cholesky |
| `tanh_alpha_opt` | `10` | Steepness of `tanh(alpha * v)` friction approximation during optimization (gradient-friendly) |
| `tanh_alpha_id` | `100` | Steepness of `tanh(alpha * v)` for the identification phase (higher fidelity) |

---

## Python API

### Basic Usage

```python
from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

# Set trajectory_type in config, or it defaults to "spline"
traj = BaseOptimalTrajectory(
    robot, active_joints, "config.yaml",
)
traj.initialize()
traj.solve()
```

### Verifying Condition Number

After solving, verify the regressor condition number:

```python
import numpy as np

W_b = traj.results["W_b"][0]  # Base regressor matrix
cond = np.linalg.cond(W_b)
print(f"Condition number: {cond:.2f}")
# Fourier trajectories typically yield condition numbers < 100
# Random trajectories often exceed 1000
```

---

## Architecture

The Fourier trajectory optimization follows a fully symbolic computation pipeline:

```
Optimization Variables (Z = MX.sym("coeffs", n_vars))
    |
    v
Fourier Expressions: Q(Z,t), V(Z,t), A(Z,t)  ← MX symbolic AD
    |
    v
Regressor: W = casbe.regressor_function.map(N_s, "openmp")(Q, V, A)
RNEA:     tau = casbe.rnea_function.map(N_s, "openmp")(Q, V, A)
    |
    v
Information Matrix: J = W_b^T * W_b / N_s
D-optimal Objective: obj = -2 * sum(log(L_ii))  ← Cholesky(J + lambda*I)
    |
    v
NLP Solver: cs.nlpsol("ipopt", nlp)  ← CasADi built-in IPOPT
```

### Key Design Points

- **Symbolic differentiation**: Position, velocity, and acceleration are derived analytically from Fourier coefficients via CasADi MX automatic differentiation
- **Parallel evaluation**: The regressor and RNEA functions use CasADi's `.map(N_s, "openmp")` for parallel evaluation across all sampling points
- **D-optimal objective**: Uses Cholesky factorization of the regularized information matrix (`J + lambda*I`) for numerically stable log-determinant computation
- **Column-major layout**: All trajectory expressions are built in `(n_act, N_s)` column-major format, avoiding transposition before `map()` calls
- **Friction model**: Uses `tanh(alpha * v)` approximation — differentiable everywhere, with configurable steepness per phase

---

## Environment Requirements

Fourier excitation trajectories require these additional components:

| Component | Version | Channel | Purpose |
|-----------|---------|---------|---------|
| **CasADi** | >= 3.7.2 | conda-forge | Symbolic computation framework and NLP solver |
| **pinocchio.casadi** | >= 4.0 | conda-forge | Symbolic robot dynamics (CasADi bindings) |
| **IPOPT** | (bundled with CasADi) | — | NLP solver |
| **HSL ma57** | (optional, recommended) | — | Sparse linear solver (accelerates IPOPT) |
| **OpenMP** | (compiler feature) | — | Parallel sampling evaluation |

### Installation

```bash
# With pixi
pixi install --environment casadi

# With conda
conda install -c conda-forge "casadi>=3.7.2" "pinocchio>=4.0"
```

### Verification

Run the environment check script to verify all components:

```bash
python scripts/check_env.py
```

### Important Notes

1. Fourier trajectory optimization uses CasADi's built-in `cs.nlpsol("ipopt")`, not cyipopt
2. HSL ma57 linear solver is preferred; falls back to mumps if unavailable (with a warning)
3. OpenMP parallelization is required — CasADi from conda-forge includes it by default; self-compiled CasADi needs `-DWITH_OPENMP=ON`
4. The `tanh(alpha * v)` friction model uses `alpha=10` during optimization and `alpha=100` for identification
5. Base parameter computation (`BaseParameterComputer`) is trajectory-type agnostic — the same `idx_b` indices are used regardless of trajectory representation

---

## Comparison: Fourier vs Spline

| Aspect | Spline (default) | Fourier |
|--------|-------------------|---------|
| Parameterization | Waypoints + cubic interpolation | Fourier series coefficients |
| Variables | Joint positions at waypoints | Harmonic coefficients (66 for 6-DOF, n_harmonics=5) |
| Backend | Numerical (cyipopt) or CasADi | CasADi only |
| NLP solver | cyipopt or cs.nlpsol | cs.nlpsol |
| Differentiability | Via finite differences or CasADi Callback | Fully symbolic AD |
| Objective | Acceleration smoothness + velocity excitation | D-optimal (max det information matrix) |

---

## Troubleshooting

### `ImportError: CasADi backend not available`

**Cause**: CasADi or pinocchio.casadi is not installed.

**Fix**: Install the casadi environment:
```bash
pixi install --environment casadi
```

### `IPOPT fails to converge`

**Cause**: Initial coefficients produce trajectories that violate joint limits.

**Fix**: The solver automatically retries with reduced amplitude (0.5x) if constraints are violated. Increase `max_attempts` or reduce joint range in configuration.

### `OpenMP not available`

**Cause**: CasADi was compiled without OpenMP support.

**Fix**: Install from conda-forge (which includes OpenMP):
```bash
conda install -c conda-forge casadi
```
