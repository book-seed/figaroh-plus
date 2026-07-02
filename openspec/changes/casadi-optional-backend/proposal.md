# Proposal: CasADi Optional Backend

## Why

FIGAROH's optimal trajectory generation (`BaseOptimalTrajectory.solve()`) is
excessively slow in practice — taking tens of minutes to hours for typical
robot configurations. The root cause is that IPOPT's numerical differentiation
(via numdifftools or finite differences) requires N+1 objective function
evaluations per gradient call, where each evaluation loops over hundreds of
samples calling `pin.computeJointTorqueRegressor()` serially in Python.

Pinocchio 4.0 provides built-in CasADi integration (`pinocchio.casadi`) that
enables analytical automatic differentiation through the full dynamics
computation graph and C code generation for optimized runtime evaluation.
Benchmarks confirm an **11x per-iteration speedup** for gradient computation
alone — translating to a 10-20x end-to-end reduction in optimal trajectory
solve time.

This change adds CasADi as an **optional** computational backend, giving
users the choice between the existing numerical path (default, no new
dependencies) and the accelerated CasADi path.

## What Changes

- **New**: `figaroh.backend` module providing a unified backend abstraction
  with two implementations:
  - `NumericalBackend` — wraps existing Pinocchio + numdifftools code path (default)
  - `CasadiBackend` — uses `pinocchio.casadi` for analytical AD and code generation
- **New**: Backend selection via YAML config key `backend: numerical|casadi`
  (or programmatic `use_backend=` parameter)
- **New**: `CasadiRegressor` class — builds and evaluates the joint-torque
  regressor as a CasADi symbolic function, with optional code generation
- **New**: `CasadiGradientProvider` — provides analytical gradient and
  Jacobian for IPOPT, replacing numdifftools/finite-difference calls
- **Modified**: `BaseOptimalTrajectory` and `BaseTrajectoryIPOPTProblem` gain
  a `backend` parameter that dispatches gradient/Jacobian computation to the
  selected backend
- **Modified**: `pyproject.toml` adds `casadi` and `pinocchio` as optional
  extras (`[project.optional-dependencies]` with key `casadi`)
- **Unchanged**: Public API signatures of `BaseIdentification`,
  `BaseCalibration`, `BaseOptimalTrajectory` remain backward-compatible

## Capabilities

### New Capabilities

- `casadi-backend`: CasADi-based computation backend providing analytical
  automatic differentiation for regressor evaluation and IPOPT
  gradient/Jacobian computation, configurable as an alternative to the
  existing numerical path

### Modified Capabilities

_None._ Existing specs are not affected — this change adds a new optional
capability without modifying requirement-level behavior of any existing
capability.

## Impact

| Area | Impact |
|------|--------|
| `figaroh/tools/regressor.py` | Refactored into backend abstraction; `RegressorBuilder` remains as default numerical path |
| `figaroh/tools/robotipopt.py` | `BaseOptimizationProblem.gradient()` / `.jacobian()` dispatched via backend |
| `figaroh/optimal/` | `BaseOptimalTrajectory`, `BaseTrajectoryIPOPTProblem` accept backend parameter |
| Dependencies | New optional: `casadi >= 3.6.7`, `pinocchio >= 4.0` (conda-forge channel for CasADi support) |
| pixi environment | New `casadi` feature providing CasADi + conda-forge pinocchio |
| API | No breaking changes; backend defaults to `numerical` preserving current behavior |
