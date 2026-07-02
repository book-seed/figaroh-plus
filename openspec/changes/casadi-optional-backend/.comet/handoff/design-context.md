# Comet Design Handoff

- Change: casadi-optional-backend
- Phase: design
- Mode: compact
- Context hash: f233e57bfb24e7c0b7c3a81661d404758a5015788132b3409c07975c3265bf1f

Generated-by: comet-handoff.sh

OpenSpec remains the canonical capability spec. This handoff is a deterministic, source-traceable context pack, not an agent-authored summary.

## openspec/changes/casadi-optional-backend/proposal.md

- Source: openspec/changes/casadi-optional-backend/proposal.md
- Lines: 1-67
- SHA256: d4aba65f075d7b2f255410a1efe5af2e0431ddfbaa57110a3b99b60dab422adf

```md
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
```

## openspec/changes/casadi-optional-backend/design.md

- Source: openspec/changes/casadi-optional-backend/design.md
- Lines: 1-225
- SHA256: f97c64b84e409a05bc8ea642e73a3135ab82d4aa77f0a2af727d70d7236fc886

[TRUNCATED]

```md
# Design: CasADi Optional Backend

## Context

FIGAROH's computation pipeline currently has a single code path: numerical
Pinocchio (`pin`) functions called from Python, with `numdifftools` providing
numerical gradient/Jacobian approximations for IPOPT. The optimal trajectory
generation (`BaseOptimalTrajectory`) is the primary bottleneck — each IPOPT
iteration requires N+1 evaluations of an objective function that builds a
regressor matrix via a serial Python loop over samples.

Switching from PyPI `pin` to conda-forge `pinocchio` (which includes compiled
CasADi Python bindings) is required. Validation shows this switch is
transparent: all 212 existing tests pass.

## Goals / Non-Goals

**Goals:**
1. Provide a `Backend` abstraction so all computation hot paths can dispatch
   to either `numerical` or `casadi` implementations
2. CasADi path uses `pinocchio.casadi` for symbolic regressor construction
   and CasADi `gradient()`/`jacobian()` for analytical AD
3. Backend selection via config (`backend: numerical|casadi`) with
   `numerical` as default — no breaking API changes
4. CasADi is an **optional** dependency; `numerical` path works without it

**Non-Goals:**
- Replacing QR/SVD (already optimized BLAS/LAPACK)
- Replacing picos SOCP/SDP solver
- Changing `BaseIdentification` / `BaseCalibration` public API signatures
- Replacing `scipy.signal` filtering or data I/O
- CasADi code generation in the initial implementation (future optimization)

## Decisions

### D1: Backend abstracts the full solver, not just gradient/Jacobian

CasADi provides a built-in IPOPT interface via `cs.nlpsol('solver', 'ipopt', nlp)`
that takes a symbolic NLP definition and **automatically** computes gradient,
Jacobian, and Hessian via AD — no separate derivative functions needed.
This is both simpler and faster than the "CasADi AD + cyipopt" approach.

The `Backend` therefore abstracts at the **solver** level, not the derivative level:

```
                 ┌────────────────────────────────┐
                 │        Backend (ABC)            │
                 ├────────────────────────────────┤
                 │ + build_regressor(q,v,a,cfg)    │
                 │ + create_solver(nlp_def) → Solver│
                 └───────────────┬────────────────┘
                                 │
            ┌────────────────────┴────────────────────┐
            ▼                                         ▼
 ┌─────────────────────────┐           ┌─────────────────────────────┐
 │   NumericalBackend      │           │      CasadiBackend           │
 │   (current code)        │           │   (pinocchio.casadi +        │
 ├─────────────────────────┤           │    cs.nlpsol)                │
 │ regressor: for-loop     │           ├─────────────────────────────┤
 │ solver: cyipopt +       │           │ regressor: symbolic via      │
 │   nd.Gradient + nd.Jacob│           │   cpin.computeJointTorque    │
 │                         │           │   Regressor                  │
 │                         │           │ solver: cs.nlpsol('ipopt')   │
 │                         │           │   with symbolic NLP —        │
 │                         │           │   gradient, Jacobian,        │
 │                         │           │   Hessian all automatic      │
 └─────────────────────────┘           └─────────────────────────────┘
```

**Key insight**: With CasADi's `nlpsol`, we don't write `gradient()` or
`jacobian()` methods at all. We define the NLP symbolically:
```python
nlp = {'x': X_sym, 'f': obj_expr, 'g': cons_expr}
solver = cs.nlpsol('solver', 'ipopt', nlp, opts)
result = solver(x0=x0, lbg=lbg, ubg=ubg)
```
CasADi handles all derivative computation internally.

Rationale: Abstracting at the solver level is cleaner because the two
backends use fundamentally different IPOPT interfaces (cyipopt callback vs
```

Full source: openspec/changes/casadi-optional-backend/design.md

## openspec/changes/casadi-optional-backend/tasks.md

- Source: openspec/changes/casadi-optional-backend/tasks.md
- Lines: 1-74
- SHA256: 9dbe0ee534888953b48e37593cece863e120c73a4da043bdcc5d52aa1ffb4a44

```md
# Tasks: CasADi Optional Backend

## 1. Backend Abstraction Layer

- [ ] 1.1 Create `src/figaroh/backend/` package with `__init__.py`, `base.py`
- [ ] 1.2 Define `Backend` abstract base class with `build_regressor(q, v, a,
  identif_config)`, `gradient(objective_fn, x)`, `jacobian(constraints_fn, x)`
  interface methods
- [ ] 1.3 Implement `NumericalBackend(Backend)` — wraps existing code from
  `tools/regressor.py` and `tools/robotipopt.py` with no logic changes
- [ ] 1.4 Add `figaroh/backend` to root `__init__.py` exports

## 2. CasADi Backend Core

- [ ] 2.1 Implement `CasadiBackend(Backend).__init__()` with lazy symbolic
  model initialization (`cpin.Model`, `cpin.computeJointTorqueRegressor`)
- [ ] 2.2 Implement `CasadiBackend.build_regressor()` — builds CasADi `Function`
  from symbolic regressor, maps over N sample columns
- [ ] 2.3 Implement `CasadiBackend.gradient()` — wraps objective in CasADi
  `Function`, uses `cs.gradient()` for analytical AD
- [ ] 2.4 Implement `CasadiBackend.jacobian()` — wraps constraints in CasADi
  `Function`, uses `cs.jacobian()` for analytical AD
- [ ] 2.5 Add error handling for missing `pinocchio.casadi` import with
  clear install instructions

## 3. Integration with Optimal Trajectory

- [ ] 3.1 Add `backend` parameter to `BaseOptimalTrajectory.__init__()`
  (default: `"numerical"`)
- [ ] 3.2 Add `backend` parameter to `BaseTrajectoryIPOPTProblem` and route
  `gradient()`/`jacobian()` calls through the backend
- [ ] 3.3 Update `_stack_base_regressors()` to accept optional backend
  override for `build_regressor()` call
- [ ] 3.4 Refactor `BaseTrajectoryIPOPTProblem.jacobian()` to use
  `self.backend.jacobian()` instead of per-column finite differences
- [ ] 3.5 Default `BaseOptimizationProblem.gradient()` uses
  `self.backend.gradient()` when backend is available

## 4. Configuration and Dependency Management

- [ ] 4.1 Add `backend` key parsing to `optimal/config.py` loader
- [ ] 4.2 Add `backend` key parsing to `identification/config.py` loader
- [ ] 4.3 Add `[project.optional-dependencies]` with `casadi` extra to
  `pyproject.toml`
- [ ] 4.4 Add `casadi` pixi feature in `pyproject.toml` with `casadi >=3.6.7`
  and conda-forge `pinocchio` dependencies
- [ ] 4.5 Update `README.md` with CasADi backend setup instructions

## 5. Testing

- [ ] 5.1 Unit test: `Backend` ABC cannot be instantiated directly
- [ ] 5.2 Unit test: `NumericalBackend` produces identical results to
  current code
- [ ] 5.3 Unit test: `CasadiBackend.build_regressor()` matches
  `NumericalBackend.build_regressor()` within 1e-8 tolerance
- [ ] 5.4 Unit test: `CasadiBackend.gradient()` matches numerical
  gradient within 1e-6 tolerance
- [ ] 5.5 Unit test: `CasadiBackend.jacobian()` matches numerical
  Jacobian within 1e-6 tolerance
- [ ] 5.6 Integration test: `BaseOptimalTrajectory(backend="numerical")`
  produces same result as current code
- [ ] 5.7 Integration test: `BaseOptimalTrajectory(backend="casadi")`
  completes successfully with CasADi installed
- [ ] 5.8 Integration test: Import error message is clear when CasADi
  not installed and `backend="casadi"` is selected
- [ ] 5.9 Regression: all 212 existing tests continue to pass

## 6. Benchmark and Documentation

- [ ] 6.1 Add benchmark script comparing `numerical` vs `casadi` backend
  for regressor build time, gradient time, and full solve time
- [ ] 6.2 Document backend architecture in `docs/` (architecture decision
  record)
- [ ] 6.3 Add docstrings to all new public classes and methods
```

## openspec/changes/casadi-optional-backend/specs/casadi-backend/spec.md

- Source: openspec/changes/casadi-optional-backend/specs/casadi-backend/spec.md
- Lines: 1-81
- SHA256: 616b8ef5c626f6068f57ce053570393fe02e62308c4642c1af9f0f9fb2c128f9

[TRUNCATED]

```md
# CasADi Backend

## ADDED Requirements

### Requirement: Backend selection via configuration
The system SHALL support selecting the computation backend via a `backend`
key in the robot configuration YAML file, with valid values `numerical`
(default) and `casadi`.

#### Scenario: Default backend is numerical
- **WHEN** no `backend` key is present in the configuration
- **THEN** the system SHALL use the `NumericalBackend` (current behavior preserved)

#### Scenario: Explicit numerical backend
- **WHEN** `backend: numerical` is set in the configuration
- **THEN** the system SHALL use `NumericalBackend` (identical to default)

#### Scenario: CasADi backend selected
- **WHEN** `backend: casadi` is set in the configuration AND the CasADi
  optional dependency is installed
- **THEN** the system SHALL use `CasadiBackend` for regressor evaluation
  and IPOPT gradient/Jacobian computation

#### Scenario: CasADi backend selected but not installed
- **WHEN** `backend: casadi` is set but `pinocchio.casadi` cannot be imported
- **THEN** the system SHALL raise a clear `ImportError` with instructions
  for installing the optional dependency

### Requirement: CasADi backend provides analytical gradient
The `CasadiBackend.gradient()` method SHALL return the analytically
differentiated gradient of the objective function using CasADi's automatic
differentiation, producing numerically equivalent results to
`NumericalBackend.gradient()` (finite differences) within a tolerance of 1e-6.

#### Scenario: Gradient matches numerical reference
- **WHEN** both `CasadiBackend.gradient(x)` and `NumericalBackend.gradient(x)`
  are called with the same input x
- **THEN** the maximum absolute difference SHALL be less than 1e-6

#### Scenario: Gradient is faster than finite differences
- **WHEN** `CasadiBackend.gradient(x)` is benchmarked against
  `NumericalBackend.gradient(x)` for a typical robot configuration
  (6+ DOF, 24+ optimization variables)
- **THEN** the CasADi gradient SHALL execute at least 5x faster

### Requirement: CasADi backend provides analytical Jacobian
The `CasadiBackend.jacobian()` method SHALL return the analytically
differentiated Jacobian of constraint functions, eliminating the per-column
finite-difference loop used by `NumericalBackend.jacobian()`.

#### Scenario: Jacobian matches numerical reference
- **WHEN** both `CasadiBackend.jacobian(x)` and `NumericalBackend.jacobian(x)`
  are called with the same input x
- **THEN** the maximum absolute difference SHALL be less than 1e-6

### Requirement: Backend-agnostic BaseOptimalTrajectory
`BaseOptimalTrajectory` SHALL accept an optional `backend` parameter
(`Literal["numerical", "casadi"]` or `Backend` instance). When omitted,
it SHALL default to `"numerical"`. All existing subclasses SHALL continue
to work without modification.

#### Scenario: Existing code unchanged
- **WHEN** `BaseOptimalTrajectory(robot, active_joints, config_file)` is
  called without `backend` argument
- **THEN** the behavior SHALL be identical to the pre-change implementation

#### Scenario: CasADi backend passed programmatically
- **WHEN** `BaseOptimalTrajectory(robot, active_joints, config_file,
  backend="casadi")` is called
- **THEN** `solve()` SHALL use `CasadiBackend` for all IPOPT derivative
  computations

### Requirement: CasADi is an optional dependency
CasADi and conda-forge `pinocchio` (with CasADi bindings) SHALL be declared
as optional dependencies. The `numerical` backend SHALL function without
CasADi installed.

#### Scenario: Numerical path works without CasADi
- **WHEN** `casadi` is not installed in the Python environment
- **THEN** all existing functionality (identification, calibration, optimal
```

Full source: openspec/changes/casadi-optional-backend/specs/casadi-backend/spec.md

