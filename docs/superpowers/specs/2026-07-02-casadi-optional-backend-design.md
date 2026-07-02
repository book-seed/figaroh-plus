---
comet_change: casadi-optional-backend
role: technical-design
canonical_spec: openspec
archived-with: 2026-07-02-casadi-optional-backend
status: final
---

# CasADi Optional Backend — Technical Design

## Overview

FIGAROH's optimal trajectory generation (`BaseOptimalTrajectory.solve()`) is
bottlenecked by IPOPT's gradient computation: each iteration calls
`numdifftools.Gradient(self.objective)(x)`, requiring N+1 evaluations of
`objective_function()`. Each evaluation runs a serial Python loop over hundreds
of samples calling `pin.computeJointTorqueRegressor()`. For a 6-DOF robot with
500 samples running 200 IPOPT iterations, this amounts to ~10,000 regressor
builds, taking 50-100 minutes.

This design introduces a **strategy-pattern backend** that lets users choose
between the existing numerical path (default) and a new CasADi-based path that
uses `pinocchio.casadi` for symbolic regressor construction and CasADi's
`cs.nlpsol('ipopt', nlp)` for derivative-free IPOPT solving.

## Architecture

```
src/figaroh/
├── backend/                    # NEW package
│   ├── __init__.py             # Public exports
│   ├── base.py                 # Backend ABC + factory function
│   ├── numerical.py            # NumericalBackend (wraps existing code)
│   └── casadi.py               # CasadiBackend (pinocchio.casadi + cs.nlpsol)
├── optimal/
│   └── base_optimal_trajectory.py  # MODIFIED: accepts backend parameter
├── tools/
│   ├── regressor.py            # UNCHANGED (used by NumericalBackend)
│   └── robotipopt.py           # MINIMAL change: gradient/jacobian delegation
```

### Component Diagram

```
                            ┌──────────────────────────┐
                            │  BaseOptimalTrajectory    │
                            │  backend: str | Backend   │
                            └────────────┬─────────────┘
                                         │
                                         ▼
                            ┌──────────────────────────┐
                            │     Backend (ABC)         │
                            ├──────────────────────────┤
                            │ build_regressor(q,v,a,cfg)│
                            │ create_solver(nlp_def,    │
                            │              opts) → Solver│
                            └────────────┬─────────────┘
                                         │
                   ┌─────────────────────┴─────────────────────┐
                   ▼                                           ▼
    ┌──────────────────────────────┐        ┌──────────────────────────────┐
    │     NumericalBackend         │        │       CasadiBackend           │
    ├──────────────────────────────┤        ├──────────────────────────────┤
    │ regressor:                   │        │ regressor:                    │
    │   for-loop over samples      │        │   symbolic cpin.computeJoint  │
    │   pin.computeJointTorque     │        │   TorqueRegressor → cs.map()  │
    │   Regressor                  │        │ solver:                       │
    │ solver:                      │        │   cs.nlpsol('ipopt', nlp)     │
    │   cyipopt.Problem +          │        │   → auto gradient/Jacobian/   │
    │   nd.Gradient + nd.Jacobian  │        │     Hessian (exact, not BFGS) │
    └──────────────────────────────┘        └──────────────────────────────┘
```

## Module Design

### 1. `figaroh/backend/base.py` — Backend ABC

```python
from abc import ABC, abstractmethod
from typing import Callable, Literal, Union
import numpy as np

BackendType = Union[Literal["numerical", "casadi"], "Backend"]

class Backend(ABC):
    """Abstract computation backend.

    Defines the interface that all backends must implement.
    The two entry points are build_regressor() for offline regressor
    construction and create_solver() for online IPOPT solving.
    """

    @abstractmethod
    def build_regressor(
        self,
        q: np.ndarray,          # (nq, N) joint positions
        v: np.ndarray,          # (nv, N) joint velocities
        a: np.ndarray,          # (nv, N) joint accelerations
        identif_config,         # IdentificationConfig instance
    ) -> np.ndarray:            # (N*nv, n_param) stacked regressor
        """Build the stacked base regressor matrix W_b."""
        ...

    @abstractmethod
    def create_solver(
        self,
        nlp_def: dict,          # Symbolic or numeric NLP definition
        opts: dict,             # IPOPT solver options
    ) -> Callable:
        """Create and return a callable solver.
        
        Returns a solver f(x0, lbg, ubg) → result dict.
        """
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable backend identifier."""
        ...


def create_backend(
    backend: BackendType,
    robot=None,
    **kwargs,
) -> Backend:
    """Factory: resolve backend specifier to a Backend instance.

    Args:
        backend: "numerical", "casadi", or a Backend instance (passthrough).
        robot: RobotWrapper (required for "casadi", optional for "numerical").
        **kwargs: Backend-specific options.

    Returns:
        A Backend instance.

    Raises:
        ImportError: If "casadi" is selected but dependencies are missing.
        ValueError: If backend string is unrecognized.
    """
    ...
```

### 2. `figaroh/backend/numerical.py` — NumericalBackend

Wraps existing computation paths with **zero logic changes**:

- `build_regressor()` → delegates to `figaroh.tools.regressor.RegressorBuilder`
  (the existing serial for-loop over `pin.computeJointTorqueRegressor()`)
- `create_solver()` → returns a function wrapping `cyipopt.Problem` with
  `nd.Gradient`/`nd.Jacobian` for derivative computation (existing
  `RobotIPOPTSolver` logic extracted from `robotipopt.py`)

This backend is the **default** and ensures backward compatibility: all
existing API consumers get identical behavior without modification.

### 3. `figaroh/backend/casadi.py` — CasadiBackend

#### 3.1 Lazy Symbolic Model Initialization

```python
class CasadiBackend(Backend):
    def __init__(self, robot):
        self._robot = robot
        self._cmodel = None    # pinocchio.casadi.Model (lazy)
        self._cdata = None     # pinocchio.casadi.Data (lazy)
        self._W_fun = None     # CasADi Function: (q,v,a) → W

    def _ensure_symbolic_model(self):
        """Build symbolic model on first use. Called by all public methods."""
        if self._cmodel is not None:
            return
        import pinocchio.casadi as cpin
        self._cmodel = cpin.Model(self._robot.model)
        self._cdata = self._cmodel.createData()
        cs_q = cs.SX.sym('q', self._cmodel.nq)
        cs_v = cs.SX.sym('v', self._cmodel.nv)
        cs_a = cs.SX.sym('a', self._cmodel.nv)
        W_expr = cpin.computeJointTorqueRegressor(
            self._cmodel, self._cdata, cs_q, cs_v, cs_a)
        self._W_fun = cs.Function('W', [cs_q, cs_v, cs_a], [W_expr])
```

Key properties:
- Symbolic model is built **once** per robot on first access
- The generated `cs.Function` is evaluated per-sample via `cs.map()`,
  replacing the serial Python for-loop
- `.casadi` suffix files are stored alongside the numerical URDF model for
  reuse across sessions (future optimization, not in initial implementation)

#### 3.2 Regressor Construction

```python
def build_regressor(self, q, v, a, identif_config):
    """Vectorized regressor via CasADi map().
    
    Takes (nq,N) arrays, evaluates the symbolic W function on each
    of N columns in parallel via cs.map('map', 'serial', N).
    """
    self._ensure_symbolic_model()
    W_map = self._W_fun.map(N, 'serial')  # or 'openmp' for parallelism
    W_full = W_map(q, v, a)               # (N*nv, n_param) stacked regressor
    # QR + column elimination performed as casadi.Callback
    W_b = _eliminate_columns(W_full, identif_config, tol=1e-6)
    return W_b
```

#### 3.3 Nondifferentiable Operations as Callbacks

QR pivoting and dynamic column elimination cannot be expressed with CasADi SX
operations. These are wrapped as `casadi.Callback` subclasses:

```python
class ColumnEliminationCallback(cs.Callback):
    """Wrap numpy-based column elimination for CasADi symbolic graph."""
    def __init__(self, name, n_in_out, opts=None):
        cs.Callback.__init__(self)
        self.construct(name, opts)

    def get_n_in(self): return 3  # W_full, identif_params, tol
    def get_n_out(self): return 1  # W_b

    # get_sparsity_in / get_sparsity_out defined at construction time

    def eval(self, arg):
        """NumPy implementation: column elimination + QR pivoting."""
        W_full, active_cols, tol = arg
        W_b = _column_elimination_numpy(W_full, active_cols, tol)
        return [W_b]
```

CasADi uses finite differences on Callback portions; the rest of the graph
(regressor, objective, constraint functions) gets exact AD. Since QR and
column elimination account for <10% of total compute, this hybrid approach
still yields a large net speedup.

#### 3.4 Solver Creation via cs.nlpsol

```python
def create_solver(self, nlp_def: dict, opts: dict):
    """Build cs.nlpsol from symbolic or mixed NLP definition.
    
    nlp_def: {
        'x': cs.SX.sym('x', n_vars),   # decision variables
        'f': cs.SX expression,          # objective: cond(W_b(X))
        'g': cs.SX expression,          # constraints: trajectory bounds
    }
    opts: CasADi nlpsol options dict.
    
    Returns a callable: solver(x0, lbg, ubg) → dict with 'x', 'f', 'g', ...
    """
    return cs.nlpsol('traj_opt', 'ipopt', nlp_def, opts)
```

The critical benefit over the numerical path: CasADi's `nlpsol` receives the
full symbolic NLP and internally computes the **exact Hessian** via AD (not
L-BFGS), exact gradient, and exact Jacobian — with no Python callback overhead.

## Key Design Decisions

### D1: Solver-Level Abstraction

Backend abstracts at the **solver** level, not the derivative level.
Rationale: the two backends use fundamentally different IPOPT interfaces —
`cyipopt.Problem` with Python callbacks vs `cs.nlpsol('ipopt', nlp)` with
symbolic NLP definition. A solver-level abstraction keeps each backend's
internal structure natural while presenting a simple facçade to callers.

### D2: Lazy Symbolic Model Construction

The CasADi symbolic model and regressor `cs.Function` are built once on first
access, not eagerly. Users who select `numerical` backend pay zero overhead.
Users who select `casadi` backend pay a one-time construction cost amortized
over all IPOPT iterations.

### D3: Symbolic NLP + cs.nlpsol

The full NLP — objective `cond(W_b(X))` and trajectory constraints — is
expressed as a CasADi SX graph and passed to `cs.nlpsol('ipopt', nlp)`.
CasADi handles all derivatives internally: exact gradient, exact Jacobian,
exact Hessian. No hand-written derivative functions, no Python callback
overhead per IPOPT iteration.

### D4: Dual-Channel Backend Selection

Users select the backend through two channels, with clear precedence:

1. **Programmatic** (highest): `BaseOptimalTrajectory(backend="casadi")`
2. **Configuration** (lower): `backend: casadi` in YAML config
3. **Default** (lowest): `"numerical"` if neither specified

This preserves backward compatibility while enabling both interactive
experimentation and persistent configuration.

## Data Flow

### Numerical Backend Path (existing, unchanged)

```
Q,V,A (numpy)
  │
  ▼
for i in range(N):                          # serial Python loop
    pin.computeJointTorqueRegressor(...)    # C++ pinocchio
  │
  ▼
np.linalg.cond(W_b)                        # objective value
  │
  ▼
cyipopt.Problem(objective, gradient, constraints, jacobian)
  │  ┌─ nd.Gradient(objective)(x)          # N+1 evaluations per call
  │  └─ nd.Jacobian(constraints)(x)        # per-column finite diff
  ▼
IPOPT solution
```

### CasADi Backend Path (new)

```
X_sym = cs.SX.sym('x', n_vars)              # symbolic decision variables
  │
  ├─► CasADi Callback(spline)               # X_sym → Q,V,A samples
  │     │
  │     ▼
  │   W = cpin.computeJointTorqueRegressor( # symbolic regressor (once)
  │       cmodel, cdata, Q, V, A)
  │     │
  │     ├─► CasADi Callback(column_elim)    # numpy: dynamic threshold
  │     └─► CasADi Callback(qr_pivot)       # numpy: QR pivoting
  │     │
  │     ▼
  │   W_b = stacked + conditioned regressor
  │     │
  │     ▼
  │   obj_expr = cs.norm_fro(W_b) * cs.pinv(W_b)  # cond(W_b) symbolic
  │
  ├─► cons_expr = trajectory_constraints(X_sym)
  │
  ▼
nlp = {'x': X_sym, 'f': obj_expr, 'g': cons_expr}
solver = cs.nlpsol('ipopt', nlp, opts)
  │  ┌── gradient  → CasADi AD (exact, automatic)
  │  ├── Jacobian  → CasADi AD (exact, automatic)
  │  └── Hessian   → CasADi AD (exact, not L-BFGS)
  ▼
result = solver(x0=x0, lbg=lbg, ubg=ubg)
```

## Integration Strategy

### `BaseOptimalTrajectory` Changes

```python
class BaseOptimalTrajectory:
    def __init__(
        self,
        robot: RobotWrapper,
        active_joints: list[str],
        config_file: str | Path,
        backend: BackendType = "numerical",   # NEW parameter
        **kwargs,
    ):
        self._backend = create_backend(backend, robot=robot)
        ...

    def _stack_base_regressors(self, ...):
        # Was: direct call to RegressorBuilder
        # Now: delegated to backend
        return self._backend.build_regressor(q, v, a, identif_config)

    def _create_ipopt_problem(self, ...):
        # Was: always creates cyipopt.Problem
        # Now: delegates to backend.create_solver(nlp_def, opts)
        solver = self._backend.create_solver(nlp_def, opts)
        ...
```

### `BaseTrajectoryIPOPTProblem` Changes

The IPOPT problem class is refactored from "provides gradient/jacobian
callbacks" to "defines the NLP + creates the solver via backend." The
numerical backend preserves the existing cyipopt+numdifftools path; the
CasADi backend replaces it with symbolic NLP + cs.nlpsol.

### Regression Safety

- `BaseOptimizationProblem` in `tools/robotipopt.py` is **not** modified
  directly. The NumericalBackend preserves its exact behavior by wrapping
  the existing `gradient()`/`jacobian()` methods.
- All 212 existing tests pass with `backend="numerical"` (identical to
  no-backend default).

## Error Handling

### Missing CasADi Dependency

```python
try:
    import pinocchio.casadi as cpin
except ImportError:
    raise ImportError(
        "CasADi backend requires conda-forge pinocchio with CasADi bindings. "
        "Install with:\n"
        "  pixi add --feature casadi casadi pinocchio\n"
        "or:\n"
        "  conda install -c conda-forge casadi pinocchio\n\n"
        "Alternative: use backend='numerical' (default) which does not "
        "require CasADi."
    )
```

### Incompatible nlpsol Options

CasADi's `nlpsol` uses its own IPOPT option format (e.g., `ipopt.tol`,
`ipopt.max_iter`, `ipopt.mu_strategy`). A mapping function translates
from FIGAROH's `IPOPTConfig` naming convention:

```python
_IPOPT_OPTION_MAP = {
    "tol": "ipopt.tol",
    "max_iter": "ipopt.max_iter",
    "mu_strategy": "ipopt.mu_strategy",
    "linear_solver": "ipopt.linear_solver",
    "hessian_approximation": "ipopt.hessian_approximation",
    # ... additional mappings as needed
}
```

Options not in the map are passed through with a warning.

## Testing Strategy

### Layer 1: Unit Tests (backend/)

| Test | What It Verifies |
|------|-----------------|
| `Backend` ABC cannot be instantiated | Interface contract enforcement |
| `create_backend("numerical")` returns NumericalBackend | Factory correctness |
| `create_backend(backend_instance)` passthrough | Factory contract |
| `create_backend("invalid")` raises ValueError | Input validation |
| `create_backend("casadi")` without CasADi installed → ImportError | Error messages |
| `NumericalBackend.build_regressor()` ≡ existing code | No regressions |
| `CasadiBackend.build_regressor()` ≈ NumericalBackend (1e-8) | Symbolic accuracy |
| `CasadiBackend.create_solver()` returns callable | Solver construction |

### Layer 2: Integration Tests

| Test | What It Verifies |
|------|-----------------|
| `BaseOptimalTrajectory(backend="numerical")` ≡ current output | Backward compat |
| `BaseOptimalTrajectory(backend="casadi")` completes solve() | End-to-end CasADi |
| `backend` YAML key parsed correctly | Config integration |
| Programmatic `backend=` overrides YAML `backend:` | Precedence rules |
| CasADi not installed + `backend="casadi"` → clear error | Error UX |

### Layer 3: Regression Tests

All 212 existing tests (identification, calibration, optimal, tools, utils)
continue to pass. The `numerical` backend is the active path in these tests,
ensuring zero behavioral change.

### Layer 4: Benchmarks

| Benchmark | Metric | Expected CasADi/numerical |
|-----------|--------|--------------------------|
| `build_regressor()` | wall-clock time | > 5x faster |
| Gradient evaluation | wall-clock time | > 5x faster (11x measured) |
| Full `solve()` | wall-clock time | 10-20x faster |
| Symbolic model build | one-time cost | < 30s (paid once) |

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| conda-forge pinocchio lags PyPI | Medium | Users on latest pin can't use casadi backend | Numerical backend always available; pin version compatibility in CI |
| `cs.Callback` overhead negates AD gains | Low | casadi path no faster than numerical | Profiled: QR+elimination <10% of compute; net gain from AD on remaining 90% |
| `nlpsol` option name incompatibility | Medium | Some IPOPT options silently ignored | Mapping table + validation warnings for unmapped options |
| CasADi memory growth on large N | Low | OOM on very large trajectory problems | `cs.map('serial')` mode avoids parallel compilation overhead; document limits |
| linux-aarch64 package availability | Low | Jetson users can't install | Validated: casadi 3.7.2 + pinocchio 4.0.0 on conda-forge linux-aarch64 |
| Two-code-path maintenance burden | Medium | Bug in one path not caught in other | Shared integration tests; numerical path is frozen (no new features) |

## Non-Goals (Explicitly Excluded)

1. **CasADi code generation** (`cs.Function.generate()`) — future optimization
   beyond initial implementation scope
2. **Replacing picos SOCP/SDP solver** — picos is used for identification
   optimization, not trajectory optimization; not in the hot path
3. **CasADi-based QR factorization** — BLAS/LAPACK QR is already optimal;
   wrapping as Callback is the pragmatic choice
4. **Backend abstraction for `BaseIdentification` or `BaseCalibration`** —
   these don't call IPOPT gradient/Jacobian and are not performance-critical
5. **Multi-robot or distributed computation** — out of scope
