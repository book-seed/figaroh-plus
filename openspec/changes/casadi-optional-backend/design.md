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
CasADi symbolic NLP). The calling code (`BaseOptimalTrajectory`) only
needs to build a problem definition and call `backend.create_solver()`.

**Alternatives considered:**
- *CasADi AD for gradient + cyipopt for solve*: Rejected — CasADi's built-in
  IPOPT wrapper provides exact Hessian (not L-BFGS approximation), eliminates
  Python callback overhead for derivatives, and is simpler to implement.
- *If/else branches inline*: Rejected — same reasons as before.

### D2: CasADi model is built lazily, once per robot

```python
class CasadiBackend(Backend):
    def __init__(self, robot):
        self._robot = robot
        self._cmodel = None   # Built on first use
        self._cdata = None
        self._W_fun = None    # Cached CasADi regressor function

    def _ensure_symbolic_model(self):
        if self._cmodel is None:
            self._cmodel = cpin.Model(self._robot.model)
            self._cdata = self._cmodel.createData()
            cs_q = cs.SX.sym('q', self._cmodel.nq)
            cs_v = cs.SX.sym('v', self._cmodel.nv)
            cs_a = cs.SX.sym('a', self._cmodel.nv)
            W_expr = cpin.computeJointTorqueRegressor(
                self._cmodel, self._cdata, cs_q, cs_v, cs_a)
            self._W_fun = cs.Function('W', [cs_q, cs_v, cs_a], [W_expr])
```

Rationale: Symbolic model construction is expensive but done once. The
generated CasADi `Function` is then evaluated for each sample at near-C speed.
Lazy initialization means users who never select `casadi` backend pay zero
cost.

### D3: Symbolic NLP is built per problem instance, solver created once

The objective function in optimal trajectory is `cond(W_b(X))` where X are
waypoint optimization variables and `W_b` involves: regressor (symbolic via
`pinocchio.casadi`) → column elimination (numpy, wrapped as CasADi
`Callback`) → QR (numpy, wrapped as `Callback`) → stacking.

The full symbolic NLP `{'x': X_sym, 'f': obj_expr, 'g': cons_expr}` is
built at problem-creation time. CasADi's `nlpsol('ipopt', nlp)` then
receives this definition and handles **all** derivatives internally:

```python
class CasadiBackend(Backend):
    def create_solver(self, nlp_def: dict, opts: dict):
        """Build CasADi nlpsol from symbolic NLP definition.
        
        nlp_def contains:
          - 'x': SX.sym('x', n_vars)              # decision variables
          - 'f': SX expression for objective      # cond(W_b(X))
          - 'g': SX expression for constraints    # trajectory constraints
        Returns a callable solver.
        """
        return cs.nlpsol('traj_opt', 'ipopt', nlp_def, opts)
    
    def build_regressor(self, q, v, a, identif_config):
        """Vectorized regressor via CasADi map()."""
        ...
```

For operations that can't be directly expressed as CasADi SX (QR pivoting,
column elimination based on dynamic thresholds), we wrap the numpy
implementation as a `casadi.Callback` so it participates in the symbolic
graph. CasADi uses finite differences on these callbacks internally while
using exact AD for the rest of the graph — still faster than the fully
numerical path.

**Why this is better than hand-crafted gradient/Jacobian:**
- CasADi's AD computes the **exact Hessian** (not L-BFGS approximation),
  dramatically improving IPOPT convergence
- No Python callback overhead per derivative evaluation
- Simpler code: we define the NLP, CasADi handles the rest

### D4: Configuration and dependency management

```yaml
# config/robot_config.yaml (new key)
backend: numerical  # or casadi
```

```toml
# pyproject.toml
[project.optional-dependencies]
casadi = ["casadi>=3.6.7"]

# Conda channel required for CasADi-enabled pinocchio
# Users install: pixi add --feature casadi casadi pinocchio
# (conda-forge pinocchio replaces PyPI pin when both present)
```

The `casadi` backend requires both `casadi` and conda-forge `pinocchio`.
If `import pinocchio.casadi` fails, the CasadiBackend raises a clear
error message telling users to install the optional dependency.

## Architecture Overview

```
User config: backend=numerical|casadi
       │
       ▼
BaseOptimalTrajectory.__init__(..., backend="numerical")
       │
       ├── backend="numerical"
       │   └── NumericalBackend
       │       ├── build_regressor  → pin.computeJointTorqueRegressor (loop)
       │       └── create_solver    → cyipopt.Problem + nd.Gradient/nd.Jacob
       │
       └── backend="casadi"
           └── CasadiBackend
               ├── build_regressor  → CasADi map(pinocchio.casadi regressor)
               └── create_solver    → cs.nlpsol('ipopt', symbolic_nlp)
                   ├── gradient     → CasADi AD (automatic)
                   ├── Jacobian     → CasADi AD (automatic)
                   └── Hessian      → CasADi AD (exact, not L-BFGS)
```

## Risks / Trade-offs

- **[Risk] conda-forge pinocchio may lag behind PyPI pin releases**
  → Mitigation: Pin to compatible versions; test both in CI.
  → If conda-forge lag becomes a problem, users can stay on numerical backend.

- **[Risk] `casadi.Callback` for non-differentiable ops (QR pivoting) adds overhead**
  → Mitigation: Keep QR decomposition on the numpy path; only wrap what CasADi
    can differentiate natively. CasADi uses finite differences on Callback ops,
    but the rest of the graph benefits from exact AD — net improvement over
    fully numerical path.

- **[Risk] CasADi nlpsol vs cyipopt option compatibility**
  → Mitigation: Map the existing `IPOPTConfig` options to CasADi's solver
    option dict format. Validate that key options (tol, max_iter, mu_strategy)
    are correctly passed through.

- **[Risk] Linux aarch64 conda-forge package availability**
  → Mitigation: Validated — both `casadi` 3.7.2 and `pinocchio` 4.0.0 are
    available on conda-forge linux-aarch64.

- **[Trade-off] Two code paths to maintain**
  → Accepted: The numerical path is the existing code, essentially frozen.
    New features target the backend interface, not specific implementations.
