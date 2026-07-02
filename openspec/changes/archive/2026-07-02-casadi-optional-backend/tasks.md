# Tasks: CasADi Optional Backend

## 1. Backend Abstraction Layer

- [x] 1.1 Create `src/figaroh/backend/` package with `__init__.py`, `base.py`
- [x] 1.2 Define `Backend` abstract base class with `build_regressor(q, v, a,
  identif_config)`, `gradient(objective_fn, x)`, `jacobian(constraints_fn, x)`
  interface methods
- [x] 1.3 Implement `NumericalBackend(Backend)` — wraps existing code from
  `tools/regressor.py` and `tools/robotipopt.py` with no logic changes
- [x] 1.4 Add `figaroh/backend` to root `__init__.py` exports

## 2. CasADi Backend Core

- [x] 2.1 Implement `CasadiBackend(Backend).__init__()` with lazy symbolic
  model initialization (`cpin.Model`, `cpin.computeJointTorqueRegressor`)
- [x] 2.2 Implement `CasadiBackend.build_regressor()` — builds CasADi `Function`
  from symbolic regressor, maps over N sample columns
- [x] 2.3 Implement `CasadiBackend.gradient()` — wraps objective in CasADi
  `Function`, uses `cs.gradient()` for analytical AD
- [x] 2.4 Implement `CasadiBackend.jacobian()` — wraps constraints in CasADi
  `Function`, uses `cs.jacobian()` for analytical AD
- [x] 2.5 Add error handling for missing `pinocchio.casadi` import with
  clear install instructions

## 3. Integration with Optimal Trajectory

- [x] 3.1 Add `backend` parameter to `BaseOptimalTrajectory.__init__()`
  (default: `"numerical"`)
- [x] 3.2 Add `backend` parameter to `BaseTrajectoryIPOPTProblem` and route
  `gradient()`/`jacobian()` calls through the backend
- [x] 3.3 Update `_stack_base_regressors()` to accept optional backend
  override for `build_regressor()` call
- [x] 3.4 Refactor `BaseTrajectoryIPOPTProblem.jacobian()` to use
  `self.backend.jacobian()` instead of per-column finite differences
- [x] 3.5 Default `BaseOptimizationProblem.gradient()` uses
  `self.backend.gradient()` when backend is available

## 4. Configuration and Dependency Management

- [x] 4.1 Add `backend` key parsing to `optimal/config.py` loader
- [x] 4.2 Add `backend` key parsing to `identification/config.py` loader
- [x] 4.3 Add `[project.optional-dependencies]` with `casadi` extra to
  `pyproject.toml`
- [x] 4.4 Add `casadi` pixi feature in `pyproject.toml` with `casadi >=3.6.7`
  and conda-forge `pinocchio` dependencies
- [x] 4.5 Update `README.md` with CasADi backend setup instructions

## 5. Testing

- [x] 5.1 Unit test: `Backend` ABC cannot be instantiated directly
- [x] 5.2 Unit test: `NumericalBackend` produces identical results to
  current code
- [x] 5.3 Unit test: `CasadiBackend.build_regressor()` matches
  `NumericalBackend.build_regressor()` within 1e-8 tolerance
- [x] 5.4 Unit test: `CasadiBackend.gradient()` matches numerical
  gradient within 1e-6 tolerance
- [x] 5.5 Unit test: `CasadiBackend.jacobian()` matches numerical
  Jacobian within 1e-6 tolerance
- [x] 5.6 Integration test: `BaseOptimalTrajectory(backend="numerical")`
  produces same result as current code
- [x] 5.7 Integration test: `BaseOptimalTrajectory(backend="casadi")`
  completes successfully with CasADi installed
- [x] 5.8 Integration test: Import error message is clear when CasADi
  not installed and `backend="casadi"` is selected
- [x] 5.9 Regression: all 212 existing tests continue to pass

## 6. Benchmark and Documentation

- [x] 6.1 Add benchmark script comparing `numerical` vs `casadi` backend
  for regressor build time, gradient time, and full solve time
- [x] 6.2 Document backend architecture in `docs/` (architecture decision
  record)
- [x] 6.3 Add docstrings to all new public classes and methods
