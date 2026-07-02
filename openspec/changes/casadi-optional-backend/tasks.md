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
