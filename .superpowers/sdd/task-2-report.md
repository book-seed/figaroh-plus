# Task 2 Report: NumericalBackend Full Implementation

## Status: COMPLETE

## Commit
- **Hash**: `ebd5c45`
- **Message**: `feat(backend): implement NumericalBackend with full wrappers`

## RED / GREEN Test Evidence

### RED Phase (before implementation)
```
tests/unit/test_backend.py::TestNumericalBackend::test_name PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_gradient_of_quadratic FAILED  (NotImplementedError)
tests/unit/test_backend.py::TestNumericalBackend::test_jacobian_of_linear_system FAILED  (NotImplementedError)
tests/unit/test_backend.py::TestNumericalBackend::test_build_regressor_delegates_to_build_regressor_basic FAILED  (NotImplementedError)
tests/unit/test_backend.py::TestNumericalBackend::test_create_solver_returns_callable FAILED  (NotImplementedError)
tests/unit/test_backend.py::TestNumericalBackend::test_create_solver_passes_custom_bounds FAILED  (NotImplementedError)
```
5 of 6 new tests failed as expected (all `NotImplementedError`). The `test_name` passed because the stub already implemented the `name` property.

### GREEN Phase (after implementation)
```
tests/unit/test_backend.py::TestBackendABC::test_backend_abc_cannot_be_instantiated PASSED
tests/unit/test_backend.py::TestBackendABC::test_concrete_backend_must_implement_abstract_methods PASSED
tests/unit/test_backend.py::TestBackendABC::test_concrete_backend_with_all_methods PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_numerical PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_default_is_numerical PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_passthrough PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_invalid_string PASSED
tests/unit/test_backend.py::TestCreateBackendFactory::test_create_backend_casadi_without_deps PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_name PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_gradient_of_quadratic PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_jacobian_of_linear_system PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_build_regressor_delegates_to_build_regressor_basic PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_create_solver_returns_callable PASSED
tests/unit/test_backend.py::TestNumericalBackend::test_create_solver_passes_custom_bounds PASSED
```
14/14 passed.

### Full Regression Suite
```
226 passed, 2 skipped, 2 warnings in 1.97s
```
Zero regressions (was 220 passed + 6 new = 226, 2 skipped remain unchanged).

## Changed Files

| File | Action | Lines |
|------|--------|-------|
| `src/figaroh/backend/numerical.py` | Modified (stub -> full) | +103/-11 |
| `tests/unit/test_backend.py` | Modified (appended tests) | +113/-0 |

## Implementation Summary

- **`__init__(self, robot=None)`**: Stores robot for use in `build_regressor`.
- **`build_regressor(q, v, a, identif_config)`**: Delegates to `figaroh.tools.regressor.build_regressor_basic` with stored robot.
- **`gradient(objective_fn, x)`**: Delegates to `numdifftools.Gradient`.
- **`jacobian(constraints_fn, x)`**: Delegates to `numdifftools.Jacobian`.
- **`create_solver(nlp_def, opts) -> Callable`**: Creates `cyipopt.Problem` lazily inside a closure. The returned solver accepts `(x0, lbg, ubg)` overrides and returns a dict with `"x"` and `"info"` keys, compatible with the `Backend` ABC contract.
- **`name`**: Returns `"numerical"` (unchanged from stub).

All dependencies (`build_regressor_basic`, `numdifftools`, `cyipopt`) are imported at module level.

## Concerns

1. **Robot injection**: The `create_backend("numerical", robot=robot)` factory in `base.py` does not forward `robot` to `NumericalBackend(**kwargs)` — it binds `robot` to its own parameter. Users who need `build_regressor` must construct `NumericalBackend(robot=robot)` directly, or the factory must be updated. This affects `base.py` which is outside the Task 2 scope.
2. **`build_regressor_basic` requires a real pinocchio robot**: Mocking was unavoidable in tests. Integration tests will need a proper robot fixture.
3. **cyipopt module-level import**: Makes the numerical backend always import cyipopt (and its IPOPT dependency), even if only the `name` property or `gradient`/`jacobian` methods are used. Future work could lazy-import cyipopt and adjust the test patch strategy accordingly.
