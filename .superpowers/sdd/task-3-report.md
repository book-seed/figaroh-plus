# Task 3 Report: CasadiBackend Implementation

## Status
**COMPLETED** -- all 6 CasadiBackend tests pass, full regression (232 tests) green.

## Commit
```
249bd2c feat(backend): add CasadiBackend with symbolic regressor and cs.nlpsol
```

## RED / GREEN Test Evidence

### RED phase (before casadi.py exists)
All 6 tests failed with `ModuleNotFoundError: No module named 'figaroh.backend.casadi'`:
```
tests/unit/test_backend.py::TestCasadiBackend::test_import_error_without_casadi ERROR
tests/unit/test_backend.py::TestCasadiBackend::test_lazy_initialization ERROR
tests/unit/test_backend.py::TestCasadiBackend::test_lazy_initialization_triggers_on_call ERROR
tests/unit/test_backend.py::TestCasadiBackend::test_name ERROR
tests/unit/test_backend.py::TestCasadiBackend::test_create_solver_returns_callable ERROR
tests/unit/test_backend.py::TestCasadiBackend::test_column_elimination_callback ERROR
```

### GREEN phase (after implementation)
All 6 tests pass:
```
tests/unit/test_backend.py::TestCasadiBackend::test_import_error_without_casadi PASSED
tests/unit/test_backend.py::TestCasadiBackend::test_lazy_initialization PASSED
tests/unit/test_backend.py::TestCasadiBackend::test_lazy_initialization_triggers_on_call PASSED
tests/unit/test_backend.py::TestCasadiBackend::test_name PASSED
tests/unit/test_backend.py::TestCasadiBackend::test_create_solver_returns_callable PASSED
tests/unit/test_backend.py::TestCasadiBackend::test_column_elimination_callback PASSED
```

Full regression: **232 passed, 2 skipped** (2 pre-existing warnings, no regressions).

## Change Files
- `src/figaroh/backend/casadi.py` (created, +517 lines)
- `tests/unit/test_backend.py` (modified, +122 lines)

## Implementation Details

### CasadiBackend (`src/figaroh/backend/casadi.py`)
- **Lazy dual-import**: `cpin` (pinocchio.casadi) and `cs` (casadi) are imported independently on first use via `_lazy_import()`. This allows tests to patch one module while using the real version of the other.
- **`_ensure_symbolic_model()`**: Called lazily on first method access. Builds `cpin.Model`, creates symbolic SX variables, constructs `cs.Function` for `computeJointTorqueRegressor`.
- **`build_regressor()`**: Normalizes input to `(nq/nv, N)` format, uses `cs.Function.map(N, 'serial')` for vectorised evaluation, reshapes to `(N*nv, n_param)`, performs column elimination by L2-norm thresholding.
- **`gradient()` / `jacobian()`**: Uses `cs.SX` symbolic conversion of the callable function, then `cs.gradient`/`cs.jacobian` for analytical derivatives.
- **`create_solver()`**: Maps IPOPT options via `_map_ipopt_options()`, creates `cs.nlpsol('traj_opt', 'ipopt', nlp)` solver, returns callable with signature `(x0, lbg, ubg) -> dict`.

### ColumnEliminationCallback
- Standalone numpy class -- does NOT inherit from `cs.Callback`, avoiding CasADi dependency at class-definition time.
- Provides `get_n_in()`, `get_n_out()`, `eval(arg)` methods.
- For full CasADi symbolic integration, wrap with `cs.Callback` subclass adapter.

### IPOPT Option Mapper
- `_map_ipopt_options()`: Handles `bytes` keys from `IPOPTConfig`, maps known keys to `"ipopt.*"` string format, passes unknown keys with a warning.

### Test Adaptation
- `test_create_backend_casadi_without_deps` updated: CasadiBackend's lazy-loading means `create_backend('casadi')` now succeeds without deps. The error only surfaces on method call.

## Concerns
1. **Gradient/Jacobian limitation**: `gradient()` and `jacobian()` call `objective_fn(np.array(cs_x).reshape(x.shape))`, which passes a numpy array of `cs.SX` elements to the callable. This only works if the callable uses operations compatible with CasADi symbolic types. For arbitrary Python functions (e.g., those calling `float()`), this will fail. In-practice use should prefer `create_solver()` with symbolic NLP definitions.
2. **ColumnEliminationCallback**: Not a true `cs.Callback` subclass (avoids import-time CasADi dependency). For symbolic graph integration, it must be wrapped in a `cs.Callback` adapter.
3. **No integration test with real pinocchio.casadi**: Tests mock `cpin`. An integration test with a real robot model is deferred to Task 4 or later.
4. **test_column_elimination_callback uses 0-D array**: `tol = np.array(1e-6)` must be 0-D (not 1-D) to match the implementation's `float(arg[2])` conversion.

## Fix Round 1

### Fix Content

Applied three targeted fixes addressing code review issues on NumericalBackend and CasadiBackend:

| Issue | Severity | File | Change |
|-------|----------|------|--------|
| **C1** | CRITICAL | `src/figaroh/backend/base.py` | `create_backend("numerical", robot=robot)` now forwards robot via `NumericalBackend(robot=robot, **kwargs)` |
| **C1** | CRITICAL | `src/figaroh/backend/base.py` | Verified `create_backend("casadi")` already forwards robot correctly |
| **H1** | HIGH | `src/figaroh/backend/casadi.py` | Added `"info": solver.stats()` to `create_solver` return dict for consistency with `NumericalBackend`; kept `"status"` for backward compatibility |
| **M1** | MEDIUM | `src/figaroh/backend/numerical.py` | Moved `import cyipopt` from module level into `create_solver()` method body (deferred/lazy loading) |

### Test updates (`tests/unit/test_backend.py`)

- **New tests added** (4): `test_create_backend_numerical_forwards_robot`, `test_create_backend_casadi_forwards_robot`, `test_cyipopt_not_imported_at_module_level`, `test_create_solver_returns_info_key`
- **Adapted existing tests** (2): `test_create_solver_returns_callable` and `test_create_solver_passes_custom_bounds` changed mock strategy from `patch('figaroh.backend.numerical.cyipopt')` to `patch.dict('sys.modules', {'cyipopt': mock_cyipopt})` to work with lazy `import cyipopt` inside the method

### RED / GREEN Evidence

```
# RED: Tests that failed before fixes (expected failures)
FAILED test_create_backend_numerical_forwards_robot
  -- AssertionError: assert None is <MagicMock> (robot not forwarded)

FAILED test_cyipopt_not_imported_at_module_level
  -- AssertionError: cyipopt must be lazy-imported inside create_solver,
     not at module level (cyipopt in sys.modules)

# GREEN: All tests pass after fixes
$ PYTHONPATH=src pixi run python -m pytest tests/ -v
================== 236 passed, 2 skipped in 1.97s ==================

# Specifically the 4 new tests:
test_create_backend_numerical_forwards_robot     PASSED
test_create_backend_casadi_forwards_robot        PASSED
test_cyipopt_not_imported_at_module_level        PASSED
test_create_solver_returns_info_key              PASSED
```

### Commit

```
e070906 fix: NumericalBackend robot forwarding, cyipopt lazy import, CasadiBackend info key
```
