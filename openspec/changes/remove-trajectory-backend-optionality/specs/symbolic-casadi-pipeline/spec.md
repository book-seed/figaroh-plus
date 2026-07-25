## MODIFIED Requirements

### Requirement: CasADi single source of truth

The system SHALL expose pre-built symbolic dynamics functions via `CasadiBackend` property interfaces (`regressor_function` and `rnea_function`), providing a single source of truth for both trajectory optimization and parameter identification. External code SHALL NOT directly access private attributes of `CasadiBackend` (other than `_cmodel`, which is read by `FourierOptimizationStrategy` to obtain joint dimensions `nq`/`nv`).

`CasadiBackend` SHALL NOT inherit from any abstract base class and SHALL NOT implement the former `Backend` ABC interface methods (`build_regressor`, `gradient`, `jacobian`, `create_solver`, `name`). Those methods are removed as they had no production callers; only the symbolic-model interfaces consumed by `FourierOptimizationStrategy` SHALL remain: `_ensure_symbolic_model()`, `_cmodel`, `regressor_function`, and `rnea_function`.

The `Backend` ABC, `BackendType`, `create_backend` factory, and `NumericalBackend` SHALL NOT exist in the codebase after this change. `FourierOptimizationStrategy` SHALL obtain its `CasadiBackend` instance via `context._backend`, which `BaseOptimalTrajectory` instantiates directly when `trajectory_type == "fourier"`.

#### Scenario: Fourier strategy accesses dynamics via property

- **WHEN** `FourierOptimizationStrategy` requires the symbolic regressor or RNEA function
- **THEN** it SHALL obtain them via `context._backend.regressor_function` and `context._backend.rnea_function` properties
- **AND** SHALL access `context._backend._cmodel` only to read joint dimensions (`nq`, `nv`)
- **AND** SHALL NOT access `_W_fun`, `_rnea_fun`, or other private attributes directly

#### Scenario: CasadiBackend instantiated directly for fourier

- **WHEN** `BaseOptimalTrajectory` is constructed with `trajectory_type == "fourier"`
- **THEN** it SHALL instantiate `CasadiBackend(robot=robot)` directly via `from figaroh.backend.casadi import CasadiBackend`
- **AND** create_backend factory SHALL NOT be invoked (it no longer exists)

#### Scenario: Removed ABC methods not present

- **WHEN** the `CasadiBackend` class is inspected
- **THEN** it SHALL NOT define `build_regressor`, `gradient`, `jacobian`, `create_solver`, `name`, or `regressor_is_jacobian_of_rnea`
- **AND** ColumnEliminationCallback SHALL NOT exist in the codebase (it had no production callers)

## REMOVED Requirements

### Requirement: Extensible additional parameter columns

**Reason**: This requirement referenced "the numerical backend's column ordering" as a contract anchor. With the removal of `NumericalBackend` (dead code, no production callers) and the `Backend` abstraction layer, there is no longer a "numerical backend" whose column ordering could anchor the symbolic regressor's column structure. The column-extension *behavior* itself (dynamic column appending based on `has_friction`/`has_actuator_inertia`/`has_joint_offset`/`has_custom_parameters` flags) is preserved in `CasadiBackend`'s symbolic regressor construction, but the cross-backend column-ordering parity contract is voided because only one backend remains.

**Migration**: No code migration required — `CasadiBackend`'s regressor construction continues to append friction/actuator-inertia/offset columns in the same order. Callers that relied on a `NumericalBackend` instance for column ordering must remove that reference, as `NumericalBackend` no longer exists. This requirement will be re-established with backend-independent wording in a follow-up if cross-backend parity becomes relevant again.
