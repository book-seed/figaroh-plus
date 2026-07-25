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

### Requirement: Symbolic regressor matrix construction

The system SHALL construct the full joint torque regressor matrix `W` symbolically via `cpin.computeJointTorqueRegressor(cmodel, cdata, cs_q, cs_v, cs_a)`.

The system SHALL build the base regressor `W_b` by selecting columns from W using the pre-computed base parameter indices `idx_b` (from `BaseParameterComputer`).

The system SHALL append additional parameter columns (friction fv/fs, actuator inertia Ia, joint offset) to the symbolic regressor when enabled in `identif_config`, before base-parameter column selection.

**Modification rationale**: The former scenario clause anchored the additional-column ordering to "the numerical backend's column order". With the removal of `NumericalBackend` (dead code, no production callers) and the `Backend` abstraction layer, there is no longer a "numerical backend" whose column ordering could anchor the symbolic regressor's column structure. The column-extension *behavior* itself (dynamic column appending based on `has_friction`/`has_actuator_inertia`/`has_joint_offset`/`has_custom_parameters` flags) is preserved unchanged in `CasadiBackend`'s symbolic regressor construction; only the cross-backend column-ordering parity contract is voided because only one backend remains.

#### Scenario: Regressor with additional parameters

- **WHEN** `identif_config.has_friction: true`
- **THEN** the symbolic regressor SHALL include viscous friction columns `fv · v_j` and static friction columns `fs · tanh(α·v_j)` for each active joint
- **AND** the additional columns SHALL be appended in a fixed, backend-independent order (friction fv/fs, then actuator inertia Ia, then joint offset) determined solely by `CasadiBackend`'s symbolic regressor construction

## REMOVED Requirements

_This change no longer removes any standalone requirement. The cross-backend column-ordering parity contract formerly embedded in the "Symbolic regressor matrix construction" requirement is re-anchored to a backend-independent ordering via the MODIFIED requirement above._

