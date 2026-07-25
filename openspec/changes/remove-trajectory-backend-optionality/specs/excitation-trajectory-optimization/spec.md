## ADDED Requirements

### Requirement: Backend determined by trajectory type

The system SHALL NOT expose a user-configurable `backend` parameter on `BaseOptimalTrajectory`. The computation backend SHALL be determined automatically and exclusively by the `trajectory_type` configuration field. When `trajectory_type` is `"fourier"`, the system SHALL instantiate `CasadiBackend` directly (no factory indirection). When `trajectory_type` is `"spline"` or absent, the system SHALL NOT instantiate any backend (the spline pipeline uses cyipopt via `RobotIPOPTSolver` and does not consume a backend abstraction).

The `trajectory_config` dictionary SHALL NOT contain a `backend` key, and the configuration loader SHALL NOT parse any `backend` entry from YAML.

#### Scenario: Fourier auto-selects CasADi backend

- **WHEN** `trajectory_type: "fourier"` is configured
- **THEN** `BaseOptimalTrajectory.__init__` SHALL instantiate `CasadiBackend(robot=robot)` and store it as `self._backend`
- **AND** SHALL NOT require the caller to pass any `backend` argument

#### Scenario: Spline creates no backend

- **WHEN** `trajectory_type` is `"spline"` or absent
- **THEN** `BaseOptimalTrajectory.__init__` SHALL set `self._backend = None`
- **AND** the cubic spline pipeline SHALL proceed unchanged via `create_ipopt_problem` → `RobotIPOPTSolver`

#### Scenario: Explicit backend argument rejected

- **WHEN** a caller invokes `BaseOptimalTrajectory(robot, config_file, backend="casadi")` or `backend="numerical"`
- **THEN** the call SHALL raise `TypeError` (unexpected keyword argument `backend`)
- **AND** no silent acceptance or deprecation warning SHALL be emitted

#### Scenario: YAML backend key ignored

- **WHEN** a YAML configuration file contains a `backend:` key under `problem` or `identification.trajectory_params`
- **THEN** the loader SHALL NOT parse the key into `trajectory_config`
- **AND** the effective backend SHALL be determined solely by `trajectory_type`

## REMOVED Requirements

### Requirement: Backward compatibility with existing spline pipeline

**Reason**: The requirement asserted that "all existing configuration files... and API calls SHALL continue to work without modification". This is superseded by the removal of the `backend` parameter: `BaseOptimalTrajectory` no longer accepts `backend=`, which is a breaking change to its API. The spline *pipeline behavior* itself (cyipopt path) is preserved, but the public API surface for backend selection is intentionally broken to eliminate a pseudo-optionality that produced only crash paths.

**Migration**: Callers passing `backend=` to `BaseOptimalTrajectory` must remove the argument; the backend is now auto-selected by `trajectory_type`. YAML files using a `backend:` key must remove it (no current config file in the repository uses this key, so no file migration is required). No runtime deprecation period is provided.
