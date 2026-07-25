# excitation-trajectory-optimization Specification

## Purpose
TBD - created by archiving change symbolic-fourier-trajectory. Update Purpose after archive.
## Requirements
### Requirement: Multi-trajectory-type architecture

The system SHALL abstract trajectory generation behind a common interface (`BaseTrajectory`) supporting multiple parameterization strategies, with concrete implementations for cubic spline (`CubicSplineTrajectory`) and Fourier series (`FourierTrajectory`).

The `BaseOptimalTrajectory` class SHALL select the active trajectory strategy based on the `trajectory_type` configuration field.

#### Scenario: Fourier path selection

- **WHEN** `trajectory_type: "fourier"` is configured
- **THEN** `BaseOptimalTrajectory` SHALL instantiate `FourierTrajectory` and route optimization through the CasADi symbolic NLP pipeline

#### Scenario: Spline path selection (default)

- **WHEN** `trajectory_type` is absent or set to `"spline"`
- **THEN** `BaseOptimalTrajectory` SHALL use the existing cubic spline pipeline unchanged

### Requirement: Base parameter computation independence

The system SHALL continue to compute base parameter indices `idx_b` via the existing `BaseParameterComputer` (random cubic spline trajectory + QR decomposition), independent of the selected `trajectory_type`. This ensures base parameters are determined by the robot's structural properties rather than the trajectory parameterization.

#### Scenario: Same idx_b regardless of trajectory type

- **WHEN** `trajectory_type` is `"fourier"` or `"spline"`
- **THEN** `BaseParameterComputer.compute_base_indices()` SHALL produce the same `idx_b` for the same robot and `identif_config`

### Requirement: Extensible additional parameter columns

The system SHALL support dynamic column appending to the symbolic regressor matrix based on `identif_config` boolean flags (`has_friction`, `has_actuator_inertia`, `has_joint_offset`, `has_custom_parameters`). The column structure SHALL match the numerical backend's column ordering.

#### Scenario: New parameter type addition

- **WHEN** a future change adds a new flag (e.g., `has_spring_stiffness`) to `identif_config`
- **THEN** adding the corresponding column to the symbolic regressor SHALL require only defining the column expression and updating the column offset counter

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

