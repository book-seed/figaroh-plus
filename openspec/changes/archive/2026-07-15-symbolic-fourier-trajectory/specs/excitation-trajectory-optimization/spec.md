## ADDED Requirements

### Requirement: Multi-trajectory-type architecture

The system SHALL abstract trajectory generation behind a common interface (`BaseTrajectory`) supporting multiple parameterization strategies, with concrete implementations for cubic spline (`CubicSplineTrajectory`) and Fourier series (`FourierTrajectory`).

The `BaseOptimalTrajectory` class SHALL select the active trajectory strategy based on the `trajectory_type` configuration field.

#### Scenario: Fourier path selection

- **WHEN** `trajectory_type: "fourier"` is configured
- **THEN** `BaseOptimalTrajectory` SHALL instantiate `FourierTrajectory` and route optimization through the CasADi symbolic NLP pipeline

#### Scenario: Spline path selection (default)

- **WHEN** `trajectory_type` is absent or set to `"spline"`
- **THEN** `BaseOptimalTrajectory` SHALL use the existing cubic spline pipeline unchanged

### Requirement: Backward compatibility with existing spline pipeline

The system SHALL preserve the exact behavior of the existing cubic-spline-based excitation trajectory optimization when `trajectory_type` is `"spline"` or unspecified. All existing configuration files, saved results, and API calls SHALL continue to work without modification.

#### Scenario: Legacy config works

- **WHEN** a pre-existing YAML configuration file without `trajectory_type` is loaded
- **THEN** the optimization SHALL behave identically to before the change
- **AND** all saved output formats SHALL remain compatible

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
