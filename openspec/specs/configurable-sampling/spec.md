# configurable-sampling Specification

## Purpose
TBD - created by archiving change symbolic-fourier-trajectory. Update Purpose after archive.
## Requirements
### Requirement: Configurable number of sample points

The system SHALL expose the number of trajectory sample points `n_samples` as a configuration parameter in the YAML config file, with a code-level default of 200.

#### Scenario: Default sample count

- **WHEN** `n_samples` is not specified in configuration
- **THEN** the system SHALL use 200 sample points

#### Scenario: Custom sample count

- **WHEN** `n_samples: 500` is specified in configuration
- **THEN** the trajectory SHALL be sampled at 500 evenly-spaced time points

### Requirement: Configurable fundamental frequency

The system SHALL expose the fundamental frequency `fourier_frequency` (ω_f in rad/s) as a configuration parameter in the YAML config file.

When `fourier_frequency` is not specified, the system SHALL default to `ω_f = 2π / T` where `T = t_s × (n_wps - 1)` is the total trajectory duration derived from the existing time-step configuration.

#### Scenario: Default frequency from duration

- **WHEN** `fourier_frequency` is absent from config and total duration is `T = t_s × (n_wps - 1)`
- **THEN** `ω_f` SHALL default to `2π / T`

#### Scenario: Explicit frequency override

- **WHEN** `fourier_frequency: 0.5` (rad/s) is specified in config
- **THEN** `ω_f` SHALL be `0.5`, regardless of trajectory duration

### Requirement: Trajectory type selection

The system SHALL support a `trajectory_type` configuration field accepting `"spline"` (default) or `"fourier"`, selecting the trajectory parameterization strategy at initialization time.

#### Scenario: Default trajectory type

- **WHEN** `trajectory_type` is absent from configuration
- **THEN** the system SHALL use cubic spline trajectory (backward compatible)

#### Scenario: Explicit Fourier selection

- **WHEN** `trajectory_type: "fourier"` is specified
- **THEN** the system SHALL use the Fourier series trajectory and the CasADi symbolic optimization pipeline

### Requirement: All new parameters have defaults

Every newly introduced configuration parameter SHALL have a sensible default value, and the absence of any new parameter in the YAML file SHALL NOT change the default system behavior (backward compatible).

#### Scenario: Empty config with new fields
- **WHEN** a configuration file containing only legacy fields is loaded
- **THEN** all new Fourier-related parameters SHALL assume their defaults
- **AND** the system SHALL function identically to before the change

