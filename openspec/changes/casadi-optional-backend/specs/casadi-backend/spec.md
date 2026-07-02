# CasADi Backend

## ADDED Requirements

### Requirement: Backend selection via configuration
The system SHALL support selecting the computation backend via a `backend`
key in the robot configuration YAML file, with valid values `numerical`
(default) and `casadi`.

#### Scenario: Default backend is numerical
- **WHEN** no `backend` key is present in the configuration
- **THEN** the system SHALL use the `NumericalBackend` (current behavior preserved)

#### Scenario: Explicit numerical backend
- **WHEN** `backend: numerical` is set in the configuration
- **THEN** the system SHALL use `NumericalBackend` (identical to default)

#### Scenario: CasADi backend selected
- **WHEN** `backend: casadi` is set in the configuration AND the CasADi
  optional dependency is installed
- **THEN** the system SHALL use `CasadiBackend` for regressor evaluation
  and IPOPT gradient/Jacobian computation

#### Scenario: CasADi backend selected but not installed
- **WHEN** `backend: casadi` is set but `pinocchio.casadi` cannot be imported
- **THEN** the system SHALL raise a clear `ImportError` with instructions
  for installing the optional dependency

### Requirement: CasADi backend provides analytical gradient
The `CasadiBackend.gradient()` method SHALL return the analytically
differentiated gradient of the objective function using CasADi's automatic
differentiation, producing numerically equivalent results to
`NumericalBackend.gradient()` (finite differences) within a tolerance of 1e-6.

#### Scenario: Gradient matches numerical reference
- **WHEN** both `CasadiBackend.gradient(x)` and `NumericalBackend.gradient(x)`
  are called with the same input x
- **THEN** the maximum absolute difference SHALL be less than 1e-6

#### Scenario: Gradient is faster than finite differences
- **WHEN** `CasadiBackend.gradient(x)` is benchmarked against
  `NumericalBackend.gradient(x)` for a typical robot configuration
  (6+ DOF, 24+ optimization variables)
- **THEN** the CasADi gradient SHALL execute at least 5x faster

### Requirement: CasADi backend provides analytical Jacobian
The `CasadiBackend.jacobian()` method SHALL return the analytically
differentiated Jacobian of constraint functions, eliminating the per-column
finite-difference loop used by `NumericalBackend.jacobian()`.

#### Scenario: Jacobian matches numerical reference
- **WHEN** both `CasadiBackend.jacobian(x)` and `NumericalBackend.jacobian(x)`
  are called with the same input x
- **THEN** the maximum absolute difference SHALL be less than 1e-6

### Requirement: Backend-agnostic BaseOptimalTrajectory
`BaseOptimalTrajectory` SHALL accept an optional `backend` parameter
(`Literal["numerical", "casadi"]` or `Backend` instance). When omitted,
it SHALL default to `"numerical"`. All existing subclasses SHALL continue
to work without modification.

#### Scenario: Existing code unchanged
- **WHEN** `BaseOptimalTrajectory(robot, active_joints, config_file)` is
  called without `backend` argument
- **THEN** the behavior SHALL be identical to the pre-change implementation

#### Scenario: CasADi backend passed programmatically
- **WHEN** `BaseOptimalTrajectory(robot, active_joints, config_file,
  backend="casadi")` is called
- **THEN** `solve()` SHALL use `CasadiBackend` for all IPOPT derivative
  computations

### Requirement: CasADi is an optional dependency
CasADi and conda-forge `pinocchio` (with CasADi bindings) SHALL be declared
as optional dependencies. The `numerical` backend SHALL function without
CasADi installed.

#### Scenario: Numerical path works without CasADi
- **WHEN** `casadi` is not installed in the Python environment
- **THEN** all existing functionality (identification, calibration, optimal
  trajectory with `numerical` backend) SHALL work without import errors
