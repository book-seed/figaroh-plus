# unified-friction-model Specification

## Purpose
TBD - created by archiving change symbolic-fourier-trajectory. Update Purpose after archive.
## Requirements
### Requirement: Unified tanh friction model for optimization and identification

The system SHALL use `tanh(α·v)` as the friction sign approximation in BOTH the trajectory optimization regressor and the parameter identification regressor, ensuring model consistency between phases.

The tanh steepness parameter `α` SHALL be independently configurable for optimization (`tanh_alpha_opt`, default 10) and identification (`tanh_alpha_id`, default 100).

#### Scenario: Optimization uses gradient-friendly α

- **WHEN** the CasADi symbolic regressor is built during trajectory optimization
- **THEN** the static friction column SHALL use `tanh(tanh_alpha_opt · v)`
- **AND** `tanh_alpha_opt` SHALL default to 10 to avoid sharp gradients at v=0

#### Scenario: Identification uses high-precision α

- **WHEN** the numerical regressor is built during parameter identification
- **THEN** the static friction column SHALL use `tanh(tanh_alpha_id · v)`
- **AND** `tanh_alpha_id` SHALL default to 100 for closer approximation to `sign(v)`

#### Scenario: tanh is differentiable in CasADi graph

- **WHEN** the objective or constraint Jacobian is computed via CasADi AD
- **THEN** `tanh(α·v)` SHALL be fully differentiable through the CasADi expression graph
- **AND** the gradient at |v| > 3/α SHALL be effectively zero (saturated), avoiding exploding gradients

### Requirement: Configurable tanh parameters

Both `tanh_alpha_opt` and `tanh_alpha_id` SHALL be configurable in the robot configuration YAML file, with the stated defaults applied when absent.

#### Scenario: Default values applied

- **WHEN** neither `tanh_alpha_opt` nor `tanh_alpha_id` is specified in config
- **THEN** `tanh_alpha_opt` SHALL default to 10
- **AND** `tanh_alpha_id` SHALL default to 100

