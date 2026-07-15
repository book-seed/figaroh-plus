# fourier-trajectory-parameterization Specification

## Purpose
TBD - created by archiving change symbolic-fourier-trajectory. Update Purpose after archive.
## Requirements
### Requirement: Fourier trajectory parameterization

The system SHALL support Fourier series trajectory parameterization with configurable number of harmonics (default 5), where each active joint trajectory is expressed as:

```
q_j(t) = a_{j,0} + Σ_{k=1}^{N} [a_{j,k}·sin(k·ω_f·t) + b_{j,k}·cos(k·ω_f·t)]
```

The system SHALL accept the fundamental frequency ω_f from configuration with a code-level default of `2π/T` (where T is total trajectory duration).

The system SHALL generate velocity and acceleration trajectories via CasADi symbolic differentiation of the Fourier position expression with respect to time.

The system SHALL use Fourier coefficients as optimization decision variables, with total variable count = `n_active_joints × (1 + 2 × n_harmonics)`.

#### Scenario: Generate Fourier trajectory from coefficients

- **WHEN** Fourier coefficients (a_k, b_k), fundamental frequency ω_f, and number of sample points N_s are provided
- **THEN** the system SHALL produce position q(t), velocity v(t) = ∂q/∂t, and acceleration a(t) = ∂²q/∂t² arrays of shape (N_s, n_joints), all computed via CasADi symbolic expressions

#### Scenario: Velocity and acceleration are symbolic derivatives

- **WHEN** the Fourier position expression q(t) is constructed as a CasADi SX expression
- **THEN** v(t) and a(t) SHALL be obtained via `cs.jacobian()` or `cs.gradient()` w.r.t. time variable, with no numerical finite-difference fallback

#### Scenario: Coefficient initialization avoids singularities and collisions

- **WHEN** initial Fourier coefficients are generated
- **THEN** the offset term a_0 SHALL be set to the joint range midpoint `(q_upper + q_lower) / 2`
- **AND** the harmonic coefficients a_k, b_k SHALL be small random values scaled to 5-10% of the joint range to avoid joint limit violations, self-collisions, and degenerate configurations

### Requirement: Multi-harmonic configurability

The system SHALL allow the number of Fourier harmonics to be configured via `n_harmonics` in the configuration file, defaulting to 5.

#### Scenario: Default harmonic count

- **WHEN** `n_harmonics` is not specified in configuration
- **THEN** the system SHALL use 5 harmonics (11 coefficients per joint)

#### Scenario: Custom harmonic count

- **WHEN** `n_harmonics: 3` is specified in configuration
- **THEN** the system SHALL use 3 harmonics (7 coefficients per joint)

