# casadi-backend Specification

## Purpose
TBD - created by archiving change casadi-optional-backend. Update Purpose after archive.
## Requirements
### Requirement: Backend selection via configuration
The system SHALL support selecting the computation backend via a `backend` key.

#### Scenario: Default backend is numerical
- **WHEN** no `backend` key is present
- **THEN** the system SHALL use the `NumericalBackend`

#### Scenario: CasADi backend selected
- **WHEN** `backend: casadi` is set
- **THEN** the system SHALL use `CasadiBackend`

