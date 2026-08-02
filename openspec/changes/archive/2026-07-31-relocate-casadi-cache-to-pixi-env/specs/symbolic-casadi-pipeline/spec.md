## MODIFIED Requirements

### Requirement: Symbolic CasADi model from URDF

The system SHALL construct a fully symbolic robot model using `pinocchio.casadi.Model` from the robot's URDF description, along with a corresponding `cpin.Model.createData()` symbolic data structure.

#### Scenario: Symbolic model construction

- **WHEN** a robot with a valid URDF is loaded
- **THEN** `cpin.Model(robot.model)` SHALL produce a CasADi symbolic model
- **AND** `cmodel.createData()` SHALL allocate symbolic data structures
- **AND** both SHALL be cached to disk: under `<project>/.cache/figaroh/casadi/` when running inside a pixi project (pyproject.toml found by walking up from `figaroh.__file__`), falling back to `~/.cache/figaroh/casadi/` (XDG-default) when no project root is found
- **AND** the cache fingerprint and `_CACHE_VERSION` invalidation mechanism SHALL remain unchanged
