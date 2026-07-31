# symbolic-casadi-pipeline — Delta Spec

## MODIFIED Requirement: Symbolic CasADi model from URDF

### MODIFIED Scenario: Symbolic model construction

- **WHEN** a robot with a valid URDF is loaded
- **THEN** `cpin.Model(robot.model)` SHALL produce a CasADi symbolic model
- **AND** `cmodel.createData()` SHALL allocate symbolic data structures
- **AND** both SHALL be cached to disk: under `<project>/.cache/figaroh/casadi/` when running inside a pixi project (pyproject.toml found by walking up from `figaroh.__file__`), falling back to `~/.cache/figaroh/casadi/` (XDG-compatible) when no project root is found
- **AND** the cache fingerprint and `_CACHE_VERSION` invalidation mechanism SHALL remain unchanged

### Rationale

Current spec (line 15 of main spec) reads:

> `~/.figaroh/casadi_cache/`

This is replaced with the project-local + XDG fallback paths described above. The
cache fingerprint (SHA256 over full inertial parameters + joint structure) and
`_CACHE_VERSION` tag behavior are preserved — only the base directory resolution
changes.
