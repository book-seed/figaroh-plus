# pixi-environment Specification

## Purpose
TBD - created by archiving change migrate-to-pixi. Update Purpose after archive.
## Requirements
### Requirement: pixi.toml as single dependency source

The project SHALL use `pixi.toml` as the single source of truth for all dependencies (conda and PyPI). The `pyproject.toml` SHALL retain only build metadata (name, version, build-system) without dependency declarations.

#### Scenario: pixi.toml exists and is valid

- **WHEN** `pixi.toml` is present in the project root
- **THEN** `pixi install` SHALL resolve all conda and PyPI dependencies and create a working development environment

#### Scenario: pyproject.toml is stripped of dependencies

- **WHEN** the migration is complete
- **THEN** `pyproject.toml` SHALL NOT contain `[project.dependencies]` or `[project.optional-dependencies]` sections

### Requirement: Multi-platform reproducible lock

The project SHALL provide a `pixi.lock` file that locks dependencies for all three target platforms: `linux-aarch64`, `linux-64`, and `osx-arm64`.

#### Scenario: Lock file resolves for all platforms

- **WHEN** `pixi install` or `pixi update` is run
- **THEN** `pixi.lock` SHALL contain dependency resolutions for all declared platforms

#### Scenario: Environment is reproducible

- **WHEN** two developers run `pixi install` from the same `pixi.lock`
- **THEN** both SHALL get identical package versions

### Requirement: Multi-environment support

The project SHALL define at least three pixi environments: `default` (core development with all deps), `docs` (documentation build deps only), and `test` (minimal test deps).

#### Scenario: Default environment contains all dependencies

- **WHEN** `pixi install` (default environment) completes
- **THEN** executing `import figaroh` in the activated shell SHALL succeed

#### Scenario: Docs environment builds documentation

- **WHEN** `pixi install -e docs` completes
- **THEN** the docs environment SHALL contain sphinx and its dependencies

#### Scenario: Test environment runs tests

- **WHEN** `pixi install -e test` completes
- **THEN** the test environment SHALL contain pytest and core dependencies sufficient to run `pytest tests/`

### Requirement: Conda dependency support

The project SHALL declare conda-specific dependencies (e.g., `cyipopt`) via pixi's conda channel integration using `conda-forge` as the primary channel.

#### Scenario: cyipopt is installed from conda-forge

- **WHEN** `pixi install` runs
- **THEN** `cyipopt` SHALL be resolved and installed from the `conda-forge` channel

### Requirement: Existing files cleanup

The `environment.yml` file SHALL be removed after migration. The `.gitignore` SHALL be updated with pixi-related entries.

#### Scenario: environment.yml is removed

- **WHEN** the migration is complete
- **THEN** `environment.yml` SHALL NOT exist in the project root

#### Scenario: .gitignore includes pixi entries

- **WHEN** the migration is complete
- **THEN** `.gitignore` SHALL include entries to exclude `.pixi/` directory

