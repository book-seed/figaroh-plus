## ADDED Requirements

### Requirement: Test task

The project SHALL provide a `test` pixi task that executes the test suite with pytest.

#### Scenario: Running test task

- **WHEN** `pixi run test` is executed
- **THEN** pytest SHALL run all tests in the `tests/` directory and report results

### Requirement: Lint task

The project SHALL provide a `lint` pixi task that runs flake8 and mypy for static analysis.

#### Scenario: Running lint task

- **WHEN** `pixi run lint` is executed
- **THEN** flake8 and mypy SHALL check the `src/` directory and report warnings/errors

### Requirement: Format task

The project SHALL provide a `format` pixi task that runs black and isort for code formatting.

#### Scenario: Running format task

- **WHEN** `pixi run format` is executed
- **THEN** black and isort SHALL format all Python files in `src/` and `tests/`

### Requirement: Docs task

The project SHALL provide a `docs` pixi task that builds Sphinx HTML documentation.

#### Scenario: Running docs task

- **WHEN** `pixi run docs` is executed
- **THEN** Sphinx SHALL generate HTML documentation in the `docs/build/html/` directory

### Requirement: Build task

The project SHALL provide a `build` pixi task that builds a Python wheel distribution.

#### Scenario: Running build task

- **WHEN** `pixi run build` is executed
- **THEN** a wheel file SHALL be produced in the `dist/` directory

### Requirement: Shell task

The project SHALL provide a `shell` pixi task that launches an interactive shell with the development environment activated.

#### Scenario: Running shell task

- **WHEN** `pixi run shell` is executed
- **THEN** an interactive shell SHALL be launched with the default environment activated and `python` available with figaroh importable

### Requirement: Clean task

The project SHALL provide a `clean` pixi task that removes build artifacts.

#### Scenario: Running clean task

- **WHEN** `pixi run clean` is executed
- **THEN** the `dist/`, `build/`, and `*.egg-info/` directories SHALL be removed

### Requirement: CI workflow integration

The `.github/workflows/docs.yml` workflow SHALL be updated to use pixi for dependency installation and documentation building.

#### Scenario: CI docs workflow builds with pixi

- **WHEN** the docs workflow runs on GitHub Actions
- **THEN** dependencies SHALL be installed via `pixi install -e docs` and documentation SHALL be built via `pixi run docs`
