# Comet Design Handoff

- Change: migrate-to-pixi
- Phase: design
- Mode: compact
- Context hash: 778dbe7868e6a913c3541718bd62b3b72908bdc194ab5ae617a9622bfcccd3ec

Generated-by: comet-handoff.sh

OpenSpec remains the canonical capability spec. This handoff is a deterministic, source-traceable context pack, not an agent-authored summary.

## openspec/changes/migrate-to-pixi/proposal.md

- Source: openspec/changes/migrate-to-pixi/proposal.md
- Lines: 1-30
- SHA256: 7cf6e0426de380b1b6cba7f72a7742cf7cde2d05f9aae26da5a50aa358c99b0f

```md
## Why

figaroh-plus 当前依赖管理分散在两处（pyproject.toml + environment.yml），没有 lock 文件导致环境不可复现，且 conda/pip 手动操作繁琐。迁移到 pixi 包管理器可以统一 conda 和 PyPI 依赖、生成跨平台可复现 lock 文件、并通过 task 系统自动化日常工作流。

## What Changes

- **新增** `pixi.toml`：主配置文件，统一声明所有依赖（conda + PyPI）、多环境定义、任务定义
- **新增** `pixi.lock`：跨平台可复现锁文件（linux-aarch64、linux-64、osx-arm64）
- **简化** `pyproject.toml`：删除 `[project.dependencies]` 和 `[project.optional-dependencies]`，仅保留构建元数据
- **删除** `environment.yml`：由 pixi.toml 替代
- **更新** `.github/workflows/docs.yml`：使用 pixi 替代原始 pip/apt 安装
- **更新** `.gitignore`：添加 pixi 相关条目（.pixi/ 目录等）

## Capabilities

### New Capabilities

- `pixi-environment`: 通过 pixi 统一管理 conda 和 PyPI 依赖，支持多环境（default、docs、test）和多平台（linux-aarch64、linux-64、osx-arm64）
- `pixi-tasks`: 自动化的 pixi task 定义，覆盖 test、lint、format、docs、build、shell、clean

### Modified Capabilities

_无_ — 本次变更为工具链改造，不影响现有功能 API。

## Impact

- **配置文件**：pixi.toml（新增）、pyproject.toml（精简）、environment.yml（删除）
- **CI/CD**：docs.yml 的依赖安装步骤改为 pixi 命令
- **开发流程**：开发者从 `conda env create` + `pip install -e .` 切换为 `pixi install` + `pixi run shell`
- **依赖**：所有现有运行时依赖保持不变，仅管理方式变更
```

## openspec/changes/migrate-to-pixi/design.md

- Source: openspec/changes/migrate-to-pixi/design.md
- Lines: 1-99
- SHA256: 0abcad9fbc905cbe06d827dcecdc4d8702d36744c7fecd9d739b8555255d9f30

[TRUNCATED]

```md
## Context

figaroh-plus 是一个 Python 机器人动力学/标定工具箱（v0.4.3），当前依赖管理方式：

```
当前状态：
  pyproject.toml  → 声明 PyPI 依赖（numpy、scipy、pin 等）+ hatchling 构建
  environment.yml → 声明 conda 环境（python 3.12 + cyipopt）
  .github/workflows/docs.yml → 手动 apt/pip 安装依赖构建文档
  无 lock 文件 → 环境不可复现
```

pixi (v0.71.0) 已安装在开发机上。项目运行在 NVIDIA Jetson (linux-aarch64)，同时需要支持 x86 CI 和 macOS 开发。

## Goals / Non-Goals

**Goals:**
- 统一依赖声明到 `pixi.toml`（conda + PyPI）
- 生成三平台（linux-aarch64、linux-64、osx-arm64）`pixi.lock`
- 定义 7 个 pixi tasks：test、lint、format、docs、build、shell、clean
- 多环境支持：default（核心开发）、docs（文档构建）、docs 和 test
- 精简 pyproject.toml 为构建元数据骨架
- 更新 CI docs.yml 使用 pixi

**Non-Goals:**
- 不修改源码和 API
- 不改变 Python 版本要求（保持 >=3.8）
- 不引入新运行时依赖
- 不迁移到其他构建系统（保持 hatchling）

## Decisions

### 1. 共存模式：pixi.toml + pyproject.toml（精简）

**选择**：pixi.toml 管理所有依赖和 tasks，pyproject.toml 精简为构建元数据骨架。

**Alternatives considered:**
- 纯 pixi.toml（删除 pyproject.toml）→ 失去 Python 生态兼容（pip install 失效、PyPI 发布受影响）
- pip-tools / Poetry → 不原生支持 conda 依赖（cyipopt），需要额外脚本桥接

**Rationale**：pixi 原生支持 conda-forge 和 PyPI，同时保持 pyproject.toml 兼容标准 Python 工具链。

### 2. 纯 pixi 模式

**选择**：`pixi.toml` 作为依赖唯一来源，pyproject.toml 不保留 `[dependencies]`。

**Rationale**：避免两处声明依赖导致的不一致风险。pixi 的 `[pypi-dependencies]` 和 `[dependencies]`（conda）覆盖所有用例。

### 3. 多环境设计

**选择**：三个 pixi feature/environment：

| 环境 | 内容 | 用途 |
|------|------|------|
| `default` | 完整 conda + PyPI 依赖 | 日常开发 |
| `docs` | sphinx + 文档依赖 | CI 文档构建 |
| `test` | pytest + 核心依赖 | CI 测试 |

**Rationale**：CI 环境不需要完整 GUI 依赖（meshcat、matplotlib），减少解析时间和体积。

### 4. tasks 设计

| task | 环境 | 命令 |
|------|------|------|
| `shell` | default | 启动带激活环境的 shell |
| `test` | default | `pytest tests/` |
| `lint` | default | `flake8 src/ && mypy src/` |
| `format` | default | `black src/ tests/ && isort src/ tests/` |
| `docs` | docs | `cd docs && make html` |
| `build` | default | `python -m build` |
| `clean` | default | `rm -rf dist/ build/ *.egg-info` |

### 5. 平台策略

三个目标平台按 tier 分级：
- **Tier 1 (linux-aarch64)**：主开发平台，lock 必须完整
- **Tier 1 (linux-64)**：CI 平台，lock 必须完整
- **Tier 2 (osx-arm64)**：开发者平台，尽力完整；已知 `rospkg` 可能不可用，必要时降级为可选项

### 6. CI 迁移
```

Full source: openspec/changes/migrate-to-pixi/design.md

## openspec/changes/migrate-to-pixi/tasks.md

- Source: openspec/changes/migrate-to-pixi/tasks.md
- Lines: 1-44
- SHA256: 6983416b441a0b6e345a82db71279b09a55cc93fdc4b8114eb1d44b59ca4b4e9

```md
## 1. 前期调研

- [ ] 1.1 验证所有依赖在 conda-forge 三平台（linux-aarch64、linux-64、osx-arm64）的可用性，记录不可用包及 fallback 方案
- [ ] 1.2 确认 `pin` 包在 conda-forge/PyPI 中的实际名称，避免名称冲突

## 2. pixi.toml 配置

- [ ] 2.1 初始化 pixi 项目：`pixi init`，设置三平台 `platforms = ["linux-aarch64", "linux-64", "osx-arm64"]`
- [ ] 2.2 添加 conda 依赖：`python`（>=3.8）、`cyipopt`（conda-forge channel）
- [ ] 2.3 添加 PyPI 依赖：`numpy`、`scipy`、`pin`、`matplotlib`、`pyyaml`、`meshcat`、`pandas`、`rospkg`、`picos`、`numdifftools`、`ndcurves`
- [ ] 2.4 添加 dev PyPI 依赖：`pytest`(>=6.0)、`pytest-cov`、`black`、`isort`、`flake8`、`mypy`
- [ ] 2.5 添加 docs PyPI 依赖：`sphinx`、`sphinx-rtd-theme`、`sphinx-autodoc-typehints`
- [ ] 2.6 定义 `default`、`docs`、`test` 三个 feature/environment
- [ ] 2.7 定义 7 个 tasks：`test`、`lint`、`format`、`docs`、`build`、`shell`、`clean`

## 3. pixi.lock 生成

- [ ] 3.1 运行 `pixi install` 在 linux-aarch64（当前平台）生成 lock
- [ ] 3.2 验证 `pixi.lock` 包含三平台解析条目

## 4. pyproject.toml 精简

- [ ] 4.1 删除 `[project.dependencies]` 节
- [ ] 4.2 删除 `[project.optional-dependencies]` 节（dev、docs、examples）
- [ ] 4.3 保留 `[build-system]` 和 `[project]` 元数据（name、version、authors 等），添加注释说明依赖由 pixi.toml 管理

## 5. 旧文件清理

- [ ] 5.1 删除 `environment.yml`
- [ ] 5.2 更新 `.gitignore`，添加 `.pixi/` 目录排除

## 6. CI 工作流更新

- [ ] 6.1 更新 `.github/workflows/docs.yml`，使用 pixi 安装依赖和构建文档
- [ ] 6.2 验证更新后的 workflow 语法正确（至少本地检查）

## 7. 验证

- [ ] 7.1 运行 `pixi run shell` 确认开发环境可用，`import figaroh` 成功
- [ ] 7.2 运行 `pixi run test` 确认 pytest 通过
- [ ] 7.3 运行 `pixi run lint` 确认代码检查正常
- [ ] 7.4 运行 `pixi run docs` 确认文档可构建
- [ ] 7.5 运行 `pixi run build` 确认 wheel 可构建
- [ ] 7.6 更新 `README.md` 开发环境说明，添加 pixi quickstart
```

## openspec/changes/migrate-to-pixi/specs/pixi-environment/spec.md

- Source: openspec/changes/migrate-to-pixi/specs/pixi-environment/spec.md
- Lines: 1-71
- SHA256: 6d2ac1df27e7ecd8d885b62d8a53683e2bc78dfc7a9e85e10dae0b4a0018eaf5

```md
## ADDED Requirements

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
```

## openspec/changes/migrate-to-pixi/specs/pixi-tasks/spec.md

- Source: openspec/changes/migrate-to-pixi/specs/pixi-tasks/spec.md
- Lines: 1-73
- SHA256: 29f29b96185543f820715cf36717cbd79e8877dc34f7ebf98ea76e2592c2a835

```md
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
```

