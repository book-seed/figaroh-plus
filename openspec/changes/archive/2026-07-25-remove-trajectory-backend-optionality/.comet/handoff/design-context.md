# Comet Design Handoff

- Change: remove-trajectory-backend-optionality
- Phase: design
- Mode: compact
- Context hash: 2ce368bc0df3fc4a3a3ef653b394e180711e28d705b24d99454cc758f7649474

Generated-by: comet-handoff.sh

OpenSpec remains the canonical capability spec. This handoff is a deterministic, source-traceable context pack, not an agent-authored summary.

## openspec/changes/remove-trajectory-backend-optionality/proposal.md

- Source: openspec/changes/remove-trajectory-backend-optionality/proposal.md
- Lines: 1-50
- SHA256: cb76bcfe333cfe79cb415ce28a8479a0aadd34ab06c77f3a0f748cc363df94e6

```md
## Why

`BaseOptimalTrajectory` 暴露了一个 `backend` 可配置参数，但 4 种 `trajectory_type × backend` 组合里只有 2 种合法，且合法组合完全由 `trajectory_type` 决定：

- `fourier` + `casadi` → 正常（fourier strategy 硬依赖 `CasadiBackend` 的符号模型接口）
- `spline` + `numerical` → 正常（spline 走 cyipopt，从不碰 `_backend`）
- `fourier` + `numerical` → `AttributeError`（UR10 默认配置 `ur10_unified_config.yaml` 触发此崩溃路径）
- `spline` + `casadi` → 进入已弃用的 `_solve_with_casadi_backend`（作者明确 deprecate）

`backend` 因此是一个**伪自由度**：它不提供真实选择，只制造崩溃路径。更根本的是，整个 `Backend` 抽象层（`Backend` ABC + `NumericalBackend` + `create_backend` 工厂）的 4 个抽象方法（`build_regressor`/`gradient`/`jacobian`/`create_solver`）在生产代码里零调用——fourier 用的是 `CasadiBackend` 的非抽象专属接口，spline 完全绕开 backend 抽象。

## What Changes

- **BREAKING**：移除 `BaseOptimalTrajectory.__init__` 的 `backend` 参数及其三级优先级解析逻辑（显式传参 > 配置文件 > 默认）。
- **BREAKING**：移除 `optimal/config.py` 中 `trajectory_config["backend"]` 键的解析（L85、L126）。YAML 配置中的 `backend:` 键不再被识别。
- backend 改由 `trajectory_type` 自动决定：`fourier` → 自动实例化 `CasadiBackend`（直接 import）；`spline` → 不创建任何 backend。
- **BREAKING**：移除整个 `src/figaroh/backend/base.py`（`Backend` ABC、`BackendType` 类型别名、`create_backend` 工厂函数）。
- **BREAKING**：移除整个 `src/figaroh/backend/numerical.py`（`NumericalBackend`——生产路径零调用的死代码）。
- 移除 `base_optimal_trajectory.py` 中已弃用的 `_solve_with_casadi_backend`（约 294 行）及其在 `solve_with_waypoints` 中的触发分支（L971-974）。
- 改造 `src/figaroh/backend/casadi.py` 的 `CasadiBackend`：不再继承 `Backend` ABC，删除 fourier 不使用的 ABC 抽象方法实现，只保留 fourier strategy 实际依赖的符号模型接口（`_ensure_symbolic_model` / `_cmodel` / `regressor_function` / `rnea_function`）及其缓存机制。
- 调整测试：删除 `test_backend.py` 中针对 `Backend` ABC 与 `NumericalBackend` 的测试，`CasadiBackend` 测试去掉 ABC 相关断言；删除 `test_config.py` 中验证 backend 参数传递/优先级的过期测试；删除 `test_fourier_e2e.py` 中的 `backend="casadi"` 传参。

## Capabilities

### New Capabilities

无新增 capability。

### Modified Capabilities

- `excitation-trajectory-optimization`: backend 选择行为从"用户显式配置 / 配置文件"改为"由 `trajectory_type` 自动决定"。移除 `backend` 可配置参数，`BaseOptimalTrajectory` 不再接受 `backend` 入参，`trajectory_config` 不再包含 `backend` 键。`fourier` 自动绑定 CasADi，`spline` 不创建 backend。
- `symbolic-casadi-pipeline`: `CasadiBackend` 不再实现 `Backend` ABC 接口，其对外契约收窄为仅暴露 fourier strategy 所需的符号模型属性（`_ensure_symbolic_model` / `_cmodel` / `_cmodel` / `regressor_function` / `rnea_function`）。`Backend` ABC、`NumericalBackend`、`create_backend` 工厂不再存在。

## Impact

- **公共 API（breaking）**：
  - `BaseOptimalTrajectory(robot, config_file, backend=...)` → 移除 `backend` 形参
  - `figaroh.backend.base.Backend` / `BackendType` / `create_backend` → 删除
  - `figaroh.backend.numerical.NumericalBackend` → 删除
  - `figaroh.backend.__init__` 的 `__all__` 收窄（仅保留 `CasadiBackend`）
- **受影响源文件**：
  - `src/figaroh/backend/base.py`（删除）、`numerical.py`（删除）、`__init__.py`（瘦身）、`casadi.py`（改造）
  - `src/figaroh/optimal/base_optimal_trajectory.py`（去 backend 参数/解析/294 行 deprecated 方法/触发分支）
  - `src/figaroh/optimal/config.py`（去 backend 键）
  - `src/figaroh/__init__.py`（确认 `from . import backend` 是否保留——design 阶段定）
  - `src/figaroh/optimal/strategies/fourier_strategy.py`（`context._backend` 获取方式不变，但需验证 CasadiBackend 改造后接口一致）
- **受影响测试**：`tests/unit/test_backend.py`（大幅删减）、`tests/unit/test_config.py`（删过期测试）、`tests/unit/test_fourier_e2e.py`（删 backend 传参）、`tests/unit/test_fourier_strategy.py`（mock 接口校验）
- **配置契约（breaking）**：YAML 配置文件中 `backend:` 键不再被识别；现有配置文件均未使用该键，无迁移负担。
- **依赖**：不引入新依赖，不改变 casadi/pinocchio 依赖要求。
- **兼容性**：无 deprecation 过渡期，直接 breaking。
```

## openspec/changes/remove-trajectory-backend-optionality/design.md

- Source: openspec/changes/remove-trajectory-backend-optionality/design.md
- Lines: 1-139
- SHA256: 04a67a4e043539cbaeb26ed8e5ec87ec4d8dcc49ade37a4389784fd77a26a2a4

[TRUNCATED]

```md
## Context

`BaseOptimalTrajectory` 当前暴露 `backend` 可配置参数，由旧 change「casadi-optional-backend」引入。其设计意图是用 `Backend` ABC（`build_regressor`/`gradient`/`jacobian`/`create_solver` 四抽象方法 + `name`）统一数值与符号两条求解路径，并通过 `create_backend` 工厂按 `numerical`/`casadi` 字符串解析。

实际落地后该抽象两端都未接上：

- **fourier 路径**用的是 `CasadiBackend` 的**非抽象专属接口**（`_ensure_symbolic_model` / `_cmodel` / `regressor_function` / `rnea_function`），ABC 的 4 个抽象方法在 fourier strategy 里零调用。
- **spline 路径**完全绕开 backend 抽象，直接走 `create_ipopt_problem` → `RobotIPOPTSolver`（cyipopt），`self._backend` 仅在 `solve_with_waypoints` 开头被读一次以决定是否进入 deprecated 分支。
- `NumericalBackend` 在生产代码中零调用，仅 `test_backend.py` 测试覆盖。

结果是 `trajectory_type × backend` 四组合里只有两种合法，且合法组合完全由 `trajectory_type` 决定；`fourier + numerical`（UR10 默认配置触发）直接 `AttributeError`，`spline + casadi` 进入作者已弃用的 `_solve_with_casadi_backend`。

## Goals / Non-Goals

**Goals:**
- backend 由 `trajectory_type` 自动决定：`fourier` → 自动创建 `CasadiBackend`，`spline` → 不创建 backend。
- 删除未被使用的 `Backend` 抽象层（ABC + `NumericalBackend` + `create_backend` 工厂 + `BackendType`）。
- 删除 deprecated 的 `_solve_with_casadi_backend`（约 294 行）及其触发分支。
- 堵住 `fourier + numerical` 的 `AttributeError` 崩溃路径。

**Non-Goals:**
- 不改 spline 的 cyipopt 求解路径（`create_ipopt_problem` → `RobotIPOPTSolver`）。
- 不改 fourier strategy 的 NLP 构建逻辑与对 CasadiBackend 的 4 个接口调用方式。
- 不重构 `CasadiBackend` 的符号模型构建/缓存机制（`_ensure_symbolic_model`、缓存键、`~/.figaroh/casadi_cache/`）。
- 不做向后兼容：`backend=` 参数与 YAML `backend:` 键直接删除，无 deprecation 过渡期。

## Decisions

### D1: backend 选择策略改为 trajectory_type 驱动

在 `BaseOptimalTrajectory.__init__` 中，依据已加载的 `trajectory_config["trajectory_type"]` 决定是否创建 backend：

```
traj_type = self.trajectory_config.get("trajectory_type", "spline")
if traj_type == "fourier":
    self._backend = CasadiBackend(robot=robot)        # 直接 import 实例化
else:  # spline
    self._backend = None                               # spline 不需要
```

**为何直接 import 而非保留工厂**：删除 `create_backend` 工厂后，`CasadiBackend` 是唯一实现，工厂层不再有分发价值。直接 `from figaroh.backend.casadi import CasadiBackend` 更直接，且把 CasADi 缺失的 `ImportError` 推迟到 fourier 实际选用时（spline 用户无需装 casadi）。

**备选（已否决）**：保留 `create_backend("casadi")` 工厂作为内部细节。否决理由——工厂此时只剩单一字符串分支，是多余间接层，与"移除伪自由度"目标相悖。

### D2: `CasadiBackend` 去 ABC 化，收窄对外契约

`CasadiBackend` 不再继承 `Backend`，删除以下 ABC 抽象方法实现（经查证 fourier 路径零调用）：

| 方法 | 行号 | fourier 是否用 | 处置 |
|------|------|---------------|------|
| `build_regressor` | casadi.py:406 | 否 | 删除 |
| `gradient` | casadi.py:507 | 否 | 删除 |
| `jacobian` | casadi.py:528 | 否 | 删除 |
| `create_solver` | casadi.py:549 | 否 | 删除 |
| `name` | casadi.py:615 | 否 | 删除 |
| `regressor_is_jacobian_of_rnea` | casadi.py:368 | 否（全工程零调用） | 删除 |

保留 fourier strategy 实际依赖的成员（调用点见 `fourier_strategy.py:78,79,140,174`）：

- `_ensure_symbolic_model()`（含缓存加载/构建逻辑）
- `self._cmodel`（`cpin.Model`）
- `regressor_function`（property → `self._W_fun`）
- `rnea_function`（property → `self._rnea_fun`）

**为何删 `name`**：`name` 仅被 `solve_with_waypoints` 的 deprecated 分支（L973 `if backend.name == "casadi"`）读取，该分支随 D3 一并删除后无任何调用者。

**注意 `_cdata`**：fourier strategy 不用 `_cdata`（仅 deprecated 方法 L865 用），但 `_cdata` 是 `_ensure_symbolic_model` 内部 rnea 构建的中间产物，作为符号模型状态保留，不删。

### D3: 删除 deprecated `_solve_with_casadi_backend` 及触发分支

- 删除 `base_optimal_trajectory.py:664-958`（`_solve_with_casadi_backend` 方法体，约 294 行）。
- 删除触发分支（**归属修正**：该分支位于 `BaseTrajectoryIPOPTProblem.solve_with_waypoints`——`base_optimal_trajectory.py` L570 类、L957 方法、L971-974 行，通过 `self.opt_traj._backend` 访问外层 `BaseOptimalTrajectory` 实例）：
  ```python
  # L971-974, BaseTrajectoryIPOPTProblem.solve_with_waypoints
  backend = self.opt_traj._backend
  if backend.name == "casadi":
      return self._solve_with_casadi_backend(wps)
  ```
  删除后 `solve_with_waypoints` 直接走 cyipopt 路径（`RobotIPOPTSolver`），与 spline 的真实行为一致。

```

Full source: openspec/changes/remove-trajectory-backend-optionality/design.md

## openspec/changes/remove-trajectory-backend-optionality/tasks.md

- Source: openspec/changes/remove-trajectory-backend-optionality/tasks.md
- Lines: 1-53
- SHA256: 90dc740cb8218e1b7bc4921b61590e1283ef2cb324768df42ca3ff6a6fcd3f3f

```md
# Tasks: Remove Trajectory Backend Optionality

## 1. 删除 backend 抽象层（base / numerical）

- [ ] 1.1 删除 `src/figaroh/backend/base.py`（`Backend` ABC、`BackendType`、`create_backend` 工厂、末尾 `NumericalBackend` import）
- [ ] 1.2 删除 `src/figaroh/backend/numerical.py`（`NumericalBackend` 死代码，生产零调用）

## 2. 改造 CasadiBackend（去 ABC 化）

- [ ] 2.1 移除 `CasadiBackend` 的 `Backend` 继承（`class CasadiBackend(Backend)` → `class CasadiBackend`）
- [ ] 2.2 删除 ABC 死方法：`build_regressor`（casadi.py:406）、`gradient`（:507）、`jacobian`（:528）、`create_solver`（:549）、`name`（:615）
- [ ] 2.3 删除 `regressor_is_jacobian_of_rnea`（casadi.py:368，全工程零调用）
- [ ] 2.4 删除 `ColumnEliminationCallback`（casadi.py:166，孤儿代码，无生产调用者）
- [ ] 2.5 核对保留成员完整：`__init__` / `_cache_dir` / `_cache_key` / `_ensure_symbolic_model` / `regressor_function` / `rnea_function`，及内部状态 `_cmodel` / `_cdata` / `_W_fun` / `_rnea_fun`

## 3. 瘦身 backend 子包 __init__

- [ ] 3.1 改造 `src/figaroh/backend/__init__.py`：移除 `base`/`numerical` import，`__all__` 收窄为 `["CasadiBackend"]`，保留 `CasadiBackend` 的 try/except 懒加载
- [ ] 3.2 确认 `src/figaroh/__init__.py` 的 `from . import backend` 仍可正常 import

## 4. 改造 BaseOptimalTrajectory 入口

- [ ] 4.1 移除 `base_optimal_trajectory.py:37` 的 `from figaroh.backend.base import BackendType, create_backend`，改为 `from figaroh.backend.casadi import CasadiBackend`
- [ ] 4.2 移除 `__init__`（L68-69）的 `backend: BackendType = "numerical"` 形参
- [ ] 4.3 移除 L89-97 的三级优先级 backend 解析逻辑，替换为：`traj_type == "fourier"` → `self._backend = CasadiBackend(robot=robot)`，否则 `self._backend = None`
- [ ] 4.4 删除 deprecated 方法 `_solve_with_casadi_backend`（L664-958，约 294 行）
- [ ] 4.5 删除 `BaseTrajectoryIPOPTProblem.solve_with_waypoints`（L570 类、L957 方法）中 L971-974 的触发分支（`backend = self.opt_traj._backend` / `if backend.name == "casadi"` / `return self._solve_with_casadi_backend(wps)`；通过 `self.opt_traj._backend` 访问外层 `BaseOptimalTrajectory` 实例）

## 5. 改造 config 加载器

- [ ] 5.1 移除 `src/figaroh/optimal/config.py:85` 的 `"backend": config["identification"].get("backend", "numerical")`
- [ ] 5.2 移除 `src/figaroh/optimal/config.py:126` 的 `"backend": problem_params.get("backend", "numerical")`
- [ ] 5.3 grep 全工程确认无其他 `trajectory_config["backend"]` 或 `identif_config["backend"]` 读取点残留

## 6. 调整测试

- [ ] 6.1 删除 `tests/unit/test_backend.py` 中针对 `Backend` ABC 的测试（TestBackend 抽象类、create_backend 工厂测试：test_create_backend_*、test_create_backend_passthrough、test_create_backend_invalid_string、test_create_backend_casadi_without_deps 等）
- [ ] 6.2 删除 `tests/unit/test_backend.py` 中针对 `NumericalBackend` 的测试（TestNumericalBackend 整个类、test_create_backend_numerical*）
- [ ] 6.3 删除 `tests/unit/test_backend.py` 中针对 `ColumnEliminationCallback` 的测试（L342 附近）
- [ ] 6.4 保留并改造 `test_backend.py` 中 `CasadiBackend` 测试（TestCasadiBackend）：去掉对 ABC 死方法（build_regressor/gradient/jacobian/create_solver/name）的断言，仅保留 `_ensure_symbolic_model` / `regressor_function` / `rnea_function` / 缓存相关测试
- [ ] 6.5 删除 `tests/unit/test_config.py:90-155` 中验证 backend 参数传递/优先级的测试（test_config_backend_*，含已失效的旧 active_joints 签名测试）
- [ ] 6.6 删除 `tests/unit/test_fourier_e2e.py:83,108` 的 `backend="casadi"` 传参
- [ ] 6.7 校验 `tests/unit/test_fourier_strategy.py` 的 mock CasadiBackend 接口（L97 附近）与新 CasadiBackend 契约一致
- [ ] 6.8 修复保留测试的旧签名错配：`tests/unit/test_fourier_e2e.py`（L83,108）将 `BaseOptimalTrajectory(simple_robot, ["joint_1"], config_file=...)` 改为 `BaseOptimalTrajectory(simple_robot, config_file=...)`（移除 `active_joints` 位置参数，对齐当前签名；修复重构前已存在的 `TypeError: got multiple values for argument 'config_file'`）
- [ ] 6.9 确认 `tests/unit/test_config.py::TestCreateConfigBackend`（仅测 `create_config()`，不构造 `BaseOptimalTrajectory`，重构前已通过）在删除 `create_config` 的 backend 解析后仍通过；若其断言依赖 backend 键则同步删除

## 7. 验证

- [ ] 7.1 casadi 环境运行 `pixi run -e casadi pytest tests/unit/`，确认除明确删除的 backend 测试外全绿
- [ ] 7.2 默认/测试环境运行全量 pytest，确认 spline 路径与现有 212 测试不受影响
- [ ] 7.3 grep 全工程确认 `Backend`、`BackendType`、`create_backend`、`NumericalBackend`、`ColumnEliminationCallback`、`_solve_with_casadi_backend` 不再出现在源码中
- [ ] 7.4 手动验收：casadi 环境 F5 启动 `figaroh-examples/examples/ur10/optimal_trajectory.py`（fourier 配置），确认进入 `FourierOptimizationStrategy.solve()` 不抛 `AttributeError`，`CasadiBackend` 被自动创建
- [ ] 7.5 手动验收：`BaseOptimalTrajectory(robot, config_file, backend="casadi")` 抛 `TypeError: unexpected keyword argument 'backend'`
```

## openspec/changes/remove-trajectory-backend-optionality/specs/excitation-trajectory-optimization/spec.md

- Source: openspec/changes/remove-trajectory-backend-optionality/specs/excitation-trajectory-optimization/spec.md
- Lines: 1-39
- SHA256: 446d5ca19c3ffbddcaac8fab8b4544d0722a8a8d490c9285053c36e5d3e5469e

```md
## ADDED Requirements

### Requirement: Backend determined by trajectory type

The system SHALL NOT expose a user-configurable `backend` parameter on `BaseOptimalTrajectory`. The computation backend SHALL be determined automatically and exclusively by the `trajectory_type` configuration field. When `trajectory_type` is `"fourier"`, the system SHALL instantiate `CasadiBackend` directly (no factory indirection). When `trajectory_type` is `"spline"` or absent, the system SHALL NOT instantiate any backend (the spline pipeline uses cyipopt via `RobotIPOPTSolver` and does not consume a backend abstraction).

The `trajectory_config` dictionary SHALL NOT contain a `backend` key, and the configuration loader SHALL NOT parse any `backend` entry from YAML.

#### Scenario: Fourier auto-selects CasADi backend

- **WHEN** `trajectory_type: "fourier"` is configured
- **THEN** `BaseOptimalTrajectory.__init__` SHALL instantiate `CasadiBackend(robot=robot)` and store it as `self._backend`
- **AND** SHALL NOT require the caller to pass any `backend` argument

#### Scenario: Spline creates no backend

- **WHEN** `trajectory_type` is `"spline"` or absent
- **THEN** `BaseOptimalTrajectory.__init__` SHALL set `self._backend = None`
- **AND** the cubic spline pipeline SHALL proceed unchanged via `create_ipopt_problem` → `RobotIPOPTSolver`

#### Scenario: Explicit backend argument rejected

- **WHEN** a caller invokes `BaseOptimalTrajectory(robot, config_file, backend="casadi")` or `backend="numerical"`
- **THEN** the call SHALL raise `TypeError` (unexpected keyword argument `backend`)
- **AND** no silent acceptance or deprecation warning SHALL be emitted

#### Scenario: YAML backend key ignored

- **WHEN** a YAML configuration file contains a `backend:` key under `problem` or `identification.trajectory_params`
- **THEN** the loader SHALL NOT parse the key into `trajectory_config`
- **AND** the effective backend SHALL be determined solely by `trajectory_type`

## REMOVED Requirements

### Requirement: Backward compatibility with existing spline pipeline

**Reason**: The requirement asserted that "all existing configuration files... and API calls SHALL continue to work without modification". This is superseded by the removal of the `backend` parameter: `BaseOptimalTrajectory` no longer accepts `backend=`, which is a breaking change to its API. The spline *pipeline behavior* itself (cyipopt path) is preserved, but the public API surface for backend selection is intentionally broken to eliminate a pseudo-optionality that produced only crash paths.

**Migration**: Callers passing `backend=` to `BaseOptimalTrajectory` must remove the argument; the backend is now auto-selected by `trajectory_type`. YAML files using a `backend:` key must remove it (no current config file in the repository uses this key, so no file migration is required). No runtime deprecation period is provided.
```

## openspec/changes/remove-trajectory-backend-optionality/specs/symbolic-casadi-pipeline/spec.md

- Source: openspec/changes/remove-trajectory-backend-optionality/specs/symbolic-casadi-pipeline/spec.md
- Lines: 1-36
- SHA256: a1ac68222a545a7d0a4dad2f471e0221bf5205e1ba4b4e9c1a3b6f8bf339e953

```md
## MODIFIED Requirements

### Requirement: CasADi single source of truth

The system SHALL expose pre-built symbolic dynamics functions via `CasadiBackend` property interfaces (`regressor_function` and `rnea_function`), providing a single source of truth for both trajectory optimization and parameter identification. External code SHALL NOT directly access private attributes of `CasadiBackend` (other than `_cmodel`, which is read by `FourierOptimizationStrategy` to obtain joint dimensions `nq`/`nv`).

`CasadiBackend` SHALL NOT inherit from any abstract base class and SHALL NOT implement the former `Backend` ABC interface methods (`build_regressor`, `gradient`, `jacobian`, `create_solver`, `name`). Those methods are removed as they had no production callers; only the symbolic-model interfaces consumed by `FourierOptimizationStrategy` SHALL remain: `_ensure_symbolic_model()`, `_cmodel`, `regressor_function`, and `rnea_function`.

The `Backend` ABC, `BackendType`, `create_backend` factory, and `NumericalBackend` SHALL NOT exist in the codebase after this change. `FourierOptimizationStrategy` SHALL obtain its `CasadiBackend` instance via `context._backend`, which `BaseOptimalTrajectory` instantiates directly when `trajectory_type == "fourier"`.

#### Scenario: Fourier strategy accesses dynamics via property

- **WHEN** `FourierOptimizationStrategy` requires the symbolic regressor or RNEA function
- **THEN** it SHALL obtain them via `context._backend.regressor_function` and `context._backend.rnea_function` properties
- **AND** SHALL access `context._backend._cmodel` only to read joint dimensions (`nq`, `nv`)
- **AND** SHALL NOT access `_W_fun`, `_rnea_fun`, or other private attributes directly

#### Scenario: CasadiBackend instantiated directly for fourier

- **WHEN** `BaseOptimalTrajectory` is constructed with `trajectory_type == "fourier"`
- **THEN** it SHALL instantiate `CasadiBackend(robot=robot)` directly via `from figaroh.backend.casadi import CasadiBackend`
- **AND** create_backend factory SHALL NOT be invoked (it no longer exists)

#### Scenario: Removed ABC methods not present

- **WHEN** the `CasadiBackend` class is inspected
- **THEN** it SHALL NOT define `build_regressor`, `gradient`, `jacobian`, `create_solver`, `name`, or `regressor_is_jacobian_of_rnea`
- **AND** ColumnEliminationCallback SHALL NOT exist in the codebase (it had no production callers)

## REMOVED Requirements

### Requirement: Extensible additional parameter columns

**Reason**: This requirement referenced "the numerical backend's column ordering" as a contract anchor. With the removal of `NumericalBackend` (dead code, no production callers) and the `Backend` abstraction layer, there is no longer a "numerical backend" whose column ordering could anchor the symbolic regressor's column structure. The column-extension *behavior* itself (dynamic column appending based on `has_friction`/`has_actuator_inertia`/`has_joint_offset`/`has_custom_parameters` flags) is preserved in `CasadiBackend`'s symbolic regressor construction, but the cross-backend column-ordering parity contract is voided because only one backend remains.

**Migration**: No code migration required — `CasadiBackend`'s regressor construction continues to append friction/actuator-inertia/offset columns in the same order. Callers that relied on a `NumericalBackend` instance for column ordering must remove that reference, as `NumericalBackend` no longer exists. This requirement will be re-established with backend-independent wording in a follow-up if cross-backend parity becomes relevant again.
```

