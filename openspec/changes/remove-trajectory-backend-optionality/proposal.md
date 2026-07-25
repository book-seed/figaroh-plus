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
