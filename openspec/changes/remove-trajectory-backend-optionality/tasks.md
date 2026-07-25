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
