# Brainstorm Summary

- Change: remove-trajectory-backend-optionality
- Date: 2026-07-25

## 确认的技术方案

6 项决策（D1-D6）经代码逐一核对全部成立：

- **D1**: backend 由 trajectory_type 驱动。`fourier` → `CasadiBackend(robot=robot)` 直接 import；`spline` → `self._backend = None`。删 `create_backend` 工厂，直接 `from figaroh.backend.casadi import CasadiBackend`，把 CasADi 缺失的 ImportError 推迟到 fourier 实际选用时。
- **D2**: `CasadiBackend` 去 ABC 化。删 6 死方法（`build_regressor`/`gradient`/`jacobian`/`create_solver`/`name`/`regressor_is_jacobian_of_rnea`），保留 `_ensure_symbolic_model`/`_cmodel`/`regressor_function`/`rnea_function`。`_cdata` 作为 `_ensure_symbolic_model` 内部 rnea 构建中间产物保留。
- **D3**: 删 `_solve_with_casadi_backend`（约 294 行）及其触发分支。**归属修正**：触发分支在 `BaseTrajectoryIPOPTProblem.solve_with_waypoints`（L570 类、L957 方法），通过 `self.opt_traj._backend` 访问外层 `BaseOptimalTrajectory` 实例。删后 `solve_with_waypoints` 直接走 cyipopt。
- **D4**: 删孤儿 `ColumnEliminationCallback`（无生产调用者，仅 deprecated 方法与 test_backend.py 引用，且 deprecated 方法实际用的是内联 `_SplineCb`）。
- **D5**: 保留并瘦身 `backend/` 子包（仅 `casadi.py` + `__init__.py`），`__all__` 收窄为 `["CasadiBackend"]`。
- **D6**: YAML `backend:` 键忽略（非报错），沿用宽松语义。

## 关键取舍与风险

- **Breaking 无过渡期**：`backend=` 参数与 YAML `backend:` 键直接删，无 deprecation 期。项目内全量 grep 确认仅测试与内部代码引用。
- **重构前测试套件已有 5 个失败**：`test_config.py::TestBaseOptimalTrajectoryBackendPrecedence`（3）+ `test_fourier_e2e.py::TestFourierPipeline`（2）均因旧 `active_joints` 位置参数签名错配而 `TypeError: got multiple values for argument 'config_file'`。本 change 删除/修正这些测试是修红而非制造红。
- **测试签名修复纳入范围**：保留的测试在删 `backend=` 同时改签名为 `BaseOptimalTrajectory(robot, config_file=...)`（移除 `["joint_1"]` 位置参数）。
- **`_cdata` 保留但 fourier 不用**：保守保留避免触动 `_ensure_symbolic_model` 内部 rnea 构建逻辑（同时填充 `_rnea_fun`，fourier 在用）。
- **`CasadiBackend` 命名保留**：虽已无 ABC，但不改名避免大面积 import 改动；语义上现为 fourier 专用符号模型持有者。

## 测试策略

- casadi 环境运行 `pixi run -e casadi pytest tests/unit/`：确认除明确删除的 backend 测试外全绿。
- 默认/测试环境运行全量 pytest：确认 spline 路径与现有测试不受影响。
- grep 全工程确认 `Backend`/`BackendType`/`create_backend`/`NumericalBackend`/`ColumnEliminationCallback`/`_solve_with_casadi_backend` 不再出现在源码。
- 手动验收 1：casadi 环境 F5 启动 `optimal_trajectory.py`（fourier 配置），确认进入 `FourierOptimizationStrategy.solve()` 不抛 `AttributeError`。
- 手动验收 2：`BaseOptimalTrajectory(robot, config_file, backend="casadi")` 抛 `TypeError: unexpected keyword argument 'backend'`。

## Spec Patch

修正 `specs/symbolic-casadi-pipeline/spec.md` delta 中两处 markdown 语法错误：
- `**AND\` create_backend\` factory` → `**AND** create_backend factory`
- `**AND\` ColumnEliminationCallback\`` → `**AND** ColumnEliminationCallback`

纯歧义修正，不改需求语义。design.md 的 D3 归属描述与 tasks.md 4.5 也需同步精确化到 `BaseTrajectoryIPOPTProblem.solve_with_waypoints`，并补充"保留测试签名修复"任务到 tasks.md 第 6 组。
