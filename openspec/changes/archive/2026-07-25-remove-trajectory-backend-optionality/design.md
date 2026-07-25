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

### D4: `ColumnEliminationCallback` 一并删除

经查证，`ColumnEliminationCallback`（casadi.py:166）：
- fourier strategy 不引用
- deprecated `_solve_with_casadi_backend` 不引用（它用内联的 `_SplineCb`）
- `CasadiBackend.build_regressor`（死方法）不引用
- 仅 `test_backend.py:342` 测试

属孤儿代码，随本次重构删除。

### D5: `backend` 子包结构保留但瘦身

删除 `base.py` 与 `numerical.py` 后，`src/figaroh/backend/` 仅剩：

```
src/figaroh/backend/
├── __init__.py     # 仅导出 CasadiBackend
└── casadi.py       # CasadiBackend（去 ABC 化）
```

**为何保留子包而非内联到 `optimal/`**：`CasadiBackend` 承载符号模型构建/磁盘缓存等独立基础设施职责，与 `optimal/` 的轨迹优化编排逻辑正交；保留 `figaroh.backend.casadi` 命名空间使依赖方向清晰（`optimal` → `backend.casadi`），且 `figaroh/__init__.py` 的 `from . import backend` 无需改动。

`__init__.py` 的 `__all__` 收窄为 `["CasadiBackend"]`，移除对 `base`/`numerical` 的 import 与 try/except 包裹。

### D6: YAML `backend:` 键处置

配置加载器（`optimal/config.py` L85、L126）停止解析 `backend` 键。

**为何"忽略"而非"报错"**：现有配置文件均未使用该键（全工程 grep 零命中），不存在迁移负担。若改为"未识别键报错"会扩大 config 校验逻辑改动范围，超出本次重构目标。沿用 yaml 宽松语义：未消费的键静默忽略。

## Risks / Trade-offs

- **[Breaking change 无过渡期]** → 任何外部代码若显式传 `backend=` 或读 `Backend`/`NumericalBackend`/`create_backend` 将直接报错。缓解：项目内全量 grep 确认仅测试与内部代码引用，无外部消费者；release notes 标注 breaking。
- **[test_backend.py 大幅删减可能误删仍有效的 CasadiBackend 测试]** → 逐方法核对 fourier 真实调用面，仅保留与 `_ensure_symbolic_model`/`regressor_function`/`rnea_function`/缓存相关的测试，删除针对 4 个 ABC 抽象方法及 `ColumnEliminationCallback` 的测试。在删除前后运行测试套件比对。
- **[`_cdata` 保留但 fourier 不用]** → 保守保留，避免触动 `_ensure_symbolic_model` 内部 rnea 构建逻辑（该逻辑同时填充 `_rnea_fun`，后者 fourier 在用）。属于"不过度重构"的取舍。
- **[`CasadiBackend` 仍叫 "Backend" 但已无 ABC]** → 命名保留，避免大面积 import 改动；语义上它现在是 fourier 专用的符号模型持有者，不再是"可替换后端"。
- **[旧 change `casadi-optional-backend` 状态畸形]** → 本 change 不触碰旧 change 目录，二者独立。旧 change 的已完成产物（`backend/` 包、`CasadiBackend` 主体）被本 change 作为已存在基础引用并改造。

## Migration Plan

无需数据迁移。部署步骤：

1. 应用源码改动（删除 `base.py`/`numerical.py`，改造 `casadi.py`/`__init__.py`，改 `base_optimal_trajectory.py`/`config.py`）。
2. 调整测试（删减 `test_backend.py`、`test_config.py`、`test_fourier_e2e.py`，校验 `test_fourier_strategy.py` mock 接口）。
3. 运行 `pixi run -e casadi pytest`（casadi 环境含 fourier 路径依赖）+ 默认环境测试，确认除明确删除的 backend 相关测试外全绿。
4. 手动验收：casadi 环境 F5 启动 `optimal_trajectory.py`（fourier 配置），确认进入 `FourierOptimizationStrategy.solve()` 不抛 `AttributeError`。

回滚：git revert 单个 commit 即可，无副作用状态。

## Open Questions

4 个关键未知项已在 design 阶段全部查证并定案：

1. **CasadiBackend ABC 死方法删除边界** → `build_regressor`/`gradient`/`jacobian`/`create_solver`/`name`/`regressor_is_jacobian_of_rnea` 全删（D2）。
2. **`backend` 子包是否保留** → 保留并瘦身（D5）。
3. **`ColumnEliminationCallback` 去留** → 删除（D4，孤儿代码）。
4. **过期测试清理** → `test_config.py` L90-155 的 backend 参数/优先级测试删除（这些测试已用旧 `active_joints` 签名，本身已失效）；`test_backend.py` 中 ABC/NumericalBackend/ColumnEliminationCallback 测试删除，CasadiBackend 测试去 ABC 断言。

无遗留 open question。
