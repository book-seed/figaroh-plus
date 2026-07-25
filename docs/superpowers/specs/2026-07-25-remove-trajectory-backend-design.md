---
comet_change: remove-trajectory-backend-optionality
role: technical-design
canonical_spec: openspec
---

# Design Doc: Remove Trajectory Backend Optionality

> 本 Design Doc 是 `openspec/changes/remove-trajectory-backend-optionality/` 三件套（proposal/design/tasks）+ delta specs 的 Superpowers 侧技术设计沉淀。Canonical spec 为 OpenSpec delta specs；本文不重复需求，仅记录实现方案、风险、测试策略与回写 OpenSpec 的 Spec Patch。

## 1. 背景与问题

`BaseOptimalTrajectory` 暴露 `backend` 可配置参数（`backend: BackendType = "numerical"`），由旧 change `casadi-optional-backend` 引入。其设计意图是用 `Backend` ABC（`build_regressor`/`gradient`/`jacobian`/`create_solver` 四抽象方法 + `name`）统一数值与符号两条求解路径，并通过 `create_backend` 工厂按 `numerical`/`casadi` 字符串解析。

落地后该抽象两端都未接上：

- **fourier 路径**用的是 `CasadiBackend` 的**非抽象专属接口**（`_ensure_symbolic_model` / `_cmodel` / `regressor_function` / `rnea_function`），ABC 的 4 个抽象方法在 fourier strategy 里零调用。
- **spline 路径**完全绕开 backend 抽象，直接走 `create_ipopt_problem` → `RobotIPOPTSolver`（cyipopt）；`self._backend` 仅在 `solve_with_waypoints` 开头被读一次以决定是否进入 deprecated 分支。
- `NumericalBackend` 在生产代码中零调用，仅 `test_backend.py` 测试覆盖。

结果 `trajectory_type × backend` 四组合里只有两种合法，且合法组合完全由 `trajectory_type` 决定；`fourier + numerical`（UR10 默认配置触发）直接 `AttributeError`，`spline + casadi` 进入作者已弃用的 `_solve_with_casadi_backend`。`backend` 参数是「伪自由度」——它给了用户选择权，但唯一合法选择由 `trajectory_type` 唯一决定，其余选择只会崩溃或进入 deprecated 路径。

## 2. 已确认的技术方案（6 项决策）

6 项决策（D1–D6）经代码逐一核对全部成立：

### D1: backend 选择策略改为 trajectory_type 驱动

在 `BaseOptimalTrajectory.__init__` 中，依据已加载的 `trajectory_config["trajectory_type"]` 决定是否创建 backend：

```python
traj_type = self.trajectory_config.get("trajectory_type", "spline")
if traj_type == "fourier":
    self._backend = CasadiBackend(robot=robot)        # 直接 import 实例化
else:  # spline
    self._backend = None                               # spline 不需要
```

- 删除 `create_backend` 工厂层，直接 `from figaroh.backend.casadi import CasadiBackend`，把 CasADi 缺失的 `ImportError` 推迟到 fourier 实际选用时（spline 用户无需装 casadi）。
- **已否决备选**：保留 `create_backend("casadi")` 工厂作为内部细节——工厂此时只剩单一字符串分支，是多余间接层，与「移除伪自由度」目标相悖。

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
- 内部状态 `_cdata` / `_W_fun` / `_rnea_fun`（其中 `_cdata` 是 `_ensure_symbolic_model` 内部 rnea 构建中间产物，保守保留避免触动 `_ensure_symbolic_model` 内部逻辑——该逻辑同时填充 fourier 在用的 `_rnea_fun`）

**为何删 `name`**：`name` 仅被 deprecated 分支 `if backend.name == "casadi"` 读取，该分支随 D3 一并删除后无任何调用者。

### D3: 删除 deprecated `_solve_with_casadi_backend` 及触发分支

- 删除 `base_optimal_trajectory.py:664-958`（`_solve_with_casadi_backend` 方法体，约 294 行），该方法属于 `BaseOptimalTrajectory`。
- 删除触发分支，**归属修正**：该分支位于 `BaseTrajectoryIPOPTProblem.solve_with_waypoints`（`base_optimal_trajectory.py` L570 类、L957 方法、L971-974 行），通过 `self.opt_traj._backend` 访问外层 `BaseOptimalTrajectory` 实例：
  ```python
  # L971-974, BaseTrajectoryIPOPTProblem.solve_with_waypoints
  backend = self.opt_traj._backend
  if backend.name == "casadi":
      return self._solve_with_casadi_backend(wps)
  ```
  删除后 `solve_with_waypoints` 直接走 cyipopt 路径（`RobotIPOPTSolver`），与 spline 的真实行为一致。

### D4: `ColumnEliminationCallback` 一并删除

经查证 `ColumnEliminationCallback`（casadi.py:166）为孤儿代码：

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

**为何「忽略」而非「报错」**：现有配置文件均未使用该键（全工程 grep 零命中），不存在迁移负担。若改为「未识别键报错」会扩大 config 校验逻辑改动范围，超出本次重构目标。沿用 yaml 宽松语义：未消费的键静默忽略。

## 3. 关键取舍与风险

- **Breaking 无过渡期**：`backend=` 参数与 YAML `backend:` 键直接删，无 deprecation 期。项目内全量 grep 确认仅测试与内部代码引用，无外部消费者；release notes 标注 breaking。
- **重构前测试套件已有 5 个失败**：`test_config.py::TestBaseOptimalTrajectoryBackendPrecedence`（3）+ `test_fourier_e2e.py::TestFourierPipeline`（2）均因旧 `active_joints` 位置参数签名错配而 `TypeError: got multiple values for argument 'config_file'`。本 change 删除/修正这些测试是「修红而非制造红」——保留的测试在删 `backend=` 同时改签名为 `BaseOptimalTrajectory(robot, config_file=...)`（移除 `["joint_1"]` 位置参数），纳入本 change 范围。
- **测试签名修复纳入范围**：保留的 `test_fourier_e2e.py`、`test_config.py::TestCreateConfigBackend`（仅测 `create_config()`，不构造 `BaseOptimalTrajectory`，目前通过）等测试在删 `backend=` 同时改签名。
- **`_cdata` 保留但 fourier 不用**：保守保留避免触动 `_ensure_symbolic_model` 内部 rnea 构建逻辑（同时填充 `_rnea_fun`，fourier 在用）。属于「不过度重构」的取舍。
- **`CasadiBackend` 命名保留**：虽已无 ABC，但不改名避免大面积 import 改动；语义上现为 fourier 专用符号模型持有者，不再是「可替换后端」。
- **旧 change `casadi-optional-backend` 状态畸形**：本 change 不触碰旧 change 目录，二者独立。旧 change 的已完成产物（`backend/` 包、`CasadiBackend` 主体）被本 change 作为已存在基础引用并改造。

## 4. 测试策略

- **casadi 环境运行** `pixi run -e casadi pytest tests/unit/`：确认除明确删除的 backend 测试外全绿（fourier 路径依赖 casadi 环境的符号模型）。
- **默认/测试环境运行全量 pytest**：确认 spline 路径与现有 212 测试不受影响。
- **grep 全工程**确认 `Backend` / `BackendType` / `create_backend` / `NumericalBackend` / `ColumnEliminationCallback` / `_solve_with_casadi_backend` 不再出现在源码。
- **手动验收 1**：casadi 环境 F5 启动 `figaroh-examples/examples/ur10/optimal_trajectory.py`（fourier 配置），确认进入 `FourierOptimizationStrategy.solve()` 不抛 `AttributeError`，`CasadiBackend` 被自动创建。
- **手动验收 2**：`BaseOptimalTrajectory(robot, config_file, backend="casadi")` 抛 `TypeError: unexpected keyword argument 'backend'`。

### 测试调整明细

| 测试文件 | 处置 |
|---------|------|
| `test_backend.py` | 删除 `Backend` ABC 测试、`NumericalBackend` 测试、`ColumnEliminationCallback` 测试；保留 `TestCasadiBackend` 但去掉对 ABC 死方法的断言，仅保留 `_ensure_symbolic_model`/`regressor_function`/`rnea_function`/缓存测试 |
| `test_config.py` | 删除 L90-155 `TestBaseOptimalTrajectoryBackendPrecedence`（backend 参数传递/优先级 + 已失效旧 active_joints 签名）；保留 `TestCreateConfigBackend`（仅测 `create_config()`） |
| `test_fourier_e2e.py` | 删除 L83,108 `backend="casadi"`；修复 `BaseOptimalTrajectory(simple_robot, ["joint_1"], config_file=...)` 签名为 `BaseOptimalTrajectory(simple_robot, config_file=...)` |
| `test_fourier_strategy.py` | 校验 L97 附近 mock `CasadiBackend` 接口与新契约一致 |

## 5. Spec Patch（回写 OpenSpec delta spec）

两处纯歧义修正（不改需求语义），回写到 `specs/symbolic-casadi-pipeline/spec.md`：

- `**AND\` create_backend\` factory` → `**AND** create_backend factory`（行尾反引号吞掉了闭合 `**`）
- `**AND\` ColumnEliminationCallback\`` → `**AND** ColumnEliminationCallback`

delta spec 内容（ADDED/MODIFIED/REMOVED requirements）已由 open 阶段产出，覆盖以下验收场景：

- `excitation-trajectory-optimization/spec.md`：ADDED「Backend determined by trajectory type」（4 scenarios）；REMOVED「Backward compatibility with existing spline pipeline」。
- `symbolic-casadi-pipeline/spec.md`：MODIFIED「CasADi single source of truth」（3 scenarios）；MODIFIED「Symbolic regressor matrix construction」（把原引用 numerical backend 列顺序的 AND 子句改为 backend-independent 固定列顺序）。原拟的 REMOVED「Extensible additional parameter columns」经核对主 spec 不存在该独立 requirement（相关内容嵌在「Symbolic regressor matrix construction」的 scenario 子句中），故改为 MODIFIED 同一 requirement 而非 REMOVED。


## 6. 实施顺序

按 tasks.md 7 组推进：

1. 删除 backend 抽象层（base/numerical）
2. 改造 CasadiBackend（去 ABC 化）
3. 瘦身 backend 子包 `__init__`
4. 改造 BaseOptimalTrajectory 入口
5. 改造 config 加载器
6. 调整测试
7. 验证

无数据迁移；回滚 = git revert 单个 commit。
