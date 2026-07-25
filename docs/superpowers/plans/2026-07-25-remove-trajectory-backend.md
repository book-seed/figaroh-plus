---
change: remove-trajectory-backend-optionality
design-doc: docs/superpowers/specs/2026-07-25-remove-trajectory-backend-design.md
base-ref: 4c40746a97f8bbdea701cab611e1aa200eb13076
---

# 移除 BaseOptimalTrajectory 的 backend 可配置参数 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: 使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans` 逐任务实施。步骤使用复选框 (`- [x]`) 语法跟踪。

**目标：** 删除 `BaseOptimalTrajectory` 的 `backend` 伪自由度参数及配套 ABC 抽象层，改由 `trajectory_type` 驱动 backend 选择（fourier → `CasadiBackend`，spline → `None`）。

**架构：** 路径 B 彻底重构。删除 `backend/base.py`（`Backend` ABC / `BackendType` / `create_backend` 工厂）与 `backend/numerical.py`（`NumericalBackend` 死代码）；`CasadiBackend` 去 ABC 化、收窄为 fourier 专用符号模型持有者；`backend/__init__.py` 仅导出 `CasadiBackend`；`BaseOptimalTrajectory.__init__` 直接 `from figaroh.backend.casadi import CasadiBackend` 实例化；config 加载器停止解析 `backend` 键。

**技术栈：** Python 3，pytest，CasADi + pinocchio.casadi（仅 casadi pixi 环境可用），cyipopt（spline 路径），OpenMP（fourier `map("openmp")`）。

## 全局约束

- base-ref = `4c40746a97f8bbdea701cab611e1aa200eb13076`；回滚 = `git revert` 单 commit。
- Breaking 无过渡期：`backend=` 参数与 YAML `backend:` 键直接删，无 deprecation 期。Release notes 标注 breaking。
- pixi 多环境：`casadi` 环境含 fourier 路径依赖（源码编译 CasADi+OpenMP）；默认环境跑全量 pytest。
- 重构前测试套件已有 5 个失败（`test_config.py` 3 + `test_fourier_e2e.py` 2），均因旧 `active_joints` 位置参数签名错配 → `TypeError: got multiple values for argument 'config_file'`。本 change 删除/修正这些测试是「修红而非制造红」。
- 保留成员（fourier strategy 实际依赖，见 `fourier_strategy.py:77-79,140,174`）：`_ensure_symbolic_model()` / `_cmodel` / `regressor_function` / `rnea_function` / 内部状态 `_cdata` / `_W_fun` / `_rnea_fun`。`_cdata` 保守保留（不触动 `_ensure_symbolic_model` 内部 rnea 构建逻辑）。
- `CasadiBackend` 命名保留（虽已无 ABC，但不改名以避免大面积 import 改动）。
- Spec Patch（回写 OpenSpec delta spec）：两处纯歧义修正见设计文档 §5，已在 design 阶段回写，不在本实施计划范围。

## 文件结构

| 文件 | 职责 | 本次处置 |
|------|------|---------|
| `src/figaroh/backend/base.py` | `Backend` ABC / `BackendType` / `create_backend` 工厂 | 删除整文件 |
| `src/figaroh/backend/numerical.py` | `NumericalBackend` 死代码 | 删除整文件 |
| `src/figaroh/backend/casadi.py` | `CasadiBackend` 符号模型构建/缓存 | 去 ABC 化，删 6 个死方法 + `ColumnEliminationCallback` |
| `src/figaroh/backend/__init__.py` | 子包导出 | 瘦身为仅导出 `CasadiBackend` |
| `src/figaroh/optimal/base_optimal_trajectory.py` | 轨迹优化编排 | 改入口签名/选 backend、删 deprecated 方法与触发分支 |
| `src/figaroh/optimal/config.py` | YAML 加载 | 停止解析 `backend` 键 |
| `tests/unit/test_backend.py` | backend 单测 | 删 ABC/Numerical/ColumnElim 测试，保留并改造 CasadiBackend 测试 |
| `tests/unit/test_config.py` | config 单测 | 删 backend 优先级/precedence 测试 + `TestCreateConfigBackend`（断言依赖被删键） |
| `tests/unit/test_fourier_e2e.py` | fourier e2e | 删 `backend=`、修签名 |
| `tests/unit/test_fourier_strategy.py` | fourier strategy 单测 | 校验 mock 契约一致 |

## TDD 适用性总览

本 change 是**删减型重构**，多数任务是删除死代码而非新功能。「红」主要体现在**测试删除/调整**而非新增失败测试：

- **纯删除/调整（非 TDD）**：1.1, 1.2, 2.1-2.5, 3.1, 3.2, 4.1, 4.4, 4.5, 5.1, 5.2, 5.3, 6.1-6.6, 6.9
- **修红（修既有红，非制造红）**：6.7, 6.8（含 test_backend.py `TestTrajectoryStrategyIntegration` 4 个测试的同源签名修复——见 6.8 扩展说明）
- **验证型 TDD（先写期望行为测试再改）**：4.2+4.3（backend 参数移除 + trajectory_type 驱动选择）适合先补一个「`backend=` 抛 TypeError」与「fourier 自动建 CasadiBackend」的断言测试，再改实现。

---

## 任务组 1：删除 backend 抽象层（base / numerical）

> 顺序注意：删 `base.py` 会让 `casadi.py:36` 的 `from .base import Backend` 与 `__init__.py:26` 失效。组 1 与组 2、3 应在同一次 commit 周期内完成，避免树处于破损态过夜。本组先做文件删除，紧接组 2 修复引用。

### Task 1.1: 删除 `src/figaroh/backend/base.py`

**Files:**
- Delete: `src/figaroh/backend/base.py`（全文：`Backend` ABC、`BackendType`、`create_backend` 工厂、`_create_casadi_backend`、末尾 `from .numerical import NumericalBackend`）

**Interfaces:**
- Consumes: 无（被删对象）
- Produces: 删除符号 `Backend` / `BackendType` / `create_backend` / `_create_casadi_backend`。下游消费者（`casadi.py:36`、`__init__.py:26`、`base_optimal_trajectory.py:37`、`test_backend.py:8`、`test_config.py` patch 点）将在后续任务修复。

- [x] **Step 1: 删除文件**

```bash
git rm src/figaroh/backend/base.py
```

- [x] **Step 2: 确认删除后残留引用清单（预期全部在后续任务修复）**

Run: `grep -rn "from figaroh.backend.base\|from .base import\|backend.base" src tests`
Expected: 命中 `casadi.py:36`、`__init__.py:26`、`test_backend.py:8`（均在组 2/3/6 修复）。

- [x] **Step 3: 暂不 commit，与组 2 一并提交**

### Task 1.2: 删除 `src/figaroh/backend/numerical.py`

**Files:**
- Delete: `src/figaroh/backend/numerical.py`（全文：`NumericalBackend` 死代码，生产零调用）

**Interfaces:**
- Consumes: 无
- Produces: 删除符号 `NumericalBackend`。消费者 `__init__.py:27`、`base.py:180`（已删）、`test_backend.py:9` 在后续任务修复。

- [x] **Step 1: 删除文件**

```bash
git rm src/figaroh/backend/numerical.py
```

- [x] **Step 2: 确认残留引用**

Run: `grep -rn "NumericalBackend\|backend.numerical" src tests`
Expected: 命中 `__init__.py:27`、`test_backend.py:9`（组 3/6 修复）。

- [x] **Step 3: 暂不 commit**

---

## 任务组 2：改造 CasadiBackend（去 ABC 化）

> 本组修复组 1 删除产生的 `casadi.py:36` 引用，并删除 ABC 死方法与孤儿 `ColumnEliminationCallback`。本组结束与组 1 一并 commit。

### Task 2.1: 移除 `CasadiBackend` 的 `Backend` 继承与 import

**Files:**
- Modify: `src/figaroh/backend/casadi.py:36`（删除 `from .base import Backend`）
- Modify: `src/figaroh/backend/casadi.py:238`（`class CasadiBackend(Backend):` → `class CasadiBackend:`）

**Interfaces:**
- Produces: `class CasadiBackend:`（无基类），构造签名不变 `__init__(self, robot: Any)`。

- [x] **Step 1: 删除 import 行**

删除 `casadi.py:36`：
```python
from .base import Backend
```

- [x] **Step 2: 去除继承**

`casadi.py:238`：
```python
class CasadiBackend:           # 原: class CasadiBackend(Backend):
```

- [x] **Step 3: 更新类 docstring（可选，保持准确）**

将类 docstring 中关于 ABC 抽象方法的暗示性描述去掉，仅保留「symbolic regressor + 缓存」职责描述。

### Task 2.2: 删除 ABC 死方法 build_regressor / gradient / jacobian / create_solver / name

**Files:**
- Modify: `src/figaroh/backend/casadi.py`
  - 删除 `build_regressor`（L406-505）
  - 删除 `gradient`（L507-526）
  - 删除 `jacobian`（L528-547）
  - 删除 `create_solver`（L549-612）
  - 删除 `name` property（L614-617）

**依据：** fourier strategy 零调用这些方法（`fourier_strategy.py` 仅用 `_ensure_symbolic_model` / `_cmodel` / `regressor_function` / `rnea_function`）。

- [x] **Step 1: 删除上述 5 个方法/属性定义**

- [x] **Step 2: 确认删除后无残留引用**

Run: `grep -n "build_regressor\|\.gradient\|\.jacobian\|create_solver\|backend.name\|\.name ==" src/figaroh/backend/casadi.py`
Expected: 无命中（`_map_ipopt_options` / `_IPOPT_OPTION_MAP` 仅被已删的 `create_solver` 使用——一并删除）。

- [x] **Step 3: 删除现已孤儿的 IPOPT 选项映射辅助**

`_IPOPT_OPTION_MAP`（L109-125）、`_map_ipopt_options`（L128-158）仅被 `create_solver` 使用，随之一并删除。删除后确认 `warnings` import 若无其他使用则保留（其他模块可能 import；保守保留 import 行）。

### Task 2.3: 删除 `regressor_is_jacobian_of_rnea`

**Files:**
- Modify: `src/figaroh/backend/casadi.py:367-404`（删除 `regressor_is_jacobian_of_rnea` staticmethod，全工程零调用）

- [x] **Step 1: 删除该方法**

- [x] **Step 2: 验证零调用**

Run: `grep -rn "regressor_is_jacobian_of_rnea" src tests`
Expected: 无命中。

### Task 2.4: 删除 `ColumnEliminationCallback`（孤儿代码）

**Files:**
- Modify: `src/figaroh/backend/casadi.py:161-230`（删除 `ColumnEliminationCallback` 类及其上方注释块 L161-164）

**依据：** fourier strategy 不引用；deprecated `_solve_with_casadi_backend`（组 4 删除）不引用（用内联 `_SplineCb`）；仅 `test_backend.py:342` 测试。

- [x] **Step 1: 删除类定义与注释**

- [x] **Step 2: 清理仅被其使用的 import**

删除后检查 `Optional`（L30 `typing` import）是否仍被使用——`CasadiBackend.__init__` 等不再用 `Optional`。若 `typing` 行仅剩 `Callable, Any, Optional`，移除 `Optional`（`Callable` 也随 `gradient/jacobian/create_solver` 删除而可能不再需要——检查后移除未用符号）。

- [x] **Step 3: 验证零调用**

Run: `grep -rn "ColumnEliminationCallback" src tests`
Expected: 仅 `test_backend.py:343` 命中（组 6.3 删除）。

### Task 2.5: 核对保留成员完整

**Files:** `src/figaroh/backend/casadi.py`（只读核对）

- [x] **Step 1: 核对保留成员存在且未被误删**

确认以下成员仍存在：
- `_CACHE_VERSION`（L252）
- `__init__`（L254，`self._robot`/`_cmodel`/`_cdata`/`_W_fun`/`_rnea_fun`）
- `_cache_dir`（L266）、`_cache_key`（L273）
- `_ensure_symbolic_model`（L291）
- `regressor_function` property（L349）
- `rnea_function` property（L358）

Run: `grep -n "_CACHE_VERSION\|def __init__\|_cache_dir\|_cache_key\|_ensure_symbolic_model\|def regressor_function\|def rnea_function\|_cmodel\|_cdata\|_W_fun\|_rnea_fun" src/figaroh/backend/casadi.py`
Expected: 全部命中。

- [x] **Step 2: Commit 组 1+2**

```bash
git add src/figaroh/backend/casadi.py
git commit -m "refactor(backend): drop Backend ABC, NumericalBackend, ColumnEliminationCallback

Remove base.py (Backend/BackendType/create_backend) and numerical.py.
De-ABC CasadiBackend: drop build_regressor/gradient/jacobian/create_solver/name
and regressor_is_jacobian_of_rnea (all zero-call in fourier path).

Refs: remove-trajectory-backend-optionality D1-D4"
```

注意：此时 `backend/__init__.py` 仍引用已删的 `base`/`numerical`，import 链暂时破损——组 3 立即修复。不要在 commit 前运行 import。

---

## 任务组 3：瘦身 backend 子包 `__init__`

### Task 3.1: 改造 `src/figaroh/backend/__init__.py`

**Files:**
- Modify: `src/figaroh/backend/__init__.py`（全文重写）

**Interfaces:**
- Produces: `figaroh.backend` 子包仅导出 `CasadiBackend`（try/except 懒加载保留，CasADi 缺失时为 `None`）。

- [x] **Step 1: 重写 `__init__.py` 全文**

```python
# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""CasADi symbolic computation backend.

CasadiBackend holds the symbolic regressor / RNEA model built via
``pinocchio.casadi`` and is the sole backend used by the Fourier
trajectory strategy. It is created automatically when
``trajectory_type == 'fourier'``; spline trajectories do not need a
backend.

Dependencies (optional): ``casadi`` + conda-forge ``pinocchio`` with
CasADi bindings. Imported lazily so spline users need not install them.
"""

try:
    from .casadi import CasadiBackend
except ImportError:
    CasadiBackend = None  # type: ignore

__all__ = ["CasadiBackend"]
```

- [x] **Step 2: 验证 import 链恢复**

Run: `pixi run python -c "import figaroh.backend; print(figaroh.backend.CasadiBackend)"`
Expected: 默认环境输出 `None`（无 casadi）；casadi 环境输出类对象。

### Task 3.2: 确认 `figaroh/__init__.py` 的 `from . import backend` 仍可 import

**Files:** `src/figaroh/__init__.py`（不改动，仅验证）

- [x] **Step 1: 验证根包 import**

Run: `pixi run python -c "import figaroh; assert hasattr(figaroh, 'backend'); print('ok')"`
Expected: `ok`

- [x] **Step 2: Commit 组 3**

```bash
git add src/figaroh/backend/__init__.py
git commit -m "refactor(backend): slim backend/__init__ to export only CasadiBackend"
```

---

## 任务组 4：改造 BaseOptimalTrajectory 入口

> 这是行为变更核心。4.2+4.3 先改实现，测试断言在组 6 验证。

### Task 4.1: 更换 import

**Files:**
- Modify: `src/figaroh/optimal/base_optimal_trajectory.py:37`

- [x] **Step 1: 替换 import**

```python
# 原 L37:
# from figaroh.backend.base import BackendType, create_backend
# 改为:
from figaroh.backend.casadi import CasadiBackend
```

### Task 4.2: 移除 `backend` 形参

**Files:**
- Modify: `src/figaroh/optimal/base_optimal_trajectory.py:68-77`（`__init__` 签名与 docstring）

- [x] **Step 1: 修改签名**

```python
    def __init__(self, robot, config_file: str = "config/robot_config.yaml"):
        """Initialize the optimal trajectory generator.

        Args:
            robot: RobotWrapper instance.
            config_file: Path to configuration YAML file.
        """
```

- [x] **Step 2: 删除 docstring 中 `backend:` 行**

### Task 4.3: 替换 backend 解析逻辑为 trajectory_type 驱动

**Files:**
- Modify: `src/figaroh/optimal/base_optimal_trajectory.py:89-97`

- [x] **Step 1: 用 D1 决策替换三级优先级逻辑**

删除 L89-97 三级优先级逻辑，替换为：
```python
        # Backend is determined solely by trajectory_type: only the Fourier
        # strategy needs the CasADi symbolic model; spline trajectories use
        # cyipopt directly and carry no backend.
        traj_type = self.trajectory_config.get("trajectory_type", "spline")
        if traj_type == "fourier":
            self._backend = CasadiBackend(robot=robot)
        else:
            self._backend = None
```

注意：下方已有 `traj_type = self.trajectory_config.get("trajectory_type", "spline")` 用于策略创建。可将两处合并为一次读取以避免重复——若合并，确保 `traj_type` 变量名不与下方冲突（可上移复用）。最小改动则保留两处独立读取（语义等价）。

- [x] **Step 2: 验证语法（仅编译）**

Run: `pixi run python -c "import ast; ast.parse(open('src/figaroh/optimal/base_optimal_trajectory.py').read()); print('ok')"`
Expected: `ok`

### Task 4.4: 删除 deprecated `_solve_with_casadi_backend`

**Files:**
- Modify: `src/figaroh/optimal/base_optimal_trajectory.py:664-955`（删除 `_solve_with_casadi_backend` 方法体，约 292 行，含其内部 `_SplineCb` 等辅助）

- [x] **Step 1: 删除 L664 至方法结束（L955，下一个方法 `solve_with_waypoints` 之前）**

确认结束行：`_solve_with_casadi_backend` 在 L664 `def` 开始，至 L955 `return success, results` 结束（L956 空行，L957 为 `solve_with_waypoints`）。

- [x] **Step 2: 验证零调用残留**

Run: `grep -rn "_solve_with_casadi_backend" src`
Expected: 仅 `solve_with_waypoints` 内 L974 触发分支（组 4.5 删除）。

### Task 4.5: 删除 `solve_with_waypoints` 触发分支

**Files:**
- Modify: `src/figaroh/optimal/base_optimal_trajectory.py:971-974`（`BaseTrajectoryIPOPTProblem.solve_with_waypoints` 内）

- [x] **Step 1: 删除触发分支**

删除 L971-974：
```python
            # Check if backend provides solver override
            backend = self.opt_traj._backend
            if backend.name == "casadi":
                return self._solve_with_casadi_backend(wps)
```

删除后 `solve_with_waypoints` 直接进入 cyipopt 路径（`IPOPTConfig.for_trajectory_optimization()` → `RobotIPOPTSolver`），与 spline 真实行为一致。

- [x] **Step 2: 验证 `backend.name` / `opt_traj._backend` 不再被 `solve_with_waypoints` 读取**

Run: `grep -n "backend.name\|opt_traj._backend" src/figaroh/optimal/base_optimal_trajectory.py`
Expected: `solve_with_waypoints` 无命中（仅 `__init__` 中 `self._backend = ...` 保留）。

- [x] **Step 3: Commit 组 4**

```bash
git add src/figaroh/optimal/base_optimal_trajectory.py
git commit -m "refactor(optimal): drive backend by trajectory_type, drop backend= param

BaseOptimalTrajectory.__init__ no longer takes backend=; fourier creates
CasadiBackend(robot=robot), spline sets None. Remove deprecated
_solve_with_casadi_backend and its solve_with_waypoints trigger.

Refs: remove-trajectory-backend-optionality D1, D3"
```

---

## 任务组 5：改造 config 加载器

### Task 5.1: 移除 legacy 路径 `backend` 键解析

**Files:**
- Modify: `src/figaroh/optimal/config.py:85`

- [x] **Step 1: 删除该行**

删除 L85：
```python
                "backend": config["identification"].get("backend", "numerical"),
```

### Task 5.2: 移除 unified 路径 `backend` 键解析

**Files:**
- Modify: `src/figaroh/optimal/config.py:126`

- [x] **Step 1: 删除该行**

删除 L126：
```python
        "backend": problem_params.get("backend", "numerical"),
```

### Task 5.3: grep 全工程确认无其他 `backend` 键读取点残留

**Files:** 全工程（只读核对）

- [x] **Step 1: 搜索 trajectory_config / identif_config 的 backend 读取点**

Run: `grep -rn 'trajectory_config\["backend"\]\|identif_config\["backend"\]\|\.get("backend"' src`
Expected: 无命中。

- [x] **Step 2: 搜索 YAML 配置文件中的 `backend:` 键**

Run: `grep -rn "backend:" --include="*.yaml" --include="*.yml" .`
Expected: 无命中（设计文档确认全工程零命中）。若有命中则属遗留配置，按 D6 静默忽略语义保留文件不改。

- [x] **Step 3: Commit 组 5**

```bash
git add src/figaroh/optimal/config.py
git commit -m "refactor(config): stop parsing backend key from YAML

Refs: remove-trajectory-backend-optionality D6"
```

---

## 任务组 6：调整测试

> 多数测试删除是对「被删符号」的覆盖清理。`TestCreateConfigBackend`（6.9）虽仅测 `create_config()`，但其断言 `"backend" in result` 依赖被删键——必须同步删除。`TestTrajectoryStrategyIntegration`（test_backend.py L486-615）4 个测试同源签名错配（`BaseOptimalTrajectory(robot, ["joint1"], config_file=...)`），属 6.8 修红范围扩展。

### Task 6.1: 删除 `Backend` ABC 与 `create_backend` 工厂测试

**Files:**
- Modify: `tests/unit/test_backend.py`
  - 删除 `class TestBackendABC`（L12-49）
  - 删除 `class TestCreateBackendFactory`（L51-107）

- [x] **Step 1: 删除上述两个类**

- [x] **Step 2: 验证零残留 `create_backend` 工厂测试**

Run: `grep -n "test_create_backend\|TestBackendABC\|TestCreateBackendFactory" tests/unit/test_backend.py`
Expected: 无命中。

### Task 6.2: 删除 `NumericalBackend` 测试

**Files:**
- Modify: `tests/unit/test_backend.py`
  - 删除 `class TestNumericalBackend`（L109-225）
  - 删除 `class TestRegressionSafety`（L358-404，依赖 `NumericalBackend.build_regressor` 对比）

- [x] **Step 1: 删除上述两个类**

- [x] **Step 2: 验证零残留 `NumericalBackend` 测试**

Run: `grep -n "NumericalBackend\|TestNumericalBackend\|TestRegressionSafety" tests/unit/test_backend.py`
Expected: 仅文件顶部 import 行 L9 命中（组 6.4 清理 import）。

### Task 6.3: 删除 `ColumnEliminationCallback` 测试

**Files:**
- Modify: `tests/unit/test_backend.py:341-356`（`test_column_elimination_callback`，位于 `TestCasadiBackend` 类内）

- [x] **Step 1: 删除该测试方法**

- [x] **Step 2: 验证零残留**

Run: `grep -n "ColumnElimination" tests/unit/test_backend.py`
Expected: 无命中。

### Task 6.4: 保留并改造 `TestCasadiBackend` 与清理 import

**Files:**
- Modify: `tests/unit/test_backend.py`
  - 删除 `TestCasadiBackend` 内对 ABC 死方法的断言测试：
    - `test_name`（L281-286）
    - `test_create_solver_returns_callable`（L289-310）
    - `test_create_solver_returns_info_key`（L312-339）
  - 保留：`test_import_error_without_casadi`（L238）、`test_lazy_initialization`（L251）、`test_lazy_initialization_triggers_on_call`（L264）
  - 保留 `TestCasadiBackendProperties`（L427-483）全部（`regressor_function`/`rnea_function`/`cache_key`）
  - 保留 `TestRootPackageExport`（L406-424）
- Modify: `tests/unit/test_backend.py:8-9`（删除失效 import）

- [x] **Step 1: 删除上述 3 个死方法测试**

- [x] **Step 2: 清理顶部 import**

将 L8-9：
```python
from figaroh.backend.base import Backend, BackendType, create_backend
from figaroh.backend.numerical import NumericalBackend
```
整体删除。后续按需新增 `from figaroh.backend.casadi import CasadiBackend`（各测试方法内已局部 import，保持现状即可）。

- [x] **Step 3: 验证 test_backend.py 可被 pytest 收集（语法正确）**

Run: `pixi run python -m pytest tests/unit/test_backend.py --collect-only -q`
Expected: 无 SyntaxError/ImportError on `base`/`numerical`。

### Task 6.5: 删除 `test_config.py` 中 backend 优先级测试

**Files:**
- Modify: `tests/unit/test_config.py:65-158`（删除 `class TestBaseOptimalTrajectoryBackendPrecedence` 全文，含 3 个测试）

- [x] **Step 1: 删除 L65-158 整个类**

这 3 个测试（`test_config_backend_used_when_default_arg` / `test_explicit_backend_wins_over_config` / `test_default_backend_when_no_config_backend`）断言 `create_backend` 调用与优先级，且使用旧 `["joint1"]` 位置参数签名（重构前已红）。

- [x] **Step 2: 验证零残留**

Run: `grep -n "TestBaseOptimalTrajectoryBackendPrecedence\|create_backend\|backend=" tests/unit/test_config.py`
Expected: 无命中（`TestLoadParamBackend` L58-63 已为空 `pass`，可一并删除或保留——保留无害）。

### Task 6.6: 删除 `test_fourier_e2e.py` 的 `backend="casadi"` 传参

**Files:**
- Modify: `tests/unit/test_fourier_e2e.py:80-84, 105-109`

- [x] **Step 1: 删除两处 `backend="casadi"`**

L80-84 与 L105-109 的 `BaseOptimalTrajectory(...)` 调用中移除 `backend="casadi"` 关键字参数。此步骤与 6.8 的签名修复在同一调用点合并执行（见 6.8）。

### Task 6.7: 校验 `test_fourier_strategy.py` mock 契约一致

**Files:**
- Modify: `tests/unit/test_fourier_strategy.py:97-117`（核对，必要时调整）

- [x] **Step 1: 核对 mock CasadiBackend 接口与新契约一致**

当前 mock（L98-117）设置了：`name`、`regressor_function`、`rnea_function`、`_cmodel`、`_cdata`。新契约下 `name` 已删，但 mock 设 `mock_backend.name = "casadi"`（L99）无害——除非 `fourier_strategy.solve` 读取它（已确认不读）。保留 mock 不动即可。

验证 `solve` 实际读取的属性均被 mock：`_ensure_symbolic_model`（L78）、`_cmodel`（L79）、`regressor_function`（L140）、`rnea_function`（L174）。

- [x] **Step 2: 补 mock `_ensure_symbolic_model` 方法**

当前 mock 是 `MagicMock()`，`_ensure_symbolic_model` 自动为 no-op MagicMock 方法——调用 `cas_be._ensure_symbolic_model()` 不会真正构建模型，符合测试意图。无需改动。

- [x] **Step 3: 运行该测试（casadi 环境，因 import casadi as cs）**

Run: `pixi run -e casadi python -m pytest tests/unit/test_fourier_strategy.py -v`
Expected: PASS。

### Task 6.8: 修复保留测试的旧签名错配（修红）

**Files:**
- Modify: `tests/unit/test_fourier_e2e.py:80-84, 105-109`
- Modify: `tests/unit/test_backend.py:513-515, 548-550, 575-577, 613-615`（`TestTrajectoryStrategyIntegration` 4 处同源错配）

**说明（扩展 tasks.md 6.8）：** `TestTrajectoryStrategyIntegration`（test_backend.py L486-615）4 个测试均使用 `BaseOptimalTrajectory(robot, ["joint1"], config_file="dummy.yaml")`，与 test_fourier_e2e.py 同源错配——重构前已红（design 文档「5 个失败」未计入 test_backend.py，但签名错配客观存在）。本任务一并修复。

- [x] **Step 1: 修 `test_fourier_e2e.py` 两处调用**

L80-84：
```python
            traj = BaseOptimalTrajectory(
                simple_robot,
                config_file="dummy.yaml",
            )
```
L105-109 同理。

- [x] **Step 2: 修 `test_backend.py` `TestTrajectoryStrategyIntegration` 4 处调用**

将 `BaseOptimalTrajectory(robot, ["joint1"], config_file="dummy.yaml")` 改为 `BaseOptimalTrajectory(robot, config_file="dummy.yaml")`（L513-515、L548-550、L575-577、L613-615）。

- [x] **Step 3: 处理 fourier 策略测试的 CasadiBackend mock**

`test_fourier_strategy_created_when_configured`（L520）配置 `trajectory_type="fourier"`，重构后 `__init__` 会执行 `CasadiBackend(robot=robot)` → 默认环境无 casadi 抛 `ImportError`。需在 `patch("figaroh.optimal.base_optimal_trajectory.create_strategy")` 之外加 `patch("figaroh.optimal.base_optimal_trajectory.CasadiBackend")`：

```python
            with patch(
                "figaroh.optimal.base_optimal_trajectory.CasadiBackend"
            ) as mock_cb:
                with patch(
                    "figaroh.optimal.base_optimal_trajectory.create_strategy"
                ) as mock_create:
                    mock_strategy = MagicMock()
                    mock_strategy.name.return_value = "fourier"
                    mock_create.return_value = mock_strategy
                    traj = BaseOptimalTrajectory(
                        robot, config_file="dummy.yaml",
                    )
```

spline 测试（L489、L556、L592）配置 `trajectory_type="spline"`，`__init__` 走 `self._backend = None`，无需 mock。

- [x] **Step 4: 运行 test_backend.py（默认环境）**

Run: `pixi run python -m pytest tests/unit/test_backend.py -v`
Expected: 除依赖 casadi 的 `TestCasadiBackend`/`TestCasadiBackendProperties`（默认环境跳过或 mock）外，`TestTrajectoryStrategyIntegration` 全绿；无 TypeError。

### Task 6.9: 确认并删除 `TestCreateConfigBackend`

**Files:**
- Modify: `tests/unit/test_config.py:7-55`

**说明：** 设计文档原「保留 `TestCreateConfigBackend`」基于「目前通过」。但组 5.2 删除 `create_config` 的 `backend` 键后，该类 3 个测试断言 `assert "backend" in result` 与 `result["backend"] == "casadi"` 将失败。按 tasks.md 6.9「若其断言依赖 backend 键则同步删除」执行——确认依赖后删除整个类。

- [x] **Step 1: 确认断言依赖 backend 键**

`TestCreateConfigBackend`（L7-55）：`test_create_config_with_backend` 断言 `"backend" in result`；`test_create_config_default_backend` 断言 `result["backend"] == "numerical"`；`test_create_config_backend_overrides_default` 断言 `result["backend"] == "casadi"`。全部依赖被删键。

- [x] **Step 2: 删除整个 `TestCreateConfigBackend` 类（L7-55）**

- [x] **Step 3: 运行 test_config.py（默认环境）**

Run: `pixi run python -m pytest tests/unit/test_config.py -v`
Expected: 全绿（`TestLoadParamBackend` 为空 `pass` 类，可保留）。

- [x] **Step 4: Commit 组 6**

```bash
git add tests/unit/test_backend.py tests/unit/test_config.py tests/unit/test_fourier_e2e.py tests/unit/test_fourier_strategy.py
git commit -m "test: drop backend tests, fix active_joints signature mismatch

Remove Backend/NumericalBackend/ColumnEliminationCallback/create_backend
tests and TestBaseOptimalTrajectoryBackendPrecedence / TestCreateConfigBackend
(both assert removed backend key). Fix BaseOptimalTrajectory(robot, [\"joint_1\"],
config_file=...) -> BaseOptimalTrajectory(robot, config_file=...) signature
mismatch pre-existing in test_fourier_e2e.py and test_backend.py
TestTrajectoryStrategyIntegration. Mock CasadiBackend in fourier strategy test."
```

---

## 任务组 7：验证

### Task 7.1: casadi 环境运行 `tests/unit/`

- [x] **Step 1: 运行 casadi 环境单测**

Run: `pixi run -e casadi pytest tests/unit/ -v`
Expected: 全绿（除明确删除的 backend 测试外）。fourier 路径依赖 casadi 环境的符号模型，`test_fourier_e2e.py`、`test_fourier_strategy.py` 应通过。

### Task 7.2: 默认环境运行全量 pytest

- [x] **Step 1: 运行默认环境全量**

Run: `pixi run pytest`
Expected: 全绿。spline 路径与现有测试不受影响。重构前 5 个失败（test_config.py 3 + test_fourier_e2e.py 2）已随组 6 修复；若仍有失败需排查是否为 casadi-only 测试在默认环境的预期跳过。

### Task 7.3: grep 全工程确认符号消失

- [x] **Step 1: 搜索被删符号**

Run: `grep -rn "Backend\b" src/figaroh | grep -v "CasadiBackend"`
Run: `grep -rn "BackendType\|create_backend\|NumericalBackend\|ColumnEliminationCallback\|_solve_with_casadi_backend" src`
Expected: src 中无命中（`CasadiBackend` 除外）。

- [x] **Step 2: 搜索 `backend=` 关键字参数与 YAML `backend:` 键**

Run: `grep -rn "backend=" src tests`
Run: `grep -rn "backend:" --include="*.yaml" --include="*.yml" .`
Expected: src/tests 无 `backend=` 命中；YAML 无 `backend:` 命中。

### Task 7.4: 手动验收 1 — fourier 配置启动

- [x] **Step 1: casadi 环境运行 UR10 fourier 示例**

Run: `pixi run -e casadi python figaroh-examples/examples/ur10/optimal_trajectory.py`
Expected: 进入 `FourierOptimizationStrategy.solve()` 不抛 `AttributeError`；`CasadiBackend` 被自动创建（`trajectory_type == "fourier"` 分支）。观察日志含 `Trajectory optimization strategy: fourier`。

### Task 7.5: 手动验收 2 — `backend=` 抛 TypeError

- [x] **Step 1: 验证 backend= 被拒**

Run:
```bash
pixi run python -c "
from unittest.mock import MagicMock
from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
try:
    BaseOptimalTrajectory(MagicMock(), config_file='dummy.yaml', backend='casadi')
    print('FAIL: no TypeError')
except TypeError as e:
    assert 'backend' in str(e) or 'unexpected keyword' in str(e).lower(), e
    print('OK: TypeError raised:', e)
"
```
Expected: `OK: TypeError raised: ... unexpected keyword argument 'backend'`

- [x] **Step 2: 最终 commit（如有验证过程产生的修复）**

```bash
git add -A
git commit -m "verify: all green on casadi + default env, symbols purged" --allow-empty
```

---

## 自检清单（Self-Review）

1. **Spec 覆盖：** D1（4.1-4.3）✓ D2（2.1-2.5）✓ D3（4.4-4.5）✓ D4（2.4）✓ D5（3.1-3.2）✓ D6（5.1-5.3）✓。测试策略 4 项（7.1-7.4）✓；7.5 额外覆盖 TypeError 验收 ✓。测试调整明细表 4 行（test_backend/test_config/test_fourier_e2e/test_fourier_strategy）✓ 全部在组 6 覆盖。
2. **Placeholder 扫描：** 无 TBD/TODO；所有代码步骤含具体代码或确切删除行号。
3. **类型一致：** `CasadiBackend` 构造签名 `__init__(self, robot: Any)` 在 4.3 与 6.8 mock 一致；`BaseOptimalTrajectory.__init__(self, robot, config_file=...)` 在 4.2、6.8、7.5 一致。
4. **扩展说明：** 6.8 扩展覆盖 `test_backend.py::TestTrajectoryStrategyIntegration`（design 文档未明列但同源错配客观存在），6.9 确认删除 `TestCreateConfigBackend`（design 原拟保留，但断言依赖被删键）——两处均经代码核对，属「修红」范畴，不扩大重构面。
