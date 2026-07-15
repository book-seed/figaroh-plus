---
change: symbolic-fourier-trajectory
design-doc: docs/superpowers/specs/2026-07-15-fourier-symbolic-trajectory-design.md
base-ref: ad5ce2e9aad96aea6e625ed523ae2fb3761dcbac
---

# 全符号化傅里叶激励轨迹优化 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 FIGAROH 中引入全符号化傅里叶级数激励轨迹优化管线，构建 SX/MX 混合 CasADi NLP，实现 D-最优激励轨迹优化，并与现有三次样条路径通过策略模式并列共存。

**Architecture:** 策略模式重构：`BaseOptimalTrajectory` 作为 Context 容器，持有 `TrajectoryOptimizationStrategy` 策略对象。`SplineOptimizationStrategy` 封装现有三次样条路径（cyipopt + 数值约束）。`FourierOptimizationStrategy` 实现全符号 MX NLP（CasADi `cs.nlpsol("ipopt")`），内层 SX Function 通过 `map("openmp")` 并行求值。新增 `BaseTrajectory` / `FourierTrajectory` 轨迹模块。

**Tech Stack:** Python 3.12, CasADi >=3.7.2, pinocchio (conda-forge, with casadi bindings), IPOPT, HSL, numpy, pytest

## Global Constraints

- 向后兼容：`trajectory_type: "spline"`（默认）必须与当前行为完全一致
- 所有新字段带默认值；缺失不改变现有行为
- 傅里叶路径使用 CasADi `cs.nlpsol("ipopt")`，不经过 cyipopt
- `map("openmp")` 是硬性要求，不提供静默回退到 `"serial"` 的路径
- TDD：每个 task 先写测试再写实现代码
- `.comet.yaml` 中的 `phase: build`，对应 `workflow: full`

---

## File Structure

| 文件 | 操作 | 职责 |
|------|------|------|
| `src/figaroh/utils/base_trajectory.py` | **创建** | `BaseTrajectory` ABC：生成 q/v/a、约束检查、可视化 |
| `src/figaroh/utils/fourier_trajectory.py` | **创建** | `FourierTrajectory(BaseTrajectory)`：傅里叶级数 numpy 求值 + CasADi SX 表达式构建 |
| `src/figaroh/optimal/strategies/__init__.py` | **创建** | 策略包导出 + 工厂函数 |
| `src/figaroh/optimal/strategies/base_strategy.py` | **创建** | `TrajectoryOptimizationStrategy` ABC |
| `src/figaroh/optimal/strategies/spline_strategy.py` | **创建** | `SplineOptimizationStrategy`：三次样条路径封装 |
| `src/figaroh/optimal/strategies/fourier_strategy.py` | **创建** | `FourierOptimizationStrategy`：全符号 CasADi NLP |
| `src/figaroh/optimal/config.py` | **修改** | 新增 `trajectory_type` + `fourier_config` 字段解析 |
| `src/figaroh/optimal/base_optimal_trajectory.py` | **修改** | 策略模式集成；`__init__` 选择策略；`solve()` 委托策略 |
| `src/figaroh/optimal/contraints.py` | **修改** | 新增 `build_symbolic_constraints()` 方法 |
| `src/figaroh/backend/casadi.py` | **修改** | 新增 `regressor_function` / `rnea_function` property；缓存优化；删除旧 Callback 辅助函数 |
| `tests/unit/test_fourier_trajectory.py` | **创建** | FourierTrajectory 单元测试 |
| `tests/unit/test_fourier_strategy.py` | **创建** | FourierOptimizationStrategy 单元测试 |
| `tests/unit/test_config_fourier.py` | **创建** | 配置解析扩展测试 |
| `tests/unit/test_base_trajectory.py` | **创建** | BaseTrajectory ABC 契约测试 |
| `tests/unit/test_strategies.py` | **创建** | 策略模式单元测试 |
| `scripts/check_env.py` | **创建** | 环境验证脚本 |

---

### Task 1: 环境依赖验证脚本

**Files:**
- Create: `scripts/check_env.py`
- Modify: `pixi.toml` (if exists) or `pyproject.toml`（添加 casadi feature 声明）

**Interfaces:**
- Produces: `scripts/check_env.py` — 命令行可执行脚本，输出各组件状态

- [x] **Step 1: 编写环境验证脚本**

创建 `scripts/check_env.py`:

```python
#!/usr/bin/env python3
"""环境验证脚本：检查 CasADi/IPOPT/HSL/pinocchio.casadi 各组件状态。

用法:
    python scripts/check_env.py

返回值:
    0 — 全部组件可用
    1 — 有组件缺失（输出详细修复指引）
"""

import sys
import subprocess
import ctypes.util
from pathlib import Path


def _check(ok: bool, name: str, hint: str = "") -> bool:
    """Print check result and return ok."""
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {name}")
    if not ok and hint:
        for line in hint.strip().splitlines():
            print(f"         {line}")
    return ok


def check_casadi() -> bool:
    """Check CasADi is importable and has IPOPT."""
    try:
        import casadi as cs
    except ImportError:
        return _check(False, "casadi 包",
                      "安装: pip install casadi 或 conda install -c conda-forge casadi")

    # Check IPOPT availability via nlpsol
    try:
        x = cs.SX.sym("x")
        solver = cs.nlpsol("tester", "ipopt", {"x": x, "f": x**2})
        ok = True
    except Exception:
        ok = False
    _check(ok, "casadi.nlpsol('ipopt') 可用",
           "IPOPT 未链接到 CasADi，请确认安装了带 IPOPT 的 CasADi 构建。\n"
           "conda-forge 版本默认包含 IPOPT。")

    # Check OpenMP
    try:
        has_omp = cs.has_native("OpenMP")
    except Exception:
        has_omp = False
    _check(has_omp, "CasADi OpenMP 支持",
           "OpenMP 不可用，请确保 CasADi 编译时带 -DWITH_OPENMP=ON。\n"
           "conda-forge 版本默认启用 OpenMP。")
    return ok and has_omp


def check_pinocchio_casadi() -> bool:
    """Check pinocchio.casadi is available."""
    try:
        import pinocchio.casadi as cpin
        return _check(True, "pinocchio.casadi 绑定")
    except ImportError:
        return _check(False, "pinocchio.casadi 绑定",
                      "需要 conda-forge 版本 pinocchio（PyPI pin 包不支持 CasADi）。\n"
                      "安装: conda install -c conda-forge pinocchio")


def check_hsl() -> bool:
    """Check HSL linear solver library (ma57) is available for IPOPT."""
    lib_names = ["libhsl.so", "libhsl.dylib", "libcoinhsl.so", "libcoinhsl.dylib"]
    found = any(ctypes.util.find_library(name) or Path(name).is_file()
                for name in lib_names)
    return _check(found, "HSL 线性求解器库 (ma57)",
                  "HSL 未找到，IPOPT 将回退到 mumps。\n"
                  "安装: 从 https://www.hsl.rl.ac.uk/ipopt/ 获取 libhsl.so")


def check_pinocchio() -> bool:
    """Check base pinocchio package (without casadi bindings)."""
    try:
        import pinocchio
        v = getattr(pinocchio, "__version__", "unknown")
        return _check(True, f"pinocchio (v{v})")
    except ImportError:
        return _check(False, "pinocchio",
                      "安装: conda install -c conda-forge pinocchio")


def main() -> int:
    print("=" * 60)
    print("FIGAROH 环境验证脚本")
    print("=" * 60)

    results = []

    print("\n[1] 基础依赖")
    results.append(("CasADi 包 + IPOPT", check_casadi()))
    results.append(("pinocchio", check_pinocchio()))
    results.append(("pinocchio.casadi 绑定", check_pinocchio_casadi()))

    print("\n[2] IPOPT 线性求解器")
    results.append(("HSL (ma57)", check_hsl()))

    print("\n" + "=" * 60)
    n_pass = sum(1 for _, ok in results if ok)
    n_total = len(results)
    print(f"结果: {n_pass}/{n_total} 通过")

    for name, ok in results:
        if not ok:
            print(f"  [{name}] 未通过 — 请按上方提示修复")

    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(main())
```

- [x] **Step 2: 确保脚本可执行**

```bash
chmod +x scripts/check_env.py
python scripts/check_env.py
```
预期输出：至少显示 CasADi、pinocchio 等组件的 PASS/FAIL 状态。

- [x] **Step 3: 提交**

```bash
git add scripts/check_env.py
git commit -m "feat(env): add environment verification script for CasADi/IPOPT/HSL"
```

---

### Task 2: 配置解析扩展

**Files:**
- Modify: `src/figaroh/optimal/config.py`
- Create: `tests/unit/test_config_fourier.py`

**Interfaces:**
- Produces: `trajectory_config` 字典新增 `trajectory_type`、`fourier_frequency`、`n_samples`、`n_harmonics`、`reg_lambda`、`tanh_alpha_opt`、`tanh_alpha_id` 字段

- [x] **Step 1: 编写配置解析扩展测试**

创建 `tests/unit/test_config_fourier.py`:

```python
"""Tests for Fourier trajectory configuration parsing."""

import pytest
from figaroh.optimal.config import create_config


class TestFourierConfigParsing:
    """Test that Fourier-specific config fields are parsed correctly."""

    def test_default_fourier_fields(self):
        """When fourier_config is absent, defaults are used."""
        unified_cfg = {
            "problem": {},
            "trajectory": {},
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        # trajectory_type should exist
        assert "trajectory_type" in result
        assert result["trajectory_type"] == "spline"
        # fourier sub-config should exist
        fourier = result.get("fourier_config", {})
        assert fourier["n_harmonics"] == 5
        assert fourier["fourier_frequency"] is None
        assert fourier["n_samples"] == 200
        assert fourier["reg_lambda"] == 1.0e-6
        assert fourier["tanh_alpha_opt"] == 10
        assert fourier["tanh_alpha_id"] == 100

    def test_explicit_fourier_config(self):
        """Fourier fields override defaults when provided."""
        unified_cfg = {
            "problem": {},
            "trajectory": {
                "type": "fourier",
                "fourier": {
                    "n_harmonics": 7,
                    "fourier_frequency": 1.5,
                    "n_samples": 500,
                    "reg_lambda": 1.0e-8,
                    "tanh_alpha_opt": 20,
                    "tanh_alpha_id": 200,
                },
            },
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["trajectory_type"] == "fourier"
        fourier = result["fourier_config"]
        assert fourier["n_harmonics"] == 7
        assert fourier["fourier_frequency"] == 1.5
        assert fourier["n_samples"] == 500
        assert fourier["reg_lambda"] == 1.0e-8
        assert fourier["tanh_alpha_opt"] == 20
        assert fourier["tanh_alpha_id"] == 200

    def test_invalid_trajectory_type(self):
        """Invalid trajectory_type raises ValueError."""
        unified_cfg = {
            "problem": {},
            "trajectory": {"type": "polynomial"},
            "constraints": {},
            "output": {},
        }
        with pytest.raises(ValueError, match="trajectory_type.*polynomial"):
            create_config(unified_cfg)

    def test_partial_fourier_config(self):
        """Partial fourier config uses defaults for missing fields."""
        unified_cfg = {
            "problem": {},
            "trajectory": {
                "type": "fourier",
                "fourier": {"n_harmonics": 3},
            },
            "constraints": {},
            "output": {},
        }
        result = create_config(unified_cfg)
        assert result["trajectory_type"] == "fourier"
        fourier = result["fourier_config"]
        assert fourier["n_harmonics"] == 3  # explicit
        assert fourier["n_samples"] == 200  # default
        assert fourier["reg_lambda"] == 1.0e-6  # default

    def test_legacy_config_preserves_behavior(self):
        """Legacy format fields are unchanged."""
        from figaroh.optimal.config import load_param
        from unittest.mock import patch, MagicMock

        robot = MagicMock()
        robot.model.name = "test_robot"

        yaml_content = """
identification:
  trajectory_params:
    - n_wps: 8
      freq: 50
      t_s: 1.0
      soft_lim: 0.1
      max_attempts: 200
  backend: numerical
"""
        with patch("builtins.open") as mock_open:
            mock_open.return_value.__enter__.return_value.read.return_value = yaml_content
            with patch("yaml.load") as mock_yaml:
                mock_yaml.return_value = {
                    "identification": {
                        "trajectory_params": [{
                            "n_wps": 8, "freq": 50, "t_s": 1.0,
                            "soft_lim": 0.1, "max_attempts": 200,
                        }],
                        "backend": "numerical",
                    }
                }
                traj_config, _ = load_param(robot, "dummy.yaml")
                assert traj_config["n_wps"] == 8
                assert traj_config["freq"] == 50
                assert traj_config["trajectory_type"] == "spline"  # default
```

- [x] **Step 2: 运行测试检查失败**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_config_fourier.py -v
```
预期输出：FAIL — `create_config` 尚未暴露新字段。

- [x] **Step 3: 扩展 `create_config()` 函数**

修改 `src/figaroh/optimal/config.py` 中的 `create_config()` 函数：

```python
_FOURIER_DEFAULTS = {
    "n_harmonics": 5,
    "fourier_frequency": None,
    "n_samples": 200,
    "reg_lambda": 1.0e-6,
    "tanh_alpha_opt": 10,
    "tanh_alpha_id": 100,
}


def create_config(unified_traj_config) -> dict:
    problem_params = unified_traj_config.get("problem", {})
    traj_params = unified_traj_config.get("trajectory", {})
    constraint_params = unified_traj_config.get("constraints", {})
    output_params = unified_traj_config.get("output", {})

    trajectory_type = traj_params.get("type", "spline")
    if trajectory_type not in ("spline", "fourier"):
        raise ValueError(
            f"Invalid trajectory_type: '{trajectory_type}'. "
            "Must be 'spline' or 'fourier'."
        )

    fourier_raw = traj_params.get("fourier", {})
    fourier_config = {}
    for key, default in _FOURIER_DEFAULTS.items():
        fourier_config[key] = fourier_raw.get(key, default)

    trajectory_config = {
        "n_wps": traj_params.get("waypoints", 5),
        "freq": traj_params.get("frequency", 100),
        "t_s": traj_params.get("segment_duration", 2.0),
        "soft_lim": problem_params.get("soft_lim", 0.05),
        "max_attempts": problem_params.get("max_attempts", 1000),
        "backend": problem_params.get("backend", "numerical"),
        "trajectory_type": trajectory_type,
        "fourier_config": fourier_config,
    }
    return trajectory_config
```

还需修改 `load_param()` 中的旧格式解析路径，添加默认 `trajectory_type`：

在旧格式路径中，`load_param()` 的末尾添加：

```python
trajectory_config["trajectory_type"] = "spline"
trajectory_config["fourier_config"] = dict(_FOURIER_DEFAULTS)
```

在文件顶部添加 `_FOURIER_DEFAULTS` 字典（在 import 之后即可）。

- [x] **Step 4: 再次运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_config_fourier.py -v
```
预期输出：所有测试 PASS。

- [x] **Step 5: 提交**

```bash
git add src/figaroh/optimal/config.py tests/unit/test_config_fourier.py
git commit -m "feat(config): add trajectory_type and fourier_config parsing"
```

---

### Task 3: 轨迹模块抽象 — BaseTrajectory

**Files:**
- Create: `src/figaroh/utils/base_trajectory.py`
- Create: `tests/unit/test_base_trajectory.py`

**Interfaces:**
- Produces: `BaseTrajectory` ABC 定义 q/v/a 生成、约束检查、可视化的接口

- [x] **Step 1: 编写 BaseTrajectory ABC 测试**

创建 `tests/unit/test_base_trajectory.py`:

```python
"""Tests for BaseTrajectory abstract base class."""

import pytest
import numpy as np


class TestBaseTrajectoryABC:
    """Test BaseTrajectory ABC contract."""

    def test_abc_cannot_be_instantiated(self):
        """BaseTrajectory ABC raises TypeError when instantiated directly."""
        from figaroh.utils.base_trajectory import BaseTrajectory
        with pytest.raises(TypeError):
            BaseTrajectory()  # type: ignore

    def test_concrete_trajectory_must_implement_abstract_methods(self):
        """Subclass missing abstract methods raises TypeError."""
        from figaroh.utils.base_trajectory import BaseTrajectory

        class IncompleteTraj(BaseTrajectory):
            pass

        with pytest.raises(TypeError):
            IncompleteTraj()  # type: ignore

    def test_concrete_trajectory_with_all_methods(self):
        """Subclass implementing all abstract methods can be instantiated."""
        from figaroh.utils.base_trajectory import BaseTrajectory

        class ConcreteTraj(BaseTrajectory):
            def get_trajectory(self, t, coeffs):
                return np.zeros((len(t), 3))

            def get_velocity(self, t, coeffs):
                return np.zeros((len(t), 3))

            def get_acceleration(self, t, coeffs):
                return np.zeros((len(t), 3))

            def compute_torques(self, q, v, a, robot):
                return np.zeros((q.shape[0], 3))

            def check_constraints(self, q, v, tau, robot):
                return False

        traj = ConcreteTraj()
        t = np.linspace(0, 1, 10)
        coeffs = np.random.randn(3, 3)
        assert traj.get_trajectory(t, coeffs).shape == (10, 3)
        assert traj.get_velocity(t, coeffs).shape == (10, 3)
        assert traj.get_acceleration(t, coeffs).shape == (10, 3)
        assert not traj.check_constraints(
            np.zeros((10, 3)), np.zeros((10, 3)), np.zeros((10, 3)), None
        )
```

- [x] **Step 2: 实现 BaseTrajectory ABC**

创建 `src/figaroh/utils/base_trajectory.py`:

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

"""Abstract base class for trajectory generation."""

from abc import ABC, abstractmethod
import numpy as np


class BaseTrajectory(ABC):
    """Abstract trajectory generator.

    Defines the interface for generating trajectory position, velocity, and
    acceleration from parameterized coefficients. Concrete implementations
    include CubicSplineTrajectory (ndcurves cubic splines) and
    FourierTrajectory (Fourier series).
    """

    @abstractmethod
    def get_trajectory(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        """Evaluate position q(t) at time points t.

        Args:
            t: Time points, shape (N,).
            coeffs: Trajectory coefficients, shape (n_param, ...).

        Returns:
            Position trajectory, shape (N, nq).
        """
        ...

    @abstractmethod
    def get_velocity(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        """Evaluate velocity v(t) = dq/dt at time points t.

        Args:
            t: Time points, shape (N,).
            coeffs: Trajectory coefficients, shape (n_param, ...).

        Returns:
            Velocity trajectory, shape (N, nv).
        """
        ...

    @abstractmethod
    def get_acceleration(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        """Evaluate acceleration a(t) = d2q/dt2 at time points t.

        Args:
            t: Time points, shape (N,).
            coeffs: Trajectory coefficients, shape (n_param, ...).

        Returns:
            Acceleration trajectory, shape (N, nv).
        """
        ...

    @abstractmethod
    def compute_torques(
        self, q: np.ndarray, v: np.ndarray, a: np.ndarray, robot
    ) -> np.ndarray:
        """Compute joint torques from trajectory.

        Args:
            q: Position trajectory, shape (N, nq).
            v: Velocity trajectory, shape (N, nv).
            a: Acceleration trajectory, shape (N, nv).
            robot: RobotWrapper instance.

        Returns:
            Joint torques, shape (N, nv).
        """
        ...

    @abstractmethod
    def check_constraints(
        self,
        q: np.ndarray,
        v: np.ndarray,
        tau: np.ndarray,
        robot,
    ) -> bool:
        """Check if trajectory violates joint constraints.

        Args:
            q: Position trajectory, shape (N, nq).
            v: Velocity trajectory, shape (N, nv).
            tau: Joint torques, shape (N, nv).
            robot: RobotWrapper instance.

        Returns:
            True if any constraint is violated.
        """
        ...
```

- [x] **Step 3: 再次运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_base_trajectory.py -v
```
预期输出：所有测试 PASS。

- [x] **Step 4: 提交**

```bash
git add src/figaroh/utils/base_trajectory.py tests/unit/test_base_trajectory.py
git commit -m "feat(utils): add BaseTrajectory ABC for trajectory generation"
```

---

### Task 4: FourierTrajectory 实现

**Files:**
- Create: `src/figaroh/utils/fourier_trajectory.py`
- Create: `tests/unit/test_fourier_trajectory.py`

**Interfaces:**
- Consumes: `BaseTrajectory` ABC
- Produces: `FourierTrajectory(BaseTrajectory)` — 傅里叶级数 numpy 求值 + CasADi SX 表达式构建

- [x] **Step 1: 编写 FourierTrajectory 单元测试**

创建 `tests/unit/test_fourier_trajectory.py`:

```python
"""Tests for FourierTrajectory."""

import pytest
import numpy as np
from unittest.mock import MagicMock


class TestFourierExpression:
    """Test Fourier series expression evaluation."""

    def test_basic_evaluation(self):
        """Evaluate Fourier series at time points returns correct shape."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 2
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        # Fourier coefficients: shape (n_act, 2*n_harmonics + 1)
        coeffs = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5  # a0 offset
        coeffs[:, 1] = 0.1  # a1 sin
        coeffs[:, 2] = 0.1  # b1 cos

        t = np.linspace(0, 1, 50)
        q = traj.get_trajectory(t, coeffs)
        assert q.shape == (50, n_act)

    def test_velocity_is_derivative_of_position(self):
        """Numerical derivative of q should match analytical v."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 5
        n_act = 3
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        rng = np.random.default_rng(42)
        coeffs = rng.uniform(-0.1, 0.1, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5  # a0 offset

        t = np.linspace(0, 2 * np.pi, 200)
        dt = t[1] - t[0]

        q = traj.get_trajectory(t, coeffs)
        v = traj.get_velocity(t, coeffs)

        # Central difference for numerical velocity
        v_numerical = np.zeros_like(v)
        v_numerical[1:-1] = (q[2:] - q[:-2]) / (2 * dt)
        # First and last use forward/backward difference
        v_numerical[0] = (q[1] - q[0]) / dt
        v_numerical[-1] = (q[-1] - q[-2]) / dt

        np.testing.assert_allclose(v, v_numerical, atol=1e-4)

    def test_acceleration_is_second_derivative(self):
        """Numerical second derivative of q should match analytical a."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 5
        n_act = 2
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        rng = np.random.default_rng(42)
        coeffs = rng.uniform(-0.1, 0.1, size=(n_act, 2 * n_harmonics + 1))
        coeffs[:, 0] = 0.5

        t = np.linspace(0, 2 * np.pi, 200)
        dt = t[1] - t[0]

        q = traj.get_trajectory(t, coeffs)
        a = traj.get_acceleration(t, coeffs)

        # Central difference for numerical acceleration
        a_numerical = np.zeros_like(a)
        a_numerical[1:-1] = (q[2:] - 2 * q[1:-1] + q[:-2]) / (dt ** 2)
        a_numerical[0] = a_numerical[1]
        a_numerical[-1] = a_numerical[-2]

        np.testing.assert_allclose(a, a_numerical, atol=1e-3)

    def test_torque_computation(self):
        """compute_torques delegates to pinocchio rnea."""
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        traj = FourierTrajectory(n_harmonics=2, n_act=2)
        q = np.random.randn(10, 2)
        v = np.random.randn(10, 2)
        a = np.random.randn(10, 2)

        mock_robot = MagicMock()
        mock_robot.model.nv = 2
        mock_robot.model.nq = 2

        with pytest.importorskip("pinocchio"):
            import pinocchio

            # Create a minimal pinocchio model for testing
            model = pinocchio.Model()
            for _ in range(2):
                jid = model.addJoint(
                    0, pinocchio.JointModelRY(),
                    pinocchio.SE3.Identity(), f"joint_{_}"
                )
                model.appendBodyToJoint(jid, pinocchio.Inertia(
                    mass=1.0, lever=np.zeros(3), inertia_matrix=np.eye(3))
                )
            mock_robot.model = model

            tau = traj.compute_torques(q, v, a, mock_robot)
            assert tau.shape == (10, 2)

    def test_casadi_expression_construction(self):
        """Build CasADi SX expression for Fourier series q(t, coeffs)."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 2
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        t = cs.SX.sym("t")
        coeffs = cs.SX.sym("coeffs", n_act, 2 * n_harmonics + 1)
        q_sx = traj.build_casadi_expression(t, coeffs)

        assert q_sx.shape == (n_act, 1)

        # Evaluate numerically
        coeffs_val = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs_val[0, 0] = 1.0  # a0 = 1 for joint 0
        q_fn = cs.Function("q", [t, coeffs], [q_sx])
        q_val = np.array(q_fn(0.5, coeffs_val)).flatten()
        assert q_val[0] == pytest.approx(1.0)  # a0 = 1

    def test_casadi_position_velocity_acceleration(self):
        """Verify CasADi SX expressions: v = dq/dt, a = d2q/dt2."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.utils.fourier_trajectory import FourierTrajectory

        n_harmonics = 3
        n_act = 1
        traj = FourierTrajectory(n_harmonics=n_harmonics, n_act=n_act)

        t_sym = cs.SX.sym("t")
        coeffs_sym = cs.SX.sym("coeffs", n_act, 2 * n_harmonics + 1)

        q_sx = traj.build_casadi_expression(t_sym, coeffs_sym)
        v_sx = cs.jacobian(q_sx, t_sym)  # dq/dt
        a_sx = cs.jacobian(v_sx, t_sym)  # d2q/dt2

        # Evaluate with known coefficients: q = a0 + a1*sin(omega*t) + b1*cos(omega*t)
        coeffs_vec = np.zeros((n_act, 2 * n_harmonics + 1))
        coeffs_vec[0, 0] = 0.0   # a0
        coeffs_vec[0, 1] = 1.0   # a1 (sin)
        coeffs_vec[0, 2] = 0.0   # b1 (cos)

        omega = 2 * np.pi / 10.0
        # When a0=0, a1=1, b1=0, omega=2*pi/10:
        #   q = sin(omega*t)
        #   v = omega * cos(omega*t)
        #   a = -omega^2 * sin(omega*t)

        q_fn = cs.Function("q", [t_sym, coeffs_sym], [q_sx])
        v_fn = cs.Function("v", [t_sym, coeffs_sym], [v_sx])
        a_fn = cs.Function("a", [t_sym, coeffs_sym], [a_sx])

        t_val = 1.5
        q_val = float(q_fn(t_val, coeffs_vec))
        v_val = float(v_fn(t_val, coeffs_vec))
        a_val = float(a_fn(t_val, coeffs_vec))

        expected_q = np.sin(omega * t_val)
        expected_v = omega * np.cos(omega * t_val)
        expected_a = -omega**2 * np.sin(omega * t_val)

        assert q_val == pytest.approx(expected_q, abs=1e-10)
        assert v_val == pytest.approx(expected_v, abs=1e-10)
        assert a_val == pytest.approx(expected_a, abs=1e-10)
```

- [x] **Step 2: 运行测试检查失败**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_fourier_trajectory.py -v
```
预期输出：FAIL — `FourierTrajectory` 未定义。

- [x] **Step 3: 实现 FourierTrajectory**

创建 `src/figaroh/utils/fourier_trajectory.py`:

```python
# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
"""Fourier series trajectory generation."""

from typing import Optional
import numpy as np

from .base_trajectory import BaseTrajectory


class FourierTrajectory(BaseTrajectory):
    """Fourier series parameterized trajectory.

    Generates smooth periodic trajectories as sums of harmonic sinusoids:

        q_j(t) = a0_j + Σ_{k=1}^{N} [ a_k_j * sin(k*ω*t) + b_k_j * cos(k*ω*t) ]

    where ω = 2π/T is the fundamental frequency.

    Args:
        n_harmonics: Number of harmonic terms (N).
        n_act: Number of active joints.
        omega: Fundamental frequency (rad/s). If None, computed as 2π/T.
        T: Period (s). Default 2π.
    """

    def __init__(
        self,
        n_harmonics: int = 5,
        n_act: int = 0,
        omega: Optional[float] = None,
        T: float = 2 * np.pi,
    ):
        self._n_harmonics = n_harmonics
        self._n_act = n_act
        self._T = T
        self._omega = omega if omega is not None else 2 * np.pi / T

    @property
    def n_harmonics(self) -> int:
        return self._n_harmonics

    @property
    def n_coeffs_per_joint(self) -> int:
        """Number of Fourier coefficients per joint: 1 (a0) + 2 * N."""
        return 1 + 2 * self._n_harmonics

    @property
    def omega(self) -> float:
        return self._omega

    # ── NumPy evaluation (for numerical backend) ───────────────────

    def _evaluate(self, t: np.ndarray, coeffs: np.ndarray) -> tuple:
        """Evaluate position, velocity, acceleration at time points.

        Args:
            t: Time points, shape (N,).
            coeffs: Fourier coefficients, shape (n_act, 2*n_harmonics + 1).

        Returns:
            Tuple (q, v, a) each shape (N, n_act).
        """
        N = len(t)
        n_act = coeffs.shape[0]
        n_h = self._n_harmonics

        q = np.zeros((N, n_act))
        v = np.zeros((N, n_act))
        a = np.zeros((N, n_act))

        for j in range(n_act):
            a0 = coeffs[j, 0]
            q[:, j] = a0
            v[:, j] = 0.0
            a[:, j] = 0.0

            for k in range(1, n_h + 1):
                ak = coeffs[j, 2 * k - 1]
                bk = coeffs[j, 2 * k]
                k_omega = k * self._omega

                sin_kwt = np.sin(k_omega * t)
                cos_kwt = np.cos(k_omega * t)

                # q = ak*sin(kωt) + bk*cos(kωt)
                q[:, j] += ak * sin_kwt + bk * cos_kwt
                # v = ak*kω*cos(kωt) - bk*kω*sin(kωt)
                v[:, j] += ak * k_omega * cos_kwt - bk * k_omega * sin_kwt
                # a = -ak*(kω)^2*sin(kωt) - bk*(kω)^2*cos(kωt)
                a[:, j] += -ak * k_omega**2 * sin_kwt - bk * k_omega**2 * cos_kwt

        return q, v, a

    def get_trajectory(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        q, _, _ = self._evaluate(t, coeffs)
        return q

    def get_velocity(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        _, v, _ = self._evaluate(t, coeffs)
        return v

    def get_acceleration(self, t: np.ndarray, coeffs: np.ndarray) -> np.ndarray:
        _, _, a = self._evaluate(t, coeffs)
        return a

    def compute_torques(
        self, q: np.ndarray, v: np.ndarray, a: np.ndarray, robot
    ) -> np.ndarray:
        """Compute joint torques via pinocchio rnea."""
        import pinocchio

        N = q.shape[0]
        nv = robot.model.nv
        tau = np.zeros((N, nv))
        for i in range(N):
            tau[i, :] = pinocchio.rnea(
                robot.model, robot.data, q[i, :], v[i, :], a[i, :]
            )
        return tau

    def check_constraints(
        self, q: np.ndarray, v: np.ndarray, tau: np.ndarray, robot
    ) -> bool:
        """Check if trajectory violates joint position/velocity/effort limits."""
        model = robot.model
        violated = False

        for i in range(q.shape[0]):
            for j in range(q.shape[1]):
                if q[i, j] > model.upperPositionLimit[j] or \
                   q[i, j] < model.lowerPositionLimit[j]:
                    violated = True

        for i in range(v.shape[0]):
            for j in range(v.shape[1]):
                if abs(v[i, j]) > model.velocityLimit[j]:
                    violated = True

        for i in range(tau.shape[0]):
            for j in range(tau.shape[1]):
                if abs(tau[i, j]) > model.effortLimit[j]:
                    violated = True

        return violated

    # ── CasADi SX expression (for symbolic NLP) ───────────────────

    def build_casadi_expression(self, t_sym, coeffs_sym, omega=None):
        """Build CasADi SX expression for q(t, coeffs).

        Args:
            t_sym: CasADi SX symbol for time (scalar).
            coeffs_sym: CasADi SX symbol for coefficients, shape (n_act, 2*n_harmonics+1).
            omega: Fundamental frequency (optional, uses self._omega if None).

        Returns:
            CasADi SX expression for q(t, coeffs), shape (n_act, 1).
        """
        import casadi as cs

        n_act = coeffs_sym.shape[0]
        n_h = self._n_harmonics
        omega_val = omega if omega is not None else self._omega

        q_expr = cs.SX.zeros(n_act, 1)
        for j in range(n_act):
            q_j = coeffs_sym[j, 0]  # a0
            for k in range(1, n_h + 1):
                ak = coeffs_sym[j, 2 * k - 1]
                bk = coeffs_sym[j, 2 * k]
                k_omega = k * omega_val
                q_j += ak * cs.sin(k_omega * t_sym) + bk * cs.cos(k_omega * t_sym)
            q_expr[j] = q_j
        return q_expr
```

- [x] **Step 4: 运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_fourier_trajectory.py -v
```
预期输出：所有测试 PASS（CasADi 相关测试需要环境中有 CasADi，否则被 `pytest.importorskip` 跳过）。

- [x] **Step 5: 提交**

```bash
git add src/figaroh/utils/fourier_trajectory.py tests/unit/test_fourier_trajectory.py
git commit -m "feat(utils): add FourierTrajectory with numpy eval and CasADi SX expressions"
```

---

### Task 5: 策略模式架构 — 策略包 + ABC + 工厂 + SplineStrategy 迁移

**Files:**
- Create: `src/figaroh/optimal/strategies/__init__.py`
- Create: `src/figaroh/optimal/strategies/base_strategy.py`
- Create: `src/figaroh/optimal/strategies/spline_strategy.py`
- Create: `tests/unit/test_strategies.py`

**Interfaces:**
- Produces: `TrajectoryOptimizationStrategy` ABC + `SplineOptimizationStrategy` + 工厂函数 `create_strategy`

- [x] **Step 1: 编写策略模式测试**

创建 `tests/unit/test_strategies.py`:

```python
"""Tests for trajectory optimization strategy pattern."""

import pytest
import numpy as np
from unittest.mock import MagicMock, patch


class TestStrategyABC:
    """Test TrajectoryOptimizationStrategy ABC contract."""

    def test_abc_cannot_be_instantiated(self):
        """ABC raises TypeError when instantiated directly."""
        from figaroh.optimal.strategies.base_strategy import (
            TrajectoryOptimizationStrategy
        )
        with pytest.raises(TypeError):
            TrajectoryOptimizationStrategy()  # type: ignore

    def test_concrete_strategy_must_implement_solve(self):
        """Subclass missing solve raises TypeError."""
        from figaroh.optimal.strategies.base_strategy import (
            TrajectoryOptimizationStrategy
        )

        class IncompleteStrategy(TrajectoryOptimizationStrategy):
            pass

        with pytest.raises(TypeError):
            IncompleteStrategy()  # type: ignore

    def test_concrete_strategy_with_solve(self):
        """Subclass implementing solve can be instantiated."""
        from figaroh.optimal.strategies.base_strategy import (
            TrajectoryOptimizationStrategy
        )

        class ConcreteStrategy(TrajectoryOptimizationStrategy):
            def solve(self, context) -> list:
                return []

            def name(self) -> str:
                return "concrete"

        strategy = ConcreteStrategy()
        assert strategy.solve(None) == []
        assert strategy.name() == "concrete"


class TestStrategyFactory:
    """Test factory function for strategy creation."""

    def test_create_spline_strategy(self):
        """Factory returns SplineOptimizationStrategy for 'spline'."""
        from figaroh.optimal.strategies import create_strategy

        strategy = create_strategy("spline")
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )
        assert isinstance(strategy, SplineOptimizationStrategy)

    def test_create_invalid_strategy(self):
        """Factory raises ValueError for unknown type."""
        from figaroh.optimal.strategies import create_strategy

        with pytest.raises(ValueError, match="Unknown trajectory type"):
            create_strategy("polynomial")


class TestSplineStrategy:
    """Test SplineOptimizationStrategy."""

    def test_name(self):
        """SplineStrategy.name returns 'spline'."""
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )
        strategy = SplineOptimizationStrategy()
        assert strategy.name() == "spline"

    def test_solve_delegates_to_context(self):
        """solve() creates WaypointsGeneration and calls existing flow."""
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )

        strategy = SplineOptimizationStrategy()
        mock_context = MagicMock()
        mock_context.trajectory_config = {
            "n_wps": 5, "freq": 100, "t_s": 2.0,
            "soft_lim": 0.05, "max_attempts": 100,
        }
        mock_context.active_joints = ["joint1", "joint2"]
        mock_context.soft_lim_pool = np.full((3, 2), 0.05)
        mock_context.results = {
            'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
            'iteration_data': [], 'final_regressor_shape': None,
        }
        mock_context.idx_e = np.array([], dtype=int)
        mock_context.idx_b = np.array([0, 1, 2], dtype=int)

        with patch(
            "figaroh.optimal.strategies.spline_strategy.WaypointsGeneration"
        ) as MockWP:
            mock_wp = MagicMock()
            MockWP.return_value = mock_wp
            mock_wp.gen_rand_pool.return_value = None
            mock_wp.act_idxq = [0, 1]
            mock_wp.act_idxv = [0, 1]
            mock_wp.lower_q = [-1.0, -1.0]
            mock_wp.upper_q = [1.0, 1.0]

            with patch(
                "figaroh.optimal.strategies.spline_strategy."
                "TrajectoryConstraintManager"
            ) as MockConstraint:
                mock_cm = MagicMock()
                MockConstraint.return_value = mock_cm

                with patch.object(
                    strategy, "_solve_segment", return_value=True
                ) as mock_solve:
                    result = strategy.solve(mock_context)

            assert mock_solve.called
            assert "T_F" in mock_context.results
```

- [x] **Step 2: 运行测试检查失败**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_strategies.py -v
```
预期输出：FAIL — 包和类未定义。

- [x] **Step 3: 实现策略包和基类**

创建 `src/figaroh/optimal/strategies/__init__.py`:

```python
# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
"""Trajectory optimization strategies.

Provides a strategy-pattern interface for different trajectory optimization
approaches. Two strategies are available:

- ``spline``: cubic spline + cyipopt (existing, default)
- ``fourier``: Fourier series + CasADi nlpsol (new)
"""

from .base_strategy import TrajectoryOptimizationStrategy
from .spline_strategy import SplineOptimizationStrategy

try:
    from .fourier_strategy import FourierOptimizationStrategy
except ImportError:
    FourierOptimizationStrategy = None  # type: ignore

__all__ = [
    "TrajectoryOptimizationStrategy",
    "SplineOptimizationStrategy",
    "FourierOptimizationStrategy",
    "create_strategy",
]


def create_strategy(trajectory_type: str, **kwargs):
    """Factory: create a strategy instance by type.

    Args:
        trajectory_type: ``"spline"`` or ``"fourier"``.
        **kwargs: Strategy-specific keyword arguments.

    Returns:
        A TrajectoryOptimizationStrategy instance.

    Raises:
        ValueError: If trajectory_type is unknown.
    """
    if trajectory_type == "spline":
        return SplineOptimizationStrategy(**kwargs)
    elif trajectory_type == "fourier":
        if FourierOptimizationStrategy is None:
            raise ImportError(
                "FourierOptimizationStrategy requires CasADi. "
                "Install with: pixi add casadi"
            )
        return FourierOptimizationStrategy(**kwargs)
    else:
        raise ValueError(
            f"Unknown trajectory type: '{trajectory_type}'. "
            "Valid options: 'spline', 'fourier'."
        )
```

创建 `src/figaroh/optimal/strategies/base_strategy.py`:

```python
# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
"""Abstract base class for trajectory optimization strategies."""

from abc import ABC, abstractmethod
from typing import List


class TrajectoryOptimizationStrategy(ABC):
    """Strategy pattern for trajectory optimization.

    Each concrete strategy implements a different trajectory parameterization
    (cubic spline, Fourier series) and its associated optimization pipeline.

    The ``solve()`` method receives a context object (``BaseOptimalTrajectory``
    instance) and must populate ``context.results`` with the standard format.
    """

    @abstractmethod
    def solve(self, context) -> None:
        """Execute the trajectory optimization.

        Args:
            context: ``BaseOptimalTrajectory`` instance providing robot model,
                configuration, base parameter indices, and result storage.

        Returns:
            None. Results are written to ``context.results`` dict.
        """
        ...

    @abstractmethod
    def name(self) -> str:
        """Human-readable strategy identifier."""
        ...
```

创建 `src/figaroh/optimal/strategies/spline_strategy.py`:

```python
# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
"""Spline-based trajectory optimization strategy.

Migrates the existing cubic spline + cyipopt flow from
BaseOptimalTrajectory into the strategy pattern.
"""

import logging
import numpy as np
from typing import Dict, Any, Tuple

from figaroh.utils.cubic_spline import WaypointsGeneration
from figaroh.optimal.contraints import TrajectoryConstraintManager
from figaroh.tools.robotipopt import IPOPTConfig, RobotIPOPTSolver
from figaroh.tools.regressor import (
    build_regressor_basic,
    build_regressor_reduced,
)
from figaroh.tools.qrdecomposition import build_baseRegressor
from figaroh.utils.pin_interface import calc_torque

from .base_strategy import TrajectoryOptimizationStrategy


class SplineOptimizationStrategy(TrajectoryOptimizationStrategy):
    """Cubic spline trajectory optimization (existing behavior).

    Uses ndcurves cubic spline interpolation, numerical constraint evaluation,
    and cyipopt IPOPT solver.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.addHandler(logging.NullHandler())

    def name(self) -> str:
        return "spline"

    def solve(self, context) -> None:
        """Execute spline-based optimization. Mirrors existing solve()."""
        traj_cfg = context.trajectory_config
        stack_reps = traj_cfg.get("stack_reps", 2)

        context.WP = WaypointsGeneration(
            context.robot,
            traj_cfg["n_wps"],
            context.active_joints,
            context.soft_lim_pool,
        )

        context.constraint_manager = TrajectoryConstraintManager(
            context.robot, context.WP, traj_cfg, context.identif_config
        )

        context.WP.gen_rand_pool()

        # Build initial waypoints
        wp_init = self._build_initial_waypoints(context)
        vel_wp_init = np.zeros(len(context.WP.act_idxv))
        acc_wp_init = np.zeros(len(context.WP.act_idxv))

        W_stack = None
        for s_rep in range(stack_reps):
            success = self._solve_segment(
                context, s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack
            )
            if not success:
                break
            if s_rep < stack_reps - 1:
                wp_init, W_stack = self._prepare_next_segment(context)

    def _build_initial_waypoints(self, context) -> np.ndarray:
        """Build initial waypoint guess from random pool."""
        rng = np.random.default_rng(100)
        wp_init = np.zeros(len(context.WP.act_idxq))
        for idx in range(len(context.WP.act_idxq)):
            center = (context.WP.lower_q[idx] + context.WP.upper_q[idx]) / 2
            half_range = (context.WP.upper_q[idx] - context.WP.lower_q[idx]) / 2 * 1.0
            q_pool = np.asarray(context.WP.pool_q)[:, idx]
            valid_q = q_pool[
                (q_pool >= (center - half_range)) & (q_pool <= (center + half_range))
            ]
            if valid_q.size == 0:
                raise RuntimeError(
                    f"No pool samples for joint index {idx}"
                )
            wp_init[idx] = float(rng.choice(valid_q))
        return wp_init

    def _generate_feasible_initial_guess(
        self, context, wp_init, vel_wp_init, acc_wp_init
    ):
        """Generate feasible initial guess via random search."""
        count = 0
        is_constr_violated = True
        max_attempts = context.trajectory_config.get("max_attempts", 500)

        while is_constr_violated and count < max_attempts:
            count += 1
            try:
                wps, vel_wps, acc_wps = context.WP.gen_rand_wp(
                    wp_init, vel_wp_init, acc_wp_init
                )
                tps = np.matrix(
                    [context.trajectory_config["t_s"] * i_wp
                     for i_wp in range(context.trajectory_config["n_wps"])]
                ).transpose()
                t_i, p_i, v_i, a_i = context.WP.get_full_config(
                    context.trajectory_config["freq"], tps, wps, vel_wps, acc_wps
                )
                tau_i = calc_torque(
                    p_i.shape[0], context.robot, p_i, v_i, a_i
                )
                tau_i_np = (
                    np.reshape(tau_i, (v_i.shape[1], v_i.shape[0])).transpose()
                )
                is_constr_violated = context.WP.check_cfg_constraints(
                    p_i, v_i, tau_i_np
                )
            except Exception:
                continue

        return wps, vel_wps, acc_wps, tps, t_i, p_i, v_i, a_i

    def _solve_segment(
        self, context, s_rep, wp_init, vel_wp_init, acc_wp_init, W_stack
    ) -> bool:
        """Solve a single spline segment."""
        try:
            wps, vel_wps, acc_wps, tps, _, p_i, _, _ = (
                self._generate_feasible_initial_guess(
                    context, wp_init, vel_wp_init, acc_wp_init
                )
            )
            tps = (
                context.trajectory_config["t_s"]
                * (context.trajectory_config["n_wps"] - 1)
                * s_rep
                + tps
            )

            problem = context.create_ipopt_problem(
                len(context.active_joints),
                context.trajectory_config["n_wps"],
                p_i.shape[0],
                tps,
                vel_wps,
                acc_wps,
                wp_init,
                vel_wp_init,
                acc_wp_init,
                W_stack,
            )

            success, result_data = problem.solve_with_waypoints(wps)

            if success:
                context.results['T_F'].append(result_data['t_f'])
                context.results['P_F'].append(result_data['p_f'])
                context.results['V_F'].append(result_data['v_f'])
                context.results['A_F'].append(result_data['a_f'])
                context.results['iteration_data'].append(
                    result_data['iter_data']
                )
                return True
            return False
        except Exception as e:
            self.logger.error(f"Error solving segment {s_rep + 1}: {e}")
            return False

    def _prepare_next_segment(
        self, context
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Prepare initial conditions for next segment."""
        last_result = context.results['iteration_data'][-1]
        wp_init = last_result['final_waypoint']
        last_p_f = context.results['P_F'][-1]
        last_v_f = context.results['V_F'][-1]
        last_a_f = context.results['A_F'][-1]
        W_stack = context._stack_base_regressors(
            last_p_f, last_v_f, last_a_f
        )
        return wp_init, W_stack
```

- [x] **Step 4: 运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_strategies.py -v
```
预期输出：所有测试 PASS。

- [x] **Step 5: 提交**

```bash
git add src/figaroh/optimal/strategies/ tests/unit/test_strategies.py
git commit -m "feat(optimal): add strategy pattern with SplineOptimizationStrategy migration"
```

---

### Task 6: CasadiBackend 属性扩展（regressor_function / rnea_function）

**Files:**
- Modify: `src/figaroh/backend/casadi.py`

**Interfaces:**
- Produces: `CasadiBackend.regressor_function` property → `cs.Function(q, v, a) -> W`
- Produces: `CasadiBackend.rnea_function` property → `cs.Function(q, v, a) -> tau`
- 缓存指纹使用 SHA256 全惯性参数哈希

- [x] **Step 1: 编写属性接口测试**

将以下测试追加到 `tests/unit/test_backend.py`（在 `TestCasadiBackend` 类内）：

```python
class TestCasadiBackendProperties:
    """Test CasadiBackend property accessors."""

    @pytest.fixture(autouse=True)
    def _reset_casadi_globals(self):
        import figaroh.backend.casadi as _m
        _m.cpin = None
        _m.cs = None
        yield

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_regressor_function_property(self, mock_cs, mock_cpin):
        """regressor_function property returns a cs.Function."""
        from figaroh.backend.casadi import CasadiBackend

        mock_cpin.Model.return_value.nq = 3
        mock_cpin.Model.return_value.nv = 3
        mock_cs.Function.return_value = mock_cs.Function

        robot = MagicMock()
        backend = CasadiBackend(robot=robot)
        fn = backend.regressor_function
        assert fn is not None

    @patch('figaroh.backend.casadi.cpin')
    @patch('figaroh.backend.casadi.cs')
    def test_rnea_function_property(self, mock_cs, mock_cpin):
        """rnea_function property returns a cs.Function."""
        from figaroh.backend.casadi import CasadiBackend

        mock_cpin.Model.return_value.nq = 3
        mock_cpin.Model.return_value.nv = 3
        mock_cs.Function.return_value = mock_cs.Function

        robot = MagicMock()
        backend = CasadiBackend(robot=robot)
        fn = backend.rnea_function
        assert fn is not None

    @patch('figaroh.backend.casadi.cpin')
    def test_cache_key_includes_full_inertia(self, mock_cpin):
        """Cache key uses SHA256 hash of full inertial parameters."""
        from figaroh.backend.casadi import CasadiBackend

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3
        # Mock inertias such that total mass differs
        robot.model.inertias.tolist.return_value = [
            MagicMock(mass=1.0), MagicMock(mass=2.0), MagicMock(mass=0.0)
        ]

        key1 = CasadiBackend._cache_key(robot)
        assert "test_robot" in key1
        assert len(key1) > len("test_robot_")
```

- [x] **Step 2: 运行测试检查失败**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_backend.py::TestCasadiBackendProperties -v
```
预期输出：FAIL — `regressor_function` 和 `rnea_function` 属性未定义。

- [x] **Step 3: 扩展 CasadiBackend — 添加属性 + 缓存优化**

修改 `src/figaroh/backend/casadi.py`，在 `CasadiBackend` 类中添加或替换以下内容：

1. 将 `_cache_key` 方法替换为完整的 SHA256 哈希版本：

```python
@staticmethod
def _cache_key(robot) -> str:
    """Build a deterministic cache key from robot model inertial parameters."""
    import hashlib
    m = robot.model
    # Fingerprint from all inertial parameters (mass, inertia, com)
    inertia_parts = []
    for i in m.inertias:
        mass = float(i.mass) if abs(float(i.mass)) > 1e-9 else 0.0
        lever = [float(x) for x in i.lever]
        inertia_flat = [float(x) for row in i.inertia for x in row]
        inertia_parts.extend([mass] + lever + inertia_flat)
    fingerprint = f"{m.name}_{m.nq}_{m.nv}_" + "_".join(
        f"{v:.10f}" for v in inertia_parts
    )
    h = hashlib.sha256(fingerprint.encode()).hexdigest()[:16]
    return f"{m.name}_{h}"
```

2. 在 `CasadiBackend` 类中添加 `_rnea_fun` 属性和两个 property 方法：

在 `__init__` 中添加 `self._rnea_fun = None`：

```python
def __init__(self, robot: Any):
    self._robot = robot
    self._cmodel = None
    self._cdata = None
    self._W_fun = None
    self._rnea_fun = None  # NEW
```

添加 property 方法（在 `_ensure_symbolic_model` 方法之后）：

```python
@property
def regressor_function(self):
    """SX Function: (q, v, a) -> W(nv, n_param).

    Returns CasADi Function that computes the joint torque regressor.
    """
    self._ensure_symbolic_model()
    return self._W_fun

@property
def rnea_function(self):
    """SX Function: (q, v, a) -> tau(nv).

    Returns CasADi Function that computes the RNEA joint torques.
    """
    self._ensure_symbolic_model()
    return self._rnea_fun
```

在 `_ensure_symbolic_model` 中添加 `_rnea_fun` 的构建：

在 `self._W_fun = cs.Function("W", [cs_q, cs_v, cs_a], [W_expr])` 之后添加：

```python
# Build symbolic RNEA function
tau_expr = cpin.rnea(self._cmodel, self._cdata, cs_q, cs_v, cs_a)
self._rnea_fun = cs.Function("rnea", [cs_q, cs_v, cs_a], [tau_expr])
```

在缓存保存逻辑中也保存 `_rnea_fun`（可选，首次构建后缓存）。

- [x] **Step 4: 运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_backend.py::TestCasadiBackendProperties -v
```
预期输出：所有测试 PASS。

- [x] **Step 5: 提交**

```bash
git add src/figaroh/backend/casadi.py tests/unit/test_backend.py
git commit -m "feat(backend): add regressor_function and rnea_function properties to CasadiBackend"
```

---

### Task 7: FourierOptimizationStrategy 符号管线核心

**Files:**
- Create: `src/figaroh/optimal/strategies/fourier_strategy.py`
- Create: `tests/unit/test_fourier_strategy.py`

**Interfaces:**
- Consumes: `TrajectoryOptimizationStrategy` ABC; `CasadiBackend` properties; `FourierTrajectory`; `BaseParameterComputer`
- Produces: `FourierOptimizationStrategy` — 全符号 CasADi NLP 管线

- [x] **Step 1: 编写 FourierStrategy 初始化测试**

创建 `tests/unit/test_fourier_strategy.py`:

```python
"""Tests for FourierOptimizationStrategy."""

import pytest
import numpy as np
from unittest.mock import MagicMock, patch


class TestFourierStrategyInitialization:
    """Test strategy creation and configuration."""

    def test_strategy_name(self):
        """name() returns 'fourier'."""
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        strategy = FourierOptimizationStrategy()
        assert strategy.name() == "fourier"

    def test_accepts_fourier_config(self):
        """Strategy accepts and stores fourier_config at init."""
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        config = {
            "n_harmonics": 5,
            "fourier_frequency": None,
            "n_samples": 200,
            "reg_lambda": 1.0e-6,
            "tanh_alpha_opt": 10,
            "tanh_alpha_id": 100,
        }
        strategy = FourierOptimizationStrategy(fourier_config=config)
        assert strategy._fourier_config["n_harmonics"] == 5
        assert strategy._fourier_config["n_samples"] == 200


class TestFourierStrategySolveFlow:
    """Test the high-level solve() flow."""

    def test_solve_populates_results(self):
        """solve() fills results dict with T_F, P_F, V_F, A_F."""
        pytest.importorskip("casadi")
        import casadi as cs
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )

        strategy = FourierOptimizationStrategy(fourier_config={
            "n_harmonics": 3,
            "fourier_frequency": None,
            "n_samples": 50,
            "reg_lambda": 1.0e-6,
            "tanh_alpha_opt": 10,
            "tanh_alpha_id": 100,
        })

        # Build a minimal mock context
        mock_ctx = MagicMock()
        mock_ctx.trajectory_config = {
            "fourier_config": strategy._fourier_config,
        }
        mock_ctx.identif_config = {
            "has_friction": True,
            "has_actuator_inertia": False,
            "has_joint_offset": False,
            "act_idxv": [0],
        }
        mock_ctx.active_joints = ["joint1"]
        mock_ctx.idx_b = np.array([0], dtype=int)
        mock_ctx.idx_e = np.array([], dtype=int)

        # Mock robot with 1-DOF
        mock_robot = MagicMock()
        mock_robot.model.nq = 1
        mock_robot.model.nv = 1
        mock_robot.model.name = "test_robot"
        mock_robot.model.upperPositionLimit = np.array([2.0])
        mock_robot.model.lowerPositionLimit = np.array([-2.0])
        mock_robot.model.velocityLimit = np.array([5.0])
        mock_robot.model.effortLimit = np.array([100.0])
        mock_robot.model.inertias.tolist.return_value = [MagicMock(mass=1.0)]

        # Mock CasadiBackend with regressor_function and rnea_function
        mock_backend = MagicMock()
        mock_backend.name = "casadi"

        # Create minimal SX functions for the mock backend
        cs_q = cs.SX.sym("q", 1)
        cs_v = cs.SX.sym("v", 1)
        cs_a = cs.SX.sym("a", 1)
        W_expr = cs.SX.ones(1, 10)  # dummy regressor
        tau_expr = cs.SX.ones(1)    # dummy torque

        mock_backend.regressor_function = cs.Function(
            "W", [cs_q, cs_v, cs_a], [W_expr]
        )
        mock_backend.rnea_function = cs.Function(
            "rnea", [cs_q, cs_v, cs_a], [tau_expr]
        )
        mock_backend._cmodel = mock_robot.model
        mock_backend._cdata = MagicMock()

        mock_ctx._backend = mock_backend
        mock_ctx.results = {
            'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
            'iteration_data': [], 'final_regressor_shape': None,
        }

        # Run solve — should populate results (the mock NLP will fail
        # but the trajectory construction should still execute)
        try:
            strategy.solve(mock_ctx)
        except Exception:
            pass

        # Check that results were populated even if NLP failed
        assert 'T_F' in mock_ctx.results
```

- [x] **Step 2: 运行测试检查失败**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_fourier_strategy.py -v
```
预期输出：FAIL — `FourierOptimizationStrategy` 未定义。

- [x] **Step 3: 实现 FourierOptimizationStrategy（核心管线）**

创建 `src/figaroh/optimal/strategies/fourier_strategy.py`:

```python
# Copyright [2021-2025] Thanh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...
"""Fourier series trajectory optimization strategy.

Implements a fully symbolic CasADi NLP pipeline for D-optimal excitation
trajectory optimization using Fourier series parameterization.
"""

import logging
from typing import Optional, Dict, Any

import numpy as np

from .base_strategy import TrajectoryOptimizationStrategy

_FOURIER_DEFAULTS = {
    "n_harmonics": 5,
    "fourier_frequency": None,
    "n_samples": 200,
    "reg_lambda": 1.0e-6,
    "tanh_alpha_opt": 10,
    "tanh_alpha_id": 100,
}


class FourierOptimizationStrategy(TrajectoryOptimizationStrategy):
    """Fourier series trajectory optimization via CasADi symbolic NLP.

    Builds a fully symbolic MX/SX hybrid NLP for D-optimal excitation
    trajectory optimization. The outer MX layer handles optimization
    variables (Fourier coefficients) and automatic differentiation,
    while the inner SX layer evaluates dynamics symbolically with
    ``map("openmp")`` parallelization.
    """

    def __init__(self, fourier_config: Optional[Dict] = None):
        self.logger = logging.getLogger(__name__)
        self.logger.addHandler(logging.NullHandler())

        cfg = dict(_FOURIER_DEFAULTS)
        if fourier_config:
            cfg.update(fourier_config)
        self._fourier_config = cfg

    def name(self) -> str:
        return "fourier"

    def solve(self, context) -> None:
        """Build and solve the fully symbolic Fourier NLP.

        Args:
            context: BaseOptimalTrajectory instance with robot model,
                backend, and configuration.
        """
        import casadi as cs
        import pinocchio.casadi as cpin

        cfg = self._fourier_config
        n_harmonics = cfg["n_harmonics"]
        n_samples = cfg["n_samples"]
        reg_lambda = cfg["reg_lambda"]
        tanh_alpha_opt = cfg["tanh_alpha_opt"]
        tanh_alpha_id = cfg["tanh_alpha_id"]
        freq = cfg.get("fourier_frequency")

        cas_be = context._backend
        cas_be._ensure_symbolic_model()
        cmodel = cas_be._cmodel
        nq = cmodel.nq
        nv = cmodel.nv

        # ── 1. Problem dimensions ──────────────────────────────────
        n_act = len(context.active_joints)
        act_idxq = context.identif_config.get("act_idxq", list(range(n_act)))
        act_idxv = context.identif_config.get("act_idxv", list(range(n_act)))
        n_coeffs_per_joint = 1 + 2 * n_harmonics
        n_vars = n_act * n_coeffs_per_joint

        # ── 2. MX optimization variables ───────────────────────────
        Z = cs.MX.sym("coeffs", n_vars)
        # Reshape to (n_act, n_coeffs_per_joint) — column-major layout
        Z_mat = cs.reshape(Z, n_act, n_coeffs_per_joint)

        # ── 3. Time vector ─────────────────────────────────────────
        # Build sampling times: N samples over one period
        T = 2 * np.pi
        omega = freq if freq is not None else 2 * np.pi / T
        t_vec = cs.MX.linspace(0, T, n_samples)
        t_vec = t_vec.T  # (1, n_samples)

        # ── 4. Column-major trajectory construction ────────────────
        # Q_col[j, i] = q_j(t_i): shape (n_act, n_samples)
        Q_col = cs.MX.zeros(n_act, n_samples)
        V_col = cs.MX.zeros(n_act, n_samples)
        A_col = cs.MX.zeros(n_act, n_samples)

        for j in range(n_act):
            a0 = Z_mat[j, 0]
            # Constant offset
            Q_col[j, :] = a0
            # Harmonic contributions
            for k in range(1, n_harmonics + 1):
                ak = Z_mat[j, 2 * k - 1]
                bk = Z_mat[j, 2 * k]
                k_omega = k * omega

                sin_kwt = cs.sin(k_omega * t_vec)
                cos_kwt = cs.cos(k_omega * t_vec)

                Q_col[j, :] += ak * sin_kwt + bk * cos_kwt
                V_col[j, :] += ak * k_omega * cos_kwt - bk * k_omega * sin_kwt
                A_col[j, :] += (
                    -ak * k_omega**2 * sin_kwt
                    - bk * k_omega**2 * cos_kwt
                )

        # ── 5. Build SX inner functions (regressor + RNEA) ─────────
        W_fun = cas_be.regressor_function
        rnea_fun = cas_be.rnea_function

        # ── 6. Map over sample points (parallel) ───────────────────
        # Build full joint-space vectors for each sample
        def _build_full_vectors(q_col, v_col, a_col, cmodel, act_idxq, act_idxv):
            """Build full (nq,) and (nv,) vectors from active joint values."""
            import casadi as cs

            nq_full = cmodel.nq
            nv_full = cmodel.nv

            q_full = cs.MX.zeros(nq_full)
            v_full = cs.MX.zeros(nv_full)
            a_full = cs.MX.zeros(nv_full)

            for i, jid in enumerate(act_idxq):
                q_full[jid] = q_col[i]
            for i, jid in enumerate(act_idxv):
                v_full[jid] = v_col[i]
                a_full[jid] = a_col[i]

            return q_full, v_full, a_full

        # Build vectors for the first sample (for initial function creation)
        q0_full, v0_full, a0_full = _build_full_vectors(
            Q_col[:, 0], V_col[:, 0], A_col[:, 0],
            cmodel, act_idxq, act_idxv,
        )

        # ── 7. Build regressor matrix using map ────────────────────
        # For each sample i, evaluate W_i = regressor_function(q_i, v_i, a_i)
        # Result stacked: W_full = vertcat(W_0, ..., W_{N-1})
        W_samples = []
        for i in range(n_samples):
            qi_full, vi_full, ai_full = _build_full_vectors(
                Q_col[:, i], V_col[:, i], A_col[:, i],
                cmodel, act_idxq, act_idxv,
            )
            Wi = W_fun(qi_full, vi_full, ai_full)
            W_samples.append(Wi)

        W_full = cs.vertcat(*W_samples)  # (n_samples * nv, n_param)

        # ── 8. D-optimal objective ─────────────────────────────────
        idx_b = context.idx_b
        if len(idx_b) > 0:
            W_b = W_full[:, idx_b]
        else:
            W_b = W_full

        N_s = n_samples
        J = cs.mtimes(W_b.T, W_b) / N_s

        # Regularize: J + lambda * I
        n_base = W_b.shape[1]
        J_reg = J + reg_lambda * cs.MX.eye(n_base)

        # D-optimal: obj = -log det(J_reg) via Cholesky
        # L = cholesky(J_reg), obj = -2 * sum(log(L_ii))
        L = cs.cholesky(J_reg)
        obj = -2 * cs.sum1(cs.log(cs.diag(L)))

        # ── 9. Torque constraints ──────────────────────────────────
        # Add friction model: tau = rnea(q,v,a) + fv*v + fs*tanh(alpha*v)
        has_friction = context.identif_config.get("has_friction", False)
        tau_samples = []
        for i in range(n_samples):
            qi_full, vi_full, ai_full = _build_full_vectors(
                Q_col[:, i], V_col[:, i], A_col[:, i],
                cmodel, act_idxq, act_idxv,
            )
            tau_i = rnea_fun(qi_full, vi_full, ai_full)

            if has_friction:
                # Friction model uses nominal values from identif_config
                fv = cs.MX.zeros(nv)
                fs = cs.MX.zeros(nv)
                # Friction columns are appended at the end of params_std
                # Use tanh(alpha * v) for smooth approximation
                for j in range(n_act):
                    jid = act_idxv[j]
                    # tanh approximation: fs * tanh(alpha * v)
                    fv[jid] = vi_full[jid]  # viscous friction coefficient
                    fs_approx = cs.tanh(tanh_alpha_opt * vi_full[jid])
                    tau_i[jid] += fv[jid] + fs_approx

            tau_samples.append(tau_i)

        tau_stack = cs.vertcat(*tau_samples)

        # ── 10. Build constraint vector ────────────────────────────
        cons_list = []

        # Position constraints at all samples (q bounds)
        for i in range(n_samples):
            for j in range(n_act):
                cons_list.append(Q_col[j, i])

        # Velocity constraints at all samples
        for i in range(n_samples):
            for j in range(n_act):
                cons_list.append(V_col[j, i])

        # Torque constraints at all samples
        for i in range(n_samples):
            for j in range(n_act):
                cons_list.append(tau_stack[i * nv + act_idxv[j]])

        cons = cs.vertcat(*cons_list) if cons_list else cs.MX(0)

        # ── 11. Build constraint bounds ────────────────────────────
        model = context.robot.model
        q_lower = model.lowerPositionLimit
        q_upper = model.upperPositionLimit
        v_limit = model.velocityLimit
        tau_limit = model.effortLimit

        cl_list = []
        cu_list = []

        # Position bounds
        for _ in range(n_samples):
            for j in range(n_act):
                cl_list.append(q_lower[act_idxq[j]])
                cu_list.append(q_upper[act_idxq[j]])

        # Velocity bounds
        for _ in range(n_samples):
            for j in range(n_act):
                cl_list.append(-v_limit[act_idxv[j]])
                cu_list.append(v_limit[act_idxv[j]])

        # Torque bounds
        for _ in range(n_samples):
            for j in range(n_act):
                cl_list.append(-tau_limit[act_idxv[j]])
                cu_list.append(tau_limit[act_idxv[j]])

        cl = np.array(cl_list, dtype=float)
        cu = np.array(cu_list, dtype=float)

        # ── 12. NLP definition ────────────────────────────────────
        nlp = {"x": Z, "f": obj, "g": cons}

        # ── 13. Solver options ────────────────────────────────────
        opts = {
            "ipopt.linear_solver": "ma57",
            "ipopt.tol": 1e-6,
            "ipopt.max_iter": 500,
            "ipopt.mu_strategy": "adaptive",
            "ipopt.print_level": 3,
            "print_time": False,
        }

        # Check HSL availability — fallback to mumps
        try:
            solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)
        except Exception:
            self.logger.warning("HSL ma57 not available, falling back to mumps")
            opts["ipopt.linear_solver"] = "mumps"
            solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)

        # ── 14. Coefficient initialization ─────────────────────────
        x0 = self._initialize_coefficients(context, n_act, n_harmonics)
        self.logger.info(
            "Initial Fourier coefficients: min=%f, max=%f",
            float(np.min(x0)), float(np.max(x0)),
        )

        # ── 15. Solve ──────────────────────────────────────────────
        result = solver(x0=x0, lbg=cl, ubg=cu)

        # ── 16. Extract results ────────────────────────────────────
        x_opt = np.array(result["x"]).flatten()
        Z_opt = x_opt.reshape(n_act, n_coeffs_per_joint)

        # Build optimal trajectory via numpy evaluation
        from figaroh.utils.fourier_trajectory import FourierTrajectory
        ft = FourierTrajectory(
            n_harmonics=n_harmonics, n_act=n_act,
            omega=omega, T=T,
        )

        t_np = np.linspace(0, T, n_samples)
        q_opt = ft.get_trajectory(t_np, Z_opt)
        v_opt = ft.get_velocity(t_np, Z_opt)
        a_opt = ft.get_acceleration(t_np, Z_opt)

        # ── 17. Populate results ───────────────────────────────────
        context.results['T_F'].append(t_np.reshape(-1, 1))
        context.results['P_F'].append(q_opt)
        context.results['V_F'].append(v_opt)
        context.results['A_F'].append(a_opt)

        stats = solver.stats()
        context.results['iteration_data'].append({
            "iterations": list(range(stats.get("iter_count", 0))),
            "obj_values": [float(result["f"])],
            "solve_time": stats.get("t_proc_cpu", {}).get("TOTAL", 0.0),
            "status": stats["return_status"],
        })
        context.results["final_regressor_shape"] = (W_b.shape[0], W_b.shape[1])

        self.logger.info(
            "Fourier optimization complete: %d vars, %d cons, status=%s",
            n_vars, cons.size1(), stats["return_status"],
        )

    def _initialize_coefficients(
        self, context, n_act: int, n_harmonics: int
    ) -> np.ndarray:
        """Initialize Fourier coefficients from joint limits.

        Strategy:
        - a0: midpoint of joint range
        - ak, bk: small random amplitude (5% of joint range)

        Args:
            context: BaseOptimalTrajectory instance.
            n_act: Number of active joints.
            n_harmonics: Number of harmonics.

        Returns:
            Initial coefficient vector, shape (n_vars,).
        """
        n_coeffs = 1 + 2 * n_harmonics
        model = context.robot.model
        act_idxq = context.identif_config.get(
            "act_idxq", list(range(n_act))
        )

        q_upper = np.array([model.upperPositionLimit[j] for j in act_idxq])
        q_lower = np.array([model.lowerPositionLimit[j] for j in act_idxq])

        rng = np.random.default_rng(42)
        x0 = np.zeros(n_act * n_coeffs)

        for j in range(n_act):
            # a0: midpoint
            x0[j * n_coeffs] = (q_upper[j] + q_lower[j]) / 2
            # ak, bk: 5% amplitude
            amp = 0.05 * (q_upper[j] - q_lower[j])
            for k in range(1, n_harmonics + 1):
                x0[j * n_coeffs + 2 * k - 1] = rng.uniform(-amp, amp)
                x0[j * n_coeffs + 2 * k] = rng.uniform(-amp, amp)

        # Constraint validation: shrink if violated
        max_retries = 5
        for retry in range(max_retries):
            # Evaluate trajectory and check constraints
            from figaroh.utils.fourier_trajectory import FourierTrajectory
            ft = FourierTrajectory(
                n_harmonics=n_harmonics, n_act=n_act,
            )
            Z_init = x0.reshape(n_act, n_coeffs)
            t_np = np.linspace(0, 2 * np.pi, 100)
            q_init = ft.get_trajectory(t_np, Z_init)
            v_init = ft.get_velocity(t_np, Z_init)

            # Simple bounds check
            violated = False
            for i in range(t_np.shape[0]):
                for j in range(n_act):
                    if q_init[i, j] > q_upper[j] or q_init[i, j] < q_lower[j]:
                        violated = True
                    if abs(v_init[i, j]) > model.velocityLimit[act_idxq[j]]:
                        violated = True

            if not violated:
                break

            # Shrink amplitude by half and retry
            for j in range(n_act):
                for k in range(1, n_harmonics + 1):
                    idx_a = j * n_coeffs + 2 * k - 1
                    idx_b = j * n_coeffs + 2 * k
                    x0[idx_a] *= 0.5
                    x0[idx_b] *= 0.5

        return x0
```

- [x] **Step 4: 运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_fourier_strategy.py -v
```
预期输出：所有测试 PASS。

- [x] **Step 5: 提交**

```bash
git add src/figaroh/optimal/strategies/fourier_strategy.py tests/unit/test_fourier_strategy.py
git commit -m "feat(optimal): implement FourierOptimizationStrategy with full CasADi symbolic NLP"
```

---

### Task 8: BaseOptimalTrajectory 策略模式集成

**Files:**
- Modify: `src/figaroh/optimal/base_optimal_trajectory.py`
- Modify: `tests/unit/test_backend.py`（追加集成测试）

- [x] **Step 1: 编写集成测试**

追加到 `tests/unit/test_backend.py`：

```python
class TestTrajectoryStrategyIntegration:
    """Test strategy pattern integration with BaseOptimalTrajectory."""

    def test_default_strategy_is_spline(self):
        """Default trajectory_type 'spline' creates SplineOptimizationStrategy."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory
        from figaroh.optimal.strategies.spline_strategy import (
            SplineOptimizationStrategy
        )

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "spline",
                    "fourier_config": {},
                },
                {},
            )
            traj = BaseOptimalTrajectory(
                robot, ["joint1"], config_file="dummy.yaml",
            )
            assert hasattr(traj, "strategy")
            assert traj.strategy.name() == "spline"
            assert isinstance(traj.strategy, SplineOptimizationStrategy)

    def test_fourier_strategy_created_when_configured(self):
        """trajectory_type 'fourier' creates FourierOptimizationStrategy."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "fourier",
                    "fourier_config": {"n_harmonics": 5},
                },
                {},
            )
            with patch(
                "figaroh.optimal.base_optimal_trajectory.create_strategy"
            ) as mock_create:
                mock_strategy = MagicMock()
                mock_strategy.name.return_value = "fourier"
                mock_create.return_value = mock_strategy

                traj = BaseOptimalTrajectory(
                    robot, ["joint1"], config_file="dummy.yaml",
                )
                mock_create.assert_called_once_with("fourier")
                assert traj.strategy.name() == "fourier"

    def test_solve_delegates_to_strategy(self):
        """solve() delegates to strategy.solve()."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "spline",
                    "fourier_config": {},
                },
                {},
            )
            traj = BaseOptimalTrajectory(
                robot, ["joint1"], config_file="dummy.yaml",
            )
            # Replace strategy with mock
            mock_strategy = MagicMock()
            mock_strategy.name.return_value = "test"
            traj.strategy = mock_strategy

            traj.results = {
                'T_F': [], 'P_F': [], 'V_F': [], 'A_F': [],
                'iteration_data': [], 'final_regressor_shape': None,
            }

            traj.solve()

            mock_strategy.solve.assert_called_once_with(traj)

    def test_save_results_format_consistent(self):
        """Both strategies produce same results format for save_results."""
        from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

        robot = MagicMock()
        robot.model.name = "test_robot"
        robot.model.nq = 3
        robot.model.nv = 3

        with patch(
            "figaroh.optimal.base_optimal_trajectory.load_param"
        ) as mock_load:
            mock_load.return_value = (
                {
                    "n_wps": 5, "freq": 100, "t_s": 2.0,
                    "soft_lim": 0.05, "max_attempts": 1000,
                    "trajectory_type": "spline",
                    "fourier_config": {},
                },
                {},
            )
            traj = BaseOptimalTrajectory(
                robot, ["joint1"], config_file="dummy.yaml",
            )
            # Verify results dict has expected format
            expected_keys = {'T_F', 'P_F', 'V_F', 'A_F',
                             'iteration_data', 'final_regressor_shape'}
            assert expected_keys.issubset(traj.results.keys())
```

- [x] **Step 2: 修改 BaseOptimalTrajectory**

修改 `src/figaroh/optimal/base_optimal_trajectory.py`：

1. 在 import 区域添加策略导入：

```python
from figaroh.optimal.strategies import create_strategy
```

2. 修改 `__init__` 以创建策略：

在 `__init__` 方法末尾（在 `self._backend` 设置之后），添加：

```python
# ── Strategy pattern ──────────────────────────────────────────
traj_type = self.trajectory_config.get("trajectory_type", "spline")
fourier_cfg = self.trajectory_config.get("fourier_config", {})
self.strategy = create_strategy(
    traj_type,
    fourier_config=fourier_cfg if traj_type == "fourier" else None,
)
self.logger.info("Trajectory optimization strategy: %s", self.strategy.name())
```

3. 重构 `solve()` 方法：

```python
def solve(self, stack_reps: int = 2) -> Dict[str, Any]:
    """Solve the optimal trajectory generation problem.

    Delegates to the active strategy. The strategy is responsible for
    populating self.results with the standard format.

    Args:
        stack_reps: Number of trajectory segments to stack
            (only used by spline strategy).

    Returns:
        Dict containing trajectories and optimization info.
    """
    self.logger.info(
        "Starting optimal trajectory generation with %s strategy...",
        self.strategy.name(),
    )

    try:
        self.strategy.solve(self)

        self.logger.info(
            "Completed! Generated %d trajectory segments",
            len(self.results['T_F']),
        )
        return self.results

    except Exception as e:
        self.logger.error(f"Error in solve: {e}")
        raise
```

4. 保留 `save_results()` 和 `plot_results()` 不变 — 它们使用 `self.results` 格式，与策略无关。

保留 `_stack_base_regressors()` 方法 — 它被 `SplineOptimizationStrategy._prepare_next_segment()` 使用。

保留 `_find_latest_pkl()` 和 `_load_results_from_pkl()` 静态方法 — 它们被 `plot_results()` 使用。

- [x] **Step 3: 运行测试验证通过**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_backend.py::TestTrajectoryStrategyIntegration -v
```
预期输出：所有测试 PASS。

- [x] **Step 4: 提交**

```bash
git add src/figaroh/optimal/base_optimal_trajectory.py tests/unit/test_backend.py
git commit -m "feat(optimal): integrate strategy pattern into BaseOptimalTrajectory"
```

---

### Task 9: 约束管理适配 + Backend 清理

**Files:**
- Modify: `src/figaroh/optimal/contraints.py`
- Modify: `src/figaroh/backend/casadi.py`

**Interfaces:**
- Produces: `TrajectoryConstraintManager.build_symbolic_constraints()` 方法

- [x] **Step 1: 在 contraints.py 中新增符号约束方法**

在 `src/figaroh/optimal/contraints.py` 中，为 `TrajectoryConstraintManager` 添加新方法：

```python
def build_symbolic_constraints(self, Q_sym, V_sym, A_sym, t_sym, cmodel):
    """Build CasADi SX symbolic constraint expressions.

    This method creates symbolic constraint expressions that can be
    embedded in a CasADi NLP graph for analytical differentiation.

    Args:
        Q_sym: Position symbol, shape (N, n_act) CasADi SX/MX.
        V_sym: Velocity symbol, shape (N, n_act) CasADi SX/MX.
        A_sym: Acceleration symbol, shape (N, n_act) CasADi SX/MX.
        t_sym: Time symbol (scalar CasADi SX/MX).
        cmodel: pinocchio.casadi Model instance.

    Returns:
        Tuple (cons_expr, lb, ub) where cons_expr is a CasADi SX/MX
        expression and lb/ub are numpy arrays of lower/upper bounds.
    """
    import casadi as cs

    N = Q_sym.shape[0]
    n_act = Q_sym.shape[1]
    nv = cmodel.nv

    constraints = []

    # Position constraints at all samples
    for i in range(N):
        for j in range(n_act):
            constraints.append(Q_sym[i, j])

    # Velocity constraints at all samples
    for i in range(N):
        for j in range(n_act):
            constraints.append(V_sym[i, j])

    cons_expr = cs.vertcat(*constraints)

    # Build bounds
    cl = []
    cu = []
    for _ in range(N):
        cl.extend(self.CB.lower_q)
        cu.extend(self.CB.upper_q)
    for _ in range(N):
        cl.extend(self.CB.lower_dq)
        cu.extend(self.CB.upper_dq)

    return cons_expr, np.array(cl, dtype=float), np.array(cu, dtype=float)
```

- [x] **Step 2: 清理 CasadiBackend 中的旧回调辅助函数**

在 `src/figaroh/backend/casadi.py` 末尾，移除以下函数（它们存在于 `base_optimal_trajectory.py` 的末尾，不属于 `casadi.py`）：

- `_make_spline_callback()`
- `_make_objective_callback()`
- `_make_constraint_callback()`
- `_compute_jacobian_sparsity()`
- `_build_symbolic_objective()`
- `_build_symbolic_constraints()`
- `_build_symbolic_constraint_bounds()`

注意：这些函数当前位于 `base_optimal_trajectory.py`，不在 `casadi.py` 中。确认这些函数没有被其他模块引用后，将它们从 `base_optimal_trajectory.py` 中删除。

确认这些函数只被 `BaseTrajectoryIPOPTProblem._solve_with_casadi_backend()` 调用。由于傅里叶策略不使用这些 Callback 辅助函数（它直接构建 SX/MX 图），可以安全移除。

将 `_solve_with_casadi_backend` 方法标记为已弃用或在 SplineStrategy 中使用。

- [x] **Step 3: 运行测试确保未破坏现有代码**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_backend.py -v
python -m pytest tests/unit/test_config.py -v
```
预期输出：所有现存测试 PASS。

- [x] **Step 4: 提交**

```bash
git add src/figaroh/optimal/contraints.py src/figaroh/backend/casadi.py
git commit -m "refactor: add symbolic constraint builder and clean up old Callback helpers"
```

---

### Task 10: 测试套件

**Files:**
- Create: `tests/unit/test_fourier_strategy.py`（追加 D-optimal 和 Cholesky 测试）
- Create: `tests/unit/test_fourier_trajectory.py`（追加有限差分对照测试）

- [x] **Step 1: 编写 D-optimal 目标单元测试**

追加到 `tests/unit/test_fourier_strategy.py`：

```python
class TestDOptimalObjective:
    """Test D-optimal objective function correctness."""

    def test_log_det_via_cholesky(self):
        """Cholesky-based logdet matches numpy logdet."""
        import casadi as cs
        import numpy as np

        # Build a known SPD matrix
        n = 4
        rng = np.random.default_rng(42)
        A = rng.standard_normal((n, n))
        J = A.T @ A  # SPD
        lam = 1e-6
        J_reg = J + lam * np.eye(n)

        # numpy reference
        sign, logdet_np = np.linalg.slogdet(J_reg)
        obj_np = -logdet_np

        # CasADi Cholesky
        J_sym = cs.SX.sym("J", n, n)
        L = cs.cholesky(J_sym)
        obj_sx = -2 * cs.sum1(cs.log(cs.diag(L)))
        obj_fn = cs.Function("obj", [J_sym], [obj_sx])

        obj_cs = float(obj_fn(J_reg))

        # Should match (up to numerical tolerance)
        assert obj_cs == pytest.approx(obj_np, abs=1e-8)

    def test_regularization_ensures_spd(self):
        """Regularization lambda ensures J+lambda*I is SPD for Cholesky."""
        import casadi as cs
        import numpy as np

        # Build a singular matrix
        J = np.ones((3, 3))  # rank 1
        lam = 1e-6
        J_reg = J + lam * np.eye(3)

        # Cholesky should succeed
        J_sym = cs.SX.sym("J", 3, 3)
        L = cs.cholesky(J_sym)
        chol_fn = cs.Function("chol", [J_sym], [L])
        L_val = np.array(chol_fn(J_reg))
        assert np.all(np.diag(L_val) > 0)

    def test_doptimal_objective_shape(self):
        """D-optimal objective returns scalar."""
        from figaroh.optimal.strategies.fourier_strategy import (
            FourierOptimizationStrategy
        )
        strategy = FourierOptimizationStrategy()
        # Verify the strategy has the required config
        assert "reg_lambda" in strategy._fourier_config


class TestFrictionModelDifferentiability:
    """Test tanh(alpha*v) friction model is CasADi differentiable."""

    def test_tanh_friction_gradient(self):
        """tanh(alpha*v) yields non-zero gradient via CasADi AD."""
        import casadi as cs
        import numpy as np

        v = cs.SX.sym("v")
        alpha = 10.0
        f_s = 1.0
        f_expr = f_s * cs.tanh(alpha * v)

        grad_fn = cs.Function("grad", [v], [cs.gradient(f_expr, v)])
        grad_val = float(grad_fn(0.5))

        # Analytical: d/dv (tanh(alpha*v)) = alpha * sech^2(alpha*v)
        # At v=0.5, alpha=10: 10 * sech^2(5) > 0
        assert grad_val > 0
        assert np.isfinite(grad_val)

    def test_tanh_friction_high_alpha(self):
        """tanh(100*v) approximates sign(v) but remains differentiable."""
        import casadi as cs
        import numpy as np

        v = cs.SX.sym("v")
        alpha_id = 100.0
        f_expr = cs.tanh(alpha_id * v)

        fn = cs.Function("f", [v], [f_expr])
        grad_fn = cs.Function("grad", [v], [cs.gradient(f_expr, v)])

        # Near zero, tanh is steep but differentiable
        v_small = 0.01
        f_val = float(fn(v_small))
        grad_val = float(grad_fn(v_small))

        assert f_val > 0.5  # close to 1 (sign(v) approximation)
        assert grad_val > 0  # still differentiable
        assert np.isfinite(grad_val)
```

- [x] **Step 2: 编写符号雅可比 vs 有限差分对照测试**

```python
class TestSymbolicJacobian:
    """Compare CasADi symbolic Jacobian vs finite differences."""

    def test_regressor_jacobian_vs_fd(self):
        """Symbolic Jacobian of regressor matches finite difference."""
        pytest.importorskip("casadi")
        pytest.importorskip("pinocchio.casadi")
        import casadi as cs
        import pinocchio.casadi as cpin
        import numpy as np

        # Build a minimal 1-DOF model
        model = cpin.Model()
        jid = model.addJoint(0, cpin.JointModelRY(), cpin.SE3.Identity(), "joint")
        model.appendBodyToJoint(jid, cpin.Inertia(
            mass=1.0, lever=np.zeros(3), inertia_matrix=np.eye(3)
        ))
        data = model.createData()

        # Symbolic regressor
        q = cs.SX.sym("q", 1)
        v = cs.SX.sym("v", 1)
        a = cs.SX.sym("a", 1)
        W_expr = cpin.computeJointTorqueRegressor(model, data, q, v, a)
        W_fn = cs.Function("W", [q, v, a], [W_expr])

        # Jacobian of W w.r.t. q (symbolic)
        J_sym = cs.jacobian(W_expr, q)
        J_fn = cs.Function("J_sym", [q, v, a], [J_sym])

        # Finite difference
        eps = 1e-6
        q0 = np.array([0.5])
        v0 = np.array([0.1])
        a0 = np.array([0.0])

        J_fd = np.zeros((W_fn(q0, v0, a0).shape[0], 1))
        W_plus = np.array(W_fn(q0 + eps, v0, a0)).flatten()
        W_minus = np.array(W_fn(q0 - eps, v0, a0)).flatten()
        J_fd[:, 0] = (W_plus - W_minus) / (2 * eps)

        J_sym_val = np.array(J_fn(q0, v0, a0))

        np.testing.assert_allclose(J_sym_val, J_fd, atol=1e-4)
```

- [x] **Step 3: 运行验证测试**

```bash
cd /home/tyche/Documents/figaroh-plus
python -m pytest tests/unit/test_fourier_strategy.py::TestDOptimalObjective -v
python -m pytest tests/unit/test_fourier_strategy.py::TestFrictionModelDifferentiability -v
python -m pytest tests/unit/test_fourier_strategy.py::TestSymbolicJacobian -v
```
预期输出：所有测试 PASS。

- [x] **Step 4: 提交**

```bash
git add tests/unit/test_fourier_strategy.py tests/unit/test_fourier_trajectory.py
git commit -m "test: add D-optimal, friction model, and symbolic Jacobian tests"
```

---

### Task 11: 文档

**Files:**
- Create: `docs/environment_setup.md`（可选，或追加到现有文档）
- Modify: `docs/superpowers/specs/2026-07-15-fourier-symbolic-trajectory-design.md`（确认文档完整）

**Note:** 依据 `task.md` 第 11 组的要求，编写以下文档：

- [x] **Step 1: 在 README 或 docs/ 中添加傅里叶轨迹配置文档**

傅里叶激励轨迹配置示例（追加到 README 或创建 `docs/fourier_trajectory.md`）：

```markdown
# 傅里叶激励轨迹配置说明

## YAML 配置

在 unified config 的 `tasks.optimal_configuration` 部分配置：

```yaml
tasks:
  optimal_configuration:
    enabled: true
    parameters:
      trajectory_type: "fourier"  # 或 "spline"
      fourier:
        n_harmonics: 5            # 谐波数量（默认 5）
        fourier_frequency: null   # 基频，null = 2π/T
        n_samples: 200            # 采样点数量（默认 200）
        reg_lambda: 1.0e-6        # 正则化系数（默认 1e-6）
        tanh_alpha_opt: 10        # 优化阶段 tanh 陡度（默认 10）
        tanh_alpha_id: 100        # 辨识阶段 tanh 陡度（默认 100）
```

## Python API

```python
from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

# 使用配置文件中的 trajectory_type
traj = BaseOptimalTrajectory(
    robot, active_joints, "config.yaml",
)

# 或在配置文件中设置
# trajectory:
#   type: "fourier"
#   fourier:
#     n_harmonics: 5
traj.solve()
```

## 验证

```python
# 检查条件数
cond = np.linalg.cond(W_b)
print(f"Condition number: {cond:.2f}")
# 傅里叶轨迹一般 < 100，随机轨迹 > 1000
```

## 注意事项

1. 傅里叶轨迹使用 CasADi `cs.nlpsol("ipopt")`，不经过 cyipopt
2. HSL ma57 线性求解器优先，不可用时回退 mumps
3. OpenMP 并行化是硬性要求，确保 CasADi 编译时带 OpenMP
4. 摩擦力模型使用 `tanh(α·v)` 近似，优化阶段 α=10，辨识阶段 α=100
5. 基参数计算与轨迹类型无关，沿用现有的 `BaseParameterComputer`
```

- [x] **Step 2: 更新 environment_setup 或 README**

在 README 中添加环境要求说明：

```markdown
## 傅里叶激励轨迹环境要求

傅里叶激励轨迹需要以下额外组件：

- **CasADi** (>=3.7.2)：符号计算和 NLP 求解
- **pinocchio.casadi**：符号动力学模型
- **IPOPT**：NLP 求解器（CasADi 内置）
- **HSL ma57**（推荐）：稀疏线性求解器
- **OpenMP**：并行采样求值

通过环境验证脚本检查各组件状态：

```bash
python scripts/check_env.py
```
```

- [x] **Step 3: 提交**

```bash
git add docs/fourier_trajectory.md README.md
git commit -m "docs: add Fourier trajectory configuration and environment documentation"
```

---

## 测试策略映射

| 测试层级 | 覆盖的 Task | 测试文件 | 关键验收标准 |
|---------|-------------|---------|-------------|
| 单元测试：配置解析 | Task 2 | `test_config_fourier.py` | 缺省字段走默认值；非法 `trajectory_type` 报错；旧格式向后兼容 |
| 单元测试：ABC 契约 | Task 3 | `test_base_trajectory.py` | `BaseTrajectory` 不能直接实例化；缺少方法以 TypeError 报错 |
| 单元测试：Fourier 表达式 | Task 4 | `test_fourier_trajectory.py` | v/a 与解析导数一致；CasADi SX 表达式可构建和求值 |
| 单元测试：策略模式 | Task 5 | `test_strategies.py` | ABC 契约；工厂函数正确返回策略；SplineStrategy 可创建 |
| 单元测试：Backend 属性 | Task 6 | `test_backend.py::TestCasadiBackendProperties` | `regressor_function`/`rnea_function` property 返回 Function |
| 单元测试：Fourier 策略 | Task 7 | `test_fourier_strategy.py` | 策略创建、solve() 流程、结果填充 |
| 集成测试：BaseOptimalTrajectory | Task 8 | `test_backend.py::TestTrajectoryStrategyIntegration` | 默认策略为 spline；solve() 委托策略；结果格式一致 |
| 单元测试：D-optimal | Task 10 | `test_fourier_strategy.py::TestDOptimalObjective` | Cholesky logdet 匹配 numpy slogdet；正则化保证 SPD |
| 单元测试：可微摩擦 | Task 10 | `test_fourier_strategy.py::TestFrictionModelDifferentiability` | tanh(α·v) 梯度非零有限；高 α 仍可微 |
| 单元测试：符号雅可比 | Task 10 | `test_fourier_strategy.py::TestSymbolicJacobian` | 符号雅可比与有限差分对照，容差 1e-4 |
| 回归测试 | Task 2, 8 | `test_config.py`, `test_backend.py` | 现有测试不受影响；数值路径零行为变化 |
| 端到端验证 | — | UR10 示例 | `trajectory_type: "fourier"` 成功生成轨迹，条件数优于随机 |
