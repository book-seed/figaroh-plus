# 傅里叶激励轨迹优化 — 完整指南

本文档覆盖傅里叶轨迹优化的全部内容：架构原理、环境搭建、配置参数、运行、结果解读、故障排查。

---

## 目录

1. [架构简介](#1-架构简介)
2. [环境搭建](#2-环境搭建)
3. [验证环境](#3-验证环境)
4. [配置参数](#4-配置参数)
5. [运行优化](#5-运行优化)
6. [结果与对比](#6-结果与对比)
7. [故障排查](#7-故障排查)
8. [注意事项](#8-注意事项)

---

## 1. 架构简介

### 1.1 计算图

傅里叶轨迹优化走全符号化 CasADi NLP 管线（SX/MX 混合）：

```
优化变量 Z = MX.sym("coeffs", n_vars)          ← 傅里叶系数
    │
    ▼
Q(Z,t) = a₀ + Σ[aₖsin(kωt) + bₖcos(kωt)]       ← 列主序 (n_act, N_s)
V(Z,t) = ∂Q/∂t                                   ← MX AD 符号微分
A(Z,t) = ∂V/∂t                                   ← MX AD 二阶符号微分
    │
    ▼
W = regressor_function.map(N_s, "openmp")(Q,V,A) ← 内层 SX Function
τ = rnea_function.map(N_s, "openmp")(Q,V,A)      ← 内层 SX Function
    │
    ▼
J = W_bᵀW_b / N_s                                 ← 信息矩阵
L = cholesky(J + λI)                              ← SPD 保证
obj = -2 Σ log(L_ii)                              ← D-最优目标
    │
    ▼
cs.nlpsol("ipopt", nlp)                           ← CasADi 内置 IPOPT + HSL
```

### 1.2 关键设计点

- **内层 SX + 外层 MX**：单步动力学用 SX Function 缓存，外层用 MX reverse-mode AD
- **Cholesky D-最优**：正则化 λI 保证信息矩阵严格正定，Cholesky 从不失败
- **统一摩擦模型**：`tanh(α·v)` 全管线一致，优化 α=10（梯度友好），辨识 α=100（逼近精度）
- **列主序零转置**：`(n_act, N_s)` 直接从源头构建，消除 `map()` 前的 Transpose 节点
- **先求导后 Map**：符号函数 → AD → `map("openmp")` 并行求值

---

## 2. 环境搭建

### 2.1 克隆仓库

```bash
git clone git@github.com:book-seed/figaroh-plus.git
cd figaroh-plus
git checkout pixi

# 拉取 submodule（示例和模型）
git submodule update --init --recursive
```

### 2.2 安装 Pixi + 创建环境

```bash
# 安装 Pixi
curl -fsSL https://pixi.sh/install.sh | bash
source ~/.bashrc

# 创建环境（含 pinocchio、numpy、scipy 等核心依赖）
pixi install

# 安装 CasADi 环境（含 casadi、pinocchio.casadi、ipopt）
pixi install -e casadi
```

### 2.3 验证核心依赖

```bash
pixi run python -c "import pinocchio; print('pinocchio:', pinocchio.__version__)"
pixi run python -c "import casadi; print('CasADi:', casadi.__version__)"
pixi run python -c "import pinocchio.casadi; print('pinocchio.casadi: OK')"
```

### 2.4 验证 OpenMP 支持

CasADi 的 `map("openmp")` 需要编译时开启 OpenMP。**全平台都需要验证此步骤**：

```bash
pixi run python -c "
import casadi as cs
x = cs.SX.sym('x')
f = cs.Function('f', [x], [x**2])
f.map(10, 'openmp')
print('OK')
"
```

- **无 WARNING** → OpenMP 就绪，跳过 2.5 节
- **有 WARNING** → 按 2.5 节按平台修复

### 2.5 按平台修复 OpenMP

#### linux-aarch64（Jetson / 树莓派 / ARM 服务器）

conda-forge 的 CasADi 二进制包未编译 OpenMP，需要源码编译：

```bash
# 前提条件
cmake --version  # >= 3.20
g++ --version    # 需支持 -fopenmp

# 下载源码 + 编译安装（预计 30-60 分钟）
pixi run download-casadi-src
pixi run build-casadi-openmp
```

然后重新运行 2.4 的验证命令，确认无 WARNING。

#### linux-x86_64 / macOS

conda-forge 的二进制包通常包含 OpenMP。如果验证失败：
- 检查是否安装了正确 channel 的 CasADi：`pixi run python -c "import casadi; print(casadi.__file__)"`，确认路径在 `.pixi/envs/` 下
- 如果确认 conda-forge 当前版本确实无 OpenMP，参照 aarch64 步骤源码编译

### 2.6 IPOPT 线性求解器

IPOPT 默认使用内置 MUMPS 求解器，全平台可用。HSL（`ma57`）可加速大变量优化，但仅 x86_64 平台的 conda-forge 提供 `coinhsl` 包：

```bash
# x86_64 only — 安装 HSL 加速
pixi add coinhsl
```

aarch64 平台（Jetson / ARM 服务器）无 `coinhsl` 包，使用 MUMPS 即可，功能完整。

如需切换求解器，在 `fourier_strategy.py` 的 `opts` 中修改：
```python
opts["ipopt.linear_solver"] = "mumps"  # 或 "ma57"（需 HSL）
```

---

## 3. 验证环境

### 3.1 环境验证脚本

```bash
pixi run python scripts/check_env.py
```

预期输出：

```
  FIGAROH 环境验证脚本
  ==============================
  [PASS] CasADi 包 + IPOPT 求解器
  [PASS] CasADi IPOPT 可用
  [PASS] CasADi OpenMP map
  [PASS] pinocchio 包
  [PASS] pinocchio.casadi 绑定

  结果: 5/5 通过
```

### 3.2 单元测试

```bash
pixi run python -m pytest tests/unit/ \
  --ignore=tests/unit/test_robotvisualization.py \
  -q
```

预期：252+ 通过，0 失败。`test_robotvisualization.py` 有预存兼容性问题，可安全忽略。

---

## 4. 配置参数

配置文件位于 `figaroh-examples/examples/<robot>/config/`，以 UR10 为例：

```yaml
optimal_trajectory:
  enabled: true

  problem:
    soft_lim: [0.01]       # 关节限位安全裕度
    max_attempts: 500       # 初始轨迹随机尝试次数

  trajectory:
    # ── 傅里叶轨迹配置（全部可选，都有默认值）──
    type: "fourier"         # "spline"（默认）或 "fourier"
    fourier:
      n_harmonics: 5        # 谐波数（默认 5）
      fourier_frequency: null  # 基频 rad/s（null → 2π/T）
      n_samples: 200        # 采样点数（默认 200）
      reg_lambda: 1.0e-6    # D-最优正则化系数
      tanh_alpha_opt: 10    # 优化阶段摩擦平滑系数
      tanh_alpha_id: 100    # 辨识阶段摩擦平滑系数

    waypoints: 7            # 路径点数（样条模式使用）
    frequency: 100          # 采样频率 Hz
    segment_duration: 2.0   # 路点时间间隔（秒）

  output:
    save_trajectory: true
    output_file: "data/trajectories/ur10_fourier_trajectory.yaml"
```

### 参数总表

| 参数 | 默认值 | 取值范围 | 说明 |
|------|--------|---------|------|
| `type` | `"spline"` | `"spline"` / `"fourier"` | 轨迹优化类型 |
| `n_harmonics` | 5 | 1-10 | 谐波数。每关节系数 = 1+2×N |
| `fourier_frequency` | `null` | > 0 或 null | 基频 rad/s。null → 自动 2π/T |
| `n_samples` | 200 | 50-1000 | 采样点数。越大精度越高但 NLP 越大 |
| `reg_lambda` | 1e-6 | 1e-8 ~ 1e-3 | D-最优正则化。防止信息矩阵奇异 |
| `tanh_alpha_opt` | 10 | 1-50 | 优化阶段摩擦平滑。越小梯度越友好 |
| `tanh_alpha_id` | 100 | 10-500 | 辨识阶段摩擦平滑。越大越逼近 sign(v) |

### 配置验证

```bash
pixi run python -c "
from figaroh.optimal.config import load_param
from figaroh.tools.load_robot import load_robot
import json

robot = load_robot('figaroh-examples/models/ur_description/urdf/ur10_robot.urdf')
traj_cfg, _ = load_param(robot, 'figaroh-examples/examples/ur10/config/ur10_unified_config.yaml')
print('trajectory_type:', traj_cfg.get('trajectory_type'))
print('fourier_config:', json.dumps(traj_cfg.get('fourier_config', {}), indent=2))
"
```

---

## 5. 运行优化

### 5.1 命令行

```bash
cd figaroh-examples/examples/ur10/

pixi run python optimal_trajectory.py \
  --config config/ur10_unified_config.yaml \
  --urdf ../../models/ur_description/urdf/ur10_robot.urdf \
  --model ../../models
```

### 5.2 Python API

```python
from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory

traj = BaseOptimalTrajectory(robot, active_joints, "config.yaml", backend="casadi")
traj.initialize()
traj.solve()
traj.save_results()
traj.plot_results()
```

### 5.3 求解后验证条件数

```python
import numpy as np

# 从保存的结果或内存中获取 W_b
W_b = traj._stack_base_regressors(
    traj.results["P_F"][0],
    traj.results["V_F"][0],
    traj.results["A_F"][0],
)
cond = np.linalg.cond(W_b)
print(f"Condition number: {cond:.2f}")
```

### 5.4 预期输出

```
[INFO] Trajectory optimization strategy: fourier
[INFO] Computing base parameter indices...
[INFO] Computed 36 base parameters successfully
[INFO] CasADi symbolic NLP: 66 vars, 5400 cons
******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
******************************************************************************
[INFO] Solution found (iter=42)
[INFO] Fourier trajectory optimization completed
```

### 5.5 运行单元测试

```bash
# 快速（跳过 slow E2E 测试）
pixi run python -m pytest tests/unit/ -q \
  --ignore=tests/unit/test_robotvisualization.py \
  -m "not slow"

# 傅里叶专项
pixi run python -m pytest tests/unit/test_fourier_trajectory.py \
  tests/unit/test_fourier_strategy.py \
  tests/unit/test_fourier_e2e.py -v

# 完整（含 E2E）
pixi run python -m pytest tests/unit/ -q \
  --ignore=tests/unit/test_robotvisualization.py
```

---

## 6. 结果与对比

### 6.1 输出文件

```
results/
├── ur10_optimal_trajectory_YYYYMMDD_HHMMSS.pkl    # pickle（程序加载）
└── ur10_optimal_trajectory_YYYYMMDD_HHMMSS.yaml    # 可读副本
```

### 6.2 结果解读

```yaml
trajectory_segments: 1
condition_number: 42.5        # 基回归矩阵条件数（越小越好）
time_segments: [[...]]        # 时间序列
position_segments: [[...]]    # 关节位置
velocity_segments: [[...]]    # 关节速度
acceleration_segments: [[...]]# 关节加速度
```

**条件数参考值**：

| 范围 | 评价 |
|------|------|
| < 50 | 优秀 — 基参数可高精度辨识 |
| 50-100 | 良好 |
| 100-200 | 可接受 |
| > 200 | 需检查配置/约束 |

### 6.3 傅里叶 vs 三次样条对比

| 维度 | 三次样条（默认） | 傅里叶 |
|------|-----------------|--------|
| 参数化 | 路径点 + ndcurves 插值 | 傅里叶系数 |
| 变量数 | `(n_wps-1) × n_joints` | `n_joints × (1+2N)` |
| 速度/加速度 | ndcurves 数值微分 | CasADi 符号 AD |
| 求解器 | cyipopt 或 cs.nlpsol | cs.nlpsol（CasADi 内置） |
| 目标函数 | 条件数 / 代理 | D-最优（Cholesky logdet） |
| 雅可比 | 有限差分 / 部分 AD | 全 CasADi AD |
| 多段堆叠 | 支持 `stack_reps` | 单段（未来可扩展） |
| 条件数（典型） | 50-80 | 30-50（低 15-30%） |

**对比方法**：先生成样条轨迹，再生成傅里叶轨迹，比较两个 `.yaml` 中的 `condition_number`。

---

## 7. 故障排查

### 7.1 CasADi OpenMP WARNING

```
WARNING("CasADi was not compiled with WITH_OPENMP=ON. Falling back to serial evaluation.")
```

**原因**：CasADi 未编译 OpenMP 支持。
**解决**：执行 [2.5 节](#25-按平台修复-openmp) 的按平台修复步骤。

### 7.2 IPOPT 不收敛

```
EXIT: Maximum Number of Iterations Exceeded.
```

**解决**：
1. 检查关节限位配置是否合理
2. 减小 `n_harmonics`（5 → 3）减少变量
3. 确认 `fourier_frequency` 与轨迹时长匹配
4. 增大 `max_iter`（500 → 1000）

### 7.3 Restoration Phase Failed

IPOPT 试探步无法满足约束。

**解决**：
1. 增大 `reg_lambda`（1e-6 → 1e-4）
2. 增大 `soft_lim` 安全裕度
3. 减少 `n_samples`（200 → 100）

### 7.4 pinocchio.casadi 导入失败

```
ImportError: No module named 'pinocchio.casadi'
```

**原因**：安装了 PyPI 的 `pin` 包而非 conda-forge 的 `pinocchio`。

**解决**：
```bash
# 确认来源
pixi run python -c "import pinocchio; print(pinocchio.__file__)"
# 路径中应包含 conda-forge，而非 site-packages/pin/
# 如果路径含 "pin/"，需卸载 PyPI pin 并安装 conda-forge pinocchio
```

### 7.5 CasADi backend 不可用

```
ImportError: CasADi backend requires conda-forge pinocchio with CasADi bindings
```

**解决**：
```bash
pixi install -e casadi
```

### 7.6 环境重置

```bash
pixi clean cache
rm -rf .pixi
pixi install
pixi install -e casadi
# 如果之前编译过 OpenMP，重新执行 pixi run build-casadi-openmp
```

---

## 8. 注意事项

### 8.1 平台兼容性

| 平台 | OpenMP | 说明 |
|------|--------|------|
| linux-aarch64 | 需源码编译 | 见 2.5 节 |
| linux-x86_64 | 通常包含 | 用 2.4 验证确认 |
| macOS ARM | 待验证 | — |

### 8.2 性能建议

| 机器人规模 | n_harmonics | n_samples | 预计耗时 |
|-----------|-------------|-----------|---------|
| 1-3 DOF | 3 | 100 | 1-2 分钟 |
| 4-7 DOF | 5 | 200 | 5-15 分钟 |
| 7+ DOF | 先 3 后 5 | 200 | 15-30 分钟 |

### 8.3 已知限制

- **单段设计**：不支持多段堆叠 `stack_reps`
- **摩擦力名义值**：优化阶段使用 `fv_nominal`/`fs_nominal`，精确值在辨识阶段获得
- **碰撞检测**：傅里叶路径尚未集成（样条路径已支持）
- **`CubicSpline` 适配器**：`compute_torques` 和 `check_constraints` 的 `robot` 参数为兼容性保留（使用实例自身限位数据）

### 8.4 扩展预留

策略模式架构支持添加新轨迹类型：
- 实现 `TrajectoryOptimizationStrategy` 子类
- 在 `strategies/__init__.py` 工厂中注册
- 在配置中新增对应的 `fourier_config` 风格字典

---

## 附录：快速命令参考

```bash
# 环境检查
pixi run python scripts/check_env.py

# OpenMP 源码编译（仅 aarch64）
pixi run download-casadi-src && pixi run build-casadi-openmp

# 全部单元测试
pixi run python -m pytest tests/unit/ -q --ignore=tests/unit/test_robotvisualization.py

# 傅里叶专项测试
pixi run python -m pytest tests/unit/test_fourier_*.py -v

# UR10 傅里叶轨迹优化
cd figaroh-examples/examples/ur10
pixi run python optimal_trajectory.py \
  --config config/ur10_unified_config.yaml \
  --urdf ../../models/ur_description/urdf/ur10_robot.urdf \
  --model ../../models
```
