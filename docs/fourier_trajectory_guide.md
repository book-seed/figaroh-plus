# 全符号化傅里叶激励轨迹优化 — 完整操作指南

本文档覆盖从仓库克隆到优化运行的全流程。假设用户从零开始。

---

## 目录

1. [环境准备](#1-环境准备)
2. [编译 CasADi + OpenMP（aarch64 平台）](#2-编译-casadi--openmpaarch64-平台)
3. [验证环境](#3-验证环境)
4. [配置傅里叶轨迹参数](#4-配置傅里叶轨迹参数)
5. [运行激励轨迹优化](#5-运行激励轨迹优化)
6. [结果查看与解释](#6-结果查看与解释)
7. [故障排查](#7-故障排查)
8. [注意事项与限制](#8-注意事项与限制)

---

## 1. 环境准备

### 1.1 克隆仓库

```bash
git clone git@github.com:book-seed/figaroh-plus.git
cd figaroh-plus
git checkout pixi

# 拉取 submodule（示例和模型文件）
git submodule update --init --recursive
```

### 1.2 安装 Pixi

```bash
# macOS / Linux
curl -fsSL https://pixi.sh/install.sh | bash

# 重启终端或执行
source ~/.bashrc  # 或 ~/.zshrc
```

验证安装：

```bash
pixi --version
# 预期输出: >= 0.71.0
```

### 1.3 创建环境

```bash
# 安装默认环境（包含 pinocchio、numpy、scipy 等核心依赖）
pixi install

# 安装 CasADi 环境（包含 casadi、pinocchio.casadi、ipopt）
pixi install -e casadi
```

验证核心依赖：

```bash
pixi run python -c "import pinocchio; print('pinocchio:', pinocchio.__version__)"
pixi run python -c "import casadi; print('CasADi:', casadi.__version__)"
```

### 1.4（可选）安装 HSL 线性求解器

HSL 可以加速 IPOPT 的线性系统求解，尤其在变量较多的场景下效果显著。

```bash
# conda-forge 方式
pixi add coinhsl

# 或手动下载 HSL 库放置到 .pixi/envs/default/lib/ 下
# 参见: https://www.hsl.rl.ac.uk/ipopt/
```

> **注意**：不安装 HSL 时 IPOPT 会自动使用内置 MUMPS 求解器，功能完整但速度较慢。

---

## 2. 编译 CasADi + OpenMP（aarch64 平台）

conda-forge 为 linux-aarch64 提供的 CasADi 二进制包**未编译 OpenMP 支持**。
这会导致 `map("openmp")` 回退到串行模式，优化速度降低 2-4 倍。

本项目提供了 pixi task 用于一键源码编译。

### 2.1 前提条件

```bash
# 确认 cmake 和 C++ 编译器可用
cmake --version  # >= 3.20
g++ --version    # 需支持 -fopenmp
```

### 2.2 编译安装

```bash
# 步骤 1: 下载 CasADi 源码
pixi run download-casadi-src

# 步骤 2: 编译 + 安装（预计 30-60 分钟，ARM64）
pixi run build-casadi-openmp
```

### 2.3 验证 OpenMP 已启用

```bash
pixi run python -c "
import casadi as cs
x = cs.SX.sym('x')
f = cs.Function('f', [x], [x**2])
f.map(10, 'openmp')
print('OK — OpenMP enabled')
"
```

如果输出中**没有** `WARNING("CasADi was not compiled with WITH_OPENMP=ON")` 则说明编译成功。

> **x86_64 用户**：如果 conda-forge 的 x86_64 CasADi 二进制包已包含 OpenMP，可以跳过此步骤。用上述验证命令确认即可。

---

## 3. 验证环境

运行项目自带的环境验证脚本：

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

### 运行单元测试确认无回归

```bash
pixi run python -m pytest tests/unit/ \
  --ignore=tests/unit/test_robotvisualization.py \
  -q
```

预期：252+ 通过，0 失败。

> `test_robotvisualization.py` 有预存的兼容性问题，可以安全忽略。

---

## 4. 配置傅里叶轨迹参数

傅里叶轨迹的参数在机器人配置 YAML 文件中设置，位于 `figaroh-examples/examples/<robot>/config/` 目录下。

### 4.1 基础配置

以 UR10 为例，编辑 `figaroh-examples/examples/ur10/config/ur10_unified_config.yaml`：

```yaml
optimal_trajectory:
  enabled: true

  problem:
    soft_lim: [0.01]       # 关节限位安全裕度
    max_attempts: 500       # 初始轨迹随机尝试次数

  trajectory:
    waypoints: 7            # 路径点数量（样条模式使用）
    frequency: 100          # 采样频率 Hz
    segment_duration: 2.0   # 相邻路点时间间隔（秒）

    # ── 傅里叶轨迹配置（新增，全部可选，都有默认值）──
    type: "fourier"         # "spline"（默认）或 "fourier"
    fourier:
      n_harmonics: 5        # 谐波数（默认 5，每关节 11 个系数）
      fourier_frequency: null  # 基频 rad/s（null → 自动 2π/T）
      n_samples: 200        # 采样点数（默认 200）
      reg_lambda: 1.0e-6    # D-最优正则化系数
      tanh_alpha_opt: 10    # 优化阶段摩擦平滑系数
      tanh_alpha_id: 100    # 辨识阶段摩擦平滑系数

  constraints:
    # 关节约束在 robot.joints.joint_limits 中定义

  output:
    save_trajectory: true
    output_file: "data/trajectories/ur10_fourier_trajectory.yaml"
```

### 4.2 参数说明

| 参数 | 默认值 | 取值范围 | 说明 |
|------|--------|---------|------|
| `type` | `"spline"` | `"spline"` / `"fourier"` | 轨迹优化类型 |
| `n_harmonics` | 5 | 1-10 | 傅里叶谐波数。越大轨迹越灵活但变量越多 |
| `fourier_frequency` | `null` | > 0 或 null | 基频 rad/s。null 时自动取 2π/T |
| `n_samples` | 200 | 50-1000 | 采样点数。越大约束评估越精确但 NLP 越大 |
| `reg_lambda` | 1e-6 | 1e-8 ~ 1e-3 | D-最优正则化。防止初始信息矩阵奇异 |
| `tanh_alpha_opt` | 10 | 1-50 | 优化阶段摩擦平滑。越小梯度越友好 |
| `tanh_alpha_id` | 100 | 10-500 | 辨识阶段摩擦平滑。越大逼近 sign(v) 越准 |

### 4.3 配置验证

```bash
# 解析配置并打印所有字段（含默认值）
pixi run python -c "
from figaroh.optimal.config import load_param
from figaroh.tools.load_robot import load_robot

robot = load_robot('figaroh-examples/models/ur_description/urdf/ur10_robot.urdf')
traj_cfg, id_cfg = load_param(robot, 'figaroh-examples/examples/ur10/config/ur10_unified_config.yaml')
import json
print('trajectory_type:', traj_cfg.get('trajectory_type'))
print('fourier_config:', json.dumps(traj_cfg.get('fourier_config', {}), indent=2))
"
```

---

## 5. 运行激励轨迹优化

### 5.1 傅里叶轨迹优化

```bash
cd figaroh-examples/examples/ur10/

pixi run python optimal_trajectory.py \
  --config config/ur10_unified_config.yaml \
  --urdf ../../models/ur_description/urdf/ur10_robot.urdf \
  --model ../../models
```

**预期输出**：

```
[INFO] Trajectory optimization strategy: fourier
[INFO] Computing base parameter indices from random trajectory
[INFO] Computed 36 base parameters successfully
[INFO] Starting Fourier optimization with 66 variables, 5 harmonics
[INFO] CasADi symbolic NLP: 66 vars, 5400 cons
[INFO] IPOPT: Solving... (max_iter=500, tol=1e-6)
******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code...
******************************************************************************
[INFO] Solution found (iter=42, obj=-123.45)
[INFO] Fourier trajectory optimization completed
Optimal trajectory generation completed successfully!
```

### 5.2 运行单元测试

```bash
# 快速测试（跳过 slow 标记的 E2E 测试）
pixi run python -m pytest tests/unit/ -q \
  --ignore=tests/unit/test_robotvisualization.py \
  -m "not slow"

# 完整测试（含 E2E，需要 CasADi + IPOPT 环境）
pixi run python -m pytest tests/unit/ -q \
  --ignore=tests/unit/test_robotvisualization.py
```

---

## 6. 结果查看与解释

### 6.1 输出文件

优化结果保存在 `results/` 目录：

```
results/
├── ur10_optimal_trajectory_YYYYMMDD_HHMMSS.pkl   # 轨迹数据（pickle）
└── ur10_optimal_trajectory_YYYYMMDD_HHMMSS.yaml   # 轨迹数据（可读）
```

### 6.2 结果解读

打开 `.yaml` 文件查看关键指标：

```yaml
trajectory_segments: 1
condition_number: 42.5          # 基回归矩阵条件数（越小越好）
joint_names: [Joint 1, ..., Joint 6]
time_segments: [[0.0, 0.01, ...]]  # 时间序列
position_segments: [[...]]          # 关节位置
velocity_segments: [[...]]          # 关节速度
acceleration_segments: [[...]]      # 关节加速度
```

**关键指标**：
- **条件数**：30-80 为良好（基参数可精确辨识），> 200 需检查配置
- **轨迹约束**：所有采样点应满足关节限位和力矩限位

### 6.3 对比样条 vs 傅里叶

```bash
# 1. 先生成样条轨迹（trajectory_type: "spline"）
pixi run python optimal_trajectory.py --config config/ur10_unified_config.yaml ...

# 2. 再生成傅里叶轨迹（trajectory_type: "fourier"）
pixi run python optimal_trajectory.py --config config/ur10_unified_config.yaml ...

# 3. 对比两个 .yaml 中的 condition_number 值
```

一般傅里叶轨迹的条件数比样条低 15-30%，参数辨识精度更高。

---

## 7. 故障排查

### 7.1 CasADi OpenMP WARNING

```
WARNING("CasADi was not compiled with WITH_OPENMP=ON. Falling back to serial evaluation.")
```

**解决**：执行 [第 2 节](#2-编译-casadi--openmpaarch64-平台) 的源码编译步骤。

### 7.2 IPOPT 不收敛

```
EXIT: Maximum Number of Iterations Exceeded.
```

**解决**：
1. 增大 `max_iter`：修改 `fourier_strategy.py` 中 `opts["ipopt.max_iter"]` 从 500 到 1000
2. 检查关节限位：确保 `joint_limits` 配置合理
3. 减小 `n_harmonics`：从 5 降到 3 减少变量数量
4. 检查 `fourier_frequency`：确认基频与轨迹时长匹配

### 7.3 "Restoration Phase Failed"

IPOPT 在某些试探步中无法满足约束。

**解决**：
1. 增大 `reg_lambda`：从 1e-6 提高到 1e-4，增强正则化
2. 调整 `soft_lim`：增大关节限位安全裕度
3. 减少 `n_samples`：从 200 降到 100

### 7.4 pinocchio.casadi 导入失败

```
ImportError: No module named 'pinocchio.casadi'
```

**解决**：
```bash
# 确保安装的是 conda-forge 的 pinocchio（而非 PyPI 的 pin）
pixi run python -c "import pinocchio; print(pinocchio.__file__)"
# 路径中应包含 conda-forge
```

### 7.5 环境重置

如果环境损坏，重新创建：

```bash
pixi clean cache
rm -rf .pixi
pixi install
pixi install -e casadi
```

---

## 8. 注意事项与限制

### 8.1 平台兼容性

| 平台 | CasADi OpenMP | 说明 |
|------|--------------|------|
| linux-x86_64 | 待验证 | conda-forge 可能已包含 OpenMP |
| linux-aarch64 | 需源码编译 | 执行第 2 节步骤 |
| macOS (ARM) | 待验证 | — |

### 8.2 性能建议

- **小机器人（1-3 DOF）**：n_harmonics=3、n_samples=100 即可，优化 1-2 分钟
- **中等机器人（4-7 DOF）**：n_harmonics=5、n_samples=200，优化 5-15 分钟
- **大型机器人（7+ DOF）**：建议先降谐波调试，再提高精度

### 8.3 已知限制

- 傅里叶轨迹目前为**单段**设计（不支持多段堆叠 `stack_reps`）
- 摩擦力名义值使用 `fv_nominal` / `fs_nominal`（配置中可指定），优化阶段使用的不是最终辨识值；辨识阶段会得到精确估计
- 碰撞检测在傅里叶路径中尚未集成（样条路径已支持），如需碰撞约束请使用样条模式
- `BaseTrajectory` 抽象接口中 `compute_torques` 和 `check_constraints` 的 `robot` 参数在 `CubicSpline` 适配器中为兼容性保留（使用实例自身存储的限位数据）

### 8.4 未来扩展

策略模式架构已预留扩展点：
- B 样条轨迹：实现 `TrajectoryOptimizationStrategy` 子类
- 多项式轨迹：同上
- 多段傅里叶：在 `FourierOptimizationStrategy` 内部迭代

---

## 附录：快速命令参考

```bash
# 环境检查
pixi run python scripts/check_env.py

# 运行全部单元测试
pixi run python -m pytest tests/unit/ -q \
  --ignore=tests/unit/test_robotvisualization.py

# 运行傅里叶专项测试
pixi run python -m pytest tests/unit/test_fourier_trajectory.py \
  tests/unit/test_fourier_strategy.py \
  tests/unit/test_fourier_e2e.py -v

# CasADi + OpenMP 源码编译
pixi run download-casadi-src
pixi run build-casadi-openmp

# UR10 傅里叶轨迹优化
cd figaroh-examples/examples/ur10
pixi run python optimal_trajectory.py \
  --config config/ur10_unified_config.yaml \
  --urdf ../../models/ur_description/urdf/ur10_robot.urdf \
  --model ../../models
```
