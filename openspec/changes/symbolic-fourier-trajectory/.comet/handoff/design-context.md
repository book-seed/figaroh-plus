# Comet Design Handoff

- Change: symbolic-fourier-trajectory
- Phase: design
- Mode: compact
- Context hash: 88db0d1083102a714ad5590f1eb2533b4c5914c51d22d177e8ced95b35498cca

Generated-by: comet-handoff.sh

OpenSpec remains the canonical capability spec. This handoff is a deterministic, source-traceable context pack, not an agent-authored summary.

## openspec/changes/symbolic-fourier-trajectory/proposal.md

- Source: openspec/changes/symbolic-fourier-trajectory/proposal.md
- Lines: 1-34
- SHA256: b5972e0e7e42e5f3ac2964b73c1c5466b683efb8bc9750208d1d4a38fc416297

```md
## Why

当前动力学辨识的激励轨迹优化仅支持三次样条曲线参数化，其速度和加速度依赖样条库数值微分，且 CasADi 优化路径中使用代理目标函数而非真正的条件数优化。引入五级傅里叶级数轨迹作为可选轨迹类型，并构建全符号化 CasADi 优化管线（Pinocchio CasADi 符号模型 → CasADi AD 求导 → D-最优目标 → IPOPT+HSL 求解），可显著提升参数辨识的数值稳定性与观测性，同时保持与现有三次样条方案的向后兼容。

## What Changes

- **新增**傅里叶级数轨迹类型（五级，每关节 11 参数），与三次样条并列可选（`trajectory_type` 配置切换）
- **新增**全符号化 CasADi NLP 管线：`cpin.Model` 符号模型 → 傅里叶系数为决策变量 → CasADi AD 符号求导得 v/a → `cpin.rnea` 力矩 → `cpin.computeJointTorqueRegressor` 回归矩阵 → QR 基参数选择 → D-最优目标 `log det(W_bᵀW_b/N_s + λI)` → IPOPT+HSL 求解
- **统一**优化阶段与辨识阶段的摩擦力模型为 `tanh(α·v)`（α 可配置，优化默认 10，辨识默认 100）
- **暴露**采样点数 `n_samples` 于配置文件，基频 ω_f 于配置文件（代码提供默认值 `2π/T`）
- **确保**回归矩阵构建顺序为"先符号求导，后 `map` 到采样点"，保证 AD 图完整
- **移植**现有 CasADi 符号模型构建骨架（`CasadiBackend._ensure_symbolic_model`），舍弃样条 Callback/代理目标/数值回退代码
- **扩展**可配置参数解析（新增字段全部带默认值，缺省不改变现有行为）
- **输出**完整可执行分步文档（环境搭建 → 配置 → 运行 → 验证）

## Capabilities

### New Capabilities

- `fourier-trajectory-parameterization`: 五级傅里叶级数轨迹参数化，每关节 1 偏移 + 5 正弦系数 + 5 余弦系数，基频可配置
- `symbolic-casadi-pipeline`: 全符号化 CasADi 优化管线，含符号模型构建、符号微分求 v/a、AD 求雅可比、D-最优目标
- `unified-friction-model`: 优化/辨识统一 `tanh(α·v)` 摩擦模型，α 可配置
- `configurable-sampling`: 采样点数 `n_samples` 和基频 `ω_f` 配置化暴露

### Modified Capabilities

- `excitation-trajectory-optimization`: 扩展为多轨迹类型架构，`trajectory_type` 配置选择 `"spline"`（默认）或 `"fourier"`

## Impact

- **受影响代码**: `src/figaroh/optimal/base_optimal_trajectory.py`（新增傅里叶 CasADi 优化路径）、`src/figaroh/utils/`（新增傅里叶轨迹模块）、`src/figaroh/backend/casadi.py`（扩展符号能力）、`src/figaroh/optimal/contraints.py`（符号约束适配）、`src/figaroh/optimal/config.py`（参数解析）
- **不受影响**: `calibration/`、`identification/`、`measurements/` 子包；三次样条功能；数值后端；QR 分解算法；数据保存/可视化接口
- **新增依赖**: CasADi IPOPT + HSL（`ma57`/`coinhsl`），`pinocchio.casadi` 绑定（已有 pixi casadi feature 基础）
- **配置变更**: 新增字段 `trajectory_type`、`fourier_frequency`、`n_samples`、`num_harmonics`、`reg_lambda`、`tanh_alpha` 等，全部带默认值
```

## openspec/changes/symbolic-fourier-trajectory/design.md

- Source: openspec/changes/symbolic-fourier-trajectory/design.md
- Lines: 1-145
- SHA256: 0d9f9d6cd92d55f488a324926efaabbb9a791b3e945d948a30375afe1e41e563

[TRUNCATED]

```md
## Context

FIGAROH 的激励轨迹优化当前仅支持三次样条曲线（`ndcurves`）。CasADi 优化路径中，样条通过 `cs.Callback` 以有限差分方式接入，目标函数使用代理（加速度平滑 + 速度激励），而非直接优化基回归矩阵条件数。本次设计在 `pixi` 分支代码基础上引入五级傅里叶级数轨迹类型，构建全符号化 CasADi NLP 管线，实现 D-最优激励轨迹优化。

## Goals / Non-Goals

**Goals:**
- 新增 `FourierTrajectory` 模块，五级傅里叶级数参数化（11 参数/关节），基频 ω_f 默认 `2π/T` 可配置
- 构建全符号化 CasADi NLP 管线：符号化 `cpin.Model` → 傅里叶系数决策变量 → AD 求 v/a → `cpin.rnea` 力矩 → `cpin.computeJointTorqueRegressor` → QR 基参数选择 → D-最优目标 → IPOPT+HSL 求解
- 统一 `tanh(α·v)` 摩擦模型于优化/辨识阶段
- 采样点数 `n_samples`、基频 `ω_f`、谐波数 `n_harmonics` 等配置化暴露
- 与三次样条并列共存，`trajectory_type` 切换，保持向后兼容
- 输出完整可执行文档

**Non-Goals:**
- 不修改/删除三次样条逻辑
- 不修改数值后端、QR 算法、数据保存/可视化
- 不涉及 `calibration/`、`identification/`、`measurements/` 子包

## Decisions

### D1: 轨迹类型架构 — 基类抽象 + 策略模式

```
BaseTrajectory        ← 抽象基类（生成 q(t), v(t), a(t)）
├── FourierTrajectory ← 五级傅里叶级数
└── CubicSplineTrajectory ← 现有三次样条（重构适配）
```

**Rationale**: 以最小侵入性加入新类型，不破坏现有逻辑。`BaseOptimalTrajectory` 通过 `trajectory_type` 配置选择具体策略。

**Alternatives considered**: 直接在现有类中 if/else 分支 → 拒绝，违反开闭原则，增加耦合。

### D2: 符号化模型构建 — 移植现有 `CasadiBackend` 骨架

从 `CasadiBackend._ensure_symbolic_model()` 移植：
- `cpin.Model(robot.model)` + `cmodel.createData()` 符号化模型和数据结构
- `cpin.computeJointTorqueRegressor(cmodel, cdata, cs_q, cs_v, cs_a)` 符号回归矩阵
- 磁盘缓存机制（`~/.figaroh/casadi_cache/`）

**舍去**：
- `_make_spline_callback()` — 样条 Callback，替换为傅里叶 CasADi SX 表达式
- `_make_objective_callback()` — 代理目标，替换为 D-最优
- `_compute_jacobian_sparsity()` — 数值雅可比回退，改为 CasADi AD
- `objective_function()` — numpy `cond(W_b)`，替换为符号 `log det`

### D3: 傅里叶轨迹 CasADi SX 表达式

```python
# q_j(t) = a_{j,0} + Σ_{k=1}^{5} [a_{j,k}*sin(k*ω_f*t) + b_{j,k}*cos(k*ω_f*t)]
# 决策变量: x = [a_{0,0}, a_{0,1}...a_{0,5}, b_{0,1}...b_{0,5}, a_{1,0}, ...]
# n_vars = n_joints * 11

cs_q = a0 + Σ_k (a_k * sin(k * ω_f * t_sym) + b_k * cos(k * ω_f * t_sym))
cs_v = jacobian(cs_q, t_sym)   # CasADi AD — 符号微分
cs_a = jacobian(cs_v, t_sym)   # CasADi AD — 二阶符号微分
```

**Rationale**: 傅里叶级数的 sin/cos 在 CasADi SX 中原生支持，整个轨迹生成链路在符号图中完成。

### D4: 目标函数 — D-最优 `log det(W_bᵀW_b/N_s + λI)`

- `W_b = W_e[:, idx_b]` — 按 QR 确定的基参数索引选择列
- `J = W_bᵀW_b / N_s` — 信息矩阵，除 N_s 消除采样点数影响
- `f = log det(J + λI)` — D-最优准则，λ 防初期奇异
- λ 默认 `1e-6`，可配置

**Rationale**: `log det` 在 CasADi 中完全可微（`cs.det`），无需 SVD。D-最优等价于最小化参数置信椭球体积，直接关联参数可辨识性。

**Alternatives considered**:
- `cond(W_b)` → 拒绝，需 SVD，CasADi 不可微
- A-最优 `tr((W_bᵀW_b)⁻¹)` → 可考虑但矩阵逆开销大
- T-最优 `trace(W_bᵀW_b)` → 过于简单，不考虑参数相关性

### D5: 求解器 — CasADi IPOPT + HSL

```python
opts = {
    "ipopt.linear_solver": "ma57",      # HSL 加速
    "ipopt.tol": 1e-6,
```

Full source: openspec/changes/symbolic-fourier-trajectory/design.md

## openspec/changes/symbolic-fourier-trajectory/tasks.md

- Source: openspec/changes/symbolic-fourier-trajectory/tasks.md
- Lines: 1-79
- SHA256: a4be3fc97ea98d5c662a2c9593ebb4acd784d468f0ea5e4949233b13ed56d6f2

```md
## 1. 环境依赖与验证

- [ ] 1.1 检查 pixi 环境中 CasADi 版本及 IPOPT 可用性，确认 `pinocchio.casadi` 绑定正常
- [ ] 1.2 安装/验证 HSL 线性求解器库（`libhsl.so` 或 `libcoinhsl.so`），确认 IPOPT 可调用 `ma57`
- [ ] 1.3 编写环境验证脚本，输出 CasADi、IPOPT、HSL、pinocchio.casadi 各组件状态
- [ ] 1.4 更新 `pixi.toml` casadi feature 依赖声明（如有缺失的 HSL/IPOPT 包）

## 2. 配置解析

- [ ] 2.1 在 `src/figaroh/optimal/config.py` 中新增字段解析：`trajectory_type`（默认 `"spline"`）、`fourier_frequency`（默认 `null` → `2π/T`）、`n_samples`（默认 200）、`n_harmonics`（默认 5）、`reg_lambda`（默认 1e-6）、`tanh_alpha_opt`（默认 10）、`tanh_alpha_id`（默认 100）
- [ ] 2.2 添加配置验证：`trajectory_type` 只接受 `"spline"` 或 `"fourier"`，拒绝非法值
- [ ] 2.3 编写配置解析单元测试：缺省字段走默认值、非法值报错、合法值正确覆盖

## 3. 轨迹模块抽象

- [ ] 3.1 新建 `src/figaroh/utils/base_trajectory.py`，定义 `BaseTrajectory` 抽象基类（接口：生成 q/v/a、约束检查、可视化）
- [ ] 3.2 重构 `CubicSpline`/`WaypointsGeneration` 为 `CubicSplineTrajectory(BaseTrajectory)`，保持原有接口不变
- [ ] 3.3 新建 `src/figaroh/utils/fourier_trajectory.py`，实现 `FourierTrajectory(BaseTrajectory)`，包含：
  - 傅里叶级数解析参数表达式（numpy 求值，用于数值后端和系数初始化）
  - 傅里叶级数 CasADi SX 符号表达式构建方法（用于 CasADi NLP）

## 4. CasADi 符号化管线核心

- [ ] 4.1 移植 `CasadiBackend._ensure_symbolic_model()` 到傅里叶优化路径：`cpin.Model` + `cmodel.createData()` + 磁盘缓存
- [ ] 4.2 实现傅里叶轨迹的 CasADi SX 表达式：q(t, coeffs)、v(t, coeffs) = ∂q/∂t、a(t, coeffs) = ∂²q/∂t²
- [ ] 4.3 实现符号化完整回归矩阵 W 构建：`cpin.computeJointTorqueRegressor` → 追加摩擦/惯量/偏置列（`tanh(α·v)`）→ QR `idx_b` 列选择 → `W_b`
- [ ] 4.4 实现 D-最优目标函数 `-log det(W_bᵀW_b/N_s + λI)` 为 CasADi SX 表达式
- [ ] 4.5 实现符号化约束：位置限位 `q_lower ≤ q ≤ q_upper`、速度限位 `v_lower ≤ v ≤ v_upper`、力矩限位 `cpin.rnea` 约束，均 CasADi SX
- [ ] 4.6 确保构建顺序：先构建符号 W(q,v,a) 函数 + CasADi AD 求导，再 `map` 到采样点并行求值

## 5. 系数初始化

- [ ] 5.1 实现智能初始化策略：偏移项 = 关节中位值，谐波系数 = 限位范围的 5-10% 小随机振幅
- [ ] 5.2 实现初始轨迹约束验证：检查初始傅里叶系数生成的轨迹是否违反关节限位/力矩约束，违反时缩小振幅重试
- [ ] 5.3 添加初始化失败处理：超过最大重试次数时抛出有意义错误信息

## 6. IPOPT 求解器集成

- [ ] 6.1 构建 CasADi `nlpsol("ipopt", nlp)` NLP 定义（变量边界、约束边界、目标、约束）
- [ ] 6.2 配置 IPOPT 选项：`linear_solver: ma57`（回退 `mumps`）、`tol: 1e-6`、`max_iter: 500`、`mu_strategy: adaptive`
- [ ] 6.3 实现 HSL 可用性检测和自动回退逻辑
- [ ] 6.4 实现求解结果提取：最优傅里叶系数 → 最优轨迹 q/v/a/τ

## 7. BaseOptimalTrajectory 集成

- [ ] 7.1 修改 `BaseOptimalTrajectory.__init__` 根据 `trajectory_type` 选择 `FourierTrajectory` 或 `CubicSplineTrajectory`
- [ ] 7.2 傅里叶路径跳过 `_generate_feasible_initial_guess` 的样条随机搜索，改用 `FourierTrajectory` 初始化策略
- [ ] 7.3 傅里叶路径的 `solve()` 使用 CasADi 符号管线（跳过 `WaypointsGeneration`、跳过 `TrajectoryConstraintManager` 数值约束）
- [ ] 7.4 确保 `save_results()` 和 `plot_results()` 对傅里叶轨迹输出格式一致

## 8. 约束管理适配

- [ ] 8.1 在 `TrajectoryConstraintManager` 中新增 `build_symbolic_constraints()` 方法，返回 CasADi SX 约束表达式和边界向量
- [ ] 8.2 确保符号约束边界与数值约束边界使用相同的关节限位数据源（`CubicSpline` 的 `lower_q`/`upper_q` 等）

## 9. CasADi Backend 清理

- [ ] 9.1 删除 `_make_spline_callback()` — 样条 Callback，不再需要
- [ ] 9.2 删除 `_make_objective_callback()` — 代理目标 Callback，不再需要
- [ ] 9.3 删除 `_make_constraint_callback()` — 约束 Callback，不再需要
- [ ] 9.4 删除 `objective_function()` — numpy `cond(W_b)`，不再需要
- [ ] 9.5 删除 `_compute_jacobian_sparsity()` — 数值雅可比回退，不再需要
- [ ] 9.6 保留 `CasadiBackend._ensure_symbolic_model()` 和 `build_regressor()` 供辨识阶段使用

## 10. 测试与验证

- [ ] 10.1 编写 `FourierTrajectory` 单元测试：傅里叶表达式 v/a 与解析导数一致
- [ ] 10.2 编写 D-最优目标单元测试：已知 W_b 手动计算 log det 对照
- [ ] 10.3 编写符号约束/回归雅可比与有限差分对照测试（容差 1e-5）
- [ ] 10.4 编写配置解析回归测试：现有配置文件不受影响
- [ ] 10.5 在 UR10 示例上端到端验证：`trajectory_type: "fourier"` 成功生成轨迹，条件数优于随机轨迹
- [ ] 10.6 验证 `tanh(α·v)` 在不同 α 值下的 CasADi 可微性

## 11. 文档

- [ ] 11.1 编写环境搭建文档：pixi 环境创建、CasADi+IPOPT+HSL 安装步骤、依赖验证脚本使用方法
- [ ] 11.2 编写配置文档：所有新增字段说明、默认值、取值约束、配置示例
- [ ] 11.3 编写运行文档：从 URDF 加载到激励轨迹生成的完整步骤、命令行参数说明
- [ ] 11.4 编写验证文档：如何验证轨迹有效性、如何验证 AD 正确性、如何对比样条与傅里叶轨迹辨识效果
```

## openspec/changes/symbolic-fourier-trajectory/specs/configurable-sampling/spec.md

- Source: openspec/changes/symbolic-fourier-trajectory/specs/configurable-sampling/spec.md
- Lines: 1-54
- SHA256: 2d1554f683e6b2bc0decf0df41987588118794dccfe502eba5a58ba8b491508a

```md
## ADDED Requirements

### Requirement: Configurable number of sample points

The system SHALL expose the number of trajectory sample points `n_samples` as a configuration parameter in the YAML config file, with a code-level default of 200.

#### Scenario: Default sample count

- **WHEN** `n_samples` is not specified in configuration
- **THEN** the system SHALL use 200 sample points

#### Scenario: Custom sample count

- **WHEN** `n_samples: 500` is specified in configuration
- **THEN** the trajectory SHALL be sampled at 500 evenly-spaced time points

### Requirement: Configurable fundamental frequency

The system SHALL expose the fundamental frequency `fourier_frequency` (ω_f in rad/s) as a configuration parameter in the YAML config file.

When `fourier_frequency` is not specified, the system SHALL default to `ω_f = 2π / T` where `T = t_s × (n_wps - 1)` is the total trajectory duration derived from the existing time-step configuration.

#### Scenario: Default frequency from duration

- **WHEN** `fourier_frequency` is absent from config and total duration is `T = t_s × (n_wps - 1)`
- **THEN** `ω_f` SHALL default to `2π / T`

#### Scenario: Explicit frequency override

- **WHEN** `fourier_frequency: 0.5` (rad/s) is specified in config
- **THEN** `ω_f` SHALL be `0.5`, regardless of trajectory duration

### Requirement: Trajectory type selection

The system SHALL support a `trajectory_type` configuration field accepting `"spline"` (default) or `"fourier"`, selecting the trajectory parameterization strategy at initialization time.

#### Scenario: Default trajectory type

- **WHEN** `trajectory_type` is absent from configuration
- **THEN** the system SHALL use cubic spline trajectory (backward compatible)

#### Scenario: Explicit Fourier selection

- **WHEN** `trajectory_type: "fourier"` is specified
- **THEN** the system SHALL use the Fourier series trajectory and the CasADi symbolic optimization pipeline

### Requirement: All new parameters have defaults

Every newly introduced configuration parameter SHALL have a sensible default value, and the absence of any new parameter in the YAML file SHALL NOT change the default system behavior (backward compatible).

#### Scenario: Empty config with new fields
- **WHEN** a configuration file containing only legacy fields is loaded
- **THEN** all new Fourier-related parameters SHALL assume their defaults
- **AND** the system SHALL function identically to before the change
```

## openspec/changes/symbolic-fourier-trajectory/specs/excitation-trajectory-optimization/spec.md

- Source: openspec/changes/symbolic-fourier-trajectory/specs/excitation-trajectory-optimization/spec.md
- Lines: 1-45
- SHA256: ad674043b3e1730798612816506332a9843bb471427f50c0fdd451e1d9e4ed9f

```md
## ADDED Requirements

### Requirement: Multi-trajectory-type architecture

The system SHALL abstract trajectory generation behind a common interface (`BaseTrajectory`) supporting multiple parameterization strategies, with concrete implementations for cubic spline (`CubicSplineTrajectory`) and Fourier series (`FourierTrajectory`).

The `BaseOptimalTrajectory` class SHALL select the active trajectory strategy based on the `trajectory_type` configuration field.

#### Scenario: Fourier path selection

- **WHEN** `trajectory_type: "fourier"` is configured
- **THEN** `BaseOptimalTrajectory` SHALL instantiate `FourierTrajectory` and route optimization through the CasADi symbolic NLP pipeline

#### Scenario: Spline path selection (default)

- **WHEN** `trajectory_type` is absent or set to `"spline"`
- **THEN** `BaseOptimalTrajectory` SHALL use the existing cubic spline pipeline unchanged

### Requirement: Backward compatibility with existing spline pipeline

The system SHALL preserve the exact behavior of the existing cubic-spline-based excitation trajectory optimization when `trajectory_type` is `"spline"` or unspecified. All existing configuration files, saved results, and API calls SHALL continue to work without modification.

#### Scenario: Legacy config works

- **WHEN** a pre-existing YAML configuration file without `trajectory_type` is loaded
- **THEN** the optimization SHALL behave identically to before the change
- **AND** all saved output formats SHALL remain compatible

### Requirement: Base parameter computation independence

The system SHALL continue to compute base parameter indices `idx_b` via the existing `BaseParameterComputer` (random cubic spline trajectory + QR decomposition), independent of the selected `trajectory_type`. This ensures base parameters are determined by the robot's structural properties rather than the trajectory parameterization.

#### Scenario: Same idx_b regardless of trajectory type

- **WHEN** `trajectory_type` is `"fourier"` or `"spline"`
- **THEN** `BaseParameterComputer.compute_base_indices()` SHALL produce the same `idx_b` for the same robot and `identif_config`

### Requirement: Extensible additional parameter columns

The system SHALL support dynamic column appending to the symbolic regressor matrix based on `identif_config` boolean flags (`has_friction`, `has_actuator_inertia`, `has_joint_offset`, `has_custom_parameters`). The column structure SHALL match the numerical backend's column ordering.

#### Scenario: New parameter type addition

- **WHEN** a future change adds a new flag (e.g., `has_spring_stiffness`) to `identif_config`
- **THEN** adding the corresponding column to the symbolic regressor SHALL require only defining the column expression and updating the column offset counter
```

## openspec/changes/symbolic-fourier-trajectory/specs/fourier-trajectory-parameterization/spec.md

- Source: openspec/changes/symbolic-fourier-trajectory/specs/fourier-trajectory-parameterization/spec.md
- Lines: 1-45
- SHA256: 8169d649dff06d2aa43b685f12d8fdc59bfe998c27f94d0bd68eb513880693de

```md
## ADDED Requirements

### Requirement: Fourier trajectory parameterization

The system SHALL support Fourier series trajectory parameterization with configurable number of harmonics (default 5), where each active joint trajectory is expressed as:

```
q_j(t) = a_{j,0} + Σ_{k=1}^{N} [a_{j,k}·sin(k·ω_f·t) + b_{j,k}·cos(k·ω_f·t)]
```

The system SHALL accept the fundamental frequency ω_f from configuration with a code-level default of `2π/T` (where T is total trajectory duration).

The system SHALL generate velocity and acceleration trajectories via CasADi symbolic differentiation of the Fourier position expression with respect to time.

The system SHALL use Fourier coefficients as optimization decision variables, with total variable count = `n_active_joints × (1 + 2 × n_harmonics)`.

#### Scenario: Generate Fourier trajectory from coefficients

- **WHEN** Fourier coefficients (a_k, b_k), fundamental frequency ω_f, and number of sample points N_s are provided
- **THEN** the system SHALL produce position q(t), velocity v(t) = ∂q/∂t, and acceleration a(t) = ∂²q/∂t² arrays of shape (N_s, n_joints), all computed via CasADi symbolic expressions

#### Scenario: Velocity and acceleration are symbolic derivatives

- **WHEN** the Fourier position expression q(t) is constructed as a CasADi SX expression
- **THEN** v(t) and a(t) SHALL be obtained via `cs.jacobian()` or `cs.gradient()` w.r.t. time variable, with no numerical finite-difference fallback

#### Scenario: Coefficient initialization avoids singularities and collisions

- **WHEN** initial Fourier coefficients are generated
- **THEN** the offset term a_0 SHALL be set to the joint range midpoint `(q_upper + q_lower) / 2`
- **AND** the harmonic coefficients a_k, b_k SHALL be small random values scaled to 5-10% of the joint range to avoid joint limit violations, self-collisions, and degenerate configurations

### Requirement: Multi-harmonic configurability

The system SHALL allow the number of Fourier harmonics to be configured via `n_harmonics` in the configuration file, defaulting to 5.

#### Scenario: Default harmonic count

- **WHEN** `n_harmonics` is not specified in configuration
- **THEN** the system SHALL use 5 harmonics (11 coefficients per joint)

#### Scenario: Custom harmonic count

- **WHEN** `n_harmonics: 3` is specified in configuration
- **THEN** the system SHALL use 3 harmonics (7 coefficients per joint)
```

## openspec/changes/symbolic-fourier-trajectory/specs/symbolic-casadi-pipeline/spec.md

- Source: openspec/changes/symbolic-fourier-trajectory/specs/symbolic-casadi-pipeline/spec.md
- Lines: 1-149
- SHA256: 00d4368f0d66cd56b287da93c9823daa291c4d796c0d983a34cb6fd1dd1219d6

[TRUNCATED]

```md
## ADDED Requirements

### Requirement: Symbolic CasADi model from URDF

The system SHALL construct a fully symbolic robot model using `pinocchio.casadi.Model` from the robot's URDF description, along with a corresponding `cpin.Model.createData()` symbolic data structure.

#### Scenario: Symbolic model construction

- **WHEN** a robot with a valid URDF is loaded
- **THEN** `cpin.Model(robot.model)` SHALL produce a CasADi symbolic model
- **AND** `cmodel.createData()` SHALL allocate symbolic data structures
- **AND** both SHALL be cached to disk at `~/.figaroh/casadi_cache/` for subsequent reuse

### Requirement: Cache fingerprint covers full inertial parameters

The system SHALL compute a cache fingerprint that covers ALL inertial parameters and joint structure, using SHA256 hash of: model name, nq, nv, each body's mass (12 decimal places), center-of-mass lever arm coordinates (12dp), inertia tensor diagonal entries (12dp), and each joint's short-name, idx_q, and idx_v. Any modification to the URDF (mass, COM, inertia, joint type, or joint index) SHALL produce a different fingerprint, causing automatic cache invalidation and rebuild.

#### Scenario: URDF parameter change invalidates cache

- **WHEN** the URDF is modified (e.g., a link's mass or COM is changed)
- **THEN** the cache fingerprint SHALL differ from the previous value
- **AND** the cached symbolic functions SHALL be rebuilt from the updated model

#### Scenario: URDF unchanged reuses cache

- **WHEN** the URDF is identical to a previously cached version
- **THEN** the cached symbolic functions SHALL be loaded from disk without rebuild

### Requirement: SX/MX hybrid architecture

The system SHALL use a hybrid SX/MX architecture: inner-layer single-step dynamics functions (regressor and RNEA) SHALL be built and cached as CasADi SX Functions; outer-layer trajectory optimization SHALL use CasADi MX symbols for Fourier coefficients and construct the NLP graph by calling inner SX Functions via `cs.Function.map(N_s, "openmp")`. Gradient computation SHALL use MX reverse-mode automatic differentiation for efficient many-input-to-few-output Jacobians.

#### Scenario: SX inner functions are cached

- **WHEN** `CasadiBackend` constructs symbolic dynamics functions
- **THEN** `regressor_function` SHALL be an SX `cs.Function(q,v,a) → W(nv, n_param)` cached to disk
- **AND** `rnea_function` SHALL be an SX `cs.Function(q,v,a) → τ(nv)` cached to disk
- **AND** both SHALL be accessible via `CasadiBackend` property interfaces

#### Scenario: MX outer layer calls SX via map

- **WHEN** the MX optimization graph is constructed from Fourier coefficient variables
- **THEN** the inner SX Functions SHALL be invoked via `.map(N_s, "openmp")` with column-major MX inputs
- **AND** all trajectory variables (Q, V, A) SHALL be built directly as `(n_act, N_s)` column-major MX expressions to eliminate Transpose nodes

### Requirement: Symbolic regressor matrix construction

The system SHALL construct the full joint torque regressor matrix `W` symbolically via `cpin.computeJointTorqueRegressor(cmodel, cdata, cs_q, cs_v, cs_a)`.

The system SHALL build the base regressor `W_b` by selecting columns from W using the pre-computed base parameter indices `idx_b` (from `BaseParameterComputer`).

The system SHALL append additional parameter columns (friction fv/fs, actuator inertia Ia, joint offset) to the symbolic regressor when enabled in `identif_config`, before base-parameter column selection.

#### Scenario: Regressor with additional parameters

- **WHEN** `identif_config.has_friction: true`
- **THEN** the symbolic regressor SHALL include viscous friction columns `fv · v_j` and static friction columns `fs · tanh(α·v_j)` for each active joint
- **AND** the column position SHALL match the numerical backend's column order

### Requirement: Build-order: symbolic function first, then map

The system SHALL construct the regressor as a CasADi SX symbolic function `W_func(cs_q, cs_v, cs_a)` FIRST, compute its Jacobian via CasADi AD, and THEN apply `cs.Function.map(N_s)` to evaluate over sample points in parallel.

#### Scenario: Jacobian in AD graph

- **WHEN** the constraint Jacobian or regressor Jacobian is required
- **THEN** it SHALL be computed by `cs.jacobian()` on the symbolic expression graph BEFORE `map()` is applied
- **AND** the resulting Jacobian SHALL match finite-difference verification to within `1e-5` relative tolerance

### Requirement: D-optimal objective function with Cholesky factorization

The system SHALL minimize the D-optimality criterion using Cholesky factorization:

```
J = W_b(x)ᵀ · W_b(x) / N_s + λ·I    (SPD guaranteed by λ > 0)
L = cholesky(J)
f(x) = -2 · Σ_i log(L_ii)
```

where `λ` defaults to `1e-6` and SHALL be configurable via `reg_lambda`. The regularization term `λ·I` guarantees `J` is strictly positive definite, ensuring Cholesky factorization always succeeds without triggering IPOPT restoration phase failures.
```

Full source: openspec/changes/symbolic-fourier-trajectory/specs/symbolic-casadi-pipeline/spec.md

## openspec/changes/symbolic-fourier-trajectory/specs/unified-friction-model/spec.md

- Source: openspec/changes/symbolic-fourier-trajectory/specs/unified-friction-model/spec.md
- Lines: 1-35
- SHA256: 9094a1b5844358e5856a4675cd9aa1b88f8a2511ad3710bbb707c7f5aa85cea7

```md
## ADDED Requirements

### Requirement: Unified tanh friction model for optimization and identification

The system SHALL use `tanh(α·v)` as the friction sign approximation in BOTH the trajectory optimization regressor and the parameter identification regressor, ensuring model consistency between phases.

The tanh steepness parameter `α` SHALL be independently configurable for optimization (`tanh_alpha_opt`, default 10) and identification (`tanh_alpha_id`, default 100).

#### Scenario: Optimization uses gradient-friendly α

- **WHEN** the CasADi symbolic regressor is built during trajectory optimization
- **THEN** the static friction column SHALL use `tanh(tanh_alpha_opt · v)`
- **AND** `tanh_alpha_opt` SHALL default to 10 to avoid sharp gradients at v=0

#### Scenario: Identification uses high-precision α

- **WHEN** the numerical regressor is built during parameter identification
- **THEN** the static friction column SHALL use `tanh(tanh_alpha_id · v)`
- **AND** `tanh_alpha_id` SHALL default to 100 for closer approximation to `sign(v)`

#### Scenario: tanh is differentiable in CasADi graph

- **WHEN** the objective or constraint Jacobian is computed via CasADi AD
- **THEN** `tanh(α·v)` SHALL be fully differentiable through the CasADi expression graph
- **AND** the gradient at |v| > 3/α SHALL be effectively zero (saturated), avoiding exploding gradients

### Requirement: Configurable tanh parameters

Both `tanh_alpha_opt` and `tanh_alpha_id` SHALL be configurable in the robot configuration YAML file, with the stated defaults applied when absent.

#### Scenario: Default values applied

- **WHEN** neither `tanh_alpha_opt` nor `tanh_alpha_id` is specified in config
- **THEN** `tanh_alpha_opt` SHALL default to 10
- **AND** `tanh_alpha_id` SHALL default to 100
```

