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
