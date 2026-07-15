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
    "ipopt.max_iter": 500,
    "ipopt.mu_strategy": "adaptive",
}
solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)
```

**Rationale**: HSL MA57 对结构化稀疏 Hessian（傅里叶轨迹的带宽特性）有更好的因式分解性能。回退到 `mumps` 当 HSL 不可用时。

### D6: 摩擦力模型 — 统一 `tanh(α·v)`

优化阶段 α 默认 10（梯度友好），辨识阶段 α 默认 100（逼近精度），均可配置。

**Rationale**: 一致模型消除优化→辨识的系统性偏差。α 适中避免梯度在 v=0 处过于尖锐，影响 IPOPT 收敛。

**Alternatives considered**:
- 代数正则化 `v/√(v²+ε²)` → CasADi 图更轻量但物理逼近精度差，且与辨识阶段 tanh 不一致
- 两阶段不同模型 → 拒绝，不一致引入辨识偏差

### D7: 构建顺序 — 先求导后 Map

```
符号 W(q,v,a) → cs.jacobian(W, coeffs) → map(N_s, "openmp")
                 ^^^ 符号图中完成 AD    ^^^ 再并行映射到采样点
```

**Rationale**: 先构建符号 W 及其导数函数，再 `map` 到采样点并行求值，确保 Jacobian 在 AD 图中计算而非事后数值差商。

### D8: 系数初始化策略

```python
# 偏移: 关节限位中位值
a0_init = (q_upper + q_lower) / 2
# 振幅: 限位范围的 5-10%，避免撞击/奇异/重力矩抵消
amp_init = 0.05 * (q_upper - q_lower)
# 正弦系数: 小随机值 × 振幅
a_k_init = rng.uniform(-amp_init, amp_init)
# 余弦系数: 小随机值 × 振幅
b_k_init = rng.uniform(-amp_init, amp_init)
```

**Rationale**: 小振幅初值避免初始轨迹违反关节限位和自碰撞。经 IPOPT 优化后振幅自然增大至约束边界。

### D9: 模块文件组织

```
src/figaroh/utils/
├── cubic_spline.py              # 现有文件，不变
├── fourier_trajectory.py        # 新增: FourierTrajectory 类 + 符号化表达式
├── base_trajectory.py           # 新增: BaseTrajectory 抽象基类

src/figaroh/optimal/
├── base_optimal_trajectory.py   # 修改: 调度逻辑，集成 FourierTrajectory
├── contraints.py                # 修改: 新增符号约束构建方法
├── config.py                    # 修改: 新增字段解析
```

## Risks / Trade-offs

| Risk | Mitigation |
|------|-----------|
| **HSL 不可用** — 用户环境未安装 HSL 库 | 自动回退 `mumps`，检测逻辑在 `_ensure_symbolic_model` 阶段 |
| **傅里叶系数维度过高** — 6 DOF 机器人 66 个变量，IPOPT 可能收敛慢 | 利用 HSL 的稀疏求解，或提供 `n_harmonics` 配置减少谐波数 |
| **D-最优在极低速度区退化** — 行列式接近 0 | λ 正则化 + 系数初始化小振幅确保初始轨迹可行 |
| **符号图过大** — N_s 过大时符号函数 map 内存爆炸 | `n_samples` 默认合理值（~200），用户可调；文档警告大值风险 |
| **Pinocchio CasADi 绑定缺失** — PyPI 的 `pin` 包不支持 CasADi | pixi 环境用 conda-forge pinocchio，已有 casadi feature 基础 |
