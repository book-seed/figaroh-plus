---
comet_change: symbolic-fourier-trajectory
role: technical-design
canonical_spec: openspec
---

# 全符号化傅里叶激励轨迹优化 — 技术设计

## 1. Context

FIGAROH 激励轨迹优化仅支持三次样条（ndcurves）。CasADi 路径中，样条通过 `cs.Callback` 以有限差分接入，目标函数使用代理（加速度平滑 + 速度激励）。本次设计引入五级傅里叶级数轨迹类型，构建 SX/MX 混合全符号 CasADi NLP 管线，实现 D-最优激励轨迹优化，并与三次样条通过策略模式并列共存。

## 2. Architecture

### 2.1 策略模式重构

```
BaseOptimalTrajectory (Context 容器)
├── robot, model, identif_config, trajectory_config
├── idx_b, idx_e (BaseParameterComputer)
├── results: Dict
│
├── strategy: TrajectoryOptimizationStrategy  ← 工厂 lazy import
│
├── initialize()      → 基参数计算 (共享)
├── solve(**kwargs)   → segments = strategy.solve(self)
│                     → 统一填充 self.results
├── save_results()    → 共享（List[np.ndarray] 格式不变）
└── plot_results()    → 共享

TrajectoryOptimizationStrategy (ABC)
├── SplineOptimizationStrategy   ← 三次样条多段堆叠 + cyipopt
└── FourierOptimizationStrategy  ← 全符号 MX NLP + cs.nlpsol("ipopt")
```

### 2.2 文件组织

```
src/figaroh/optimal/
├── base_optimal_trajectory.py    # 重构：Context 容器
├── strategies/
│   ├── __init__.py
│   ├── base_strategy.py          # TrajectoryOptimizationStrategy ABC
│   ├── spline_strategy.py        # 从 base_optimal_trajectory.py 迁移
│   └── fourier_strategy.py       # 全符号 CasADi NLP
├── contraints.py                 # 适配：新增符号约束构建
└── config.py                     # 扩展：策略配置解析 + 新字段默认值

src/figaroh/utils/
├── cubic_spline.py               # 不变
├── fourier_trajectory.py         # 新增：FourierTrajectory 类
└── base_trajectory.py            # 新增：BaseTrajectory ABC

src/figaroh/backend/
└── casadi.py                     # 扩展：regressor_function / rnea_function property
```

### 2.3 配置隔离

```yaml
trajectory_type: "fourier"  # "spline" | "fourier"

spline_config:
  n_wps: 5
  t_s: 0.5
  stack_reps: 2

fourier_config:
  n_harmonics: 5
  fourier_frequency: null    # null → 2π/T
  n_samples: 200
  reg_lambda: 1.0e-6
  tanh_alpha_opt: 10
  tanh_alpha_id: 100
```

所有新字段带默认值；缺失不改变现有行为（向后兼容）。工厂方法 lazy import：

```python
def _create_strategy(trajectory_type, **configs):
    if trajectory_type == "spline":
        from figaroh.optimal.strategies.spline_strategy import SplineOptimizationStrategy
        return SplineOptimizationStrategy(...)
    elif trajectory_type == "fourier":
        from figaroh.optimal.strategies.fourier_strategy import FourierOptimizationStrategy
        return FourierOptimizationStrategy(...)
```

## 3. SX/MX 混合符号管线

### 3.1 计算图架构

```
┌────────────────────────────────────────────────────────────────┐
│  MX 外层（优化变量 + AD）                                        │
│                                                                │
│  Z = MX.sym("coeffs", n_vars)      ← 傅里叶系数 (66 vars)      │
│  Q(Z,t) = a₀ + Σ[aₖsin + bₖcos]    ← 列主序 (n_act, N_s)      │
│  V(Z,t) = ∂Q/∂t                     ← MX AD (符号微分)         │
│  A(Z,t) = ∂V/∂t                     ← MX AD (二阶)             │
│       │                                                        │
│       ▼                                                        │
│  ┌─────────────────────────────────┐                           │
│  │  SX 内层（单步动力学，已缓存）     │                           │
│  │                                 │                           │
│  │  regressor_function(q,v,a) → W  │  ← CasadiBackend property │
│  │  rnea_function(q,v,a) → τ_rnea  │  ← CasadiBackend property │
│  │                                 │                           │
│  │  .map(N_s, "openmp") ← 并行求值  │                           │
│  └─────────────────────────────────┘                           │
│       │                                                        │
│       ▼                                                        │
│  W_full = vertcat(W₁,...,W_Ns)     ← (N_s*nv, n_param)        │
│  W_b = W_full[:, idx_b]             ← 基参数列选择              │
│  J = W_bᵀW_b / N_s                  ← 信息矩阵 (n_base×n_base) │
│  L = cholesky(J + λI)               ← SPD 保证                 │
│  obj = -2 Σ log(L_ii)               ← D-最优目标               │
│                                                                │
│  τ = rnea(Q,V,A) + fv·V + fs·tanh(α·V)  ← 力矩约束            │
│  cons = [q_bounds, v_bounds, τ_bounds]                          │
│                                                                │
│  solver = cs.nlpsol("ipopt", nlp) → reverse-mode AD            │
└────────────────────────────────────────────────────────────────┘
```

### 3.2 内层 SX Function 缓存

`CasadiBackend` 提供 property 接口，避免外部直接读取私有属性：

```python
class CasadiBackend:
    @property
    def regressor_function(self) -> cs.Function:
        """SX Function: (q, v, a) → W(nv, n_param)"""
        self._ensure_symbolic_model()
        return self._W_fun

    @property
    def rnea_function(self) -> cs.Function:
        """SX Function: (q, v, a) → τ(nv)"""
        self._ensure_symbolic_model()
        return self._rnea_fun
```

缓存指纹覆盖全惯性参数 + 关节结构，SHA256 哈希。URDF 任何修改（质量、质心、惯量、关节类型/索引）自动导致指纹变化 → 缓存重建。

### 3.3 内存排布：列主序零 Transpose

轨迹 MX 表达式直接从源头构建为 `(n_act, N_s)` 列主序格式：

```python
# Q_col[j, i] = q_j(t_i) — 第 j 关节在采样点 i
Q_col = cs.MX.zeros(n_act, N_s)
for j in range(n_act):
    for i in range(N_s):
        Q_col[j, i] = fourier_expr(Z[j, :], omega_f, t[i])

# map 输入直接匹配，无 .T
W_mapped = cas_be.regressor_function.map(N_s, "openmp")(Q_col, V_col, A_col)
```

### 3.4 OpenMP 硬性要求

- `map` 默认 `"openmp"` 模式
- 环境验证脚本检测 OpenMP 可用性，不可用时**阻断并提示修复方法**
- 安装文档明确：conda-forge CasADi 默认带 OpenMP，自编译需 `-DWITH_OPENMP=ON`
- 不提供静默回退到 `"serial"` 的路径

## 4. D-最优目标函数

### 4.1 Cholesky 因式分解

```
传统: obj = -log det(J + λI)        ← cs.det — 数值精度风险
优化: L = cholesky(J + λI)           ← J+λI 数学上严格 SPD
      obj = -2 Σ_i log(L_ii)
```

正则化 λ 默认 1e-6（可配置），保证 J+λI 严格正定 → Cholesky 必定成功 → 不存在 Trial Step 触发 Restoration Phase Failed 的风险。

### 4.2 摩擦力统一模型

- 优化阶段：`f_s = fs · tanh(α_opt · v)`，α_opt 默认 10（梯度友好）
- 辨识阶段：`f_s = fs · tanh(α_id · v)`，α_id 默认 100（逼近精度）
- 统一在 CasADi SX/MX 图中构建，AD 可微
- 摩擦力名义值（fv, fs）来自 `params_std`，优化阶段使用名义值即可（误差在安全裕度内）

## 5. 系数初始化

```python
# 偏移：关节限位中位值
a0_init = (q_upper + q_lower) / 2
# 振幅：限位范围 5%
amp_init = 0.05 * (q_upper - q_lower)
# 谐波系数：小随机值
a_k_init = rng.uniform(-amp_init, amp_init)
b_k_init = rng.uniform(-amp_init, amp_init)
```

约束验证：初始系数生成轨迹后检查关限位/力矩约束，违反时缩小振幅（×0.5）重试，最多 N 次。

## 6. IPOPT 求解器

```python
opts = {
    "ipopt.linear_solver": "ma57",       # HSL 加速优先
    "ipopt.tol": 1e-6,
    "ipopt.max_iter": 500,
    "ipopt.mu_strategy": "adaptive",
}
solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)
```

- HSL 优先 `ma57`，不可用时回退 `mumps`（带日志警告）
- 使用 CasADi 自带 IPOPT（`cs.nlpsol`），不经过 cyipopt

## 7. 结果标准化

### 7.1 TrajectorySegment

```python
@dataclass
class TrajectorySegment:
    t: np.ndarray      # (N_s, 1)
    q: np.ndarray      # (N_s, nq)
    v: np.ndarray      # (N_s, nv)
    a: np.ndarray      # (N_s, nv)
    obj_value: float
    iterations: int
```

策略 `solve()` 返回 `List[TrajectorySegment]` — 均为 numpy 数组，策略边界隔离 CasADi 符号。

### 7.2 策略内部重构

`FourierOptimizationStrategy.solve()` 内部负责：
1. 提取最优系数 `x_opt`
2. 用 `fourier_q_fun(x_opt)` 数值求值生成 `(n_act, N_s)` 的 q/v/a numpy 数组
3. 装入 `TrajectorySegment` 返回

### 7.3 Context 统一封装

```python
# BaseOptimalTrajectory.solve()
segments = self.strategy.solve(self)
for seg in segments:
    self.results['T_F'].append(seg.t)
    self.results['P_F'].append(seg.q)
    self.results['V_F'].append(seg.v)
    self.results['A_F'].append(seg.a)
```

`self.results` 保持 `List[np.ndarray]` 格式不变，`save_results()` / `plot_results()` 完全复用。

### 7.4 条件数后处理

优化循环外用 numpy 事后计算（不进入 CasADi 图）：

```python
W_b = cas_be.regressor_function.map(N_s)(q_opt, v_opt, a_opt)
cond_val = float(np.linalg.cond(W_b))
```

## 8. 基参数获取

沿用 `BaseParameterComputer.compute_base_indices()`（随机三次样条 + QR），轨迹类型无关。`idx_b` 为 numpy 整数数组，在 CasADi MX 图中以列选择 `W_full[:, idx_b]` 形式使用。

## 9. 可扩展参数列

回归矩阵追加列遵循现有 `identif_config` 机制：

| 标志 | 列 |
|------|-----|
| `has_friction` | `fv·v_j` + `fs·tanh(α·v_j)` |
| `has_actuator_inertia` | `Ia·a_j` |
| `has_joint_offset` | `1.0` |
| 未来扩展 | 新增标志 + 列表达式 + 列偏移计数 |

列位置与数值后端保持一致。

## 10. Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| MX 图过大 | `map("openmp")` 并行 + HSL MA57 稀疏 Cholesky |
| 初始系数退化 | λI 正则化保证 SPD |
| HSL 不可用 | 环境验证阻断 + 安装文档明确步骤 |
| URDF 缓存过期 | 全惯性参数 SHA256 自动失效 |
| 摩擦力名义值不准 | 安全裕度吸收 20-30% 误差 |
| CasADi IPOPT 版本兼容 | pixi lock 锁定版本 |

## 11. Testing

- 傅里叶 v/a 与解析导数对照
- Cholesky logdet vs `cs.det` + numpy 数值对照
- 符号雅可比 vs 有限差分（容差 1e-5）
- UR10 端到端：`trajectory_type: "fourier"` → 条件数 < 随机轨迹
- `tanh(α·v)` 可微性
- 配置解析回归测试
- 环境验证脚本全绿

---

## 12. Documentation

Complete user-facing documentation has been created at:

- [`docs/fourier_trajectory.md`](../../fourier_trajectory.md) — YAML config reference, Python API, architecture overview, environment requirements, comparison with spline, and troubleshooting
- [`README.md`](../../../README.md) — Fourier trajectory section with quick-start config and environment requirements table
