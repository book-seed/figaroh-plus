# FIGAROH 最优轨迹 — 类关系文档

> 覆盖 UR10 例程中的最优轨迹生成全链路，含三次样条和傅里叶两条路径。

---

## 总体架构

激励轨迹优化通过**策略模式**支持两种轨迹类型：

| 策略 | 求解器 | 轨迹参数化 |
|------|--------|-----------|
| `SplineOptimizationStrategy` | cyipopt（数值）或 cs.nlpsol（CasADi） | 三次样条（ndcurves） |
| `FourierOptimizationStrategy` | cs.nlpsol（CasADi 内置 IPOPT） | 五级傅里叶级数（全符号 MX NLP） |

两条路径共享 Context 容器（`BaseOptimalTrajectory`）、基参数计算（`BaseParameterComputer`）、结果保存/可视化。

---

## 1. 策略架构（v0.4.3+）

```
BaseOptimalTrajectory (Context 容器)
│
├── trajectory_config, identif_config
├── _backend: Backend
├── base_computer: BaseParameterComputer
│
├── strategy: TrajectoryOptimizationStrategy   ← 工厂 lazy import
│   │
│   ├── SplineOptimizationStrategy             ← 三次样条 + cyipopt
│   │     └── 持有 WaypointsGeneration (extends CubicSpline → BaseTrajectory)
│   │
│   └── FourierOptimizationStrategy            ← 傅里叶 + cs.nlpsol("ipopt")
│         └── 使用 FourierTrajectory (BaseTrajectory)
│
├── initialize()  →  基参数计算 (共享)
├── solve()       →  strategy.solve(self) → 统一填充 results
├── save_results()/plot_results() → 共享
└── results: Dict (List[np.ndarray] 格式不变)
```

**工厂函数**（`src/figaroh/optimal/strategies/__init__.py`）：
```python
def create_strategy(trajectory_type, **configs):
    if trajectory_type == "spline":
        from figaroh.optimal.strategies.spline_strategy import SplineOptimizationStrategy
        return SplineOptimizationStrategy(...)
    elif trajectory_type == "fourier":
        from figaroh.optimal.strategies.fourier_strategy import FourierOptimizationStrategy
        return FourierOptimizationStrategy(...)
```

---

## 2. 轨迹模块抽象

```
BaseTrajectory (ABC)                       # src/figaroh/utils/base_trajectory.py
├── get_trajectory(t, coeffs) → np.ndarray
├── get_velocity(t, coeffs) → np.ndarray
├── get_acceleration(t, coeffs) → np.ndarray
├── compute_torques(q, v, a, robot) → np.ndarray
└── check_constraints(q, v, tau, robot) → bool
    │
    ├── CubicSpline (Backward Compatible)   # src/figaroh/utils/cubic_spline.py
    │     ├── 保持所有原有 API 不变
    │     ├── 新增 5 个 BaseTrajectory 适配器方法
    │     └── WaypointsGeneration 自动继承
    │
    └── FourierTrajectory                    # src/figaroh/utils/fourier_trajectory.py
          ├── numpy 解析求值 (q/v/a)
          ├── CasADi SX 符号表达式构建
          └── 系数初始化智能策略
```

---

## 3. 路径一：三次样条轨迹（SplineOptimizationStrategy）

### 入口调用链

```
optimal_trajectory.py::main()
  │
  ├── load_robot()
  ├── BaseOptimalTrajectory(robot, active_joints, config)
  │     └── __init__: 根据 trajectory_type 创建策略（默认 spline）
  ├── traj.initialize()
  │     └── BaseParameterComputer.compute_base_indices() → idx_b
  └── traj.solve()
        └── strategy.solve(context)  → SplineOptimizationStrategy
              ├── WaypointsGeneration → 随机路点 → 样条插值
              ├── create_ipopt_problem() → BaseTrajectoryIPOPTProblem
              ├── problem.solve_with_waypoints(wps)
              │     ├── numerical: cyipopt.Problem
              │     └── casadi: cs.nlpsol (deprecated, 建议用 Fourier)
              └── 填充 context.results['T_F', 'P_F', 'V_F', 'A_F']
```

### 关键类层次（样条路径专用）

```
BaseOptimizationProblem (ABC)              # figaroh/tools/robotipopt.py
  ▲
  │
BaseTrajectoryIPOPTProblem                 # figaroh/optimal/base_optimal_trajectory.py
  ├── get_variable_bounds()
  ├── get_constraint_bounds()
  ├── objective() / constraints() / jacobian()
  └── solve_with_waypoints()
      ▲
      │
UR10TrajectoryIPOPTProblem                 # ur10/utils/ur10_tools.py
  └── (机器人特定的约束/边界实现)
```

---

## 4. 路径二：傅里叶轨迹（FourierOptimizationStrategy）

### SX/MX 混合计算图

```
决策变量
  Z = MX.sym("coeffs", n_vars)          ← 66 vars (6-DOF, n_harmonics=5)
    │
    ▼
Q(Z,t) = a₀ + Σ[aₖsin(kωt) + bₖcos(kωt)]  ← 列主序 (n_act, N_s)，零 Transpose
V(Z,t) = ∂Q/∂t   (MX AD)
A(Z,t) = ∂V/∂t   (MX AD)
    │
    ├── regressor_function.map(N_s, "openmp")(Q,V,A) → W_full
    │     └── 内层 SX Function (CasadiBackend 缓存)
    │
    └── rnea_function.map(N_s, "openmp")(Q,V,A) → τ_rnea
          └── 内层 SX Function (CasadiBackend 缓存)
    │
W_b = W_full[:, idx_b]
J = W_bᵀW_b / N_s
L = cholesky(J + λI)                     ← SPD 保证
obj = -2 Σ log(L_ii)                     ← D-最优
    │
τ = τ_rnea + fv·V + fs·tanh(α_opt·V)     ← 力矩约束含摩擦
    │
cs.nlpsol("ipopt", nlp)                  ← 内置 IPOPT (MUMPS)
```

### 关键类

```
FourierOptimizationStrategy              # strategies/fourier_strategy.py
├── __init__(fourier_config)
├── solve(context)
│     ├── 构建 MX 符号图 (q/v/a/obj/cons)
│     ├── cs.nlpsol("ipopt").solve(x0)
│     └── 最优系数 → FourierTrajectory → numpy → context.results
└── _initialize_coefficients(context)     # 智能初始化
```

---

## 5. CasADi Backend（共享）

```
Backend (ABC)                              # figaroh/backend/base.py
  ▲
  │
CasadiBackend                             # figaroh/backend/casadi.py
├── regressor_function (property)         # SX Function: (q,v,a) → W
├── rnea_function (property)             # SX Function: (q,v,a) → τ
├── _ensure_symbolic_model()             # cpin.Model + 磁盘缓存
└── _cache_key()                          # SHA256 (全惯性参数 + 关节结构)
```

---

## 6. 辅助类（两条路径共享）

```
BaseOptimalTrajectory 持有:
  │
  ├── _backend: Backend
  │     ├── build_regressor()   (辨识阶段)
  │     ├── regressor_function  (优化阶段, CasADi only)
  │     └── rnea_function       (优化阶段, CasADi only)
  │
  ├── base_computer: BaseParameterComputer
  │     └── compute_base_indices()  → idx_e, idx_b  (QR 分解)
  │
  ├── constraint_manager: TrajectoryConstraintManager
  │     ├── evaluate_constraints()          (样条路径)
  │     └── build_symbolic_constraints()    (傅里叶路径 — 未集成)
  │
  └── results: Dict
        ├── T_F, P_F, V_F, A_F     (List[np.ndarray])
        ├── iteration_data
        └── final_regressor_shape
```

---

## 7. 关系矩阵

| 类 | 文件 | 继承/实现 | 角色 |
|----|------|----------|------|
| `BaseOptimalTrajectory` | `optimal/base_optimal_trajectory.py` | — | Context 容器 |
| `TrajectoryOptimizationStrategy` | `strategies/base_strategy.py` | ABC | 策略接口 |
| `SplineOptimizationStrategy` | `strategies/spline_strategy.py` | `TrajectoryOptimizationStrategy` | 样条路径 |
| `FourierOptimizationStrategy` | `strategies/fourier_strategy.py` | `TrajectoryOptimizationStrategy` | 傅里叶路径 |
| `BaseTrajectory` | `utils/base_trajectory.py` | ABC | 轨迹生成接口 |
| `CubicSpline` | `utils/cubic_spline.py` | `BaseTrajectory` | 三次样条 |
| `WaypointsGeneration` | `utils/cubic_spline.py` | `CubicSpline` | 路点生成 |
| `FourierTrajectory` | `utils/fourier_trajectory.py` | `BaseTrajectory` | 傅里叶级数 |
| `CasadiBackend` | `backend/casadi.py` | `Backend` | SX 符号动力学 |
| `BaseParameterComputer` | `optimal/base_parameter.py` | — | 基参数索引 |
| `TrajectoryConstraintManager` | `optimal/contraints.py` | — | 约束管理 |
| `BaseOptimizationProblem` | `tools/robotipopt.py` | ABC | IPOPT 问题抽象 |
| `BaseTrajectoryIPOPTProblem` | `optimal/base_optimal_trajectory.py` | `BaseOptimizationProblem` | 样条 IPOPT 问题 |

---

## 8. 关键设计模式

| 模式 | 应用处 | 说明 |
|------|--------|------|
| **Strategy** | `TrajectoryOptimizationStrategy` → Spline / Fourier | 运行时切换优化管线 |
| **Template Method** | `BaseOptimalTrajectory.initialize()` → `solve()` | 流程骨架固定，策略实现 `solve(context)` |
| **Abstract Factory** | `create_strategy(trajectory_type)` | Lazy import，配置驱动 |
| **Facade** | `CasadiBackend.regressor_function` / `rnea_function` | 隐藏 cpin 构建和缓存细节 |
| **ABC** | `BaseTrajectory`, `TrajectoryOptimizationStrategy` | 接口契约 |
