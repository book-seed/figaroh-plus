# optimal 模块（实现）

> 模块：[src/figaroh/optimal/](../../../src/figaroh/optimal/) · 推导见 [algorithm](algorithm.md)

[D-最优实验设计](../algorithms/d-optimal-design.md) 的工程实现层：生成**激励轨迹**（动力学辨识用）与选取**标定位姿**（几何标定用）。前者由策略模式分发到 Fourier（符号 NLP）或样条（数值 cyipopt）两条路径，后者由 SOCP / DetMax 求解。

## 文件清单

| 文件 | 职责 |
|------|------|
| [base_optimal_trajectory.py](../../../src/figaroh/optimal/base_optimal_trajectory.py) | 激励轨迹框架；持有 strategy、base_computer、constraint_manager；结果存取（pickle/yaml） |
| [base_optimal_calibration.py](../../../src/figaroh/optimal/base_optimal_calibration.py) | 标定位姿选取框架 + `SOCPOptimizer` + `Detmax` |
| [base_parameter.py](../../../src/figaroh/optimal/base_parameter.py) | `BaseParameterComputer`：随机样条 → QR → `idx_b`/`idx_e` |
| [config.py](../../../src/figaroh/optimal/config.py) | YAML / unified 配置解析；模块级 `load_param` |
| [contraints.py](../../../src/figaroh/optimal/contraints.py) | `TrajectoryConstraintManager`（样条路径约束与界） |
| [strategies/](../../../src/figaroh/optimal/strategies/) | 策略模式：[base](../../../src/figaroh/optimal/strategies/base_strategy.py) · [fourier](../../../src/figaroh/optimal/strategies/fourier_strategy.py) · [spline](../../../src/figaroh/optimal/strategies/spline_strategy.py) |

## 公共 API

| 符号 | 签名 / 说明 |
|------|------------|
| `BaseOptimalTrajectory` | `(robot, config_file="config/robot_config.yaml")` — **无 `backend=` 参数**；`__init__` 内按 `trajectory_type` 调 `create_strategy` 选策略；`initialize()` 算 `idx_b`/`idx_e`；`solve(stack_reps=2)` 委托 `strategy.solve(self)` |
| `create_strategy`（工厂） | `create_strategy(trajectory_type, **kwargs)` → `SplineOptimizationStrategy()` 或 `FourierOptimizationStrategy(fourier_config=...)`；未知类型 `ValueError` |
| `TrajectoryOptimizationStrategy` | 策略 ABC（[base_strategy](../../../src/figaroh/optimal/strategies/base_strategy.py)）；抽象 `solve(context)` / `name()` |
| `SplineOptimizationStrategy` | 三次样条 + cyipopt；`objective=cond(W_b)`；多段堆叠 |
| `FourierOptimizationStrategy` | Fourier 系数 + CasADi `nlpsol("ipopt")`；`__init__(fourier_config=None)`；`solve` 内自建 `CasadiBackend` |
| `BaseParameterComputer` | `(robot, identif_config, soft_lim_pool)`；`compute_base_indices()→(idx_e, idx_b)` |
| `TrajectoryConstraintManager` | `(robot, CB, trajectory_config, identif_config)`；变量界与约束求值（样条路径） |
| `BaseOptimalCalibration` | `(robot, config_file=...)`（ABC）；候选位姿池 → SOCP 选子集；`solve(save_file=False)` |
| `SOCPOptimizer` | `(subX_dict, calib_config)`；picos + cvxopt 解 D-最优 SOCP |
| `Detmax` | `(candidate_pool, NbChosen)`；贪心 add/remove 交换 |
| `BaseTrajectoryIPOPTProblem` | 样条 IPOPT 问题基类（`objective`/`constraints`/`jacobian`） |

## 数据流

### Fourier 路径（D-最优 NLP，符号）

```
config(fourier) → BaseOptimalTrajectory.__init__
  → create_strategy("fourier", fourier_config) = FourierOptimizationStrategy
initialize(): BaseParameterComputer.compute_base_indices() → idx_e, idx_b
solve() → strategy.solve(self):
  CasadiBackend(robot)              # 策略内部自建，唯一消费者
  Z = MX.sym("coeffs", n_vars)      # 决策变量 = Fourier 系数
  reshape(Z, n_act, 1+2N_h)         # 列主序 → a0 / ak / bk
  Q,V,A = 解析 sin/cos 导数          # v=∂q/∂t, a=∂²q/∂t² (闭式, 非 cs.gradient)
  scatter → Q_full/V_full/A_full    # 全关节 (nq,nv × Ns)
  W_fun.map(Ns,"openmp") → W_full   # (Ns·nv, n_param)  [SX 内层, MX 穿透]
  W_b = W_full[:, idx_b]
  J = W_bᵀW_b/Ns + λI  →  logdet_fn(SX 包装 chol)  →  obj = -logdet
  τ = rnea.map(Ns) + fv·v + fs·tanh(α·v)           # 力矩约束
  cs.nlpsol("ipopt","mumps") → solve(x0, lbg, ubg)
  → FourierTrajectory 提取 q,v,a → results
```

### 样条路径（条件数 NLP，数值）

```
config(spline) → create_strategy("spline") = SplineOptimizationStrategy
initialize(): idx_e, idx_b (同上, 与 trajectory_type 无关)
solve() → strategy.solve(context):
  WaypointsGeneration.gen_rand_pool()
  for s_rep in range(stack_reps):            # 多段堆叠 (仅样条)
    _generate_feasible_initial_guess()        # 随机路点直至可行
    context.create_ipopt_problem(...)         # 子类实现 (BaseTrajectoryIPOPTProblem)
    problem.solve_with_waypoints(wps):
      objective = cond(W_b)                    # _stack_base_regressors
      constraints = TrajectoryConstraintManager.evaluate_constraints (数值)
      jacobian = 前向有限差分
      cyipopt(IPOPT) → results
    _prepare_next_segment(): 末路点 → 下一段 wp_init; W_stack 行堆叠
```

## 关键 gotcha

- **backend 已 sink 进 fourier 策略**：`CasadiBackend` 由 `FourierOptimizationStrategy.solve` 内部 `CasadiBackend(robot=context.robot)` 构造，`BaseOptimalTrajectory` 无 `backend=` 参数；样条策略不持有 backend。
- **`load_param` 的 `@staticmethod` 残留**：[config.py](../../../src/figaroh/optimal/config.py) 的 `load_param` 是**模块级函数**却挂 `@staticmethod`（其上是空类 `ConfigurationManager`）。装饰器在模块级无效果，可安全移除。
- **`BaseOptimalTrajectory` 非 ABC 但用 `@abstractmethod`**：类声明 `class BaseOptimalTrajectory:`（无 `ABC`/`ABCMeta`），却给 `create_ipopt_problem` 标 `@abstractmethod`——无 metaclass 时该装饰器**不阻止实例化**，子类未实现也只在该方法 `raise NotImplementedError`。标定侧 `BaseOptimalCalibration(ABC)` 才是真 ABC。
- **`omega` 默认 1.0 ≠ spec**：`fourier_frequency=None` 时 `omega=1.0`、周期 `T=2π`；spec 期望 $\omega=2\pi/T_{\text{traj}}$（由期望轨迹周期决定）。当前默认与真实周期解耦。
- **`build_symbolic_constraints` 未接线**：`TrajectoryConstraintManager.build_symbolic_constraints`（SX 约束表达式）存在但 Fourier 策略**未调用**——Fourier 在 MX 图内自建 `cons_list`；样条路径走数值 `evaluate_constraints`。同样 `FourierTrajectory.build_casadi_expression` 也未被策略调用（策略内联手写 MX 表达式）。
- **`stack_reps` 仅样条**：`BaseOptimalTrajectory.solve(stack_reps=2)` 签名有该参数但**未透传**给 `strategy.solve`；样条策略改从 `trajectory_config["stack_reps"]` 读取，Fourier 不分段（单条轨迹）。
- **CasADi reshape 列主序**：`cs.reshape(Z, n_act, n_coeffs)` 按列主序填充，而 `_initialize_coefficients` 与结果提取 `x_opt.reshape(...)` 用 numpy 行主序——同一扁平向量索引语义不一致，存储轨迹的系数排列与 NLP 内部不同。
- **`tanh_alpha_opt=10`**：优化用 $\alpha_{\text{opt}}=10$（更平滑，IPOPT 友好），`tanh_alpha_id=100` 保留给辨识但本模块未用；spec 期望 $\alpha\approx100$。

→ 算法设计见 [algorithm](algorithm.md)
