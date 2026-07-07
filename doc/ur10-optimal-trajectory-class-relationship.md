# UR10 最优轨迹生成 — 类关系文档

> 基于 `figaroh-examples/examples/ur10/` 路径下的例程分析。

---

## 总体架构

UR10 例程涉及两条独立的代码路径：

| 入口脚本 | 核心类 | 目的 |
|---------|--------|------|
| `optimal_trajectory.py` | `OptimalTrajectoryIPOPT` | 生成用于**动力学参数辨识**的最优激励轨迹 |
| `optimal_config.py` | `UR10OptimalCalibration` | 生成用于**运动学校准**的最优构型 |

两条路径共享底层工具类（加载机器人、配置解析），但业务逻辑独立。

---

## 路径一：最优轨迹生成（IPOPT 优化）

### 入口调用链

```
optimal_trajectory.py::main()
  │
  ├── load_robot()                         # figaroh.tools.robot
  ├── OptimalTrajectoryIPOPT(robot, config)
  ├── ur10_traj.initialize()
  ├── ur10_traj.solve(stack_reps=2)
  └── ur10_traj.plot_results()
```

### 完整类层次

```
                        ┌─────────────────────────────┐
                        │     backend.Backend (ABC)    │  figaroh/backend/base.py
                        │  - build_regressor()         │
                        └──────────────┬──────────────┘
                                       │
              ┌────────────────────────┼────────────────────────┐
              │ NumericaBackend        │ CasadiBackend          │
              └────────────────────────┴────────────────────────┘
                                       ▲
                                       │ 持有
                                       │
  ┌────────────────────────────────────┴──────────────────────────────────────┐
  │                                                                             │
  │  BaseOptimalTrajectory                   figaroh/optimal/base_optimal_trajectory.py
  │  ├── robot, model, active_joints
  │  ├── trajectory_config, identif_config
  │  ├── _backend: Backend
  │  ├── base_computer: BaseParameterComputer   ← figaroh/optimal/base_parameter.py
  │  ├── WP: WaypointsGeneration                ← figaroh/utils/cubic_spline.py
  │  │     └── (extends CubicSpline)
  │  ├── constraint_manager: TrajectoryConstraintManager  ← figaroh/optimal/contraints.py
  │  │
  │  ├── solve(stack_reps)
  │  │   ├── _generate_feasible_initial_guess()
  │  │   ├── _solve_segment()
  │  │   │     ├── create_ipopt_problem()  ← abstract, 子类实现
  │  │   │     └── problem.solve_with_waypoints(wps)
  │  │   └── _prepare_next_segment()
  │  │
  │  ├── objective_function()   # 目标: 基回归矩阵条件数
  │  └── _stack_base_regressors()
  │
  └──────────────────────────────────────────────────────────────────────────────┘
                    ▲
                    │ 继承
  ┌─────────────────┴──────────────────────────────────────────────────────────┐
  │                                                                             │
  │  OptimalTrajectoryIPOPT                  ur10/utils/ur10_tools.py:594       │
  │  ├── __init__() → super().__init__()                                        │
  │  └── create_ipopt_problem() → UR10TrajectoryIPOPTProblem(...)               │
  │                                                                             │
  └──────────────────────────────────────────────────────────────────────────────┘
                    │
                    │ 创建 (工厂方法)
                    ▼
  ┌──────────────────────────────────────────────────────────────────────────────┐
  │                                                                             │
  │  UR10TrajectoryIPOPTProblem               ur10/utils/ur10_tools.py:642      │
  │                                                                             │
  └──────────────────────────────────────────────────────────────────────────────┘
                    ▲
                    │ 继承
                    │
  ┌─────────────────┴──────────────────────────────────────────────────────────┐
  │                                                                             │
  │  BaseTrajectoryIPOPTProblem              base_optimal_trajectory.py:580     │
  │  ├── 持有 → opt_traj: BaseOptimalTrajectory                                │
  │  ├── get_variable_bounds()                                                  │
  │  ├── get_constraint_bounds()                                                │
  │  ├── get_initial_guess()                                                    │
  │  ├── objective()                                                            │
  │  ├── constraints()                                                          │
  │  ├── jacobian()                                                             │
  │  └── solve_with_waypoints()                                                 │
  │                                                                             │
  └──────────────────────────────────────────────────────────────────────────────┘
                    ▲
                    │ 继承
                    │
  ┌─────────────────┴──────────────────────────────────────────────────────────┐
  │                                                                             │
  │  BaseOptimizationProblem (ABC)            figaroh/tools/robotipopt.py:376   │
  │  ├── @abstractmethod:                                                       │
  │  │     get_variable_bounds(), get_constraint_bounds(), get_initial_guess() │
  │  │     objective(), constraints(), jacobian()                               │
  │  └── callback_data, iteration_data                                          │
  │                                                                             │
  └──────────────────────────────────────────────────────────────────────────────┘
```

### IPOPT 求解层

```
BaseTrajectoryIPOPTProblem.solve_with_waypoints(wps)
  │
  ├── 若 backend == "casadi":
  │     └── _solve_with_casadi_backend(wps)
  │           ├── _make_spline_callback()      # 将 scipy 样条包装为 CasADi Callback
  │           ├── _build_symbolic_objective()  # SX 符号目标: sum(A²) - 0.01*sum(V²)
  │           ├── _build_symbolic_constraints() # SX 符号约束: 位置+速度+力矩(RNEA)
  │           └── cs.nlpsol("ipopt").solve(x0)
  │
  └── 否则 (numerical backend):
        └── RobotIPOPTSolver(self, config).solve()
              └── cyipopt.Problem(n, m, ...).solve(x0)
```

### 辅助类关系

```
BaseOptimalTrajectory 持有:
  │
  ├── self.WP: WaypointsGeneration (extends CubicSpline)
  │     ├── gen_rand_pool()       # 生成路点候选池 (10 个/关节)
  │     ├── gen_rand_wp()         # 随机采样路点
  │     ├── get_full_config()     # 三次样条插值 → p,v,a 轨迹
  │     └── check_cfg_constraints() # 检验位置/速度/力矩约束
  │
  ├── self.base_computer: BaseParameterComputer
  │     └── compute_base_indices() → idx_e, idx_b  # 辨识所需基参数索引
  │
  ├── self.constraint_manager: TrajectoryConstraintManager
  │     ├── get_variable_bounds()
  │     ├── get_constraint_bounds()
  │     └── evaluate_constraints()
  │
  └── self._backend: Backend
        ├── build_regressor()     # 构建回归矩阵
        └── (CasADi 变体) _ensure_symbolic_model()
```

### 数据流总览

```
solve() 入口
  │
  │  wp_init = 随机/中点
  │
  ├─ _generate_feasible_initial_guess()
  │   │
  │   │  Strategy 1: uniform (静态 + 扰动) → 可行则返回
  │   │  Strategy 2: random search (最多 500 次) → 可行则返回
  │   │
  │   └→ 返回: wps, vel_wps, acc_wps, tps, t_i, p_i, v_i, a_i
  │
  ├─ create_ipopt_problem(n_joints, n_wps, Ns, tps, vel_wps, acc_wps,
  │                        wp_init, vel_wp_init, acc_wp_init, W_stack)
  │   └→ 创建 UR10TrajectoryIPOPTProblem
  │
  ├─ problem.solve_with_waypoints(wps)
  │   │
  │   │  x0 = wps[1:] (去掉第一个固定路点) → IPOPT 决策变量
  │   │  nlp.solve(x0) → 迭代优化
  │   │
  │   └→ x_opt → 重构为 wps_opt → 样条插值 → p_f, v_f, a_f
  │
  └→ results['T_F'], ['P_F'], ['V_F'], ['A_F'] 存储
```

---

## 路径二：最优构型生成（D-optimal 标定）

### 类层次

```
BaseCalibration (ABC)                          # figaroh/calibration/base_calibration.py
  ▲
  │ 继承
  │
UR10Calibration                               # ur10/utils/ur10_tools.py:70
  └── cost_function()   # UR10 特定的运动学校准代价函数

──────────── (独立继承链) ────────────

BaseOptimalCalibration (ABC)                  # figaroh/optimal/base_optimal_calibration.py
  ▲
  │ 继承
  │
UR10OptimalCalibration                       # ur10/utils/ur10_tools.py:209
  ├── load_candidate_configurations()
  └── _generate_random_configurations()
      └── 使用内部 SOCPOptimizer 求解

──────────── (另一独立链) ────────────

BaseIdentification (ABC)                      # figaroh/identification/base_identification.py
  ▲
  │ 继承
  │
UR10Identification                           # ur10/utils/ur10_tools.py:138
  └── load_trajectory_data()
```

### 入口调用链（optimal_config.py）

```
optimal_config.py::main()
  │
  ├── load_robot()
  ├── UR10OptimalCalibration(robot, config)
  │     └── super().__init__() → BaseOptimalCalibration(robot, config)
  └── opt_calib.solve(save_file=True)
        └── SOCPOptimizer 求解 D-optimal 构型选择
```

---

## 关系矩阵

| 类 | 文件 | 继承自 | 持有/依赖 |
|----|------|--------|----------|
| `OptimalTrajectoryIPOPT` | `ur10/utils/ur10_tools.py:594` | `BaseOptimalTrajectory` | — |
| `BaseOptimalTrajectory` | `optimal/base_optimal_trajectory.py:52` | — | `Backend`, `WaypointsGeneration`, `BaseParameterComputer`, `TrajectoryConstraintManager` |
| `UR10TrajectoryIPOPTProblem` | `ur10/utils/ur10_tools.py:642` | `BaseTrajectoryIPOPTProblem` | `OptimalTrajectoryIPOPT` |
| `BaseTrajectoryIPOPTProblem` | `optimal/base_optimal_trajectory.py:580` | `BaseOptimizationProblem` | `BaseOptimalTrajectory` |
| `BaseOptimizationProblem` | `tools/robotipopt.py:376` | `ABC` | — |
| `RobotIPOPTSolver` | `tools/robotipopt.py:621` | — | `BaseOptimizationProblem`, `cyipopt.Problem` |
| `WaypointsGeneration` | `utils/cubic_spline.py:496` | `CubicSpline` | — |
| `CubicSpline` | `utils/cubic_spline.py:106` | — | — |
| `UR10Calibration` | `ur10/utils/ur10_tools.py:70` | `BaseCalibration` | — |
| `UR10OptimalCalibration` | `ur10/utils/ur10_tools.py:209` | `BaseOptimalCalibration` | — |
| `UR10Identification` | `ur10/utils/ur10_tools.py:138` | `BaseIdentification` | — |
| `UR10OptimalTrajectory` | `ur10/utils/ur10_tools.py:312` | — (独立类) | — |

---

## 关键设计模式

| 模式 | 应用处 | 说明 |
|------|--------|------|
| **Template Method** | `BaseOptimalTrajectory.solve()` | 流程骨架固定，子类通过 `create_ipopt_problem()` 插入具体问题 |
| **Strategy** | `Backend` (numerical / casadi) | 运行时切换回归矩阵计算和优化求解方式 |
| **Factory Method** | `OptimalTrajectoryIPOPT.create_ipopt_problem()` | 创建 `UR10TrajectoryIPOPTProblem` 实例 |
| **ABC** | `BaseOptimizationProblem`, `BaseCalibration` | 定义接口契约，强制子类实现特定方法 |
