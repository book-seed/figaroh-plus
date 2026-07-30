# tools 模块（代码）

> 模块：[src/figaroh/tools/](../../../src/figaroh/tools/) · 实现见 [algorithm](algorithm.md)

## 1. 定位

`figaroh.tools` 是工程**核心机器人学基础设施层**：位于上层 `identification`/`optimal`/`calibration` 与底层 `pinocchio`/`cyipopt`/`scipy` 之间，提供数值回归子、QR 基参数约简、线性求解、IPOPT 抽象、碰撞检测、可视化、机器人加载与合成数据。不含 CasADi 符号后端（见 [backend](../backend/algorithm.md)）。

## 2. 文件清单

| 文件 | 职责 |
|------|------|
| [robot.py](../../../src/figaroh/tools/robot.py) | `Robot`（继承 `RobotWrapper`）：free-flyer 配置、可视化分发、`create_robot` 工厂 |
| [load_robot.py](../../../src/figaroh/tools/load_robot.py) | `load_robot` 多后端分发、`_get_models_directory` 四级回退 |
| [regressor.py](../../../src/figaroh/tools/regressor.py) | `RegressorBuilder`+`RegressorConfig` 数值回归子；遗留 `eliminate_*`/`build_*` |
| [qrdecomposition.py](../../../src/figaroh/tools/qrdecomposition.py) | `QRDecomposer`+`QRResult` QR 双分解、β、M；遗留 `QR_pivoting`/`double_QR` |
| [solver.py](../../../src/figaroh/tools/solver.py) | `LinearSolver`（10 法）+`solve_linear_system` |
| [robotipopt.py](../../../src/figaroh/tools/robotipopt.py) | `IPOPTConfig`+`BaseOptimizationProblem(ABC)`+`RobotIPOPTSolver`+`TrajectoryOptimizationProblem` |
| [robotcollisions.py](../../../src/figaroh/tools/robotcollisions.py) | `CollisionManager`+`CollisionWrapper` |
| [robotvisualization.py](../../../src/figaroh/tools/robotvisualization.py) | `RobotVisualizer`+`VisualizationConfig` |
| [randomdata.py](../../../src/figaroh/tools/randomdata.py) | `generate_waypoints(_fext)`/`get_torque_rand` |
| [__init__.py](../../../src/figaroh/tools/__init__.py) | 导入全部子模块（无 `__all__` 导出） |

## 3. 公共 API

| 类 / 函数 | 来源 | 要点 |
|-----------|------|------|
| `RegressorBuilder` / `RegressorConfig` | regressor.py | `build_basic_regressor` 数值回归子 W；`has_friction`/`has_actuator_inertia`/`has_joint_offset` 控制附加列 |
| `QRDecomposer` / `QRResult` | qrdecomposition.py | `decompose` 统一入口（method 归一化）；`double_decomposition`/`decompose_with_pivoting`；`expand_mapping_matrix_to_full` |
| `LinearSolver.METHODS` | solver.py | `lstsq/qr/svd/ridge/lasso/elastic_net/tikhonov/constrained/robust/weighted` |
| `IPOPTConfig` | robotipopt.py | `to_ipopt_options`（字节键）；`for_trajectory_optimization`/`for_parameter_identification` 预设 |
| `BaseOptimizationProblem(ABC)` | robotipopt.py | 5 抽象方法；`gradient`/`jacobian` 默认 numdifftools 自动微分；`hessian` 默认 `False` |
| `RobotIPOPTSolver` | robotipopt.py | `solve` 惰性 import cyipopt；`status∈{-1,0,1}` 视为成功 |
| `TrajectoryOptimizationProblem` | robotipopt.py | `set_objective/constraint_function` 注入回调 |
| `CollisionManager` / `CollisionWrapper` | robotcollisions.py | `check_collisions`/`get_all_distances`/`visualize_collisions` |
| `RobotVisualizer` / `VisualizationConfig` | robotvisualization.py | `display_com`/`axes`/`force`/`bounding_boxes`/`joints` |
| `load_robot` / `create_robot` | load_robot.py / robot.py | `loader∈{figaroh,robot_description,yourdfpy}` |
| `generate_waypoints(_fext)` / `get_torque_rand` | randomdata.py | 随机激励数据 + 合成力矩（含摩擦/电机/偏置/耦合腕） |

## 4. 数据流

### 辨识链

```
URDF ──load_robot──▶ Robot
                         │
   generate_waypoints/get_torque_rand ─▶ q,v,a,tau（或真实测量）
                         │
        build_regressor_basic (RegressorBuilder)
                         ▼
              W ∈ R^{N·nv × (10+add)·nv}
                         │
      eliminate_non_dynaffect / get_index_eliminate（删零列）
                         ▼
                       W_e
                         │
        double_QR (QRDecomposer.double_decomposition)
              ┌──────────┴───────────┐
              ▼                        ▼
        W_b, φ_b, M            base_param_expressions
              │
        LinearSolver.solve（可选精修）
              ▼
   physical_consistency / reconstruction（base → full θ）
```

> 辨识质量指标、LS 求解与全参数重建见 [identification](../identification/algorithm.md) 与 [物理一致性](../algorithms/physical-consistency.md)。

### 最优轨迹链

```
URDF ──load_robot──▶ Robot
                         │
   BaseOptimizationProblem 子类（objective/constraints/bounds）
                         │
       IPOPTConfig.for_trajectory_optimization()
                         ▼
        RobotIPOPTSolver.solve（cyipopt 惰性导入）
                         │
                         ▼
              x_opt（最优轨迹 / 位姿）
                         │
        RegressorBuilder ─▶ W_b ─▶ D-最优目标
```

> 注：Fourier D-最优 NLP 走 CasADi `cs.nlpsol("ipopt")` 符号路径（见 [optimal](../optimal/algorithm.md)），**非**本模块 `RobotIPOPTSolver`（cyipopt 路径）。后者用于一般机器人非线性规划。

## 5. 关键 gotcha

- **`_reorder_parameters` 死代码**：方法已定义（Pinocchio→barycentric 列重排 `[4,5,7,6,8,9,1,2,3,0]`），但 `_build_joint/external_wrench_regressor` 中调用被注释，实际返回未重排的 **Pinocchio 顺序** W。回归子列序必须与 $\pi_{\text{std}}$ 一致（见 [rnea-regressor](../algorithms/rnea-regressor.md)）。
- **`_fill_joint_regressor_sample` 的 None 陷阱**：形参 `identif_config=None`，但方法体直接 `identif_config["act_idxv"]`——若 `build_basic_regressor(q,v,a)` 不传 identif_config 会 `TypeError`；附加列仅在 `j in act_idxv` 时填充。
- **`_get_nonzero_inertias` 潜伏**：`__init__` 计算非零质量连杆索引存 `self.nonzero_inertias`，但 build 方法未消费（仍遍历全部 `nv`），当前为未用状态。
- **randomdata 首行垃圾数据**：`generate_waypoints` 用 `np.empty((1,...))` 预分配后 `vstack`，返回首行为**未初始化内存**——消费方必须丢弃 row 0。
- **`_solve_weighted` O(N²) 内存**：1D 权被 `np.diag(W)` 膨胀成 $N{\times}N$ 对角阵再 `W@A`；应改用 $\sqrt{w}\odot A$ 逐行缩放。robust IRLS 内 `np.diag(weights)` 同样问题。
- **cyipopt 惰性导入**：`import cyipopt` 在 `RobotIPOPTSolver.solve()` 内部，模块导入不要求装 cyipopt；缺失时抛 `ImportError`。
- **遗留 API 仍被主链消费**：辨识/最优主链实际调用的是 `build_regressor_basic`/`eliminate_non_dynaffect`/`get_index_eliminate`/`build_regressor_reduced`/`double_QR`/`QR_pivoting`/`get_baseParams`/`CollisionWrapper`/`display_*` 等遗留函数与类——增强类与遗留函数**双 API 并存**，新代码应优先用 `QRDecomposer.decompose`。

→ 算法设计见 [algorithm](algorithm.md)
