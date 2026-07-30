# utils 模块代码页

> 模块：[src/figaroh/utils/](../../../src/figaroh/utils/) · 算法推导见 [algorithm](algorithm.md) · 基础见 [RNEA 回归子](../algorithms/rnea-regressor.md)

---

## 1. 定位

**辅助层**。为辨识 / 标定 / 最优实验设计提供四类基础设施：

1. **轨迹参数化** —— 样条与 Fourier 级数，统一在 `BaseTrajectory` 接口下；
2. **配置解析** —— 统一 YAML 系统（模板继承 / 变体 / 任务继承 / 变量展开）+ 遗留格式兼容；
3. **结果管理** —— 四类任务的标准化绘图与多格式落盘；
4. **横切关注点** —— Pinocchio 薄封装、自定义异常与装饰器。

被 [identification](../identification/algorithm.md)、[optimal](../optimal/algorithm.md)（Fourier 用于 NLP）、[calibration](../calibration/algorithm.md) 复用。

---

## 2. 文件清单

| 文件 | 职责 | 核心导出 |
|------|------|---------|
| [base_trajectory.py](../../../src/figaroh/utils/base_trajectory.py) | 轨迹抽象基类 | `BaseTrajectory(ABC)` |
| [cubic_spline.py](../../../src/figaroh/utils/cubic_spline.py) | ndcurves 三次样条 + 航点生成 | `CubicSpline`、`WaypointsGeneration` |
| [fourier_trajectory.py](../../../src/figaroh/utils/fourier_trajectory.py) | Fourier 级数轨迹（解析导数 + CasADi q 表达式） | `FourierTrajectory` |
| [config_parser.py](../../../src/figaroh/utils/config_parser.py) | 统一配置解析 | `UnifiedConfigParser`、`create_task_config`、`is_unified_config`、`get_param_from_yaml` |
| [results_manager.py](../../../src/figaroh/utils/results_manager.py) | 标准化绘图/落盘 | `ResultsManager` + 3 便利函数 |
| [error_handling.py](../../../src/figaroh/utils/error_handling.py) | 异常层级 + 装饰器 + 上下文管理 | `FigarohExampleError` 及子类、4 装饰器、`ErrorContext` |
| [pin_interface.py](../../../src/figaroh/utils/pin_interface.py) | Pinocchio 薄封装 | `init_robot`、`calc_torque` |
| [__init__.py](../../../src/figaroh/utils/__init__.py) | 包导出（**不完整**，见 §6） | 见 §6 gotcha |

---

## 3. 公共 API

### 3.1 `BaseTrajectory(ABC)` —— 5 抽象方法

| 方法 | 签名 | 返回 |
|------|------|------|
| `get_trajectory(t, coeffs)` | 时间点 → 位置 | `(N, nq)` |
| `get_velocity(t, coeffs)` | `dq/dt` | `(N, nv)` |
| `get_acceleration(t, coeffs)` | `d²q/dt²` | `(N, nv)` |
| `compute_torques(q, v, a, robot)` | 由 RNEA 算力矩 | `(N, nv)` |
| `check_constraints(q, v, tau, robot)` | 关节限值校验 | `bool`（True=违约） |

### 3.2 `CubicSpline` + `WaypointsGeneration`

- `CubicSpline(robot, num_waypoints, active_joints, soft_lim_pool)`：构造活跃关节索引集（`act_idxq`/`act_idxv`），从 `rmodel` 抽取位置/速度/力矩限值；`soft_lim_pool`（shape `(3, n_active)`）按行收缩 pos/vel/effort 限值。
- `get_active_config(freq, time_points, waypoints, vel_waypoints?, acc_waypoints?)`：逐段构造 `ndcurves.exact_cubic`（带 `curve_constraints`），组装进 `piecewise`，在等距采样点求 `pc(t)` / `pc.derivate(t,1|2)` 得 `q,dq,ddq`。
- `get_full_config(...)`：在 `get_active_config` 基础上用 `robot.q0`/`v0` 填充非活跃关节，输出全构型。
- `check_cfg_constraints(q, v?, tau?)`：逐样本逐活跃关节比对收缩后的上下限。
- `WaypointsGeneration(CubicSpline)`：维护 `n_set=15` 的 pos/vel/acc 航点池，`gen_rand_pool()` 按限值均匀填池，`gen_rand_wp()` 随机抽样并拒绝相邻重复，`gen_equal_wp()` 生成等距航点。

### 3.3 `FourierTrajectory`

- `FourierTrajectory(n_harmonics, n_act, omega?, T=2π)`：`omega` 缺省取 `2π/T`；`n_coeffs_per_joint = 1 + 2·n_harmonics`。
- `_evaluate(t, coeffs) -> (q, v, a)`：**解析内联**计算位置/速度/加速度（见 [algorithm](algorithm.md) §2）。
- `build_casadi_expression(t_sym, coeffs_sym, omega?)`：构造 CasADi **SX 位置表达式** `q(t)`（标量时间），仅测试使用。
- `compute_torques` / `check_constraints`：直接循环 `pinocchio.rnea`，限值取自 `robot.model`。

### 3.4 统一配置 `UnifiedConfigParser` + 函数

- `UnifiedConfigParser(config_path, variant?)`：`parse()` 主流程为 加载 → `extends` 继承 → `variant` → 任务继承 → 变量展开 → 校验 → 注入 `_metadata`。
- `create_task_config(robot, unified_config, task_name)`：从统一配置抽取单任务配置（合并 robot 属性 + environment）。
- `is_unified_config(config_file)`：探测指示键（`_metadata`/`schema_version`/`extends`/`variants`/`tasks`/`common`/`variables` 或任一值含 `inherits_from`）。
- `get_param_from_yaml(robot, config_data, task_type='auto', variant?)`：统一入口，自动分流 文件路径 / 统一 dict / 遗留 dict。

### 3.5 `ResultsManager`

四类绘图（由 `task_type` 选择 `PLOT_STYLES`）：`plot_calibration_results`、`plot_identification_results`、`plot_optimal_calibration_results`、`plot_optimal_trajectory_results`。`save_results` 支持 `yaml/csv/json/npz/pkl`，经 `_convert_for_serialization` 将 `ndarray`→list、`np.integer/np.floating`→float、`np.bool_`→bool。

### 3.6 `error_handling`

异常层级：`FigarohExampleError`（基）→ `RobotInitializationError` / `ConfigurationError` / `DataProcessingError` / `CalibrationError` / `IdentificationError` / `ValidationError`。
4 装饰器：`validate_robot_initialization`、`validate_input_data`、`handle_calibration_errors`、`handle_identification_errors`。
上下文管理 `ErrorContext(operation_name, raise_on_error=True)`：进入时 log info，异常时记录并按标志重抛/吞掉。

### 3.7 `pin_interface`

- `init_robot(robot)`：`pin.framesForwardKinematics(model, data, q0)` + `pin.updateFramePlacements(model, data)`，初始化各 frame 位姿。
- `calc_torque(N, robot, q, v, a)`：循环 `pin.rnea`，**列主序**填充 `tau[j·N+i]`。

---

## 4. 数据流

**配置流**：

```
YAML ──extends──▶ _resolve_template_path(4 形式) ──▶ _deep_merge(父, 子)
                                                       │
        variant ──▶ _apply_variant(点号 extends)        │
                       │                                │
tasks.inherits_from ──▶ _resolve_task_inheritance(拓扑排序, 环检测)
                       │                                │
                       └──────────┬─────────────────────┘
                                  ▼
                  _expand_variables(${VAR}: env > config嵌套 > cache > 原值)
                                  ▼
                  _validate_task_type  ──▶  create_task_config  ──▶  任务 dict
```

**轨迹求值流**：

```
coeffs / waypoints + t  ──▶  BaseTrajectory.get_{trajectory,velocity,acceleration}
                                      │
                ┌─────────────────────┴─────────────────────┐
                ▼                                           ▼
          CubicSpline                               FourierTrajectory
   (ndcurves exact_cubic +                   (解析 sin/cos 级数：
    curve_constraints, piecewise 组装)        q=a0+Σ a_k sin+b_k cos;
   pc(t) / pc.derivate(t,1|2)                v,a 解析导数内联)
                │                                           │
                └─────────────────────┬─────────────────────┘
                                      ▼
                          q, v, a  (N, nq/nv)
                                      ▼
                  pin_interface.calc_torque (RNEA, 列主序 tau[j·N+i])
                                      ▼
                        tau  ──▶  回归子 / ResultsManager
```

---

## 5. 关键 gotcha

| # | 现象 | 位置 / 后果 |
|---|------|------------|
| 1 | `__init__.py` 导出**不完整**：仅导出 `ResultsManager` + 6 个 error_handling 符号 | 未导出 `CubicSpline`/`WaypointsGeneration`/`FourierTrajectory`/`UnifiedConfigParser`/`create_task_config`/`is_unified_config`/`get_param_from_yaml`/`ConfigurationError`/`ValidationError`/`RobotInitializationError`/`DataProcessingError`/`pin_interface`，须从子模块直接 import |
| 2 | `build_casadi_expression` **仅测试用** | 仅在 [test_fourier_trajectory.py](../../../tests/unit/test_fourier_trajectory.py) 调用；[Fourier NLP 策略](../optimal/algorithm.md) 自行向量化构造 q/v/a，**不调用**此方法；且它只构造位置 q（无 v/a） |
| 3 | `CubicSpline.soft_lim_pool` **文档漂移** | docstring 描述标量 `soft_lim`(0–1) 默认 0、示例 `soft_lim=0.1`，但真实参数是 `soft_lim_pool`（`(3, n_active)` 数组）；且 `assert np.array(soft_lim_pool).shape==(3,n)` **无条件执行**，传默认 `None` 必抛 `AssertionError` |
| 4 | `ResultsManager.COLORS` **键名拼写错误** | 字典键为 `'meured'`（缺 `as`），但 `_plot_pose_comparison` 读 `self.COLORS['measured']` → 运行时 `KeyError` |
| 5 | `calc_torque` **列主序** | 返回 1D `tau[j·N+i]`（关节主序），与回归子行序 `base_idx=j·N+i` 对齐；消费方需 `reshape(nv, N).T → (N, nv)`。`FourierTrajectory.compute_torques` 却直接返回 `(N, nv)`，两者不一致 |
| 6 | `check_self_collision` **桩** | 恒返回 `False`，未实现碰撞检测 |
| 7 | 弃用函数 | `get_calibration_param_from_yaml` / `get_identification_param_from_yaml` 发 `DeprecationWarning`，改用 `get_param_from_yaml(task_type=...)` |

---
→ 算法设计见 [algorithm](algorithm.md)
