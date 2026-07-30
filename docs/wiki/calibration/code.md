# 标定模块代码页

> 模块：[src/figaroh/calibration/](../../../src/figaroh/calibration/) · 算法设计见 [algorithm](algorithm.md) · 已知问题见 [known-issues](known-issues.md)

## 模块定位

`calibration` 是 FIGAROH 的**几何（运动学）标定**模块：给定外部测量得到的末端位姿 $\bm{P}_{EE}^{\text{meas}}$ 与对应关节构型 $\bm{q}$，辨识一组几何参数偏移 $\bm{var}$，使更新后的正运动学 $\mathrm{FKM}(\bm{q},\bm{var})$ 重现测量。与 [identification](../identification/algorithm.md)（动力学参数，$\tau=W\pi$）平行，但回归子是 FKM 对几何参数的雅可比，而非 RNEA 对惯性参数的雅可比。核心抽象类 `BaseCalibration` 采用模板方法模式，机器人特定的代价函数由子类实现（见 [UR10Calibration](../../../figaroh-examples/examples/ur10/utils/ur10_tools.py)）。

## 文件清单

| 文件 | 职责 |
|------|------|
| [base_calibration.py](../../../src/figaroh/calibration/base_calibration.py) | `BaseCalibration` ABC：初始化/求解/评估/可视化全流程 |
| [calibration_tools.py](../../../src/figaroh/calibration/calibration_tools.py) | FKM 更新、回归子聚合、基参数约简、数据/坐标变换工具 |
| [config.py](../../../src/figaroh/calibration/config.py) | YAML 解析、统一→旧式配置转换、`get_sup_joints` |
| [data_loader.py](../../../src/figaroh/calibration/data_loader.py) | CSV 读取、位姿/关节数据装载 |
| [parameter.py](../../../src/figaroh/calibration/parameter.py) | 参数名模板与命名组装 |

## 公共 API

| 符号 | 定义处 | 作用 |
|------|--------|------|
| `BaseCalibration` (ABC) | base_calibration | 标定主类，模板方法 |
| `__init__(robot, config_file, del_list)` | base_calibration | 装载配置、设 `nvars`/`_data_path` |
| `initialize()` | base_calibration | `load_data_set()` + `create_param_list()` |
| `solve(method="lm", max_iterations, outlier_threshold, ...)` | base_calibration | 编排优化→评估→存储→（可选）绘图/保存 |
| `load_param(config_file, setting_type)` | base_calibration | 旧式/统一配置分发 |
| `create_param_list(q=None)` | base_calibration | 调 `calculate_base_kinematics_regressor` 选基参数 |
| `cost_function(var)` → residuals | base_calibration（钩子） | **子类必须重写**；默认实现仅告警 |
| `apply_measurement_weighting(residuals, pos_weight, orient_weight)` | base_calibration | 位置/姿态 $1/\sigma$ 加权 |
| `calc_stddev(result)` | base_calibration | $\sigma^2_\rho\,\mathrm{pinv}(J^\top J)$ 参数协方差 |
| `get_pose_from_measure(res_)` | base_calibration | 包装 `calc_updated_fkm` |
| `calc_updated_fkm(model, data, var, q, calib_config)` | calibration_tools | 更新关节放置→算 $\bm{P}_{EE}$（世界系） |
| `calculate_base_kinematics_regressor(q, model, data, calib_config, tol_qr)` | calibration_tools | 聚合 R→去零列→QR 选基→回填 `param_name` |
| `get_sup_joints(model, start_frame, end_frame)` | config | 支撑关节链（相对运动贡献者） |
| `get_param_from_yaml(robot, calib_data)` / `unified_to_legacy_config(robot, cfg)` | config | 配置→`calib_config` dict |
| `load_data(path, model, calib_config, del_list)` | data_loader | 返回扁平 `PEEm_exp`, `q_exp` |
| 模板常量 `FULL_PARAMTPL`/`JOINT_OFFSETTPL`/`BASE_TPL`/`EE_TPL`/`ELAS_TPL` | parameter | 6 维几何/偏移/基/末端/弹性命名模板 |

## 端到端数据流

```
YAML config ──load_param──▶ calib_config ──┐
                                           ▼
CSV data ──load_data──▶ PEE_measured, q_measured ──┐
                                                   ▼
        create_param_list: calculate_base_kinematics_regressor
            │  R = Σ_i computeFrameKinematicRegressor(q_i)   (NbSample 行)
            ├─ eliminate_non_dynaffect       (去零影响列)
            ├─ get_baseIndex / get_baseParams (列主元 QR 选基)
            └─ add_base_name / add_pee_name   (组装 param_name)
                              │
                              ▼  param_name = 基参数集 (nvars)
        solve_optimisation ─▶ _optimize_with_outlier_removal
            │  loop (max_iterations):
            │    least_squares(cost_function, var, method="lm", max_nfev=1000)
            │      └─ cost_function(var) → calc_updated_fkm → 残差 → 加权
            │    _detect_outliers (per-sample RMS, mean+k·std, k=3)
            ▼  var_ (标定参数), outlier_indices
        _evaluate_solution → calc_stddev  (C_param = σ²_ρ · pinv(JᵀJ))
            ▼
        export_urdf → URDFComparison.fk_consistency_check   (见 known-issues G)
```

## 依赖与关键配置键

依赖：`pinocchio`、`scipy.optimize.least_squares`、`pandas`、`numpy`、`yaml`；底层约简来自 [tools/qrdecomposition.py](../../../src/figaroh/tools/qrdecomposition.py) 与 [tools/regressor.py](../../../src/figaroh/tools/regressor.py)（`eliminate_non_dynaffect` 等）。`calib_config` 是贯穿全模块的 dict，关键键：`param_name`/`nvars`（辨识集）、`NbSample`/`NbMarkers`/`measurability`/`calibration_index`（样本与可测自由度）、`actJoint_idx`/`config_idx`（支撑关节）、`calib_model`（`full_params`/`joint_offset`）、`known_baseframe`/`known_tipframe`（需手填，见 gotcha）、`coeff_regularize`、`q0`、`data_file`。

## 最小调用示例

```python
from figaroh.calibration import BaseCalibration          # __init__ 仅导出 BaseCalibration
# 子类化并实现 cost_function（见 figaroh-examples/.../ur10_tools.py::UR10Calibration）
calib = UR10Calibration(robot, "config/ur10_unified_config.yaml", del_list=[])
calib.calib_config["known_baseframe"] = False            # 手填（gotcha C）
calib.calib_config["known_tipframe"]  = False
calib.initialize()                                        # load_data_set + create_param_list
result = calib.solve(plotting=False, enable_logging=True)  # LM + 离群点轮
print(calib.evaluation_metrics["rmse"])
print(calib.std_dev)                                      # 参数标准差
```

> 注：示例完整管线见 [ur10/calibration.py](../../../figaroh-examples/examples/ur10/calibration.py)，其 export/verify/viz 步骤依赖的 `urdf_exporter`/`export_validation` 当前不存在（见 [known-issues G](known-issues.md)）。

## 关键 Gotcha

- **`_compute_logmap_residuals` 全仓库无定义（阻断级桩）**：UR10/Tiago/Talos 三套示例的 `cost_function` 与标定后残差统计都调用 `self._compute_logmap_residuals(PEE_measured, PEEe)`，但 `src/` 内**无任何 `def`**。一旦走真实求解路径立即 `AttributeError`。设计意图算法见 [algorithm §5](algorithm.md#5-se3-log-map-残差设计意图当前为桩未实现)。
- **`save_results` 守卫失效**：守卫 `if not hasattr(self, 'result') or self.results_data is None` 检查的是 `self.result`，而 `_store_optimization_results` 只写 `self.results_data`，从不写 `self.result`。故 `hasattr(self,'result')` 恒为 `False`，守卫恒短路、提前 `return`，**`save_results` 实际从不落盘**。
- **`known_baseframe`/`known_tipframe` 需手填**：`create_param_list` 会读这两键决定是否调 `add_base_name`/`add_pee_name`，但 `get_param_from_yaml`/`unified_to_legacy_config` 均不写它们。缺键即 `KeyError`。三套示例都在 `__init__` 后手动 `calib_config["known_baseframe"]=False`。
- **离群点只检测不剔除**：`_optimize_with_outlier_removal` 把新离群点 `extend` 进 `outlier_indices`，但下一轮仅 `current_var = result.x`，**未从 `PEE_measured`/`q_measured` 删行**，全量重跑。累加列表仅作记录。
- **`q0` 共享引用**：[data_loader.py](../../../src/figaroh/calibration/data_loader.py) 中 `config = calib_config["q0"]`（无 `.copy()`）随后 `config[config_idx] = q_act[i,:]` 原地改写，导致 `calib_config["q0"]` 被污染成末样本构型。
- **代价函数残差语义不一致**：默认 `cost_function` 用逐元素差 `PEE_measured − PEEe`（6 维 xyz+rpy），子类示例却调用 log-map 残差桩。二者尺度/几何含义不同，混用会错估权与标准差。

→ 算法设计见 [algorithm](algorithm.md) → 已知问题见 [known-issues](known-issues.md)
