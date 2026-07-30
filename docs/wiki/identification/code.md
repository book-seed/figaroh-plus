# Identification 模块（代码页）

> 模块：[src/figaroh/identification/](../../../src/figaroh/identification/)　算法推导见 [algorithm](algorithm.md)。

## 模块定位

`identification` 是 FIGAROH 动力学参数辨识的**编排层**：把 YAML 配置、实测轨迹数据、Pinocchio 机器人模型与 [tools](../tools/algorithm.md) 构建的回归子粘合成一条端到端流水线——信号滤波→数值微分→回归子→零列消除→（可选）降采样→QR 双分解求基参数→质量指标→（可选）物理一致性投影 / 全参数重建。它不自己算 RNEA 回归子（交给 `figaroh.tools.regressor`）也不自己解 SDP（交给 `physical_consistency.py`/`reconstruction.py` 内的 picos 建模），只负责流程编排与结果封装。`BaseIdentification` 是抽象基类，机器人特定子类须实现 `load_trajectory_data`。

## 文件清单

| 文件 | 行 | 职责 |
|------|-----|------|
| [base_identification.py](../../../src/figaroh/identification/base_identification.py) | 1380 | `BaseIdentification` 抽象基类，端到端 solve 流水线 |
| [parameter.py](../../../src/figaroh/identification/parameter.py) | 332 | 标准惯量参数提取、附加参数（摩擦/惯量/偏置）组装 |
| [config.py](../../../src/figaroh/identification/config.py) | 376 | YAML 配置解析（legacy + unified 双格式互转） |
| [identification_tools.py](../../../src/figaroh/identification/identification_tools.py) | 294 | WLS、relative stdev、微分、Butterworth 等工具函数 |
| [physical_consistency.py](../../../src/figaroh/identification/physical_consistency.py) | 411 | pseudo-inertia SDP 投影（默认关，picos） |
| [reconstruction.py](../../../src/figaroh/identification/reconstruction.py) | 789 | base→full 重建（零空间闭式 / SDP / 交替投影） |
| [cad_constraints.py](../../../src/figaroh/identification/cad_constraints.py) | 337 | CAD 先验凸约束（质量界/一阶矩界/对称性） |
| [__init__.py](../../../src/figaroh/identification/__init__.py) | 79 | 公共 API 汇出 |

## 公共 API

### BaseIdentification（主入口）

| 方法 | 签名 | 返回 | 说明 |
|------|------|------|------|
| `__init__` | `(robot, config_file="config/robot_config.yaml")` | — | 载入模型与 YAML 配置 |
| `initialize` | `(truncate=None)` | — | process_data→regressor→std params→tau_ref |
| `solve` | `(decimate=True, decimation_factor=10, zero_tolerance=1e-3, plotting=True, save_results=False)` | `phi_base` | 默认 QR 路径，decimate 默认**开** |
| `solve_with_custom_solver` | `(method='lstsq', regularization=None, alpha=0.0, constraints=None, bounds=None, decimate=False, ...)` | `phi_base` | LinearSolver 备选路径，decimate 默认**关** |
| `load_param` | `(config_file, setting_type='identification')` | — | 自动探测 legacy/unified 格式 |
| `process_data` | `(truncate=None)` | — | 滤波+微分+全配置填充 |
| `calculate_full_regressor` | `()` | — | 委托 `tools.regressor.build_regressor_basic` |
| `initialize_standard_parameters` | `()` | — | 惯量+附加+自定义参数 |
| `compute_reference_torque` | `()` | — | 由标称参数算参考力矩 |
| `load_trajectory_data` | `()`（abstract） | `(t,q,dq,τ)` | 子类实现数据加载 |

### 模块级函数（节选）

| 函数 | 所在文件 | 说明 |
|------|---------|------|
| `relative_stdev(W_b, phi_b, tau)` | identification_tools | Pressé & Gautier 1991 相对标准差 (%) |
| `weigthed_least_squares(robot, phi_b, W_b, τm, τe, cfg)` | identification_tools | Gautier 1997 迭代加权 LS（按 `idx_tau_stop` 分关节） |
| `base_param_from_standard`, `index_in_base_params` | identification_tools | 标准↔基参数映射 |
| `reconstruct_full_parameters(base_result, method='nullspace'\|'sdp'\|'auto', ...)` | reconstruction | base→full 重建统一入口 |
| `reconstruct_theta_r(M, phi_base, *, theta0, weights, rcond)` | reconstruction | 零空间闭式重建核心 |
| `project_robot_p10_lmi(p10_by_link, ...)` | physical_consistency | 整机器人 SDP 投影 |
| `check_p10_feasibility(p10, ...)` | physical_consistency | pseudo-inertia $P\succeq0$ 可行性 |
| `build_cad_constraints_from_config(cfg, model)` | cad_constraints | 由 YAML/URDF 构 CAD 约束 |
| `get_standard_parameters(model, cfg)` | parameter | 10 惯量参数提取 |
| `add_standard_additional_parameters(model, cfg)` | parameter | 摩擦/惯量/偏置附加参数 |

## 端到端数据流

```
YAML config ──► load_param ──► identif_config
                                      │
CSV/实机数据 ──► load_trajectory_data (子类实现)
                                      │
                                      ▼
            process_data: medfilt → filtfilt(零相) → np.gradient 微分
                                      │
                                      ▼
            _build_full_configuration (act_idx 散布到全关节)
                                      │
              calculate_full_regressor ──► tools.build_regressor_basic
                                      │                  │
                          compute_reference_torque       W (N·nv, n_param)
                                      │
                          _eliminate_zero_columns (tol_e=1e-3)
                                      │
                            [solve: decimate=True] ──► _apply_decimation
                                      │
                  _calculate_base_parameters ◄── tools.QRDecomposer.double_decomposition
                                      │        (tol_qr=1e-6, 存 _M_matrix / _params_r_for_recon)
                                      ▼
                       phi_base = R1⁻¹ Q1ᵀ τ ,  W_b , M
                                      │
              _compute_quality_metrics: relative_stdev · RMSE · ρ · cond
                                      │
              _store_results ──┬──► _apply_physical_consistency_if_enabled  (默认关)
                                └──► _apply_reconstruction_if_enabled        (默认关)
                                      │
                              ResultsManager → yaml/csv/npz
```

## 关键 gotcha

1. **`reorder_inertial_parameters` 是死代码**　函数实现完整且经 `__init__` 导出，但在 `get_standard_parameters` 中**调用处被注释**（`reordered_params = pinocchio_params`），主路径保留 Pinocchio 顺序 `[m, mx, my, mz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`。回归矩阵列序必须与此一致——勿误以为发生了重排。

2. **摩擦 sign ≠ spec tanh**　辨识侧回归子（`tools.regressor`，本模块经 `build_regressor_basic` 调用）对 Coulomb 摩擦用 $\mathrm{sgn}(\dot q)$；而 [unified-friction-model spec](../../../openspec/specs/unified-friction-model/spec.md) 要求 $\tanh(\alpha\dot q)$（Fourier 优化侧用之）。已知 spec 偏离，辨识结果不可直接喂回优化侧的 tanh 模型。

3. **`act_idxq`/`act_idxv` 需手填**　配置解析器（legacy 与 unified）**均不**填充这两个键，但 `compute_reference_torque`/`_build_full_configuration`/`_apply_decimation` 强制读取。它们是活动关节在 `model.nq/nv` 中的索引，必须由机器人特定子类或用户手动注入 `identif_config`，否则 `KeyError`。

4. **物理一致性 / 重建默认关**　`_apply_*_if_enabled` 读 `identif_config["physical_consistency"]`/`["reconstruction"]` 子字典，`enabled` 默认 `False`。整条物理约束链（SDP 投影、零空间/SDP 重建、CAD 约束）是**可选增强**，不影响默认辨识流程；需显式开启且依赖 `picos`+`cvxopt`。

5. **`solve` 与 `solve_with_custom_solver` 的 decimate 默认相反**　`solve(decimate=True)` 默认降采样；`solve_with_custom_solver(decimate=False)` 默认不降采样。混用两条路径或迁移参数时易因默认值不同导致样本规模不一致、指标不可比。

→ 算法设计见 [algorithm](algorithm.md)
