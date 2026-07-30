# 标定模块已知问题

> 模块：[src/figaroh/calibration/](../../../src/figaroh/calibration/) · 算法见 [algorithm](algorithm.md) · 代码见 [code](code.md)

本页列出 `calibration` 模块当前实现的缺陷与偏离，按阻断严重度排序。相关算法推导见 [algorithm](algorithm.md)，公共 API 与数据流见 [code](code.md)。

**影响范围**：A/B 直接阻断 `solve()` 与结果落盘（走 UR10 等真实路径即崩）；C 在 `initialize()` 报 `KeyError`，靠示例手动补键掩盖；D/E 使标定结果数值不可靠但不报错；F/J 在评估侧引入尺度误差；G 使示例的 export/verify/viz 子命令不可用；H/I 仅在特定配置（`non_geom=True`、非 Tiago 机型调 `add_eemarker_frame`）下触发。

**修复优先序**：建议按 A → B → C → E → D → G → F/H → I → J 推进。A 是其余 log-map 相关推导（[algorithm §5](algorithm.md#5-se3-log-map-残差设计意图当前为桩未实现)）落地的前提；B 与 A 共同决定「能否得到可用标定结果文件」；C/E 是配置与数据层的低改动量修复，应优先于 D 的策略重设计。

## 优先级总览

| ID | 标题 | 严重度 | 位置 |
|----|------|--------|------|
| A | `_compute_logmap_residuals` 无定义（阻断级桩） | 🔴 阻断 | [base_calibration](../../../src/figaroh/calibration/base_calibration.py) 调用方 |
| B | `save_results` 守卫失效，从不落盘 | 🔴 阻断 | [base_calibration `save_results`](../../../src/figaroh/calibration/base_calibration.py) |
| C | `known_baseframe`/`known_tipframe` 配置不写，须手填 | 🟠 高 | [config.py](../../../src/figaroh/calibration/config.py) |
| D | 离群点只检测不剔除 | 🟠 高 | `_optimize_with_outlier_removal` |
| E | `q0` 共享引用被原地污染 | 🟠 高 | [data_loader `load_data`](../../../src/figaroh/calibration/data_loader.py) |
| F | 代价函数残差语义不一致（差 vs log-map） | 🟡 中 | `cost_function` 默认 vs 子类 |
| G | URDF 导出/校验模块不存在 | 🟠 高 | 示例 [calibration.py](../../../figaroh-examples/examples/ur10/calibration.py) import |
| H | `update_forward_kinematics` 的 `non_geom` 分支引用越界 `key` | 🟡 中 | [calibration_tools](../../../src/figaroh/calibration/calibration_tools.py) |
| I | `add_eemarker_frame` 硬编码父关节 | 🟡 中 | [parameter.py](../../../src/figaroh/calibration/parameter.py) |
| J | RPY 欧拉角直接相减对大转角奇异 | 🟢 低 | 默认 `cost_function` |

## 逐条说明

**A. `_compute_logmap_residuals` 无定义（阻断级桩）** — `src/` 内无任何 `def _compute_logmap_residuals`，但 UR10/Tiago/Talos 三套示例的 `cost_function`（[ur10_tools.py:115](../../../figaroh-examples/examples/ur10/utils/ur10_tools.py)、tiago_tools、talos_tools）与标定后残差统计（各 `calibration.py`）均调用 `self._compute_logmap_residuals(...)`。走真实求解路径立即 `AttributeError`。设计意图算法见 [algorithm §5](algorithm.md#5-se3-log-map-残差设计意图当前为桩未实现)。

**B. `save_results` 守卫失效** — 守卫 `if not hasattr(self, 'result') or self.results_data is None:` 检查 `self.result`，而 `_store_optimization_results` 只写 `self.results_data`、从不写 `self.result`。`hasattr(self,'result')` 恒 `False` → 守卫恒短路、提前 `return`，`save_results` **从不落盘**。修法：改查 `self.results_data`。

**C. `known_baseframe`/`known_tipframe` 配置不写** — `create_param_list` 读这两键决定是否调 `add_base_name`/`add_pee_name`（[base_calibration:425](../../../src/figaroh/calibration/base_calibration.py)），但 `get_param_from_yaml`/`unified_to_legacy_config` 均不写它们。缺键即 `KeyError`，三套示例都靠 `__init__` 后手动 `calib_config["known_baseframe"]=False` 绕过。修法：在配置解析器补默认值。

**D. 离群点只检测不剔除** — `_optimize_with_outlier_removal` 把新离群点 `extend` 进 `outlier_indices`，下一轮仅 `current_var = result.x`，**未从 `PEE_measured`/`q_measured` 删行**，全量重跑。`outlier_indices` 仅作记录，不影响代价。伪代码与影响见 [algorithm §9](algorithm.md#9-迭代离群点检测)。

**E. `q0` 共享引用被原地污染** — [data_loader.py:161](../../../src/figaroh/calibration/data_loader.py) `config = calib_config["q0"]`（无 `.copy()`）随后 `config[config_idx] = q_act[i,:]` 原地改写，循环结束后 `calib_config["q0"]` 被污染成末样本构型。修法：`config = calib_config["q0"].copy()`。

**F. 代价函数残差语义不一致** — 默认 `cost_function` 用逐元素差 `PEE_measured - PEEe`（6 维 xyz+rpy，[base_calibration:552](../../../src/figaroh/calibration/base_calibration.py)），子类示例却调用 log-map 残差桩（A）。二者尺度/几何含义不同，混用会错估权（[algorithm §6](algorithm.md#6-单位感知加权)）与标准差（[algorithm §10](algorithm.md#10-参数标准差)）。

**G. URDF 导出/校验模块不存在** — `figaroh.tools.urdf_exporter`（`export_urdf`/`frame_settings_doc`）与 `figaroh.tools.export_validation`（`URDFComparison`）两模块在仓库内不存在，`figaroh.tools` 仅含 load_robot/qrdecomposition/regressor/robot*/solver。示例 [calibration.py:60-64](../../../figaroh-examples/examples/ur10/calibration.py) 的 import 会 `ImportError`，export/verify/viz 管线不可用。

**H. `update_forward_kinematics` 的 `non_geom` 分支引用越界 `key`** — [calibration_tools.py:414/432](../../../src/figaroh/calibration/calibration_tools.py) `if j_name in key:` 中的 `key` 是上一层 `for key in param_dict.keys()` 循环退出后的残留值（非当前关节），逻辑错误。`calc_updated_fkm` 版已修正此模式（显式 `if j_name in key` 在 `for key` 内），故默认走 `calc_updated_fkm` 不触发；`update_forward_kinematics` 的 `non_geom=True` 路径有隐患。

**I. `add_eemarker_frame` 硬编码父关节** — [parameter.py:227](../../../src/figaroh/calibration/parameter.py) 硬编码 `parent_jointId = model.getJointId("arm_7_joint")`，仅适用 Tiago；UR10/Talos 误用即错。注释自承认需配置化。

**J. RPY 欧拉角直接相减对大转角奇异** — 默认 `cost_function` 与评估都用 `PEE_measured - PEEe`（含 rpy 分量相减），大转角下欧拉角有奇异且不满足 $\mathrm{SE}(3)$ 群结构。设计意图改用 log-map（[algorithm §5](algorithm.md#5-se3-log-map-残差设计意图当前为桩未实现)），但该实现为桩（A）。

## 测试覆盖

`calibration` 模块**无直接单测**，`tests/unit/` 下无 `test_calibration*.py`，也无测试 import 该模块。间接覆盖仅限其依赖的底层 QR/回归子工具：[test_qr_decomposition.py](../../../tests/unit/test_qr_decomposition.py)（`get_baseParams`/`get_baseIndex`/`build_baseRegressor`）、[test_regressor.py](../../../tests/unit/test_regressor.py)（`eliminate_non_dynaffect`）。故 A–E 类阻断缺陷无法被现有 CI 捕获。

## 复现路径

- A/B/F/J：实例化任一示例子类（如 `UR10Calibration`）并 `solve()`，即触发 `AttributeError: ..._compute_logmap_residuals`（A）；`solve(..., save_results=True)` 不报错但无文件产出（B）。
- C：删去示例中 `calib_config["known_baseframe"]=False` 两行后 `initialize()` 抛 `KeyError`。
- D/E：对比多轮离群点检测前后 `outlier_indices` 内容不变 vs `PEE_measured` 行数不变（D）；`load_data` 后检查 `calib_config["q0"]` 是否被末样本构型覆盖（E）。
- G：直接 `python figaroh-examples/examples/ur10/calibration.py` 默认管线在 import 阶段即 `ImportError`（仅 `--calibrate-only` 路径可避开 export/verify/viz）。
- H/I：仅在 `calib_config["non_geom"]=True` 走 `update_forward_kinematics`（H），或非 Tiago 机型调 `add_eemarker_frame`（I）时触发。

## 交叉引用

- 算法推导（log-map、加权、正则、LM、标准差）：[algorithm](algorithm.md)
- 公共 API 与端到端数据流：[code](code.md)
- 底层约简工具：[基参数约简](../algorithms/base-parameter-reduction.md)、[RNEA 回归子](../algorithms/rnea-regressor.md)
- 兄弟模块：[identification/algorithm](../identification/algorithm.md)、[tools/algorithm](../tools/algorithm.md)、[utils/algorithm](../utils/algorithm.md)

