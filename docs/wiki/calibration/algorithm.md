# 标定模块算法推导

> 模块：[src/figaroh/calibration/](../../../src/figaroh/calibration/) · 实现见 [code](code.md) · 已知问题见 [known-issues](known-issues.md)

本页给出几何标定的完整数学推导与伪代码。记号约定：$\bm{q}\in\mathbb{R}^{n_q}$ 关节构型；$\bm{var}\in\mathbb{R}^{n_{\text{var}}}$ 待辨识几何参数偏移；$\bm{P}_{EE}\in\mathbb{R}^{6}$ 末端位姿（xyz + rpy）。

## 1. 几何标定问题

给定 $N$ 个测量样本 $\{(\bm{q}_i,\,\bm{P}_{EE,i}^{\text{meas}})\}_{i=1}^{N}$，求几何参数偏移 $\bm{var}$，使更新后的正运动学（FKM）重现测量：

$$
\min_{\bm{var}}\ \sum_{i=1}^{N}\big\|\,\bm{P}_{EE,i}^{\text{meas}}-\mathrm{FKM}(\bm{q}_i,\bm{var})\,\big\|_{\bm{W}}^{2}
$$

其中 $\|\cdot\|_{\bm{W}}$ 为单位感知加权范数（§6），$\mathrm{FKM}$ 由 `calc_updated_fkm` 实现：用 $\bm{var}$ 改写 `model.jointPlacements`，再 `pin.framesForwardKinematics` 求末端位姿，最后**复原**模型（参见 [code](code.md) 数据流）。这是非线性最小二乘（FKM 对 $\bm{var}$ 非线性），用 Levenberg–Marquardt 求解（§7）。

> 与动力学辨识 $\tau=W\pi$ 的对照：那里 $\tau$ 对 $\pi$ **线性**（[RNEA 回归子](../algorithms/rnea-regressor.md)），故可直接 LS；这里 $\bm{P}_{EE}$ 对 $\bm{var}$ **非线性**，必须迭代。

## 2. 参数模型

每个关节 $j$ 的几何偏移由 6 个量刻画（`FULL_PARAMTPL`，类比 DH 偏移）：

$$
\bm{\delta}_j=\big(\underbrace{d_{px},d_{py},d_{pz}}_{\text{平移}},\ \underbrace{d_{\phi x},d_{\phi y},d_{\phi z}}_{\text{RPY 旋转}}\big)^\top\in\mathbb{R}^{6}
$$

`JOINT_OFFSETTPL` 模式则只保留与关节运动方向对应的那 1 个零位偏移（如 revolute $z$ 取 `offsetRZ_*`）。基坐标系变换用 `BASE_TPL`（6 维）描述世界→start frame 的 $\mathrm{SE}(3)$；末端标记用 `EE_TPL`（$\text{NbMarkers}\times\text{measurability}$）描述 end frame→marker。

`known_baseframe`/`known_tipframe`（需手填，见 [code](code.md)）控制是否把基/末端参数并入辨识集：为 `False` 时 `add_base_name`/`add_pee_name` 将对应名字追加进 `param_name`。`get_sup_joints(model, start, end)` 沿运动树求两 frame 间的支撑关节（剔除 `universe`），决定哪些关节的 $\bm{\delta}_j$ 进入参数向量。

## 3. 运动学回归子

类比动力学回归子 $H=\partial\mathrm{RNEA}/\partial\pi$（[RNEA 回归子](../algorithms/rnea-regressor.md)），**运动学回归子**是 FKM 对几何参数的雅可比。Pinocchio 提供：

$$
\bm{R}=\texttt{computeFrameKinematicRegressor}(\text{IDX\_TOOL}, \text{LOCAL})\in\mathbb{R}^{6\times 6(n_j-1)}
$$

刻画末端位姿对**每个关节 6 维放置扰动**的线性映射；帧雅可比 $\bm{J}=\texttt{computeFrameJacobian}\in\mathbb{R}^{6\times n_v}$ 刻画对关节角的线性映射（`joint_offset` 模式用 $\bm{J}$）。当 start frame 非 `universe` 时，`get_rel_kinreg`/`get_rel_jac` 通过两 frame 支撑支路的 $\mathrm{SE}(3)$ 伴随 $\mathrm{Ad}^{-1}$ 求相对回归子/雅可比（仅取支承关节列）。

## 4. 基运动学参数约简

`calculate_base_kinematics_regressor` 完成（对照 [基参数约简](../algorithms/base-parameter-reduction.md)）：

1. **聚合**：`calculate_identifiable_kinematics_model` 把 $N$ 个构型的 $\bm{R}_i$（按 `measurability` 选行、删零行）纵向堆叠成 $\bm{R}\in\mathbb{R}^{6N\times 6(n_j-1)}$。无输入 $\bm{q}$ 时用 `pin.randomConfiguration` 生成随机构型（仅用于结构分析）。
2. **去零列**：`eliminate_non_dynaffect(R, geo_params, tol_e=1e-6)` 删 $\|\bm{R}_{:,j}\|_2\approx0$ 的几何参数列 → $\bm{R}_e$。
3. **QR 选基**：`get_baseIndex(R_e, params_e, tol_qr)` 列主元 QR 求秩 $r$ 与基列索引；`get_baseParams` 返回基回归子 $\bm{R}_b$ 与基参数名。`build_baseRegressor(R_e, idx_base)` 用给定基索引从**输入构型**的 $\bm{R}_e$ 取列。
4. **回填**：基参数名 `append` 进 `calib_config["param_name"]`；`add_base_name`/`add_pee_name` 补基/末端名。

结果：辨识集从全 $6n_j$ 几何参数约简为 $r$ 个可单独观测的基参数（$n_{\text{var}}=r$）。

## 5. SE(3) log-map 残差（设计意图，当前为桩，未实现）

> ⚠️ `_compute_logmap_residuals` 在 `src/figaroh/calibration/` 内**无定义**，被三套示例 `cost_function` 调用（见 [code](code.md)）。以下为其设计意图算法，**当前为桩**。

逐元素位姿差 $\bm{P}^{\text{meas}}-\bm{P}^{\text{est}}$ 把平移与 RPY 欧拉角直接相减，在大转角下欧拉角奇异且不满足 $\mathrm{SE}(3)$ 群结构。几何正确的残差是**李群 log-map**：

$$
\bm{r}_i=\log\!\big(\mathrm{SE3}(\bm{P}_{EE,i}^{\text{meas}})^{-1}\cdot\mathrm{SE3}(\bm{P}_{EE,i}^{\text{est}})\big)\in\mathfrak{se}(3)\cong\mathbb{R}^{6}
$$

即「把测量位姿作为参考系，求估计位姿相对它的微小刚体运动」$\rightarrow$ 切空间向量 $[\bm{\rho};\bm{\omega}]$（$\bm{\rho}$ 平移、$\bm{\omega}$ 旋量）。优点：处理任意大转角无奇异；平移与旋转都在李代数上加性、单位一致；接近解时 $\bm{r}\to\bm{0}$ 严格成立。实现用 `pin.log(SE3_meas⁻¹ · SE3_est)`（`pinocchio.log` 为 $\mathrm{SE}(3)$ 群对数）。伪代码：

```
function _compute_logmap_residuals(PEE_measured, PEE_est):  # 设计意图，未实现
    r = []
    for i in 0..NbSample-1:
        M_meas = xyzrpy_to_SE3(PEE_measured[:, i])   # 6 维 → SE3
        M_est  = xyzrpy_to_SE3(PEE_est[:, i])
        dM     = M_meas.inverse() * M_est            # 相对刚体运动
        r_i    = pin.log(dM).vector                  # ∈ ℝ⁶ [ρ; ω]
        for dof, meas in enumerate(measurability):
            if meas: r.append(r_i[dof])
    return np.array(r)
```

## 6. 单位感知加权

位置（米）与姿态（弧度）量级相差 $10^3$，须加权。`apply_measurement_weighting` 按 $\text{NbMarkers}\times\text{measurability}\times\text{NbSample}$ 遍历残差，对前 3 维乘 `pos_weight`、后 3 维乘 `orient_weight`；缺省 $w_p=1/\sigma_p$、$w_o=1/\sigma_o$（`measurement_std.position=0.001`、`orientation=0.01`）。

**推导**：若测量噪声 $\bm{\epsilon}_i\sim\mathcal{N}(\bm{0},\bm{\Sigma})$ 且各分量独立、异方差（$\sigma_p\neq\sigma_o$），加权 LS 的最优权为 $\bm{W}=\bm{\Sigma}^{-1/2}=\mathrm{diag}(1/\sigma_k)$：使 $\|\bm{W}(\bm{y}-\hat{\bm{y}})\|^2=\sum_k(y_k-\hat y_k)^2/\sigma_k^2$ 恰为负对数似然，估计达 Cramér–Rao 下界（Gauss–Markov 最优）。UR10 示例显式传 `pos_weight=1.0, orient_weight=0.5`。

## 7. 正则化

UR10 `cost_function` 在加权残差后追加正则项：取 $\bm{var}$ 中**剔除前 6（基）与末 $\text{NbMarkers}\times\text{calibration\_index}$（末端）** 的中间参数 $\bm{var}_{\text{mid}}$，追加 $\sqrt{c}\,\bm{var}_{\text{mid}}$（$c=$`coeff_regularize`）。

**为何只正则非基非末端**：基坐标变换与末端标记是绝对量、有物理基准且可观测；中间各关节几何偏移在数值上易出现共线/病态，正则化稳定数值而不偏置有物理意义的基/末端解。追加形式 $\sqrt{c}\,\bm{var}_{\text{mid}}$ 使其在最小二乘雅可比中贡献 $\sqrt{c}\,\bm{I}$ 列，等价于 $\bm{J}^\top\bm{J}+c\,\bm{I}$ 的 Tikhonov 阻尼。

## 8. Levenberg–Marquardt

`scipy.optimize.least_squares(cost_function, var, method="lm", max_nfev=1000)`：高斯–牛顿步 + 阻尼。每步在当前 $\bm{var}_k$ 线性化残差 $\bm{r}(\bm{var})\approx\bm{r}_k+\bm{J}_k\Delta\bm{var}$，解

$$
(\bm{J}_k^\top\bm{J}_k+\lambda\,\mathrm{diag}(\bm{J}_k^\top\bm{J}_k))\,\Delta\bm{var}=-\bm{J}_k^\top\bm{r}_k
$$

$\lambda$ 自适应：下降快则减小、步不佳则增大。`method="lm"` 即 MINPACK LMDER。伪代码：

```
function solve_optimisation(var_init, method="lm", max_nfev=1000):
    var = var_init                                 # zeros (initialize_variables mode=0)
    for iter in 0..max_iterations-1:               # 外层离群点轮
        result = least_squares(self.cost_function, var, method="lm", max_nfev=1000)
        if not result.success: break
        PEE_est = get_pose_from_measure(result.x)  # calc_updated_fkm
        resid   = PEE_est - PEE_measured
        new_out = _detect_outliers(resid, k=3)      # §9
        if empty(new_out): break
        outlier_indices ∪= new_out
        var = result.x                             # ⚠ 未删数据行（见 code gotcha）
    return result, outlier_indices
```

## 9. 迭代离群点检测

每轮把残差按样本重排为 $(\text{calibration\_index}, \text{NbSample})$，逐样本算 RMS $\sqrt{\overline{r_i^2}}$；阈值 $\mu+k\sigma$（$\mu,\sigma$ 为 RMS 序列的均值/标准差，$k=3$），超阈样本索引累加进 `outlier_indices`。**注意当前实现只检测累加，不从 `PEE_measured`/`q_measured` 删行**（见 [code](code.md)）。

```
function _detect_outliers(residuals, k=3):
    R = residuals.reshape(calibration_index, NbSample)
    rms = sqrt(mean(R**2, axis=0))                 # per-sample
    thr = mean(rms) + k * std(rms)
    return [i for i where rms[i] > thr]
```

## 10. 参数标准差

残差方差与参数协方差由雅可比线性化传播：

$$
\hat\sigma^2_{\rho}=\frac{\text{result.cost}^{\,2}}{N_{\text{dof}}-n_{\text{var}}},\qquad N_{\text{dof}}=N_{\text{sample}}\cdot\text{calibration\_index}
$$

$$
\bm{C}_{\text{param}}=\hat\sigma^2_{\rho}\,\mathrm{pinv}(\bm{J}^\top\bm{J}),\qquad \text{std\_dev}_i=\sqrt{(\bm{C}_{\text{param}})_{ii}}
$$

其中 $\bm{J}=$`result.jac`（LM 在最优解处的雅可比），`pinv` 应对秩亏稳健。百分比 $\text{std\_pctg}_i=|\text{std\_dev}_i/\hat{\var}_i|$（$\hat\var_i=0$ 时记 0）。**推导**：高斯–牛顿最优解渐近 $\mathrm{Cov}(\hat{\bm\var})=\sigma^2(\bm{J}^\top\bm{J})^{-1}$；$\sigma^2$ 用残差自由度 $N_{\text{dof}}-n_{\text{var}}$ 估计。实现见 `calc_stddev`。

## 11. URDF 回写 + FK 一致性校验

标定后 `export_urdf(nominal_urdf, params_dict)` 把辨识到的关节放置偏移写回 URDF（基/末端 frame 参数不自动写入，由控制器侧配置）；`URDFComparison(nominal, modified).fk_consistency_check(n_samples=200)` 在 $n$ 个随机构型上比对两 URDF 的 FK，输出位置/姿态 RMSE 与最大误差。

> ⚠️ `figaroh.tools.urdf_exporter` / `figaroh.tools.export_validation` 两模块在仓库内**不存在**，示例 `calibration.py` 的 export/verify/viz 管线会 `ImportError`。属已知问题，见 [known-issues](known-issues.md)。

## 12. 复杂度

- 回归子聚合：$O(N\cdot n_v)$（每构型 `computeFrameKinematicRegressor` 为 $O(n_v)$）。
- 基参数约简：$O(N\,n^2)$（QR，$n=6n_j$ 参数列），详见 [基参数约简 §9](../algorithms/base-parameter-reduction.md)。
- LM 每次残差求值：$O(N\,n_v)$（FKM 是 $O(n_v)$），雅可比数值微分约 $n_{\text{var}}$ 次残差求值 → $O(N\,n_v\,n_{\text{var}})$。
- 标准差：$\bm{J}^\top\bm{J}$ 为 $O(N\,n_{\text{var}}^2)$，`pinv` 为 $O(n_{\text{var}}^3)$。

总体对样本数线性、对参数数二次，适合 $N\gg n_{\text{var}}$ 的标定场景。
