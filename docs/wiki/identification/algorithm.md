# Identification 模块（算法页）

> 模块：[src/figaroh/identification/](../../../src/figaroh/identification/)　实现见 [code](code.md)。

本页只推导 `identification` 模块**特有**的算法环节；与其它模块共享的数学（RNEA 回归子、基参数约简、物理一致性、D-最优）引用对应基础页，不重复。

---

## 1. 扩展动力学模型与参数向量组装顺序

### 1.1 共享部分（引用）

逆动力学对惯性参数线性，回归子 $H=\partial\mathrm{rnea}/\partial\pi$，每连杆 10 个 barycentric 参数，扩展模型追加摩擦/电机惯量/零位偏置列——完整推导见 [RNEA 与回归子](../algorithms/rnea-regressor.md) §2–§4。最终满足

$$
\tau_{\text{vec}}=W\,\pi_{\text{ext}},\qquad W\in\mathbb{R}^{N n_v\times n_{\text{param}}}
$$

### 1.2 模块特有：`has_*` 开关与列序对齐

`identif_config` 三个布尔开关控制附加列：

| 开关 | 附加参数类型 | 回归子追加列数 |
|------|------------|--------------|
| `has_friction` | 粘性 $f_v$ + Coulomb $f_s$ | $2 n_v$（$f_v$ 列值 $\dot q_j$，$f_s$ 列值 $\mathrm{sgn}(\dot q_j)$） |
| `has_actuator_inertia` | 电机转子惯量 $I_a$ | $n_v$（列值 $\ddot q_j$） |
| `has_joint_offset` | 零位偏置 $o$ | $n_v$（列值 $1$） |

回归子总列数 $n_{\text{param}}=(10+2\,\text{has\_friction}+\text{has\_actuator\_inertia}+\text{has\_joint\_offset})\,n_v$。

### 1.3 参数向量组装顺序（与回归子列对齐）

`get_standard_parameters` + `add_standard_additional_parameters` 按**与回归子列完全一致**的顺序组装 `standard_parameter` 字典（Python 3.7+ 保序）：

1. **惯量块**（外层按关节 `model.names[1:]`，内层 10 个 Pinocchio 参数）：对每个活动连杆 $j$ 追加 $[m, mx, my, mz, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]_j$，共 $10 n_v$ 项。
2. **附加块**（`add_standard_additional_parameters`：**外层按类型，内层按关节**）：依次追加
   $[\{f_{v,j}\}_j,\;\{f_{s,j}\}_j,\;\{I_{a,j}\}_j,\;\{o_j\}_j]$，每段 $n_v$ 项。

即 $\pi_{\text{ext}}=[\,\underbrace{\text{惯量 }10n_v}_{\text{关节主序}};\;\underbrace{f_v,f_s,I_a,o}_{\text{类型主序，各 }n_v}\,]$，与 [rnea-regressor](../algorithms/rnea-regressor.md) §4 的扩展向量一致。

> ⚠️ **列数对齐**：`add_standard_additional_parameters` **总是**写入全部 4 类附加参数名（未启用项值填 0），而回归子只按 `has_*` 开关追加列。仅当三开关**全开**（$2+1+1=4$ 类全有列）或**全关**（不追加附加块）时，`standard_parameter.values()` 长度才等于回归子列数；部分开关会致 `compute_reference_torque` 中 $\tau_{\text{ref}}=W\,\phi_{\text{ref}}$ 维度错配。unified 配置默认三开关全开，故默认对齐。

> ⚠️ `reorder_inertial_parameters`（[parameter.py](../../../src/figaroh/identification/parameter.py)）实现完整但**调用处被注释**，主路径保留 Pinocchio 顺序——回归子列序以此为准。

---

## 2. 信号处理

实测 $q(t)$ 含高频噪声；辨识需 $\dot q,\ddot q,\tau$。模块用"中值→零相位低通→数值微分→（可选）降采样"链。

### 2.1 零相位 Butterworth（medfilt + filtfilt）

**设计**：`signal.butter(nbutter, f_butter/(f_sample/2), "low")` 得 IIR 系数 $(b,a)$，归一化截止 $W_n=2f_{\text{butter}}/f_{\text{sample}}$（Nyquist $=f_{\text{sample}}/2$）。`signal.filtfilt(b,a,x,padtype="odd",padlen=3(\max\{|b|,|a|\}-1))` 前向+反向各滤一次。

**为何 filtfilt 零相位**：设前向滤波器传递函数 $H(z)$，幅频 $|H(e^{j\omega})|=M(\omega)$、相频 $\arg H=\varphi(\omega)$。反向滤波 = 把序列时间反转后前向滤波再反转，等价于施加 $H(z^{-1})$。在单位圆上 $H(e^{-j\omega})=H(e^{j\omega})^*=M(\omega)\,e^{-j\varphi(\omega)}$（实系数共轭对称）。故组合传递

$$
G(e^{j\omega})=H(e^{j\omega})\,H(e^{-j\omega})=M(\omega)^2\,e^{j\varphi-j\varphi}=M(\omega)^2
$$

相位恒为 0，幅频被**平方**（更陡，等效阶数 $2n_{\text{butter}}$）。零相位保证滤波后的 $q,\dot q,\ddot q,\tau$ 时间对齐、无群延迟——这对回归子逐采样点配对至关重要。

**为何先中值后低通**：`medfilt(x, med_fil)`（窗 `med_fil=5`）是非线性边缘保持去噪，擅长剔除**冲激/尖峰**（传感器毛刺，谱极宽）。若先低通，尖峰被线性滤波抹成宽带偏差污染信号带；先中值剔尖峰，再用 Butterworth 去高频高斯噪声，两者互补。反序（低通→中值）会让中值作用在已平滑数据上，去毛刺能力下降。

```
function filter_signal(x, nbutter, f_butter, f_sample, med_fil):
    b,a = butter(nbutter, f_butter/(f_sample/2), "low")
    for each column j:
        xj = medfilt(x[:,j], med_fil)                          # 去尖峰
        x[:,j] = filtfilt(b, a, xj, padtype="odd",
                          padlen=3*(max(len(b),len(a))-1))    # 零相低通
    return x
```

### 2.2 数值微分

`_differentiate_signal` 支持四法（`method` 参数）。默认 `gradient` 用 `np.gradient(sig, t)`：内部二阶中心差分，边界一阶单边。其余三法：

- **前向**：$\dot x_i=(x_{i+1}-x_i)/(t_{i+1}-t_i)$，末点 $\dot x_{N-1}:=\dot x_{N-2}$ 外推。
- **后向**：$\dot x_i=(x_i-x_{i-1})/(t_i-t_{i-1})$，首点 $\dot x_0:=\dot x_1$ 外推。
- **中心**：$\dot x_i=(x_{i+1}-x_{i-1})/(t_{i+1}-t_{i-1})$（$i=1..N-2$），边界单边。

> `identification_tools.calculate_first_second_order_differentiation` 是另一变体：一阶用 `pin.difference`（李群/广义坐标的对数映射差，非欧氏减法）算 $\dot q$，二阶对 $\dot q$ 再 `np.gradient(edge_order=1)`，末尾裁两样本。该函数为遗留工具，主路径 `filter_kinematics_data` 走 `_differentiate_signal`。

```
function differentiate(t, x, method):
    if method=="gradient":  return gradient(x, t)             # 二阶中心/边界一阶
    if method=="forward":   d[:-1]=diff(x)/diff(t); d[-1]=d[-2]
    if method=="backward":  d[1:]=diff(x)/diff(t);  d[0]=d[1]
    if method=="central":   d[1:-1]=(x[2:]-x[:-2])/(t[2:]-t[:-2]); 边界单边
```

### 2.3 降采样（decimate）

`signal.decimate(x, q, zero_phase=True)` 先内置抗混叠低通（Chebyshev/Butterworth）再抽稀 $q$ 倍。`zero_phase=True` 内部走 filtfilt，保证降采样后样本与原信号时间对齐。`_apply_decimation` 逐关节对 $\tau$ 降采样，`_decimate_regressor_matrix` 逐关节逐列降采样回归子，再按关节主序重排。降采样把 $m=N n_v$ 缩为 $m'=(N/q)n_v$，加速 QR 与 LS，代价是冗余度下降。

**复杂度**：滤波 $O(N\log N)$（FFT 卷积）每通道；微分 $O(N)$；decimate $O(N)$。整体线性于样本数。

---

## 3. 基参数约简流程

### 3.1 共享部分（引用）

列消除（去零范数列）→ 列主元 QR 选基 → 双 QR 求依赖 $\beta$ → 映射 $M=[I_r\,|\,\beta]$ 使 $\phi_{\text{base}}=M\,\theta_r$，且 $W\pi=W_b\phi_{\text{base}}$——完整推导见 [基参数约简](../algorithms/base-parameter-reduction.md)。

### 3.2 模块特有：`BaseIdentification` 编排

```
function reduce_to_base(W, tau, std_params, zero_tol, tol_qr):
    idx_elim, active = get_index_eliminate(W, std_params, tol_e=zero_tol)   # ‖W[:,j]‖²<tol_e 删
    W_e = build_regressor_reduced(W, idx_elim)                              # 去零列
    # (可选) decimate W_e, tau
    decomposer = QRDecomposer(tolerance=tol_qr)                             # tol_qr=1e-6
    W_b, base_dict, base_names, phi_base, phi_std = decomposer.double_decomposition(
        tau, W_e, active, std_params)
    M = decomposer.get_M()                                                  # (r, n_e)
    params_r = decomposer.get_M_labels()[1] or active.keys()                # θ_r 名
    # 内部: phi_base = round(solve(R1, Q1ᵀ tau), 6)
    return W_b, phi_base, M, params_r
```

`_eliminate_zero_columns` 用 `zero_tolerance=1e-3`（`solve` 默认 `zero_tolerance=0.001`，比约简页示例 `tol_e=1e-6` 宽松）。`_calculate_base_parameters` 调 `QRDecomposer.double_decomposition`（`tol_qr` 取 `self.tol_qr`，缺省 `1e-6`），并把 $M$、`params_r` 存入 `self._M_matrix`/`self._params_r_for_recon` 供 §6 重建复用。`solve_with_custom_solver` 走 `double_QR`（函数式入口）+ `LinearSolver.solve(W_b, tau)`：先求 $W_b$，再用自定义求解器解 $\phi_b$。

**复杂度**：QR 双分解 $O(m\,n_e^2)$，对样本数线性、参数数二次（详见基础页）。

---

## 4. 求解

### 4.1 默认 QR 路径

`double_decomposition` 内部由 $W_b=Q_1 R_1$ 直接得基参数（[基础页](../algorithms/base-parameter-reduction.md) §5）：

$$
\hat\phi_{\text{base}}=R_1^{-1}Q_1^\top\tau
$$

数值上比正则方程 $(W_b^\top W_b)^{-1}W_b^\top\tau$ 稳（免显式构 $W_b^\top W_b$）。预测力矩 $\hat\tau=W_b\hat\phi_{\text{base}}$。

### 4.2 备选 `LinearSolver`（10 法）

`solve_with_custom_solver` 用 [tools/algorithm](../tools/algorithm.md) 的 `LinearSolver`（[solver.py](../../../src/figaroh/tools/solver.py)），支持 10 法：`lstsq`、`qr`、`svd`、`ridge`（L2/Tikhonov）、`lasso`（L1）、`elastic_net`、`tikhonov`、`constrained`、`robust`、`weighted`。先经 `double_QR` 得 $W_b$，再 `solver.solve(W_b, tau)`。正则项由 `regularization`（`l1`/`l2`/`elastic_net`）+ `alpha` 控制；`bounds`/`constraints` 走 `constrained`。各法推导见 tools 算法页。

### 4.3 WLS：Gautier 1997 迭代加权

**动机**：不同关节力矩噪声方差不同（异方差）。OLS 等权对待所有行，估计非有效。Gautier 1997 按关节估 $\sigma$，以 $1/\sigma$ 加权。注：此为 `identification_tools.weigthed_least_squares` 提供的**独立工具**，不在 `BaseIdentification.solve` 默认流程内。

**记号**：$\tau$ 按关节主序分块 $\tau=[\tau^{(1)};\dots;\tau^{(n_v)}]$，关节 $j$ 块含 $m_j$ 行（代码 $m_j=$`idx_tau_stop` 增量，默认各块等长 `nb_samples`）。残差 $r^{(j)}=\tau^{(j)}-W_b^{(j)}\hat\phi_b$，$r=\dim\phi_b$。

**推导**：模型 $\tau=W_b\phi_b+\varepsilon$，$\varepsilon^{(j)}\sim\mathcal N(0,\sigma_j^2 I)$。关节 $j$ 的 $\sigma_j$ 由残差估（代码用一阶矩形式）：

$$
\hat\sigma_j=\frac{\big\|r^{(j)}\big\|_2}{\,m_j-r\,}
$$

（分母 $m_j-r$ 为自由度；经典 Gauss 估为 $\sqrt{\|r^{(j)}\|^2/(m_j-r)}$，代码取 $\|r^{(j)}\|/(m_j-r)$，仅影响各关节相对权的标量。）构对角权

$$
P=\mathrm{diag}\big(1/\hat\sigma_j\text{，按行广播到关节 }j\text{ 的样本}\big)
$$

加权 LS 闭式解：

$$
\hat\phi_b=\big(W_b^\top P^\top P\,W_b\big)^{-1}W_b^\top P^\top P\,\tau=\mathrm{pinv}(P W_b)\,P\tau
$$

（代码 `phi_b = pinv(P@W_b) @ (P@tau_meas)`，再 `np.around(.,6)`。）代码以"逐关节"循环实现：每把一个关节的权填入 $P$ 即重解一次 $\phi_b$（用更新后的 $\tau_{\text{est}}=W_b\phi_b$ 算下一关节残差）——这是逐关节增量重加权，非全局迭代至收敛；效果近似 Gautier 单轮加权。

```
function wls_gautier(W_b, tau_meas, idx_tau_stop, n_base):
    phi = phi_ols                                          # 初值
    P = zeros(m, m);  start = 0
    for j in 0..n_v-1:
        stop = idx_tau_stop[j]
        r_j = tau_meas[start:stop] - W_b[start:stop]@phi
        sigma_j = norm(r_j) / ((stop-start) - n_base)
        for k in [start, stop):  P[k,k] = 1/sigma_j
        phi = pinv(P@W_b) @ (P@tau_meas)                  # 重解
        start = stop
    return round(phi, 6)
```

> WLS 需 `identif_config["idx_tau_stop"]`（各关节块结束索引），同样**需手填**（见 [code](code.md) §gotcha）。

**复杂度**：每关节一次 `pinv(PW_b)` $\approx O(m\,r^2+r^3)$；$n_v$ 关节共 $O(n_v\,m\,r^2)$，比 OLS 贵 $n_v$ 倍。

---

## 5. 质量指标

### 5.1 相对标准差（Pressé & Gautier 1991）——完整推导

**模型**：$\tau=W_b\phi_b+\varepsilon$，$\varepsilon\sim\mathcal N(0,\sigma_\rho^2 I)$，$m=$ 行数，$r=\dim\phi_b$。

**残差方差估**：$\hat\tau=W_b\hat\phi_b$，残差 $e=\tau-\hat\tau$。$\sigma_\rho^2$ 的无偏估（自由度 $m-r$）：

$$
\hat\sigma_\rho^2=\frac{\|e\|_2^2}{m-r}=\frac{\|\tau-W_b\hat\phi_b\|_2^2}{m-r}
$$

**参数协方差**：OLS 估计 $\hat\phi_b=(W_b^\top W_b)^{-1}W_b^\top\tau$ 的协差

$$
\mathrm{Cov}(\hat\phi_b)=\sigma_\rho^2\,(W_b^\top W_b)^{-1}
$$

以 $\hat\sigma_\rho^2$ 代入：$C_x=\hat\sigma_\rho^2\,(W_b^\top W_b)^{-1}$。

**相对标准差**：第 $i$ 参数的标准差 $\mathrm{std}_i=\sqrt{C_x[i,i]}$，相对值（%）：

$$
\boxed{\;\mathrm{stdev}_i^{\text{rel}}=100\cdot\frac{\sqrt{C_x[i,i]}}{|\hat\phi_{b,i}|}=100\cdot\frac{\hat\sigma_\rho\sqrt{[(W_b^\top W_b)^{-1}]_{ii}}}{|\hat\phi_{b,i}|}\;}
$$

经验阈值 $\mathrm{stdev}_i^{\text{rel}}<10\%$ 视该参数可靠。代码 [`relative_stdev`](../../../src/figaroh/identification/identification_tools.py) 逐字实现，`round(…,2)`。

> 此为 **OLS** 协方差（用 $W_b^\top W_b$）。若走 WLS，协差应为 $\hat\sigma_\rho^2(W_b^\top P^\top P W_b)^{-1}$；当前 `relative_stdev` 不接 WLS 的 $P$，故 WLS 路径的 stdev 仍按等权算——已知简化。

```
function relative_stdev(W_b, phi_b, tau):
    e = tau - W_b@phi_b
    sig2 = (eᵀ e) / (rows(W_b) - len(phi_b))            # σ_ρ²
    Cx = sig2 * inv(W_bᵀ W_b)
    for i: stdev_rel[i] = round(100*sqrt(Cx[i,i])/abs(phi_b[i]), 2)
    return stdev_rel
```

### 5.2 RMSE

残差 RMS：$\mathrm{RMSE}=\sqrt{\tfrac1m\|e\|_2^2}=\sqrt{\tfrac1m\sum_i(\tau_i-\hat\tau_i)^2}$。存 `self.rms_error`。

### 5.3 相关系数

$\rho=\mathrm{corr}(\tau_{\text{noised}},\tau_{\text{identif}})$，`np.corrcoef` 取 $[0,1]$。接近 1 表示拟合好。退化（方差为 0 等）时置 1.0。

### 5.4 条件数

$\kappa(W_b)=\sigma_{\max}(W_b)/\sigma_{\min}(W_b)$，`np.linalg.cond`，存 `result["condition number"]`。$\kappa$ 大示 $W_b$ 病态、参数难分辨——应回 [D-最优](../algorithms/d-optimal-design.md) 重设计激励轨迹。

---

## 6. 物理一致性 + 重建 + CAD

### 6.1 共享部分（引用）

pseudo-inertia $P=\begin{bmatrix}\Sigma&h\\h^\top&m\end{bmatrix}\succeq0\iff$ 物理可行；最近可行 SDP 投影；base→full 零空间闭式重建（Option A）与 SDP 重建（Option B）；CAD 线性约束（质量界/一阶矩界/对称性）——完整推导见 [物理一致性](../algorithms/physical-consistency.md)。

### 6.2 模块特有：`_apply_*_if_enabled` 启用机制

两条后处理链挂在 `_store_results` 末尾，**默认关**：

```
function _store_results(...):
    result = {...}                                       # 基参数、RMSE、cond 等
    _apply_physical_consistency_if_enabled(results)      # 读 identif_config["physical_consistency"]
    _apply_reconstruction_if_enabled(results)            # 读 identif_config["reconstruction"]
    ResultsManager(...).save(...)
```

- **物理一致性**（`_apply_physical_consistency_if_enabled`）：读 `identif_config["physical_consistency"]` 子字典，`enabled` 默认 `False`。开启时：先把 `standard_parameter` 拆成 per-joint $p_{10}$；`skip_if_feasible=True` 且全连杆已可行则跳过；否则 `project_robot_p10_lmi` 逐连杆 SDP 投影（`mass_min`/`psd_eig_tol=-1e-10`/`solver="cvxopt"`），可选叠 CAD 约束；投影后参数写回 `result["physical consistency"]["projected_parameters"]`。
- **重建**（`_apply_reconstruction_if_enabled`）：读 `identif_config["reconstruction"]`，`enabled` 默认 `False`。开启时：取 §3.2 存的 $M$、$\phi_b$、`params_r`，构 `BaseResult`，调 `reconstruct_full_parameters(method="nullspace"|"sdp"|"auto", prior_source="dict"|"urdf"|"yaml", ...)`。`auto` 探测 picos：有则 SDP（含 per-joint $P\succeq0$、$m\ge m_{\min}$、CAD 约束），无则回退零空间闭式。

### 6.3 交替投影（`run_reconstruction`）

`reconstruction.alternate_physical_consistency=True` 且 `max_iters>1` 时：每轮 Option A 重建 → 对结果 SDP 投影 → 以投影后参数为新先验 $\theta_0$ 再重建，共 `max_iters-1` 轮，迭代逼近"既满足 $M\theta=\phi_b$ 又物理可行"的解，末轮做可行性复查。

**复杂度**：物理投影每连杆一个 4×4 LMI 的 SDP（变量 10），移动浮窗内点 $O(1)$ 量级，整机线性于连杆数；重建 Option A 闭式 $O(n_e r+r^3)$，Option B 为 $n_e$ 维 SDP 较贵（详见基础页）。

---

## 相关

- 算法基础：[RNEA 与回归子](../algorithms/rnea-regressor.md)｜[基参数约简](../algorithms/base-parameter-reduction.md)｜[物理一致性](../algorithms/physical-consistency.md)｜[D-最优设计](../algorithms/d-optimal-design.md)
- 兄弟模块：[tools/algorithm](../tools/algorithm.md)（回归子构造、QR、LinearSolver）｜[optimal/algorithm](../optimal/algorithm.md)（激励轨迹）
- 测试：[test_qr_decomposition](../../../tests/unit/test_qr_decomposition.py)
