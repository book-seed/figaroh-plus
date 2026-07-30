# 物理一致性与全参数重建

> 跨模块算法基础。被 [identification](../identification/algorithm.md) 引用。实现：[identification/physical_consistency.py](../../../src/figaroh/identification/physical_consistency.py)、[reconstruction.py](../../../src/figaroh/identification/reconstruction.py)。

---

## 1. 为什么需要物理一致性

LS 辨识得到的 $\hat\pi$ 可能**物理不可行**：负质量、非正定惯性张量。这样的参数无法回写到 URDF 仿真，且数值上不可信。需把辨识结果**投影**到物理可行集。这是可选项（`physical_consistency.enabled`，默认关闭），不改变辨识默认流程。

---

## 2. Pseudo-inertia 矩阵与物理可行条件

每连杆 10 参数 $p_{10}=[m, m c_x, m c_y, m c_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^\top$（Pinocchio 顺序）。组装 **4×4 pseudo-inertia 矩阵**：

$$
P = \begin{bmatrix} \Sigma & h \\ h^\top & m \end{bmatrix},\qquad
\Sigma=\begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix},\quad
h=m\boldsymbol{c}
$$

**定理（物理一致性）**：一组惯量参数物理可行 ⟺ $P\succeq 0$（半正定）且 $m>0$。

> **Schur 补视角**：$P\succeq0 \iff m\ge0$ 且 $\Sigma - h h^\top/m \succeq0$（当 $m>0$）。后者即"绕质心的惯性张量减去平移惯量后正定"——刚体惯性的固有性质。

代码：`pseudo_inertia_matrix_from_p10(p10)`（优先 `pin.PseudoInertia.FromDynamicParameters(p10).toMatrix()`，回退显式分块）；`check_p10_feasibility` 用 `eigvalsh(P).min() >= psd_eig_tol`（`-1e-10`，容许数值零空间）。

---

## 3. 最近可行投影（SDP/LMI）

给定辨识（或先验）参数 $\hat p_{10}$，求最近可行点：

$$
\min_{p}\;\big\|W_p\,(p-\hat p_{10})\big\|_2^2 \quad\text{s.t.}\quad P(p)\succeq0,\;\; m\ge m_{\min}
$$

- $W_p$：对角权（平衡 $m$、$h$、$\Sigma$ 三组尺度，`weights.mode="auto"` 按各组范数归一，或 `"manual"`）。
- 约束 $P\succeq0$ 是**线性矩阵不等式（LMI）**（因 $P$ 对 $p$ 线性）。
- 故为**半正定规划（SDP）**，picos 建模、cvxopt 求解。

### picos 建模

变量：$m\in\mathbb{R}$、$h\in\mathbb{R}^3$、$\Sigma\in\mathbb{S}^3$（对称）。约束：

$$
\begin{bmatrix}\Sigma & h\\ h^\top & m\end{bmatrix}\succeq0,\qquad m\ge m_{\min}
$$

目标：$\min \sum_k w_k^2(p_k-\hat p_k)^2$（$m$、$h$ 用 `pc.SquaredNorm`；$\Sigma$ 按 6 个上三角元素逐一平方累加，**避免 Frobenius 双计**——picos 版本敏感）。

求解后用放宽容差 $\min(\textit{psd\_eig\_tol},-10^{-8})$ 复查可行性，置 `status` 为 `projected`/`infeasible`/`error`。

### 整机器人

`project_robot_p10_lmi(p10_by_link)` 逐连杆独立投影（连杆间无耦合约束），可选叠加 [CAD 约束](#5-cad-约束)（质量/CoM 界、对称性）。`skip_if_feasible=True` 时若全连杆已可行则跳过。

---

## 4. 全参数重建（base → full）

辨识只得到**基参数** $\phi_{\text{base}}\in\mathbb{R}^r$（$r<n$）。完整参数 $\theta_r$（去零列后，$n_e$ 维）满足

$$
\phi_{\text{base}} = M\,\theta_r,\qquad M\in\mathbb{R}^{r\times n_e},\;\mathrm{rank}(M)=r<n
$$

（$M$ 来自 [基参数约简](base-parameter-reduction.md)。）这是**欠定**方程组，解不唯一。需选一个"好"的解。

### Option A：零空间投影（最小修正）

给定先验 $\theta_0$（来自 URDF 标称值 / YAML / CAD），求在 $M\theta=\phi_{\text{base}}$ 约束下**最接近先验**（加权）的解：

$$
\min_\theta\;\big\|W^{1/2}(\theta-\theta_0)\big\|^2 \quad\text{s.t.}\quad M\theta=\phi_{\text{base}}
$$

**推导（Lagrange 乘子）**：设 $W=\mathrm{diag}(w)$（正）。Lagrangian $L=\tfrac12(\theta-\theta_0)^\top W(\theta-\theta_0)+\lambda^\top(M\theta-\phi_{\text{base}})$。

一阶条件：$\nabla_\theta L = W(\theta-\theta_0) + M^\top\lambda = 0 \Rightarrow \theta=\theta_0 - W^{-1}M^\top\lambda$。代入约束 $M\theta=\phi_{\text{base}}$：

$$
M(\theta_0-W^{-1}M^\top\lambda)=\phi_{\text{base}} \;\Longrightarrow\; \lambda=(M W^{-1}M^\top)^{-1}(M\theta_0-\phi_{\text{base}})
$$

故闭式解：

$$
\boxed{\;\theta = \theta_0 - W^{-1}M^\top\,(M W^{-1}M^\top)^{-1}(M\theta_0-\phi_{\text{base}})\;}
$$

（$MW^{-1}M^\top$ 非奇异因 $M$ 满行秩、$W$ 正定；奇异时回退 `lstsq`。）代码：`reconstruct_theta_r`，`rcond=1e-12`。

### Option B：SDP 重建（加物理一致性）

在 Option A 基础上追加 $P(\theta)\succeq0$、$m\ge m_{\min}$，及 CAD 约束。目标用 **Schur 补 epigraph** 化为 SDP：

$$
\min t\quad\text{s.t.}\quad M\theta=\phi_{\text{base}},\;\;\begin{bmatrix}t & (\mathrm{diag}(w)(\theta-\theta_0))^\top\\ \mathrm{diag}(w)(\theta-\theta_0) & I\end{bmatrix}\succeq0,\;\;P_j(\theta)\succeq0\;\forall j
$$

（Schur 补保证 $t\ge\|W^{1/2}(\theta-\theta_0)\|^2$。）picos 求解。`method="auto"` 探测 picos 可用性：有则用 SDP，无则回退 Option A。

### 交替投影（run_reconstruction）

若 `alternate_physical_consistency=true` 且 `max_iters>1`：每轮做一次 Option A 重建 → 对结果做物理投影（§3）→ 以投影后参数为新 $\theta_0$ 再重建，共 `max_iters-1` 轮，迭代逼近"既满足 $M\theta=\phi_{\text{base}}$ 又物理可行"的解。

---

## 5. CAD 约束

来自 CAD/URDF 的先验，均为线性/凸约束，可与 LMI 叠加（`cad_constraints.py`）：

| 约束 | 形式 | 说明 |
|------|------|------|
| 质量界 | $m_{\min}\le m\le m_{\max}$ | `add_mass_bounds`；`bounds_from_urdf` 默认 $[m/100, 100m]$ |
| 一阶矩界 | $h_{k,\min}\le h_k\le h_{k,\max}$ | `add_com_bounds`（约束 $h=m\boldsymbol{c}$，保持**线性**，非 CoM 位置） |
| 对称性 | $\theta_{k,a}=\theta_{k,b}$ | `add_symmetry_constraints`（如左右臂对称连杆） |

`apply_cad_constraints_to_problem` 把这些追加到 picos problem（等式/不等式），返回追加约束数。

---

## 6. 伪代码

### 物理投影

```
function project_p10_lmi(p_hat, mass_min, psd_eig_tol, weights, solver):
    picos: variables m(1), h(3), Sigma(3x3 Symmetric)
    P = block([[Sigma, h],[h.T, m]])
    constraints: m >= mass_min ; P >> 0
    optional: mass_bounds, com_bounds (from CAD)
    objective: minimize ||W(p - p_hat)||^2     # m,h 用 SquaredNorm; Sigma 按6上三角元素
    solve(solver="cvxopt", max_seconds=...)
    p_new = assemble_p10(m, h, Sigma)
    if min_eigval(P(p_new)) >= min(psd_eig_tol, -1e-8): status="projected"
    else: status="infeasible"
    return p_new, report
```

### 全参数重建（Option A 闭式）

```
function reconstruct_theta_r(M, phi_base, theta0, weights):
    W_inv = 1/weights                          # 对角
    A  = M @ diag(W_inv) @ M.T                  # (r,r) 正定
    rhs = M @ theta0 - phi_base
    lam = solve(A, rhs)                        # 奇异则 lstsq
    theta = theta0 - W_inv * (M.T @ lam)       # 闭式解
    residual = M @ theta - phi_base            # 应 ≈ 0
    return theta, residual
```

---

## 7. 复杂度

- 物理投影：每连杆一个 SDP，变量数 10、LMI 维度 4 → 移动浮窗内点法 $O(1)$ 量级（极小），机器人规模线性于连杆数。
- Option A 重建：$O(n_e r + r^3)$（矩阵乘 + $r\times r$ 求逆），闭式。
- Option B 重建：SDP 规模随 $n_e$ 与连杆数增长，较 Option A 贵；故默认 Option A，`auto` 在 picos 缺失时回退。
