# 基参数约简（QR 双分解）

> 跨模块算法基础。被 [identification](../identification/algorithm.md)、[tools](../tools/algorithm.md)、[calibration](../calibration/algorithm.md)、[optimal](../optimal/algorithm.md) 引用。实现：[tools/qrdecomposition.py](../../../src/figaroh/tools/qrdecomposition.py)。

---

## 1. 为什么需要约简

由 [RNEA 与回归子](rnea-regressor.md) 得 $\tau = W\,\pi$，$W\in\mathbb{R}^{m\times n}$（$m=N n_v$ 样本行，$n=10n_v+\text{附加}$ 参数列）。问题是：**$W$ 通常列秩亏**——某些惯性参数（如基座连杆在固定基座下不可观测的项）永远以线性组合形式出现，无法单独辨识。直接对 $\pi$ 做最小二乘会得到病态/非唯一解。

**基参数（base parameters）** $\phi_{\text{base}}\in\mathbb{R}^{r}$ 是 $W$ 列空间的最小独立生成集（$r=\mathrm{rank}(W)<n$），使

$$
W\,\pi = W_b\,\phi_{\text{base}}
$$

且 $W_b\in\mathbb{R}^{m\times r}$ 列满秩。约简分三步：列消除 → 列主元 QR 选基 → 双 QR 求依赖。

---

## 2. 第一步：列消除（去零影响列）

某些列对力矩**完全无贡献**（$\|W_{:,j}\|_2 \approx 0$）。判据：

$$
\|W_{:,j}\|_2^2 = (W^\top W)_{jj} < \varepsilon_e
$$

删除这些列，得 $W_e\in\mathbb{R}^{m\times n_e}$ 与剩余参数名 $\theta_r$（$n_e\le n$）。代码：`eliminate_non_dynaffect` / `get_index_eliminate`（`tol_e=1e-6`）。

> 这一步只删**零范数列**，不处理线性依赖（依赖在第三步处理）。

---

## 3. 第二步：列主元 QR 选基列

对 $W_e$ 做带列主元的 QR：

$$
W_e\,P = Q\,R,\qquad Q\in\mathbb{R}^{m\times m},\; R\in\mathbb{R}^{m\times n_e},\; P\text{ 为置换}
$$

数值秩：

$$
r = \#\{\,|R_{jj}| > \varepsilon_{qr}\,\}
$$

（`tol_qr=1e-6`，相对容差可选 `max(rel_tol·|R_{00}|, tol)`）。主元置换 $P$ 把**最独立的列**排到前 $r$ 位：

$$
\text{base 列索引} = \mathrm{sort}(P_{:,0:r}),\qquad \text{依赖列索引} = \mathrm{sort}(P_{:,r:})
$$

`sort` 保证确定性（`_identify_base_parameters`）。

---

## 4. 第三步：双 QR 求依赖系数 β

### 4.1 分块

把 $W_e$ 的列按基/依赖重排为 $[W_b \;|\; W_d]$（$W_b\in\mathbb{R}^{m\times r}$，$W_d\in\mathbb{R}^{m\times(n_e-r)}$）。对合并矩阵做**第二次** QR：

$$
[\,W_b \;\big|\; W_d\,] = Q_r\,R_r,\qquad R_r = \begin{bmatrix} R_1 & R_2 \\ 0 & R_3 \end{bmatrix}
$$

其中 $R_1\in\mathbb{R}^{r\times r}$ 上三角（非奇异，因 $W_b$ 列满秩），$R_2\in\mathbb{R}^{r\times(n_e-r)}$。

### 4.2 推导依赖系数 β

由 $W_b = Q_{r,1}\,R_1$ 与 $W_d = Q_{r,1}\,R_2$（$R_3\approx0$ 因 $W_d$ 在 $W_b$ 的列空间内）：

$$
W_d = W_b\,R_1^{-1}R_2 \;\triangleq\; W_b\,\beta,\qquad \beta = R_1^{-1}R_2 \in\mathbb{R}^{r\times(n_e-r)}
$$

即**每个依赖列是基列的线性组合**，组合系数即 $\beta$。代码：`beta = solve(R1, R2)`。

### 4.3 构造映射矩阵 M

将参数向量对应分块 $\theta_r = [\theta_b;\;\theta_d]$（基参数 + 依赖参数，均在 $W_e$ 列序下）。则：

$$
W_e\,\theta_r = W_b\,\theta_b + W_d\,\theta_d = W_b\,(\theta_b + \beta\,\theta_d) = W_b\,\phi_{\text{base}}
$$

定义**基参数**

$$
\phi_{\text{base}} = \theta_b + \beta\,\theta_d
$$

即 $\phi_{\text{base}} = M\,\theta_r$，其中

$$
M = \big[\,I_r \;\big|\; \beta\,\big]\in\mathbb{R}^{r\times n_e}
$$

（在「基在前、依赖在后」的排序下；再按主元置换 $P$ 映射回 $W_e$ 原列序 → `_build_M_from_pivoting`；double 路径用 `_build_M_from_partition`）。

**基参数表达式**：第 $i$ 个基参数

$$
\phi_{\text{base},i} = \theta_{b,i} + \sum_{j} \beta_{ij}\,\theta_{d,j}
$$

代码以字符串如 `"Ixx_j1 + 0.5*mx_j2"` 记录（`_build_parameter_expressions`，$|\beta|>\varepsilon_\beta$ 才显示）。

---

## 5. 辨识求解

$W_b$ 列满秩，超定 LS 有唯一解：

$$
\hat\phi_{\text{base}} = (W_b^\top W_b)^{-1}W_b^\top\tau \;=\; R_1^{-1}Q_1^\top\tau
$$

（后一式来自 $W_b=Q_1R_1$ 的 QR 解，数值更稳。）代码：`phi_b = round(solve(R1, Q1ᵀ @ tau), 6)`。

预测力矩 $\hat\tau = W_b\,\hat\phi_{\text{base}}$。质量指标见 [identification/algorithm](../identification/algorithm.md) §相对标准差。

---

## 6. 扩展到全参数序（M 的展开）

$M$ 是在 $W_e$（去零列后）列序下定义的。若要映射回**完整标准参数序** $\pi$（含被删的零列，对应位置填 0）：

$$
M_{\text{full}}\in\mathbb{R}^{r\times n},\qquad M_{\text{full}}[:,\,\text{keep}]=M,\quad M_{\text{full}}[:,\,\text{deleted}]=0
$$

代码：`QRDecomposer.expand_mapping_matrix_to_full(M, params_r, full_param_names)`。这用于 [全参数重建](physical-consistency.md)（由 $\phi_{\text{base}}$ 反解 $\theta_r$）。

---

## 7. 两种路径对比

| | `decompose_with_pivoting` | `double_decomposition` |
|---|---|---|
| QR 次数 | 1（带主元） | 2（主元选基 + 重组合并） |
| $\beta$ | $R_1^{-1}R_2$（来自带主元 R） | $R_1^{-1}R_2$（来自第二次 QR） |
| $M$ 构造 | `_build_M_from_pivoting`（按 $P$ 反映射） | `_build_M_from_partition`（基/依赖分块） |
| 数值性 | 直接，但对病态 $W_e$ 主元可能不稳 | 二次 QR 更稳，工程默认 |

辨识默认 `double_decomposition`（`tol_qr=1e-6`）。

---

## 8. 伪代码（double_decomposition）

```
# 输入: tau ∈ R^m, W_e ∈ R^{m×n_e}, params_r (n_e 个参数名), tol_qr, tol_beta
# 输出: W_b, phi_base, M, base_param_expr

function double_decomposition(tau, W_e, params_r, tol_qr, tol_beta):
    # 1) 选基列
    Q,R,P = qr(W_e, pivoting=true)         # W_e @ P = Q @ R
    rank  = count(|diag(R)| > tol_qr)
    base_idx    = sort(P[:rank]); dep_idx = sort(P[rank:])
    # 2) 重排为 [W_b | W_d]
    W_b = W_e[:, base_idx];  W_d = W_e[:, dep_idx]
    # 3) 第二次 QR
    Q_r,R_r = qr([W_b | W_d])
    R1 = R_r[:rank,:rank];   R2 = R_r[:rank, rank:]
    beta = solve(R1, R2)                    # (rank, n_e-rank) 依赖系数
    # 4) 基参数解
    Q1 = Q_r[:, :rank]
    phi_base = round(solve(R1, Q1ᵀ @ tau), 6)
    W_b_final = Q1 @ R1
    # 5) M: phi_base = M @ theta_r  (在 W_e 列序下)
    M = zeros(rank, n_e)
    for i in 0..rank-1: M[i, base_idx[i]] = 1
    for k in 0..n_e-rank-1: M[:, dep_idx[k]] = beta[:, k]
    # 6) 表达式
    expr[i] = params_r[base_idx[i]] + Σ_{k} (|beta[i,k]|>tol_beta ? sign(beta[i,k])*round(|beta[i,k]|,6)*params_r[dep_idx[k]] : "")
    return W_b_final, phi_base, M, expr
```

**不变量**：`M @ theta_r ≈ phi_base`，且 `W_e @ theta_r ≈ W_b @ (M @ theta_r)`（列空间一致）——这正是 [test_qr_decomposition](../../../tests/unit/test_qr_decomposition.py) 的 M 不变量断言。

---

## 9. 复杂度

- 列消除：$O(m\,n)$（算列范数）。
- QR：$O(m\,n_e^2)$（主元 QR）；第二次 QR $O(m\,n_e^2)$。
- $\beta$ 求解：$O(r^2(n_e-r) + r^3)$（三角 solve）。
- 总体对样本数 $m$ 线性、对参数数 $n$ 二次，适合 $m\gg n$ 的辨识场景。
