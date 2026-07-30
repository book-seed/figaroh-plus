# D-最优实验设计

> 跨模块算法基础。被 [optimal](../optimal/algorithm.md)（激励轨迹 + 标定位姿）、[identification](../identification/algorithm.md)（条件数指标）引用。

---

## 1. 动机：让数据对参数最"信息丰富"

辨识精度由回归子 $W_b$（[基参数回归子](base-parameter-reduction.md)）的几何条件决定。给定测量噪声 $\varepsilon\sim\mathcal{N}(0,\sigma^2 I)$：

$$
\tau = W_b\,\phi_{\text{base}} + \varepsilon
$$

LS 估计 $\hat\phi = (W_b^\top W_b)^{-1}W_b^\top\tau$ 的协方差为

$$
\mathrm{Cov}(\hat\phi) = \sigma^2\,(W_b^\top W_b)^{-1}
$$

**实验设计的任务**：在约束下选择轨迹/位姿，使 $(W_b^\top W_b)^{-1}$ 在某意义下"小"——即让 $W_b^\top W_b$（信息矩阵）"大"。

---

## 2. 信息矩阵

定义**信息矩阵**（per-sample 归一）：

$$
\mathcal{I} = \frac{1}{N}\,W_b^\top W_b \;\in\;\mathbb{R}^{r\times r}
$$

（$r$ = 基参数数；$N$ = 采样点数。归一化使量纲与 $N$ 无关，便于跨实验比较。）代码中 `J = W_bᵀ @ W_b / N_s`。

$\mathcal{I}$ 正定要求 $W_b$ 列满秩——已由基参数约简保证；但数值上需加正则 $\lambda I$ 防 Cholesky 失败：

$$
\mathcal{I}_{\text{reg}} = \mathcal{I} + \lambda I,\qquad \lambda = \texttt{reg\_lambda}\;(10^{-6})
$$

---

## 3. D-最优准则：最大化 $\det(\mathcal{I})$

**D-最优** = 最小化参数协差体积 $\propto \det(\mathcal{I}^{-1})$ = 最大化 $\det(\mathcal{I})$：

$$
\max_{\text{轨迹/位姿}}\;\det(\mathcal{I}) \;\;\Longleftrightarrow\;\; \min\;-\log\det(\mathcal{I}_{\text{reg}})
$$

等价性：$\det(\mathcal{I}^{-1}) = 1/\det(\mathcal{I})$，故最小化协差体积 $\det(\mathrm{Cov})^{1/r} = \sigma^2/\det(\mathcal{I})^{1/r}$ 即最大化 $\det(\mathcal{I})$。

### 3.1 Cholesky logdet 实现

$\mathcal{I}_{\text{reg}}$ 对称正定 → Cholesky 分解 $\mathcal{I}_{\text{reg}} = L L^\top$（$L$ 下三角）。则

$$
\det(\mathcal{I}_{\text{reg}}) = \prod_i L_{ii}^2 \;\;\Longrightarrow\;\; \log\det = 2\sum_i \log L_{ii}
$$

故目标函数（代码）：

$$
\boxed{\;\;f = -2\sum_{i=1}^{r}\log\big(\mathrm{diag}(\mathrm{chol}(\mathcal{I}_{\text{reg}}))_i\big) = -\log\det(\mathcal{I}_{\text{reg}})\;\;}
$$

> ⚠️ CasADi 的 `cs.chol` 仅支持 SX/DM，不支持 MX。故 Fourier 策略把 $\mathcal{I}_{\text{reg}}$（MX）包进一个 SX `cs.Function`（输入 $J_{sx}$，输出 $2\sum\log\mathrm{diag}(\mathrm{chol}(J_{sx}))$），在 MX 图中调用——`logdet_fn(J_reg)`。这正是 [backend](../backend/algorithm.md) 的 SX/MX 混合模式。

### 3.2 其它最优准则（变体）

| 准则 | 目标 | 几何意义 |
|------|------|---------|
| **D-最优** | $\max\det(\mathcal{I})$ | 协差椭球体积最小 |
| **A-最优** | $\min\mathrm{tr}(\mathcal{I}^{-1})$ | 协差椭球轴长平方和最小 |
| **E-最优** | $\max\lambda_{\min}(\mathcal{I})$ | 最短轴最大 |
| **cond** | $\min\,\kappa(W_b)=\sigma_{\max}/\sigma_{\min}$ | 各向同性（样条路径用） |

本工程 Fourier 路径用 **D-最优**（`obj=-logdet`），样条路径用**条件数** `np.linalg.cond(W_b)`。

---

## 4. 应用一：D-最优激励轨迹（Fourier NLP）

决策变量 = Fourier 系数 $Z\in\mathbb{R}^{n_{\text{act}}(2N_h+1)}$（见 [utils/algorithm](../utils/algorithm.md) 的 Fourier 参数化）。轨迹 $q(t),\dot q(t),\ddot q(t)$ 由 $Z$ 经解析公式给出 → 代入 [回归子](rnea-regressor.md) 得 $W_b(Z)$ → 目标 $f(Z)=-\log\det(W_b(Z)^\top W_b(Z)/N + \lambda I)$。

**约束**（逐采样点逐关节）：

- 位置：$\underline{q} \le q(t) \le \overline{q}$（取自 `model.lowerPositionLimit/upperPositionLimit`，软限 `soft_lim` 收缩）
- 速度：$|\dot q(t)| \le \overline{\dot q}$
- 力矩：$|\tau(t)| \le \overline{\tau}$，其中 $\tau(t) = \mathrm{rnea}(q,\dot q,\ddot q) + f_v\dot q + f_s\tanh(\alpha\,\dot q)$（含摩擦，`has_friction` 时）

NLP：$\min_Z f(Z)$ s.t. 上述约束，`cs.nlpsol("ipopt","mumps")` 求解（`tol=1e-4`，`mu_strategy=adaptive`，`hessian_approximation=limited-memory`）。

### 复杂度与并行

- 内层 SX `regressor_function`/`rnea_function` 经 `.map(N,"openmp")` 并行求 $N$ 采样点 → $O(N)$ 并行。
- 外层 MX 自动微分穿透 SX Function → IPOPT 得到解析梯度。
- 总成本 dominated by IPOPT 迭代 × 每次 map 求值；典型 UR10 6-DOF、$N_h=5$、$N=200$ 约 1–3 分钟（[developer-guide](../../developer-guide.md)）。

---

## 5. 应用二：D-最优标定位姿选取（SOCP）

几何标定从候选位姿池 $\{q_i\}_{i=1}^{M}$ 中选子集（权重 $w_i\ge0$，$\sum w_i\le1$），使信息矩阵最大。每位姿贡献 $X_i = R_i^\top R_i$（$R_i$ = 该位姿的运动学回归子，见 [calibration/algorithm](../calibration/algorithm.md)）。加权信息矩阵 $\mathcal{I}(w)=\sum_i w_i X_i$。

**D-最优 → SOCP**（det 的 concave 根 $\detroot$）：

$$
\max_{w,t}\; t \quad\text{s.t.}\quad t \le \mathrm{detroot}\Big(\sum_i w_i X_i\Big),\;\;\sum_i w_i\le1,\;\;w_i\ge0
$$

其中 $\mathrm{detroot}(A)=\det(A)^{1/(2r)}$ 是 $\det$ 的 concave 根，使约束成为 SOC（二阶锥）可表。picos 建模，`cvxopt` 求解（[optimal/SOCPOptimizer](../optimal/code.md)）。选 $w_i>\varepsilon_{\text{opt}}$ 的位姿。

### DetMax 贪心交换（备选）

SOCP 之外，`Detmax` 用贪心 add/remove 交换：随机初始化子集 → 反复 ADD（加入最能提升 $\det\mathcal{I}$ 的候选）与 REMOVE（删去最不损的成员）直至收敛（`opt_k==rm_j`）。$O(\text{iters}\cdot M\cdot\text{NbChosen})$，适合 $M$ 较大时。

---

## 6. 伪代码（Fourier D-最优 NLP 目标）

```
# 输入: 决策变量 Z (MX), n_harmonics, n_samples, idx_b, reg_lambda, CasadiBackend cas_be
# 输出: 目标 obj (标量 MX), 约束 cons, 界 cl/cu

function build_fourier_nlp(Z, cfg, idx_b, cas_be):
    cas_be._ensure_symbolic_model()
    nq, nv = cas_be._cmodel.nq, cas_be._cmodel.nv
    omega = cfg.fourier_frequency or 1.0 ; T = 2*pi
    t = MX(linspace(0, T, cfg.n_samples))
    # 1) Fourier 系数 -> Q/V/A (解析导数, 列主序 MX)
    Z_mat = reshape(Z, n_act, 1+2*n_harmonics)
    for j: Q[j,:] = a0 + Σ ak sin(kωt) + bk cos(kωt)
           V[j,:] = Σ ak kω cos(kωt) - bk kω sin(kωt)
           A[j,:] = Σ -ak (kω)² sin - bk (kω)² cos
    # 2) 散布到全关节
    Q_full/V_full/A_full = scatter(Q,V,A, act_idxq, act_idxv, nq, nv)
    # 3) 回归子并行求值
    W_fun = cas_be.regressor_function            # SX (q,v,a)->W
    W_raw = W_fun.map(n_samples,"openmp")(Q_full,V_full,A_full)   # (nv, Ns*n_param)
    W_full = vertcat([W_raw[:, i*n_param:(i+1)*n_param] for i in 0..Ns-1])  # (Ns*nv, n_param)
    # 4) D-最优目标 (logdet, SX 包装)
    W_b = W_full[:, idx_b]
    J = W_bᵀ @ W_b / n_samples ;  J_reg = J + reg_lambda * eye(n_base)
    J_sx = SX.sym("J", n_base, n_base)
    logdet_fn = Function("logdet",[J_sx],[2*sum1(log(diag(chol(J_sx))))])
    obj = -logdet_fn(J_reg)
    # 5) 力矩约束 (含摩擦, map openmp)
    tau = cas_be.rnea_function.map(n_samples,"openmp")(Q_full,V_full,A_full)
    if has_friction: tau[act_idxv,:] += fv*V_full + fs*tanh(alpha_opt*V_full)
    tau_flat = reshape(tau, nv*n_samples, 1)
    cons = vertcat(Q_col flattened, V_col flattened, tau_flat)
    return obj, cons, cl, cu
```

完整 NLP 求解与结果提取见 [optimal/algorithm](../optimal/algorithm.md)。
