# tools 模块（算法）

> 模块：[src/figaroh/tools/](../../../src/figaroh/tools/) · 实现见 [code](code.md)

本模块是数值侧算法实现。符号侧（CasADi 解析回归子、Fourier D-最优 NLP）见 [backend](../backend/algorithm.md) 与 [optimal](../optimal/algorithm.md)；辨识管线与质量指标见 [identification](../identification/algorithm.md)，几何标定见 [calibration](../calibration/algorithm.md)。跨模块数学公共部分见 [RNEA 与回归子](../algorithms/rnea-regressor.md) 和 [基参数约简](../algorithms/base-parameter-reduction.md)。

---

## 1. 回归子构建（数值）

### 1.1 共享数学

逆动力学对惯性参数线性：$\tau = \mathrm{RNEA}(q,\dot q,\ddot q,\pi) = H(q,\dot q,\ddot q)\,\pi_{\text{std}}$，回归子 $H\in\mathbb{R}^{n_v\times 10n_v}$ 即 RNEA 对参数的雅可比（详见 [rnea-regressor](../algorithms/rnea-regressor.md) §2-3）。每连杆 10 参数（Pinocchio 顺序 $[m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]$），全机器人 $\pi_{\text{std}}\in\mathbb{R}^{10n_v}$。扩展模型追加摩擦/电机惯量/零位偏置列（[rnea-regressor](../algorithms/rnea-regressor.md) §4）。

### 1.2 模块特有实现

[regressor.py](../../../src/figaroh/tools/regressor.py) 的 `RegressorBuilder` 逐采样点调用 `pin.computeJointTorqueRegressor` 数值求 $H$，按行堆叠成 $W$。

**行序（关节主序）**：对样本 $i$、关节 $j$，

$$
\text{base\_idx} = j\cdot N + i
$$

即同一关节的 $N$ 个采样行连续。这使 $\tau_{\text{vec}}$ 与 $W$ 行序一致（$\tau[j{\cdot}N{+}i]$ 对应 $W$ 同行）。

**列序（步长 $(10+\text{add})\cdot n_v$）**：前 $10n_v$ 列为刚体参数（直接取 $W_{\text{temp}}[j,:]$），其后按 `additional_columns` 布局附加列：

| 段 | 列偏移（`param_start=10·nv`） | 值 |
|----|------|----|
| 粘性摩擦 $f_v$ | `param_start + j` | $\dot q_j(i)$ |
| Coulomb $f_s$ | `param_start + nv + j` | $\mathrm{sgn}(\dot q_j(i))$ |
| 电机惯量 $I_a$ | `param_start + 2·nv + j` | $\ddot q_j(i)$ |
| 零位偏置 $o$ | `param_start + 3·nv + j` | $1$ |

`add = 2·has_friction + has_actuator_inertia + has_joint_offset`，列数 $n_{\text{param}}=(10+\text{add})\cdot n_v$。

**附加列填充 `_fill_joint_regressor_sample`**：仅在 `j in identif_config["act_idxv"]`（活动关节集）时填附加列——非活动关节无摩擦/电机项。

**`_get_nonzero_inertias`**：`__init__` 计算 $[i\;|\;m_i>10^{-9}]$ 存 `self.nonzero_inertias`，原意是过滤零质量虚拟连杆以减负，但 build 方法当前未消费，仍遍历全部 $n_v$（潜伏未用）。

**`_reorder_parameters` 死代码**：定义了 Pinocchio→barycentric 列重排（`param_order=[4,5,7,6,8,9,1,2,3,0]`，即 $[I_{xx},I_{xy},I_{xz},I_{yy},I_{yz},I_{zz},mc_x,mc_y,mc_z,m]$），但调用被注释，实际返回 **Pinocchio 顺序**——回归子列序与 $\pi_{\text{std}}$ 一致，下游 QR/LS 须沿用此序。

### 1.3 伪代码

```
# 输入: robot, q[N,nq], v[N,nv], a[N,nv], identif_config
# 输出: W ∈ R^{N·nv, (10+add)·nv}
function build_basic_regressor(robot, q, v, a, identif_config):
    nv  = robot.model.nv
    add = 2*cfg.has_friction + cfg.has_actuator_inertia + cfg.has_joint_offset
    W   = zeros(N*nv, (10+add)*nv)
    for i in 0..N-1:
        H = pin.computeJointTorqueRegressor(robot.model, robot.data, q[i], v[i], a[i])  # (nv,10*nv)
        for j in 0..nv-1:                       # 关节主序堆叠
            base = j*N + i
            W[base, 0:10*nv] = H[j, :]
            if j in identif_config["act_idxv"]:
                p = 10*nv
                if cfg.has_friction:
                    W[base, p+j]        = v[i,j]            # fv
                    W[base, p+nv+j]     = sign(v[i,j])      # fs
                if cfg.has_actuator_inertia:
                    W[base, p+2*nv+j]   = a[i,j]             # Ia
                if cfg.has_joint_offset:
                    W[base, p+3*nv+j]   = 1.0                # off
    return W   # Pinocchio 顺序，未走 _reorder_parameters
```

---

## 2. QR 双分解（核心）

数学公共部分见 [base-parameter-reduction](../algorithms/base-parameter-reduction.md)。本节给模块特有实现与完整推导。

### 2.1 列消除（去零影响列）

判据为列 $L_2$ 范数平方，即 $W^\top W$ 对角元：

$$
\|W_{:,j}\|_2^2 = (W^\top W)_{jj} < \varepsilon_e\;\;(=10^{-6}) \;\Longrightarrow\;\text{删除}
$$

`eliminate_non_dynaffect` 保留 $\ge\varepsilon_e$ 的列；`get_index_eliminate` 返回待删索引与剩余参数名 `params_r`；`build_regressor_reduced(W, idx_e)=np.delete(W, idx_e, 1)`。得 $W_e\in\mathbb{R}^{m\times n_e}$（$m=N n_v$，$n_e\le n$）。

### 2.2 QRDecomposer.decompose 统一入口

`decompose(W_e, params_r, tau=None, params_std=None, method="double")`：`method` 经 `strip().lower()` 归一化（`"double"/"double_qr"/"double-qr"` 与 `"pivoting"/"pivot"/"qr_pivoting"/"qr-pivoting"`），返回结构化 `QRResult`。`tau` 缺省置零向量（足够求秩/基，但 $\phi_b$ 无意义）。

### 2.3 数值秩 _find_rank（相对/绝对容差）

$$
\text{thr} = \begin{cases}\max(\text{rel\_tol}\cdot|R_{00}|,\;\text{tol}) & \text{rel\_tol}\neq\text{None}\\ \text{tol}\;(=10^{-8}) & \text{否则}\end{cases},\quad r=\#\{i:|R_{ii}|>\text{thr}\}
$$

相对容差按最大主元 $|R_{00}|$ 缩放，跨回归子尺度更稳健；绝对 `tol` 作 floor 防近零首对角元误判。

### 2.4 路径 A：decompose_with_pivoting（单次列主元 QR）

`scipy.linalg.qr(W_e, pivoting=True)` 给 $W_e P = Q R$（$P$ 置换）。参数按主元重排 `params_sorted = [params_r[P[i]]]`，前 $r$ 个为基列。

**基提取** `_extract_base_components(R, Q, rank)`：$R_1=R[:r,:r]$、$Q_1=Q[:,:r]$、$R_2=R[:r,r:]$。

> ⚠️ 统一入口 `decompose(method="pivoting")` 对 $Q_1$ 取了 `np.linalg.qr(W_e[:,P])[0]`（对主元列再做一次无主元 QR），与 `decompose_with_pivoting` 直接用原始 $Q$ 略有差异；遗留 `QR_pivoting` 函数走后者。

**依赖系数**：$W_d = Q_1 R_2$（$R_3\approx0$），故

$$
\beta = R_1^{-1}R_2\in\mathbb{R}^{r\times(n_e-r)},\qquad W_d = W_b\,\beta
$$

**基参数值**（QR 解，数值稳于正规方程）：

$$
\phi_b = \mathrm{round}\big(R_1^{-1}Q_1^\top\tau,\;6\big),\qquad W_b = Q_1 R_1
$$

### 2.5 路径 B：double_decomposition（双 QR，工程默认）

**第一步 `_identify_base_parameters`**：与路径 A 同样用 `scipy.linalg.qr(W_e, pivoting=True)` 选基，但 `base_indices=sorted(P[:rank])`、`regroup_indices=sorted(P[rank:])`——**排序保证确定性**，不依赖主元次序。

**第二步 `_regroup_parameters`**：按基/依赖索引切片 $W_e$ 为 $[W_b\;|\;W_d]$（`np.c_` 拼接），保持参数名同步分块。

**第三次分解**：对 $[W_b|W_d]$ 做无主元 QR：$[W_b|W_d]=Q_r R_r$，分块 $R_r=\begin{bmatrix}R_1&R_2\\0&R_3\end{bmatrix}$。因 $W_b$ 列满秩，$R_1$ 非奇异上三角。

**β 与 $\phi_b$**：与路径 A 同公式 $\beta=R_1^{-1}R_2$、$\phi_b=\mathrm{round}(R_1^{-1}Q_1^\top\tau,6)$，但 $R_1,R_2,Q_1$ 来自第二次 QR——对病态 $W_e$ 更稳。

### 2.6 M 构造（核心不变量 $\phi_{\text{base}}=M\theta_r$）

参数向量按基/依赖分块 $\theta_r=[\theta_b;\theta_d]$（$W_e$ 列序）。由 $W_e\theta_r=W_b\theta_b+W_d\theta_d=W_b(\theta_b+\beta\theta_d)$，定义

$$
\phi_{\text{base}} = \theta_b + \beta\,\theta_d = M\,\theta_r,\qquad M=[\,I_r\;|\;\beta\,]\in\mathbb{R}^{r\times n_e}
$$

两路径把 $M$ 映射回 `params_r` 原列序的方式不同：

- **`_build_M_from_partition`（double）**：基列处置单位行 $M[i,\text{base\_idx}_i]=1$，依赖列置 $\beta$ 列 $M[:,\text{regroup\_idx}_k]=\beta_{:,k}$。
- **`_build_M_from_pivoting`（pivoting）**：先在主元序下构造 $M_{\text{sorted}}=[I_r|\beta]$，再按 $P$ 反映射 $M[:,\text{original\_idx}]=M_{\text{sorted}}[:,\text{sorted\_idx}]$。

### 2.7 表达式与标称值

`_build_parameter_expressions`：$\phi_i=\theta_{b,i}+\sum_k\beta_{i,k}\theta_{d,k}$，仅 $|\beta_{i,k}|>\varepsilon_\beta(=10^{-6})$ 显示，系数 `round(|β|,6)`，符号用 ` + / - `。`compute_nominal_base_parameters` 用先验 `params_std` 代入同一表达式得 $\phi_{b,\text{nom}}$（`round(...,5)`），仅 double 路径在 `params_std≠None` 时计算。

### 2.8 expand_mapping_matrix_to_full

$M$ 定义在去零列后的 `params_r`（$n_e$ 列）序上。映射回完整标准参数序 $\pi$（$n$ 列，含被删零列）：

$$
M_{\text{full}}[:,\text{keep}]=M,\qquad M_{\text{full}}[:,\text{deleted}]=0
$$

按参数名匹配定位列；用于 [全参数重建](../algorithms/physical-consistency.md)（由 $\phi_{\text{base}}$ 反解 $\theta_r$）。

### 2.9 double_decomposition 完整伪代码

```
# 输入: tau∈R^m, W_e∈R^{m×n_e}, params_r (n_e 个名), params_std?
# 输出: W_b, base_parameters, params_base_expr, phi_b, [phi_b_nom]
function double_decomposition(tau, W_e, params_r, params_std):
    # 1) 选基（scipy 列主元 QR，排序保确定性）
    _, R, P = scipy.linalg.qr(W_e, pivoting=True)
    r   = _find_rank(R)
    base_idx   = sorted(P[:r])
    regroup_idx = sorted(P[r:])
    # 2) 分块
    W_b = W_e[:, base_idx];  W_d = W_e[:, regroup_idx]
    params_base   = [params_r[i] for i in base_idx]
    params_regroup = [params_r[i] for i in regroup_idx]
    # 3) 第二次 QR（无主元，对 [W_b|W_d]）
    Q_r, R_r = np.linalg.qr(np.c_[W_b, W_d])
    R1 = R_r[:r,:r];  Q1 = Q_r[:,:r];  R2 = R_r[:r, r:]
    # 4) β 与 φ_b（QR 解，full 精度，仅显示 round）
    beta   = solve(R1, R2) if R2.size else empty((r,0))
    phi_b  = round(solve(R1, Q1.T @ tau), 6)
    W_b    = Q1 @ R1
    # 5) M: φ_base = M @ θ_r  （W_e 原列序）
    M = zeros(r, n_e)
    for row, b in enumerate(base_idx):   M[row, b] = 1.0
    for col, d in enumerate(regroup_idx): M[:, d]  = beta[:, col]
    # 6) 表达式
    expr[i] = params_base[i] + Σ_k (|β[i,k]|>tol_β ? sign*round(|β[i,k]|,6)*params_regroup[k] : "")
    # 7) 标称值（可选）
    if params_std: phi_b_nom = round([std[p]+Σ β·std[dep]], 5)
    return W_b, dict(zip(expr,phi_b)), expr, phi_b, [phi_b_nom]
```

**M 构造伪代码（_build_M_from_partition）**

```
function _build_M_from_partition(beta, base_idx, regroup_idx, n):
    r = len(base_idx);  M = zeros(r, n)
    for row, b in enumerate(base_idx):  M[row, b] = 1.0          # 基列：单位行
    if beta.size:
        for col, d in enumerate(regroup_idx):  M[:, d] = beta[:, col]   # 依赖列：β 列
    return M
```

**不变量**（[test_qr_decomposition](../../../tests/unit/test_qr_decomposition.py) 断言）：$M\theta_r\approx\phi_{\text{base}}$，且 $W_e\theta_r\approx W_b(M\theta_r)$（列空间一致）。

---

## 3. 10 法线性求解器

[solver.py](../../../src/figaroh/tools/solver.py) `LinearSolver` 解超定 $Ax=b$（$A\in\mathbb{R}^{m\times n}$，$m\ge n$）。`solve` 统一校验输入（欠定告警）、分发到 `_solve_<method>`、最后 `_compute_solution_quality` 填 `solver_info`（RSS/RMSE/$R^2$/条件数）。

| 方法 | 公式 / 算法 |
|------|------------|
| `lstsq` | `np.linalg.lstsq`（SVD）$x=V\Sigma^+U^\top b$；返回 residuals/rank/singular_values |
| `qr` | `np.linalg.qr(A)` → $x=$`solve_triangular(R, Qᵀb)` |
| `svd` | $r=\#\{s_i>s_0\cdot\max(m,n)\cdot\epsilon\}$；$s^{-1}_i=1/s_i$（$i<r$）伪逆 $x=V\Sigma^+U^\top b$；记条件数 $s_0/s_r$ |
| `ridge` | $(A^\top A+\alpha I)x=A^\top b$，`linalg.solve(..., assume_a="pos")` |
| `lasso` | `sklearn.Lasso`（坐标下降，`fit_intercept=False`）；缺 sklearn 回退 `ridge` 并告警 |
| `elastic_net` | `sklearn.ElasticNet(alpha, l1_ratio)`；缺 sklearn 回退 `ridge` |
| `tikhonov` | $(A^\top A+\alpha L^\top L)x=A^\top b$，$L$ 取 `constraints["L"]`（缺省 $I$） |
| `constrained` | SLSQP：$\min\frac12\|Ax-b\|^2$，jac $=A^\top(Ax-b)$；等式 $A_{\text{eq}}x=b_{\text{eq}}$、不等式 $A_{\text{ineq}}x\le b_{\text{ineq}}$、界 `bounds` |
| `robust` | IRLS + Huber（见 §3.1） |
| `weighted` | $\sqrt{w}\odot$ 逐行缩放 → `lstsq`（但实现用 `np.diag` 膨胀，见 [code](code.md) §5） |

### 3.1 IRLS + Huber 完整推导

稳健回归最小化 Huber 损失 $\sum_i\rho(r_i/\sigma)$，$r_i=(Ax-b)_i$。Huber 函数（调谐常数 $c=1.345$，在 Gaussian 处达 95% 效率）：

$$
\rho(u)=\begin{cases}\tfrac12 u^2 & |u|\le c\\ c|u|-\tfrac12 c^2 & |u|>c\end{cases},\qquad \rho'(u)=\begin{cases}u & |u|\le c\\ c\,\mathrm{sgn}(u) & |u|>c\end{cases}
$$

IRLS 把稳健 LS 化为迭代加权 LS。对 $\sum\rho(r_i/\sigma)$，权为**影响函数除以残差**（标准化）：

$$
w(u)=\frac{\rho'(u)}{u}=\begin{cases}1 & |u|\le c\\ \dfrac{c}{|u|} & |u|>c\end{cases},\qquad u_i=\frac{r_i}{\sigma}
$$

（小残差权 1，大残差权 $c/|u|$ 下降，实现降权离群点。）

**尺度 $\sigma$**：MAD 稳健估计

$$
\sigma = 1.4826\cdot\mathrm{median}\big(|r|\big)
$$

（$1.4826=1/\Phi^{-1}(0.75)$ 把 MAD 标准化为 Gaussian $\sigma$ 相合估计。）$\sigma<\text{tol}$ 时残差近零，提前收敛退出。

**迭代**：$x\leftarrow\mathrm{lstsq}(\mathrm{diag}(w)\cdot A,\;\mathrm{diag}(w)\cdot b)$，收敛判据 $\|x_{\text{new}}-x\|<\text{tol}$。

### 3.2 IRLS 伪代码

```
# 输入: A∈R^{m×n}, b∈R^m, max_iter, tol, c=1.345
# 输出: x
function _solve_robust(A, b):
    x = lstsq(A, b)                          # 初值：普通 LS
    for it in 0..max_iter-1:
        r   = A @ x - b
        sig = 1.4826 * median(|r|)           # MAD 尺度
        if sig < tol: break                  # 残差近零
        u   = r / sig
        w   = where(|u| <= c, 1.0, c / |u|)  # Huber 权 w = ρ'(u)/u
        x_new = lstsq(diag(w) @ A, diag(w) @ b)
        if norm(x_new - x) < tol: x = x_new; break
        x = x_new
    return x, weights
```

> 实现以 `np.diag(weights)` 构 $m\times m$ 对角阵，$O(m^2)$ 内存；应改 $\sqrt{w}\odot A$ 逐行缩放。

---

## 4. IPOPT 抽象

[robotipopt.py](../../../src/figaroh/tools/robotipopt.py) 提供 cyipopt（IPOPT 的 Python 绑定）上层封装。

### 4.1 BaseOptimizationProblem(ABC)

5 个抽象方法：`get_variable_bounds`/`get_constraint_bounds`/`get_initial_guess`/`objective(x)`/`constraints(x)`。4 个默认：

- `gradient(x)`：`nd.Gradient(self.objective)(x)`（numdifftools 自动微分；失败返 0 向量）。
- `jacobian(x)`：`nd.Jacobian(self.constraints)(x)`；空约束返 $(0,n)$，标量约束 reshape 成 $(1,n)$；失败返单位阵。
- `hessian(x, lagrange, obj_factor)`：默认返 `False` → 用 IPOPT 的 `hessian_approximation`（limited-memory/exact）。
- `intermediate(...)`：回调，把 `(iter_count, obj_value, max(inf_pr,inf_du))` 追加到 `iteration_data` 并 log；返 `True` 继续。

### 4.2 IPOPTConfig 与 to_ipopt_options

`@dataclass`，字段含 `tolerance`/`acceptable_tolerance`/`max_iterations`/`max_cpu_time`/`print_level`/`hessian_approximation`/`warm_start`/`check_derivatives`/`linear_solver`/`custom_options`。`to_ipopt_options` 转为 IPOPT 选项 dict：**所有字符串键与值编码为字节**（`b"tol"`、`b"hessian_approximation": b"limited-memory"` 等），因 cyipopt 走 C 接口要求 bytes；数值保持 float/int；`custom_options` 逐项 `str→bytes`。

两预设：`for_trajectory_optimization`（`tol=1e-4`，limited-memory，`mu_strategy=adaptive`，`adaptive_mu_globalization=obj-constr-filter`，`max_iter=1000`）；`for_parameter_identification`（`tol=1e-8`，exact Hessian，`mu_strategy=monotone`，`fixed_variable_treatment=make_parameter`，`max_iter=5000`）。

### 4.3 RobotIPOPTSolver.solve 伪代码

```
# 输入: problem(BaseOptimizationProblem), config(IPOPTConfig)
# 输出: (success, results)
function solve(self):
    import cyipopt                          # 惰性导入，缺失抛 ImportError
    x0 = problem.get_initial_guess()
    lb, ub = problem.get_variable_bounds()
    cl, cu = problem.get_constraint_bounds()
    nlp = cyipopt.Problem(n=len(x0), m=len(cl), problem_obj=problem,
                          lb=lb, ub=ub, cl=cl, cu=cu)
    for key, val in config.to_ipopt_options().items():
        nlp.add_option(key, val)           # 字节键
    x_opt, info = nlp.solve(x0)
    success = info["status"] in {-1, 0, 1}  # -1/0/1 = 可接受/求解/可接受
    return success, {x_opt, obj_val, status, status_msg, solve_time,
                     iterations, iteration_data, callback_data, ipopt_info}
```

`TrajectoryOptimizationProblem` 继承 ABC，提供 `set_objective_function`/`set_constraint_function` 注入回调；`create_trajectory_solver` 工厂把四个函数闭包进 `CustomProblem`。

> 与 [optimal](../optimal/algorithm.md) 的 CasADi 路径区别：Fourier D-最优 NLP 用 `cs.nlpsol("ipopt","mumps")`，梯度由 CasADi 自动微分穿透 SX/MX 得解析式；本模块 `RobotIPOPTSolver` 走 cyipopt，梯度默认 numdifftools 数值自动微分——适合无解析梯度的通用机器人 NLP。

---

## 5. 碰撞检测

[robotcollisions.py](../../../src/figaroh/tools/robotcollisions.py) `CollisionManager` 封装 Pinocchio 几何：

- `setup_collision_pairs`：`geom_model.addAllCollisionPairs()` 全配对，再用 SRDF `pin.removeCollisionPairs(model, geom_model, srdf_path)` 移除主动忽略对（如相邻连杆）。
- `geom_data.collisionRequests.enable_contact=True`：开启接触点提取。
- `check_collisions(q)`：`pin.updateGeometryPlacements` 同步几何位姿后 `pin.computeCollisions(..., q, False)` 返回 bool。
- `get_collision_details`：遍历 `collisionResults`，取 `isCollision()` 的对及 `getContact(0)`。
- `get_all_distances`：逐对 `pin.computeDistance(...).min_distance`。
- `visualize_collisions`：在接触点 `contact.pos` 放红色球（`addSphere`，半径 0.01），缓存 `_vis_cache`，`_cleanup_old_contacts` 删旧。

`CollisionWrapper` 是遗留别名层，`rmodel/rdata/gmodel/gdata` 旧名 + `add_collisions`/`computeCollisions`/`getCollisionList` 等 camelCase 方法，供主链调用。

---

## 6. 机器人加载

[load_robot.py](../../../src/figaroh/tools/load_robot.py) `load_robot(robot_urdf, loader, ...)` 三后端分发：

- `figaroh`（默认）：`_load_figaroh_original` → `Robot(urdf, package_dirs, isFext)`（继承 RobotWrapper）。
- `robot_description`：`_load_robot_description` 走 `robot_descriptions` 包，自动补 `_description` 后缀，`isFext` 映射 `root_joint=JointModelFreeFlyer`。
- `yourdfpy`：`_load_yourdfpy` 返回 `yourdfpy.URDF` 对象（适配 viser 可视化）。

`_prepare_package_dirs`：显式 `package_dirs` > `robot_pkg` 经 `rospkg` 解析 > URDF 同目录；`"models"` 触发 `_get_models_directory`。

`_get_models_directory` **四级回退**定位 `figaroh-examples/models`：① `import figaroh_examples` 取 `__file__`；② 遍历 `sys.path` 找包目录/`.egg-info` + `importlib.util.find_spec`；③ `importlib.metadata`/`pkg_resources` 取 distribution 文件；④ 开发路径回退（相对 `__file__` 上溯、`~/figaroh-examples`、cwd）。皆败则抛 `ImportError`。

[robot.py](../../../src/figaroh/tools/robot.py) `Robot`：`initFromURDF(root_joint=JointModelFreeFlyer if isFext)`，free-flyer 限位 `(-1,1)`、四元数 `[-1,1]`；`display_q0` 按 `gepetto`/`meshcat` 选可视化器；`num_joints`/`actuated_joint_names` 跳过 universe 与可选 free-flyer。

---

## 7. 复杂度

- **回归子构建**：$N$ 次调用 `computeJointTorqueRegressor`，每次 $O(n_v)$（Pinocchio 递推），总计 $O(N\,n_v)$；附加列填充 $O(N\,n_v)$。
- **列消除**：$O(m\,n)$（算 $W^\top W$ 对角）。
- **QR**：列主元 QR $O(m\,n_e^2)$；double 路径第二次 QR $O(m\,n_e^2)$；$\beta$ 三角 solve $O(r^2(n_e-r)+r^3)$。总体对样本数 $m=N n_v$ 线性、对参数数 $n$ 二次，适合 $m\gg n$。
- **求解器**：lstsq/svd/qr $O(m n^2)$；ridge/tikhonov $O(m n^2+n^3)$；constrained(SLSQP) 迭代，每次 $O(m n^2)$；robust $O(\text{iter}\cdot m n^2)$；weighted 同 lstsq（实现因 `np.diag` 退化到 $O(m^2 n)$）。
- **IPOPT**：内点法，每次迭代解 KKT（规模随变量/约束数）；默认 limited-memory Hessian 近似避免 $O(n^2)$ Hessian 存储。

### 7.1 关键不变量

- 回归子列序 = Pinocchio $\pi_{\text{std}}$ 序（`_reorder_parameters` 死代码未生效）。
- $M\theta_r=\phi_{\text{base}}$ 且 $W_e\theta_r=W_b\phi_{\text{base}}$（QR 双分解列空间不变量）。
- $M_{\text{full}}$ 在被删零列处置 0，保持与 $n$ 维完整参数序对齐。
