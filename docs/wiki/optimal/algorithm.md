# optimal 模块（算法）

> 模块：[src/figaroh/optimal/](../../../src/figaroh/optimal/) · 实现见 [code](code.md)

本模块把 [D-最优实验设计](../algorithms/d-optimal-design.md) 落到两条工程管线：**激励轨迹**（Fourier 符号 NLP / 样条数值 NLP）与**标定位姿**（SOCP / DetMax）。回归子与基参数约简的数学见 [RNEA 与回归子](../algorithms/rnea-regressor.md) 和 [基参数约简](../algorithms/base-parameter-reduction.md)；符号计算图（SX/MX 混合、`.map("openmp")`）见 [backend/algorithm](../backend/algorithm.md)。

---

## 1. D-最优激励轨迹 NLP（Fourier 路径，核心）

### 1.1 目标回顾

由 [D-最优设计 §3](../algorithms/d-optimal-design.md) ：最大化信息矩阵行列式等价于

$$
\min_Z\; f(Z) = -\log\det\big(\mathcal{I}_{\text{reg}}\big),\qquad \mathcal{I}_{\text{reg}} = \frac{1}{N}W_b^\top W_b + \lambda I
$$

其中 $W_b = W_{\text{full}}[:,\,\texttt{idx\_b}]$ 取基参数列（[§3](#3-基参数索引计算baseparametercomputer)），$N$ 为采样点数，$\lambda=\texttt{reg\_lambda}=10^{-6}$ 防正定性失败。

### 1.2 决策变量与 Fourier 参数化

决策变量为 Fourier 系数扁平向量 $Z\in\mathbb{R}^{n_{\text{act}}(2N_h+1)}$（MX 符号 `cs.MX.sym("coeffs", n_vars)`）。每关节 $N_h$ 次谐波，系数布局 $[a_0,\, a_1,b_1,\, \dots,\, a_{N_h},b_{N_h}]$。

**速度参数化**：$a_k, b_k$ 是**速度**谐波幅值（非位置）。$a_0$ 为均值位置。速度无 DC 分量（等价零均值），保证周期性。

```python
Z = cs.MX.sym("coeffs", n_act * (1 + 2*n_harmonics))
Z_mat = cs.reshape(Z, n_act, 1 + 2*n_harmonics)   # 列主序！
```

> ⚠️ **列主序**：CasADi 矩阵列主序（Fortran）存储，故 `Z_mat[j,k] = Z[j + k·n_act]`。而 numpy `x0.reshape(n_act, n_coeffs)`（行主序）满足 `Z_opt[j,k] = x[j·n_coeffs + k]`。`_initialize_coefficients` 与结果提取均用 numpy 行主序，与 NLP 内 CasADi 列主序**索引语义不一致**（见 [code gotcha](code.md)）。

### 1.3 Q / V / A 解析公式（速度参数化，为何不用 `cs.gradient`）

轨迹（[utils/fourier_trajectory](../../../src/figaroh/utils/fourier_trajectory.py) 的符号化）采用**速度参数化**以避免 $(k\omega)^2$ 加速度放大导致的数值病态：

$$
\dot q_j(t) = \sum_{k=1}^{N_h}\big[a_{k,j}\sin(k\omega t) + b_{k,j}\cos(k\omega t)\big]
$$

位置通过解析积分得到（$a_{0,j}$ 为均值位置）：

$$
q_j(t) = a_{0,j} + \int_0^t \dot q_j(\tau)\,d\tau
       = a_{0,j} + \sum_{k=1}^{N_h}\Big[-\frac{a_{k,j}}{k\omega}\cos(k\omega t)
                                          + \frac{b_{k,j}}{k\omega}\sin(k\omega t)\Big]
$$

加速度通过解析求导得到：

$$
\ddot q_j(t) = \frac{d}{dt}\dot q_j(t)
             = \sum_{k=1}^{N_h}\big[a_{k,j}(k\omega)\cos(k\omega t)
                               - b_{k,j}(k\omega)\sin(k\omega t)\big]
$$

**相比位置参数化（$q$ 的系数为位置幅值）的优势**：加速度幅值 $\propto k\omega$ 而非 $(k\omega)^2$，使各谐波对 IPOPT 梯度的贡献更均衡，避免高频分量在优化中"失活"。

**周期性条件**：$T = 2\pi/\omega$ 时 $\dot q(0)=\dot q(T)$ 自动满足（无 DC 分量）；$q(0)=q(T)$ 同样成立（零均值速度的积分为周期函数）。

**为何手推而非 `cs.gradient`**：$q(t;Z)$ 本就是 $\sin/\cos$ 的闭式和，其导数和积分是初等函数，已知无需再微分。若用 `cs.gradient(q, t)` 在 MX 图上再构造自动微分图，会 (1) 冗余放大表达式图、(2) 重新推导平凡已知量、(3) 使 `.map("openmp")` 的输入 $q,v,a$ 不再是 $Z$ 的紧凑显式表达式。手写闭式让 $Q,V,A$ 成为 $Z$ 的三个紧凑 MX 表达式，直接喂入 `W_fun.map`。

> 时间向量 $t=\texttt{linspace}(0,T,N)$，其中 $T=2\pi/\omega$。$\omega$ 由 `fourier_frequency` 显式指定，或默认 $\omega = 2\pi / T_{\text{traj}}$（$T_{\text{traj}} = t_s \times (n_{\text{wps}}-1)$，符合 spec § configurable-sampling）。

### 1.4 全关节散布

活动关节系数经 `act_idxq`/`act_idxv` 散布到全关节向量：

```python
Q_full[act_idxq[j], :] = Q_col[j, :]   # (nq, Ns)
V_full[act_idxv[j], :] = V_col[j, :]   # (nv, Ns)
A_full[act_idxv[j], :] = A_col[j, :]
```

非活动关节置零（初始 `MX.zeros`）。

### 1.5 回归子并行求值（MX/SX 穿透）

[CasadiBackend](../../../src/figaroh/backend/casadi.py) 的 `regressor_function` 是 SX `cs.Function` $(q,v,a)\mapsto W\in\mathbb{R}^{n_v\times n_{\text{param}}}$（[RNEA 回归子 §3.1](../algorithms/rnea-regressor.md)）。外层 MX 调 `.map(N,"openmp")` 并行求 $N$ 个采样点：

```python
W_fun = cas_be.regressor_function              # SX
n_param = W_fun.size_out(0)[1]
W_raw  = W_fun.map(N, "openmp")(Q_full, V_full, A_full)   # (nv, Ns*n_param)
# 垂直堆叠每采样块 → (Ns*nv, n_param)
W_full = cs.vertcat(*[W_raw[:, i*n_param:(i+1)*n_param] for i in range(N)])
```

CasADi 自动微分穿透 SX Function：IPOPT 得到解析梯度，无需有限差分。

### 1.6 D-最优目标（Cholesky logdet，SX 包装）

取基参数列 $W_b = W_{\text{full}}[:,\,\texttt{idx\_b}]$（$r$ 列），信息矩阵与正则：

```python
J     = cs.mtimes(W_b.T, W_b) / N_s          # r×r
J_reg = J + reg_lambda * cs.MX.eye(n_base)
```

> ⚠️ `cs.chol` 仅支持 SX/DM，**不支持 MX**。故把 $\mathcal{I}_{\text{reg}}$（MX）包进一个 SX `cs.Function`：

$$
\log\det(\mathcal{I}_{\text{reg}}) = 2\sum_{i=1}^{r}\log\big(\mathrm{diag}(\mathrm{chol}(\mathcal{I}_{\text{reg}}))_i\big)
$$

```python
J_sx       = cs.SX.sym("J_reg", n_base, n_base)
logdet_sx  = 2 * cs.sum1(cs.log(cs.diag(cs.chol(J_sx))))
logdet_fn  = cs.Function("logdet", [J_sx], [logdet_sx])
obj        = -logdet_fn(J_reg)               # MX 图中调用
```

CasADi 对该 SX Function 自动生成伴随灵敏度——与回归子 `.map` 同构的 MX/SX 混合模式。

### 1.7 力矩约束（含摩擦，tanh 近似 Coulomb）

RNEA 经 `rnea_function.map(N,"openmp")` 并行求值，摩擦叠加在活动关节上：

$$
\tau_j(t) = \mathrm{RNEA}_j(q,\dot q,\ddot q) + f_{v,j}\,\dot q_j + f_{s,j}\,\tanh(\alpha\,\dot q_j)
$$

**tanh 近似 Coulomb 的可微性**：物理 Coulomb 项 $f_s\,\mathrm{sgn}(\dot q)$ 在 $\dot q=0$ 处不连续、不可微，破坏 IPOPT 的光滑 NLP 假设。光滑代理 $\tanh(\alpha\dot q)$ 随 $\alpha\to\infty$ 收敛到 $\mathrm{sgn}(\dot q)$（$|\alpha\dot q|\gg1$ 时一致），并在 $\dot q=0$ 附近形成宽度 $O(1/\alpha)$ 的光滑边界层。导数 $\frac{d}{d\dot q}\tanh(\alpha\dot q)=\alpha\,\mathrm{sech}^2(\alpha\dot q)$ 处处有界 → IPOPT 得光滑梯度。

**$\alpha_{\text{opt}}=10$ vs $\alpha_{\text{id}}=100$**：大 $\alpha$ 更逼近真 Coulomb 但梯度更陡（边界层窄 → 病态）；小 $\alpha$ 更平滑但偏离 sgn 行为。故优化取 $\alpha_{\text{opt}}=10$（偏 IPOPT 收敛性），辨识取 $\alpha_{\text{id}}=100$（偏保真度）。本模块仅用 $\alpha_{\text{opt}}$；`tanh_alpha_id` 读取但标注 "reserved, not used"。摩擦系数 `fv_nominal=0.1`、`fs_nominal=0.5` 为标称值，`has_friction=False` 时跳过。

力矩扁平化与约束序：

```python
tau_raw  = rnea_fun.map(N,"openmp")(Q_full, V_full, A_full)   # (nv, Ns)
# 加摩擦（has_friction）...
tau_flat = cs.reshape(tau_raw, nv*N, 1)        # 列主序 → sample-major: i*nv + j
# 约束取 tau_flat[i*nv + act_idxv[j]]
```

### 1.8 位置 / 速度约束

Fourier 路径用**硬限**（不经 `soft_lim` 收缩，与样条路径的 `CubicSpline` 软限不同）：

- 位置：$\underline q_j \le q_j(t_i)\le\overline q_j$，界取 `model.lowerPositionLimit/upperPositionLimit`。
- 速度：$|\dot q_j(t_i)|\le\overline{\dot q}_j$，界取 `model.velocityLimit`。
- 力矩：$|\tau_j(t_i)|\le\overline\tau_j$，界取 `model.effortLimit`。

约束向量 $g=[\text{pos}(N\cdot n_{\text{act}})\,|\,\text{vel}(N\cdot n_{\text{act}})\,|\,\text{tau}(N\cdot n_{\text{act}})]$，逐采样逐活动关节。

### 1.9 IPOPT 选项与系数初始化

```python
opts = {"ipopt.linear_solver": "mumps",
        "ipopt.hessian_approximation": "limited-memory",
        "ipopt.tol": 1e-4, "ipopt.acceptable_tol": 1e-3,
        "ipopt.max_iter": 500, "ipopt.mu_strategy": "adaptive",
        "ipopt.print_level": 3, "print_time": False}
solver = cs.nlpsol("fourier_opt", "ipopt", nlp, opts)
```

初始化 `_initialize_coefficients`：

- $a_{0,j}=$ 关节范围中点（均值位置）；
- $a_{k,j},b_{k,j}\sim\mathcal{U}(-\text{amp},\text{amp})$，$\text{amp}=5\%\cdot\overline{\dot q}_j\,/\,N_h$（速度幅值，基于速度极限并按谐波数均摊），种子 `default_rng(42)`；
- 可行性：在 $t=\texttt{linspace}(0,T,100)$ 上检查位置/速度越限，若违规则将 $a_k,b_k$ **减半**重试，最多 5 次。

### 1.10 完整 NLP 构造与求解伪代码

```
# 输入: context (robot, identif_config, idx_b), fourier_config
# 输出: context.results 填入 T_F/P_F/V_F/A_F

function FourierOptimizationStrategy.solve(context):
    cas_be = CasadiBackend(context.robot); cas_be._ensure_symbolic_model()
    cmodel = cas_be._cmodel ; nq, nv = cmodel.nq, cmodel.nv
    n_h, N, reg, alpha_opt = cfg.n_harmonics, cfg.n_samples, cfg.reg_lambda, cfg.tanh_alpha_opt
    n_act = len(context.active_joints)
    act_idxq, act_idxv = identif_config["act_idxq"], identif_config["act_idxv"]
    n_coeff = 1 + 2*n_h ; n_vars = n_act * n_coeff

    # ── 决策变量与参数化 ──
    Z = MX.sym("coeffs", n_vars)
    Z_mat = reshape(Z, n_act, n_coeff)          # 列主序
    # omega 默认: fourier_frequency 显式 → 用该值；否则 ω=2π/(t_s*(n_wps-1)) (spec)
    freq = cfg.fourier_frequency
    omega = freq if freq is not None else 2*pi/(t_s*(n_wps-1))
    T = 2*pi/omega
    t = MX(linspace(0, T, N)).T                  # (1, N)
    Q_col = MX.zeros(n_act, N); V_col = ...; A_col = ...
    for j in 0..n_act-1:
        Q_col[j,:] = Z_mat[j,0]                  # a0 (均值位置)
        for k in 1..n_h:
            ak, bk = Z_mat[j, 2k-1], Z_mat[j, 2k]  # 速度 sin/cos 幅值
            kw = k*omega
            # v(t) = ak*sin(kωt) + bk*cos(kωt)
            V_col[j,:] += ak*sin(kw*t) + bk*cos(kw*t)
            # q(t) = a0 - ak/(kω)*cos(kωt) + bk/(kω)*sin(kωt)
            Q_col[j,:] += -ak/kw*cos(kw*t) + bk/kw*sin(kw*t)
            # a(t) = ak*kω*cos(kωt) - bk*kω*sin(kωt)
            A_col[j,:] += ak*kw*cos(kw*t) - bk*kw*sin(kw*t)

    # ── 全关节散布 ──
    Q_full, V_full, A_full = MX.zeros(nq,N), MX.zeros(nv,N), MX.zeros(nv,N)
    for j in 0..n_act-1:
        Q_full[act_idxq[j],:] = Q_col[j,:]
        V_full[act_idxv[j],:] = V_col[j,:]
        A_full[act_idxv[j],:] = A_col[j,:]

    # ── 回归子并行 ──
    W_fun = cas_be.regressor_function            # SX
    n_param = W_fun.size_out(0)[1]
    W_raw = W_fun.map(N,"openmp")(Q_full, V_full, A_full)   # (nv, N*n_param)
    W_full = vertcat(*[W_raw[:, i*n_param:(i+1)*n_param] for i in 0..N-1])

    # ── D-最优目标 ──
    W_b = W_full[:, context.idx_b]               # (N*nv, r)
    J = mtimes(W_b.T, W_b) / N ; J_reg = J + reg*MX.eye(r)
    J_sx = SX.sym("J_reg", r, r)
    logdet_fn = Function("logdet",[J_sx],[ 2*sum1(log(diag(chol(J_sx)))) ])
    obj = -logdet_fn(J_reg)

    # ── 力矩约束 + 摩擦 ──
    tau = rnea_fun.map(N,"openmp")(Q_full, V_full, A_full)  # (nv, N)
    if identif_config["has_friction"]:
        for j in 0..n_act-1:
            jid = act_idxv[j]
            tau[jid,:] += fv_nominal*V_full[jid,:] + fs_nominal*tanh(alpha_opt*V_full[jid,:])
    tau_flat = reshape(tau, nv*N, 1)             # 列主序 → sample-major

    # ── 约束向量与界 ──
    g = vertcat( [Q_col[j,i]      for i,j] ,     # 位置
                  [V_col[j,i]      for i,j] ,     # 速度
                  [tau_flat[i*nv + act_idxv[j]] for i,j] )   # 力矩
    cl = [ q_lower[act_idxq[j]] for ... ] ; cu = [ q_upper[...] for ... ]   # 同序拼接

    # ── 求解 ──
    nlp = {"x": Z, "f": obj, "g": g}
    solver = nlpsol("fourier_opt", "ipopt", nlp, opts)   # mumps/limited-mem/adaptive mu
    x0 = _initialize_coefficients(context, n_act, n_h, omega, T)  # a0=中点, ak/bk=5%速度限/谐波数, 减半重试×5
    res = solver(x0=x0, lbg=cl, ubg=cu)

    # ── 结果提取 ──
    x_opt = array(res["x"]).flatten()
    Z_opt = x_opt.reshape(n_act, n_coeff)        # numpy 行主序 (与 NLP 列主序不一致!)
    ft = FourierTrajectory(n_h, n_act, omega, T)
    t_np = linspace(0, T, N)
    q_opt = ft.get_trajectory(t_np, Z_opt)
    v_opt = ft.get_velocity(t_np, Z_opt)
    a_opt = ft.get_acceleration(t_np, Z_opt)
    context.results["T_F"].append(t_np.reshape(-1,1))
    context.results["P_F"].append(q_opt) ; ["V_F"].append(v_opt) ; ["A_F"].append(a_opt)
    context.results["iteration_data"].append({...})
```

---

## 2. 样条路径（条件数 NLP，数值）

### 2.1 三次样条参数化

决策变量 $X$ = 除首路点外的路点位置（首路点 $w_{\text{init}}$ 固定，由上一段末尾传递），形状 $(n_{\text{wps}}-1)\cdot n_{\text{act}}$。样条数学（ndcurves `exact_cubic`、C2 连续、可选路点速度/加速度约束）见 [utils/algorithm](../utils/algorithm.md)。

`WaypointsGeneration`（[utils/cubic_spline](../../../src/figaroh/utils/cubic_spline.py)）：

- `gen_rand_pool()`：在**软限**（`soft_lim_pool` 收缩后的位置/速度/加速度范围）内每关节均匀取 15 个候选；
- `gen_rand_wp(wp_init,...)`：从池随机抽样组成 $n_{\text{wps}}$ 个路点，避免**相邻重复**（防样条退化）；速度/加速度可置零；
- `get_full_config(freq, tps, wps, vel_wps, acc_wps)`：ndcurves 分段拼接，按 `freq` 采样得 $(t, q, v, a)$，非活动关节填零。

### 2.2 目标：条件数

```
objective_function(X, ...):
    wps = vstack(wp_init, reshape(X, (n_wps-1, n_act))).T
    t_f, p_f, v_f, a_f = WP.get_full_config(freq, tps, wps, vel_wps, acc_wps)
    W_b = _stack_base_regressors(p_f, v_f, a_f, W_stack)   # 堆叠前段
    return np.linalg.cond(W_b)
```

`_stack_base_regressors`：`build_regressor_basic`（[tools/regressor](../../../src/figaroh/tools/regressor.py)，数值 RNEA 回归子）→ `build_regressor_reduced(W, idx_e)` → `build_baseRegressor(W_e, idx_b)`；若 `W_stack` 非空则 `vstack`，使多段轨迹的回归子行级联、联合条件数最小。

### 2.3 约束与雅可比（数值）

`TrajectoryConstraintManager.evaluate_constraints`（[contraints](../../../src/figaroh/optimal/contraints.py)）：位置采样、速度采样、力矩（`calc_torque` = pinocchio RNEA）、碰撞距离（`CollisionWrapper`）。雅可比用**前向有限差分**（`eps=1e-6`，逐列扰动）：

$$
J_{:,i} \approx \frac{c(X+\varepsilon e_i) - c(X)}{\varepsilon}
$$

### 2.4 多段堆叠与 IPOPT

`stack_reps` 段循环（仅样条）：每段 `_solve_segment` → `BaseTrajectoryIPOPTProblem.solve_with_waypoints`（cyipopt，`IPOPTConfig.for_trajectory_optimization()`，`tol=1e-3`、`acceptable=1e-2`、`max_iter=200`、`mu_strategy=adaptive`）。段间 `_prepare_next_segment`：取末路点为下段 $w_{\text{init}}$，并 `vstack` 回归子为下段 `W_stack`。

### 2.5 伪代码

```
function SplineOptimizationStrategy.solve(context):
    WP = WaypointsGeneration(robot, n_wps, active_joints, soft_lim_pool)
    constraint_manager = TrajectoryConstraintManager(robot, WP, traj_cfg, identif_cfg)
    WP.gen_rand_pool()
    wp_init = _build_initial_waypoints(context)          # 从池中取可行中位
    vel_wp_init = acc_wp_init = zeros(n_act)
    W_stack = None
    for s_rep in 0..stack_reps-1:                        # trajectory_config["stack_reps"]
        wps, vel_wps, acc_wps, tps, _, p_i, _, _ =
            _generate_feasible_initial_guess(wp_init, vel_wp_init, acc_wp_init)
        tps = t_s*(n_wps-1)*s_rep + tps                  # 段时间偏移
        problem = context.create_ipopt_problem(          # 子类实现 (BaseTrajectoryIPOPTProblem)
            n_joints, n_wps, Ns, tps, vel_wps, acc_wps,
            wp_init, vel_wp_init, acc_wp_init, W_stack)
        ok, res = problem.solve_with_waypoints(wps)
        if not ok: break
        context.results["T_F"/"P_F"/"V_F"/"A_F"/"iteration_data"].append(...)
        if s_rep < stack_reps-1:
            wp_init, W_stack = _prepare_next_segment(context)   # 末路点 + W_stack
```

---

## 3. 基参数索引计算（BaseParameterComputer）

基参数理论（列消除 → 列主元 QR → 双 QR）见 [基参数约简](../algorithms/base-parameter-reduction.md)。本模块用一条**随机样条轨迹**采样得 $W$，再做 QR：

```
function BaseParameterComputer.compute_base_indices(robot, identif_config, soft_lim_pool):
    WP = WaypointsGeneration(robot, n_wps_r=100, active_joints, soft_lim_pool)
    WP.gen_rand_pool()
    wps, vel_wps, acc_wps = WP.gen_rand_wp(vel_set_zero=True, acc_set_zero=True)  # v=a=0
    tps = matrix([0.5*i for i in 0..n_wps_r-1]).T
    t, p, v, a = WP.get_full_config(freq_r=100, tps, wps, vel_wps, acc_wps)
    # ── 回归子与 QR ──
    W = build_regressor_basic(robot, p, v, a, identif_config)   # 数值 RNEA 回归子
    standard_parameter = get_standard_parameters(model, identif_config)
    # 附加摩擦/电机惯量/偏置列 (has_friction / has_actuator_inertia / has_joint_offset)
    idx_e = get_index_eliminate(W, standard_parameter, tol_e=0.001)   # 去零列 (tol=1e-3)
    W_e   = build_regressor_reduced(W, idx_e)
    idx_b = get_baseIndex(W_e, par_r_)                       # 列主元 QR 选基列
    return idx_e, idx_b
```

**与 `trajectory_type` 无关**：无论 Fourier 还是样条优化，基参数索引都由这条独立随机样条计算（速度/加速度置零即可，QR 只需 $W$ 列空间结构）。注意 `tol_e=0.001` 比约简文档默认 `1e-6` 宽松。

---

## 4. D-最优标定位姿选取

几何标定从候选位姿池 $\{q_i\}_{i=1}^{M}$ 中选子集，每位姿贡献信息矩阵 $X_i = R_i^\top R_i$（$R_i$ = 该位姿运动学回归子，见 [calibration/algorithm](../calibration/algorithm.md)）。理论框架见 [D-最优设计 §5](../algorithms/d-optimal-design.md)。

### 4.1 SOCP（SOCPOptimizer，picos + cvxopt）

$$
\max_{w,t}\; t \quad\text{s.t.}\quad t \le \mathrm{detroot}\!\Big(\sum_{i} w_i X_i\Big),\quad \sum_i w_i \le 1,\quad w_i \ge 0
$$

其中 $\mathrm{detroot}(A)=\det(A)^{1/(2r)}$ 是 $\det$ 的 concave 根，使约束成为 SOC 可表。`BaseOptimalCalibration.calculate_regressor` 算 $R_i$、`sub_info_matrix` 分解 $X_i=R_i^\top R_i$；`calculate_detroot_whole` 算全池上界 $\mathrm{detroot}(R^\top R)/\sqrt{r}$。`SOCPOptimizer` 用 picos 建模、cvxopt 求解，选 $w_i>\varepsilon_{\text{opt}}=10^{-5}$ 的位姿，并断言不少于 `minNbChosen`。

```
function SOCPOptimizer.solve(subX_dict, calib_config):
    P = picos.Problem()
    w = picos.RealVariable("w", M, lower=0)        # w >= 0
    t = picos.RealVariable("t", 1)
    Mw = sum(w[i] * X_i for i in 0..M-1)          # 加权信息矩阵
    P.add_constraint(1 | w <= 1)                   # sum(w) <= 1
    P.add_constraint(t <= picos.DetRootN(Mw))       # SOC 约束
    P.set_objective("max", t)
    sol = P.solve(solver="cvxopt")
    return [float(w[i]) for i], sorted dict       # 选 w_i > 1e-5
```

### 4.2 DetMax 贪心交换（备选）

`Detmax` 用离散贪心 add/remove 交换求近优子集。准则函数 `get_critD(S)=\mathrm{detroot}(\sum_{i\in S}X_i)`（注意：不除 $\sqrt r$，与 `detroot_whole` 不同）。主循环收敛判据 `opt_k == rm_j`（最佳加入 = 最佳移除，无改进）。

```
function Detmax.main_algo(pool, nd):
    pool_idx = keys(pool)
    cur_set  = random.sample(pool_idx, nd)         # 随机初始化 nd 个
    opt_k = (pool - cur_set)[0] ; rm_j = cur_set[0]
    while opt_k != rm_j:                           # 收敛: 最佳加入==最佳移除
        # ADD: 在剩余池中找使 critD 最大的 k
        for k in (pool - cur_set):
            c = get_critD(cur_set + [k])
            if c > opt_critD: opt_critD=c; opt_k=k
        cur_set.append(opt_k)
        # REMOVE: 在 cur_set 中找移除后 critD 损失最小的 j
        for j in cur_set:
            d = opt_critD - get_critD(cur_set - {j})   # 移除造成的下降
            if d < delta_critD: delta_critD=d; rm_j=j
        cur_set.remove(rm_j)
        opt_critD = get_critD(cur_set); opt_critD_list.append(opt_critD)
    return opt_critD_list                          # 收敛历史
```

---

## 5. 复杂度

| 路径 | 主要成本 | 典型规模 |
|------|---------|---------|
| Fourier NLP/IPOPT 迭代 | $W_{\text{fun}}.\text{map}(N)$ + $\text{rnea}.\text{map}(N)$（OpenMP 并行）+ Cholesky logdet $O(r^3)$ | UR10 6-DOF、$N_h=5$、$N=200$：1–3 分钟 |
| 样条 NLP/IPOPT 迭代 | 目标 `cond(W_b)`：回归子 $O(N\cdot n_v\cdot n_{\text{param}})$ + SVD；雅可比有限差分 $=n_{\text{vars}}$ 次约束求值；× `stack_reps` 段 | 随 $n_{\text{wps}}$、$N$、`stack_reps` 增长 |
| 基参数索引 | 随机样条采样 + 回归子 $O(m\cdot n)$ + QR $O(m\,n^2)$ | $m=N_s\cdot n_v$、$n=10n_v$，对 $n$ 二次 |
| SOCP | cvxopt 内点法，约 $O(M\cdot r^3)$ | $M$ 候选位姿数 |
| DetMax | $O(\text{iters}\cdot M\cdot n_d)$，每次 `get_critD` 含 Cholesky $O(r^3)$ | 适合 $M$ 中等；对初始化敏感 |

Fourier 路径的总成本由 IPOPT 迭代数 $I$ 与每次 `.map` 求值主导：$O\big(I\cdot(N\cdot c_{\text{rnea}}+N\cdot c_{\text{reg}}+r^3)\big)$，内层 SX OpenMP 并行把 $N$ 维线性项摊到多核。

> 相关测试：[test_fourier_strategy](../../../tests/unit/test_fourier_strategy.py) · [test_fourier_e2e](../../../tests/unit/test_fourier_e2e.py) · [test_fourier_trajectory](../../../tests/unit/test_fourier_trajectory.py) · [test_config_fourier](../../../tests/unit/test_config_fourier.py)
