# RNEA 与回归子

> 跨模块算法基础。被 [identification](../identification/algorithm.md)、[tools](../tools/algorithm.md)、[backend](../backend/algorithm.md)、[optimal](../optimal/algorithm.md) 引用。

---

## 1. 问题与记号

刚体机械臂的正动力学（前向）与逆动力学（后向）。辨识与轨迹优化关心的是**逆动力学**：

$$
\tau = \mathrm{RNEA}(q,\dot q,\ddot q,\pi)
$$

- $q,\dot q,\ddot q\in\mathbb{R}^{n_v}$：广义位置/速度/加速度（$n_v$ = 机器人自由度数 `model.nv`）。
- $\pi$：待辨识的**惯性参数向量**。
- $\tau\in\mathbb{R}^{n_v}$：关节力矩。
- RNEA = Recursive Newton–Euler Algorithm（Pinocchio 的 `pin.rnea`），$O(n_v)$ 递推。

**关键事实**：逆动力学对惯性参数是**线性**的。这正是辨识与最优实验设计的数学根基。

---

## 2. 每连杆 10 个 barycentric 惯量参数

每个连杆 $i$ 的惯性由 10 个参数刻画（Pinocchio 顺序）：

$$
\pi_i = \big(m,\; m c_x,\; m c_y,\; m c_z,\; I_{xx},\; I_{xy},\; I_{yy},\; I_{xz},\; I_{yz},\; I_{zz}\big)^\top
$$

即 $[m,\; m\boldsymbol{c},\; \mathrm{vec}(\Sigma)]$，其中 $\boldsymbol{c}$ 为质心（相对连杆 frame），$\Sigma$ 为绕质心的 $3\times3$ 对称惯性张量。Pinocchio 的 `inertia.toDynamicParameters()` 恰返回这 10 维向量。

全机器人标准参数向量（$n_v$ 个活动连杆）：

$$
\pi_{\text{std}} = [\pi_1;\;\pi_2;\;\dots;\;\pi_{n_v}]\in\mathbb{R}^{10 n_v}
$$

> ⚠️ **Pinocchio 顺序 vs 经典 barycentric 顺序**：经典文献常用 $[I_{xx},I_{xy},I_{xz},I_{yy},I_{yz},I_{zz}, m c_x, m c_y, m c_z, m]$。本工程 `identification/parameter.py` 中 `reorder_inertial_parameters` 本欲重排，但**被注释掉**，实际保留 Pinocchio 顺序。回归矩阵列序必须与 $\pi_{\text{std}}$ 一致。

---

## 3. 回归子 = 逆动力学对参数的雅可比

由线性性，存在**回归子矩阵** $H\in\mathbb{R}^{n_v\times 10n_v}$，使得：

$$
\tau = H(q,\dot q,\ddot q)\,\pi_{\text{std}}
$$

且

$$
H(q,\dot q,\ddot q) = \frac{\partial\,\mathrm{RNEA}(q,\dot q,\ddot q,\pi)}{\partial \pi}\bigg|_{\pi}
$$

即回归子是 RNEA 对参数向量的雅可比。**它不是另一套计算**，而是对逆动力学做关于参数的自动微分。

### 3.1 CasADi 符号实现（为什么能得到解析回归子）

在 [backend/CasadiBackend](../backend/algorithm.md) 中：

1. `cpin.Model(robot.model)` 构造 `pinocchio.casadi` 符号模型；
2. 建符号变量 `q=cs.SX.sym("q",nq)`、`v`、`a`；
3. `W_expr = cpin.computeJointTorqueRegressor(cmodel, cdata, q, v, a)` —— 这是 Pinocchio C++ 层对 $\partial\mathrm{rnea}/\partial\pi$ 的高效解析实现，返回 SX 表达式；
4. `rnea_expr = cpin.rnea(cmodel, cdata, q, v, a)` —— 力矩本身；
5. 封装为 `cs.Function`（缓存到磁盘）。

外层 Fourier NLP 把 `q,v,a` 构造成 MX（决策变量的函数），通过 `W_fun.map(N,"openmp")` 并行求值 $N$ 个采样点，CasADi 自动微分穿透 SX Function。

### 3.2 数值实现（tools/RegressorBuilder）

[tools/regressor.py](../../../src/figaroh/tools/regressor.py) 逐采样点调用 `pin.computeJointTorqueRegressor(model, data, q[i], v[i], a[i])`，按行 `base_idx = j·N + i` 堆叠：

$$
W = \begin{bmatrix} H(q_1,\dot q_1,\ddot q_1) \\ H(q_2,\dot q_2,\ddot q_2) \\ \vdots \\ H(q_N,\dot q_N,\ddot q_N) \end{bmatrix}\in\mathbb{R}^{N n_v\times 10 n_v},\qquad
\tau_{\text{vec}} = \begin{bmatrix}\tau_1\\\vdots\\\tau_N\end{bmatrix}\in\mathbb{R}^{N n_v}
$$

满足 $\tau_{\text{vec}} = W\,\pi_{\text{std}}$。

---

## 4. 扩展动力学模型（摩擦 / 驱动器惯量 / 关节偏置）

实际机器人除刚体惯性外，关节还有摩擦、电机惯量、零位偏置。这些项同样对参数线性，故可**追加列**到回归子：

$$
\tau_j = \underbrace{H_j\,\pi_{\text{std}}}_{\text{刚体}} + \underbrace{f_{v,j}\,\dot q_j}_{\text{粘性摩擦}} + \underbrace{f_{s,j}\,\mathrm{sgn}(\dot q_j)}_{\text{Coulomb}} + \underbrace{I_{a,j}\,\ddot q_j}_{\text{电机惯量}} + \underbrace{o_j}_{\text{零位偏置}}
$$

扩展参数向量：

$$
\pi_{\text{ext}} = [\,\pi_{\text{std}}\;(10n_v);\; \{f_v\}(n_v);\; \{f_s\}(n_v);\; \{I_a\}(n_v);\; \{o\}(n_v)\,]
$$

扩展回归子 $W_{\text{ext}} = [W \;|\; \Phi_{\text{fric}}\; \Phi_{\text{ia}}\; \Phi_{\text{off}}]$，其中附加列在样本 $i$、关节 $j$ 处取值：

| 参数 | 附加列值 | 说明 |
|------|---------|------|
| $f_{v,j}$ | $\dot q_j(i)$ | 粘性摩擦，线性于速度 |
| $f_{s,j}$ | $\mathrm{sgn}(\dot q_j(i))$ | Coulomb（**代码实现**）；spec 要求 $\tanh(\alpha\,\dot q_j)$ 可微近似 |
| $I_{a,j}$ | $\ddot q_j(i)$ | 电机转子惯量 |
| $o_j$ | $1$ | 常值零位偏置 |

> ⚠️ [unified-friction-model spec](../../../openspec/specs/unified-friction-model/spec.md) 要求 Coulomb 用 $\tanh(\alpha\,\dot q)$（$\alpha\approx100$）以保证可微；但 [regressor.py](../../../src/figaroh/tools/regressor.py) 辨识侧仍用 $\mathrm{sgn}$，Fourier 优化侧用 $\tanh$。两者不一致是已知 spec 偏离。

开关由 `identif_config` 的 `has_friction`/`has_actuator_inertia`/`has_joint_offset` 控制，附加列数 `additional_columns = 2·has_friction + has_actuator_inertia + has_joint_offset`。

---

## 5. 由回归子到辨识 / 最优设计

得到 $\tau_{\text{vec}} = W\,\pi$（$W$ 列数 = 参数数）后，两条下游路径：

- **辨识**（参数估计）：超定最小二乘 $\hat\pi = (W^\top W)^{-1}W^\top\tau$。但 $W$ 常列秩亏，需先做 [基参数约简](base-parameter-reduction.md)。
- **最优实验设计**：选择轨迹使 $W$ 信息量最大化 → [D-最优](d-optimal-design.md)。

---

## 6. 伪代码

```
# 输入: robot (pinocchio model), 采样序列 q[N,nq], v[N,nv], a[N,nv], identif_config
# 输出: 全参数回归子 W ∈ R^{N*nv, n_param}, 参数名序列

function build_regressor(robot, q, v, a, cfg):
    nv = robot.model.nv
    add = 2*cfg.has_friction + cfg.has_actuator_inertia + cfg.has_joint_offset
    n_param = (10 + add) * nv
    W = zeros(N*nv, n_param)
    for i in 0..N-1:
        H = pin.computeJointTorqueRegressor(robot.model, robot.data, q[i], v[i], a[i])  # (nv, 10*nv)
        for j in 0..nv-1:                       # 关节主序堆叠
            W[j*N + i, 0:10*nv] = H[j, :]
        p = 10*nv                                # 附加列起点
        if cfg.has_friction:
            for j: W[j*N+i, p+j]        = v[i,j]            # f_v
                    W[j*N+i, p+nv+j]    = sign(v[i,j])      # f_s  (代码; spec=tanh)
            p += 2*nv
        if cfg.has_actuator_inertia:
            for j: W[j*N+i, p+j] = a[i,j]                   # I_a
            p += nv
        if cfg.has_joint_offset:
            for j: W[j*N+i, p+j] = 1.0                       # off
    return W
```

CasADi 符号版见 [backend/algorithm](../backend/algorithm.md)：内层 SX `computeJointTorqueRegressor`，外层 MX `W_fun.map(N,"openmp")` 并行求值。
