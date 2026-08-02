# backend 算法推导

> 模块：[src/figaroh/backend/](../../../src/figaroh/backend/) · 实现见 [code](code.md) · 唯一消费者 [optimal/algorithm](../optimal/algorithm.md)

---

## 1. 回归子 = RNEA 对参数的雅可比

逆动力学对惯性参数线性，故存在回归子 $H\in\mathbb{R}^{n_v\times 10n_v}$ 使 $\tau = H(q,\dot q,\ddot q)\,\pi_{\text{std}}$，且 $H = \partial\,\mathrm{RNEA}/\partial\pi$。完整推导见 [rnea-regressor §3](../algorithms/rnea-regressor.md)，此处不重复。

本模块用 `pinocchio.casadi`（下称 `cpin`）直接得到该雅可比的 **CasADi SX 符号表达式**：

- `W_expr = cpin.computeJointTorqueRegressor(cmodel, cdata, q, v, a)` —— Pinocchio C++ 层对 $\partial\mathrm{rnea}/\partial\pi$ 的高效解析实现，输入 SX 符号变量，返回 SX 表达式 $H$。
- `tau_expr = cpin.rnea(cmodel, cdata, q, v, a)` —— 力矩 $\tau$ 本身的 SX 表达式。

二者封装为 `cs.Function`，成为外层 NLP 可调用的符号算子。

---

## 2. CasADi 符号计算图与 SX/MX 混合自动微分

### 2.1 为什么分两层

Fourier NLP 的结构是「单步动力学 × $N$ 采样点 + 全局目标」：

- **单步动力学**（回归子、RNEA、logdet）输入输出固定、计算密集 → 用 **SX**。
- **轨迹层**（Fourier 系数 $Z$ → $N$ 点 $Q/V/A$ → 信息矩阵 → 目标）决策变量多、需对 $Z$ 自动微分、构成 NLP → 用 **MX**。

### 2.2 内层 SX Function

```
# 单步，输入输出维度固定，构建一次、跨 solve 复用
cs_q = cs.SX.sym("q", nq); cs_v = cs.SX.sym("v", nv); cs_a = cs.SX.sym("a", nv)

W_expr   = cpin.computeJointTorqueRegressor(cmodel, cdata, cs_q, cs_v, cs_a)   # (nv, 10*nv)
tau_expr = cpin.rnea(cmodel, cdata, cs_q, cs_v, cs_a)                         # (nv,)

W_fun   = cs.Function("W",   [cs_q, cs_v, cs_a], [W_expr])      # SX Function
rnea_fun = cs.Function("rnea", [cs_q, cs_v, cs_a], [tau_expr])  # SX Function
```

**为什么用 SX：**

1. **单步动力学**——SX 适合固定规模、无循环展开的局部分支；$n_v\times 10n_v$ 规模适中，SX 的稀疏性已足够。
2. **`cs.chol` 支持**——`cs.chol` 仅接受 DM/SX，不支持 MX（见 §2.5 logdet）。

### 2.3 外层 MX

```
Z = cs.MX.sym("coeffs", n_vars)          # 决策变量：Fourier 系数
# 由 Z 经 sin/cos 构造 N 点 Q/V/A —— 全是 MX 表达式
Q_full, V_full, A_full = build_trajectory(Z, ...)   # (nq, N), (nv, N), (nv, N) MX
```

**为什么用 MX：**

1. **自动微分**——MX 图对 $Z$ 做 reverse-mode AD，得到 $\nabla_Z f$ 与约束雅可比，供 IPOPT 求解。若用 SX 直接展开 $N$ 点，表达式树爆炸。
2. **NLP 语义**——`cs.nlpsol("ipopt", {x:Z, f:obj, g:cons})` 的决策变量必须是 MX。

### 2.4 `.map(N, "openmp")`：SX 嵌入 MX 并行求值

`W_fun.map(n_samples, "openmp")` 把单步 SX Function 复制 $N$ 份并行求值，返回的仍是一个可被 MX 图调用的 Function：

```
W_map_fun = W_fun.map(n_samples, "openmp")
W_raw = W_map_fun(Q_full, V_full, A_full)   # MX 调用，输出 (nv, N*n_param)
W_full = vertcat([W_raw[:, i*P:(i+1)*P] for i in range(N)])   # (N*nv, n_param) MX
```

**CasADi AD 如何穿透 SX Function**：在外层 MX 图中，`W_fun`（SX Function）被视为一个**可微的复合算子**。CasADi 对 MX 做 reverse-mode AD 时，遇到 SX Function 调用节点，会自动展开其内部 SX 表达式树（即 `computeJointTorqueRegressor` 的解析式）参与 AD——等价于把 SX 子图内联进 MX 图。因此无需手写 $W$ 对 $Z$ 的雅可比，IPOPT 所需梯度由 AD 自动穿透得到。`rnea_fun.map(...)` 同理。

### 2.5 logdet_fn：为何把 MX 的 $J_{\text{reg}}$ 包成 SX Function

D-最优目标 $f = -\log\det(J_{\text{reg}})$，其中 $J_{\text{reg}} = W_b^\top W_b/N + \lambda I$ 是 MX 表达式（依赖决策变量 $Z$）。用 Cholesky 计算 $\log\det$：

$$
L = \mathrm{chol}(J_{\text{reg}}),\qquad \log\det = 2\sum_i \log L_{ii}
$$

但 **`cs.chol` 不支持 MX**（仅 DM/SX）。解法是把 Cholesky-logdet 包进一个 SX Function，再从 MX 图调用：

```
J_sx = cs.SX.sym("J_reg", n_base, n_base)    # 抽象输入占位
L_sx = cs.chol(J_sx)                          # SX 层做 Cholesky（可行）
logdet_sx = 2 * cs.sum1(cs.log(cs.diag(L_sx)))
logdet_fn = cs.Function("logdet", [J_sx], [logdet_sx])   # SX Function

obj = -logdet_fn(J_reg)    # MX 调用：把 MX 的 J_reg 喂给 SX Function
```

这复用了 §2.4 的同一穿透机制：`logdet_fn` 内部用 SX 做 `chol`，被 MX 调用时 CasADi AD 自动展开其 SX 子图，使 $\log\det$ 对 $Z$ 可微。详见 [d-optimal-design §3](../algorithms/d-optimal-design.md)。

### 2.6 `_ensure_symbolic_model` 伪代码

```
function _ensure_symbolic_model(self):
    if self._cmodel is not None:           # 已构造，幂等
        return
    _lazy_import()                          # 首次导入 cs / cpin
    self._cmodel = cpin.Model(self._robot.model)
    self._cdata  = self._cmodel.createData()
    cs_q = cs.SX.sym("q", self._cmodel.nq)
    cs_v = cs.SX.sym("v", self._cmodel.nv)
    cs_a = cs.SX.sym("a", self._cmodel.nv)
    W_expr   = cpin.computeJointTorqueRegressor(self._cmodel, self._cdata, cs_q, cs_v, cs_a)
    tau_expr = cpin.rnea(self._cmodel, self._cdata, cs_q, cs_v, cs_a)
    self._W_fun    = cs.Function("W",   [cs_q,cs_v,cs_a], [W_expr])
    self._rnea_fun = cs.Function("rnea", [cs_q,cs_v,cs_a], [tau_expr])
```

---

## 3. 惰性初始化与惰性导入

**惰性初始化**：`__init__` 仅存 `self._robot`，`_cmodel/_W_fun/_rnea_fun = None`。符号模型在首次访问 `regressor_function` / `rnea_function` 属性（或外部直接调 `_ensure_symbolic_model()`）时才构建。`cpin.computeJointTorqueRegressor` 和 `cpin.rnea` 是 C++ 原生实现，`cs.Function` 构造开销在毫秒级，无需磁盘缓存。这使 spline 用户（不需要后端）零开销。

**惰性导入** `_lazy_import()`：模块级 `cpin`、`cs` 初始为 `None`，分两条独立 `try/except`：

- `cs` 失败 → 提示装 `casadi`。
- `cpin` 失败 → 进一步探测 `pinocchio.__version__` 是否存在：存在说明装的是 PyPI `pin`（无 CasADi 绑定），提示卸载 `pin` 换 conda-forge `pinocchio`；不存在则提示直接装 conda-forge。

两条独立 `try` 使测试可 mock 一边、用真另一边。

---

## 4. 复杂度

| 操作 | 复杂度 / 耗时 | 说明 |
|------|--------------|------|
| 符号模型构建 | $O(\text{C++ native})\approx 0.05\,\text{s}$ | `computeJointTorqueRegressor` 为 Pinocchio C++ 解析实现，`cs.Function` 构造为毫秒级 |
| `.map(N,"openmp")` 求值 | $O(N/\text{cores})$ | $N$ 采样点并行，SX Function 单步固定规模 |
| 外层 MX AD | 与 $N\cdot n_v$ 成正比 | reverse-mode，输出少输入多时高效 |

> 测试见 [tests/unit/test_backend.py](../../../tests/unit/test_backend.py)
