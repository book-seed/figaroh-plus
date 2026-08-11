# utils 模块算法

> 模块：[src/figaroh/utils/](../../../src/figaroh/utils/) · 实现见 [code](code.md) · 基础见 [RNEA 回归子](../algorithms/rnea-regressor.md)

---

## 0. 概览

`utils` 提供三套参数化数学，下游见 [optimal](../optimal/algorithm.md)（Fourier 用于 NLP 激励轨迹）、[identification](../identification/algorithm.md)（回归子消费 q,v,a）、[calibration](../calibration/algorithm.md)：

1. **三次样条**（`cubic_spline.py`，基于 [ndcurves](https://github.com/humanoid-path-planner/ndcurves) `exact_cubic` + `curve_constraints`）；
2. **Fourier 级数**（`fourier_trajectory.py`，解析导数内联 + CasADi SX 位置表达式）；
3. **统一配置系统**（`config_parser.py`，模板继承 + 变体 + 任务继承 + 变量展开）。

力矩计算走 [pin_interface](../../../src/figaroh/utils/pin_interface.py) 的 `calc_torque`，列主序与 [RNEA 回归子](../algorithms/rnea-regressor.md)的行序 `base_idx=j·N+i` 对齐。

---

## 1. 三次样条参数化（核心）

### 1.1 分段三次 Hermite / 样条基

给定航点 $\{p_i\}_{i=0}^{M-1}\subset\mathbb{R}^{n_a}$（$n_a$ 个活跃关节）与时间戳 $\{t_i\}$，在每段 $[t_i,t_{i+1}]$ 上构造三次多项式

$$
p_i(t)=c_{i,0}+c_{i,1}(t-t_i)+c_{i,2}(t-t_i)^2+c_{i,3}(t-t_i)^3,\qquad t\in[t_i,t_{i+1}].
$$

单段 4 个自由度。**三次 Hermite 段**用两端的位置与速度唯一确定：

$$
p_i(t_i)=p_i,\quad p_i(t_{i+1})=p_{i+1},\quad p_i'(t_i)=v_i,\quad p_i'(t_{i+1})=v_{i+1}.
$$

4 条件 = 4 自由度，故 Hermite 段唯一。但 Hermite 只保证 $C^1$（位置+速度连续），不保证 $C^2$（加速度连续）。

### 1.2 为何 `exact_cubic` 在航点处满足 $C^2$ 连续

**三次样条**（clamped / 完全三次样条）的关键：不是为每段独立选 Hermite 速度，而是**全局求解**速度 $\{v_i\}$，使相邻段在内部航点处加速度也连续。

设 $M$ 个航点、$M-1$ 段，共 $4(M-1)$ 个系数。约束方程：

| 类别 | 方程数 | 内容 |
|------|--------|------|
| 插值 | $2(M-1)$ | 每段两端位置命中航点 |
| $C^1$ 连续 | $M-2$ | 内部航点处相邻段一阶导相等 |
| $C^2$ 连续 | $M-2$ | 内部航点处相邻段二阶导相等 |
| 边界 | $2$ | 钳制（clamped）：端点速度 $\dot p(t_0),\dot p(t_{M-1})$ |

合计 $2(M-1)+(M-2)+(M-2)+2=4(M-2)+2+2=4M-4=4(M-1)$，与自由度一致 → 唯一解（三对角系统，$O(M)$ 解）。

`ndcurves.exact_cubic(waypoints, time_points, constraints)` 即实现此：返回精确穿过每个航点的分段三次曲线，在内部航点 $C^2$ 连续，并由 `curve_constraints` 的 `init_vel`/`end_vel`/`init_acc`/`end_acc` 施加钳制边界条件。

### 1.3 分段组装（FIGAROH 实现）

[cubic_spline.py](../../../src/figaroh/utils/cubic_spline.py) 的 `get_active_config` **逐航点对**构造 `exact_cubic` 并 `append` 进 `ndcurves.piecewise`：

```
for i in 0..M-2:                      # 每段 2 个航点
    c = ndcurves.curve_constraints()
    c.init_vel = vel_wp[:, i]          # 段起点速度
    c.end_vel  = vel_wp[:, i+1]       # 段终点速度
    c.init_acc = acc_wp[:, i]         # 段起点加速度
    c.end_acc  = acc_wp[:, i+1]       # 段终点加速度
    ec = ndcurves.exact_cubic(wp[:, i:i+2], tp[i:i+2], c)
    pc.append(ec)
```

**$C^2$ 连续性来源**：段 $i$ 的 `end_vel`/`end_acc` 与段 $i{+}1$ 的 `init_vel`/`init_acc` 取自**同一**用户值 `vel_wp[:, i+1]` / `acc_wp[:, i+1]`，故在拼接处位置（插值命中）、速度、加速度三者一致 → $C^2$ 连续。无 `vel/acc_waypoints` 时退化为自然样条（边界速度/加速度由 ndcurves 默认）。

### 1.4 由 freq 推 $N$

```
delta_t = 1 / freq
T       = pc.max() - pc.min()         # 总行程时间
N       = int(T / delta_t) + 1        # 采样点数
t       = linspace(pc.min(), pc.max(), N)
q   = [pc(t[i])        for i in 0..N-1]
dq  = [pc.derivate(t[i], 1) for i in 0..N-1]   # 一阶导
ddq = [pc.derivate(t[i], 2) for i in 0..N-1]   # 二阶导
```

`pc.derivate(t, order)` 返回曲线在 $t$ 处的 `order` 阶导数，故 `order=1/2` 直接给出速度/加速度。

### 1.5 `soft_lim` 收缩限值与 $\ddot q = k\cdot\dot q$

模块级常量 $k=1.5$（`cubic_spline.py` 顶部）。加速度限值取速度限值的 $1.5$ 倍：

$$
\ddot q_{\max}=k\,\dot q_{\max},\qquad \ddot q_{\min}=k\,\dot q_{\min},\qquad k=1.5.
$$

`soft_lim_pool`（shape $(3,n_a)$）按行分别收缩 **位置 / 速度 / 力矩** 限值。以位置为例，记原始区间 $\Delta q=|q_{\max}-q_{\min}|$、收缩因子 $s_j=$ `soft_lim_pool[0,j]`：

$$
q_{\max}'=q_{\max}-s_j\,\Delta q,\qquad q_{\min}'=q_{\min}+s_j\,\Delta q.
$$

即向区间中心对称收缩 $s_j$ 比例。速度（行 1）、力矩（行 2）同理；收缩后 $\ddot q$ 限值由新 $\dot q$ 限值再乘 $k$ 重算。

---

## 2. Fourier 级数参数化（核心）

### 2.1 公式（速度参数化）

对关节 $j$，周期 $T$，基频 $\omega=2\pi/T$，$N_h$ 阶谐波。**速度**为首要参数化对象（避免 $(k\omega)^2$ 加速度放大）：

$$
\dot q_j(t)=\sum_{k=1}^{N_h}\big[a_{k,j}\sin(k\omega t)+b_{k,j}\cos(k\omega t)\big].
$$

位置通过解析积分得到（$a_{0,j}$ 为均值位置）：

$$
q_j(t)=a_{0,j}+\int_0^t \dot q_j(\tau)\,d\tau
     =a_{0,j}+\sum_{k=1}^{N_h}\Big[-\frac{a_{k,j}}{k\omega}\cos(k\omega t)+\frac{b_{k,j}}{k\omega}\sin(k\omega t)\Big].
$$

### 2.2 解析加速度

对速度求导得加速度（$a_{0,j}$ 不参与，无 DC 速度分量保证周期性）：

$$
\ddot q_j(t)=\frac{d}{dt}\dot q_j(t)=\sum_{k=1}^{N_h}\big[a_{k,j}\,k\omega\,\cos(k\omega t)-b_{k,j}\,k\omega\,\sin(k\omega t)\big].
$$

**相比位置参数化的优势**：加速度幅值 $\propto k\omega$ 而非 $(k\omega)^2$，高频谐波的 IPOPT 梯度贡献更均衡，避免高频分量在优化中"失活"。位置参数化中 $\ddot q$ 含 $(k\omega)^2$ 放大，使高频系数对力矩约束极度敏感。

### 2.3 系数布局（列主序）

每关节系数数 `n_coeffs_per_joint = 1 + 2·N_h`，按列主序存于 `coeffs[j, :]`：

$$
\texttt{coeffs[j,0]}=a_0\;\text{(均值位置)},\quad \texttt{coeffs[j,2k-1]}=a_k\;\text{(速度 sin 幅值)},\quad \texttt{coeffs[j,2k]}=b_k\;\text{(速度 cos 幅值)},\quad k=1,\dots,N_h.
$$

即 $[a_0,\,a_1,b_1,\,a_2,b_2,\,\dots,\,a_{N_h},b_{N_h}]$。`_evaluate` 据此索引取 $a_k=\texttt{coeffs[j,2k-1]}$、$b_k=\texttt{coeffs[j,2k]}$。

### 2.4 基频 $\omega=2\pi/T$

构造时 $\omega$ 缺省取 $2\pi/T$；若显式传 `omega` 则覆盖。谐波频率为 $k\omega$（$k=1,\dots,N_h$）。**Fourier NLP** 中 $T=2\pi/\omega$（始终覆盖恰好一个周期），$\omega$ 由 `fourier_frequency` 显式指定或默认 $\omega = 2\pi/T_{\text{traj}}$。

### 2.5 为何解析内联而非 `cs.jacobian`

spec 文本要求 Fourier 轨迹支持 jacobian 求导，但 [fourier_trajectory.py](../../../src/figaroh/utils/fourier_trajectory.py) 的 `_evaluate` **直接内联解析公式**（上式）计算 $q,v,a$，而不在运行时调 `cs.jacobian`。原因：

- 解析公式 $O(N_h)$ 每采样点，比符号自动微分开销低、无 CasADi 运行时依赖；
- 速度和加速度由速度参数化直接给出（$v$ 即级数本身，$a$ 求导一次），比位置参数化少一次求导，结构更紧凑。

**验证方式**：测试 [test_fourier_trajectory.py](../../../tests/unit/test_fourier_trajectory.py) 用 `cs.jacobian`（对 `build_casadi_expression` 产出的 SX $q$ 求 $\partial q/\partial t$、$\partial^2 q/\partial t^2$）与 numpy `_evaluate` 结果比对，即"实现解析、测试用 jacobian 验证"。

### 2.6 `build_casadi_expression`（仅构造 $q$，速度参数化）

此方法以**标量** CasADi SX `t_sym` 与系数符号 `coeffs_sym` 构造 $q_j(t)$ 的 SX 表达式，**只构造位置**，不构造 $v,a$：

$$
q_j^{\text{SX}}(t)=\texttt{coeffs\_sym[j,0]}+\sum_{k=1}^{N_h}\Big[-\frac{\texttt{coeffs\_sym[j,2k-1]}}{k\omega}\cos(k\omega t)+\frac{\texttt{coeffs\_sym[j,2k]}}{k\omega}\sin(k\omega t)\Big].
$$

它**未被** [Fourier NLP 策略](../optimal/algorithm.md) 调用——后者自行向量化构造 q/v/a（对时间向量 `t_vec`），因此 `build_casadi_expression` 仅服务于测试（§2.5 的 jacobian 验证）。

### 2.7 伪代码

```
# _evaluate(t, coeffs) -> (q, v, a)  —— 速度参数化
function _evaluate(t, coeffs):
    N = len(t); n_act = coeffs.shape[0]; n_h = N_h; omega = self._omega
    q = zeros(N, n_act); v = zeros(N, n_act); a = zeros(N, n_act)
    for j in 0..n_act-1:
        q[:,j] = coeffs[j,0]                       # a0 = 均值位置
        for k in 1..n_h:
            ak = coeffs[j, 2k-1]; bk = coeffs[j, 2k]  # 速度 sin/cos 幅值
            kw = k * omega
            s = sin(kw * t); c = cos(kw * t)
            # v(t) = ak*sin(kωt) + bk*cos(kωt)
            v[:,j] += ak*s + bk*c
            # q(t) = a0 - ak/(kω)*cos(kωt) + bk/(kω)*sin(kωt)
            q[:,j] += -ak/kw*c + bk/kw*s
            # a(t) = ak*kω*cos(kωt) - bk*kω*sin(kωt)
            a[:,j] += ak*kw*c - bk*kw*s
    return q, v, a
```

```
# build_casadi_expression(t_sym, coeffs_sym, omega?) -> SX(n_act, 1)
function build_casadi_expression(t_sym, coeffs_sym, omega):
    n_act = coeffs_sym.shape[0]; w = omega or self.omega
    q_expr = cs.SX.zeros(n_act, 1)
    for j in 0..n_act-1:
        q_j = coeffs_sym[j,0]                       # a0 = 均值位置
        for k in 1..N_h:
            ak = coeffs_sym[j, 2k-1]; bk = coeffs_sym[j, 2k]  # 速度 sin/cos 幅值
            kw = k * w
            # q = a0 - ak/(kω)*cos + bk/(kω)*sin
            q_j += -ak/kw*cs.cos(kw*t_sym) + bk/kw*cs.sin(kw*t_sym)
        q_expr[j] = q_j
    return q_expr
```

---

## 3. 统一配置系统（完整推导）

### 3.1 模板继承 `extends`

配置可声明 `extends: <template_ref>`。`_resolve_inheritance` 弹出 `extends`，经 `_resolve_template_path` 定位父模板，递归解析父模板自身的 `extends`（带缓存 `_template_cache`），最后 `_deep_merge(父, 子)`——**子覆盖父**。

### 3.2 `_resolve_template_path` 的 4 种形式

| 形式 | 判定 | 解析 |
|------|------|------|
| 绝对路径 | `Path(ref).is_absolute()` | 直接用 |
| `templates/...` 前缀 | `startswith("templates/")` | 在 4 个候选目录（config 同级 `templates/`、父级、祖父级、`cwd/templates/`）中搜索 |
| `../templates/...` 相对 | 非 absolute、非 `templates/` 前缀 | 相对 `config_path.parent` |
| `filename.yaml` 相对 | 同上 else 分支 | 相对 `config_path.parent` |

找不到则抛 `ConfigurationError`。

### 3.3 `_deep_merge(base, override)`

深合并：`base` 深拷贝为结果，遍历 `override`：

- 同键且双方均 `dict` → 递归 `_deep_merge`；
- 同键且双方均 `list` → **默认替换**；但若键名以 `_extend` 结尾，则去掉后缀（`real_key = key[:-8]`），把 override 列表 **extend** 进 base 的 `real_key` 列表（触发"列表扩展"语义）；
- 否则 → 用 override 值替换；
- 新键 → 直接加入。

### 3.4 变体 `variants` 与 `_apply_variant`

`_apply_variant(config, name)` 取 `config["variants"][name]`（深拷贝）。若变体自身声明 `extends: <点号路径>`，弹出后用 `_get_nested_value(config, path)`（点号分割逐层取值，如 `tasks.calib.parameters`）取基配置，`_deep_merge(基, 变体)`；随后把变体 `_deep_merge` 进整个 config 的拷贝，并移除 `variants` 段。

### 3.5 任务继承与拓扑排序

`tasks.<name>` 可声明 `inherits_from: <parent_task>`。`_resolve_task_inheritance` 用 DFS 拓扑排序解析依赖：先解析父任务，再 `_deep_merge(父, 子)`（子弹出 `inherits_from`）。

```
# 拓扑排序（带环检测）
function resolve_task(name, visited, temp):
    if name in temp:               # 回边 → 环
        raise ConfigurationError("Circular task inheritance: " + name)
    if name in visited: return
    temp.add(name)
    task = tasks[name]
    if "inherits_from" in task:
        parent = task["inherits_from"]
        if parent not in tasks: raise ConfigurationError("Parent task not found")
        resolve_task(parent, visited, temp)            # 先父后子
        merged = deep_merge(resolved[parent], task - {inherits_from})
        resolved[name] = merged
    else:
        resolved[name] = deepcopy(task)
    temp.remove(name); visited.add(name)

visited = {}
for name in tasks: resolve_task(name, visited, set())   # 每个任务独立 temp 集
```

`temp_visited` 集合检测回边（环）并抛 `ConfigurationError`。

### 3.6 变量展开 `${VAR}`

正则 `\$\{([^}]+)\}` 匹配 `${...}`，对每个变量名按**优先级**解析：

1. **环境变量** `os.environ.get(name)`；
2. **config 嵌套** `_get_nested_value(config, name)`（点号路径，可引用配置内任意值）；
3. **缓存** `_variable_cache.get(name)`；
4. 都失败 → **保留原值** `${VAR}` 并告警。

递归遍历 dict / list，逐字符串值替换。

### 3.7 校验 `_validate_task_type`

| 任务类型 | 必需 section |
|---------|-------------|
| `kinematic_calibration` | `kinematics`、`measurements` |
| `dynamic_identification` | `problem`、`signal_processing` |

缺任一则抛 `ConfigurationError`。`_is_compatible_version` 接受主版本号 $\le 2$（`1.x`/`2.x`）。

### 3.8 遗留格式探测 `is_unified_config`

读取文件，若含任一**指示键**（`_metadata`/`schema_version`/`extends`/`variants`/`tasks`/`common`/`variables`），或任一值为含 `inherits_from` 的 dict，则判为统一格式；否则遗留。`get_param_from_yaml` 据此分流：文件路径 → `parse_configuration`；含 `tasks` 的 dict → 统一；否则 `_parse_legacy_format`（按 `markers`/`calib_level` 等指示键判定 calibration/identification，委派对应模块的遗留 parser）。

### 3.9 伪代码

```
# parse() 主流程
function parse():
    config = _load_config_file(config_path)
    if "extends" in config: config = _resolve_inheritance(config)
    if variant:             config = _apply_variant(config, variant)
    config = _resolve_task_inheritance(config)
    config = _expand_variables(config)
    _validate_configuration(config)
    config["_metadata"] = ConfigMetadata(source_file, variant, cwd)
    return config
```

```
# _deep_merge(base, override) -> dict
function _deep_merge(base, override):
    result = deepcopy(base)
    for key, value in override.items():
        if key in result:
            if dict(result[key]) and dict(value):
                result[key] = _deep_merge(result[key], value)
            elif list(result[key]) and list(value):
                if key.endswith("_extend"):
                    real = key[:-8]
                    result[real] = (result[real] + value) if real in result else value
                else:
                    result[key] = deepcopy(value)        # 默认替换
            else:
                result[key] = deepcopy(value)
        else:
            result[key] = deepcopy(value)
    return result
```

---

## 4. `pin_interface.calc_torque`

[init_robot](../../../src/figaroh/utils/pin_interface.py) 调 `pin.framesForwardKinematics` + `pin.updateFramePlacements` 初始化 frame 位姿。

`calc_torque(N, robot, q, v, a)` 逐采样点调 `pin.rnea`，但按**列主序**（关节主序）扁平化：

$$
\texttt{tau}[j\cdot N + i] = \mathrm{RNEA}(q_i,\dot q_i,\ddot q_i)[j],\qquad \texttt{tau}\in\mathbb{R}^{n_v\cdot N}.
$$

即关节 $j$ 的全部 $N$ 个采样值连续存放于 $[jN,\,(j+1)N)$。这与 [RNEA 回归子](../algorithms/rnea-regressor.md) §3.2 的行序 `base_idx=j·N+i` 一致，可直接拼回归子。**消费方**需将其 reshape 为 $(n_v, N)$ 再转置得 $(N, n_v)$（见 [code](code.md) §3.2 `CubicSpline.compute_torques` 的 `reshape(nv,N).T`）。注意 `FourierTrajectory.compute_torques` 直接循环 `pinocchio.rnea` 返回 $(N, n_v)$，与 `calc_torque` 的列主序约定不一致。

---

## 5. 结果管理（`ResultsManager`）

四类绘图，由 `task_type` 选 `PLOT_STYLES`（figsize/dpi）：

| 方法 | 内容 |
|------|------|
| `plot_calibration_results` | 位姿比较 + 残差 + 参数值（GridSpec 3×2） |
| `plot_identification_results` | 力矩比较（实测 vs 估计）+ 残差 |
| `plot_optimal_calibration_results` | 配置权重 + 条件数演化 + 信息矩阵热图 |
| `plot_optimal_trajectory_results` | 各关节 pos/vel/acc 三列多行（多段） |

`save_results` 支持 `yaml/csv/json/npz/pkl`，经 `_convert_for_serialization` 将 `ndarray`→`list`、`np.integer`/`np.floating`→`float`、`np.bool_`→`bool`；`pkl` 先尝试原对象 pickle，失败回退到转换后表示。唯一落盘绘图是 `plot_optimal_trajectory_results`（`fig.savefig(...)`），其余仅 `plt.show()`。

---

## 6. 复杂度

| 操作 | 复杂度 | 说明 |
|------|--------|------|
| 样条求值（单段） | $O(N)$ | $N$ 采样点，每点 $O(1)$ 多项式求值 |
| 样条系数求解 | $O(M)$ | $M$ 航点，三对角系统 |
| Fourier 求值 | $O(N\cdot n_a\cdot N_h)$ | 每关节每谐波独立 |
| `calc_torque` | $O(N\cdot n_v)$ | RNEA 本身 $O(n_v)$，逐采样点 |
| 配置解析 | $O(T+V+E)$ | $T$ 任务拓扑排序、$V$ 变量展开、$E$ extends 递归（带缓存） |
| `_deep_merge` | $O(\|K\|)$ | 键数线性 |
