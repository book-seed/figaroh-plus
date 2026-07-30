# FIGAROH Wiki

> **F**ree dynamics **I**dentification and **G**eometrical c**A**libration of **RO**bot and **H**uman
>
> 仓库根：`/home/tyche/Documents/figaroh-plus` · 版本 0.4.3 · 分支 `pixi`

FIGAROH 是一个基于 [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 的 Python 工具箱，提供刚体多体系统的**动力学辨识**与**几何标定**框架，支持串联（工业机械臂）与树结构（人形、移动机械臂）系统，遵循 URDF 建模规范。

本 wiki 按「**模块 ↔ 算法**」双层组织：每个模块有一对页面——[code](identification/code.md) 讲实现接口与数据流，[algorithm](identification/algorithm.md) 讲数学推导与伪代码。跨模块共享的核心算法放在 [算法基础](algorithms/rnea-regressor.md)。

---

## 总览导航

### 算法基础（跨模块，被各模块引用）

| 页面 | 核心内容 |
|------|---------|
| [RNEA 与回归子](algorithms/rnea-regressor.md) | 逆动力学 τ=RNEA(q,q̇,q̈,π)、10 惯量参数、回归子 H=∂τ/∂π、扩展动力学模型 |
| [基参数约简（QR）](algorithms/base-parameter-reduction.md) | 列消除 + 列主元 QR + 双 QR、依赖系数 β、映射矩阵 M（φ_base = M·θ_r） |
| [D-最优实验设计](algorithms/d-optimal-design.md) | 信息矩阵、logdet 准则、Cholesky logdet、A/E-最优变体 |
| [物理一致性与重建](algorithms/physical-consistency.md) | pseudo-inertia P≽0、SDP/LMI 投影、base→全参数零空间/SDP 重建 |

### 模块（code / algorithm 配对）

| 模块 | code（实现） | algorithm（推导） | 算法主线 |
|------|-------------|------------------|---------|
| [identification](identification/code.md) | [code](identification/code.md) · [algorithm](identification/algorithm.md) | 扩展动力学建模 → 回归子 → 基参数 → LS/WLS → 质量指标 → 物理一致性 → 重建 |
| [optimal](optimal/code.md) | [code](optimal/code.md) · [algorithm](optimal/algorithm.md) | D-最优激励轨迹（Fourier/样条 NLP）+ D-最优标定位姿（SOCP/DetMax） |
| [calibration](calibration/code.md) | [code](calibration/code.md) · [algorithm](calibration/algorithm.md) | 几何标定：SE3 log-map 残差 + Levenberg–Marquardt + 离群点 + QR 基参数 |
| [tools](tools/code.md) | [code](tools/code.md) · [algorithm](tools/algorithm.md) | 回归子构建、QR 双分解、10 法线性求解器、IPOPT 抽象 |
| [utils](utils/code.md) | [code](utils/code.md) · [algorithm](utils/algorithm.md) | 样条/Fourier 参数化、统一配置继承、结果管理 |
| [backend](backend/code.md) | [code](backend/code.md) · [algorithm](backend/algorithm.md) | CasADi 符号计算图、SX/MX 自动微分、磁盘缓存 |
| [measurements](measurements/code.md) | [code](measurements/code.md) · [algorithm](measurements/algorithm.md) | 测量表示（SE3/wrench/current，遗留桩） |
| [visualisation](visualisation/code.md) | [code](visualisation/code.md) · [algorithm](visualisation/algorithm.md) | meshcat 3D 可视化 |

---

## 核心流水线一览

### 动力学辨识

```
配置 → load_robot → BaseIdentification 子类 → initialize() → solve()
  initialize: 滤波/微分 → 回归子 W=H(q,dq,ddq) → 标准参数 φ_std → 参考力矩
  solve: 去零列 → (decimate) → QR 基参数(W_b, φ_base, M) → 质量指标 → [物理一致性] → [重建] → 导出
  详见：algorithms/rnea-regressor · base-parameter-reduction · physical-consistency
```

### 最优激励轨迹

```
配置(trajectory.type) → BaseOptimalTrajectory(robot, config) → create_strategy
  fourier: CasadiBackend 符号图 → Z(MX) → Q/V/A → W_b.map(N,"openmp") → obj=-logdet(W_bᵀW_b+λI) → cs.nlpsol("ipopt")
  spline:  WaypointsGeneration → cyipopt → objective=cond(W_b)
  详见：algorithms/d-optimal-design · optimal/algorithm
```

### 几何标定

```
配置 + PEE测量 + q → BaseCalibration 子类 → initialize() → solve()
  initialize: load_data → QR 基运动学参数
  solve: LM(least_squares, cost_function=SE3 log-map 残差) → 离群点 → 评估 → URDF 回写
  详见：calibration/algorithm
```

---

## 关键约定

- **回归子行序**：`base_idx = j·N + sample_idx`（关节主序），与 `tau[j·N + i]` 对齐。
- **回归子列序**：每连杆 10 惯量参数（Pinocchio 顺序 `m,mx,my,mz,Ixx,Ixy,Iyy,Ixz,Iyz,Izz`）→ 可选 `fv/fs/Ia/off`（按类型分组）。
- **基参数关系**：`φ_base = M·θ_r`，`M` 由 QR 双分解构造（见 [base-parameter-reduction](algorithms/base-parameter-reduction.md)）。
- **后端**：`CasadiBackend` 是唯一后端，由 Fourier 策略在 `solve()` 内按需构造（无 `backend=` 参数）。

> 数学记号：本文档用 LaTeX 记号（`$...$` 行内、`$$...$$` 块），多数 wiki 渲染器（MathJax/KaTeX）支持；纯文本环境可读 `code` 块中的 ASCII 形式。
