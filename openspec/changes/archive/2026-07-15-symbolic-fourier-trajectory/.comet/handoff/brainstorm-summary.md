# Brainstorm Summary

- Change: symbolic-fourier-trajectory
- Date: 2026-07-15

## 确认的技术方案

### 架构：策略模式 + 配置隔离

- `BaseOptimalTrajectory` 重构为 Context 容器（持有 robot、config、results、基参数）
- `TrajectoryOptimizationStrategy` 抽象基类定义 `solve(context) → List[TrajectorySegment]`
- `SplineOptimizationStrategy`：现有三次样条多段堆叠逻辑完整迁移
- `FourierOptimizationStrategy`：全符号 CasADi MX NLP + `cs.nlpsol("ipopt")`
- 工厂方法 lazy import，配置字典隔离（`spline_config` / `fourier_config`）

### SX/MX 混合管线

- **内层 SX**：`CasadiBackend` 提供 `regressor_function` 和 `rnea_function` property（SX Function，缓存到 `~/.figaroh/casadi_cache/`）
- **外层 MX**：傅里叶系数 `MX.sym("coeffs")` → q/v/a MX 表达式（列主序 `(n_act, N_s)`）→ `f_single.map(N_s, "openmp")` 调用内层 SX Function → MX reverse-mode AD
- 缓存指纹覆盖全惯性参数 + 关节结构 → SHA256，URDF 任何修改自动失效

### D-最优目标函数

- `obj = -2 * Σ log(L_ii)`，`L = cholesky(W_bᵀW_b/N_s + λI)`，`λ = 1e-6` 默认可配置
- 正则化保证 SPD → Cholesky 必定成功 → 无 Trial Step 报错风险

### 力矩约束

- `τ_total = cpín.rnea(q,v,a) + fv·v + fs·tanh(α_opt·v)`，摩擦力名义值来自 `params_std`
- `tanh(α·v)` 优化阶段 α 默认 10（梯度友好），辨识阶段 α 默认 100（逼近精度）

### 基参数

- 沿用 `BaseParameterComputer`（随机三次样条 + QR → `idx_b`），轨迹类型无关
- `idx_b` 数值 numpy 索引 → CasADi MX 图中 `W_b = W_full[:, idx_b]`

### IPOPT 求解器

- CasADi 内置 `cs.nlpsol("ipopt", nlp)`，HSL `ma57` 优先，回退 `mumps`
- `map("openmp")` 硬性要求，环境不支持时阻断（不静默降级）

### 结果标准化与兼容

- `TrajectorySegment` dataclass 携带 numpy 数值数组（策略边界隔离）
- 策略内部负责最优系数 → q/v/a numpy 重构
- 条件数在优化循环外用 `numpy.linalg.cond` 事后计算
- Context 统一填充 `self.results`（`List[np.ndarray]` 格式不变）
- 样条专有类（`BaseTrajectoryIPOPTProblem`、`RobotIPOPTSolver`、`cyipopt`）归 `SplineOptimizationStrategy` 独占

## 关键取舍与风险

| Risk | Mitigation |
|------|-----------|
| CasADi MX 图过大（10-20 万节点）| `map("openmp")` 并行 + HSL 稀疏求解 |
| 初始系数振幅过小导致信息矩阵退化 | λ 正则化保证 SPD |
| HSL 环境缺失 | 环境验证脚本阻断，安装文档明确步骤 |
| URDF 缓存过期 | 全惯性参数 SHA256 指纹自动失效 |
| 摩擦力名义值不准确 | 优化阶段用名义值即可（20-30% 误差在安全裕度内被吸收） |

## 测试策略

- 傅里叶 v/a 与解析导数对照
- Cholesky logdet 与 `cs.det` + numpy 数值对照
- 符号雅可比与有限差分对照（容差 1e-5）
- UR10 端到端：`trajectory_type: "fourier"` 成功生成轨迹 + 条件数 < 随机轨迹
- `tanh(α·v)` 可微性验证
- 配置解析回归测试（现有配置不受影响）

## Spec Patch

回写 `symbolic-casadi-pipeline/spec.md`：
1. 补充 Requirement: "力矩约束 = RNEA + 摩擦力"
2. 补充 Requirement: "缓存指纹包含完整惯性参数"
3. 补充 Requirement: "SX/MX 混合架构"（内层 SX Function 缓存 → 外层 MX map + reverse-mode AD）
4. 补充 Scenario: "Cholesky 因式分解计算 logdet，正则化保证 SPD"
