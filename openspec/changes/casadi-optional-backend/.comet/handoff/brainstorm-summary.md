# Brainstorm Summary

- Change: casadi-optional-backend
- Date: 2026-07-02

## 确认的技术方案

**Backend 抽象层 — Solver 级策略模式**：
- `Backend` ABC 定义 `build_regressor()` 和 `create_solver(nlp_def, opts)` 两个接口
- `NumericalBackend`：包装现有 for-loop regressor + cyipopt.Problem + nd.Gradient/nd.Jacob
- `CasadiBackend`：pinocchio.casadi 符号 regressor + cs.nlpsol('ipopt', nlp) 内置精确 Hessian
- 新增 `src/figaroh/backend/` package

**不可符号化操作处理 — CasADi Callback 包装**：
- QR 分解（列 pivoting）和列消去（动态阈值判定）无法直接用 CasADi SX 表达
- 封装为 `casadi.Callback`，CasADi 对 Callback 部分用有限差分，其余链路解析 AD
- 这两处计算量占比 <10%，总体仍大幅优于全数值路径

**后端选择 — 配置 + 程序化双通道**：
- `backend: numerical|casadi` 在 YAML config 中设置默认值
- `BaseOptimalTrajectory(backend=...)` 参数可覆盖
- 优先级：程序化参数 > YAML config > 默认值 `"numerical"`
- 未安装 CasADi 时选择 `casadi` 后端 → 明确的 ImportError + 安装指引

**核心数据流（CasADi 路径）**：
```
X_sym → Callback(spline) → Q,V,A → CasADi map(regressor) → W
      → Callback(列消去+QR) → W_b → CasADi norm_fro*pinv → cond(W_b)
      → nlp = {'x': X_sym, 'f': obj, 'g': cons}
      → solver = nlpsol('ipopt', nlp)
      → solver(x0, lbg, ubg)  ← 自动 gradient/Jacobian/Hessian
```

## 关键取舍与风险

- **取舍**：Solver 级抽象意味着 `BaseTrajectoryIPOPTProblem` 需适度重构（从"提供回调"变为"定义 NLP + 创建 solver"）
- **取舍**：conda-forge pinocchio 替换 PyPI pin（需 CasADi 绑定的代价）
- **风险**：`cs.nlpsol` 选项名与 `IPOPTConfig` 不完全对应 → 需写映射表
- **风险**：conda-forge 版本滞后 → numerical 后端始终可用作 fallback
- **已消除风险**：CasADi v3.7.2 + conda-forge pinocchio 4.0.0 在 linux-aarch64 上均可用，所有 212 个现有测试兼容

## 测试策略

- **单元测试**：Backend ABC、NumericalBackend 一致性、CasadiBackend 数值精度 (1e-8)、错误提示
- **集成测试**：两种后端 solve() 结果一致、backend 参数优先级、未安装 CasADi 时的降级行为
- **回归测试**：全部 212 个现有测试通过
- **基准测试**：regressor 构建、梯度计算、端到端 solve() 的 numerical vs casadi 时间对比

## Spec Patch

无（现有 delta spec 的验收场景已完整，无需回写）
