## Context

`FourierOptimizationStrategy` 用 MX 构建优化变量与轨迹，用 SX 构建回归子/RNEA 函数并通过 `.map("openmp")` 嵌入 MX 图（MX/SX 混合，见 [fourier_strategy.py:39-46](src/figaroh/optimal/strategies/fourier_strategy.py#L39-L46)）。D-optimal 目标 `-log(det(J_reg))` 原本直接在 MX 上写 `cs.cholesky(J_reg)`，但 CasADi 的 `chol`（无 `cholesky`）只支持 DM/SX 不支持 MX；`det` 在 MX 上又无梯度（`Determinant` 节点无 `eval`）。实测对比：

| 路径 | 结果 |
|------|------|
| `cs.cholesky`（现状） | `AttributeError: module 'casadi' has no attribute 'cholesky'` |
| `cs.chol` 改名（MX） | `NotImplementedError: chol(DM), chol(SX). You have: (MX)` |
| `cs.det`（MX） | NLP 能构建但 IPOPT 无梯度 → `Invalid_Number_Detected`, `f=0` |
| `cs.qr`（MX） | 同样 NotImplementedError，仅 DM/SX |

## Goals / Non-Goals

**Goals:**
- 让 Fourier 策略能真正求解，e2e 从红转绿。
- 保持目标函数数学语义不变（`-log(det(J_reg))`，chol-based logdet）。
- 消除假阳性测试，防回归。

**Non-Goals:**
- 不改 D-optimal 准则本身（不切 A/E-optimal）。
- 不重构 MX/SX 混合架构。
- 不处理卡住的 `sink-backend-into-fourier-strategy` change（另行归档）。

## Decisions

1. **chol 目标包进 SX Function（最小搬迁）**：把 `chol(J)` + `2·Σlog(diag)` 封装为 `cs.Function("logdet", [J_sx], [...])`，输入 `n_base×n_base` 的 `J_reg`，在 MX 图中 `obj = -logdet_fn(J_reg)` 调用。CasADi 自动对该 SX Function 做 reverse-mode AD，梯度精确（实测 `max|grad_AD − grad_FD| = 4.1e-9`，NLP `Solve_Succeeded`/13 iter）。`mtimes`/`MX.eye` 留在 MX 原地不动（本就支持 MX），只搬崩坏的 chol 段。与回归子 `W_fun`/`rnea_fun` 的"SX Function 经由 MX 调用"组织方式一致——文件 docstring 本就声明是 MX/SX hybrid。

2. **reshape 显式维度**：`tau_raw.T.reshape(-1)` → `cs.reshape(tau_raw, nv * n_samples, 1)`。列主序展平 `(nv, Ns)` 正好得到 sample-major 序（实测 `[0,10,1,11,2,12] = i*nv+j`），匹配既有约束索引 `tau_flat[i*nv + act_idxv[j]]` 与 line 197 注释"per-sample, all joints"。

3. **收紧测试**：`test_solve_populates_results` 去掉 `try/except Exception: pass`，断言 solve 不抛异常且 `T_F/P_F/V_F/A_F` 被填充。e2e `test_solve_produces_results` 作为验收门。

## Risks / Trade-offs

- **chol 需 SPD**：`reg_lambda`（默认 1e-6）保证 `J_reg` 正定；若 `reg_lambda=0` 且回归子秩亏，LDL 出现非正枢轴→`sqrt`→NaN→IPOPT 报错。这是正确行为（目标在该点本就未定义），与原设计意图一致，非回归。已有 `test_regularization_ensures_spd` 覆盖。
- **性能**：SX Function 是 `n_base×n_base`（基参数数，通常 10–80）小矩阵 chol，构建~免费、每迭代求值+Jacobian 成本可忽略；大头 `W_fun.map(200,"openmp")` 原封未动。
- **可移植性**：不依赖任何 MX-only 线代节点；纯 SX chol 在 CasADi 3.7.2 已验证。
