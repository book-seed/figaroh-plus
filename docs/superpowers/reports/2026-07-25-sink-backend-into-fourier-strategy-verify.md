# Verify Report: sink-backend-into-fourier-strategy

- **Date**: 2026-07-25
- **Mode**: light (override; scale 误判为 full，因 openspec/comet 产物计入文件数。提交区间复核实际源码改动仅 5 文件：3 源 + 2 测试)
- **Branch**: pixi
- **Commit**: e7f851a (tweak: sink _backend into fourier strategy)

## 变更摘要

将 `self._backend = CasadiBackend(robot=...)` 从 `BaseOptimalTrajectory.__init__` 下沉到
`FourierOptimizationStrategy.solve` 内部按需构造；base 不再 import casadi、不再持有
`_backend`。`CasadiBackend` 补 TODO 标注未来 identification 接入时改模块级共享单例。
行为不变；同步更新 `test_backend.py` / `test_fourier_strategy.py` 以反映新所有权。

## 6 项检查

| # | 检查项 | 结果 | 证据 |
|---|--------|------|------|
| 1 | tasks.md 全部 `[x]` | PASS | `grep -c '^\- \[ \]'` = 0 |
| 2 | diff 与 tasks 一致 | PASS | 5 文件（casadi.py / base_optimal_trajectory.py / fourier_strategy.py + 2 测试），与 tasks 描述吻合 |
| 3 | 编译/导入 | PASS | `import figaroh.optimal.base_optimal_trajectory, .strategies.fourier_strategy, figaroh.backend.casadi` OK |
| 4 | 相关测试 (default env) | PASS | `pytest test_backend test_fourier_strategy test_config test_config_fourier` → 32 passed |
| 4b | fourier e2e (casadi env) | WARNING | `test_solve_produces_results` 失败，根因 `cs.cholesky` API 不兼容 |
| 5 | 安全 | PASS | 无硬编码密钥、无 unsafe、无外部 I/O 变更 |
| 6 | code review | SKIP | review_mode: off |

## 已知失败项处理

### test_fourier_e2e::test_solve_produces_results — WARNING（接受偏差）

- **现象**：`AttributeError: module 'casadi' has no attribute 'cholesky'`，位于
  [fourier_strategy.py:175](src/figaroh/optimal/strategies/fourier_strategy.py#L175)。
- **根因**：CasADi 版本 API 差异（`cs.cholesky` 不存在），与本次 `_backend` 下沉无关。
- **证据**：stash 本次改动后在 baseline (HEAD=755c5cc) 跑同一测试**同样失败**，
  失败点不在本次改动的三处代码内（casadi TODO 注释 / base 删 import / fourier 改
  `context._backend` 读取为局部构造）。
- **判定**：WARNING（非 CRITICAL/IMPORTANT），属预先存在的环境兼容问题。
- **影响范围**：仅 fourier 路径的完整 e2e；不阻断本次纯结构下沉的验证。
- **建议**：另起 change 修复 CasADi cholesky API 兼容（改用 `cs.qr` 或 numpy 后处理）。

## 结论

无 CRITICAL / IMPORTANT 问题；唯一失败项为预先存在的基线兼容问题，接受偏差。
本次下沉行为等价，验证通过。
