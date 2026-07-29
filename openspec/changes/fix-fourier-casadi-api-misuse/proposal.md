## Why

`FourierOptimizationStrategy.solve()` 在 D-optimal 目标处崩溃：[fourier_strategy.py:175](src/figaroh/optimal/strategies/fourier_strategy.py#L175) 调用 `cs.cholesky(J_reg)`——CasADi 不存在该函数。导致 [test_fourier_e2e.py::test_solve_produces_results](tests/unit/test_fourier_e2e.py) 当前红（`module 'casadi' has no attribute 'cholesky'`）。这是遗留缺陷（源自 `bca665e`，当年 `symbolic-fourier-trajectory` change 的 verify 未拦住），Fourier 轨迹优化从未真正求解成功过。

## What Changes

- 修复 [fourier_strategy.py:175](src/figaroh/optimal/strategies/fourier_strategy.py#L175) 的 `cs.cholesky(J_reg)`：CasADi 无 `cholesky`（正确名 `chol`），且 `chol` 仅支持 DM/SX、不支持 MX，而该 NLP 全程用 MX。改为把 chol-based logdet 目标包进 SX Function（输入 `n_base×n_base` 的 `J_reg`），在 MX 图中调用，CasADi 自动 AD。
- 修复 [fourier_strategy.py:198](src/figaroh/optimal/strategies/fourier_strategy.py#L198) 的 `tau_raw.T.reshape(-1)`：numpy 单参数写法，CasADi `reshape` 需显式 `(rows, cols)`。改为 `cs.reshape(tau_raw, nv * n_samples, 1)`，列主序展平 `(nv, Ns)` 得 sample-major 序，匹配 [line 216](src/figaroh/optimal/strategies/fourier_strategy.py#L216) 的 `tau_flat[i*nv + j]` 索引。
- 收紧 [test_fourier_strategy.py](tests/unit/test_fourier_strategy.py) 的 `test_solve_populates_results`：去掉 `try/except Exception: pass` 吞异常，改为断言 solve 成功，消除假阳性。

## Capabilities

### New Capabilities
<!-- 无新增 capability -->

### Modified Capabilities
<!-- 无 spec 级别需求变更；仅修复实现使其符合既有 symbolic-casadi-pipeline spec 的 D-optimal 验收意图，不改任何验收场景。 -->

## Impact

- 受影响文件：`src/figaroh/optimal/strategies/fourier_strategy.py`（line 175-176 目标、line 198 reshape）、`tests/unit/test_fourier_strategy.py`（`test_solve_populates_results` 断言）。
- 行为变化：Fourier 策略从"崩溃、无结果"变为"能正常求解 D-optimal 轨迹"；目标函数数学语义不变（`-log(det(J_reg))`）。
- 无接口变化、无依赖变化、无架构变化。
