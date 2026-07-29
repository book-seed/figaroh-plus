## 1. 修复 solve() 崩溃（不存在的 CasADi API + 未定义符号）

- [x] 1.1 fourier_strategy.py — chol-based logdet 目标封装为 SX Function（替代 line 175 `cs.cholesky`，留 `mtimes`/`MX.eye` 在 MX 不动）；line 198 `tau_raw.T.reshape(-1)` 改为 `cs.reshape(tau_raw, nv * n_samples, 1)`
- [x] 1.2 fourier_strategy.py `_initialize_coefficients` — 补 `act_idxv` 定义（修复 1.1 后才暴露的 `NameError: act_idxv`），对齐父 `solve()` 方法

## 2. 收紧测试防回归

- [x] 2.1 test_fourier_strategy.py `test_solve_populates_results` — 去掉 `try/except Exception: pass` 吞异常，改为断言 solve 成功且结果被填充

## 3. 验收

- [x] 3.1 tests/unit/test_fourier_e2e.py::test_solve_produces_results 从红转绿（+ 完整 fourier/backend 套件 22 passed 无回归）
