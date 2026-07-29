# Verify Report — fix-fourier-casadi-api-misuse

- **Change**: fix-fourier-casadi-api-misuse (hotfix)
- **Date**: 2026-07-29
- **verify_mode**: light（手动覆盖；见下"规模评估"）
- **base_ref**: dc5600e (pixi HEAD)
- **branch handling**: 合并回 pixi（fast-forward，无冲突），特性分支已删除

## 规模评估与 verify_mode 覆盖

`comet-state scale` 自动判定为 `full`，触发指标：
- Tasks: 4（阈值 3）—— act_idxv 多出 1 个 task
- Changed files: 7（阈值 4）

复核提交区间 `dc5600e...HEAD` 后手动覆盖为 `light`，理由：
- 7 个"变更文件"中 5 个为 openspec/comet 簿记（proposal/design/tasks/.openspec.yaml/.comet.yaml），**真正实现足迹仅 2 文件**：`src/figaroh/optimal/strategies/fourier_strategy.py`（+27/-16）、`tests/unit/test_fourier_strategy.py`（+16/-16）。
- hotfix 无 design doc（`design_doc: null`）、无 delta spec（0 capabilities），full 验证的 design doc 一致性 / delta spec 漂移 / spec 场景检查在此**不适用**。
- 实质差异仅"实现是否符合 design.md 决策"与"proposal 目标是否满足"，已纳入轻量检查 2/4。

## 轻量验证 6 项（fresh evidence）

| # | 检查 | 结果 | 证据 |
|---|------|------|------|
| 1 | tasks.md 全勾选 | PASS | 未勾选 `[ ]` = 0；已勾选 `[x]` = 4 |
| 2 | 改动文件与 tasks 一致 | PASS | `git diff --stat dc5600e...HEAD -- src/ tests/`：2 文件，匹配 tasks 1.1/1.2（fourier_strategy.py）/2.1（test）；3.1 为验收门 |
| 3 | 构建通过 | PASS | `python -c "import figaroh; import figaroh.optimal.strategies.fourier_strategy"` → `import OK` |
| 4 | 相关测试通过 | PASS | `pytest test_fourier_strategy.py test_fourier_e2e.py test_backend.py` → **22 passed**（fresh）。e2e `test_solve_produces_results` 红→绿 |
| 5 | 无安全问题 | PASS | diff 扫描无 `eval/exec/os.system/subprocess/__import__/secret/password/api_key/token=/pickle.load`；新增均为纯符号计算（SX Function/chol/log/reshape）与断言 |
| 6 | code review | SKIP | `review_mode: off`，跳过自动 code review（按 hotfix 预设） |

**结论**：6 项全部通过，无 CRITICAL / IMPORTANT 问题。验证通过。

## 修复摘要（对照 proposal/design）

1. `cs.cholesky(J_reg)` → chol-based logdet 封装为 SX Function、MX 图调用（chol 仅 DM/SX；实测 `Solve_Succeeded`，梯度 `max|AD-FD|=4e-9`）。
2. `tau_raw.T.reshape(-1)` → `cs.reshape(tau_raw, nv*n_samples, 1)`（列主序得 sample-major，匹配 `tau_flat[i*nv+j]`）。
3. `_initialize_coefficients` 补 `act_idxv` 定义（修复 1-2 后才暴露的 `NameError`），对齐父 `solve()`。
4. `test_solve_populates_results` 去掉 `try/except Exception: pass` 吞异常，断言 solve 成功且结果填充。

## branch handling

- 选项 1（合并回 pixi）执行：`git checkout pixi` → `git merge --ff-only fix-fourier-casadi-api-misuse`（fast-forward，pixi 现 = f4f47d0）→ 合并结果重跑测试 **22 passed** → `git branch -d fix-fourier-casadi-api-misuse` 已删。
- pixi 现 `ahead 4` of `origin/pixi`（本地合并，未 push）。
- 注：archive 阶段尚未跑，将在 pixi 上产生归档提交。
