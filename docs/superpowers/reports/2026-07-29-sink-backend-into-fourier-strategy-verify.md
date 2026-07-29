# Verify Report (re-verify): sink-backend-into-fourier-strategy

- **Change**: sink-backend-into-fourier-strategy (tweak)
- **Date**: 2026-07-29（恢复 7-25 搁置的 verify）
- **Mode**: light（override；scale 误判 full，因 base_ref 755c5cc...HEAD 现含 sink-backend + check-env 修复 + fix-fourier-casadi-api-misuse hotfix + 全部 openspec 簿记共 27 文件。sink-backend 实际 scope 仅 5 文件：3 源 + 2 测试，见 commit e7f851a）
- **base_ref**: 755c5cc8
- **branch**: pixi（branch_status: handled，承自 7-25，本轮跳过分支处理）
- **Commit**: e7f851a (tweak: sink _backend into fourier strategy)

## 与 7-25 报告的关系

7-25 首轮 verify 实质完成 6 项检查并写报告，但流程在 guard 前搁置（`verify_result` 仍 pending）。首轮唯一 WARNING 是 e2e `test_solve_produces_results` 因 `cs.cholesky` 失败，当时正确判定为**基线遗留问题**（stash 验证 baseline 755c5cc 同样失败），接受偏差并建议"另起 change 修复"。

该建议已由 **fix-fourier-casadi-api-misuse**（hotfix，2026-07-29 归档）落实：`cs.cholesky` → chol-based SX Function、reshape、act_idxv 等已修复。**本轮 e2e 转绿，此前接受的偏差已不复存在。**

## 轻量验证 6 项（fresh, 2026-07-29）

| # | 检查 | 结果 | 证据 |
|---|------|------|------|
| 1 | tasks.md 全勾选 | PASS | 未勾选 `[ ]` = 0；已勾选 `[x]` = 4 |
| 2 | diff 与 tasks 一致 | PASS | `git show --stat e7f851a`：5 实现文件（casadi.py / base_optimal_trajectory.py / fourier_strategy.py + test_backend.py + test_fourier_strategy.py），与 tasks 4 项吻合 |
| 3 | 编译/导入 | PASS | `import figaroh.optimal.base_optimal_trajectory, .strategies.fourier_strategy, figaroh.backend.casadi` → `import OK` |
| 4 | 相关测试 (casadi env, 含 e2e) | PASS | `pytest test_backend test_fourier_strategy test_fourier_e2e test_config test_config_fourier` → **34 passed**。e2e `test_solve_produces_results` **绿**（cholesky 已修复，偏差消除）|
| 5 | 安全 | PASS | `git show e7f851a -- src/ tests/` 扫描无 eval/exec/os.system/subprocess/__import__/secret/password/api_key/pickle.load |
| 6 | code review | SKIP | `review_mode: off`（tweak 预设） |

## 已知失败项处理

**无。** 7-25 报告中接受的 cholesky WARNING 已由 fix-fourier-casadi-api-misuse hotfix 解决，本轮 e2e 干净通过，无 CRITICAL / IMPORTANT / WARNING 偏差。

## 结论

无 CRITICAL / IMPORTANT 问题，无未决偏差。本次 `_backend` 下沉行为等价、结构正确；fourier 路径经 hotfix 后功能完整。**验证通过。**
