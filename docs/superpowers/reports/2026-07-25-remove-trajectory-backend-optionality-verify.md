# Verification Report: remove-trajectory-backend-optionality

- **Date**: 2026-07-25
- **Change**: remove-trajectory-backend-optionality
- **Verify mode**: full
- **Base-ref**: 4c40746a97f8bbdea701cab611e1aa200eb13076
- **Branch**: worktree-remove-trajectory-backend-optionality
- **Workflow**: full

## Summary

| Dimension    | Status |
|--------------|--------|
| Completeness | 31/31 tasks complete; 2 capabilities, 4 ADDED + 3 MODIFIED scenarios 全部有实现证据 |
| Correctness  | 4 ADDED + 3 MODIFIED scenarios 验证通过；符号清理零残留 |
| Coherence    | D1-D6 决策全部落地，与 design.md / Design Doc 一致；无非目标范围改动 |

**Final Assessment**: 无 CRITICAL / IMPORTANT 问题。16 个失败测试经基线（pixi d7db3e0）复跑确认全部为预存环境问题（casadi 3.7.2 移除 `cholesky`、meshcat/numpy 转换器污染），非本次重构回归。Ready for archive。

## 1. 构建与测试证据（fresh run）

### 编译 / import 链
- `pixi run python -c "import figaroh; from figaroh.optimal.base_optimal_trajectory import BaseOptimalTrajectory; from figaroh.backend.casadi import CasadiBackend"` → **import ok**（去除 `backend.base` 后链路恢复）

### 直接相关测试（本次新增/修改的测试文件）
| 命令 | 结果 |
|------|------|
| `pixi run pytest tests/unit/test_backend.py tests/unit/test_config.py tests/unit/test_fourier_strategy.py` | **23 passed** |
| `pixi run -e casadi pytest tests/unit/test_backend.py tests/unit/test_config.py tests/unit/test_fourier_strategy.py` | **23 passed** |

### 全量单元测试对照基线
| 环境 | 本次 (HEAD) | 基线 (pixi d7db3e0) | 差异说明 |
|------|------------|---------------------|---------|
| 默认 | 16 failed, 258 passed, 2 skipped | 16 failed, 284 passed, 2 skipped | 失败数一致（16）；通过数差 -26 = 本次删除的 backend 死代码测试 |
| casadi | 16 failed, 258 passed, 2 skipped | 16 failed, 284 passed, 2 skipped | 同上 |

**关键结论**：失败数完全一致（基线 16 / 本次 16），失败集合相同（visualization/qr/check_env/fourier_e2e::test_solve_produces_results），**无新增回归**。本 change 还修复了基线的签名错配失败（`test_fourier_e2e` 之前因 `active_joints` 位置参数 TypeError 无法进入 solve，现已能进入 solve 阶段，唯一剩 `cs.cholesky` 预存问题）。

## 2. 预存失败归因（非本次回归）

16 个失败均与 backend 重构无关：

1. **`test_fourier_e2e::test_solve_produces_results`** — `AttributeError: module 'casadi' has no attribute 'cholesky'`（fourier_strategy.py:170）。casadi 3.7.2 移除了顶层 `cholesky`。base-ref `4c40746` 同样有 `cs.cholesky` 代码 → 预存。属 Non-Goal「不改 fourier NLP 构建逻辑」，应在独立 hotfix 修复（改用 `cs.qr` 等）。
2. **`test_robotvisualization.py`**（11 个）— meshcat/numpy 转换器重复注册污染，全量运行时触发，单跑通过。预存。
3. **`test_qr_decomposition.py::TestNumericalImprovements`**（2 个）— 数值算法细节，与 backend 无关。预存。
4. **`test_check_env.py::test_script_output_format`**（1 个）— 环境检查脚本输出格式。预存。

代码审查 subagent 已在 base commit 临时 worktree 复跑确认 `test_solve_produces_results` 的 `cs.cholesky` 失败为预存。

## 3. ADDED Spec Scenarios 验证（excitation-trajectory-optimization）

| Scenario | 验证方式 | 结果 |
|----------|---------|------|
| Fourier auto-selects CasADi backend | 构造 fourier 配置，断言 `isinstance(_backend, CasadiBackend)` + `strategy.name()=="fourier"` | **PASS** |
| Spline creates no backend | 构造 spline 配置，断言 `_backend is None` | **PASS** |
| Explicit backend argument rejected | `BaseOptimalTrajectory(..., backend="casadi")` 抛 `TypeError` 含 'backend' | **PASS** |
| YAML backend key ignored | `test_create_config_has_no_backend_key`：legacy `backend:` 键不进 `result` | **PASS** |

## 4. MODIFIED Spec Scenarios 验证（symbolic-casadi-pipeline）

| Scenario | 证据 | 结果 |
|----------|------|------|
| Fourier accesses dynamics via property | fourier_strategy.py:78,79,140,174 仅访问 `_ensure_symbolic_model`/`_cmodel`/`regressor_function`/`rnea_function`；无 `_W_fun`/`_rnea_fun`/`_cdata` 直接访问 | **PASS** |
| CasadiBackend instantiated directly (no factory) | base_optimal_trajectory.py:37 `from figaroh.backend.casadi import CasadiBackend`；:92 `CasadiBackend(robot=robot)`；`create_backend` 全工程零命中 | **PASS** |
| Removed ABC methods not present | casadi.py 无 `build_regressor`/`gradient`/`jacobian`/`create_solver`/`name`/`regressor_is_jacobian_of_rnea`/`ColumnEliminationCallback` | **PASS** |

## 5. Coherence — D1-D6 决策对照

| 决策 | 实现 | 一致 |
|------|------|------|
| D1 backend 由 trajectory_type 驱动 | base_optimal_trajectory.py:88-94 `if traj_type=="fourier": CasadiBackend(robot=robot) else: None` | ✓ |
| D2 CasadiBackend 去 ABC 化，保留 4 成员 + `_cdata`/`_W_fun`/`_rnea_fun` | casadi.py `class CasadiBackend:`（无基类）；保留 `_ensure_symbolic_model`/`_cmodel`/`_cdata`/`_W_fun`/`_rnea_fun`/`regressor_function`/`rnea_function` | ✓ |
| D3 删 `_solve_with_casadi_backend` + 触发分支 | 方法删除；`solve_with_waypoints`（BaseTrajectoryIPOPTProblem）无 `_backend` 读取，直接走 cyipopt | ✓ |
| D4 删 ColumnEliminationCallback | 已删，全工程零残留 | ✓ |
| D5 backend 子包瘦身 | `__init__.py` 仅 `__all__=["CasadiBackend"]` + try/except 懒加载 | ✓ |
| D6 YAML backend 键忽略 | config.py 不解析；`test_create_config_has_no_backend_key` 验证 | ✓ |

## 6. Non-Goal 遵守

- ✓ 未改 spline 的 cyipopt 求解路径（`solve_with_waypoints` 走 `RobotIPOPTSolver`）
- ✓ 未改 fourier strategy 的 NLP 构建逻辑（`fourier_strategy.py` 仅 docstring 措辞调整）
- ✓ 无向后兼容（`backend=` 直接 TypeError，无 deprecation）
- ✓ 未重构 `CasadiBackend` 的符号模型构建/缓存机制（`_ensure_symbolic_model`/`_cache_key` 保留）

## 7. 代码审查结论

review_mode=standard 审查（9 commits diff）结论：**可合并**。
- Critical: 0
- Important: 0
- Minor: 2（`mock_backend.name` 死属性、fourier docstring 措辞）—— 已在 commit `5d0cbae` 修复
- 外部 docs/README 同步：超出本 change 范围（计划聚焦 src/tests），记录为后续清理

## 8. Issues

### CRITICAL
无。

### IMPORTANT
无。

### SUGGESTION（非阻塞，后续处理）
1. 外部文档同步：`README.md:68,109`、`docs/developer-guide.md`、`docs/fourier-optimal-trajectory.md:193` 仍有 `backend="casadi"` / `active_joints` 旧调用样例，会误导新用户。建议单独 change 跟进。
2. 预存 `cs.cholesky` 失败：建议 hotfix 将 fourier_strategy.py:170 的 `cs.cholesky` 替换为 casadi 3.7.2 可用的分解。

## 9. 接受的偏差

| 偏差 | 原因 | 影响范围 |
|------|------|---------|
| 16 个预存测试失败未修复 | 全部为 casadi/meshcat 环境问题，基线即失败，属 Non-Goal 范围外 | 不影响 backend 重构正确性；四个 ADDED + 三个 MODIFIED 验收场景全通过 |
| 外部 docs/README 未同步 | 计划范围聚焦 src/tests，docs 同步未纳入 tasks | 用户照旧文档调用会遇 TypeError，需后续 change 修复 |

## 10. 安全检查

- 无硬编码密钥
- 无新增 unsafe 操作
- 无外部数据丢失风险（纯删除死代码 + 签名收窄）

**Verification passed. Ready for archive.**
