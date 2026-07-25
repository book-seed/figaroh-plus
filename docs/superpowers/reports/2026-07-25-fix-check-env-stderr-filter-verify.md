# Verify Report: fix-check-env-stderr-filter

- **Date**: 2026-07-25
- **Mode**: light (override; scale 误判为 full，因 openspec/comet 产物计入文件数。提交区间复核实际源码改动仅 2 文件：scripts + tests)
- **Branch**: pixi
- **Commit**: (tweak: fix check_env test stderr filter and script output format)

## 变更摘要

修复 `tests/unit/test_check_env.py::test_script_output_format` 失败。根因经
systematic-debugging 定位为三类问题：

1. CasADi C++ 运行时在非 OpenMP 编译时向 stderr 打 `WARNING(...)`，测试过滤
   漏掉该格式（仅排除 `RuntimeWarning`/`DeprecationWarning`）。
2. 脚本标题 `print("FIGAROH 环境验证")` 与其 docstring `"""FIGAROH 环境验证脚本。"""`
   及测试期望不一致（缺「脚本」二字）。
3. 脚本输出扁平单节 `[依赖]`，测试期望分节编号 `[1] 基础依赖` / `[2] IPOPT 线性求解器`。

修复：测试 stderr 过滤追加排除 `CasADi` 行；脚本标题对齐 docstring；输出重构为
分节编号（`[1] 基础依赖` / `[2] IPOPT 线性求解器` / `[3] CasADi OpenMP`）。
脚本逻辑不变，仍正确识别 OpenMP 状态并输出 PASS/FAIL。

## 6 项检查

| # | 检查项 | 结果 | 证据 |
|---|--------|------|------|
| 1 | tasks.md 全部 `[x]` | PASS | `grep -c '^\- \[ \]'` = 0 |
| 2 | diff 与 tasks 一致 | PASS | 2 文件（check_casadi_env.py / test_check_env.py），与 tasks 吻合 |
| 3 | 语法/可运行 | PASS | `ast.parse` OK |
| 4 | test_check_env (default env) | PASS | 6 passed |
| 4b | test_check_env (casadi env) | PASS | 6 passed（无 OpenMP warning 场景亦通过） |
| 5 | 安全 | PASS | 无硬编码密钥、无 unsafe、无外部 I/O 变更 |
| 6 | code review | SKIP | review_mode: off |

## 结论

无 CRITICAL / IMPORTANT 问题。test_check_env 6 项在 default 与 casadi 两个 env
均通过。验证通过。
