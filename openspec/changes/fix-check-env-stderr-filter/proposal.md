## Why

`tests/unit/test_check_env.py::test_script_output_format` 失败。运行 `scripts/check_casadi_env.py` 时，default env 使用的 CasADi（非自编译 OpenMP 版本）在 C++ 运行时向 stderr 输出 `CasADi - ... WARNING("CasADi was not compiled with WITH_OPENMP=ON. Falling back to serial evaluation.")`。测试的 stderr 过滤（L46-50）只排除含 `RuntimeWarning`/`DeprecationWarning` 关键字的行，漏掉了 CasADi 这条 C++ 层 `WARNING(...)` 格式，导致 `assert not stderr_filtered.strip()` 失败。

根因：CasADi C++ 运行时的 stderr warning 无法被脚本内的 Python `warnings.filterwarnings` 拦截，属上游已知行为。脚本逻辑本身正确识别 OpenMP 状态并输出 `[PASS]`，测试断言过严而非脚本有 bug。

## What Changes

- 在 `tests/unit/test_check_env.py` 的 stderr 过滤逻辑中追加排除 CasADi C++ warning 行（匹配含 `CasADi` 的行）。
- 修复 `scripts/check_casadi_env.py` 标题输出漂移：脚本 docstring 与测试均期望 `FIGAROH 环境验证脚本`，但 L93 `print` 实际输出 `FIGAROH 环境验证`（缺「脚本」）。让 print 输出对齐 docstring 意图。

## Capabilities

### New Capabilities
<!-- 无新增 capability -->

### Modified Capabilities
<!-- 无 spec 级别需求变更；仅测试过滤逻辑调整，不改任何验收场景。 -->

## Impact

- 受影响文件：`tests/unit/test_check_env.py`（stderr 过滤条件）、`scripts/check_casadi_env.py`（标题字符串，1 行）。
- 行为不变：脚本仍正确识别 OpenMP 状态；标题对齐其 docstring 意图；仅放宽测试对上游 C++ warning 的容差。
- 无接口变化、无依赖变化。
